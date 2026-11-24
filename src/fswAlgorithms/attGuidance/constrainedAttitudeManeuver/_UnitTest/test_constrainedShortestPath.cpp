/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "fswAlgorithms/attGuidance/constrainedAttitudeManeuver/constrainedAttitudeManeuver.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {
constexpr double distanceTolerance = 2e-12; // [-] Accumulated MRP edge distance

/*! @brief Evaluate the edge metric independently of the production distance function.
 *  @param from First attitude node.
 *  @param to Second attitude node.
 *  @return Smallest Euclidean distance between the eligible MRP representations.
 */
double edgeDistance(const Node& from, const Node& to)
{
    const Eigen::MRPd first = cArray2EigenMRPd(from.sigma_BN);
    const Eigen::MRPd second = cArray2EigenMRPd(to.sigma_BN);
    std::vector<Eigen::MRPd> firstSets{ first };
    std::vector<Eigen::MRPd> secondSets{ second };
    const double shadowThreshold = 1e-8; // [-] Production edge-metric domain
    if (first.norm() > shadowThreshold) {
        firstSets.push_back(first.shadow());
    }
    if (second.norm() > shadowThreshold) {
        secondSets.push_back(second.shadow());
    }
    double result = std::numeric_limits<double>::infinity();
    for (const auto& firstSet : firstSets) {
        for (const auto& secondSet : secondSets) {
            result = std::min(result, (firstSet.coeffs() - secondSet.coeffs()).norm());
        }
    }
    return result;
}

/*! @brief Find the shortest distance on the actual generated directed adjacency graph using Dijkstra.
 *  @param start Search origin.
 *  @param goal Search destination.
 *  @return Minimum cost, or infinity if the goal is unreachable.
 *  @note Uses independent costs, no heuristic or production search state, and a set with explicit decrease-key.
 */
double dijkstra(const Node* start, const Node* goal)
{
    std::map<const Node*, double> costs{ { start, 0.0 } };
    std::set<std::pair<double, const Node*>> frontier{ { 0.0, start } };
    while (!frontier.empty()) {
        const auto [cost, current] = *frontier.begin();
        frontier.erase(frontier.begin());
        if (current == goal) {
            return cost;
        }
        for (int i = 0; i < current->neighborCount; ++i) {
            const Node* next = current->neighbors[i];
            const double candidate = cost + edgeDistance(*current, *next);
            const auto previous = costs.find(next);
            if (previous == costs.end() || candidate < previous->second) {
                if (previous != costs.end()) {
                    frontier.erase({ previous->second, next });
                }
                costs[next] = candidate;
                frontier.emplace(candidate, next);
            }
        }
    }
    return std::numeric_limits<double>::infinity();
}

/*! @brief Read the endpoint selected by grid generation.
 *  @param model Planner containing the graph.
 *  @param key Endpoint grid indices.
 *  @return The generated endpoint node.
 */
Node* endpoint(ConstrainedAttitudeManeuver& model, const int key[3])
{
    return &model.NodesMap.at(key[0]).at(key[1]).at(key[2]);
}

/*! @brief Sum independently evaluated edge costs along a returned path.
 *  @param model Planner containing the path.
 *  @return Total dimensionless MRP distance.
 */
double pathDistance(const ConstrainedAttitudeManeuver& model)
{
    double result = 0.0; // [-]
    for (int i = 1; i < model.path.N; ++i) {
        const Node* from = model.path.list[static_cast<std::size_t>(i - 1)];
        const Node* to = model.path.list[static_cast<std::size_t>(i)];
        EXPECT_NE(std::find(from->neighbors, from->neighbors + from->neighborCount, to),
                  from->neighbors + from->neighborCount);
        result += edgeDistance(*from, *to);
    }
    return result;
}

//! @brief Own the planner and the messages used to reproduce the example grids without SPICE.
struct ScenarioGrid
{
    ConstrainedAttitudeManeuver model{ 8 };
    Message<SCStatesMsgPayload> stateMessage;
    Message<VehicleConfigMsgPayload> vehicleMessage;
    Message<SpicePlanetStateMsgPayload> bodyMessage;

    /*! @brief Configure one of the three example grids or the alternate-start regression.
     *  @param scenario Index from zero through three; three uses the alternate start in case zero.
     */
    explicit ScenarioGrid(std::size_t scenario)
    {
        const std::array<Eigen::MRPd, 4> starts = { // [-] Example attitudes and issue #1726 alternate start
                                                    Eigen::MRPd(0.522, -0.065, 0.539),
                                                    Eigen::MRPd(0.314, -0.251, 0.228),
                                                    Eigen::MRPd(-0.378, 0.119, -0.176),
                                                    Eigen::MRPd(-3.0 / 7.0, -2.0 / 7.0, 0.0)
        };
        const std::array<Eigen::MRPd, 3> goals = { // [-]
                                                   Eigen::MRPd(0.342, 0.223, -0.432),
                                                   Eigen::MRPd(0.326, -0.206, -0.823),
                                                   Eigen::MRPd(0.350, 0.220, -0.440)
        };
        SCStatesMsgPayload state{};
        eigenMRPd2CArray(starts.at(scenario), state.sigma_BN);
        stateMessage.write(&state, 0, 0);
        VehicleConfigMsgPayload vehicle{};
        vehicle.ISCPntB_B[0] = vehicle.ISCPntB_B[4] = vehicle.ISCPntB_B[8] = 1.0; // [kg m^2]
        vehicleMessage.write(&vehicle, 0, 0);
        SpicePlanetStateMsgPayload body{};
        // Snapshot of the Sun direction relative to the spacecraft at the example's reset epoch.
        body.PositionVector[0] = 0.4192278421178079;  // [m] Only the direction affects this grid.
        body.PositionVector[1] = -0.8329654563927242; // [m]
        body.PositionVector[2] = -0.3611323370313256; // [m]
        bodyMessage.write(&body, 0, 0);
        model.scStateInMsg.subscribeTo(&stateMessage);
        model.vehicleConfigInMsg.subscribeTo(&vehicleMessage);
        model.keepInCelBodyInMsg.subscribeTo(&bodyMessage);
        model.keepOutCelBodyInMsg.subscribeTo(&bodyMessage);
        double tracker[3] = { 1.0, 0.0, 0.0 };             // [-]
        double sensorY[3] = { 0.0, 1.0, 0.0 };             // [-]
        double sensorZ[3] = { 0.0, 0.0, 1.0 };             // [-]
        model.appendKeepOutDirection(tracker, 20.0 * D2R); // [rad]
        model.appendKeepInDirection(sensorY, 70.0 * D2R);  // [rad]
        if (scenario == 1 || scenario == 2) {
            model.appendKeepInDirection(sensorZ, 70.0 * D2R); // [rad]
        }
        eigenMRPd2CArray(goals.at(scenario == 3 ? 0 : scenario), model.sigma_BN_goal);
        model.costFcnType = 0;
        model.BSplineType = 0;
        model.avgOmega = 0.04; // [rad/s]
        model.bskLogger.setLogLevel(BSK_SILENT);
        model.ReadInputs();
        Node start, goal;
        eigenMRPd2CArray(starts.at(scenario), start.sigma_BN);
        eigenMRPd2CArray(goals.at(scenario == 3 ? 0 : scenario), goal.sigma_BN);
        model.GenerateGrid(start, goal);
    }
};

class ConstrainedShortestPathExamples : public ::testing::TestWithParam<int>
{};

/*! @brief Match Dijkstra costs, preserve legacy results, and isolate repeated searches on the same graph. */
TEST_P(ConstrainedShortestPathExamples, MatchesDijkstra)
{
    const auto scenario = static_cast<std::size_t>(GetParam());
    ScenarioGrid fixture(scenario);
    auto& model = fixture.model;
    const double expected = dijkstra(endpoint(model, model.keyS), endpoint(model, model.keyG));
    const std::array<double, 4> legacyCosts = { 1.29704, 1.26916, 0.85442, 1.19731 }; // [-] Issue #1726
    const std::array<std::size_t, 4> legacyExpansions = { 17, 37, 5, 12 };
    ASSERT_FALSE(model.useShortestPath);
    model.AStar();
    const auto legacyPath = model.path.list;
    const double legacyCost = pathDistance(model);
    EXPECT_NEAR(legacyCost, legacyCosts.at(scenario), 5e-6); // [-] Published costs are rounded.
    EXPECT_EQ(model.expandedNodeCount, legacyExpansions.at(scenario));
    // Recorded from the legacy implementation before introducing the optional heuristic.
    const std::array<std::vector<std::array<int, 3>>, 3> legacyKeys = {
        { { { 4, 0, 4 }, { 5, 0, 3 }, { 5, 0, 2 }, { 5, 0, 1 }, { 5, 0, 0 }, { 4, 1, -1 }, { 3, 1, -2 }, { 2, 2, -3 } },
          { { 2, -2, 2 }, { 2, -2, 3 }, { 1, -2, 4 }, { 0, -1, 5 }, { -1, 0, 6 }, { -2, 1, 7 }, { 2, -1, -6 } },
          { { -3, 1, -1 }, { -2, 1, -2 }, { -1, 1, -3 }, { 0, 1, -3 }, { 1, 1, -3 }, { 2, 2, -3 } } }
    };
    if (scenario < legacyKeys.size()) {
        const auto& keys = legacyKeys.at(scenario);
        ASSERT_EQ(legacyPath.size(), keys.size());
        for (std::size_t i = 0; i < keys.size(); ++i) {
            EXPECT_EQ(legacyPath[i], endpoint(model, keys[i].data()));
        }
    }

    model.useShortestPath = true;
    for (int repeat = 0; repeat < 2; ++repeat) {
        model.AStar();
        ASSERT_GT(model.path.N, 1);
        EXPECT_NEAR(pathDistance(model), expected, distanceTolerance);
        if (GetParam() < 2) {
            EXPECT_GT(model.expandedNodeCount, static_cast<std::size_t>(4 * model.N * model.N));
        }
    }
    EXPECT_DOUBLE_EQ(endpoint(model, model.keyG)->heuristic, 0.0);
    for (const auto& plane : model.NodesMap) {
        for (const auto& row : plane.second) {
            for (const auto& entry : row.second) {
                const Node& from = entry.second;
                for (int i = 0; i < from.neighborCount; ++i) {
                    const Node& to = *from.neighbors[i];
                    EXPECT_LE(from.heuristic, edgeDistance(from, to) + to.heuristic + distanceTolerance);
                }
            }
        }
    }
    if (GetParam() == 3) {
        EXPECT_NEAR(expected, 1.13077, 5e-6); // [-]
        EXPECT_LT(expected, 0.95 * legacyCost);
    }
    model.useShortestPath = false;
    model.AStar();
    EXPECT_EQ(model.path.list, legacyPath);
    EXPECT_DOUBLE_EQ(pathDistance(model), legacyCost);
}

INSTANTIATE_TEST_SUITE_P(Scenarios, ConstrainedShortestPathExamples, ::testing::Range(0, 4));

class ConstrainedShortestPathRefinement : public ::testing::TestWithParam<int>
{};

/*! @brief Compare shortest paths throughout the supported refinement range, including boundary shadow links. */
TEST_P(ConstrainedShortestPathRefinement, MatchesDijkstra)
{
    ConstrainedAttitudeManeuver model(GetParam());
    Node start, goal;
    start.sigma_BN[2] = -0.8; // [-]
    goal.sigma_BN[1] = 0.8;   // [-]
    model.GenerateGrid(start, goal);
    const double expected = dijkstra(endpoint(model, model.keyS), endpoint(model, model.keyG));
    model.useShortestPath = true;
    model.AStar();
    EXPECT_NEAR(pathDistance(model), expected, distanceTolerance);
}

INSTANTIATE_TEST_SUITE_P(Refinements, ConstrainedShortestPathRefinement, ::testing::Range(2, 21));

/*! @brief Use one quarter of the principal angle for either representation of a half-turn goal. */
TEST(ConstrainedShortestPath, PrincipalAngleHeuristic)
{
    for (double goalZ : { -1.0, 1.0 }) { // [-] Equivalent unit-sphere MRPs
        ConstrainedAttitudeManeuver model(5);
        Node start, goal;
        goal.sigma_BN[2] = goalZ;
        model.GenerateGrid(start, goal);
        model.useShortestPath = true;
        model.AStar();
        EXPECT_NEAR(endpoint(model, model.keyS)->heuristic, MPI / 4.0, distanceTolerance);
        EXPECT_DOUBLE_EQ(endpoint(model, model.keyG)->heuristic, 0.0);
    }
}

/*! @brief Exhaust a large reachable graph without the legacy expansion or storage limits. */
TEST(ConstrainedShortestPath, ExhaustsLargeDisconnectedGraph)
{
    ConstrainedAttitudeManeuver model(20);
    model.bskLogger.setLogLevel(BSK_SILENT);
    Node start, goal;
    goal.sigma_BN[2] = 0.8; // [-]
    model.GenerateGrid(start, goal);
    Node* destination = endpoint(model, model.keyG);
    std::size_t nodeCount = 0;
    for (auto& plane : model.NodesMap) {
        for (auto& row : plane.second) {
            for (auto& entry : row.second) {
                Node& node = entry.second;
                const auto end = std::remove(node.neighbors, node.neighbors + node.neighborCount, destination);
                node.neighborCount = static_cast<int>(end - node.neighbors);
                ++nodeCount;
            }
        }
    }
    ASSERT_GT(nodeCount, 10000U);
    EXPECT_TRUE(std::isinf(dijkstra(endpoint(model, model.keyS), destination)));
    model.useShortestPath = true;
    try {
        model.AStar();
        FAIL() << "A disconnected goal must not return a partial path.";
    } catch (const BasiliskError& error) {
        EXPECT_NE(std::string(error.what()).find("open list exhausted"), std::string::npos);
    }
    EXPECT_EQ(model.expandedNodeCount, nodeCount - 1);
    EXPECT_EQ(model.path.N, 0);
    EXPECT_THROW(model.UpdateState(0), BasiliskError);
}

/*! @brief Reconstruct paths longer than the former list capacity without recursion or truncation. */
TEST(ConstrainedShortestPath, ReconstructsLongPath)
{
    ConstrainedAttitudeManeuver model(20);
    const int nodeCount = 10050;
    for (int i = 0; i < nodeCount; ++i) {
        Node& node = model.NodesMap[i][0][0];
        node.sigma_BN[0] = 0.5 * i / (nodeCount - 1); // [-]
        if (i > 0) {
            model.NodesMap[i - 1][0][0].appendNeighbor(&node);
        }
    }
    model.keyS[0] = model.keyS[1] = model.keyS[2] = 0;
    model.keyG[0] = nodeCount - 1;
    model.keyG[1] = model.keyG[2] = 0;
    model.useShortestPath = true;
    model.AStar();
    EXPECT_EQ(model.path.N, nodeCount);
    EXPECT_EQ(model.expandedNodeCount, static_cast<std::size_t>(nodeCount - 1));
    EXPECT_NEAR(pathDistance(model), 0.5, distanceTolerance); // [-]
}
} // namespace
