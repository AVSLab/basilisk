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

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "fswAlgorithms/orbitControl/lambertPlanner/lambertPlanner.h"
#include "fswAlgorithms/orbitControl/lambertSolver/lambertSolver.h"
#include <array>
#include <gtest/gtest.h>
#include <initializer_list>
#include <limits>

namespace {
constexpr std::array<int, 4> invalidDirections = { std::numeric_limits<int>::min(),
                                                   -1,
                                                   2,
                                                   std::numeric_limits<int>::max() };
}

/** @brief Zero-initialized messages and new planners default to prograde. */
TEST(LambertTransferDirection, DefaultsToPrograde)
{
    LambertProblemMsgPayload problem{};
    LambertPlanner planner;
    EXPECT_EQ(problem.transferDirection, PROGRADE);
    EXPECT_EQ(planner.getTransferDirection(), PROGRADE);
}

/** @brief Validate raw integers without changing the last accepted planner direction. */
TEST(LambertTransferDirection, PlannerRejectsInvalidIntegers)
{
    LambertPlanner planner;
    planner.setTransferDirection(RETROGRADE);
    for (int direction : invalidDirections) {
        SCOPED_TRACE(direction);
        EXPECT_THROW(planner.setTransferDirection(direction), BasiliskError);
        EXPECT_EQ(planner.getTransferDirection(), RETROGRADE);
    }
    planner.setTransferDirection(PROGRADE);
    EXPECT_EQ(planner.getTransferDirection(), PROGRADE);
}

/** @brief Message transport preserves raw integers until the solver validates them. */
TEST(LambertTransferDirection, SolverRejectsInvalidIntegers)
{
    for (SolverMethod method : { GOODING, IZZO }) {
        SCOPED_TRACE(method);
        LambertProblemMsgPayload problem{};
        problem.solverMethod = method;
        problem.mu = 3.986004360e14;   // [m^3/s^2]
        problem.transferTime = 2000.0; // [s]
        problem.r1_N[0] = 8.0e6;       // [m]
        problem.r2_N[1] = 8.0e6;       // [m]
        Message<LambertProblemMsgPayload> message;
        LambertSolver solver;
        auto solutionReader = solver.lambertSolutionOutMsg.addSubscriber();
        solver.lambertProblemInMsg.subscribeTo(&message);
        solver.Reset(0);
        for (int direction : invalidDirections) {
            SCOPED_TRACE(direction);
            problem.transferDirection = direction;
            message.write(&problem, 0, 0);
            EXPECT_EQ(solver.lambertProblemInMsg().transferDirection, direction);
            EXPECT_THROW(solver.UpdateState(0), BasiliskError);
        }
        problem.transferDirection = RETROGRADE;
        message.write(&problem, 0, 0);
        EXPECT_NO_THROW(solver.UpdateState(0));
        const auto solution = solutionReader();
        ASSERT_EQ(solution.valid, 1);
        EXPECT_LT(solution.v1_N[1], 0.0); // [m/s]
    }
}
