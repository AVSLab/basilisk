/*
 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 Distributed under the ISC license; see LICENSE.
 */

#include "simulation/dynamics/_GeneralModuleFiles/gravityEffector.h"

#include <gtest/gtest.h>
#include <memory>

/** @brief Reject a partial shared sample before changing the energy accumulator, then recover on a peer's refresh. */
TEST(GravityEffectorEnergy, IncompleteSharedSamplePreservesAccumulator)
{
    auto fixed = std::make_shared<GravBodyData>();
    fixed->mu = 64.0; // [m^3/s^2]
    auto moving = std::make_shared<GravBodyData>();
    moving->mu = 36.0; // [m^3/s^2]
    Message<SpicePlanetStateMsgPayload> original, replacement;
    SpicePlanetStateMsgPayload payload{};
    payload.PositionVector[0] = 2.0; // [m]
    original.write(&payload, 0, 0);
    moving->planetBodyInMsg.subscribeTo(&original);

    GravityEffector first, peer;
    const Eigen::Vector3d position{ 8.0, 0.0, 0.0 }; // [m]
    constexpr double initial = 100.0;                // [m^2/s^2]
    for (auto* gravity : { &first, &peer }) {
        gravity->addGravBody(fixed);
        gravity->addGravBody(moving);
        gravity->Reset(0);
        double potential = initial;
        gravity->updateEnergyContributions(position, potential, 0);
        EXPECT_DOUBLE_EQ(potential, initial - 14.0); // [m^2/s^2]
    }

    moving->planetBodyInMsg.subscribeTo(&replacement);
    first.UpdateState(0);
    double potential = initial;
    for (auto* gravity : { &first, &peer }) {
        EXPECT_THROW(gravity->updateEnergyContributions(position, potential, 0), BasiliskError);
        EXPECT_DOUBLE_EQ(potential, initial);
    }
    payload.PositionVector[0] = 5.0; // [m]
    replacement.write(&payload, 0, 0);
    EXPECT_THROW(first.updateEnergyContributions(position, potential, 0), BasiliskError);
    EXPECT_DOUBLE_EQ(potential, initial);

    peer.UpdateState(0);
    for (auto* gravity : { &first, &peer }) {
        potential = initial;
        gravity->updateEnergyContributions(position, potential, 0);
        EXPECT_DOUBLE_EQ(potential, initial - 20.0); // [m^2/s^2]
    }
}
