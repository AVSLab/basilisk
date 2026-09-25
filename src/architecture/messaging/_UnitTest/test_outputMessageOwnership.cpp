/*
 * Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 * Distributed under the ISC license; see LICENSE.
 */

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdlib>
#include <new>
#include <utility>

#include "fswAlgorithms/effectorInterfaces/hingedJointArrayMotor/hingedJointArrayMotor.h"
#include "fswAlgorithms/effectorInterfaces/jointMotionCompensator/jointMotionCompensator.h"
#include "fswAlgorithms/effectorInterfaces/thrJointCompensation/thrJointCompensation.h"
#include "simulation/deviceInterface/facetedSpacecraftModel/facetedSpacecraftModel.h"
#include "simulation/deviceInterface/facetedSpacecraftProjectedArea/facetedSpacecraftProjectedArea.h"
#include "simulation/dynamics/dualHingedRigidBodies/dualHingedRigidBodyStateEffector.h"
#include "simulation/dynamics/spinningBodies/spinningBodiesTwoDOF/spinningBodyTwoDOFStateEffector.h"
#include "simulation/environment/ExponentialAtmosphere/exponentialAtmosphere.h"
#include "simulation/environment/ZeroWindModel/zeroWindModel.h"
#include "simulation/environment/groundMapping/groundMapping.h"
#include "simulation/environment/magneticFieldCenteredDipole/magneticFieldCenteredDipole.h"
#include "simulation/environment/spaceWeatherData/spaceWeatherData.h"
#include "simulation/mujocoDynamics/thrOnTimeToForce/thrOnTimeToForce.h"
#include "simulation/vizard/dataFileToViz/dataFileToViz.h"

namespace {
// This test executable replaces ordinary new/delete so it can detect actual missing
// cleanup and inject allocation failures on platforms without LeakSanitizer. Bookkeeping
// uses fixed storage and never allocates. Tests and module setup run on one thread.
struct AllocationProbe;
AllocationProbe* activeProbe = nullptr;

struct AllocationProbe
{
    std::array<void*, 4096> live{};
    std::size_t count = 0;
    int failAfter = -1;

    explicit AllocationProbe(int failAfter = -1)
      : failAfter(failAfter)
    {
        activeProbe = this;
    }
    ~AllocationProbe() { activeProbe = nullptr; }

    void remember(void* pointer)
    {
        if (this->count == this->live.size()) {
            std::abort();
        }
        this->live[this->count++] = pointer;
    }

    void forget(void* pointer)
    {
        for (std::size_t index = 0; index < this->count; ++index) {
            if (this->live[index] == pointer) {
                this->live[index] = this->live[--this->count];
                return;
            }
        }
    }
};
} // namespace

void*
operator new(std::size_t size)
{
    if (activeProbe != nullptr) {
        if (activeProbe->failAfter == 0) {
            throw std::bad_alloc();
        }
        if (activeProbe->failAfter > 0) {
            --activeProbe->failAfter;
        }
    }
    void* pointer = std::malloc(size == 0 ? 1 : size);
    if (pointer == nullptr) {
        throw std::bad_alloc();
    }
    if (activeProbe != nullptr) {
        activeProbe->remember(pointer);
    }
    return pointer;
}

void
operator delete(void* pointer) noexcept
{
    if (activeProbe != nullptr) {
        activeProbe->forget(pointer);
    }
    std::free(pointer);
}

void*
operator new[](std::size_t size)
{
    return ::operator new(size);
}
void
operator delete[](void* pointer) noexcept
{
    ::operator delete(pointer);
}
void
operator delete(void* pointer, std::size_t) noexcept
{
    ::operator delete(pointer);
}
void
operator delete[](void* pointer, std::size_t) noexcept
{
    ::operator delete(pointer);
}

namespace {
template<typename Factory>
std::pair<bool, std::size_t>
runTrial(const Factory& factory, int failAfter = -1)
{
    AllocationProbe probe(failAfter);
    bool failed = false;
    try {
        factory();
    } catch (const std::bad_alloc&) {
        failed = true;
    }
    return { failed, probe.count };
}

template<typename Factory>
void
expectCleanup(const Factory& factory)
{
    factory(); // Warm up library caches before measuring module-owned allocations.
    auto result = runTrial(factory);
    EXPECT_FALSE(result.first);
    EXPECT_EQ(result.second, 0U);
}

template<typename Factory>
void
expectCleanupAfterAllocationFailure(const Factory& factory)
{
    factory();
    // Fail each allocation in turn, including those between paired output vectors.
    for (int failAfter = 0; failAfter < 256; ++failAfter) {
        auto result = runTrial(factory, failAfter);
        ASSERT_EQ(result.second, 0U) << "Unreleased allocation when failing allocation " << failAfter;
        if (!result.first) {
            return;
        }
    }
    FAIL() << "Allocation-failure sweep never reached successful setup";
}
} // namespace

TEST(OutputMessageOwnership, EnvironmentModelsReleaseGrowingOutputCollections)
{
    expectCleanup([] {
        Message<SCStatesMsgPayload> input;
        ExponentialAtmosphere atmosphere;
        MagneticFieldCenteredDipole magneticField;
        ZeroWindModel wind;
        for (int spacecraft = 0; spacecraft < 20; ++spacecraft) {
            atmosphere.addSpacecraftToModel(&input);
            magneticField.addSpacecraftToModel(&input);
            wind.addSpacecraftToModel(&input);
        }
    });
}

TEST(OutputMessageOwnership, PairedEnvironmentOutputsUnwindAfterAllocationFailure)
{
    expectCleanupAfterAllocationFailure([] {
        GroundMapping model;
        Eigen::Vector3d location = Eigen::Vector3d::Zero(); // [m]
        model.addPointToModel(location);
        model.addPointToModel(location);
    });
}

TEST(OutputMessageOwnership, FixedEnvironmentOutputsUnwindAfterConstructorFailure)
{
    expectCleanupAfterAllocationFailure([] { SpaceWeatherData model; });
}

TEST(OutputMessageOwnership, TwoAxisEffectorOutputsAreReleased)
{
    expectCleanup([] { SpinningBodyTwoDOFStateEffector model; });
}

TEST(OutputMessageOwnership, TwoAxisEffectorOutputsUnwindAfterConstructorFailure)
{
    expectCleanupAfterAllocationFailure([] { SpinningBodyTwoDOFStateEffector model; });
    expectCleanupAfterAllocationFailure([] { DualHingedRigidBodyStateEffector model; });
}

TEST(OutputMessageOwnership, JointControllerOutputsAreReleased)
{
    expectCleanup([] {
        HingedJointArrayMotor motor;
        ThrJointCompensation compensation;
        JointMotionCompensator motion;
        for (int joint = 0; joint < 20; ++joint) {
            motor.addHingedJoint();
            compensation.addHingedJoint();
            motion.addSpacecraft();
        }
    });
}

TEST(OutputMessageOwnership, JointControllerOutputsUnwindAfterAllocationFailure)
{
    expectCleanupAfterAllocationFailure([] {
        JointMotionCompensator model;
        model.addSpacecraft();
        model.addSpacecraft();
    });
}

TEST(OutputMessageOwnership, ThrusterForceOutputsAreReleased)
{
    expectCleanup([] {
        ThrOnTimeToForce model;
        for (int thruster = 0; thruster < 20; ++thruster) {
            model.addThruster();
        }
    });
}

TEST(OutputMessageOwnership, NestedVisualizationOutputsAreReleased)
{
    expectCleanup([] {
        DataFileToViz model;
        model.setNumOfSatellites(2);
        model.appendThrClusterMap({ ThrClusterMap{} }, { 3 });
        model.appendThrClusterMap({ ThrClusterMap{}, ThrClusterMap{} }, { 2, 1 });
        model.appendNumOfRWs(4);
        model.appendNumOfRWs(2);
    });
}

TEST(OutputMessageOwnership, NestedVisualizationOutputsUnwindAfterAllocationFailure)
{
    expectCleanupAfterAllocationFailure([] {
        DataFileToViz model;
        model.setNumOfSatellites(2);
        model.appendThrClusterMap({ ThrClusterMap{} }, { 3 });
        model.appendNumOfRWs(4);
    });
}

TEST(OutputMessageOwnership, FacetReconfigurationReleasesReplacedMessages)
{
    FacetedSpacecraftModel model;
    FacetedSpacecraftProjectedArea projectedArea;
    // Warm up vector capacity so only message allocations remain during measurement.
    model.setNumTotalFacets(4);
    projectedArea.setNumFacets(4);
    std::size_t remaining;
    {
        AllocationProbe probe;
        model.setNumTotalFacets(3);
        projectedArea.setNumFacets(3);
        model.setNumTotalFacets(0);
        projectedArea.setNumFacets(0);
        remaining = probe.count;
    }
    EXPECT_EQ(remaining, 0U);
}

TEST(OutputMessageOwnership, FacetOutputsUnwindAfterAllocationFailure)
{
    expectCleanupAfterAllocationFailure([] {
        FacetedSpacecraftModel model;
        model.setNumTotalFacets(3);
        model.setNumTotalFacets(1);
    });
    expectCleanupAfterAllocationFailure([] {
        FacetedSpacecraftProjectedArea model;
        model.setNumFacets(3);
        model.setNumFacets(1);
    });
}
