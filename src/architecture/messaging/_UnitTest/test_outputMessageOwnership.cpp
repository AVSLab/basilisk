/*
 * Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 * Distributed under the ISC license; see LICENSE.
 */

#include "architecture/messaging/ownedMessage.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <malloc.h>
#endif

#include "architecture/utilities/bskLogging.h"
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
#include "simulation/environment/spiceInterface/spiceInterface.h"
#include "simulation/mujocoDynamics/thrOnTimeToForce/thrOnTimeToForce.h"
#include "simulation/vizard/dataFileToViz/dataFileToViz.h"

namespace {
// This test executable replaces ordinary and aligned new/delete to detect missing
// cleanup and inject allocation failures on platforms without LeakSanitizer. Direct
// malloc/free calls are outside the probe. Bookkeeping uses fixed storage and never
// allocates. Tests and module setup run on one thread.
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

    void beforeAllocation()
    {
        if (this->failAfter == 0) {
            throw std::bad_alloc();
        }
        if (this->failAfter > 0) {
            --this->failAfter;
        }
    }

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
        activeProbe->beforeAllocation();
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

void*
operator new(std::size_t size, std::align_val_t alignment)
{
    if (activeProbe != nullptr) {
        activeProbe->beforeAllocation();
    }
    void* pointer = nullptr;
#ifdef _WIN32
    pointer = _aligned_malloc(size == 0 ? 1 : size, static_cast<std::size_t>(alignment));
#else
    if (posix_memalign(&pointer, static_cast<std::size_t>(alignment), size == 0 ? 1 : size) != 0) {
        throw std::bad_alloc();
    }
#endif
    if (pointer == nullptr) {
        throw std::bad_alloc();
    }
    if (activeProbe != nullptr) {
        activeProbe->remember(pointer);
    }
    return pointer;
}

void
operator delete(void* pointer, std::align_val_t) noexcept
{
    if (activeProbe != nullptr) {
        activeProbe->forget(pointer);
    }
#ifdef _WIN32
    _aligned_free(pointer);
#else
    std::free(pointer);
#endif
}

void*
operator new[](std::size_t size, std::align_val_t alignment)
{
    return ::operator new(size, alignment);
}
void
operator delete[](void* pointer, std::align_val_t alignment) noexcept
{
    ::operator delete(pointer, alignment);
}
void
operator delete(void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
    ::operator delete(pointer, alignment);
}
void
operator delete[](void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
    ::operator delete(pointer, alignment);
}

namespace {
struct alignas(64) OverAlignedValue
{
    std::byte value{};
};
static_assert(alignof(OverAlignedValue) > __STDCPP_DEFAULT_NEW_ALIGNMENT__);

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
#if defined(_MSC_VER) && _ITERATOR_DEBUG_LEVEL != 0
    // MSVC's debug STL allocates iterator proxies inside noexcept container
    // constructors. A global-new failure there terminates instead of unwinding.
    // Release runs these sweeps; ordinary ownership checks still run in Debug.
    GTEST_SKIP() << "Global allocation-failure sweeps cannot unwind MSVC debug iterator-proxy construction";
#endif
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

/** @brief Aligned scalar and array storage is tracked through sized and unsized deletion. */
TEST(AllocationProbe, TracksAlignedAllocationsAndAllDeleteForms)
{
    constexpr auto alignment = std::align_val_t{ alignof(OverAlignedValue) };
    constexpr std::size_t arraySize = 3 * sizeof(OverAlignedValue);
    for (bool sizedDelete : { false, true }) {
        std::array<std::size_t, 4> counts{};
        bool addressesAligned;
        {
            AllocationProbe probe;
            // Direct calls prevent the compiler from eliding the allocations under test.
            void* scalar = ::operator new(sizeof(OverAlignedValue), alignment);
            counts[0] = probe.count;
            void* array = ::operator new[](arraySize, alignment);
            counts[1] = probe.count;
            addressesAligned = reinterpret_cast<std::uintptr_t>(scalar) % alignof(OverAlignedValue) == 0 &&
                               reinterpret_cast<std::uintptr_t>(array) % alignof(OverAlignedValue) == 0;
            if (sizedDelete) {
                ::operator delete(scalar, sizeof(OverAlignedValue), alignment);
                counts[2] = probe.count;
                ::operator delete[](array, arraySize, alignment);
            } else {
                ::operator delete(scalar, alignment);
                counts[2] = probe.count;
                ::operator delete[](array, alignment);
            }
            counts[3] = probe.count;
        }
        EXPECT_TRUE(addressesAligned);
        EXPECT_EQ(counts, (std::array<std::size_t, 4>{ 1, 2, 1, 0 }));
    }
}

/** @brief Failure injection covers aligned scalar and array allocations without leaking. */
TEST(AllocationProbe, InjectsAlignedScalarAndArrayAllocationFailures)
{
    for (bool arrayAllocation : { false, true }) {
        const auto factory = [arrayAllocation] {
            constexpr auto alignment = std::align_val_t{ alignof(OverAlignedValue) };
            if (arrayAllocation) {
                void* pointer = ::operator new[](3 * sizeof(OverAlignedValue), alignment);
                ::operator delete[](pointer, alignment);
            } else {
                void* pointer = ::operator new(sizeof(OverAlignedValue), alignment);
                ::operator delete(pointer, alignment);
            }
        };
        auto failed = runTrial(factory, 0);
        EXPECT_TRUE(failed.first);
        EXPECT_EQ(failed.second, 0U);
        auto successful = runTrial(factory, 1);
        EXPECT_FALSE(successful.first);
        EXPECT_EQ(successful.second, 0U);
    }
}

/** @brief Growing either vector preserves existing message addresses and subscriptions. */
TEST(OwnedMessageHelper, PreservesViewsAndPayloadsDuringGrowth)
{
    std::vector<std::unique_ptr<Message<SCStatesMsgPayload>>> owners;
    std::vector<Message<SCStatesMsgPayload>*> views;
    addOwnedMessage(owners, views);
    auto* firstMessage = views.front();
    auto reader = firstMessage->addSubscriber();
    SCStatesMsgPayload payload{};
    payload.r_BN_N[0] = 125.5; // [m]
    firstMessage->write(&payload, 1, 0);

    const auto initialOwnerCapacity = owners.capacity();
    const auto initialViewCapacity = views.capacity();
    for (std::size_t index = 0; index < 64; ++index) {
        addOwnedMessage(owners, views);
    }

    ASSERT_EQ(owners.size(), views.size());
    EXPECT_GT(owners.capacity(), initialOwnerCapacity);
    EXPECT_GT(views.capacity(), initialViewCapacity);
    for (std::size_t index = 0; index < owners.size(); ++index) {
        EXPECT_EQ(owners[index].get(), views[index]);
    }
    EXPECT_EQ(views.front(), firstMessage);
    EXPECT_DOUBLE_EQ(reader().r_BN_N[0], payload.r_BN_N[0]);
    EXPECT_FALSE(views.back()->addSubscriber().isWritten());
}

/** @brief A shared owner vector can back separate groups of borrowed messages. */
TEST(OwnedMessageHelper, SupportsSeparateViewGroups)
{
    std::vector<std::unique_ptr<Message<SCStatesMsgPayload>>> owners;
    std::vector<Message<SCStatesMsgPayload>*> firstGroup;
    std::vector<Message<SCStatesMsgPayload>*> secondGroup;
    addOwnedMessage(owners, firstGroup);
    addOwnedMessage(owners, secondGroup);
    addOwnedMessage(owners, firstGroup);

    ASSERT_EQ(owners.size(), 3U);
    ASSERT_EQ(firstGroup.size(), 2U);
    ASSERT_EQ(secondGroup.size(), 1U);
    EXPECT_EQ(firstGroup[0], owners[0].get());
    EXPECT_EQ(secondGroup[0], owners[1].get());
    EXPECT_EQ(firstGroup[1], owners[2].get());
}

/** @brief Allocation failures preserve both collections and release the rejected message. */
TEST(OwnedMessageHelper, RollsBackEveryAllocationFailure)
{
    for (bool populated : { false, true }) {
        for (bool reserveOwners : { false, true }) {
            bool reachedSuccess = false;
            int failureCount = 0;
            for (int failAfter = 0; failAfter < 16; ++failAfter) {
                SCOPED_TRACE(failAfter);
                bool failed = false;
                bool sizesCorrect;
                bool entriesPreserved = true;
                std::size_t remainingAllocations;
                {
                    AllocationProbe probe;
                    {
                        std::vector<std::unique_ptr<Message<SCStatesMsgPayload>>> owners;
                        std::vector<Message<SCStatesMsgPayload>*> views;
                        if (populated) {
                            addOwnedMessage(owners, views);
                            // Fill the view storage so the next append must allocate.
                            while (views.size() < views.capacity()) {
                                addOwnedMessage(owners, views);
                            }
                        }
                        if (reserveOwners) {
                            owners.reserve(owners.size() + 1);
                        }
                        const auto previousViews = views;
                        probe.failAfter = failAfter;
                        try {
                            addOwnedMessage(owners, views);
                        } catch (const std::bad_alloc&) {
                            failed = true;
                            ++failureCount;
                        }
                        probe.failAfter = -1;
                        const auto expectedSize = previousViews.size() + (failed ? 0U : 1U);
                        sizesCorrect = owners.size() == expectedSize && views.size() == expectedSize;
                        for (std::size_t index = 0; index < previousViews.size(); ++index) {
                            entriesPreserved = entriesPreserved && index < owners.size() && index < views.size() &&
                                               owners[index].get() == previousViews[index] &&
                                               views[index] == previousViews[index];
                        }
                        if (!failed) {
                            entriesPreserved = entriesPreserved && !owners.empty() && !views.empty() &&
                                               owners.back().get() == views.back();
                        }
                    }
                    remainingAllocations = probe.count;
                }
                EXPECT_TRUE(sizesCorrect);
                EXPECT_TRUE(entriesPreserved);
                EXPECT_EQ(remainingAllocations, 0U);
                if (!failed) {
                    reachedSuccess = true;
                    break;
                }
            }
            EXPECT_TRUE(reachedSuccess);
            EXPECT_GE(failureCount, 2); // At least message construction and view-vector growth must fail.
        }
    }
}

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

TEST(SpiceBufferOwnership, RejectedSpacecraftNamesReleaseAllocatedOutputs)
{
    // Rejecting the name after output allocation must still allow the model to release those outputs.
    bool rejected = false;
    expectCleanup([&rejected] {
        SpiceInterface model;
        try {
            model.addSpacecraftNames({ std::string(MAX_BODY_NAME_LENGTH, 'x') });
        } catch (const BasiliskError&) {
            rejected = true;
        }
    });
    EXPECT_TRUE(rejected);
}

TEST(SpiceBufferOwnership, ConstructorUnwindsAfterAllocationFailure)
{
    expectCleanupAfterAllocationFailure([] { SpiceInterface model; });
}

TEST(SpiceBufferOwnership, SpacecraftSetupUnwindsAfterAllocationFailure)
{
    expectCleanupAfterAllocationFailure([] {
        SpiceInterface model;
        model.addSpacecraftNames({"EARTH", "MARS"});
    });
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
