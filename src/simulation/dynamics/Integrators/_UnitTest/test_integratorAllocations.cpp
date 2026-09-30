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

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <atomic>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <thread>
#include <typeinfo>

#include "allocationProbe.h"
#include "allocationTracker.h"
#include "integratorTestAccess.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorDRI1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorEulerHeun.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorMayurama.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorRDI1WM.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorRKMil.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorRS.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSIESME.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSOSRA.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSOSRI.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSRA1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSRIW1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorW2Ito1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorW2Ito2.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorAdaptiveRungeKutta.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorRungeKutta.h"

namespace {
using integrator_allocation_test::AllocationApi;
using integrator_allocation_test::AllocationSnapshot;
using integrator_allocation_test::ScopedAllocationSuspension;
using integrator_allocation_test::ScopedAllocationTracking;
using integrator_allocation_test::trackAllocations;
using integrator_step_test::stepIntegrator;

class AllocationDynamics final : public DynamicObject
{
  public:
    explicit AllocationDynamics(bool stochastic, size_t noiseCount = 1)
      : stochastic(stochastic)
      , noiseCount(stochastic ? noiseCount : 0)
    {
        this->reset();
    }

    void reset()
    {
        StateSpec spec;
        spec.state = { 4, 1 };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        spec.noiseCount = this->noiseCount;
        this->state = this->dynManager.registerState("allocationState", spec);
        Eigen::MatrixXd initial(4, 1);
        initial << 1.0, -2.0, 3.0, -4.0;
        this->state->setState(initial);
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double time, double timeStep) override
    {
        ScopedAllocationSuspension suspendUserCode;
        const auto value = this->state->stateView();
        auto derivative = this->state->derivativeView();
        derivative(0) = value(1) + time;
        derivative(1) = -value(0) + timeStep;
        derivative(2) = 0.5 * value(3);
        derivative(3) = -0.25 * value(2);

        if (this->allocateInUserCode) {
            volatile void* memory = std::malloc(32);
            std::free(const_cast<void*>(memory));
        }
    }

    void equationsOfMotionDiffusion(double, double) override
    {
        ScopedAllocationSuspension suspendUserCode;
        if (this->stochastic) {
            for (size_t noise = 0; noise < this->noiseCount; ++noise) {
                auto diffusion = this->state->diffusionView(noise);
                const double scale = static_cast<double>(noise + 1);
                diffusion << 0.1 * scale, 0.2 * scale, 0.3 * scale, 0.4 * scale;
            }
        }
    }

    StateData* state = nullptr;
    bool stochastic = false;
    size_t noiseCount = 0;
    bool allocateInUserCode = false;
};

RKCoefficients<4>
rk4Coefficients()
{
    RKCoefficients<4> coefficients;
    coefficients.aMatrix = { { { { 0.0, 0.0, 0.0, 0.0 } },
                               { { 0.5, 0.0, 0.0, 0.0 } },
                               { { 0.0, 0.5, 0.0, 0.0 } },
                               { { 0.0, 0.0, 1.0, 0.0 } } } };
    coefficients.bArray = { { 1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0 } };
    coefficients.cArray = { { 0.0, 0.5, 0.5, 1.0 } };
    return coefficients;
}

RKAdaptiveCoefficients<4>
adaptiveCoefficients()
{
    RKAdaptiveCoefficients<4> coefficients;
    coefficients.aMatrix = { { { { 0.0, 0.0, 0.0, 0.0 } },
                               { { 0.5, 0.0, 0.0, 0.0 } },
                               { { 0.0, 0.75, 0.0, 0.0 } },
                               { { 2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0, 0.0 } } } };
    coefficients.bArray = { { 7.0 / 24.0, 0.25, 1.0 / 3.0, 0.125 } };
    coefficients.bStarArray = { { 2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0, 0.0 } };
    coefficients.cArray = { { 0.0, 0.5, 0.75, 1.0 } };
    return coefficients;
}

class ScopedEigenNoMalloc
{
  public:
    ScopedEigenNoMalloc()
      : previous(Eigen::internal::is_malloc_allowed())
    {
        Eigen::internal::set_is_malloc_allowed(false);
    }

    ~ScopedEigenNoMalloc() { Eigen::internal::set_is_malloc_allowed(this->previous); }

  private:
    bool previous;
};

class LegacyAllocatingNoiseGenerator final : public GaussianNoiseGenerator
{
  public:
    void setSeed(size_t) override {}

    GaussianNoiseSample generate(size_t m, double) override
    {
        ++this->generateCalls;
        GaussianNoiseSample sample;
        sample.dW = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(m), 0.125);
        sample.dZ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(m));
        return sample;
    }

    size_t generateCalls = 0;
};

template<typename Integrator>
AllocationSnapshot
trackStochasticStep(Integrator& integrator, double timeStep = 0.125)
{
    return trackAllocations([&]() {
        ScopedEigenNoMalloc noEigenMalloc;
        stepIntegrator(integrator, 0.0, timeStep);
    });
}

template<typename Integrator>
void
expectFullyTrackedStochasticStepDoesNotAllocate()
{
    SCOPED_TRACE(typeid(Integrator).name());
    AllocationDynamics dynamics(true);
    Integrator integrator(&dynamics);
    integrator.setRNGSeed(123456);
    stepIntegrator(integrator, 0.0, 0.0);
    EXPECT_EQ(trackStochasticStep(integrator).totalCalls(), 0U);
}

void
recordAllocatorCoverage()
{
    testing::Test::RecordProperty("allocator_interception",
                                  integrator_allocation_test::allocatorInterceptionDescription());
}
}

TEST(IntegratorAllocationTracker, CppOperatorPositiveControls)
{
    recordAllocatorCoverage();
    const AllocationSnapshot cppSnapshot = trackAllocations([]() {
        void* value = basilisk_allocation_probe_new(sizeof(int));
        basilisk_allocation_probe_delete(value);
    });
    EXPECT_GE(cppSnapshot[AllocationApi::CppNew], 1U);
    EXPECT_GE(cppSnapshot[AllocationApi::CppDelete], 1U);

#if defined(__cpp_aligned_new)
    const AllocationSnapshot cppAlignedSnapshot = trackAllocations([]() {
        void* memory = basilisk_allocation_probe_aligned_new(128, 64);
        basilisk_allocation_probe_aligned_delete(memory, 64);
    });
    EXPECT_GE(cppAlignedSnapshot[AllocationApi::CppAlignedNew], 1U);
    EXPECT_GE(cppAlignedSnapshot[AllocationApi::CppAlignedDelete], 1U);
#endif
}

TEST(IntegratorAllocationTracker, CAllocatorPositiveControlsWhenAvailable)
{
    recordAllocatorCoverage();
    if (!integrator_allocation_test::cAllocatorInterceptionAvailable()) {
        GTEST_SKIP() << integrator_allocation_test::allocatorInterceptionDescription();
    }

    const AllocationSnapshot mallocSnapshot = trackAllocations([]() {
        void* memory = basilisk_allocation_probe_malloc(31);
        basilisk_allocation_probe_free(memory);
    });
    EXPECT_GE(mallocSnapshot[AllocationApi::Malloc], 1U);
    EXPECT_GE(mallocSnapshot[AllocationApi::Free], 1U);

    const AllocationSnapshot callocSnapshot = trackAllocations([]() {
        void* memory = basilisk_allocation_probe_calloc(3, 17);
        basilisk_allocation_probe_free(memory);
    });
    EXPECT_GE(callocSnapshot[AllocationApi::Calloc], 1U);
    EXPECT_GE(callocSnapshot[AllocationApi::Free], 1U);

    const AllocationSnapshot reallocSnapshot = trackAllocations([]() {
        void* memory = basilisk_allocation_probe_malloc(19);
        memory = basilisk_allocation_probe_realloc(memory, 47);
        basilisk_allocation_probe_free(memory);
    });
    EXPECT_GE(reallocSnapshot[AllocationApi::Malloc], 1U);
    EXPECT_GE(reallocSnapshot[AllocationApi::Realloc], 1U);
    EXPECT_GE(reallocSnapshot[AllocationApi::Free], 1U);

#if !defined(_WIN32)
    const AllocationSnapshot alignedSnapshot = trackAllocations([]() {
        void* first = basilisk_allocation_probe_aligned_alloc(64, 128);
        basilisk_allocation_probe_free(first);

        void* second = nullptr;
        EXPECT_EQ(basilisk_allocation_probe_posix_memalign(&second, 64, 127), 0);
        basilisk_allocation_probe_free(second);
    });
    EXPECT_GE(alignedSnapshot[AllocationApi::AlignedAlloc], 1U);
    EXPECT_GE(alignedSnapshot[AllocationApi::PosixMemalign], 1U);
    EXPECT_GE(alignedSnapshot[AllocationApi::Free], 2U);
#endif
}

TEST(IntegratorAllocationTracker, CoverageDescriptionMarksPartialPlatforms)
{
    const std::string description = integrator_allocation_test::allocatorInterceptionDescription();
    recordAllocatorCoverage();
    if (integrator_allocation_test::cAllocatorInterceptionAvailable()) {
        EXPECT_EQ(description.find("partial coverage"), std::string::npos);
    } else {
        EXPECT_NE(description.find("partial coverage"), std::string::npos);
    }
}

TEST(IntegratorAllocationTracker, SuspensionIsNestedAndTrackingIsThreadLocal)
{
    std::atomic<bool> startWorker{ false };
    std::atomic<bool> workerDone{ false };
    std::thread worker([&]() {
        while (!startWorker.load(std::memory_order_acquire)) {
        }
        void* value = basilisk_allocation_probe_new(sizeof(int));
        basilisk_allocation_probe_delete(value);
        workerDone.store(true, std::memory_order_release);
    });

    const AllocationSnapshot snapshot = trackAllocations([&]() {
        {
            ScopedAllocationSuspension outer;
            ScopedAllocationSuspension inner;
            void* suspended = basilisk_allocation_probe_new(sizeof(int));
            basilisk_allocation_probe_delete(suspended);
        }

        startWorker.store(true, std::memory_order_release);
        while (!workerDone.load(std::memory_order_acquire)) {
        }

        void* tracked = basilisk_allocation_probe_new(sizeof(int));
        basilisk_allocation_probe_delete(tracked);
    });
    worker.join();

    EXPECT_EQ(snapshot[AllocationApi::CppNew], 1U);
    EXPECT_EQ(snapshot[AllocationApi::CppDelete], 1U);
}

TEST(IntegratorAllocationTracker, EigenRuntimeNoMallocPositiveControl)
{
    ASSERT_DEATH(
      {
          Eigen::internal::set_is_malloc_allowed(false);
          Eigen::MatrixXd matrix(64, 64);
          matrix.setZero();
      },
      "heap allocation is forbidden");
}

TEST(IntegratorAllocationTracker, FixedAdaptiveAndStochasticUseNoObservedAllocatorsAfterBind)
{
    recordAllocatorCoverage();
    AllocationDynamics fixedDynamics(false);
    AllocationDynamics adaptiveDynamics(false);
    AllocationDynamics stochasticDynamics(true);
    fixedDynamics.allocateInUserCode = true;
    adaptiveDynamics.allocateInUserCode = true;
    stochasticDynamics.allocateInUserCode = true;

    svIntegratorRungeKutta<4> fixedIntegrator(&fixedDynamics, rk4Coefficients());
    svIntegratorAdaptiveRungeKutta<4> adaptiveIntegrator(&adaptiveDynamics, adaptiveCoefficients(), 3.0);
    adaptiveIntegrator.setRelativeTolerance(1.0);
    adaptiveIntegrator.setAbsoluteTolerance(1.0);
    svStochasticIntegratorMayurama stochasticIntegrator(&stochasticDynamics);
    stochasticIntegrator.setRNGSeed(123456);

    stepIntegrator(fixedIntegrator, 0.0, 0.0);
    stepIntegrator(adaptiveIntegrator, 0.0, 0.0);
    stepIntegrator(stochasticIntegrator, 0.0, 0.0);

    const AllocationSnapshot fixedSnapshot = trackAllocations([&]() {
        ScopedEigenNoMalloc noEigenMalloc;
        stepIntegrator(fixedIntegrator, 0.0, 0.125);
    });
    const AllocationSnapshot adaptiveSnapshot = trackAllocations([&]() {
        ScopedEigenNoMalloc noEigenMalloc;
        stepIntegrator(adaptiveIntegrator, 0.0, 0.125);
    });
    const AllocationSnapshot stochasticSnapshot = trackAllocations([&]() {
        ScopedEigenNoMalloc noEigenMalloc;
        stepIntegrator(stochasticIntegrator, 0.0, 0.125);
    });

    EXPECT_EQ(fixedSnapshot.totalCalls(), 0U);
    EXPECT_EQ(adaptiveSnapshot.totalCalls(), 0U);
    EXPECT_EQ(stochasticSnapshot.totalCalls(), 0U);
}

TEST(IntegratorAllocationTracker, DynamicObjectIntegrationPhaseUsesNoObservedAllocatorsAfterBind)
{
    recordAllocatorCoverage();
    AllocationDynamics dynamics(false);
    dynamics.setIntegrator(new svIntegratorRungeKutta<4>(&dynamics, rk4Coefficients()));
    dynamics.integrateState(0);

    const AllocationSnapshot snapshot = trackAllocations([&]() {
        ScopedEigenNoMalloc noEigenMalloc;
        dynamics.integrateState(0);
    });

    EXPECT_EQ(snapshot.totalCalls(), 0U);
}

TEST(IntegratorAllocationTracker, StrongSRIAndReplayUseNoObservedAllocatorsAfterBind)
{
    recordAllocatorCoverage();
    AllocationDynamics sosriDynamics(true);
    AllocationDynamics sriw1Dynamics(true);
    AllocationDynamics replayDynamics(true);
    svStochasticIntegratorSOSRI sosri(&sosriDynamics);
    svStochasticIntegratorSRIW1 sriw1(&sriw1Dynamics);
    svStochasticIntegratorMayurama replay(&replayDynamics);
    sosri.setRNGSeed(123456);
    sriw1.setRNGSeed(123456);

    auto replayNoise = std::make_shared<PrescribedGaussianNoiseGenerator>();
    replayNoise->pushStep({ 0.125 });
    replayNoise->pushStep({ -0.25 });
    replay.setNoiseGenerator(replayNoise);

    stepIntegrator(sosri, 0.0, 0.0);
    stepIntegrator(sriw1, 0.0, 0.0);
    stepIntegrator(replay, 0.0, 0.0);

    const AllocationSnapshot sosriSnapshot = trackStochasticStep(sosri);
    const AllocationSnapshot sriw1Snapshot = trackStochasticStep(sriw1);
    EXPECT_EQ(sosriSnapshot.allocationCalls(), 0U);
    EXPECT_EQ(sriw1Snapshot.allocationCalls(), 0U);
    EXPECT_EQ(trackStochasticStep(replay).totalCalls(), 0U);
    EXPECT_EQ(replayNoise->remaining(), 1U);
}

TEST(IntegratorAllocationTracker, WeakItoFamiliesUseNoObservedAllocatorsAfterBind)
{
    recordAllocatorCoverage();
    AllocationDynamics w2ItoDynamics(true);
    AllocationDynamics dri1Dynamics(true);
    svStochasticIntegratorW2Ito1 w2Ito(&w2ItoDynamics);
    svStochasticIntegratorDRI1 dri1(&dri1Dynamics);
    w2Ito.setRNGSeed(123456);
    dri1.setRNGSeed(123456);

    stepIntegrator(w2Ito, 0.0, 0.0);
    stepIntegrator(dri1, 0.0, 0.0);

    EXPECT_EQ(trackStochasticStep(w2Ito).totalCalls(), 0U);
    EXPECT_EQ(trackStochasticStep(dri1).totalCalls(), 0U);
}

TEST(IntegratorAllocationTracker, WeakCrossNoisePathsUseNoObservedAllocatorsAfterBind)
{
    recordAllocatorCoverage();
    AllocationDynamics dri1Dynamics(true, 2);
    AllocationDynamics rs1Dynamics(true, 2);
    AllocationDynamics rs2Dynamics(true, 2);
    svStochasticIntegratorDRI1 dri1(&dri1Dynamics);
    svStochasticIntegratorRS1 rs1(&rs1Dynamics);
    svStochasticIntegratorRS2 rs2(&rs2Dynamics);
    dri1.setRNGSeed(123456);
    rs1.setRNGSeed(123456);
    rs2.setRNGSeed(123456);

    stepIntegrator(dri1, 0.0, 0.0);
    stepIntegrator(rs1, 0.0, 0.0);
    stepIntegrator(rs2, 0.0, 0.0);

    EXPECT_EQ(trackStochasticStep(dri1).totalCalls(), 0U);
    EXPECT_EQ(trackStochasticStep(rs1).totalCalls(), 0U);
    EXPECT_EQ(trackStochasticStep(rs2).totalCalls(), 0U);
}

TEST(IntegratorAllocationTracker, AllConcreteStochasticFamiliesUseNoObservedAllocatorsAfterBind)
{
    recordAllocatorCoverage();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorMayurama>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorEulerHeun>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorRKMil>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorRDI1WM>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorW2Ito1>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorW2Ito2>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorDRI1>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorDRI1NM>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorRI1>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorRI3>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorRI5>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorRI6>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorRS1>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorRS2>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorSIEA>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorSMEA>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorSIEB>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorSMEB>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorSOSRA>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorSRA1>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorSOSRI>();
    expectFullyTrackedStochasticStepDoesNotAllocate<svStochasticIntegratorSRIW1>();
}

TEST(IntegratorAllocationTracker, LegacyNoiseFallbackRemainsCompatible)
{
    recordAllocatorCoverage();
    AllocationDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);
    const auto generator = std::make_shared<LegacyAllocatingNoiseGenerator>();
    integrator.setNoiseGenerator(generator);
    stepIntegrator(integrator, 0.0, 0.0);
    EXPECT_EQ(generator->generateCalls, 0U);

    const double timeStep = 0.125; // [s]
    const AllocationSnapshot snapshot = trackAllocations([&]() { stepIntegrator(integrator, 0.0, timeStep); });

    EXPECT_EQ(generator->generateCalls, 1U);
    // Euler-Maruyama: initial state + timeStep * drift + 0.125 * diffusion.
    Eigen::Vector4d expected;
    expected << 0.7625, -2.084375, 2.7875, -4.04375;
    EXPECT_TRUE(dynamics.state->stateView().isApprox(expected, 1.0e-14));

    // Eigen allocates through C APIs, which the Windows tracker cannot intercept.
    if (integrator_allocation_test::cAllocatorInterceptionAvailable()) {
        EXPECT_GT(snapshot.totalCalls(), 0U);
    }
}
