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

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "allocationTracker.h"
#include "integratorTestAccess.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorRS.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSIESME.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSOSRA.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSOSRI.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSRA1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSRIW1.h"

namespace {
using integrator_step_test::stepIntegrator;

uint64_t
doubleBits(double value)
{
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

class TableauDynamics final : public DynamicObject
{
  public:
    explicit TableauDynamics(size_t noiseCount)
    {
        StateSpec alphaSpec;
        alphaSpec.state = { 1, 1 };
        alphaSpec.derivative = alphaSpec.state;
        alphaSpec.diffusionTangent = alphaSpec.state;
        alphaSpec.noiseCount = noiseCount;
        this->alpha = this->dynManager.registerState("alpha", alphaSpec);

        StateSpec zetaSpec = alphaSpec;
        zetaSpec.noiseCount = noiseCount == 2 ? 1 : noiseCount;
        this->zeta = this->dynManager.registerState("zeta", zetaSpec);
        if (noiseCount == 1) {
            this->dynManager.registerSharedNoiseSource({ { *this->alpha, 0 }, { *this->zeta, 0 } });
        } else if (noiseCount == 2) {
            this->dynManager.registerSharedNoiseSource({ { *this->alpha, 1 }, { *this->zeta, 0 } });
        }

        this->alpha->stateView()(0, 0) = 0.75;
        this->zeta->stateView()(0, 0) = -1.25;
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double time, double timeStep) override
    {
        const double alphaValue = this->alpha->stateView()(0, 0);
        const double zetaValue = this->zeta->stateView()(0, 0);
        this->alpha->derivativeView()(0, 0) = 0.125 * alphaValue - 0.375 * zetaValue + time;
        this->zeta->derivativeView()(0, 0) = alphaValue * zetaValue + 0.25 * timeStep;
    }

    void equationsOfMotionDiffusion(double time, double) override
    {
        if (this->throwDuringDiffusion) {
            throw std::runtime_error("deliberate post-sample failure");
        }
        const double alphaValue = this->alpha->stateView()(0, 0);
        const double zetaValue = this->zeta->stateView()(0, 0);
        if (this->alpha->getNumNoiseSources() > 0) {
            this->alpha->diffusionView(0)(0, 0) = 0.5 + 0.125 * alphaValue;
        }
        if (this->alpha->getNumNoiseSources() > 1) {
            this->alpha->diffusionView(1)(0, 0) = -0.25 * zetaValue + 0.0625 * time;
        }
        if (this->zeta->getNumNoiseSources() > 0) {
            this->zeta->diffusionView(0)(0, 0) = 0.75 + 0.2 * alphaValue - 0.1 * zetaValue;
        }
    }

    StateData* alpha = nullptr;
    StateData* zeta = nullptr;
    bool throwDuringDiffusion = false;
};

class SideEffectDiffusionDynamics final : public DynamicObject
{
  public:
    explicit SideEffectDiffusionDynamics(bool mutateUnboundState)
      : mutateUnboundState(mutateUnboundState)
    {
        StateSpec drivenSpec;
        drivenSpec.state = { 1, 1 };
        drivenSpec.derivative = drivenSpec.state;
        drivenSpec.diffusionTangent = drivenSpec.state;
        drivenSpec.noiseCount = 2;
        this->driven = this->dynManager.registerState("driven", drivenSpec);

        StateSpec observerSpec;
        observerSpec.state = { 1, 1 };
        observerSpec.derivative = observerSpec.state;
        observerSpec.diffusionTangent = observerSpec.state;
        this->observer = this->dynManager.registerState("observer", observerSpec);

        this->driven->stateView()(0, 0) = 0.75;
        this->observer->stateView()(0, 0) = -0.25;
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override
    {
        this->driven->derivativeView()(0, 0) = 0.125 * this->driven->stateView()(0, 0);
        this->observer->derivativeView()(0, 0) = 0.0;
    }

    void equationsOfMotionDiffusion(double, double) override
    {
        const double observerValue = this->observer->stateView()(0, 0);
        this->driven->diffusionView(0)(0, 0) = 0.5 + observerValue;
        this->driven->diffusionView(1)(0, 0) = -0.25 + 0.5 * observerValue;
        if (this->mutateUnboundState) {
            this->observer->stateView()(0, 0) += 7.0;
        }
    }

    StateData* driven = nullptr;
    StateData* observer = nullptr;
    bool mutateUnboundState = false;
};

std::shared_ptr<PrescribedGaussianNoiseGenerator>
prescribed(size_t noiseCount, size_t stepCount)
{
    auto generator = std::make_shared<PrescribedGaussianNoiseGenerator>();
    for (size_t step = 0; step < stepCount; ++step) {
        std::vector<double> dW(noiseCount);
        std::vector<double> dZ(noiseCount);
        for (size_t index = 0; index < noiseCount; ++index) {
            dW[index] = (0.125 + 0.0625 * static_cast<double>(step)) * (index == 0 ? 1.0 : -1.0);
            dZ[index] = (-0.375 + 0.03125 * static_cast<double>(step)) * (index == 0 ? 1.0 : -1.0);
        }
        generator->pushStep(dW, dZ);
    }
    return generator;
}

template<typename Integrator>
void
expectTopologyCasesAndFixedSeedBits()
{
    for (size_t noiseCount = 0; noiseCount <= 2; ++noiseCount) {
        TableauDynamics dynamics(noiseCount);
        Integrator integrator(&dynamics);
        integrator.setNoiseGenerator(prescribed(noiseCount, 1));
        EXPECT_NO_THROW(stepIntegrator(integrator, 0.75, 0.125));
        EXPECT_TRUE(std::isfinite(dynamics.alpha->stateView()(0, 0)));
        EXPECT_TRUE(std::isfinite(dynamics.zeta->stateView()(0, 0)));
    }

    TableauDynamics firstDynamics(2);
    TableauDynamics secondDynamics(2);
    Integrator first(&firstDynamics);
    Integrator second(&secondDynamics);
    first.setRNGSeed(31847);
    second.setRNGSeed(31847);
    for (size_t step = 0; step < 8; ++step) {
        const double time = 0.0625 * static_cast<double>(step);
        stepIntegrator(first, time, 0.0625);
        stepIntegrator(second, time, 0.0625);
        EXPECT_EQ(doubleBits(firstDynamics.alpha->stateView()(0, 0)),
                  doubleBits(secondDynamics.alpha->stateView()(0, 0)));
        EXPECT_EQ(doubleBits(firstDynamics.zeta->stateView()(0, 0)),
                  doubleBits(secondDynamics.zeta->stateView()(0, 0)));
    }
}

template<typename Integrator>
void
expectZeroDurationAndRollback()
{
    TableauDynamics dynamics(2);
    Integrator integrator(&dynamics);
    auto generator = prescribed(2, 2);
    integrator.setNoiseGenerator(generator);

    stepIntegrator(integrator, 0.0, 0.0);
    EXPECT_EQ(generator->remaining(), 2U);
    EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(0.75));
    EXPECT_EQ(doubleBits(dynamics.zeta->stateView()(0, 0)), doubleBits(-1.25));

    dynamics.throwDuringDiffusion = true;
    EXPECT_THROW(stepIntegrator(integrator, 0.0, 0.125), std::runtime_error);
    EXPECT_EQ(generator->remaining(), 1U);
    EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(0.75));
    EXPECT_EQ(doubleBits(dynamics.zeta->stateView()(0, 0)), doubleBits(-1.25));

    dynamics.throwDuringDiffusion = false;
    EXPECT_NO_THROW(stepIntegrator(integrator, 0.0, 0.125));
    EXPECT_EQ(generator->remaining(), 0U);
}

template<typename Integrator>
void
expectNoAllocationsAfterBind()
{
    TableauDynamics dynamics(2);
    Integrator integrator(&dynamics);
    integrator.setRNGSeed(90210);
    stepIntegrator(integrator, 0.0, 0.0);

    const auto observed =
      integrator_allocation_test::trackAllocations([&]() { stepIntegrator(integrator, 0.0, 0.125); });
    EXPECT_EQ(observed.allocationCalls(), 0U);
}

template<typename Integrator>
void
expectDiffusionSideEffectsAreIsolated()
{
    SideEffectDiffusionDynamics reference(false);
    SideEffectDiffusionDynamics sideEffecting(true);
    Integrator referenceIntegrator(&reference);
    Integrator sideEffectingIntegrator(&sideEffecting);
    referenceIntegrator.setNoiseGenerator(prescribed(2, 1));
    sideEffectingIntegrator.setNoiseGenerator(prescribed(2, 1));

    stepIntegrator(referenceIntegrator, 0.75, 0.125);
    stepIntegrator(sideEffectingIntegrator, 0.75, 0.125);

    EXPECT_EQ(doubleBits(sideEffecting.driven->stateView()(0, 0)), doubleBits(reference.driven->stateView()(0, 0)));
    EXPECT_EQ(doubleBits(sideEffecting.observer->stateView()(0, 0)), doubleBits(reference.observer->stateView()(0, 0)));
}

template<template<typename> class Check>
void
forEachTableauIntegrator()
{
    Check<svStochasticIntegratorRS1>()();
    Check<svStochasticIntegratorRS2>()();
    Check<svStochasticIntegratorSIEA>()();
    Check<svStochasticIntegratorSMEA>()();
    Check<svStochasticIntegratorSIEB>()();
    Check<svStochasticIntegratorSMEB>()();
    Check<svStochasticIntegratorSOSRA>()();
    Check<svStochasticIntegratorSOSRI>()();
    Check<svStochasticIntegratorSRA1>()();
    Check<svStochasticIntegratorSRIW1>()();
}

template<typename Integrator>
struct TopologyCheck
{
    void operator()() const { expectTopologyCasesAndFixedSeedBits<Integrator>(); }
};

template<typename Integrator>
struct LifecycleCheck
{
    void operator()() const { expectZeroDurationAndRollback<Integrator>(); }
};

template<typename Integrator>
struct AllocationCheck
{
    void operator()() const { expectNoAllocationsAfterBind<Integrator>(); }
};
}

TEST(FlatStochasticTableaux, FinalizedTopologyAndFixedSeedBits)
{
    forEachTableauIntegrator<TopologyCheck>();
}

TEST(FlatStochasticTableaux, ZeroDurationAndRollbackWithoutRngRewind)
{
    forEachTableauIntegrator<LifecycleCheck>();
}

TEST(FlatStochasticTableaux, NoAllocationsAfterBind)
{
    forEachTableauIntegrator<AllocationCheck>();
}

TEST(FlatStochasticTableaux, ProductionSRIIsolatesDiffusionSideEffects)
{
    expectDiffusionSideEffectsAreIsolated<svStochasticIntegratorSOSRI>();
    expectDiffusionSideEffectsAreIsolated<svStochasticIntegratorSRIW1>();
}

TEST(FlatStochasticTableaux, GenericTableauxRejectMalformedCoefficients)
{
    TableauDynamics dynamics(1);

    SRACoefficients<2> sra;
    sra.A0[0][0] = 0.25;
    EXPECT_THROW((svIntegratorStrongStochasticRungeKuttaSRA<2>(&dynamics, sra)), std::invalid_argument);

    sra = {};
    sra.beta2[1] = std::numeric_limits<double>::infinity();
    EXPECT_THROW((svIntegratorStrongStochasticRungeKuttaSRA<2>(&dynamics, sra)), std::invalid_argument);

    SRICoefficients<2> sri;
    sri.B1[0][1] = -0.5;
    EXPECT_THROW((svIntegratorStrongStochasticRungeKuttaSRI<2>(&dynamics, sri)), std::invalid_argument);

    sri = {};
    sri.c1[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW((svIntegratorStrongStochasticRungeKuttaSRI<2>(&dynamics, sri)), std::invalid_argument);
}
