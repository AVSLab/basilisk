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
#include <memory>
#include <stdexcept>
#include <vector>

#include "allocationTracker.h"
#include "integratorTestAccess.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorEulerHeun.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorMayurama.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorRDI1WM.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorRKMil.h"
#include "simulation/dynamics/_GeneralModuleFiles/stochasticWeakRandomVariables.h"

namespace {
using integrator_step_test::stepIntegrator;

uint64_t
doubleBits(double value)
{
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

class BasicStochasticDynamics final : public DynamicObject
{
  public:
    explicit BasicStochasticDynamics(size_t globalNoiseCount = 2, bool reverseRegistration = false)
    {
        StateSpec alphaSpec;
        alphaSpec.state = { 1, 1 };
        alphaSpec.derivative = alphaSpec.state;
        alphaSpec.diffusionTangent = alphaSpec.state;
        alphaSpec.noiseCount = globalNoiseCount;
        StateSpec zetaSpec = alphaSpec;
        zetaSpec.noiseCount = globalNoiseCount == 0 ? 0 : 1;
        if (reverseRegistration) {
            this->zeta = this->dynManager.registerState("zeta", zetaSpec);
            this->alpha = this->dynManager.registerState("alpha", alphaSpec);
        } else {
            this->alpha = this->dynManager.registerState("alpha", alphaSpec);
            this->zeta = this->dynManager.registerState("zeta", zetaSpec);
        }
        if (globalNoiseCount > 0) {
            this->dynManager.registerSharedNoiseSource({ { *this->alpha, globalNoiseCount - 1 }, { *this->zeta, 0 } });
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
        if (this->alpha->getNumNoiseSources() == 0) {
            return;
        }
        const double alphaValue = this->alpha->stateView()(0, 0);
        const double zetaValue = this->zeta->stateView()(0, 0);
        this->alpha->diffusionView(0)(0, 0) = 0.5 + 0.125 * alphaValue;
        if (this->alpha->getNumNoiseSources() > 1) {
            this->alpha->diffusionView(1)(0, 0) = -0.25 * zetaValue + 0.0625 * time;
        }
        this->zeta->diffusionView(0)(0, 0) = 0.75 + 0.2 * alphaValue - 0.1 * zetaValue;
    }

    StateData* alpha = nullptr;
    StateData* zeta = nullptr;
    bool throwDuringDiffusion = false;
};

template<typename FlatIntegrator>
void
expectZeroDurationAndRollback()
{
    BasicStochasticDynamics dynamics;
    FlatIntegrator integrator(&dynamics);
    auto generator = std::make_shared<PrescribedGaussianNoiseGenerator>();
    generator->pushStep({ 0.125, -0.25 });
    generator->pushStep({ 0.5, 0.75 });
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

template<typename FlatIntegrator>
void
expectNoAllocationsAfterBind()
{
    BasicStochasticDynamics dynamics;
    FlatIntegrator integrator(&dynamics);
    integrator.setRNGSeed(90210);
    stepIntegrator(integrator, 0.0, 0.0);

    const auto observed =
      integrator_allocation_test::trackAllocations([&]() { stepIntegrator(integrator, 0.0, 0.125); });
    EXPECT_EQ(observed.allocationCalls(), 0U);
}

template<typename FlatIntegrator>
void
expectRegistrationOrderInvariant()
{
    BasicStochasticDynamics forward(2, false);
    BasicStochasticDynamics reverse(2, true);
    FlatIntegrator forwardIntegrator(&forward);
    FlatIntegrator reverseIntegrator(&reverse);
    auto forwardGenerator = std::make_shared<PrescribedGaussianNoiseGenerator>();
    auto reverseGenerator = std::make_shared<PrescribedGaussianNoiseGenerator>();
    forwardGenerator->pushStep({ 0.125, -0.25 }, { -0.375, 0.5 });
    reverseGenerator->pushStep({ 0.125, -0.25 }, { -0.375, 0.5 });
    forwardIntegrator.setNoiseGenerator(forwardGenerator);
    reverseIntegrator.setNoiseGenerator(reverseGenerator);

    stepIntegrator(forwardIntegrator, 0.75, 0.125);
    stepIntegrator(reverseIntegrator, 0.75, 0.125);

    EXPECT_EQ(doubleBits(forward.alpha->stateView()(0, 0)), doubleBits(reverse.alpha->stateView()(0, 0)));
    EXPECT_EQ(doubleBits(forward.zeta->stateView()(0, 0)), doubleBits(reverse.zeta->stateView()(0, 0)));
}
}

TEST(FlatStochasticBasic, ZeroDurationDoesNotSample)
{
    expectZeroDurationAndRollback<svStochasticIntegratorEulerHeun>();
    expectZeroDurationAndRollback<svStochasticIntegratorMayurama>();
    expectZeroDurationAndRollback<svStochasticIntegratorRKMil>();
    expectZeroDurationAndRollback<svStochasticIntegratorRDI1WM>();
}

TEST(FlatStochasticBasic, BookkeepingDoesNotAllocateAfterBind)
{
    expectNoAllocationsAfterBind<svStochasticIntegratorEulerHeun>();
    expectNoAllocationsAfterBind<svStochasticIntegratorMayurama>();
    expectNoAllocationsAfterBind<svStochasticIntegratorRKMil>();
    expectNoAllocationsAfterBind<svStochasticIntegratorRDI1WM>();
}

TEST(FlatStochasticBasic, CanonicalNoiseOffsetsIgnoreRegistrationOrder)
{
    expectRegistrationOrderInvariant<svStochasticIntegratorEulerHeun>();
    expectRegistrationOrderInvariant<svStochasticIntegratorRKMil>();
    expectRegistrationOrderInvariant<svStochasticIntegratorRDI1WM>();
}
