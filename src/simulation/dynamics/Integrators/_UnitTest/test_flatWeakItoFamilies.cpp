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
#include "simulation/dynamics/Integrators/svStochasticIntegratorDRI1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorW2Ito1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorW2Ito2.h"

namespace {
using integrator_test::stepIntegrator;

uint64_t
doubleBits(double value)
{
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

class WeakItoDynamics final : public DynamicObject
{
  public:
    explicit WeakItoDynamics(size_t noiseCount, bool mixedTopology = false)
      : mixedTopology(mixedTopology)
    {
        StateSpec spec;
        spec.state = { 1, 1 };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        spec.noiseCount = noiseCount;
        this->first = this->dynManager.registerState("first", spec);
        if (mixedTopology) {
            spec.noiseCount = 1;
        }
        this->second = this->dynManager.registerState("second", spec);
        if (mixedTopology) {
            this->dynManager.registerSharedNoiseSource({ { *this->first, 1 }, { *this->second, 0 } });
        } else {
            for (size_t noiseIndex = 0; noiseIndex < noiseCount; ++noiseIndex) {
                this->dynManager.registerSharedNoiseSource(
                  { { *this->first, noiseIndex }, { *this->second, noiseIndex } });
            }
        }
        this->dynManager.finalizeStates();
        this->reset();
    }

    void reset()
    {
        this->first->stateView()(0, 0) = 0.5;
        this->second->stateView()(0, 0) = 1.0;
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override
    {
        const double x0 = this->first->stateView()(0, 0);
        const double x1 = this->second->stateView()(0, 0);
        this->first->derivativeView()(0, 0) = -0.5 * x0 + 0.2 * x1;
        this->second->derivativeView()(0, 0) = 0.1 * x0 - 0.4 * x1;
    }

    void equationsOfMotionDiffusion(double, double) override
    {
        if (this->throwDuringDiffusion) {
            throw std::runtime_error("deliberate post-sample failure");
        }
        const double x0 = this->first->stateView()(0, 0);
        const double x1 = this->second->stateView()(0, 0);
        if (this->mixedTopology) {
            this->first->diffusionView(0)(0, 0) = 0.3 * x0;
            this->first->diffusionView(1)(0, 0) = 0.15 * x1;
            this->second->diffusionView(0)(0, 0) = 0.25 * x0;
            return;
        }
        if (this->first->getNumNoiseSources() > 0) {
            this->first->diffusionView(0)(0, 0) = 0.3 * x0;
            this->second->diffusionView(0)(0, 0) = 0.1 * x1;
        }
        if (this->first->getNumNoiseSources() > 1) {
            this->first->diffusionView(1)(0, 0) = 0.15 * x1;
            this->second->diffusionView(1)(0, 0) = 0.25 * x0;
        }
    }

    StateData* first = nullptr;
    StateData* second = nullptr;
    bool throwDuringDiffusion = false;
    bool mixedTopology = false;
};

class OrderedPolicy final : public StateUpdatePolicy
{
  public:
    bool topologyEquals(const StateUpdatePolicy& other) const override
    {
        return dynamic_cast<const OrderedPolicy*>(&other) != nullptr;
    }

    void validate(const StateSpec&) const override {}

    void buildDriftCandidate(ConstMatrixView base,
                             ConstMatrixView drift,
                             double timeStep,
                             MutableMatrixView output) const override
    {
        output = base;
        output += drift * timeStep;
        output(0, 0) += 0.125 * base(0, 0) * drift(0, 0);
    }

    void applyNoiseIncrement(MutableMatrixView state, ConstMatrixView diffusion, double pseudoStep) const override
    {
        state(0, 0) = state(0, 0) * (1.0 + pseudoStep) + diffusion(0, 0);
    }
};

class OrderedWeakItoDynamics final : public DynamicObject
{
  public:
    OrderedWeakItoDynamics()
    {
        StateSpec spec;
        spec.state = { 1, 1 };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        spec.noiseCount = 2;
        spec.updateKind = StateUpdateKind::Special;
        this->state = this->dynManager.registerState("ordered", spec, std::make_unique<OrderedPolicy>());
        this->state->setState(Eigen::MatrixXd::Constant(1, 1, 1.0));
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override
    {
        const double x = this->state->stateView()(0, 0);
        this->state->derivativeView()(0, 0) = 0.25 + 0.125 * x;
    }

    void equationsOfMotionDiffusion(double, double) override
    {
        const double x = this->state->stateView()(0, 0);
        this->state->diffusionView(0)(0, 0) = 2.0 + 0.5 * x;
        this->state->diffusionView(1)(0, 0) = 3.0 - 0.25 * x;
    }

    StateData* state = nullptr;
};

std::shared_ptr<PrescribedGaussianNoiseGenerator>
prescribed(std::initializer_list<double> dW, std::initializer_list<double> dZ)
{
    auto generator = std::make_shared<PrescribedGaussianNoiseGenerator>();
    generator->pushStep(std::vector<double>(dW), std::vector<double>(dZ));
    return generator;
}

template<typename Integrator>
void
expectZeroDurationAndRollback()
{
    WeakItoDynamics dynamics(2);
    Integrator integrator(&dynamics);
    auto generator = std::make_shared<PrescribedGaussianNoiseGenerator>();
    generator->pushStep({ 0.1, -0.2 }, { 0.3, -0.4 });
    generator->pushStep({ -0.2, 0.1 }, { -0.4, 0.3 });
    integrator.setNoiseGenerator(generator);

    stepIntegrator(integrator, 0.0, 0.0);
    EXPECT_EQ(generator->remaining(), 2U);

    dynamics.throwDuringDiffusion = true;
    EXPECT_THROW(stepIntegrator(integrator, 0.0, 0.125), std::runtime_error);
    EXPECT_EQ(generator->remaining(), 1U);
    EXPECT_DOUBLE_EQ(dynamics.first->stateView()(0, 0), 0.5);
    EXPECT_DOUBLE_EQ(dynamics.second->stateView()(0, 0), 1.0);

    dynamics.throwDuringDiffusion = false;
    EXPECT_NO_THROW(stepIntegrator(integrator, 0.0, 0.125));
    EXPECT_EQ(generator->remaining(), 0U);
}

template<typename Integrator>
void
expectNoAllocationAfterBind()
{
    WeakItoDynamics dynamics(2);
    Integrator integrator(&dynamics);
    integrator.setRNGSeed(23847);
    stepIntegrator(integrator, 0.0, 0.0);

    const auto observed = integrator_test::trackAllocations([&]() { stepIntegrator(integrator, 0.0, 0.125); });
    EXPECT_EQ(observed.allocationCalls(), 0U);
}

template<typename Integrator>
void
expectNoiseCountPath(size_t noiseCount)
{
    WeakItoDynamics dynamics(noiseCount);
    Integrator integrator(&dynamics);
    auto generator = std::make_shared<PrescribedGaussianNoiseGenerator>();
    generator->pushStep(std::vector<double>(noiseCount, 0.125), std::vector<double>(noiseCount, -0.25));
    integrator.setNoiseGenerator(generator);
    stepIntegrator(integrator, 0.0, 0.125);
    EXPECT_TRUE(std::isfinite(dynamics.first->stateView()(0, 0)));
    EXPECT_TRUE(std::isfinite(dynamics.second->stateView()(0, 0)));
}

template<typename Integrator>
void
expectMixedTopology(uint64_t expectedFirstBits, uint64_t expectedSecondBits)
{
    WeakItoDynamics dynamics(2, true);
    Integrator integrator(&dynamics);
    integrator.setNoiseGenerator(prescribed({ 0.5, -0.45 }, { 0.3, -0.2 }));
    stepIntegrator(integrator, 0.0, 0.125);

    EXPECT_EQ(doubleBits(dynamics.first->stateView()(0, 0)), expectedFirstBits);
    EXPECT_EQ(doubleBits(dynamics.second->stateView()(0, 0)), expectedSecondBits);
}

template<typename Integrator>
void
expectNoncommutativeSpecialPolicy(uint64_t expectedBits)
{
    OrderedWeakItoDynamics dynamics;
    Integrator integrator(&dynamics);
    integrator.setNoiseGenerator(prescribed({ 0.5, -0.45 }, { 0.3, -0.2 }));

    stepIntegrator(integrator, 0.0, 0.125);

    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), expectedBits);
}
}

TEST(FlatWeakItoFamilies, W2ItoPaperReferenceBits)
{
    {
        WeakItoDynamics dynamics(2);
        svStochasticIntegratorW2Ito1 integrator(&dynamics);
        integrator.setNoiseGenerator(
          prescribed({ 0.017225447734423038, 0.1831984663246682 }, { 0.19143508986887522, -0.1501571687945868 }));
        stepIntegrator(integrator, 0.0, 0.125);
        EXPECT_DOUBLE_EQ(dynamics.first->stateView()(0, 0), 0.4886843990979673);
        EXPECT_DOUBLE_EQ(dynamics.second->stateView()(0, 0), 0.9537920887170881);
    }
    {
        WeakItoDynamics dynamics(2);
        svStochasticIntegratorW2Ito2 integrator(&dynamics);
        integrator.setNoiseGenerator(
          prescribed({ 0.08714682860793878, 0.07301273674452739 }, { -0.37346576044084245, -0.5112353004834385 }));
        stepIntegrator(integrator, 0.0, 0.125);
        EXPECT_DOUBLE_EQ(dynamics.first->stateView()(0, 0), 0.48945470733642576);
        EXPECT_DOUBLE_EQ(dynamics.second->stateView()(0, 0), 0.9543821243286132);
    }
}

TEST(FlatWeakItoFamilies, DRI1PaperReferenceBits)
{
    WeakItoDynamics dynamics(2);
    svStochasticIntegratorDRI1 integrator(&dynamics);
    integrator.setNoiseGenerator(
      prescribed({ -0.13976011088567203, 0.09330800242534518 }, { 0.21044072177715215, -0.20696984404638655 }));
    stepIntegrator(integrator, 0.0, 0.125);
    EXPECT_DOUBLE_EQ(dynamics.first->stateView()(0, 0), 0.4875825195312499);
    EXPECT_DOUBLE_EQ(dynamics.second->stateView()(0, 0), 0.9558728841145833);
}

TEST(FlatWeakItoFamilies, DRI1ModeIsFrozenAtBinding)
{
    WeakItoDynamics dynamics(2);
    svStochasticIntegratorDRI1 integrator(&dynamics);
    integrator.setNonMixing(true);
    stepIntegrator(integrator, 0.0, 0.0);
    EXPECT_TRUE(integrator.getNonMixing());
    EXPECT_THROW(integrator.setNonMixing(false), std::logic_error);
    EXPECT_TRUE(integrator.getNonMixing());
}

TEST(FlatWeakItoFamilies, RIFamilyPaperReferenceBits)
{
    {
        WeakItoDynamics dynamics(2);
        svStochasticIntegratorRI1 integrator(&dynamics);
        integrator.setNoiseGenerator(
          prescribed({ 0.3214313724988835, 0.26565882991430884 }, { -0.07854918074676295, 0.06465323920073543 }));
        stepIntegrator(integrator, 0.0, 0.125);
        EXPECT_DOUBLE_EQ(dynamics.first->stateView()(0, 0), 0.49130517578125005);
        EXPECT_DOUBLE_EQ(dynamics.second->stateView()(0, 0), 0.9527543945312499);
    }
    {
        WeakItoDynamics dynamics(2);
        svStochasticIntegratorRI3 integrator(&dynamics);
        integrator.setNoiseGenerator(
          prescribed({ 0.14084505263670893, -0.19898809924898814 }, { -0.4066940192839459, 0.059843829693812356 }));
        stepIntegrator(integrator, 0.0, 0.125);
        EXPECT_DOUBLE_EQ(dynamics.first->stateView()(0, 0), 0.49130517578125005);
        EXPECT_DOUBLE_EQ(dynamics.second->stateView()(0, 0), 0.9527543945312499);
    }
    {
        WeakItoDynamics dynamics(2);
        svStochasticIntegratorRI5 integrator(&dynamics);
        integrator.setNoiseGenerator(
          prescribed({ -0.014115498539039673, -0.44467708215560936 }, { 0.09780314160098916, -0.12406419972642377 }));
        stepIntegrator(integrator, 0.0, 0.125);
        EXPECT_DOUBLE_EQ(dynamics.first->stateView()(0, 0), 0.5789940869001351);
        EXPECT_DOUBLE_EQ(dynamics.second->stateView()(0, 0), 1.0376344341650983);
    }
    {
        WeakItoDynamics dynamics(2);
        svStochasticIntegratorRI6 integrator(&dynamics);
        integrator.setNoiseGenerator(
          prescribed({ -0.4024065724993792, 0.2726672203670614 }, { -0.2912841140397067, 0.22757946014224292 }));
        stepIntegrator(integrator, 0.0, 0.125);
        EXPECT_DOUBLE_EQ(dynamics.first->stateView()(0, 0), 0.41054982655179995);
        EXPECT_DOUBLE_EQ(dynamics.second->stateView()(0, 0), 0.8957075905642796);
    }
}

TEST(FlatWeakItoFamilies, AllConcreteVariantsHandleZeroAndScalarNoise)
{
    expectNoiseCountPath<svStochasticIntegratorW2Ito1>(0);
    expectNoiseCountPath<svStochasticIntegratorW2Ito1>(1);
    expectNoiseCountPath<svStochasticIntegratorW2Ito2>(0);
    expectNoiseCountPath<svStochasticIntegratorW2Ito2>(1);
    expectNoiseCountPath<svStochasticIntegratorDRI1>(0);
    expectNoiseCountPath<svStochasticIntegratorDRI1>(1);
    expectNoiseCountPath<svStochasticIntegratorDRI1NM>(0);
    expectNoiseCountPath<svStochasticIntegratorDRI1NM>(1);
    expectNoiseCountPath<svStochasticIntegratorRI1>(0);
    expectNoiseCountPath<svStochasticIntegratorRI1>(1);
    expectNoiseCountPath<svStochasticIntegratorRI3>(0);
    expectNoiseCountPath<svStochasticIntegratorRI3>(1);
    expectNoiseCountPath<svStochasticIntegratorRI5>(0);
    expectNoiseCountPath<svStochasticIntegratorRI5>(1);
    expectNoiseCountPath<svStochasticIntegratorRI6>(0);
    expectNoiseCountPath<svStochasticIntegratorRI6>(1);
}

TEST(FlatWeakItoFamilies, MixedSharedAndUnsharedNoiseTopology)
{
    expectMixedTopology<svStochasticIntegratorW2Ito1>(UINT64_C(0x3fdf22e09f4a6057), UINT64_C(0x3fec6b1086efba3f));
    expectMixedTopology<svStochasticIntegratorW2Ito2>(UINT64_C(0x3fdf0f3650431c31), UINT64_C(0x3fec67da8284e8a4));
    expectMixedTopology<svStochasticIntegratorDRI1>(UINT64_C(0x3fdf6949ae960c2e), UINT64_C(0x3fec428fd341a349));
    expectMixedTopology<svStochasticIntegratorDRI1NM>(UINT64_C(0x3fe00f75bcab47a0), UINT64_C(0x3fec68cd43e57a52));
    expectMixedTopology<svStochasticIntegratorRI1>(UINT64_C(0x3fdf67106e542e0a), UINT64_C(0x3fec422b6d3db529));
    expectMixedTopology<svStochasticIntegratorRI3>(UINT64_C(0x3fdf65f5872a2ab8), UINT64_C(0x3fec42b8e0d2b6d2));
    expectMixedTopology<svStochasticIntegratorRI5>(UINT64_C(0x3fdf56b39b8891f1), UINT64_C(0x3ff080bad0214133));
    expectMixedTopology<svStochasticIntegratorRI6>(UINT64_C(0x3fdf65eb1a1c7e86), UINT64_C(0x3fec42c43c36bc05));
}

TEST(FlatWeakItoFamilies, NoncommutativeSpecialPolicyPreservesLocalNoiseOrder)
{
    expectNoncommutativeSpecialPolicy<svStochasticIntegratorW2Ito1>(UINT64_C(0x401e9f0a64cc9653));
    expectNoncommutativeSpecialPolicy<svStochasticIntegratorW2Ito2>(UINT64_C(0x401cee91377cd13f));
    expectNoncommutativeSpecialPolicy<svStochasticIntegratorDRI1>(UINT64_C(0x402e86e6cdf0d484));
    expectNoncommutativeSpecialPolicy<svStochasticIntegratorDRI1NM>(UINT64_C(0x4021f973804c2951));
}

TEST(FlatWeakItoFamilies, ZeroDurationAndRollbackDoNotRewindNoise)
{
    expectZeroDurationAndRollback<svStochasticIntegratorW2Ito1>();
    expectZeroDurationAndRollback<svStochasticIntegratorW2Ito2>();
    expectZeroDurationAndRollback<svStochasticIntegratorDRI1>();
    expectZeroDurationAndRollback<svStochasticIntegratorDRI1NM>();
    expectZeroDurationAndRollback<svStochasticIntegratorRI1>();
    expectZeroDurationAndRollback<svStochasticIntegratorRI3>();
    expectZeroDurationAndRollback<svStochasticIntegratorRI5>();
    expectZeroDurationAndRollback<svStochasticIntegratorRI6>();
}

TEST(FlatWeakItoFamilies, IntegrationAllocatesNothingAfterBind)
{
    expectNoAllocationAfterBind<svStochasticIntegratorW2Ito1>();
    expectNoAllocationAfterBind<svStochasticIntegratorW2Ito2>();
    expectNoAllocationAfterBind<svStochasticIntegratorDRI1>();
    expectNoAllocationAfterBind<svStochasticIntegratorDRI1NM>();
    expectNoAllocationAfterBind<svStochasticIntegratorRI1>();
    expectNoAllocationAfterBind<svStochasticIntegratorRI3>();
    expectNoAllocationAfterBind<svStochasticIntegratorRI5>();
    expectNoAllocationAfterBind<svStochasticIntegratorRI6>();
}
