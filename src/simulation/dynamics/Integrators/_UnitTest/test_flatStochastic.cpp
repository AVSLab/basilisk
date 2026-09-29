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

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
#include "simulation/dynamics/_GeneralModuleFiles/stochasticRKIntegratorBase.h"

namespace {
using integrator_test::stepIntegrator;

uint64_t
doubleBits(double value)
{
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

class TopologyDynamics final : public DynamicObject
{
  public:
    explicit TopologyDynamics(bool finalized)
    {
        if (finalized) {
            StateSpec spec;
            spec.state = { 1, 1 };
            spec.derivative = spec.state;
            spec.diffusionTangent = spec.state;
            spec.noiseCount = 2;
            this->zeta = this->dynManager.registerState("zeta", spec);
            this->alpha = this->dynManager.registerState("alpha", spec);
        } else {
            this->zeta = this->dynManager.registerState(1, 1, "zeta");
            this->alpha = this->dynManager.registerState(1, 1, "alpha");
        }

        Eigen::MatrixXd initial(1, 1);
        initial(0, 0) = 0.0;
        this->zeta->setState(initial);
        this->alpha->setState(initial);
        if (finalized) {
            this->dynManager.registerSharedNoiseSource({ { *this->zeta, 1 }, { *this->alpha, 0 } });
            this->dynManager.finalizeStates();
        }
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override
    {
        this->alpha->derivativeView()(0, 0) = 0.25;
        this->zeta->derivativeView()(0, 0) = -0.5;
    }

    void equationsOfMotionDiffusion(double, double) override
    {
        if (this->throwDuringDiffusion) {
            throw std::runtime_error("deliberate post-sample diffusion failure");
        }
        this->alpha->diffusionView(0)(0, 0) = 1.0;
        this->alpha->diffusionView(1)(0, 0) = 10.0;
        this->zeta->diffusionView(0)(0, 0) = 100.0;
        this->zeta->diffusionView(1)(0, 0) = 1000.0;
        if (this->overwriteDriftDuringDiffusion) {
            this->alpha->derivativeView()(0, 0) = 10000.0;
            this->zeta->derivativeView()(0, 0) = -10000.0;
        }
    }

    StateData* alpha = nullptr;
    StateData* zeta = nullptr;
    bool throwDuringDiffusion = false;
    bool overwriteDriftDuringDiffusion = false;
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
        output = base + drift * timeStep;
    }

    void applyNoiseIncrement(MutableMatrixView state, ConstMatrixView diffusion, double pseudoStep) const override
    {
        state(0, 0) = state(0, 0) * (1.0 + pseudoStep) + diffusion(0, 0);
    }
};

class OrderedDynamics final : public DynamicObject
{
  public:
    explicit OrderedDynamics(bool shareReversedLocalSource = false, bool extraSlot = false, bool euclidean = false)
    {
        StateSpec spec;
        spec.state = { 1, 1 };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        spec.noiseCount = 2;
        spec.updateKind = euclidean ? StateUpdateKind::Euclidean : StateUpdateKind::Special;
        this->state = euclidean ? this->dynManager.registerState("qposProxy", spec)
                                : this->dynManager.registerState("qposProxy", spec, std::make_unique<OrderedPolicy>());
        Eigen::MatrixXd initial(1, 1);
        initial(0, 0) = 1.0;
        this->state->setState(initial);
        if (shareReversedLocalSource) {
            spec.noiseCount = 1;
            spec.updateKind = StateUpdateKind::Euclidean;
            this->alpha = this->dynManager.registerState("alpha", spec);
            this->dynManager.registerSharedNoiseSource({ { *this->alpha, 0 }, { *this->state, 1 } });
        }
        if (extraSlot) {
            spec.noiseCount = 1;
            spec.updateKind = StateUpdateKind::Euclidean;
            this->dynManager.registerState("zzUnused", spec);
        }
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override { this->state->derivativeView()(0, 0) = 0.0; }

    void equationsOfMotionDiffusion(double, double) override
    {
        this->state->diffusionView(0)(0, 0) = 2.0;
        this->state->diffusionView(1)(0, 0) = 3.0;
        if (this->alpha) {
            this->alpha->diffusionView(0)(0, 0) = 1.0;
        }
    }

    StateData* state = nullptr;
    StateData* alpha = nullptr;
};

class CandidateOrderProbe final : public StateVecStochasticIntegrator
{
  public:
    using StateVecStochasticIntegrator::StateVecStochasticIntegrator;

    void apply(const std::string& path,
               const Eigen::VectorXd& pseudoSteps,
               size_t slotBegin,
               size_t slotEnd,
               Eigen::Index diffusionSizeAdjustment = 0)
    {
        this->bindStochasticTopology();
        this->gatherStochasticStates();
        this->gatherStochasticDerivatives();
        this->gatherStochasticDiffusions();
        const auto& base = this->stochasticAcceptedState();
        const auto& drift = this->stochasticPackedDerivatives();
        Eigen::VectorXd diffusion = this->stochasticPackedDiffusions();
        diffusion.conservativeResize(diffusion.size() + diffusionSizeAdjustment);
        const double timeStep = 1.0; // [s]
        if (path == "buffered") {
            this->buildStochasticCandidate(base, drift, timeStep, diffusion, pseudoSteps, slotBegin, slotEnd);
        } else if (path == "inPlace") {
            this->buildStochasticCandidateInPlace(base, drift, timeStep, diffusion, pseudoSteps, slotBegin, slotEnd);
        } else if (path == "diffusion") {
            this->applyStochasticDiffusionUpdateInPlace(diffusion, pseudoSteps);
        } else if (path == "euclideanFinal") {
            const Eigen::VectorXd zeroSteps = Eigen::VectorXd::Zero(pseudoSteps.size());
            this->beginAllEuclideanFinalCandidate(base, drift, timeStep, diffusion, zeroSteps);
            this->appendAllEuclideanFinalCandidate(drift, timeStep, diffusion, pseudoSteps);
            this->commitAllEuclideanFinalCandidate();
        } else {
            throw std::invalid_argument("Unknown candidate test path.");
        }
    }

  protected:
    void prepareIntegrationBinding() override { this->bindStochasticTopology(); }
    void validateIntegrationBinding() const override { this->validateStochasticTopology(); }
    void integrateImpl(double, double) override {}
};

class NoiseCallbackPolicy final : public StateUpdatePolicy
{
  public:
    explicit NoiseCallbackPolicy(std::function<void()>* callback)
      : callback(callback)
    {
    }

    bool topologyEquals(const StateUpdatePolicy& other) const override
    {
        return dynamic_cast<const NoiseCallbackPolicy*>(&other) != nullptr;
    }

    void validate(const StateSpec&) const override {}

    void buildDriftCandidate(ConstMatrixView base,
                             ConstMatrixView drift,
                             double timeStep,
                             MutableMatrixView output) const override
    {
        output = base + drift * timeStep;
    }

    void applyNoiseIncrement(MutableMatrixView state, ConstMatrixView diffusion, double pseudoStep) const override
    {
        state += diffusion * pseudoStep;
        if (*this->callback) {
            (*this->callback)();
        }
    }

  private:
    std::function<void()>* callback;
};

class NoisePolicyDynamics final : public DynamicObject
{
  public:
    explicit NoisePolicyDynamics(std::function<void()>* callback)
      : callback(callback)
    {
    }

    void UpdateState(uint64_t) override {}

    void reset()
    {
        StateSpec spec;
        spec.state = { 1, 1 };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        spec.noiseCount = 1;
        spec.updateKind = StateUpdateKind::Special;
        this->state = this->dynManager.registerState(
          "noiseCallbackState", spec, std::make_unique<NoiseCallbackPolicy>(this->callback));
        this->state->stateView()(0, 0) = 0.0;
        this->dynManager.finalizeStates();
    }

    void preIntegration(uint64_t callTimeNanos) override
    {
        this->timeStep = static_cast<double>(callTimeNanos - this->timeBeforeNanos) * 1.0e-9;
    }

    void equationsOfMotion(double, double) override { this->state->derivativeView()(0, 0) = 0.0; }

    void equationsOfMotionDiffusion(double, double) override { this->state->diffusionView(0)(0, 0) = 1.0; }

    void postIntegration(uint64_t callTimeNanos) override
    {
        this->timeBefore = static_cast<double>(callTimeNanos) * 1.0e-9;
        this->timeBeforeNanos = callTimeNanos;
    }

  private:
    std::function<void()>* callback;
    StateData* state = nullptr;
};

class SparseEquivalenceDynamics final : public DynamicObject
{
  public:
    SparseEquivalenceDynamics()
    {
        StateSpec scalarSpec;
        scalarSpec.state = { 1, 1 };
        scalarSpec.derivative = scalarSpec.state;
        scalarSpec.diffusionTangent = scalarSpec.state;
        scalarSpec.noiseCount = 2;
        this->alpha = this->dynManager.registerState("alpha", scalarSpec);

        StateSpec specialSpec = scalarSpec;
        specialSpec.updateKind = StateUpdateKind::Special;
        this->beta = this->dynManager.registerState("beta", specialSpec, std::make_unique<OrderedPolicy>());

        StateSpec fillerSpec;
        fillerSpec.state = { 32, 1 };
        fillerSpec.derivative = fillerSpec.state;
        fillerSpec.diffusionTangent = fillerSpec.state;
        this->filler = this->dynManager.registerState("filler", fillerSpec);

        this->alpha->stateView()(0, 0) = 0.75;
        this->beta->stateView()(0, 0) = -0.5;
        this->filler->stateView().setZero();
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override
    {
        this->alpha->derivativeView()(0, 0) = 0.25;
        this->beta->derivativeView()(0, 0) = -0.125;
        this->filler->derivativeView().setZero();
    }

    void equationsOfMotionDiffusion(double, double) override
    {
        const double alphaValue = this->alpha->stateView()(0, 0);
        const double betaValue = this->beta->stateView()(0, 0);
        this->alpha->diffusionView(0)(0, 0) = 1.0 + alphaValue - 0.5 * betaValue;
        this->alpha->diffusionView(1)(0, 0) = -0.25 + 0.75 * alphaValue;
        this->beta->diffusionView(0)(0, 0) = 0.5 + betaValue;
        this->beta->diffusionView(1)(0, 0) = 2.0 - alphaValue + betaValue;

        this->observed.at(this->observationCount++) = alphaValue + 3.0 * betaValue + this->filler->stateView()(0, 0);
        this->filler->stateView()(0, 0) += 7.0;
    }

    StateData* alpha = nullptr;
    StateData* beta = nullptr;
    StateData* filler = nullptr;
    std::array<double, 5> observed{};
    size_t observationCount = 0;
};

class SparseSequenceIntegrator final : public StochasticRKIntegratorBase
{
  public:
    SparseSequenceIntegrator(DynamicObject* dynamics, bool sparse)
      : StochasticRKIntegratorBase(dynamics)
      , useSparsePath(sparse)
    {
    }

    void integrateImpl(double currentTime, double timeStep) override
    {
        this->bindFlatStochasticStorage([this] {
            const auto noiseCount = static_cast<Eigen::Index>(this->globalNoiseCount());
            this->pseudoSteps.resize(noiseCount);
            this->initialDiffusions.resize(this->stochasticPackedDiffusions().size());
        });
        this->gatherStochasticStates();
        for (DynamicObject* object : this->dynamics()) {
            object->equationsOfMotion(currentTime, timeStep);
        }
        this->gatherStochasticDerivatives();
        for (DynamicObject* object : this->dynamics()) {
            object->equationsOfMotionDiffusion(currentTime, timeStep);
        }
        this->gatherStochasticDiffusions(this->initialDiffusions);

        if (this->useSparsePath) {
            this->beginStochasticSparseCandidates(
              this->stochasticAcceptedState(), this->stochasticPackedDerivatives(), timeStep);
        }
        for (size_t slot = 0; slot < this->globalNoiseCount(); ++slot) {
            const double pseudoStep = 0.125 * static_cast<double>(slot + 1);
            if (this->useSparsePath) {
                this->buildStochasticNoiseSlotCandidate(this->initialDiffusions, pseudoStep, slot);
            } else {
                this->pseudoSteps.setZero();
                this->pseudoSteps(static_cast<Eigen::Index>(slot)) = pseudoStep;
                this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                               this->stochasticPackedDerivatives(),
                                               timeStep,
                                               this->initialDiffusions,
                                               this->pseudoSteps,
                                               slot,
                                               slot + 1);
            }
            for (DynamicObject* object : this->dynamics()) {
                object->equationsOfMotionDiffusion(currentTime, timeStep);
            }
        }
    }

  private:
    bool useSparsePath = false;
    Eigen::VectorXd pseudoSteps;
    Eigen::VectorXd initialDiffusions;
};

class BindingInvariantProbe final : public StochasticRKIntegratorBase
{
  public:
    using StochasticRKIntegratorBase::StochasticRKIntegratorBase;

    void integrateImpl(double, double) override {}

    void bind() { this->bindFlatStochasticStorage(); }

    void gatherIntoWrongSizedDiffusionBuffer()
    {
        Eigen::VectorXd output(this->stochasticPackedDiffusions().size() + 1);
        this->gatherStochasticDiffusions(output);
    }

    void buildSparseCandidateWithoutBegin()
    {
        this->buildStochasticNoiseSlotCandidate(this->stochasticPackedDiffusions(), 1.0, 0);
    }

    void appendFinalCandidateWithoutBegin()
    {
        Eigen::VectorXd pseudoSteps(static_cast<Eigen::Index>(this->globalNoiseCount()));
        pseudoSteps.setZero();
        this->appendAllEuclideanFinalCandidate(
          this->stochasticPackedDerivatives(), 1.0, this->stochasticPackedDiffusions(), pseudoSteps);
    }
};







class LegacyCustomNoiseGenerator final : public GaussianNoiseGenerator
{
  public:
    void setSeed(size_t) override {}

    GaussianNoiseSample generate(size_t m, double) override
    {
        ++this->fullCalls;
        GaussianNoiseSample sample;
        sample.dW.resize(static_cast<Eigen::Index>(m));
        sample.dZ.resize(static_cast<Eigen::Index>(m));
        sample.dW.setConstant(0.25);
        sample.dZ.setConstant(9.0);
        return sample;
    }

    size_t fullCalls = 0;
};

class InvalidLegacyNoiseGenerator final : public GaussianNoiseGenerator
{
  public:
    void setSeed(size_t) override {}

    GaussianNoiseSample generate(size_t m, double) override
    {
        GaussianNoiseSample sample;
        sample.dW.resize(static_cast<Eigen::Index>(m == 0 ? 0 : m - 1));
        sample.dZ.resize(static_cast<Eigen::Index>(m));
        return sample;
    }
};

class AuxiliaryCountNoiseGenerator final : public GaussianNoiseGenerator
{
  public:
    void setSeed(size_t) override {}

    GaussianNoiseSample generate(size_t m, double) override
    {
        GaussianNoiseSample sample;
        sample.dW = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(m));
        sample.dZ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(m));
        return sample;
    }

    void generateWithAuxiliaryInto(Eigen::VectorXd& dW,
                                   Eigen::VectorXd& dZ,
                                   size_t m,
                                   size_t auxiliaryCount,
                                   double) override
    {
        requireAuxiliaryOutputSize(dW, dZ, m, auxiliaryCount);
        ++this->calls;
        this->lastWienerCount = m;
        this->lastAuxiliaryCount = auxiliaryCount;
        dW.setZero();
        if (auxiliaryCount > 0) {
            dZ.head(static_cast<Eigen::Index>(auxiliaryCount)).setZero();
        }
    }

    size_t calls = 0;
    size_t lastWienerCount = 0;
    size_t lastAuxiliaryCount = 0;
};

class ReentrantNoiseGenerator final : public GaussianNoiseGenerator
{
  public:
    ReentrantNoiseGenerator(StochasticRKIntegratorBase& owner,
                            std::shared_ptr<GaussianNoiseGenerator> replacement,
                            bool& destroyed,
                            bool& destroyedDuringCall)
      : owner(owner)
      , replacement(std::move(replacement))
      , destroyed(destroyed)
      , destroyedDuringCall(destroyedDuringCall)
    {
    }

    ~ReentrantNoiseGenerator() override { this->destroyed = true; }

    void setSeed(size_t) override { this->replaceSelf(); }

    GaussianNoiseSample generate(size_t, double) override
    {
        throw std::logic_error("generate fallback should not be used");
    }

    void generateWienerInto(Eigen::VectorXd& dW, Eigen::VectorXd& dZScratch, size_t m, double) override
    {
        requireOutputSize(dW, dZScratch, m);
        this->replaceSelf();
        dW.setZero();
    }

    void generateWithAuxiliaryInto(Eigen::VectorXd& dW,
                                   Eigen::VectorXd& dZ,
                                   size_t m,
                                   size_t auxiliaryCount,
                                   double) override
    {
        requireAuxiliaryOutputSize(dW, dZ, m, auxiliaryCount);
        this->replaceSelf();
        dW.setZero();
        dZ.head(static_cast<Eigen::Index>(auxiliaryCount)).setZero();
    }

  private:
    void replaceSelf()
    {
        this->owner.setNoiseGenerator(this->replacement);
        this->destroyedDuringCall = this->destroyed;
    }

    StochasticRKIntegratorBase& owner;
    std::shared_ptr<GaussianNoiseGenerator> replacement;
    bool& destroyed;
    bool& destroyedDuringCall;
};

class FourNoiseDynamics final : public DynamicObject
{
  public:
    FourNoiseDynamics()
    {
        StateSpec spec;
        spec.state = { 1, 1 };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        spec.noiseCount = 4;
        this->state = this->dynManager.registerState("fourNoise", spec);
        this->state->stateView()(0, 0) = 0.5;
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override { this->state->derivativeView()(0, 0) = 0.0; }

    void equationsOfMotionDiffusion(double, double) override
    {
        for (size_t index = 0; index < 4; ++index) {
            this->state->diffusionView(index)(0, 0) = 0.125 * static_cast<double>(index + 1);
        }
    }

    StateData* state = nullptr;
};

class RawPseudoStepPolicy final : public StateUpdatePolicy
{
  public:
    bool topologyEquals(const StateUpdatePolicy& other) const override
    {
        return dynamic_cast<const RawPseudoStepPolicy*>(&other) != nullptr;
    }

    void validate(const StateSpec&) const override {}

    void buildDriftCandidate(ConstMatrixView base, ConstMatrixView, double, MutableMatrixView output) const override
    {
        output = base;
    }

    void applyNoiseIncrement(MutableMatrixView state, ConstMatrixView, double pseudoStep) const override
    {
        state(0, 0) = pseudoStep;
    }
};

class RawPseudoStepDynamics final : public DynamicObject
{
  public:
    RawPseudoStepDynamics()
    {
        StateSpec spec;
        spec.state = { 1, 1 };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        spec.noiseCount = 2;
        spec.updateKind = StateUpdateKind::Special;
        this->state = this->dynManager.registerState("raw", spec, std::make_unique<RawPseudoStepPolicy>());
        this->state->stateView()(0, 0) = 1.0;
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override { this->state->derivativeView()(0, 0) = 0.0; }

    void equationsOfMotionDiffusion(double, double) override
    {
        this->state->diffusionView(0)(0, 0) = 1.0;
        this->state->diffusionView(1)(0, 0) = 1.0;
    }

    StateData* state = nullptr;
};

class SlotRangeIntegrator final : public StochasticRKIntegratorBase
{
  public:
    using StochasticRKIntegratorBase::StochasticRKIntegratorBase;

    void setActiveSlots(size_t begin, size_t end)
    {
        this->slotBegin = begin;
        this->slotEnd = end;
    }

    void integrateImpl(double currentTime, double timeStep) override
    {
        this->bindFlatStochasticStorage();
        this->gatherStochasticStates();
        this->generateNoise(timeStep);
        for (DynamicObject* object : this->dynamics()) {
            object->equationsOfMotion(currentTime, timeStep);
        }
        this->gatherStochasticDerivatives();
        for (DynamicObject* object : this->dynamics()) {
            object->equationsOfMotionDiffusion(currentTime, timeStep);
        }
        this->gatherStochasticDiffusions();
        this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                       this->stochasticPackedDerivatives(),
                                       timeStep,
                                       this->stochasticPackedDiffusions(),
                                       this->flatDW(),
                                       this->slotBegin,
                                       this->slotEnd);
    }

  private:
    size_t slotBegin = 0;
    size_t slotEnd = 0;
};

class AcceptanceRollbackIntegrator final : public StochasticRKIntegratorBase
{
  public:
    using StochasticRKIntegratorBase::StochasticRKIntegratorBase;

    void setFailureAfterAcceptance(size_t value) { this->failureAfterAcceptance = value; }

    void integrateImpl(double currentTime, double timeStep) override
    {
        this->bindFlatStochasticStorage();
        this->gatherStochasticStates();
        this->generateNoise(timeStep);

        try {
            for (DynamicObject* object : this->dynamics()) {
                object->equationsOfMotion(currentTime, timeStep);
            }
            this->gatherStochasticDerivatives();
            for (DynamicObject* object : this->dynamics()) {
                object->equationsOfMotionDiffusion(currentTime, timeStep);
            }
            this->gatherStochasticDiffusions();

            for (size_t acceptance = 1; acceptance <= 3; ++acceptance) {
                this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                               this->stochasticPackedDerivatives(),
                                               timeStep,
                                               this->stochasticPackedDiffusions(),
                                               this->flatDW());
                this->acceptStochasticCandidate();
                if (acceptance == this->failureAfterAcceptance) {
                    throw std::runtime_error("deliberate post-acceptance failure");
                }
            }
        } catch (...) {
            this->restoreStochasticStates();
            throw;
        }
    }

  private:
    size_t failureAfterAcceptance = 0;
};

class FinalCandidateProbe final : public StochasticRKIntegratorBase
{
  public:
    FinalCandidateProbe(DynamicObject* dynamics, bool coalesce)
      : StochasticRKIntegratorBase(dynamics)
      , coalesce(coalesce)
    {
    }

    void integrateImpl(double currentTime, double timeStep) override
    {
        this->bindFlatStochasticStorage([this] {
            const auto noiseCount = static_cast<Eigen::Index>(this->globalNoiseCount());
            this->firstSteps.resize(noiseCount);
            this->secondSteps.resize(noiseCount);
            this->thirdSteps.resize(noiseCount);
            for (Eigen::Index index = 0; index < noiseCount; ++index) {
                this->firstSteps(index) = 0.125 * static_cast<double>(index + 1);
                this->secondSteps(index) = -0.0625 * static_cast<double>(index + 2);
                this->thirdSteps(index) = 0.03125 * static_cast<double>(index + 3);
            }
        });
        this->gatherStochasticStates();
        for (DynamicObject* object : this->dynamics()) {
            object->equationsOfMotion(currentTime, timeStep);
        }
        this->gatherStochasticDerivatives();
        for (DynamicObject* object : this->dynamics()) {
            object->equationsOfMotionDiffusion(currentTime, timeStep);
        }
        this->gatherStochasticDiffusions();

        if (this->coalesce) {
            this->beginAllEuclideanFinalCandidate(this->stochasticAcceptedState(),
                                                  this->stochasticPackedDerivatives(),
                                                  timeStep,
                                                  this->stochasticPackedDiffusions(),
                                                  this->firstSteps);
            this->appendAllEuclideanFinalCandidate(
              this->stochasticPackedDerivatives(), 0.0, this->stochasticPackedDiffusions(), this->secondSteps);
            this->appendAllEuclideanFinalCandidate(
              this->stochasticPackedDerivatives(), 0.0, this->stochasticPackedDiffusions(), this->thirdSteps);
            this->commitAllEuclideanFinalCandidate();
            return;
        }

        this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                       this->stochasticPackedDerivatives(),
                                       timeStep,
                                       this->stochasticPackedDiffusions(),
                                       this->firstSteps);
        this->acceptStochasticCandidate();
        this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                       this->stochasticPackedDerivatives(),
                                       0.0,
                                       this->stochasticPackedDiffusions(),
                                       this->secondSteps);
        this->acceptStochasticCandidate();
        this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                       this->stochasticPackedDerivatives(),
                                       0.0,
                                       this->stochasticPackedDiffusions(),
                                       this->thirdSteps);
    }

  private:
    bool coalesce = false;
    Eigen::VectorXd firstSteps;
    Eigen::VectorXd secondSteps;
    Eigen::VectorXd thirdSteps;
};

std::shared_ptr<PrescribedGaussianNoiseGenerator>
prescribed(std::initializer_list<double> dW)
{
    auto generator = std::make_shared<PrescribedGaussianNoiseGenerator>();
    generator->pushStep(std::vector<double>(dW));
    return generator;
}

void
expectTopologyStep(TopologyDynamics& dynamics)
{
    svStochasticIntegratorMayurama integrator(&dynamics);
    integrator.setNoiseGenerator(prescribed({ 2.0, 3.0, 5.0 }));
    stepIntegrator(integrator, 1.0, 2.0);

    EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(32.5));
    EXPECT_EQ(doubleBits(dynamics.zeta->stateView()(0, 0)), doubleBits(2499.0));
}
}

TEST(FlatStochastic, LexicalSlotsAndLocalOrderWithFinalizedManager)
{
    TopologyDynamics dynamics(true);
    expectTopologyStep(dynamics);
}

TEST(FlatStochastic, UnfinalizedManagerIsRejected)
{
    TopologyDynamics dynamics(false);
    svStochasticIntegratorMayurama integrator(&dynamics);
    EXPECT_THROW(stepIntegrator(integrator, 1.0, 2.0), std::logic_error);
}

TEST(FlatStochastic, EulerMaruyamaRetainsDriftAcrossDiffusionCallback)
{
    TopologyDynamics dynamics(true);
    dynamics.overwriteDriftDuringDiffusion = true;
    expectTopologyStep(dynamics);
}

TEST(FlatStochastic, RandomGeneratorSubclassRetainsVirtualDispatch)
{
    class ConstantGenerator final : public RandomGaussianNoiseGenerator
    {
      public:
        void generateWienerInto(Eigen::VectorXd& dW, size_t, double) override
        {
            ++this->calls;
            dW.setConstant(2.0);
        }
        size_t calls = 0;
    };

    TopologyDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);
    auto generator = std::make_shared<ConstantGenerator>();
    integrator.setNoiseGenerator(generator);

    stepIntegrator(integrator, 1.0, 2.0);

    EXPECT_EQ(generator->calls, 1U);
    EXPECT_DOUBLE_EQ(dynamics.alpha->stateView()(0, 0), 22.5);
    EXPECT_DOUBLE_EQ(dynamics.zeta->stateView()(0, 0), 2199.0);
}

TEST(FlatStochastic, ManagerLocalSharedIdsDoNotCollideAcrossObjects)
{
    TopologyDynamics primary(true);
    TopologyDynamics secondary(true);
    auto* primaryIntegrator = new svStochasticIntegratorMayurama(&primary);
    primary.setIntegrator(primaryIntegrator);
    secondary.setIntegrator(new svStochasticIntegratorMayurama(&secondary));
    primary.syncDynamicsIntegration(&secondary);
    primaryIntegrator->setNoiseGenerator(prescribed({ 2.0, 3.0, 5.0, 7.0, 11.0, 13.0 }));

    stepIntegrator(primaryIntegrator, 1.0, 2.0);

    EXPECT_EQ(doubleBits(primary.alpha->stateView()(0, 0)), doubleBits(32.5));
    EXPECT_EQ(doubleBits(primary.zeta->stateView()(0, 0)), doubleBits(2499.0));
    EXPECT_EQ(doubleBits(secondary.alpha->stateView()(0, 0)), doubleBits(117.5));
    EXPECT_EQ(doubleBits(secondary.zeta->stateView()(0, 0)), doubleBits(8299.0));
}

TEST(FlatStochastic, UpdatePreservesNoncommutativeLocalOrder)
{
    OrderedDynamics dynamics;
    svStochasticIntegratorMayurama integrator(&dynamics);
    integrator.setNoiseGenerator(prescribed({ 2.0, 3.0 }));

    stepIntegrator(integrator, 0.0, 1.0);

    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(23.0));
}

TEST(FlatStochastic, SharedSlotsPreserveNoncommutativeLocalOrder)
{
    OrderedDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);
    integrator.setNoiseGenerator(prescribed({ 2.0, 3.0 }));

    stepIntegrator(integrator, 0.0, 1.0);

    // Local source 0 uses global draw 1, then local source 1 uses global draw 0.
    // The ordered policy gives (1 * (1 + 3) + 2) * (1 + 2) + 3.
    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(21.0));
    EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(2.0));
}

TEST(FlatStochastic, RKMilPreservesSharedLocalPolicyOrder)
{
    OrderedDynamics dynamics(true);
    svStochasticIntegratorRKMil integrator(&dynamics);
    integrator.setNoiseGenerator(prescribed({ 2.0, 3.0 }));
    const double timeStep = 1.0; // [s]

    stepIntegrator(integrator, 0.0, timeStep);

    // This policy acts even on zero increments: K = 1 + 2 + 3 = 6.
    // Local Wiener updates give (6 * 4 + 2) * 3 + 3 = 81.
    // Constant diffusion gives zero ggPrime, so the last policy updates
    // multiply by (1 + 4) and (1 + 1.5), yielding 1012.5.
    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(1012.5));
    EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(2.0));
}

TEST(FlatStochastic, CandidateSizeErrorsLeaveLiveStatesUnchanged)
{
    for (const std::string path : { "buffered", "inPlace", "diffusion", "euclideanFinal" }) {
        SCOPED_TRACE(path);
        for (bool resizeDiffusion : { false, true }) {
            SCOPED_TRACE(resizeDiffusion);
            for (Eigen::Index adjustment : { -1, 1 }) {
                SCOPED_TRACE(adjustment);
                OrderedDynamics dynamics(true, false, true);
                const double timeStep = 1.0; // [s]
                dynamics.equationsOfMotionDiffusion(0.0, timeStep);
                CandidateOrderProbe integrator(&dynamics);
                const Eigen::VectorXd steps = Eigen::VectorXd::Ones(2 + (resizeDiffusion ? 0 : adjustment));

                EXPECT_THROW(integrator.apply(path, steps, 0, 2, resizeDiffusion ? adjustment : 0),
                             std::invalid_argument);

                EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(1.0));
                EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(0.0));
            }
        }
    }
}

TEST(FlatStochastic, SharedSlotRangesPreserveLocalPolicyOrder)
{
    for (const std::string path : { "buffered", "inPlace" }) {
        SCOPED_TRACE(path);
        for (const auto& range :
             std::array<std::array<size_t, 2>, 4>{ { { { 0, 2 } }, { { 0, 1 } }, { { 1, 2 } }, { { 1, 1 } } } }) {
            SCOPED_TRACE(range[0]);
            SCOPED_TRACE(range[1]);
            OrderedDynamics dynamics(true, true);
            dynamics.equationsOfMotionDiffusion(0.0, 1.0);
            CandidateOrderProbe integrator(&dynamics);
            Eigen::VectorXd steps(3);
            steps << 2.0, 3.0, 5.0;

            integrator.apply(path, steps, range[0], range[1]);

            const double expected = range[0] == range[1] ? 1.0 : range[1] - range[0] == 2 ? 21.0 : 6.0;
            EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(expected));
        }
    }
}

TEST(FlatStochastic, SharedSlotsPreserveEuclideanRoundingInEveryCandidatePath)
{
    for (const std::string path : { "buffered", "inPlace", "diffusion", "euclideanFinal" }) {
        SCOPED_TRACE(path);
        OrderedDynamics dynamics(true, false, true);
        dynamics.state->stateView()(0, 0) = 1.0e16;
        dynamics.state->diffusionView(0)(0, 0) = -1.0e16;
        dynamics.state->diffusionView(1)(0, 0) = 1.0;
        CandidateOrderProbe integrator(&dynamics);
        const Eigen::VectorXd steps = Eigen::VectorXd::Ones(2);

        integrator.apply(path, steps, 0, 2);

        EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(1.0));
    }
}

TEST(FlatStochastic, ZeroDurationBindsWithoutConsumingNoise)
{
    TopologyDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);
    auto generator = prescribed({ 2.0, 3.0, 5.0 });
    integrator.setNoiseGenerator(generator);

    stepIntegrator(integrator, 0.0, 0.0);

    EXPECT_EQ(generator->remaining(), 1U);
    EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(0.0));
    EXPECT_EQ(doubleBits(dynamics.zeta->stateView()(0, 0)), doubleBits(0.0));
}

TEST(FlatStochastic, InvalidTimeStepsAreRejectedBeforeSampling)
{
    TopologyDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);
    auto generator = std::make_shared<LegacyCustomNoiseGenerator>();
    integrator.setNoiseGenerator(generator);

    EXPECT_THROW(stepIntegrator(integrator, 0.0, -1.0), std::invalid_argument);
    EXPECT_THROW(stepIntegrator(integrator, 0.0, std::numeric_limits<double>::infinity()), std::invalid_argument);
    EXPECT_EQ(generator->fullCalls, 0U);
    EXPECT_DOUBLE_EQ(dynamics.alpha->stateView()(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(dynamics.zeta->stateView()(0, 0), 0.0);
}

TEST(FlatStochastic, FailedPostSampleAttemptRollsBackWithoutRewinding)
{
    TopologyDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);
    auto generator = std::make_shared<PrescribedGaussianNoiseGenerator>();
    generator->pushStep({ 2.0, 3.0, 5.0 });
    generator->pushStep({ 7.0, 11.0, 13.0 });
    integrator.setNoiseGenerator(generator);
    dynamics.throwDuringDiffusion = true;

    EXPECT_THROW(stepIntegrator(integrator, 0.0, 1.0), std::runtime_error);
    EXPECT_EQ(generator->remaining(), 1U);
    EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(0.0));
    EXPECT_EQ(doubleBits(dynamics.zeta->stateView()(0, 0)), doubleBits(0.0));

    dynamics.throwDuringDiffusion = false;
    stepIntegrator(integrator, 0.0, 1.0);
    EXPECT_EQ(generator->remaining(), 0U);
    EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(117.25));
    EXPECT_EQ(doubleBits(dynamics.zeta->stateView()(0, 0)), doubleBits(8299.5));
}

TEST(FlatStochastic, BookkeepingAndNativeGenerationDoNotAllocateAfterBind)
{
    TopologyDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);
    integrator.setRNGSeed(90210);
    stepIntegrator(integrator, 0.0, 0.0);

    const auto observed = integrator_test::trackAllocations([&]() { stepIntegrator(integrator, 0.0, 0.125); });
    EXPECT_EQ(observed.allocationCalls(), 0U);
}

TEST(FlatStochastic, SparseMultiNoiseCandidatesMatchFullRebuilds)
{
    SparseEquivalenceDynamics fullDynamics;
    SparseEquivalenceDynamics sparseDynamics;
    SparseSequenceIntegrator full(&fullDynamics, false);
    SparseSequenceIntegrator sparse(&sparseDynamics, true);

    stepIntegrator(full, 0.5, 0.25);
    stepIntegrator(sparse, 0.5, 0.25);

    ASSERT_EQ(fullDynamics.observationCount, sparseDynamics.observationCount);
    for (size_t index = 0; index < fullDynamics.observationCount; ++index) {
        EXPECT_EQ(doubleBits(fullDynamics.observed[index]), doubleBits(sparseDynamics.observed[index]));
    }
    EXPECT_EQ(doubleBits(fullDynamics.alpha->stateView()(0, 0)), doubleBits(sparseDynamics.alpha->stateView()(0, 0)));
    EXPECT_EQ(doubleBits(fullDynamics.beta->stateView()(0, 0)), doubleBits(sparseDynamics.beta->stateView()(0, 0)));
    EXPECT_EQ(doubleBits(fullDynamics.filler->stateView()(0, 0)), doubleBits(sparseDynamics.filler->stateView()(0, 0)));
}

TEST(FlatStochastic, WienerOnlyFallbackSupportsLegacyCustomGenerators)
{
    TopologyDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);
    auto generator = std::make_shared<LegacyCustomNoiseGenerator>();
    integrator.setNoiseGenerator(generator);
    stepIntegrator(integrator, 0.0, 0.125);

    EXPECT_EQ(generator->fullCalls, 1U);
    EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(2.78125));
    EXPECT_EQ(doubleBits(dynamics.zeta->stateView()(0, 0)), doubleBits(274.9375));
}

TEST(FlatStochastic, DefaultGenerateIntoValidatesLegacySampleDimensions)
{
    InvalidLegacyNoiseGenerator generator;
    Eigen::VectorXd dW = Eigen::VectorXd::Constant(2, 3.0);
    Eigen::VectorXd dZ = Eigen::VectorXd::Constant(2, 4.0);

    EXPECT_THROW(generator.generateInto(dW, dZ, 2, 1.0), std::invalid_argument);
    EXPECT_EQ(doubleBits(dW(0)), doubleBits(3.0));
    EXPECT_EQ(doubleBits(dZ(0)), doubleBits(4.0));
}

TEST(FlatStochastic, NullNoiseGeneratorAssignmentIsRejected)
{
    TopologyDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);

    EXPECT_THROW(integrator.setNoiseGenerator(nullptr), std::invalid_argument);
    EXPECT_NO_THROW(integrator.setRNGSeed(1));
}

TEST(FlatStochastic, ReentrantGeneratorReplacementRetainsActiveCall)
{
    TopologyDynamics dynamics(true);
    svStochasticIntegratorMayurama integrator(&dynamics);
    auto replacement = prescribed({ 0.0, 0.0, 0.0 });

    bool seedGeneratorDestroyed = false;
    bool seedGeneratorDestroyedDuringCall = false;
    auto seedGenerator = std::make_shared<ReentrantNoiseGenerator>(
      integrator, replacement, seedGeneratorDestroyed, seedGeneratorDestroyedDuringCall);
    std::weak_ptr<GaussianNoiseGenerator> seedLifetime = seedGenerator;
    integrator.setNoiseGenerator(seedGenerator);
    seedGenerator.reset();

    integrator.setRNGSeed(1);

    EXPECT_FALSE(seedGeneratorDestroyedDuringCall);
    EXPECT_TRUE(seedGeneratorDestroyed);
    EXPECT_TRUE(seedLifetime.expired());

    bool sampleGeneratorDestroyed = false;
    bool sampleGeneratorDestroyedDuringCall = false;
    auto sampleGenerator = std::make_shared<ReentrantNoiseGenerator>(
      integrator, replacement, sampleGeneratorDestroyed, sampleGeneratorDestroyedDuringCall);
    std::weak_ptr<GaussianNoiseGenerator> sampleLifetime = sampleGenerator;
    integrator.setNoiseGenerator(sampleGenerator);
    sampleGenerator.reset();

    stepIntegrator(integrator, 0.0, 0.125);

    EXPECT_FALSE(sampleGeneratorDestroyedDuringCall);
    EXPECT_TRUE(sampleGeneratorDestroyed);
    EXPECT_TRUE(sampleLifetime.expired());

    svStochasticIntegratorSRA1 auxiliaryIntegrator(&dynamics);
    bool auxiliaryGeneratorDestroyed = false;
    bool auxiliaryGeneratorDestroyedDuringCall = false;
    auto auxiliaryGenerator = std::make_shared<ReentrantNoiseGenerator>(
      auxiliaryIntegrator, replacement, auxiliaryGeneratorDestroyed, auxiliaryGeneratorDestroyedDuringCall);
    std::weak_ptr<GaussianNoiseGenerator> auxiliaryLifetime = auxiliaryGenerator;
    auxiliaryIntegrator.setNoiseGenerator(auxiliaryGenerator);
    auxiliaryGenerator.reset();

    stepIntegrator(auxiliaryIntegrator, 0.0, 0.125);

    EXPECT_FALSE(auxiliaryGeneratorDestroyedDuringCall);
    EXPECT_TRUE(auxiliaryGeneratorDestroyed);
    EXPECT_TRUE(auxiliaryLifetime.expired());
}

TEST(FlatStochastic, AuxiliaryCountAdapterPreservesLegacyGenerators)
{
    LegacyCustomNoiseGenerator generator;
    Eigen::VectorXd dW(4);
    Eigen::VectorXd dZ(4);
    generator.generateWithAuxiliaryInto(dW, dZ, 4, 2, 0.125);

    EXPECT_EQ(generator.fullCalls, 1U);
    EXPECT_EQ(doubleBits(dW(3)), doubleBits(0.25));
    EXPECT_EQ(doubleBits(dZ(3)), doubleBits(9.0));
}

TEST(FlatStochastic, ConcreteWeakMethodsRequestOnlyUsedAuxiliaryDraws)
{
    {
        FourNoiseDynamics dynamics;
        svStochasticIntegratorW2Ito1 integrator(&dynamics);
        auto generator = std::make_shared<AuxiliaryCountNoiseGenerator>();
        integrator.setNoiseGenerator(generator);
        stepIntegrator(integrator, 0.0, 0.125);
        EXPECT_EQ(generator->calls, 1U);
        EXPECT_EQ(generator->lastWienerCount, 4U);
        EXPECT_EQ(generator->lastAuxiliaryCount, 2U);
    }
    {
        FourNoiseDynamics dynamics;
        svStochasticIntegratorDRI1NM integrator(&dynamics);
        auto generator = std::make_shared<AuxiliaryCountNoiseGenerator>();
        integrator.setNoiseGenerator(generator);
        stepIntegrator(integrator, 0.0, 0.125);
        EXPECT_EQ(generator->calls, 1U);
        EXPECT_EQ(generator->lastWienerCount, 4U);
        EXPECT_EQ(generator->lastAuxiliaryCount, 0U);
    }
    {
        FourNoiseDynamics dynamics;
        svStochasticIntegratorRS1 integrator(&dynamics);
        auto generator = std::make_shared<AuxiliaryCountNoiseGenerator>();
        integrator.setNoiseGenerator(generator);
        stepIntegrator(integrator, 0.0, 0.125);
        EXPECT_EQ(generator->calls, 1U);
        EXPECT_EQ(generator->lastWienerCount, 4U);
        EXPECT_EQ(generator->lastAuxiliaryCount, 3U);
    }
}

TEST(FlatStochastic, EuclideanFinalCandidateCoalescingIsBitwiseEquivalent)
{
    TopologyDynamics sequentialDynamics(true);
    TopologyDynamics coalescedDynamics(true);
    FinalCandidateProbe sequential(&sequentialDynamics, false);
    FinalCandidateProbe coalesced(&coalescedDynamics, true);

    stepIntegrator(sequential, 0.75, 0.125);
    stepIntegrator(coalesced, 0.75, 0.125);

    EXPECT_EQ(doubleBits(sequentialDynamics.alpha->stateView()(0, 0)),
              doubleBits(coalescedDynamics.alpha->stateView()(0, 0)));
    EXPECT_EQ(doubleBits(sequentialDynamics.zeta->stateView()(0, 0)),
              doubleBits(coalescedDynamics.zeta->stateView()(0, 0)));
}

TEST(FlatStochastic, EuclideanFinalCandidateRejectsSpecialTopology)
{
    OrderedDynamics dynamics;
    FinalCandidateProbe coalesced(&dynamics, true);

    EXPECT_THROW(stepIntegrator(coalesced, 0.0, 0.125), std::logic_error);
    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(1.0));
}

TEST(FlatStochastic, PrescribedReplayUsesCursorSemantics)
{
    PrescribedGaussianNoiseGenerator generator;
    generator.pushStep({ 1.0, 2.0 }, { 3.0, 4.0 });
    generator.pushStep({ 5.0, 6.0 }, { 7.0, 8.0 });

    Eigen::VectorXd dW(2);
    Eigen::VectorXd unusedDZ(2);
    generator.generateWienerInto(dW, unusedDZ, 2, 1.0);
    EXPECT_EQ(generator.remaining(), 1U);
    EXPECT_EQ(doubleBits(dW(0)), doubleBits(1.0));

    generator.pushStep({ 9.0, 10.0 });
    EXPECT_EQ(generator.remaining(), 2U);
    generator.generateWienerInto(dW, unusedDZ, 2, 1.0);
    EXPECT_EQ(doubleBits(dW(0)), doubleBits(5.0));
    generator.generateWienerInto(dW, unusedDZ, 2, 1.0);
    EXPECT_EQ(doubleBits(dW(0)), doubleBits(9.0));

    generator.clear();
    EXPECT_EQ(generator.remaining(), 0U);
    EXPECT_THROW(generator.generateWienerInto(dW, unusedDZ, 2, 1.0), std::runtime_error);
}

TEST(FlatStochastic, PrescribedReplayCanDiscardConsumedSamples)
{
    PrescribedGaussianNoiseGenerator generator;
    generator.pushStep({ 1.0, 2.0 }, { 3.0, 4.0 });
    generator.pushStep({ 5.0, 6.0 }, { 7.0, 8.0 });
    generator.pushStep({ 9.0, 10.0 }, { 11.0, 12.0 });

    Eigen::VectorXd dW(2);
    Eigen::VectorXd dZ(2);
    generator.generateInto(dW, dZ, 2, 1.0);
    generator.discardConsumed();

    EXPECT_EQ(generator.remaining(), 2U);
    generator.generateInto(dW, dZ, 2, 1.0);
    EXPECT_EQ(doubleBits(dW(0)), doubleBits(5.0));
    EXPECT_EQ(doubleBits(dZ(0)), doubleBits(7.0));

    generator.pushStep({ 13.0, 14.0 }, { 15.0, 16.0 });
    generator.discardConsumed();
    EXPECT_EQ(generator.remaining(), 2U);

    generator.generateInto(dW, dZ, 2, 1.0);
    EXPECT_EQ(doubleBits(dW(0)), doubleBits(9.0));
    generator.generateInto(dW, dZ, 2, 1.0);
    EXPECT_EQ(doubleBits(dW(0)), doubleBits(13.0));
    EXPECT_EQ(generator.remaining(), 0U);

    generator.discardConsumed();
    EXPECT_EQ(generator.remaining(), 0U);
}

TEST(FlatStochastic, ActiveSlotRangePreservesRawPseudoStepBits)
{
    RawPseudoStepDynamics dynamics;
    SlotRangeIntegrator integrator(&dynamics);
    const double negativeZero = -0.0;
    uint64_t nanBits = UINT64_C(0x7ff8000000001234);
    double payloadNaN = 0.0;
    std::memcpy(&payloadNaN, &nanBits, sizeof(payloadNaN));
    integrator.setNoiseGenerator(prescribed({ negativeZero, payloadNaN }));

    integrator.setActiveSlots(0, 1);
    stepIntegrator(integrator, 0.0, 1.0);

    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(negativeZero));
}

TEST(FlatStochastic, FullSlotRangePreservesNaNPayloadBits)
{
    RawPseudoStepDynamics dynamics;
    SlotRangeIntegrator integrator(&dynamics);
    uint64_t nanBits = UINT64_C(0x7ff8000000001234);
    double payloadNaN = 0.0;
    std::memcpy(&payloadNaN, &nanBits, sizeof(payloadNaN));
    integrator.setNoiseGenerator(prescribed({ -0.0, payloadNaN }));

    integrator.setActiveSlots(0, 2);
    stepIntegrator(integrator, 0.0, 1.0);

    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), nanBits);
}

TEST(FlatStochastic, InvalidSlotRangesAreRejected)
{
    {
        RawPseudoStepDynamics dynamics;
        SlotRangeIntegrator integrator(&dynamics);
        integrator.setNoiseGenerator(prescribed({ 1.0, 2.0 }));
        integrator.setActiveSlots(1, 0);
        EXPECT_THROW(stepIntegrator(integrator, 0.0, 1.0), std::invalid_argument);
    }
    {
        RawPseudoStepDynamics dynamics;
        SlotRangeIntegrator integrator(&dynamics);
        integrator.setNoiseGenerator(prescribed({ 1.0, 2.0 }));
        integrator.setActiveSlots(0, 3);
        EXPECT_THROW(stepIntegrator(integrator, 0.0, 1.0), std::invalid_argument);
    }
}

TEST(FlatStochastic, CandidateAssemblyRejectsStaleScratchUse)
{
    TopologyDynamics dynamics(true);
    BindingInvariantProbe probe(&dynamics);
    probe.bind();

    EXPECT_THROW(probe.gatherIntoWrongSizedDiffusionBuffer(), std::invalid_argument);
    EXPECT_THROW(probe.buildSparseCandidateWithoutBegin(), std::logic_error);
    EXPECT_THROW(probe.appendFinalCandidateWithoutBegin(), std::logic_error);
}

TEST(FlatStochastic, FailuresAfterEachAcceptanceRestoreEntryState)
{
    for (size_t failureAfter = 1; failureAfter <= 3; ++failureAfter) {
        TopologyDynamics dynamics(true);
        AcceptanceRollbackIntegrator integrator(&dynamics);
        integrator.setNoiseGenerator(prescribed({ 2.0, 3.0, 5.0 }));
        integrator.setFailureAfterAcceptance(failureAfter);

        EXPECT_THROW(stepIntegrator(integrator, 1.0, 2.0), std::runtime_error);
        EXPECT_EQ(doubleBits(dynamics.alpha->stateView()(0, 0)), doubleBits(0.0));
        EXPECT_EQ(doubleBits(dynamics.zeta->stateView()(0, 0)), doubleBits(0.0));
    }
}

namespace {
template<typename Method>
StochasticRKIntegratorBase*
makeLifecycleIntegrator(DynamicObject* object)
{
    return new Method(object);
}
}

TEST(FlatStochastic, SurvivingDynamicsAdvanceAfterSynchronizedPeerDestruction)
{
    using Factory = StochasticRKIntegratorBase* (*)(DynamicObject*);
    const std::pair<const char*, Factory> methods[] = {
        { "Mayurama", makeLifecycleIntegrator<svStochasticIntegratorMayurama> },
        { "EulerHeun", makeLifecycleIntegrator<svStochasticIntegratorEulerHeun> },
        { "RKMil", makeLifecycleIntegrator<svStochasticIntegratorRKMil> },
        { "RDI1WM", makeLifecycleIntegrator<svStochasticIntegratorRDI1WM> },
        { "SRA1", makeLifecycleIntegrator<svStochasticIntegratorSRA1> },
        { "SOSRA", makeLifecycleIntegrator<svStochasticIntegratorSOSRA> },
        { "SRIW1", makeLifecycleIntegrator<svStochasticIntegratorSRIW1> },
        { "SOSRI", makeLifecycleIntegrator<svStochasticIntegratorSOSRI> },
        { "DRI1", makeLifecycleIntegrator<svStochasticIntegratorDRI1> },
        { "DRI1NM", makeLifecycleIntegrator<svStochasticIntegratorDRI1NM> },
        { "RI1", makeLifecycleIntegrator<svStochasticIntegratorRI1> },
        { "RI3", makeLifecycleIntegrator<svStochasticIntegratorRI3> },
        { "RI5", makeLifecycleIntegrator<svStochasticIntegratorRI5> },
        { "RI6", makeLifecycleIntegrator<svStochasticIntegratorRI6> },
        { "W2Ito1", makeLifecycleIntegrator<svStochasticIntegratorW2Ito1> },
        { "W2Ito2", makeLifecycleIntegrator<svStochasticIntegratorW2Ito2> },
        { "RS1", makeLifecycleIntegrator<svStochasticIntegratorRS1> },
        { "RS2", makeLifecycleIntegrator<svStochasticIntegratorRS2> },
        { "SIEA", makeLifecycleIntegrator<svStochasticIntegratorSIEA> },
        { "SMEA", makeLifecycleIntegrator<svStochasticIntegratorSMEA> },
        { "SIEB", makeLifecycleIntegrator<svStochasticIntegratorSIEB> },
        { "SMEB", makeLifecycleIntegrator<svStochasticIntegratorSMEB> },
    };
    const auto samples = [](size_t noiseCount) {
        auto generator = std::make_shared<PrescribedGaussianNoiseGenerator>();
        generator->pushStep(std::vector<double>(noiseCount, 0.25), std::vector<double>(noiseCount, 0.125));
        return generator;
    };
    for (const auto& method : methods) {
        for (bool destroyPrimary : { false, true }) {
            SCOPED_TRACE(::testing::Message() << method.first << ", destroyPrimary=" << destroyPrimary);
            auto primary = std::make_unique<TopologyDynamics>(true);
            auto secondary = std::make_unique<TopologyDynamics>(true);
            auto* primaryIntegrator = method.second(primary.get());
            auto* secondaryIntegrator = method.second(secondary.get());
            primary->setIntegrator(primaryIntegrator);
            secondary->setIntegrator(secondaryIntegrator);
            primary->timeStep = 2.0;   // [s]
            secondary->timeStep = 2.0; // [s]
            primary->syncDynamicsIntegration(secondary.get());
            primaryIntegrator->setNoiseGenerator(samples(6));
            primary->integrateState(0);

            auto* survivor = destroyPrimary ? secondary.get() : primary.get();
            auto* survivorIntegrator = destroyPrimary ? secondaryIntegrator : primaryIntegrator;
            TopologyDynamics control(true);
            control.alpha->setState(survivor->alpha->stateView());
            control.zeta->setState(survivor->zeta->stateView());
            control.timeStep = 2.0; // [s]
            auto* controlIntegrator = method.second(&control);
            control.setIntegrator(controlIntegrator);
            if (destroyPrimary) {
                primary.reset();
            } else {
                secondary.reset();
            }
            // Compare refreshed group storage with a fresh single-object workspace
            // using the same nonzero Wiener and auxiliary samples.
            survivorIntegrator->setNoiseGenerator(samples(3));
            controlIntegrator->setNoiseGenerator(samples(3));
            ASSERT_NO_THROW(survivor->integrateState(0));
            control.integrateState(0);
            EXPECT_EQ(survivorIntegrator->getDynamicsCount(), 1U);
            EXPECT_NEAR(survivor->alpha->stateView()(0, 0), control.alpha->stateView()(0, 0), 1e-10);
            EXPECT_NEAR(survivor->zeta->stateView()(0, 0), control.zeta->stateView()(0, 0), 1e-10);
        }
    }
}
