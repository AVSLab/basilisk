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
#include "svStochasticIntegratorRKMil.h"

#include "../_GeneralModuleFiles/stateData.h"

#include <algorithm>
#include <cmath>

/** @brief Method workspace allocated after state binding. */
struct svStochasticIntegratorRKMil::FlatStorage
{
    Eigen::VectorXd drift; ///< Drift evaluated at the step-entry state.
    Eigen::VectorXd zeroDrift; ///< Zero drift for candidates perturbed only by diffusion.
    Eigen::VectorXd firstDiffusions; ///< Packed diffusions evaluated at the step-entry state.
    Eigen::VectorXd ggPrime; ///< Packed diffusion directional-derivative approximation for the Milstein correction.
    Eigen::VectorXd kState; ///< Diffusion support state used to estimate the Milstein correction.
    Eigen::VectorXd intermediateState; ///< Scratch candidate for composing the final update.
    Eigen::VectorXd zeroPseudoSteps; ///< Zero noise weights for a drift-only candidate.
    Eigen::VectorXd supportPseudoSteps; ///< Noise weights used to construct diffusion support states.
    Eigen::VectorXd milsteinPseudoSteps; ///< Per-source weights for the Milstein correction.
};

svStochasticIntegratorRKMil::svStochasticIntegratorRKMil(DynamicObject* dynIn)
  : StochasticRKIntegratorBase(dynIn)
{
}

svStochasticIntegratorRKMil::~svStochasticIntegratorRKMil() noexcept = default;

void
svStochasticIntegratorRKMil::bindFlatStorage()
{
    auto storage = std::make_unique<FlatStorage>();
    const auto stateSize = this->stochasticAcceptedState().size();
    const auto derivativeSize = this->stochasticPackedDerivatives().size();
    const auto diffusionSize = this->stochasticPackedDiffusions().size();
    const auto noiseSize = static_cast<Eigen::Index>(this->globalNoiseCount());
    storage->drift.resize(derivativeSize);
    storage->zeroDrift.resize(derivativeSize);
    storage->firstDiffusions.resize(diffusionSize);
    storage->ggPrime.resize(diffusionSize);
    storage->kState.resize(stateSize);
    storage->intermediateState.resize(stateSize);
    storage->zeroPseudoSteps.resize(noiseSize);
    storage->zeroPseudoSteps.setZero();
    storage->supportPseudoSteps.resize(noiseSize);
    storage->milsteinPseudoSteps.resize(noiseSize);
    this->flatStorage = std::move(storage);
}

void
svStochasticIntegratorRKMil::applyCandidate(const Eigen::VectorXd& base,
                                            const Eigen::VectorXd& drift,
                                            double timeStep,
                                            const Eigen::VectorXd& diffusions,
                                            const Eigen::VectorXd& pseudoSteps,
                                            Eigen::VectorXd& output)
{
    const auto& localNoiseSlots = this->stochasticLocalNoiseSlots();
    const auto& packedOffsets = this->stochasticLocalNoisePackedOffsets();
    for (const auto& descriptor : this->stochasticStateDescriptors()) {
        auto state = descriptor.stateView(output);
        // Keep the common Euclidean update inline for small state records.
        if (descriptor.usesEuclideanUpdate()) {
            state = descriptor.stateView(base) + descriptor.derivativeView(drift) * timeStep;
            for (size_t localNoiseIndex = 0; localNoiseIndex < descriptor.noiseCount; ++localNoiseIndex) {
                const size_t localIndex = descriptor.localNoiseOffset + localNoiseIndex;
                const size_t globalSlot = localNoiseSlots.at(localIndex);
                const size_t offset = packedOffsets.at(localIndex);
                state +=
                  descriptor.diffusionView(diffusions, offset) * pseudoSteps(static_cast<Eigen::Index>(globalSlot));
            }
        } else {
            this->buildStochasticDriftCandidate(
              descriptor, descriptor.stateView(base), descriptor.derivativeView(drift), timeStep, state);
            this->applyStochasticNoiseInLocalOrder(
              descriptor, state, diffusions, pseudoSteps, 0, this->globalNoiseCount());
        }
        std::copy_n(state.data(), descriptor.stateCount, descriptor.state->stateView().data());
    }
}

void
svStochasticIntegratorRKMil::integrateImpl(double currentTime, double timeStep)
{
    if (timeStep == 0.0) {
        return;
    }

    const double h = timeStep;
    const double sqh = std::sqrt(h);

    this->gatherStochasticStates();
    this->generateWienerNoise(timeStep);
    this->flatStorage->supportPseudoSteps.setConstant(sqh);

    try {
        this->evaluateDerivatives(currentTime, timeStep);
        this->gatherStochasticDerivatives(this->flatStorage->drift);
        this->evaluateDiffusions(currentTime, timeStep);
        this->gatherStochasticDiffusions(this->flatStorage->firstDiffusions);

        this->applyCandidate(this->stochasticAcceptedState(),
                             this->flatStorage->drift,
                             h,
                             this->flatStorage->firstDiffusions,
                             this->flatStorage->zeroPseudoSteps,
                             this->flatStorage->kState);

        // Retain multiplication for nonfinite and signed-zero behavior.
        this->flatStorage->zeroDrift = this->flatStorage->drift * 0.0;
        this->applyCandidate(this->flatStorage->kState,
                             this->flatStorage->zeroDrift,
                             0.0,
                             this->flatStorage->firstDiffusions,
                             this->flatStorage->supportPseudoSteps,
                             this->flatStorage->intermediateState);

        this->evaluateDiffusions(currentTime, timeStep);
        this->gatherStochasticDiffusions(this->flatStorage->ggPrime);
        const double inverseSqh = 1.0 / sqh;
        this->flatStorage->ggPrime = (this->flatStorage->ggPrime - this->flatStorage->firstDiffusions) * inverseSqh;

        for (Eigen::Index index = 0; index < this->flatDW().size(); ++index) {
            const double increment = this->flatDW()(index);
            this->flatStorage->milsteinPseudoSteps(index) = (increment * increment - h) / 2.0;
        }

        this->applyCandidate(this->flatStorage->kState,
                             this->flatStorage->zeroDrift,
                             0.0,
                             this->flatStorage->firstDiffusions,
                             this->flatDW(),
                             this->flatStorage->intermediateState);
        this->applyCandidate(this->flatStorage->intermediateState,
                             this->flatStorage->zeroDrift,
                             0.0,
                             this->flatStorage->ggPrime,
                             this->flatStorage->milsteinPseudoSteps,
                             this->flatStorage->kState);
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}
