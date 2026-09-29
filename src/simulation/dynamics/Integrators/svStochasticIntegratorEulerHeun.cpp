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
#include "svStochasticIntegratorEulerHeun.h"

#include "../_GeneralModuleFiles/stateData.h"

/** @brief Method workspace allocated after state binding. */
struct svStochasticIntegratorEulerHeun::FlatStorage
{
    Eigen::VectorXd firstDrift; ///< Drift evaluated at the step-entry state.
    Eigen::VectorXd firstDiffusions; ///< Packed diffusions evaluated at the step-entry state.
};

svStochasticIntegratorEulerHeun::svStochasticIntegratorEulerHeun(DynamicObject* dynIn)
  : StochasticRKIntegratorBase(dynIn)
{
}

svStochasticIntegratorEulerHeun::~svStochasticIntegratorEulerHeun() noexcept = default;

void
svStochasticIntegratorEulerHeun::bindFlatStorage()
{
    auto storage = std::make_unique<FlatStorage>();
    storage->firstDrift.resize(this->stochasticPackedDerivatives().size());
    storage->firstDiffusions.resize(this->stochasticPackedDiffusions().size());
    this->flatStorage = std::move(storage);
}

void
svStochasticIntegratorEulerHeun::integrateImpl(double currentTime, double timeStep)
{
    if (timeStep == 0.0) {
        return;
    }

    this->gatherStochasticStates();
    this->generateWienerNoise(timeStep);

    try {
        this->evaluateDerivatives(currentTime, timeStep);
        this->gatherStochasticDerivatives(this->flatStorage->firstDrift);
        this->evaluateDiffusions(currentTime, timeStep);
        this->gatherStochasticDiffusions(this->flatStorage->firstDiffusions);

        this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                       this->flatStorage->firstDrift,
                                       timeStep,
                                       this->flatStorage->firstDiffusions,
                                       this->flatDW());

        this->evaluateDerivatives(currentTime + timeStep, timeStep);
        for (const auto& descriptor : this->stochasticStateDescriptors()) {
            Eigen::Map<Eigen::MatrixXd> firstDrift(this->flatStorage->firstDrift.data() + descriptor.derivativeOffset,
                                                   descriptor.derivativeRows,
                                                   descriptor.derivativeColumns);
            firstDrift += descriptor.state->derivativeView();
            firstDrift *= 0.5;
        }

        this->evaluateDiffusions(currentTime + timeStep, timeStep);
        const auto& packedOffsets = this->stochasticLocalNoisePackedOffsets();
        for (const auto& descriptor : this->stochasticStateDescriptors()) {
            for (size_t localNoiseIndex = 0; localNoiseIndex < descriptor.noiseCount; ++localNoiseIndex) {
                const size_t offset = packedOffsets.at(descriptor.localNoiseOffset + localNoiseIndex);
                Eigen::Map<Eigen::MatrixXd> firstDiffusion(this->flatStorage->firstDiffusions.data() + offset,
                                                           descriptor.diffusionRows,
                                                           descriptor.diffusionColumns);
                firstDiffusion += descriptor.state->diffusionView(localNoiseIndex);
                firstDiffusion *= 0.5;
            }
        }

        this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                       this->flatStorage->firstDrift,
                                       timeStep,
                                       this->flatStorage->firstDiffusions,
                                       this->flatDW());
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}
