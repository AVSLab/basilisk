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
#include "svStochasticIntegratorRDI1WM.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "../_GeneralModuleFiles/stochasticWeakRandomVariables.h"

// Coefficients for the RDI1WM tableau.
namespace {
constexpr double a021 = 2.0 / 3.0;
constexpr double b021 = 2.0 / 3.0;
constexpr double alpha1 = 1.0 / 4.0;
constexpr double alpha2 = 3.0 / 4.0;
constexpr double c02 = 2.0 / 3.0;
constexpr double beta11 = 1.0;
} // namespace

/** @brief Method workspace allocated after state binding. */
struct svStochasticIntegratorRDI1WM::FlatStorage
{
    Eigen::VectorXd firstDrift; ///< Drift evaluated at the step-entry state.
    Eigen::VectorXd secondDrift; ///< Drift evaluated at the method's second stage.
    Eigen::VectorXd firstDiffusions; ///< Packed diffusions evaluated at the step-entry state.
    Eigen::VectorXd combinedDrift; ///< Weighted combination of the method's drift stages.
    Eigen::VectorXd scaledDrift; ///< Scratch for a scaled drift contribution.
    Eigen::VectorXd scaledDiffusions; ///< Packed scratch for scaled diffusion contributions.
    Eigen::VectorXd threePointIncrements; ///< Discrete weak increments derived from Gaussian Wiener samples.
};

svStochasticIntegratorRDI1WM::svStochasticIntegratorRDI1WM(DynamicObject* dynIn)
  : StochasticRKIntegratorBase(dynIn)
{
}

svStochasticIntegratorRDI1WM::~svStochasticIntegratorRDI1WM() noexcept = default;

void
svStochasticIntegratorRDI1WM::bindFlatStorage()
{
    auto storage = std::make_unique<FlatStorage>();
    const auto derivativeSize = this->stochasticPackedDerivatives().size();
    const auto diffusionSize = this->stochasticPackedDiffusions().size();
    const auto noiseSize = static_cast<Eigen::Index>(this->globalNoiseCount());
    storage->firstDrift.resize(derivativeSize);
    storage->secondDrift.resize(derivativeSize);
    storage->firstDiffusions.resize(diffusionSize);
    storage->combinedDrift.resize(derivativeSize);
    storage->scaledDrift.resize(derivativeSize);
    storage->scaledDiffusions.resize(diffusionSize);
    storage->threePointIncrements.resize(noiseSize);
    this->flatStorage = std::move(storage);
}

void
svStochasticIntegratorRDI1WM::integrateImpl(double currentTime, double timeStep)
{
    if (timeStep == 0.0) {
        return;
    }

    this->gatherStochasticStates();
    this->generateWienerNoise(timeStep);
    for (Eigen::Index index = 0; index < this->flatDW().size(); ++index) {
        this->flatStorage->threePointIncrements(index) = stochasticWeakRV::threePoint(this->flatDW()(index), timeStep);
    }

    try {
        this->evaluateDerivatives(currentTime, timeStep);
        this->gatherStochasticDerivatives(this->flatStorage->firstDrift);
        this->evaluateDiffusions(currentTime, timeStep);
        this->gatherStochasticDiffusions(this->flatStorage->firstDiffusions);

        const auto& packedOffsets = this->stochasticLocalNoisePackedOffsets();
        for (const auto& descriptor : this->stochasticStateDescriptors()) {
            const auto derivativeOffset = static_cast<Eigen::Index>(descriptor.derivativeOffset);
            const auto derivativeCount = static_cast<Eigen::Index>(descriptor.derivativeCount);
            this->flatStorage->scaledDrift.segment(derivativeOffset, derivativeCount) =
              this->flatStorage->firstDrift.segment(derivativeOffset, derivativeCount) * a021;
            for (size_t localNoiseIndex = 0; localNoiseIndex < descriptor.noiseCount; ++localNoiseIndex) {
                const auto diffusionOffset =
                  static_cast<Eigen::Index>(packedOffsets.at(descriptor.localNoiseOffset + localNoiseIndex));
                const auto diffusionCount = static_cast<Eigen::Index>(descriptor.diffusionCount);
                this->flatStorage->scaledDiffusions.segment(diffusionOffset, diffusionCount) =
                  this->flatStorage->firstDiffusions.segment(diffusionOffset, diffusionCount) * b021;
            }
        }
        this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                       this->flatStorage->scaledDrift,
                                       timeStep,
                                       this->flatStorage->scaledDiffusions,
                                       this->flatStorage->threePointIncrements);

        this->evaluateDerivatives(currentTime + c02 * timeStep, timeStep);
        this->gatherStochasticDerivatives(this->flatStorage->secondDrift);

        for (const auto& descriptor : this->stochasticStateDescriptors()) {
            const auto offset = static_cast<Eigen::Index>(descriptor.derivativeOffset);
            Eigen::Map<const Eigen::MatrixXd> firstDrift(
              this->flatStorage->firstDrift.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
            Eigen::Map<const Eigen::MatrixXd> secondDrift(
              this->flatStorage->secondDrift.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
            Eigen::Map<Eigen::MatrixXd> combinedDrift(this->flatStorage->combinedDrift.data() + offset,
                                                      descriptor.derivativeRows,
                                                      descriptor.derivativeColumns);
            Eigen::Map<Eigen::MatrixXd> scaledDrift(
              this->flatStorage->scaledDrift.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
            combinedDrift = firstDrift * alpha1;
            scaledDrift = secondDrift * alpha2;
            combinedDrift += scaledDrift;
        }
        for (const auto& descriptor : this->stochasticStateDescriptors()) {
            for (size_t localNoiseIndex = 0; localNoiseIndex < descriptor.noiseCount; ++localNoiseIndex) {
                const size_t offset = packedOffsets.at(descriptor.localNoiseOffset + localNoiseIndex);
                Eigen::Map<Eigen::MatrixXd>(this->flatStorage->scaledDiffusions.data() + offset,
                                            descriptor.diffusionRows,
                                            descriptor.diffusionColumns) =
                  Eigen::Map<const Eigen::MatrixXd>(this->flatStorage->firstDiffusions.data() + offset,
                                                    descriptor.diffusionRows,
                                                    descriptor.diffusionColumns) *
                  beta11;
            }
        }
        this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                       this->flatStorage->combinedDrift,
                                       timeStep,
                                       this->flatStorage->scaledDiffusions,
                                       this->flatStorage->threePointIncrements);
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}
