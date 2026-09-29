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
#include "svStochasticIntegratorDRI1.h"
#include "../_GeneralModuleFiles/stochasticWeakRandomVariables.h"

#include <cmath>
#include <stdexcept>
#include <typeinfo>

svStochasticIntegratorDRI1::svStochasticIntegratorDRI1(DynamicObject* dyn)
    : StochasticRKIntegratorBase(dyn), coefficients(svStochasticIntegratorDRI1::getCoefficients())
{
}

svStochasticIntegratorDRI1::svStochasticIntegratorDRI1(DynamicObject* dyn,
                                                       const DRI1Coefficients& coefficients)
    : StochasticRKIntegratorBase(dyn), coefficients(coefficients)
{
}

void
svStochasticIntegratorDRI1::setNonMixing(bool enabled)
{
    if (enabled == this->nonMixing) {
        return;
    }
    if (this->methodStorageBound) {
        throw std::logic_error("DRI1 non-mixing mode cannot change after integrator binding.");
    }
    this->nonMixing = enabled;
}

// Coefficients for the DRI1 tableau.
DRI1Coefficients svStochasticIntegratorDRI1::getCoefficients()
{
    DRI1Coefficients c;
    c.a021 = 1.0 / 2.0;
    c.a031 = -1.0;
    c.a032 = 2.0;
    c.a121 = 342.0 / 491.0;
    c.a131 = 342.0 / 491.0;
    c.b021 = (6.0 - std::sqrt(6.0)) / 10.0;
    c.b031 = (3.0 + 2.0 * std::sqrt(6.0)) / 5.0;
    c.b121 = 3.0 * std::sqrt(38.0 / 491.0);
    c.b131 = -3.0 * std::sqrt(38.0 / 491.0);
    c.b221 = -214.0 / 513.0 * std::sqrt(1105.0 / 991.0);
    c.b222 = -491.0 / 513.0 * std::sqrt(221.0 / 4955.0);
    c.b223 = -491.0 / 513.0 * std::sqrt(221.0 / 4955.0);
    c.b231 = 214.0 / 513.0 * std::sqrt(1105.0 / 991.0);
    c.b232 = 491.0 / 513.0 * std::sqrt(221.0 / 4955.0);
    c.b233 = 491.0 / 513.0 * std::sqrt(221.0 / 4955.0);
    c.alpha1 = 1.0 / 6.0;
    c.alpha2 = 2.0 / 3.0;
    c.alpha3 = 1.0 / 6.0;
    c.c02 = 1.0 / 2.0;
    c.c03 = 1.0;
    c.c12 = 342.0 / 491.0;
    c.c13 = 342.0 / 491.0;
    c.beta11 = 193.0 / 684.0;
    c.beta12 = 491.0 / 1368.0;
    c.beta13 = 491.0 / 1368.0;
    c.beta22 = 1.0 / 6.0 * std::sqrt(491.0 / 38.0);
    c.beta23 = -1.0 / 6.0 * std::sqrt(491.0 / 38.0);
    c.beta31 = -4955.0 / 7072.0;
    c.beta32 = 4955.0 / 14144.0;
    c.beta33 = 4955.0 / 14144.0;
    c.beta42 = -1.0 / 8.0 * std::sqrt(4955.0 / 221.0);
    c.beta43 = 1.0 / 8.0 * std::sqrt(4955.0 / 221.0);
    return c;
}

// ---- RI1 / RI3 / RI5 / RI6 (Roessler 2009) : share DRI1's step, different tableaux ----
// Coefficients for the RI1/RI3/RI5/RI6 tableaux. b221=1,
// b231=-1, b222=b223=b232=b233=0 for all of these (only the diagonal cross term is used).

svStochasticIntegratorRI1::svStochasticIntegratorRI1(DynamicObject* dyn)
    : svStochasticIntegratorDRI1(dyn, svStochasticIntegratorRI1::getCoefficients())
{}

DRI1Coefficients svStochasticIntegratorRI1::getCoefficients()
{
    DRI1Coefficients c;
    c.a021 = 2.0 / 3.0; c.a031 = -1.0 / 3.0; c.a032 = 1.0;
    c.a121 = 1.0; c.a131 = 1.0;
    c.b021 = 1.0; c.b031 = 0.0;
    c.b121 = 1.0; c.b131 = -1.0;
    c.b221 = 1.0; c.b222 = 0.0; c.b223 = 0.0; c.b231 = -1.0; c.b232 = 0.0; c.b233 = 0.0;
    c.alpha1 = 1.0 / 4.0; c.alpha2 = 1.0 / 2.0; c.alpha3 = 1.0 / 4.0;
    c.c02 = 2.0 / 3.0; c.c03 = 2.0 / 3.0; c.c12 = 1.0; c.c13 = 1.0;
    c.beta11 = 1.0 / 2.0; c.beta12 = 1.0 / 4.0; c.beta13 = 1.0 / 4.0;
    c.beta22 = 1.0 / 2.0; c.beta23 = -1.0 / 2.0;
    c.beta31 = -1.0 / 2.0; c.beta32 = 1.0 / 4.0; c.beta33 = 1.0 / 4.0;
    c.beta42 = 1.0 / 2.0; c.beta43 = -1.0 / 2.0;
    return c;
}

svStochasticIntegratorRI3::svStochasticIntegratorRI3(DynamicObject* dyn)
    : svStochasticIntegratorDRI1(dyn, svStochasticIntegratorRI3::getCoefficients())
{}

DRI1Coefficients svStochasticIntegratorRI3::getCoefficients()
{
    DRI1Coefficients c;
    c.a021 = 1.0; c.a031 = 1.0 / 4.0; c.a032 = 1.0 / 4.0;
    c.a121 = 1.0; c.a131 = 1.0;
    c.b021 = (3.0 - 2.0 * std::sqrt(6.0)) / 5.0; c.b031 = (6.0 + std::sqrt(6.0)) / 10.0;
    c.b121 = 1.0; c.b131 = -1.0;
    c.b221 = 1.0; c.b222 = 0.0; c.b223 = 0.0; c.b231 = -1.0; c.b232 = 0.0; c.b233 = 0.0;
    c.alpha1 = 1.0 / 6.0; c.alpha2 = 1.0 / 6.0; c.alpha3 = 2.0 / 3.0;
    c.c02 = 1.0; c.c03 = 1.0 / 2.0; c.c12 = 1.0; c.c13 = 1.0;
    c.beta11 = 1.0 / 2.0; c.beta12 = 1.0 / 4.0; c.beta13 = 1.0 / 4.0;
    c.beta22 = 1.0 / 2.0; c.beta23 = -1.0 / 2.0;
    c.beta31 = -1.0 / 2.0; c.beta32 = 1.0 / 4.0; c.beta33 = 1.0 / 4.0;
    c.beta42 = 1.0 / 2.0; c.beta43 = -1.0 / 2.0;
    return c;
}

svStochasticIntegratorRI5::svStochasticIntegratorRI5(DynamicObject* dyn)
    : svStochasticIntegratorDRI1(dyn, svStochasticIntegratorRI5::getCoefficients())
{}

DRI1Coefficients svStochasticIntegratorRI5::getCoefficients()
{
    DRI1Coefficients c;
    c.a021 = 1.0; c.a031 = 25.0 / 144.0; c.a032 = 35.0 / 144.0;
    c.a121 = 1.0 / 4.0; c.a131 = 1.0 / 4.0;
    c.b021 = 1.0 / 3.0; c.b031 = -5.0 / 6.0;
    c.b121 = 1.0 / 2.0; c.b131 = -1.0 / 2.0;
    c.b221 = 1.0; c.b222 = 0.0; c.b223 = 0.0; c.b231 = -1.0; c.b232 = 0.0; c.b233 = 0.0;
    c.alpha1 = 1.0 / 10.0; c.alpha2 = 3.0 / 14.0; c.alpha3 = 24.0 / 35.0;
    c.c02 = 1.0; c.c03 = 5.0 / 12.0; c.c12 = 1.0 / 4.0; c.c13 = 1.0 / 4.0;
    c.beta11 = 1.0; c.beta12 = -1.0; c.beta13 = -1.0;
    c.beta22 = 1.0; c.beta23 = -1.0;
    c.beta31 = 1.0 / 2.0; c.beta32 = -1.0 / 4.0; c.beta33 = -1.0 / 4.0;
    c.beta42 = 1.0 / 2.0; c.beta43 = -1.0 / 2.0;
    return c;
}

svStochasticIntegratorRI6::svStochasticIntegratorRI6(DynamicObject* dyn)
    : svStochasticIntegratorDRI1(dyn, svStochasticIntegratorRI6::getCoefficients())
{}

DRI1Coefficients svStochasticIntegratorRI6::getCoefficients()
{
    DRI1Coefficients c;
    c.a021 = 1.0; c.a031 = 0.0; c.a032 = 0.0;
    c.a121 = 1.0; c.a131 = 1.0;
    c.b021 = 1.0; c.b031 = 0.0;
    c.b121 = 1.0; c.b131 = -1.0;
    c.b221 = 1.0; c.b222 = 0.0; c.b223 = 0.0; c.b231 = -1.0; c.b232 = 0.0; c.b233 = 0.0;
    c.alpha1 = 1.0 / 2.0; c.alpha2 = 1.0 / 2.0; c.alpha3 = 0.0;
    c.c02 = 1.0; c.c03 = 0.0; c.c12 = 1.0; c.c13 = 1.0;
    c.beta11 = 1.0 / 2.0; c.beta12 = 1.0 / 4.0; c.beta13 = 1.0 / 4.0;
    c.beta22 = 1.0 / 2.0; c.beta23 = -1.0 / 2.0;
    c.beta31 = -1.0 / 2.0; c.beta32 = 1.0 / 4.0; c.beta33 = 1.0 / 4.0;
    c.beta42 = 1.0 / 2.0; c.beta43 = -1.0 / 2.0;
    return c;
}

void
svStochasticIntegratorDRI1::bindMethodStorage()
{
    const size_t noiseCount = this->globalNoiseCount();
    const auto derivativeSize = this->stochasticPackedDerivatives().size();
    const auto diffusionSize = this->stochasticPackedDiffusions().size();
    const auto noiseSize = static_cast<Eigen::Index>(noiseCount);
    const auto hatStageCount = (!this->nonMixing && noiseCount > 1) ? noiseSize : Eigen::Index{ 0 };

    Eigen::VectorXd newCombinedDerivative(derivativeSize);
    Eigen::VectorXd newFirstDiffusionDrift(derivativeSize);
    Eigen::VectorXd newSecondDiffusionDrift(derivativeSize);
    Eigen::VectorXd newCombinedDiffusion(diffusionSize);
    Eigen::MatrixXd newDerivativeStages(derivativeSize, 3);
    Eigen::MatrixXd newDiffusionStages(diffusionSize, 3);
    Eigen::MatrixXd newFirstHatDiffusions(diffusionSize, hatStageCount);
    Eigen::MatrixXd newSecondHatDiffusions(diffusionSize, hatStageCount);
    Eigen::VectorXd newWeakDW(noiseSize);
    Eigen::VectorXd newDiagonalIntegral(noiseSize);
    Eigen::VectorXd newWeakDZ(noiseSize);
    Eigen::VectorXd newPseudoSteps(noiseSize);

    this->combinedDerivative.swap(newCombinedDerivative);
    this->firstDiffusionDrift.swap(newFirstDiffusionDrift);
    this->secondDiffusionDrift.swap(newSecondDiffusionDrift);
    this->combinedDiffusion.swap(newCombinedDiffusion);
    this->derivativeStages.swap(newDerivativeStages);
    this->diffusionStages.swap(newDiffusionStages);
    this->firstHatDiffusions.swap(newFirstHatDiffusions);
    this->secondHatDiffusions.swap(newSecondHatDiffusions);
    this->weakDW.swap(newWeakDW);
    this->diagonalIntegral.swap(newDiagonalIntegral);
    this->weakDZ.swap(newWeakDZ);
    this->pseudoSteps.swap(newPseudoSteps);
    this->methodStorageBound = true;
}

void
svStochasticIntegratorDRI1::gatherDerivativeStage(size_t stageIndex)
{
    auto stage = this->derivativeStages.col(static_cast<Eigen::Index>(stageIndex));
    this->gatherStochasticDerivatives(stage.data(), stage.size());
}

void
svStochasticIntegratorDRI1::gatherAllDiffusionStages(size_t stageIndex)
{
    for (size_t globalNoiseIndex = 0; globalNoiseIndex < this->globalNoiseCount(); ++globalNoiseIndex) {
        this->gatherDiffusionStage(globalNoiseIndex, stageIndex);
    }
}

void
svStochasticIntegratorDRI1::gatherDiffusionStage(size_t globalNoiseIndex, size_t stageIndex)
{
    const auto& slot = this->stochasticNoiseSlots().at(globalNoiseIndex);
    const size_t bindingEnd = slot.bindingBegin + slot.bindingCount;
    const auto& bindings = this->stochasticNoiseBindings();
    for (size_t bindingIndex = slot.bindingBegin; bindingIndex < bindingEnd; ++bindingIndex) {
        const auto& binding = bindings.at(bindingIndex);
        this->diffusionStages.block(static_cast<Eigen::Index>(binding.packedOffset),
                                    static_cast<Eigen::Index>(stageIndex),
                                    static_cast<Eigen::Index>(binding.scalarCount),
                                    1) =
          Eigen::Map<const Eigen::VectorXd>(binding.liveData, static_cast<Eigen::Index>(binding.scalarCount));
    }
}

void
svStochasticIntegratorDRI1::gatherHatDiffusions(size_t hatStageIndex, bool secondHat)
{
    Eigen::MatrixXd& storage = secondHat ? this->secondHatDiffusions : this->firstHatDiffusions;
    const auto& packedOffsets = this->stochasticLocalNoisePackedOffsets();
    for (const auto& descriptor : this->stochasticStateDescriptors()) {
        for (size_t localNoiseIndex = 0; localNoiseIndex < descriptor.noiseCount; ++localNoiseIndex) {
            const size_t offset = packedOffsets.at(descriptor.localNoiseOffset + localNoiseIndex);
            storage.block(static_cast<Eigen::Index>(offset),
                          static_cast<Eigen::Index>(hatStageIndex),
                          static_cast<Eigen::Index>(descriptor.diffusionCount),
                          1) =
              Eigen::Map<const Eigen::VectorXd>(descriptor.state->diffusionView(localNoiseIndex).data(),
                                                static_cast<Eigen::Index>(descriptor.diffusionCount));
        }
    }
}

void
svStochasticIntegratorDRI1::scaleDrift(size_t stageIndex, double factor, Eigen::VectorXd& output)
{
    for (const auto& descriptor : this->stochasticStateDescriptors()) {
        const auto offset = static_cast<Eigen::Index>(descriptor.derivativeOffset);
        Eigen::Map<Eigen::MatrixXd> combined(
          output.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
        const Eigen::Map<const Eigen::MatrixXd> stage(
          this->derivativeStages.col(static_cast<Eigen::Index>(stageIndex)).data() + offset,
          descriptor.derivativeRows,
          descriptor.derivativeColumns);
        combined = stage * factor;
    }
}

void
svStochasticIntegratorDRI1::combineTwoDrifts(size_t firstStage,
                                             double firstFactor,
                                             size_t secondStage,
                                             double secondFactor)
{
    for (const auto& descriptor : this->stochasticStateDescriptors()) {
        const auto offset = static_cast<Eigen::Index>(descriptor.derivativeOffset);
        Eigen::Map<Eigen::MatrixXd> combined(
          this->combinedDerivative.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
        const Eigen::Map<const Eigen::MatrixXd> first(
          this->derivativeStages.col(static_cast<Eigen::Index>(firstStage)).data() + offset,
          descriptor.derivativeRows,
          descriptor.derivativeColumns);
        const Eigen::Map<const Eigen::MatrixXd> second(
          this->derivativeStages.col(static_cast<Eigen::Index>(secondStage)).data() + offset,
          descriptor.derivativeRows,
          descriptor.derivativeColumns);
        combined = first * firstFactor;
        combined += second * secondFactor;
    }
}

void
svStochasticIntegratorDRI1::combineThreeDrifts(double firstFactor, double secondFactor, double thirdFactor)
{
    for (const auto& descriptor : this->stochasticStateDescriptors()) {
        const auto offset = static_cast<Eigen::Index>(descriptor.derivativeOffset);
        Eigen::Map<Eigen::MatrixXd> combined(
          this->combinedDerivative.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
        const Eigen::Map<const Eigen::MatrixXd> first(
          this->derivativeStages.col(0).data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
        const Eigen::Map<const Eigen::MatrixXd> second(
          this->derivativeStages.col(1).data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
        const Eigen::Map<const Eigen::MatrixXd> third(
          this->derivativeStages.col(2).data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
        combined = first * firstFactor;
        combined += second * secondFactor;
        combined += third * thirdFactor;
    }
}

void
svStochasticIntegratorDRI1::scaleDiffusion(size_t globalNoiseIndex, size_t stageIndex, double factor)
{
    const auto& slot = this->stochasticNoiseSlots().at(globalNoiseIndex);
    const size_t bindingEnd = slot.bindingBegin + slot.bindingCount;
    const auto& bindings = this->stochasticNoiseBindings();
    const auto& descriptors = this->stochasticStateDescriptors();
    for (size_t bindingIndex = slot.bindingBegin; bindingIndex < bindingEnd; ++bindingIndex) {
        const auto& binding = bindings.at(bindingIndex);
        const auto& descriptor = descriptors.at(binding.boundStateIndex);
        const auto offset = static_cast<Eigen::Index>(binding.packedOffset);
        Eigen::Map<Eigen::MatrixXd> combined(
          this->combinedDiffusion.data() + offset, descriptor.diffusionRows, descriptor.diffusionColumns);
        const Eigen::Map<const Eigen::MatrixXd> stage(
          this->diffusionStages.col(static_cast<Eigen::Index>(stageIndex)).data() + offset,
          descriptor.diffusionRows,
          descriptor.diffusionColumns);
        combined = stage * factor;
    }
}

void
svStochasticIntegratorDRI1::combineThreeDiffusions(size_t globalNoiseIndex,
                                                   double firstFactor,
                                                   double secondFactor,
                                                   double thirdFactor)
{
    const auto& slot = this->stochasticNoiseSlots().at(globalNoiseIndex);
    const size_t bindingEnd = slot.bindingBegin + slot.bindingCount;
    const auto& bindings = this->stochasticNoiseBindings();
    const auto& descriptors = this->stochasticStateDescriptors();
    for (size_t bindingIndex = slot.bindingBegin; bindingIndex < bindingEnd; ++bindingIndex) {
        const auto& binding = bindings.at(bindingIndex);
        const auto& descriptor = descriptors.at(binding.boundStateIndex);
        const auto offset = static_cast<Eigen::Index>(binding.packedOffset);
        Eigen::Map<Eigen::MatrixXd> combined(
          this->combinedDiffusion.data() + offset, descriptor.diffusionRows, descriptor.diffusionColumns);
        const Eigen::Map<const Eigen::MatrixXd> first(
          this->diffusionStages.col(0).data() + offset, descriptor.diffusionRows, descriptor.diffusionColumns);
        const Eigen::Map<const Eigen::MatrixXd> second(
          this->diffusionStages.col(1).data() + offset, descriptor.diffusionRows, descriptor.diffusionColumns);
        const Eigen::Map<const Eigen::MatrixXd> third(
          this->diffusionStages.col(2).data() + offset, descriptor.diffusionRows, descriptor.diffusionColumns);
        combined = first * firstFactor;
        combined += second * secondFactor;
        combined += third * thirdFactor;
    }
}

void
svStochasticIntegratorDRI1::copyHatDiffusion(size_t globalNoiseIndex, size_t hatStageIndex, bool secondHat)
{
    const Eigen::MatrixXd& storage = secondHat ? this->secondHatDiffusions : this->firstHatDiffusions;
    const auto& slot = this->stochasticNoiseSlots().at(globalNoiseIndex);
    const size_t bindingEnd = slot.bindingBegin + slot.bindingCount;
    const auto& bindings = this->stochasticNoiseBindings();
    for (size_t bindingIndex = slot.bindingBegin; bindingIndex < bindingEnd; ++bindingIndex) {
        const auto& binding = bindings.at(bindingIndex);
        const auto offset = static_cast<Eigen::Index>(binding.packedOffset);
        Eigen::Map<Eigen::VectorXd>(this->combinedDiffusion.data() + offset,
                                    static_cast<Eigen::Index>(binding.scalarCount)) =
          storage.block(
            offset, static_cast<Eigen::Index>(hatStageIndex), static_cast<Eigen::Index>(binding.scalarCount), 1);
    }
}

void
svStochasticIntegratorDRI1::integrateImpl(double currentTime, double timeStep)
{
    if (timeStep == 0.0) {
        return;
    }

    const DRI1Coefficients& c = this->coefficients;
    const size_t m = this->globalNoiseCount();
    const bool doCrossNoise = (m > 1) && !this->nonMixing;
    const size_t auxiliaryCount = doCrossNoise ? m - 1 : 0;
    this->gatherStochasticStates();
    this->generateNoise(timeStep, auxiliaryCount);

    const double h = timeStep;
    const double sqh = std::sqrt(h);

    // Discrete random variables (deterministic functions of the Gaussian dW/dZ):
    //   _dW : three-point in {-sqrt(3h), 0, +sqrt(3h)}
    //   chi1: (_dW^2 - h)/2      (diagonal of Ihat2)
    //   _dZ : two-point in {-sqrt(h), +sqrt(h)}  (only used for cross-noise, m>1)
    for (size_t k = 0; k < m; k++) {
        const auto index = static_cast<Eigen::Index>(k);
        this->weakDW(index) = stochasticWeakRV::threePoint(this->flatDW()(index), h);
        this->diagonalIntegral(index) = (this->weakDW(index) * this->weakDW(index) - h) / 2.0;
        if (k < auxiliaryCount) {
            this->weakDZ(index) = stochasticWeakRV::twoPoint(this->flatDZ()(index), sqh);
        }
    }

    try {
        this->evaluateDerivatives(currentTime, timeStep);
        this->gatherDerivativeStage(0);
        this->evaluateDiffusions(currentTime, timeStep);
        this->gatherAllDiffusionStages(0);

        this->scaleDrift(0, c.a021, this->combinedDerivative);
        for (size_t k = 0; k < m; ++k) {
            this->scaleDiffusion(k, 0, c.b021);
        }
        this->buildStochasticCandidateInPlace(
          this->stochasticAcceptedState(), this->combinedDerivative, timeStep, this->combinedDiffusion, this->weakDW);
        this->evaluateDerivatives(currentTime + c.c02 * timeStep, timeStep);
        this->gatherDerivativeStage(1);

        this->combineTwoDrifts(0, c.a031, 1, c.a032);
        for (size_t k = 0; k < m; ++k) {
            this->scaleDiffusion(k, 0, c.b031);
        }
        this->buildStochasticCandidateInPlace(
          this->stochasticAcceptedState(), this->combinedDerivative, timeStep, this->combinedDiffusion, this->weakDW);
        this->evaluateDerivatives(currentTime + c.c03 * timeStep, timeStep);
        this->gatherDerivativeStage(2);

        this->scaleDrift(0, c.a121, this->firstDiffusionDrift);
        this->scaleDrift(0, c.a131, this->secondDiffusionDrift);
        for (size_t k = 0; k < m; k++) {
            this->scaleDiffusion(k, 0, 1.0);
            this->pseudoSteps(static_cast<Eigen::Index>(k)) = c.b121 * sqh;
            this->buildStochasticCandidateInPlace(this->stochasticAcceptedState(),
                                                  this->firstDiffusionDrift,
                                                  timeStep,
                                                  this->combinedDiffusion,
                                                  this->pseudoSteps,
                                                  k,
                                                  k + 1);
            this->evaluateDiffusions(currentTime + c.c12 * timeStep, timeStep);
            this->gatherDiffusionStage(k, 1);

            this->scaleDiffusion(k, 0, 1.0);
            this->pseudoSteps(static_cast<Eigen::Index>(k)) = c.b131 * sqh;
            this->buildStochasticCandidateInPlace(this->stochasticAcceptedState(),
                                                  this->secondDiffusionDrift,
                                                  timeStep,
                                                  this->combinedDiffusion,
                                                  this->pseudoSteps,
                                                  k,
                                                  k + 1);
            this->evaluateDiffusions(currentTime + c.c13 * timeStep, timeStep);
            this->gatherDiffusionStage(k, 2);
        }

        if (doCrossNoise) {
            for (size_t l = 0; l < m; l++) {
                this->combineThreeDiffusions(l, c.b221, c.b222, c.b223);
                this->pseudoSteps(static_cast<Eigen::Index>(l)) = sqh;
                this->buildStochasticCandidateInPlace(this->stochasticAcceptedState(),
                                                      this->combinedDerivative,
                                                      0.0,
                                                      this->combinedDiffusion,
                                                      this->pseudoSteps,
                                                      l,
                                                      l + 1);
                this->evaluateDiffusions(currentTime, timeStep);
                this->gatherHatDiffusions(l, false);

                this->combineThreeDiffusions(l, c.b231, c.b232, c.b233);
                this->pseudoSteps(static_cast<Eigen::Index>(l)) = sqh;
                this->buildStochasticCandidateInPlace(this->stochasticAcceptedState(),
                                                      this->combinedDerivative,
                                                      0.0,
                                                      this->combinedDiffusion,
                                                      this->pseudoSteps,
                                                      l,
                                                      l + 1);
                this->evaluateDiffusions(currentTime, timeStep);
                this->gatherHatDiffusions(l, true);
            }
        }

        this->combineThreeDrifts(c.alpha1, c.alpha2, c.alpha3);
        const double self31 = doCrossNoise ? static_cast<double>(m - 1) * c.beta31 : 0.0;
        for (size_t k = 0; k < m; k++) {
            this->scaleDiffusion(k, 0, c.beta11 + self31);
        }
        this->buildStochasticCandidateInPlace(
          this->stochasticAcceptedState(), this->combinedDerivative, timeStep, this->combinedDiffusion, this->weakDW);

        for (size_t k = 0; k < m; k++) {
            this->scaleDiffusion(k, 1, 1.0);
            const auto index = static_cast<Eigen::Index>(k);
            this->pseudoSteps(index) = this->weakDW(index) * c.beta12 + this->diagonalIntegral(index) * c.beta22 / sqh;
        }
        this->applyStochasticDiffusionUpdateInPlace(this->combinedDiffusion, this->pseudoSteps);

        for (size_t k = 0; k < m; k++) {
            this->scaleDiffusion(k, 2, 1.0);
            const auto index = static_cast<Eigen::Index>(k);
            this->pseudoSteps(index) = this->weakDW(index) * c.beta13 + this->diagonalIntegral(index) * c.beta23 / sqh;
        }
        this->applyStochasticDiffusionUpdateInPlace(this->combinedDiffusion, this->pseudoSteps);

        if (doCrossNoise) {
            auto ihat2 = [&](size_t k, size_t l) -> double {
                if (k < l) {
                    return (this->weakDW(k) * this->weakDW(l) - sqh * this->weakDZ(k)) / 2.0;
                }
                return (this->weakDW(k) * this->weakDW(l) + sqh * this->weakDZ(l)) / 2.0;
            };
            for (size_t l = 0; l < m; l++) {
                for (size_t k = 0; k < m; k++) {
                    if (k == l) {
                        continue;
                    }
                    const double w2 = this->weakDW(k) * c.beta32 + ihat2(k, l) * c.beta42 / sqh;
                    const double w3 = this->weakDW(k) * c.beta33 + ihat2(k, l) * c.beta43 / sqh;

                    this->copyHatDiffusion(k, l, false);
                    this->applyStochasticNoiseSlotInPlace(this->combinedDiffusion, k, w2);

                    this->copyHatDiffusion(k, l, true);
                    this->applyStochasticNoiseSlotInPlace(this->combinedDiffusion, k, w3);
                }
            }
        }
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}
