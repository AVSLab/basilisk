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
#include "svStochasticIntegratorW2Ito.h"
#include "../_GeneralModuleFiles/stochasticWeakRandomVariables.h"

#include <algorithm>
#include <cmath>
svStochasticIntegratorW2Ito::svStochasticIntegratorW2Ito(DynamicObject* dyn,
                                                         const W2ItoCoefficients& coefficients)
    : StochasticRKIntegratorBase(dyn), coefficients(coefficients)
{
}

void
svStochasticIntegratorW2Ito::bindMethodStorage()
{
    const size_t noiseCount = this->globalNoiseCount();
    const auto derivativeSize = this->stochasticPackedDerivatives().size();
    const auto diffusionSize = this->stochasticPackedDiffusions().size();
    const auto noiseSize = static_cast<Eigen::Index>(noiseCount);
    const auto stageCount = static_cast<Eigen::Index>(this->coefficients.numStages());

    Eigen::VectorXd newCombinedDerivative(derivativeSize);
    Eigen::VectorXd newScaledDerivative(derivativeSize);
    Eigen::VectorXd newCombinedDiffusion(diffusionSize);
    Eigen::VectorXd newScaledDiffusion(diffusionSize);
    Eigen::MatrixXd newDerivativeStages(derivativeSize, stageCount);
    Eigen::MatrixXd newDiffusionStages(diffusionSize, stageCount);
    Eigen::VectorXd newWeakDW(noiseSize);
    Eigen::VectorXd newDiagonalIntegral(noiseSize);
    Eigen::VectorXd newPseudoSteps(noiseSize);

    this->combinedDerivative.swap(newCombinedDerivative);
    this->scaledDerivative.swap(newScaledDerivative);
    this->combinedDiffusion.swap(newCombinedDiffusion);
    this->scaledDiffusion.swap(newScaledDiffusion);
    this->derivativeStages.swap(newDerivativeStages);
    this->diffusionStages.swap(newDiffusionStages);
    this->weakDW.swap(newWeakDW);
    this->diagonalIntegral.swap(newDiagonalIntegral);
    this->pseudoSteps.swap(newPseudoSteps);
}

void
svStochasticIntegratorW2Ito::gatherDerivativeStage(size_t stageIndex)
{
    auto stage = this->derivativeStages.col(static_cast<Eigen::Index>(stageIndex));
    this->gatherStochasticDerivatives(stage.data(), stage.size());
}

void
svStochasticIntegratorW2Ito::gatherAllDiffusionStages(size_t stageIndex)
{
    for (size_t globalNoiseIndex = 0; globalNoiseIndex < this->globalNoiseCount(); ++globalNoiseIndex) {
        this->gatherDiffusionStage(globalNoiseIndex, stageIndex);
    }
}

void
svStochasticIntegratorW2Ito::gatherDiffusionStage(size_t globalNoiseIndex, size_t stageIndex)
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
svStochasticIntegratorW2Ito::combineDrifts(const std::vector<double>& factors, size_t length)
{
    for (const auto& descriptor : this->stochasticStateDescriptors()) {
        const auto offset = static_cast<Eigen::Index>(descriptor.derivativeOffset);
        Eigen::Map<Eigen::MatrixXd> combined(
          this->combinedDerivative.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
        Eigen::Map<Eigen::MatrixXd> scaled(
          this->scaledDerivative.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
        const Eigen::Map<const Eigen::MatrixXd> first(
          this->derivativeStages.col(0).data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
        combined = first * factors.at(0);
        for (size_t stageIndex = 1; stageIndex < length; ++stageIndex) {
            const double factor = factors.at(stageIndex);
            if (factor == 0.0) {
                continue;
            }
            const Eigen::Map<const Eigen::MatrixXd> stage(
              this->derivativeStages.col(static_cast<Eigen::Index>(stageIndex)).data() + offset,
              descriptor.derivativeRows,
              descriptor.derivativeColumns);
            scaled = stage * factor;
            combined += scaled;
        }
    }
}

void
svStochasticIntegratorW2Ito::combineDiffusion(size_t globalNoiseIndex,
                                              const std::vector<double>& factors,
                                              size_t length)
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
        Eigen::Map<Eigen::MatrixXd> scaled(
          this->scaledDiffusion.data() + offset, descriptor.diffusionRows, descriptor.diffusionColumns);
        const Eigen::Map<const Eigen::MatrixXd> first(
          this->diffusionStages.col(0).data() + offset, descriptor.diffusionRows, descriptor.diffusionColumns);
        combined = first * factors.at(0);
        for (size_t stageIndex = 1; stageIndex < length; ++stageIndex) {
            const double factor = factors.at(stageIndex);
            if (factor == 0.0) {
                continue;
            }
            const Eigen::Map<const Eigen::MatrixXd> stage(
              this->diffusionStages.col(static_cast<Eigen::Index>(stageIndex)).data() + offset,
              descriptor.diffusionRows,
              descriptor.diffusionColumns);
            scaled = stage * factor;
            combined += scaled;
        }
    }
}

void
svStochasticIntegratorW2Ito::integrateImpl(double currentTime, double timeStep)
{
    if (timeStep == 0.0) {
        return;
    }

    const W2ItoCoefficients& c = this->coefficients;
    const size_t s = c.numStages();
    const size_t m = this->globalNoiseCount();
    this->gatherStochasticStates();
    this->generateNoise(timeStep, std::min<size_t>(m, 2));

    const double h = timeStep;
    const double sqh = std::sqrt(h);

    // Discrete random variables (Tang & Xiao eq. 3.2-3.3):
    //   _dW : three-point in {-sqrt(3h), 0, +sqrt(3h)}   (one per source, from dW)
    //   xi  : eta1*sqrt(h)                               (single scalar, from dZ[0])
    //   eta2: two-point sign                             (single scalar, from dZ[1]; m>1 only)
    //   Ikk[k]  = (_dW[k]^2/xi - xi)/2                   (diagonal iterated integral)
    //   Ikl(k,l)= (_dW[l] -/+ eta2*_dW[l])/2             (mixed iterated integral, k!=l)
    // xi/eta2 are drawn from dZ only when there is at least one noise source; a system
    // with m == 0 (a deterministic ODE) has a length-0 dZ, so guard the access and let
    // the step degenerate cleanly into the underlying deterministic Runge-Kutta scheme.
    const double eta1 = (m > 0) ? stochasticWeakRV::twoPoint(this->flatDZ()(0), 1.0) : 1.0;
    const double eta2 = (m > 1) ? stochasticWeakRV::twoPoint(this->flatDZ()(1), 1.0) : 0.0;
    const double xi = sqh * eta1;
    for (size_t k = 0; k < m; k++) {
        const auto index = static_cast<Eigen::Index>(k);
        this->weakDW(index) = stochasticWeakRV::threePoint(this->flatDW()(index), h);
        this->diagonalIntegral(index) = (this->weakDW(index) * this->weakDW(index) / xi - xi) / 2.0;
    }
    auto Ikl = [&](size_t k, size_t l) -> double {
        const auto index = static_cast<Eigen::Index>(l);
        if (k < l)
            return 0.5 * (this->weakDW(index) - eta2 * this->weakDW(index));
        return 0.5 * (this->weakDW(index) + eta2 * this->weakDW(index)); // k > l
    };

    try {
        this->evaluateDerivatives(currentTime, timeStep);
        this->gatherDerivativeStage(0);
        this->evaluateDiffusions(currentTime, timeStep);
        this->gatherAllDiffusionStages(0);

        for (size_t i = 1; i < s; i++) {
            this->combineDrifts(c.A0.at(i), i);
            for (size_t k = 0; k < m; k++) {
                this->combineDiffusion(k, c.B0.at(i), i);
            }
            this->buildStochasticCandidateInPlace(this->stochasticAcceptedState(),
                                                  this->combinedDerivative,
                                                  timeStep,
                                                  this->combinedDiffusion,
                                                  this->weakDW);
            this->evaluateDerivatives(currentTime + c.c0(i) * timeStep, timeStep);
            this->gatherDerivativeStage(i);

            this->combineDrifts(c.A1.at(i), i);
            for (size_t l = 0; l < m; l++) {
                this->combineDiffusion(l, c.B2.at(i), i);
            }
            for (size_t k = 0; k < m; k++) {
                this->combineDiffusion(k, c.B1.at(i), i);
                for (size_t l = 0; l < m; l++) {
                    this->pseudoSteps(static_cast<Eigen::Index>(l)) = (l == k) ? xi : Ikl(k, l);
                }
                this->buildStochasticCandidateInPlace(this->stochasticAcceptedState(),
                                                      this->combinedDerivative,
                                                      timeStep,
                                                      this->combinedDiffusion,
                                                      this->pseudoSteps);
                this->evaluateDiffusions(currentTime + c.c1(i) * timeStep, timeStep);
                this->gatherDiffusionStage(k, i);
                this->combineDiffusion(k, c.B2.at(i), i);
            }
        }

        this->combineDrifts(c.alpha, s);
        for (size_t k = 0; k < m; k++) {
            this->combineDiffusion(k, c.beta0, s);
        }
        this->buildStochasticCandidateInPlace(
          this->stochasticAcceptedState(), this->combinedDerivative, timeStep, this->combinedDiffusion, this->weakDW);
        for (size_t k = 0; k < m; k++) {
            this->combineDiffusion(k, c.beta1, s);
        }
        this->applyStochasticDiffusionUpdateInPlace(this->combinedDiffusion, this->diagonalIntegral);
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}
