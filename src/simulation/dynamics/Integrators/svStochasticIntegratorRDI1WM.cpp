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

void svStochasticIntegratorRDI1WM::integrate(double currentTime, double timeStep)
{
    if (timeStep == 0) return;

    const size_t m = prepareStageBuffers(2, 1, 1, 1);
    captureStates(0); // Preserve the initial state while callbacks evaluate later stages.

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);

    // Three-point distributed increment per noise source.
    auto& Ihat = this->noiseBuffers[0];
    for (size_t k = 0; k < m; k++) {
        const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
        Ihat(eigenK) = stochasticWeakRV::threePoint(sample.dW(eigenK), timeStep);
    }

    // Stage 0.
    evaluateStageDerivatives(currentTime, timeStep, 0);
    evaluateStageDiffusions(currentTime, timeStep, 0);

    // H02 = x_n + a021*k1*h + b021*g1*Ihat
    restoreStates(0);
    applyDerivativeSum(&a021, 1);
    for (size_t k = 0; k < m; k++) {
        applyDiffusionSum(k, &b021, 1);
    }
    propagateStateWithCachedNoise(timeStep, Ihat);
    evaluateStageDerivatives(currentTime + c02 * timeStep, timeStep, 1);

    // x_{n+1} = x_n + (alpha1*k1 + alpha2*k2)*h + beta11*g1*Ihat
    restoreStates(0);
    const double driftWeights[] = {alpha1, alpha2};
    applyDerivativeSum(driftWeights, 2, false);
    for (size_t k = 0; k < m; k++) {
        applyDiffusionSum(k, &beta11, 1);
    }
    propagateStateWithCachedNoise(timeStep, Ihat);

    // The dynPtrs now hold x_{n+1}.
}
