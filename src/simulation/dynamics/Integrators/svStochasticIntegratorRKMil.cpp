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

#include <cmath>

void svStochasticIntegratorRKMil::integrate(double currentTime, double timeStep)
{
    // Initialization must not consume a noise sample.
    if (timeStep == 0) return;

    const size_t noiseCount = prepareStageBuffers(2, 2, 2, 2);
    captureStates(0);
    const GaussianNoiseSample sample = this->rvGenerator->generate(noiseCount, timeStep);
    const double sqh = std::sqrt(timeStep);
    auto& pseudoStep = this->noiseBuffers[0];
    auto& milStep = this->noiseBuffers[1];

    evaluateStageDerivatives(currentTime, timeStep, 0);
    evaluateStageDiffusions(currentTime, timeStep, 0); // L = g(x_n)

    // K = x_n + h*f; preserve it independently of the later support point.
    restoreStates(0);
    applyStageDerivatives(0);
    pseudoStep.setZero();
    propagateStateWithCachedNoise(timeStep, pseudoStep);
    captureStates(1);

    // uTilde = K + sqrt(h)*sum_k L_k. Multiplication by zero retains the
    // original behavior for non-finite derivatives as well as ordinary values.
    restoreStates(1);
    for (size_t index = 0; index < this->derivativeStages[0].size(); ++index)
        this->derivativeStages[1][index] = this->derivativeStages[0][index] * 0.0;
    applyStageDerivatives(1);
    for (size_t source = 0; source < noiseCount; ++source) applyStageDiffusion(source, 0);
    pseudoStep.setConstant(sqh);
    propagateStateWithCachedNoise(0.0, pseudoStep);

    // Reuse gTilde storage for ggprime = (gTilde - L)/sqrt(h).
    evaluateStageDiffusions(currentTime, timeStep, 1);
    for (size_t source = 0; source < noiseCount; ++source) {
        auto& correction = this->diffusionStages[1][source];
        const auto& initial = this->diffusionStages[0][source];
        for (size_t index = 0; index < correction.size(); ++index) {
            correction[index] -= initial[index];
            correction[index] *= 1.0 / sqh;
        }
    }
    for (Eigen::Index source = 0; source < milStep.size(); ++source)
        milStep(source) = (sample.dW(source) * sample.dW(source) - timeStep) / 2.0;

    // Preserve the two sequential virtual propagations, including on manifolds.
    restoreStates(1);
    applyStageDerivatives(1);
    for (size_t source = 0; source < noiseCount; ++source) applyStageDiffusion(source, 0);
    propagateStateWithCachedNoise(0.0, sample.dW);
    for (size_t source = 0; source < noiseCount; ++source) applyStageDiffusion(source, 1);
    propagateStateWithCachedNoise(0.0, milStep);
}
