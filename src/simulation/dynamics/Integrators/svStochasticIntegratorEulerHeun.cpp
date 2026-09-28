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

void svStochasticIntegratorEulerHeun::integrate(double currentTime, double timeStep)
{
    // Initialization must not consume a noise sample.
    if (timeStep == 0) return;

    const size_t noiseCount = prepareStageBuffers(2, 2, 1, 0);
    captureStates(0);
    const GaussianNoiseSample sample = this->rvGenerator->generate(noiseCount, timeStep);

    // Predictor at (t_n, x_n), with owned drift and diffusion snapshots.
    evaluateStageDerivatives(currentTime, timeStep, 0);
    evaluateStageDiffusions(currentTime, timeStep, 0);
    restoreStates(0);
    applyStageDerivatives(0);
    for (size_t source = 0; source < noiseCount; ++source) applyStageDiffusion(source, 0);
    propagateStateWithCachedNoise(timeStep, sample.dW);

    // Corrector at (t_{n+1}, xBar).
    evaluateStageDerivatives(currentTime + timeStep, timeStep, 1);
    evaluateStageDiffusions(currentTime + timeStep, timeStep, 1);
    restoreStates(0);
    // Preserve (first + second) * 0.5 rather than distributing the multiplication.
    for (size_t index = 0; index < this->derivativeStages[0].size(); ++index) {
        this->derivativeStages[0][index] += this->derivativeStages[1][index];
        this->derivativeStages[0][index] *= 0.5;
    }
    applyStageDerivatives(0);
    for (size_t source = 0; source < noiseCount; ++source) {
        auto& first = this->diffusionStages[0][source];
        const auto& second = this->diffusionStages[1][source];
        for (size_t index = 0; index < first.size(); ++index) {
            first[index] += second[index];
            first[index] *= 0.5;
        }
        applyStageDiffusion(source, 0);
    }
    propagateStateWithCachedNoise(timeStep, sample.dW);
}
