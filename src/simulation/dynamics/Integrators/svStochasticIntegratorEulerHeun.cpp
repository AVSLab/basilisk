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
    // A zero-duration step advances nothing and must not consume a noise sample.
    // (Basilisk issues an integrate() call with timeStep == 0 at initialization.)
    if (timeStep == 0) return;

    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);

    const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps = noiseIndexMaps();
    const size_t m = stateIdToNoiseIndexMaps.size();

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const Eigen::VectorXd& dW = sample.dW;

    // --- Predictor at (t_n, x_n): f1, g1 ---
    ExtendedStateVector f1 = computeDerivatives(currentTime, timeStep);
    std::vector<ExtendedStateVector> g1 =
        computeDiffusions(currentTime, timeStep, stateIdToNoiseIndexMaps);

    // xBar = x_n + h * f1 + sum_k g1_k * dW_k
    currentState.setStates(dynPtrs);
    f1.setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        g1.at(k).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateState(timeStep, dW, stateIdToNoiseIndexMaps);

    // --- Corrector evaluations at (t_{n+1}, xBar): f2, g2 ---
    ExtendedStateVector f2 = computeDerivatives(currentTime + timeStep, timeStep);
    std::vector<ExtendedStateVector> g2 =
        computeDiffusions(currentTime + timeStep, timeStep, stateIdToNoiseIndexMaps);

    // x_{n+1} = x_n + (h/2)(f1+f2) + sum_k (dW_k/2)(g1_k+g2_k)
    currentState.setStates(dynPtrs);
    ((f1 += f2) * 0.5).setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        ((g1.at(k) += g2.at(k)) * 0.5).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateState(timeStep, dW, stateIdToNoiseIndexMaps);

    // The dynPtrs now hold x_{n+1}.
}
