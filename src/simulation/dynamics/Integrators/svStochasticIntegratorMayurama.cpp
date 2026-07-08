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
#include "svStochasticIntegratorMayurama.h"

void svStochasticIntegratorMayurama::integrate(double currentTime, double timeStep)
{
    // A zero-duration step advances nothing and must not consume a noise sample.
    // (Basilisk issues an integrate() call with timeStep == 0 at initialization.)
    if (timeStep == 0) return;

    const std::vector<StateIdToIndexMap>& maps = noiseIndexMaps();
    const size_t m = maps.size();

    // Euler-Mayurama uses only the Wiener increment dW as the per-source pseudo-step.
    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);

    // f and g are evaluated at the current state x_n (the dynPtrs already hold it), then
    // x_{n+1} = x_n + f*h + sum_k g_k * dW_k.
    computeDerivatives(currentTime, timeStep).setDerivatives(dynPtrs);
    std::vector<ExtendedStateVector> g = computeDiffusions(currentTime, timeStep, maps);
    for (size_t k = 0; k < m; k++) {
        g.at(k).setDiffusions(dynPtrs, maps.at(k));
    }
    propagateState(timeStep, sample.dW, maps);

    // The dynPtrs now hold x_{n+1}.
}
