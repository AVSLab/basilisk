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
    // A zero-duration step advances nothing and must not consume a noise sample.
    // (Basilisk issues an integrate() call with timeStep == 0 at initialization.)
    if (timeStep == 0) return;

    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);

    const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps = noiseIndexMaps();
    const size_t m = stateIdToNoiseIndexMaps.size();

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const Eigen::VectorXd& dW = sample.dW;

    const double h = timeStep;
    const double sqh = std::sqrt(h);

    // --- Evaluate f and g at x_n ---
    ExtendedStateVector f = computeDerivatives(currentTime, timeStep);
    std::vector<ExtendedStateVector> L =
        computeDiffusions(currentTime, timeStep, stateIdToNoiseIndexMaps);

    // --- K = x_n + h * f (drift-only Euler predictor) ---
    currentState.setStates(dynPtrs);
    f.setDerivatives(dynPtrs);
    // Zero pseudo-time steps: K carries no noise contribution.
    propagateState(timeStep, Eigen::VectorXd::Zero(m), stateIdToNoiseIndexMaps);
    const ExtendedStateVector K = ExtendedStateVector::fromStates(dynPtrs);

    // --- uTilde = K + sqrt(h) * sum_k L_k  (support point for the finite difference) ---
    // (drift is not re-applied here; timeStep passed to propagateState multiplies the
    // derivative, so we set the derivative to zero and drive purely with the noise term.)
    K.setStates(dynPtrs);
    ExtendedStateVector zeroDeriv = f * 0.0;
    zeroDeriv.setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        L.at(k).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateState(0.0, sqh * Eigen::VectorXd::Ones(m), stateIdToNoiseIndexMaps);

    // --- gTilde_k = g_k(uTilde);  ggprime_k = (gTilde_k - L_k) / sqrt(h) ---
    std::vector<ExtendedStateVector> gTilde =
        computeDiffusions(currentTime, timeStep, stateIdToNoiseIndexMaps);
    std::vector<ExtendedStateVector> ggprime;
    ggprime.reserve(m);
    for (size_t k = 0; k < m; k++) {
        ggprime.push_back((gTilde.at(k) - L.at(k)) * (1.0 / sqh));
    }

    // --- x_{n+1} = K + sum_k L_k dW_k + sum_k ggprime_k (dW_k^2 - h)/2 ---
    // Milstein pseudo-time step for the ggprime term.
    Eigen::VectorXd milStep(m);
    for (size_t k = 0; k < m; k++) {
        milStep(k) = (dW(k) * dW(k) - h) / 2.0;
    }

    // Start from K, add the L*dW term (drift set to zero so it is not double-counted).
    K.setStates(dynPtrs);
    zeroDeriv.setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        L.at(k).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateState(0.0, dW, stateIdToNoiseIndexMaps);

    // Add the Milstein correction term.
    for (size_t k = 0; k < m; k++) {
        ggprime.at(k).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateState(0.0, milStep, stateIdToNoiseIndexMaps);

    // The dynPtrs now hold x_{n+1}.
}
