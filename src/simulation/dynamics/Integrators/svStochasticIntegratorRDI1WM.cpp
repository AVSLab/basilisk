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

    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);
    const std::vector<StateIdToIndexMap>& maps = noiseIndexMaps();
    const size_t m = maps.size();

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);

    // Three-point distributed increment per noise source.
    Eigen::VectorXd Ihat(m);
    for (size_t k = 0; k < m; k++) {
        Ihat(k) = stochasticWeakRV::threePoint(sample.dW(k), timeStep);
    }

    // Stage 0.
    ExtendedStateVector k1 = computeDerivatives(currentTime, timeStep);
    std::vector<ExtendedStateVector> g1 = computeDiffusions(currentTime, timeStep, maps);

    // H02 = x_n + a021*k1*h + b021*g1*Ihat
    currentState.setStates(dynPtrs);
    (k1 * a021).setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        (g1.at(k) * b021).setDiffusions(dynPtrs, maps.at(k));
    }
    propagateState(timeStep, Ihat, maps);
    ExtendedStateVector k2 = computeDerivatives(currentTime + c02 * timeStep, timeStep);

    // x_{n+1} = x_n + (alpha1*k1 + alpha2*k2)*h + beta11*g1*Ihat
    currentState.setStates(dynPtrs);
    ExtendedStateVector drift = k1 * alpha1;
    drift += k2 * alpha2;
    drift.setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        (g1.at(k) * beta11).setDiffusions(dynPtrs, maps.at(k));
    }
    propagateState(timeStep, Ihat, maps);

    // The dynPtrs now hold x_{n+1}.
}
