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

#include <cmath>

svStochasticIntegratorW2Ito::svStochasticIntegratorW2Ito(DynamicObject* dyn,
                                                         const W2ItoCoefficients& coefficients)
    : StochasticRKIntegratorBase(dyn), coefficients(coefficients)
{
}

ExtendedStateVector svStochasticIntegratorW2Ito::scaledSum(
    const std::vector<double>& factors, const std::vector<ExtendedStateVector>& vectors,
    size_t length)
{
    ExtendedStateVector result = vectors.at(0) * factors.at(0);
    for (size_t i = 1; i < length; i++) {
        if (factors.at(i) == 0.0) continue;
        result += vectors.at(i) * factors.at(i);
    }
    return result;
}

void svStochasticIntegratorW2Ito::integrate(double currentTime, double timeStep)
{
    if (timeStep == 0) return;

    const W2ItoCoefficients& c = this->coefficients;
    const size_t s = c.numStages();
    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);
    const std::vector<StateIdToIndexMap>& maps = noiseIndexMaps();
    const size_t m = maps.size();

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
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
    const double eta1 = (m > 0) ? stochasticWeakRV::twoPoint(sample.dZ(0), 1.0) : 1.0;
    const double eta2 = (m > 1) ? stochasticWeakRV::twoPoint(sample.dZ(1), 1.0) : 0.0;
    const double xi = sqh * eta1;
    Eigen::VectorXd _dW(m), Ikk(m);
    for (size_t k = 0; k < m; k++) {
        _dW(k) = stochasticWeakRV::threePoint(sample.dW(k), h);
        Ikk(k) = (_dW(k) * _dW(k) / xi - xi) / 2.0;
    }
    auto Ikl = [&](size_t k, size_t l) -> double {
        if (k < l) return 0.5 * (_dW(l) - eta2 * _dW(l));
        return 0.5 * (_dW(l) + eta2 * _dW(l)); // k > l
    };

    // Stage function evaluations. f_H0[i] = f(H_i^(0)); g_Hk[k][i] = g_k(H_i^(k)).
    std::vector<ExtendedStateVector> f_H0(s);
    std::vector<std::vector<ExtendedStateVector>> g_Hk(m, std::vector<ExtendedStateVector>(s));

    // Stage 0: H_0^(0) == H_0^(k) == x_n, so evaluate f and every g at the current state.
    f_H0.at(0) = computeDerivatives(currentTime, timeStep);
    {
        std::vector<ExtendedStateVector> diffs = computeDiffusions(currentTime, timeStep, maps);
        for (size_t k = 0; k < m; k++) g_Hk.at(k).at(0) = diffs.at(k);
    }

    // Stages i = 1..s-1.
    for (size_t i = 1; i < s; i++) {
        // H_i^(0) = x_n + h*sum_{j<i} A0[i][j] f(H0[j]) + sum_k Ihat_k*sum_{j<i} B0[i][j] g_k(Hk[j])
        currentState.setStates(dynPtrs);
        scaledSum(c.A0.at(i), f_H0, i).setDerivatives(dynPtrs);
        for (size_t k = 0; k < m; k++) {
            scaledSum(c.B0.at(i), g_Hk.at(k), i).setDiffusions(dynPtrs, maps.at(k));
        }
        propagateState(timeStep, _dW, maps);
        f_H0.at(i) = computeDerivatives(currentTime + c.c0(i) * timeStep, timeStep);

        // H_i^(k) = x_n + h*sum_{j<i} A1[i][j] f(H0[j]) + xi*sum_{j<i} B1[i][j] g_k(Hk[j])
        //               + sum_{l!=k} Ihat_(k,l)*sum_{j<i} B2[i][j] g_l(Hl[j])
        for (size_t k = 0; k < m; k++) {
            currentState.setStates(dynPtrs);
            scaledSum(c.A1.at(i), f_H0, i).setDerivatives(dynPtrs);
            scaledSum(c.B1.at(i), g_Hk.at(k), i).setDiffusions(dynPtrs, maps.at(k));
            for (size_t l = 0; l < m; l++) {
                if (l == k) continue;
                scaledSum(c.B2.at(i), g_Hk.at(l), i).setDiffusions(dynPtrs, maps.at(l));
            }
            Eigen::VectorXd step(m);
            for (size_t l = 0; l < m; l++) step(l) = (l == k) ? xi : Ikl(k, l);
            propagateState(timeStep, step, maps);
            g_Hk.at(k).at(i) = computeDiffusion(currentTime + c.c1(i) * timeStep, timeStep, maps.at(k));
        }
    }

    // State update (paper eq. 3.1):
    //   y_{n+1} = x_n + h*sum_i alpha[i] f(H0[i])
    //                 + sum_k Ihat_k    * sum_i beta0[i] g_k(Hk[i])
    //                 + sum_k Ihat_(k,k)* sum_i beta1[i] g_k(Hk[i])
    currentState.setStates(dynPtrs);
    scaledSum(c.alpha, f_H0, s).setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        scaledSum(c.beta0, g_Hk.at(k), s).setDiffusions(dynPtrs, maps.at(k));
    }
    propagateState(timeStep, _dW, maps);
    for (size_t k = 0; k < m; k++) {
        scaledSum(c.beta1, g_Hk.at(k), s).setDiffusions(dynPtrs, maps.at(k));
    }
    propagateState(0, Ikk, maps);

    // The dynPtrs now hold x_{n+1}.
}
