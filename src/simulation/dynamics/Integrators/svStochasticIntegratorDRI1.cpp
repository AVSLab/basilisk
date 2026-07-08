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

svStochasticIntegratorDRI1::svStochasticIntegratorDRI1(DynamicObject* dyn)
    : StochasticRKIntegratorBase(dyn), coefficients(svStochasticIntegratorDRI1::getCoefficients())
{
}

svStochasticIntegratorDRI1::svStochasticIntegratorDRI1(DynamicObject* dyn,
                                                       const DRI1Coefficients& coefficients)
    : StochasticRKIntegratorBase(dyn), coefficients(coefficients)
{
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

void svStochasticIntegratorDRI1::integrate(double currentTime, double timeStep)
{
    if (timeStep == 0) return;

    const DRI1Coefficients& c = this->coefficients;
    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);
    const std::vector<StateIdToIndexMap>& maps = noiseIndexMaps();
    const size_t m = maps.size();

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const double h = timeStep;
    const double sqh = std::sqrt(h);

    // Discrete random variables (deterministic functions of the Gaussian dW/dZ):
    //   _dW : three-point in {-sqrt(3h), 0, +sqrt(3h)}
    //   chi1: (_dW^2 - h)/2      (diagonal of Ihat2)
    //   _dZ : two-point in {-sqrt(h), +sqrt(h)}  (only used for cross-noise, m>1)
    Eigen::VectorXd _dW(m), chi1(m), _dZ(m);
    for (size_t k = 0; k < m; k++) {
        _dW(k) = stochasticWeakRV::threePoint(sample.dW(k), h);
        chi1(k) = (_dW(k) * _dW(k) - h) / 2.0;
        _dZ(k) = stochasticWeakRV::twoPoint(sample.dZ(k), sqh);
    }

    // ---- Drift stages (shared across noise sources) ----
    // k1 = f(x_n); g1 = g(x_n)
    ExtendedStateVector k1 = computeDerivatives(currentTime, timeStep);
    std::vector<ExtendedStateVector> g1 = computeDiffusions(currentTime, timeStep, maps);

    // H02 = x_n + a021*k1*h + b021*g1*_dW ; k2 = f(H02, t+c02*h)
    currentState.setStates(dynPtrs);
    (k1 * c.a021).setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) (g1.at(k) * c.b021).setDiffusions(dynPtrs, maps.at(k));
    propagateState(timeStep, _dW, maps);
    ExtendedStateVector k2 = computeDerivatives(currentTime + c.c02 * timeStep, timeStep);

    // H03 = x_n + (a031*k1 + a032*k2)*h + b031*g1*_dW ; k3 = f(H03, t+c03*h)
    currentState.setStates(dynPtrs);
    {
        ExtendedStateVector d = k1 * c.a031;
        d += k2 * c.a032;
        d.setDerivatives(dynPtrs);
    }
    for (size_t k = 0; k < m; k++) (g1.at(k) * c.b031).setDiffusions(dynPtrs, maps.at(k));
    propagateState(timeStep, _dW, maps);
    ExtendedStateVector k3 = computeDerivatives(currentTime + c.c03 * timeStep, timeStep);

    // ---- Diffusion stages, per noise source k ----
    // H12[k] = x_n + a121*k1*h + b121*g1[k]*sqrt(h)*e_k
    // H13[k] = x_n + a131*k1*h + b131*g1[k]*sqrt(h)*e_k
    // g2[k] = g(H12[k]); g3[k] = g(H13[k])   (only component k used)
    std::vector<ExtendedStateVector> g2(m), g3(m);
    for (size_t k = 0; k < m; k++) {
        Eigen::VectorXd stepK = Eigen::VectorXd::Zero(m);

        currentState.setStates(dynPtrs);
        (k1 * c.a121).setDerivatives(dynPtrs);
        g1.at(k).setDiffusions(dynPtrs, maps.at(k));
        stepK(k) = c.b121 * sqh;
        propagateState(timeStep, stepK, maps);
        g2.at(k) = computeDiffusion(currentTime + c.c12 * timeStep, timeStep, maps.at(k));

        currentState.setStates(dynPtrs);
        (k1 * c.a131).setDerivatives(dynPtrs);
        g1.at(k).setDiffusions(dynPtrs, maps.at(k));
        stepK(k) = c.b131 * sqh;
        propagateState(timeStep, stepK, maps);
        g3.at(k) = computeDiffusion(currentTime + c.c13 * timeStep, timeStep, maps.at(k));
    }

    // ---- Cross-noise stages (only needed when m > 1) ----
    // Hhat2[l] = x_n + sqrt(h)*(b221*g1[l] + b222*g2[l] + b223*g3[l]) e_l
    // Hhat3[l] = x_n + sqrt(h)*(b231*g1[l] + b232*g2[l] + b233*g3[l]) e_l
    // We must keep g evaluated at these states for ALL noise sources (so we can read
    // the k-th component later), so gHat2Full[l] is the full per-source diffusion set
    // g(Hhat2[l]); gHat2Full[l][k] is the diffusion of source k at state Hhat2[l].
    std::vector<std::vector<ExtendedStateVector>> gHat2Full(m), gHat3Full(m);
    const bool doCrossNoise = (m > 1) && !this->nonMixing;
    if (doCrossNoise) {
        for (size_t l = 0; l < m; l++) {
            Eigen::VectorXd stepL = Eigen::VectorXd::Zero(m);

            // Hhat2[l]
            currentState.setStates(dynPtrs);
            {
                ExtendedStateVector d = g1.at(l) * c.b221;
                d += g2.at(l) * c.b222;
                d += g3.at(l) * c.b223;
                d.setDiffusions(dynPtrs, maps.at(l));
            }
            stepL(l) = sqh;
            propagateState(0, stepL, maps);
            gHat2Full.at(l) = computeDiffusions(currentTime, timeStep, maps);

            // Hhat3[l]
            currentState.setStates(dynPtrs);
            {
                ExtendedStateVector d = g1.at(l) * c.b231;
                d += g2.at(l) * c.b232;
                d += g3.at(l) * c.b233;
                d.setDiffusions(dynPtrs, maps.at(l));
            }
            stepL(l) = sqh;
            propagateState(0, stepL, maps);
            gHat3Full.at(l) = computeDiffusions(currentTime, timeStep, maps);
        }
    }

    // ---- State update ----
    // Drift: x_n + (alpha1*k1 + alpha2*k2 + alpha3*k3)*h
    currentState.setStates(dynPtrs);
    {
        ExtendedStateVector d = k1 * c.alpha1;
        d += k2 * c.alpha2;
        d += k3 * c.alpha3;
        d.setDerivatives(dynPtrs);
    }
    // Noise line 1: beta11 * g1 * _dW  (plus the (m-1)*beta31 g1 self term that
    // accompanies the cross-noise contribution; omitted in the non-mixing variant).
    for (size_t k = 0; k < m; k++) {
        double self31 = doCrossNoise ? (double)(m - 1) * c.beta31 : 0.0;
        (g1.at(k) * (c.beta11 + self31)).setDiffusions(dynPtrs, maps.at(k));
    }
    propagateState(timeStep, _dW, maps);

    // Noise from g2/g3: (_dW*beta12 + chi1*beta22/sqrt(h)) g2 + (_dW*beta13 + chi1*beta23/sqrt(h)) g3
    for (size_t k = 0; k < m; k++) g2.at(k).setDiffusions(dynPtrs, maps.at(k));
    {
        Eigen::VectorXd step(m);
        for (size_t k = 0; k < m; k++) step(k) = _dW(k) * c.beta12 + chi1(k) * c.beta22 / sqh;
        propagateState(0, step, maps);
    }
    for (size_t k = 0; k < m; k++) g3.at(k).setDiffusions(dynPtrs, maps.at(k));
    {
        Eigen::VectorXd step(m);
        for (size_t k = 0; k < m; k++) step(k) = _dW(k) * c.beta13 + chi1(k) * c.beta23 / sqh;
        propagateState(0, step, maps);
    }

    // Cross-noise contribution (m > 1 only):
    //   for each k, sum over l != k of
    //     g(Hhat2[l])[k] * (_dW[k]*beta32 + ihat2(k,l)*beta42/sqrt(h))
    //   + g(Hhat3[l])[k] * (_dW[k]*beta33 + ihat2(k,l)*beta43/sqrt(h))
    // where ihat2(k,l) = (_dW[k]*_dW[l] - sqrt(h)*_dZ[k])/2      if k < l
    //                    (_dW[k]*_dW[l] + sqrt(h)*_dZ[l])/2      if l < k
    if (doCrossNoise) {
        auto ihat2 = [&](size_t k, size_t l) -> double {
            if (k < l) return (_dW(k) * _dW(l) - sqh * _dZ(k)) / 2.0;
            return (_dW(k) * _dW(l) + sqh * _dZ(l)) / 2.0; // l < k
        };
        // For every ordered pair (k, l) with l != k, the update adds to state k:
        //   g_k(Hhat2[l]) * (_dW[k]*beta32 + ihat2(k,l)*beta42/sqrt(h))
        // + g_k(Hhat3[l]) * (_dW[k]*beta33 + ihat2(k,l)*beta43/sqrt(h))
        // where g_k(state) is source k's diffusion evaluated at that stage state.
        // We realise each such scalar contribution by setting source k's diffusion to
        // the stored value and propagating with a pseudo-step selecting source k.
        for (size_t l = 0; l < m; l++) {
            for (size_t k = 0; k < m; k++) {
                if (k == l) continue;
                const double w2 = _dW(k) * c.beta32 + ihat2(k, l) * c.beta42 / sqh;
                const double w3 = _dW(k) * c.beta33 + ihat2(k, l) * c.beta43 / sqh;

                gHat2Full.at(l).at(k).setDiffusions(dynPtrs, maps.at(k));
                {
                    Eigen::VectorXd step = Eigen::VectorXd::Zero(m);
                    step(k) = w2;
                    propagateState(0, step, maps);
                }
                gHat3Full.at(l).at(k).setDiffusions(dynPtrs, maps.at(k));
                {
                    Eigen::VectorXd step = Eigen::VectorXd::Zero(m);
                    step(k) = w3;
                    propagateState(0, step, maps);
                }
            }
        }
    }

    // The dynPtrs now hold x_{n+1}.
}
