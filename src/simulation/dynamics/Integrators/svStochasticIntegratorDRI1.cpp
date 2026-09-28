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
    const size_t m = prepareStageBuffers(3, 3, 1, 4, this->nonMixing ? 0 : 2);
    captureStates(0); // Preserve the initial state while callbacks evaluate later stages.
    const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const double h = timeStep;
    const double sqh = std::sqrt(h);

    // Discrete random variables (deterministic functions of the Gaussian dW/dZ):
    //   _dW : three-point in {-sqrt(3h), 0, +sqrt(3h)}
    //   chi1: (_dW^2 - h)/2      (diagonal of Ihat2)
    //   _dZ : two-point in {-sqrt(h), +sqrt(h)}  (only used for cross-noise, m>1)
    auto& _dW = this->noiseBuffers[0];
    auto& chi1 = this->noiseBuffers[1];
    auto& _dZ = this->noiseBuffers[2];
    for (size_t k = 0; k < m; k++) {
        const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
        _dW(eigenK) = stochasticWeakRV::threePoint(sample.dW(eigenK), h);
        chi1(eigenK) = (_dW(eigenK) * _dW(eigenK) - h) / 2.0;
        _dZ(eigenK) = stochasticWeakRV::twoPoint(sample.dZ(eigenK), sqh);
    }

    // ---- Drift stages (shared across noise sources) ----
    // k1 = f(x_n); g1 = g(x_n)
    evaluateStageDerivatives(currentTime, timeStep, 0);
    evaluateStageDiffusions(currentTime, timeStep, 0);

    // H02 = x_n + a021*k1*h + b021*g1*_dW ; k2 = f(H02, t+c02*h)
    restoreStates(0);
    applyDerivativeSum(&c.a021, 1);
    for (size_t k = 0; k < m; k++) applyDiffusionSum(k, &c.b021, 1);
    propagateStateWithCachedNoise(timeStep, _dW);
    evaluateStageDerivatives(currentTime + c.c02 * timeStep, timeStep, 1);

    // H03 = x_n + (a031*k1 + a032*k2)*h + b031*g1*_dW ; k3 = f(H03, t+c03*h)
    restoreStates(0);
    {
        const double weights[] = {c.a031, c.a032};
        applyDerivativeSum(weights, 2, false);
    }
    for (size_t k = 0; k < m; k++) applyDiffusionSum(k, &c.b031, 1);
    propagateStateWithCachedNoise(timeStep, _dW);
    evaluateStageDerivatives(currentTime + c.c03 * timeStep, timeStep, 2);

    // ---- Diffusion stages, per noise source k ----
    // H12[k] = x_n + a121*k1*h + b121*g1[k]*sqrt(h)*e_k
    // H13[k] = x_n + a131*k1*h + b131*g1[k]*sqrt(h)*e_k
    // g2[k] = g(H12[k]); g3[k] = g(H13[k])   (only component k used)
    for (size_t k = 0; k < m; k++) {
        auto& stepK = this->noiseBuffers[3];
        stepK.setZero();
        const Eigen::Index eigenK = static_cast<Eigen::Index>(k);

        restoreStates(0);
        applyDerivativeSum(&c.a121, 1);
        applyStageDiffusion(k, 0);
        stepK(eigenK) = c.b121 * sqh;
        propagateStateWithCachedNoise(timeStep, stepK);
        evaluateStageDiffusion(currentTime + c.c12 * timeStep, timeStep, k, 1);

        restoreStates(0);
        applyDerivativeSum(&c.a131, 1);
        applyStageDiffusion(k, 0);
        stepK(eigenK) = c.b131 * sqh;
        propagateStateWithCachedNoise(timeStep, stepK);
        evaluateStageDiffusion(currentTime + c.c13 * timeStep, timeStep, k, 2);
    }

    // ---- Cross-noise stages (only needed when m > 1) ----
    // Hhat2[l] = x_n + sqrt(h)*(b221*g1[l] + b222*g2[l] + b223*g3[l]) e_l
    // Hhat3[l] = x_n + sqrt(h)*(b231*g1[l] + b232*g2[l] + b233*g3[l]) e_l
    // We must keep g evaluated at these states for ALL noise sources (so we can read
    // the k-th component later), so gHat2Full[l] is the full per-source diffusion set
    // g(Hhat2[l]); gHat2Full[l][k] is the diffusion of source k at state Hhat2[l].
    // Buffer stages 3 + 2*l and 4 + 2*l own these full diffusion snapshots.
    const bool doCrossNoise = (m > 1) && !this->nonMixing;
    if (doCrossNoise) {
        for (size_t l = 0; l < m; l++) {
            auto& stepL = this->noiseBuffers[3];
            stepL.setZero();
            const Eigen::Index eigenL = static_cast<Eigen::Index>(l);

            // Hhat2[l]
            restoreStates(0);
            {
                const double weights[] = {c.b221, c.b222, c.b223};
                applyDiffusionSum(l, weights, 3, 0, false);
            }
            stepL(eigenL) = sqh;
            propagateStateWithCachedNoise(0, stepL);
            evaluateStageDiffusions(currentTime, timeStep, 3 + 2 * l);

            // Hhat3[l]
            restoreStates(0);
            {
                const double weights[] = {c.b231, c.b232, c.b233};
                applyDiffusionSum(l, weights, 3, 0, false);
            }
            stepL(eigenL) = sqh;
            propagateStateWithCachedNoise(0, stepL);
            evaluateStageDiffusions(currentTime, timeStep, 4 + 2 * l);
        }
    }

    // ---- State update ----
    // Drift: x_n + (alpha1*k1 + alpha2*k2 + alpha3*k3)*h
    restoreStates(0);
    {
        const double weights[] = {c.alpha1, c.alpha2, c.alpha3};
        applyDerivativeSum(weights, 3, false);
    }
    // Noise line 1: beta11 * g1 * _dW  (plus the (m-1)*beta31 g1 self term that
    // accompanies the cross-noise contribution; omitted in the non-mixing variant).
    for (size_t k = 0; k < m; k++) {
        double self31 = doCrossNoise ? (double)(m - 1) * c.beta31 : 0.0;
        const double weight = c.beta11 + self31;
        applyDiffusionSum(k, &weight, 1);
    }
    propagateStateWithCachedNoise(timeStep, _dW);

    // Noise from g2/g3: (_dW*beta12 + chi1*beta22/sqrt(h)) g2 + (_dW*beta13 + chi1*beta23/sqrt(h)) g3
    for (size_t k = 0; k < m; k++) applyStageDiffusion(k, 1);
    {
        auto& step = this->noiseBuffers[3];
        for (Eigen::Index k = 0; k < noiseCount; k++) {
            step(k) = _dW(k) * c.beta12 + chi1(k) * c.beta22 / sqh;
        }
        propagateStateWithCachedNoise(0, step);
    }
    for (size_t k = 0; k < m; k++) applyStageDiffusion(k, 2);
    {
        auto& step = this->noiseBuffers[3];
        for (Eigen::Index k = 0; k < noiseCount; k++) {
            step(k) = _dW(k) * c.beta13 + chi1(k) * c.beta23 / sqh;
        }
        propagateStateWithCachedNoise(0, step);
    }

    // Cross-noise contribution (m > 1 only):
    //   for each k, sum over l != k of
    //     g(Hhat2[l])[k] * (_dW[k]*beta32 + ihat2(k,l)*beta42/sqrt(h))
    //   + g(Hhat3[l])[k] * (_dW[k]*beta33 + ihat2(k,l)*beta43/sqrt(h))
    // where ihat2(k,l) = (_dW[k]*_dW[l] - sqrt(h)*_dZ[k])/2      if k < l
    //                    (_dW[k]*_dW[l] + sqrt(h)*_dZ[l])/2      if l < k
    if (doCrossNoise) {
        auto ihat2 = [&](size_t k, size_t l) -> double {
            const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
            const Eigen::Index eigenL = static_cast<Eigen::Index>(l);
            if (k < l) return (_dW(eigenK) * _dW(eigenL) - sqh * _dZ(eigenK)) / 2.0;
            return (_dW(eigenK) * _dW(eigenL) + sqh * _dZ(eigenL)) / 2.0; // l < k
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
                const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
                const double w2 = _dW(eigenK) * c.beta32 + ihat2(k, l) * c.beta42 / sqh;
                const double w3 = _dW(eigenK) * c.beta33 + ihat2(k, l) * c.beta43 / sqh;

                applyStageDiffusion(k, 3 + 2 * l);
                {
                    auto& step = this->noiseBuffers[3];
                    step.setZero();
                    step(eigenK) = w2;
                    propagateStateWithCachedNoise(0, step);
                }
                applyStageDiffusion(k, 4 + 2 * l);
                {
                    auto& step = this->noiseBuffers[3];
                    step.setZero();
                    step(eigenK) = w3;
                    propagateStateWithCachedNoise(0, step);
                }
            }
        }
    }

    // The dynPtrs now hold x_{n+1}.
}
