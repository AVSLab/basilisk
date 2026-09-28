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
#include "svIntegratorWeakSIESME.h"

svIntegratorWeakSIESME::svIntegratorWeakSIESME(DynamicObject* dynIn,
                                               const SIESMECoefficients& coefficients)
    : StochasticRKIntegratorBase(dynIn), coefficients(coefficients)
{
}

void svIntegratorWeakSIESME::integrate(double currentTime, double timeStep)
{
    // A zero-duration step advances nothing and must not consume a noise sample.
    if (timeStep == 0) return;

    const SIESMECoefficients& c = this->coefficients;
    const size_t m = prepareStageBuffers(2, 3, 1, 3);
    captureStates(0); // Preserve the initial state while callbacks evaluate later stages.
    const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const Eigen::VectorXd& dW = sample.dW;

    const double h = timeStep;
    const double sqh = std::sqrt(h);

    // Polynomial moments of the Gaussian increment (per noise source).
    auto& W2 = this->noiseBuffers[0]; // dW^2 / sqrt(h)
    auto& W3 = this->noiseBuffers[1]; // nu2 * dW^3 / h
    for (size_t k = 0; k < m; k++) {
        const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
        W2(eigenK) = dW(eigenK) * dW(eigenK) / sqh;
        W3(eigenK) = c.nu2 * dW(eigenK) * dW(eigenK) * dW(eigenK) / h;
    }

    // --- Stage 0: k0 = f(x_n), g0 = g(x_n) ---
    evaluateStageDerivatives(currentTime, timeStep, 0);
    evaluateStageDiffusions(currentTime, timeStep, 0);

    // --- k1 stage: state = x_n + lambda0*k0*h + g0.*(nu1*dW + W3); k1 = f(state, t+mu0*h) ---
    restoreStates(0);
    applyDerivativeSum(&c.lambda0, 1);
    for (size_t k = 0; k < m; k++) {
        applyStageDiffusion(k, 0);
    }
    {
        auto& step = this->noiseBuffers[2];
        for (Eigen::Index k = 0; k < noiseCount; k++) step(k) = c.nu1 * dW(k) + W3(k);
        propagateStateWithCachedNoise(timeStep, step);
    }
    evaluateStageDerivatives(currentTime + c.mu0 * timeStep, timeStep, 1);

    // --- g1 stage: state = x_n + lambdabar0*k0*h + g0.*(beta2*sqrt(h) + beta3*W2) ---
    restoreStates(0);
    applyDerivativeSum(&c.lambdabar0, 1);
    for (size_t k = 0; k < m; k++) {
        applyStageDiffusion(k, 0);
    }
    {
        auto& step = this->noiseBuffers[2];
        for (Eigen::Index k = 0; k < noiseCount; k++) step(k) = c.beta2 * sqh + c.beta3 * W2(k);
        propagateStateWithCachedNoise(timeStep, step);
    }
    evaluateStageDiffusions(currentTime + c.mubar0 * timeStep, timeStep, 1);

    // --- g2 stage: state = x_n + lambdabar0*k0*h + g0.*(delta2*sqrt(h) + delta3*W2) ---
    restoreStates(0);
    applyDerivativeSum(&c.lambdabar0, 1);
    for (size_t k = 0; k < m; k++) {
        applyStageDiffusion(k, 0);
    }
    {
        auto& step = this->noiseBuffers[2];
        for (Eigen::Index k = 0; k < noiseCount; k++) step(k) = c.delta2 * sqh + c.delta3 * W2(k);
        propagateStateWithCachedNoise(timeStep, step);
    }
    evaluateStageDiffusions(currentTime + c.mubar0 * timeStep, timeStep, 2);

    // --- State update ---
    // x_{n+1} = x_n + (alpha1*k0 + alpha2*k1)*h
    //               + gamma1*g0.*dW
    //               + (lambda1*dW + lambda2*sqrt(h) + lambda3*W2).*g1
    //               + (mu1*dW + mu2*sqrt(h) + mu3*W2).*g2
    restoreStates(0);
    const double driftWeights[] = {c.alpha1, c.alpha2};
    applyDerivativeSum(driftWeights, 2, false);
    for (size_t k = 0; k < m; k++) {
        applyStageDiffusion(k, 0);
    }
    {
        auto& step = this->noiseBuffers[2];
        for (Eigen::Index k = 0; k < noiseCount; k++) step(k) = c.gamma1 * dW(k);
        propagateStateWithCachedNoise(timeStep, step);
    }
    for (size_t k = 0; k < m; k++) {
        applyStageDiffusion(k, 1);
    }
    {
        auto& step = this->noiseBuffers[2];
        for (Eigen::Index k = 0; k < noiseCount; k++) {
            step(k) = c.lambda1 * dW(k) + c.lambda2 * sqh + c.lambda3 * W2(k);
        }
        propagateStateWithCachedNoise(0, step);
    }
    for (size_t k = 0; k < m; k++) {
        applyStageDiffusion(k, 2);
    }
    {
        auto& step = this->noiseBuffers[2];
        for (Eigen::Index k = 0; k < noiseCount; k++) {
            step(k) = c.mu1 * dW(k) + c.mu2 * sqh + c.mu3 * W2(k);
        }
        propagateStateWithCachedNoise(0, step);
    }

    // The dynPtrs now hold x_{n+1}.
}
