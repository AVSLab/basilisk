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
    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);

    const std::vector<StateIdToIndexMap>& maps = noiseIndexMaps();
    const size_t m = maps.size();

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const Eigen::VectorXd& dW = sample.dW;

    const double h = timeStep;
    const double sqh = std::sqrt(h);

    // Polynomial moments of the Gaussian increment (per noise source).
    Eigen::VectorXd W2(m); // dW^2 / sqrt(h)
    Eigen::VectorXd W3(m); // nu2 * dW^3 / h
    for (size_t k = 0; k < m; k++) {
        W2(k) = dW(k) * dW(k) / sqh;
        W3(k) = c.nu2 * dW(k) * dW(k) * dW(k) / h;
    }

    // --- Stage 0: k0 = f(x_n), g0 = g(x_n) ---
    ExtendedStateVector k0 = computeDerivatives(currentTime, timeStep);
    std::vector<ExtendedStateVector> g0 = computeDiffusions(currentTime, timeStep, maps);

    // --- k1 stage: state = x_n + lambda0*k0*h + g0.*(nu1*dW + W3); k1 = f(state, t+mu0*h) ---
    currentState.setStates(dynPtrs);
    (k0 * c.lambda0).setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        g0.at(k).setDiffusions(dynPtrs, maps.at(k));
    }
    {
        Eigen::VectorXd step(m);
        for (size_t k = 0; k < m; k++) step(k) = c.nu1 * dW(k) + W3(k);
        propagateState(timeStep, step, maps);
    }
    ExtendedStateVector k1 = computeDerivatives(currentTime + c.mu0 * timeStep, timeStep);

    // --- g1 stage: state = x_n + lambdabar0*k0*h + g0.*(beta2*sqrt(h) + beta3*W2) ---
    currentState.setStates(dynPtrs);
    (k0 * c.lambdabar0).setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        g0.at(k).setDiffusions(dynPtrs, maps.at(k));
    }
    {
        Eigen::VectorXd step(m);
        for (size_t k = 0; k < m; k++) step(k) = c.beta2 * sqh + c.beta3 * W2(k);
        propagateState(timeStep, step, maps);
    }
    std::vector<ExtendedStateVector> g1 =
        computeDiffusions(currentTime + c.mubar0 * timeStep, timeStep, maps);

    // --- g2 stage: state = x_n + lambdabar0*k0*h + g0.*(delta2*sqrt(h) + delta3*W2) ---
    currentState.setStates(dynPtrs);
    (k0 * c.lambdabar0).setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        g0.at(k).setDiffusions(dynPtrs, maps.at(k));
    }
    {
        Eigen::VectorXd step(m);
        for (size_t k = 0; k < m; k++) step(k) = c.delta2 * sqh + c.delta3 * W2(k);
        propagateState(timeStep, step, maps);
    }
    std::vector<ExtendedStateVector> g2 =
        computeDiffusions(currentTime + c.mubar0 * timeStep, timeStep, maps);

    // --- State update ---
    // x_{n+1} = x_n + (alpha1*k0 + alpha2*k1)*h
    //               + gamma1*g0.*dW
    //               + (lambda1*dW + lambda2*sqrt(h) + lambda3*W2).*g1
    //               + (mu1*dW + mu2*sqrt(h) + mu3*W2).*g2
    currentState.setStates(dynPtrs);
    ExtendedStateVector drift = k0 * c.alpha1;
    drift += k1 * c.alpha2;
    drift.setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        g0.at(k).setDiffusions(dynPtrs, maps.at(k));
    }
    {
        Eigen::VectorXd step(m);
        for (size_t k = 0; k < m; k++) step(k) = c.gamma1 * dW(k);
        propagateState(timeStep, step, maps);
    }
    for (size_t k = 0; k < m; k++) {
        g1.at(k).setDiffusions(dynPtrs, maps.at(k));
    }
    {
        Eigen::VectorXd step(m);
        for (size_t k = 0; k < m; k++) step(k) = c.lambda1 * dW(k) + c.lambda2 * sqh + c.lambda3 * W2(k);
        propagateState(0, step, maps);
    }
    for (size_t k = 0; k < m; k++) {
        g2.at(k).setDiffusions(dynPtrs, maps.at(k));
    }
    {
        Eigen::VectorXd step(m);
        for (size_t k = 0; k < m; k++) step(k) = c.mu1 * dW(k) + c.mu2 * sqh + c.mu3 * W2(k);
        propagateState(0, step, maps);
    }

    // The dynPtrs now hold x_{n+1}.
}
