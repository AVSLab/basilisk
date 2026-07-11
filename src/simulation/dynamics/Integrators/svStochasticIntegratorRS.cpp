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
#include "svStochasticIntegratorRS.h"
#include "../_GeneralModuleFiles/stochasticWeakRandomVariables.h"

#include <cmath>

svStochasticIntegratorRS::svStochasticIntegratorRS(DynamicObject* dynIn,
                                                   const RSCoefficients& coefficients)
    : StochasticRKIntegratorBase(dynIn), coefficients(coefficients)
{
}

// ---- RS1 / RS2 coefficients (RS1 / RS2 tableaux) ----

svStochasticIntegratorRS1::svStochasticIntegratorRS1(DynamicObject* dyn)
    : svStochasticIntegratorRS(dyn, svStochasticIntegratorRS1::getCoefficients())
{}

RSCoefficients svStochasticIntegratorRS1::getCoefficients()
{
    RSCoefficients c;
    c.a021 = 0.0; c.a031 = 1.0; c.a032 = 0.0;
    c.a131 = 1.0; c.a141 = 1.0;
    c.b031 = 1.0 / 4.0; c.b032 = 3.0 / 4.0;
    c.b121 = 2.0 / 3.0;
    c.b131 = 1.0 / 12.0; c.b132 = 1.0 / 4.0;
    c.b141 = -5.0 / 4.0; c.b142 = 1.0 / 4.0; c.b143 = 2.0;
    c.b221 = 1.0; c.b231 = -1.0;
    c.b331 = 1.0 / 4.0; c.b332 = 3.0 / 4.0;
    c.b341 = 1.0 / 4.0; c.b342 = 3.0 / 4.0;
    c.alpha1 = 0.0; c.alpha2 = 0.0; c.alpha3 = 1.0 / 2.0; c.alpha4 = 1.0 / 2.0;
    c.c02 = 0.0; c.c03 = 1.0; c.c13 = 1.0; c.c14 = 1.0;
    c.beta11 = 1.0 / 8.0; c.beta12 = 3.0 / 8.0; c.beta13 = 3.0 / 8.0; c.beta14 = 1.0 / 8.0;
    c.beta22 = -1.0 / 4.0; c.beta23 = 1.0 / 4.0;
    return c;
}

svStochasticIntegratorRS2::svStochasticIntegratorRS2(DynamicObject* dyn)
    : svStochasticIntegratorRS(dyn, svStochasticIntegratorRS2::getCoefficients())
{}

RSCoefficients svStochasticIntegratorRS2::getCoefficients()
{
    RSCoefficients c;
    c.a021 = 2.0 / 3.0; c.a031 = 1.0 / 6.0; c.a032 = 1.0 / 2.0;
    c.a131 = 1.0; c.a141 = 1.0;
    c.b031 = 1.0 / 4.0; c.b032 = 3.0 / 4.0;
    c.b121 = 2.0 / 3.0;
    c.b131 = 1.0 / 12.0; c.b132 = 1.0 / 4.0;
    c.b141 = -5.0 / 4.0; c.b142 = 1.0 / 4.0; c.b143 = 2.0;
    c.b221 = 1.0; c.b231 = -1.0;
    c.b331 = 1.0 / 4.0; c.b332 = 3.0 / 4.0;
    c.b341 = 1.0 / 4.0; c.b342 = 3.0 / 4.0;
    c.alpha1 = 1.0 / 4.0; c.alpha2 = 1.0 / 4.0; c.alpha3 = 1.0 / 2.0; c.alpha4 = 0.0;
    c.c02 = 2.0 / 3.0; c.c03 = 2.0 / 3.0; c.c13 = 1.0; c.c14 = 1.0;
    c.beta11 = 1.0 / 8.0; c.beta12 = 3.0 / 8.0; c.beta13 = 3.0 / 8.0; c.beta14 = 1.0 / 8.0;
    c.beta22 = -1.0 / 4.0; c.beta23 = 1.0 / 4.0;
    return c;
}

void svStochasticIntegratorRS::integrate(double currentTime, double timeStep)
{
    if (timeStep == 0) return;

    const RSCoefficients& c = this->coefficients;
    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);
    const std::vector<StateIdToIndexMap>& maps = noiseIndexMaps();
    const size_t m = maps.size();

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const double h = timeStep;
    const double sqh = std::sqrt(h);

    // Random variables of Roessler (2007). Ihat[k], k=0..m-1, is three-point distributed
    // ({+-sqrt(3h) w.p. 1/6, 0 w.p. 2/3}); Itilde[k], k=0..m-2, is two-point ({+-sqrt(h)
    // w.p. 1/2}). Only 2m-1 independent variables are used. Both are deterministic
    // functions of the Gaussian dW/dZ, so the prescribed-noise test harness drives them.
    Eigen::VectorXd Ihat(m);
    for (size_t k = 0; k < m; k++) {
        Ihat(k) = stochasticWeakRV::threePoint(sample.dW(k), h);
    }
    Eigen::VectorXd Itilde = Eigen::VectorXd::Zero(m); // index m-1 unused
    for (size_t k = 0; k + 1 < m; k++) {
        Itilde(k) = stochasticWeakRV::twoPoint(sample.dZ(k), sqh);
    }
    // Mixed iterated integral, eq. (5.2): Ihat2(k,l) = Ihat[k] Itilde[l] if l<k,
    //                                                 -Ihat[l] Itilde[k] if k<l.
    auto Ihat2 = [&](size_t k, size_t l) -> double {
        if (l < k) return Ihat(k) * Itilde(l);
        return -Ihat(l) * Itilde(k); // k < l
    };

    // scaledSum4(coefRow, stages, upto): sum_j coefRow[j] * stages[j] over j < upto,
    // returning a full ExtendedStateVector. Used for both f-stage and g-stage combinations.
    auto scaledSum4 = [&](const std::array<double, 4>& row,
                          const std::array<ExtendedStateVector, 4>& v,
                          size_t upto) -> ExtendedStateVector {
        ExtendedStateVector acc = v.at(0) * row.at(0);
        for (size_t j = 1; j < upto; j++) {
            if (row.at(j) != 0.0) acc += v.at(j) * row.at(j);
        }
        return acc;
    };

    // ---- Drift stages ----
    // f_H0[i] = f(t_n + c0[i] h, H0[i]) ; H0 shared across noise sources.
    // b_Hk[k][i] = b^k(t_n + c1[i] h, H^(k)_i) (source k's diffusion at its own stage state).
    std::array<ExtendedStateVector, 4> f_H0;
    std::vector<std::array<ExtendedStateVector, 4>> b_Hk(m);

    f_H0.at(0) = computeDerivatives(currentTime, timeStep);
    {
        std::vector<ExtendedStateVector> g0 = computeDiffusions(currentTime, timeStep, maps);
        for (size_t k = 0; k < m; k++) b_Hk.at(k).at(0) = g0.at(k);
    }

    // A0/A1/A2 and B0/B1/B2/B3 rows as std::array for scaledSum helpers.
    auto row = [](double x0, double x1, double x2, double x3) {
        return std::array<double, 4>{x0, x1, x2, x3};
    };
    const std::array<std::array<double, 4>, 4> A0 = {
        row(0, 0, 0, 0), row(c.a021, 0, 0, 0), row(c.a031, c.a032, 0, 0), row(0, 0, 0, 0)};
    const std::array<std::array<double, 4>, 4> A1 = {
        row(0, 0, 0, 0), row(0, 0, 0, 0), row(c.a131, 0, 0, 0), row(c.a141, 0, 0, 0)};
    const std::array<std::array<double, 4>, 4> B0 = {
        row(0, 0, 0, 0), row(0, 0, 0, 0), row(c.b031, c.b032, 0, 0), row(0, 0, 0, 0)};
    const std::array<std::array<double, 4>, 4> B1 = {
        row(0, 0, 0, 0), row(c.b121, 0, 0, 0), row(c.b131, c.b132, 0, 0),
        row(c.b141, c.b142, c.b143, 0)};
    const std::array<std::array<double, 4>, 4> B3 = {
        row(0, 0, 0, 0), row(0, 0, 0, 0), row(c.b331, c.b332, 0, 0), row(c.b341, c.b342, 0, 0)};
    const std::array<double, 4> c0nodes = {0.0, c.c02, c.c03, 0.0};
    const std::array<double, 4> c1nodes = {0.0, 0.0, c.c13, c.c14};

    // Compute the drift and diffusion stages for i = 1, 2, 3 (i = 0 is x_n).
    for (size_t i = 1; i < 4; i++) {
        // H0[i] = x_n + h sum_j A0[i][j] f(H0[j]) + sum_l Ihat[l] sum_j B0[i][j] b^l(H^(l)_j)
        currentState.setStates(dynPtrs);
        scaledSum4(A0.at(i), f_H0, i).setDerivatives(dynPtrs);
        for (size_t l = 0; l < m; l++) {
            scaledSum4(B0.at(i), b_Hk.at(l), i).setDiffusions(dynPtrs, maps.at(l));
        }
        propagateState(timeStep, Ihat, maps);
        f_H0.at(i) = computeDerivatives(currentTime + c0nodes.at(i) * timeStep, timeStep);

        // H^(k)_i = x_n + h sum_j A1[i][j] f(H0[j])
        //               + Ihat[k] sum_j B1[i][j] b^k(H^(k)_j)
        //               + sum_{l!=k} Ihat[l] sum_j B3[i][j] b^l(H^(l)_j)
        for (size_t k = 0; k < m; k++) {
            currentState.setStates(dynPtrs);
            scaledSum4(A1.at(i), f_H0, i).setDerivatives(dynPtrs);
            for (size_t l = 0; l < m; l++) {
                const std::array<double, 4>& brow = (l == k) ? B1.at(i) : B3.at(i);
                scaledSum4(brow, b_Hk.at(l), i).setDiffusions(dynPtrs, maps.at(l));
            }
            propagateState(timeStep, Ihat, maps);
            b_Hk.at(k).at(i) =
                computeDiffusion(currentTime + c1nodes.at(i) * timeStep, timeStep, maps.at(k));
        }
    }

    // ---- Cross-noise stages Hhat^(k)_i and b^k(Hhat^(k)_i) (needed only for m > 1) ----
    // Hhat^(k)_i = x_n + h sum_j A2[i][j] f(H0[j])
    //                  + sum_{l!=k} (Ihat2(k,l)/sqrt(h)) sum_j B2[i][j] b^l(H^(l)_j)
    // Only i where B2 has a nonzero row (i = 1, 2 for RS1/RS2) contribute. A2 = 0 here.
    std::vector<std::array<ExtendedStateVector, 4>> b_Hhat(m);
    const std::array<std::array<double, 4>, 4> B2 = {
        row(0, 0, 0, 0), row(c.b221, 0, 0, 0), row(c.b231, 0, 0, 0), row(0, 0, 0, 0)};
    if (m > 1) {
        for (size_t k = 0; k < m; k++) {
            // stage 0 is x_n; b^k there was already computed as b_Hk[k][0].
            b_Hhat.at(k).at(0) = b_Hk.at(k).at(0);
            for (size_t i = 1; i < 4; i++) {
                // Hhat^(k)_i = x_n + sum_{l!=k} (Ihat2(k,l)/sqrt(h)) sum_j B2[i][j] b^l(H^(l)_j).
                // A2 = 0 (no drift term) and there is no l==k self term, so start from x_n
                // and accumulate only the cross-noise (l != k) contributions. The pseudo-step
                // for source k stays 0 (propagateState with timeStep 0 adds no drift).
                currentState.setStates(dynPtrs);
                Eigen::VectorXd step = Eigen::VectorXd::Zero(m);
                for (size_t l = 0; l < m; l++) {
                    if (l == k) continue;
                    scaledSum4(B2.at(i), b_Hk.at(l), i).setDiffusions(dynPtrs, maps.at(l));
                    step(l) = Ihat2(k, l) / sqh;
                }
                propagateState(0, step, maps);
                b_Hhat.at(k).at(i) = computeDiffusion(currentTime, timeStep, maps.at(k));
            }
        }
    }

    // ---- State update, eq. (5.1) ----
    // u = x_n + h sum_i (alpha[i]) f(H0[i])        [note alpha uses k1 for i=0 and i=3 in RS1]
    //         + sum_i sum_k beta1[i] b^k(H^(k)_i) Ihat[k]
    //         + sum_i sum_k beta2[i] b^k(Hhat^(k)_i) sqrt(h)
    // Roessler's alpha already encodes the k4 = k1 reuse via the alpha vector below.
    currentState.setStates(dynPtrs);
    {
        const std::array<double, 4> alpha = {c.alpha1, c.alpha2, c.alpha3, c.alpha4};
        // f_H0[3] for RS1 has c0node 0 so equals f(x_n) = f_H0[0]; the alpha4 weight is
        // applied to f_H0[3] which was computed at node c0[3]=0, matching k4 = k1.
        scaledSum4(alpha, f_H0, 4).setDerivatives(dynPtrs);
    }
    // beta1 diffusion term (weighted by Ihat[k]).
    {
        const std::array<double, 4> beta1 = {c.beta11, c.beta12, c.beta13, c.beta14};
        for (size_t k = 0; k < m; k++) {
            scaledSum4(beta1, b_Hk.at(k), 4).setDiffusions(dynPtrs, maps.at(k));
        }
        propagateState(timeStep, Ihat, maps);
    }
    // beta2 diffusion term (weighted by sqrt(h)), using the cross-noise stage diffusions.
    if (m > 1) {
        const std::array<double, 4> beta2 = {0.0, c.beta22, c.beta23, 0.0};
        for (size_t k = 0; k < m; k++) {
            scaledSum4(beta2, b_Hhat.at(k), 4).setDiffusions(dynPtrs, maps.at(k));
        }
        Eigen::VectorXd step(m);
        for (size_t k = 0; k < m; k++) step(k) = sqh;
        propagateState(0, step, maps);
    }

    // The dynPtrs now hold x_{n+1}.
}
