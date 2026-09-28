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
// Retained recurrences from commit 7436aaae5e, before weak stage-buffer reuse.
// Keep these independent of the reusable-stage helpers under test.
#ifndef stochasticWeakReference_h
#define stochasticWeakReference_h
#include "simulation/dynamics/Integrators/svStochasticIntegratorW2Ito1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorW2Ito.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorW2Ito2.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorDRI1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorRS.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSIESME.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorRDI1WM.h"
#include "simulation/dynamics/_GeneralModuleFiles/extendedStateVector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stochasticNoiseGenerator.h"
#include "simulation/dynamics/_GeneralModuleFiles/stochasticWeakRandomVariables.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorWeakSIESME.h"
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace stochasticWeakReference {
/** @brief Retained W2Ito recurrence using independent owning snapshots. */
template<class Integrator>
class W2Ito : public Integrator {
public:
    using Integrator::Integrator;
    /** @brief Advance the test dynamics using the retained reference recurrence.
     * @param currentTime Start time of the integration step in seconds.
     * @param timeStep Integration step duration in seconds; zero leaves the state
     * unchanged and consumes no noise sample.
     */
    void integrate(double currentTime, double timeStep) override
    {
        if (timeStep == 0) return;

        const W2ItoCoefficients& c = this->coefficients;
        const size_t s = c.numStages();
        const ExtendedStateVector currentState = ExtendedStateVector::fromStates(this->dynPtrs);
        const std::vector<StateIdToIndexMap>& maps = this->noiseIndexMaps();
        const size_t m = maps.size();
        const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

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
        Eigen::VectorXd _dW(noiseCount), Ikk(noiseCount);
        for (size_t k = 0; k < m; k++) {
            const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
            _dW(eigenK) = stochasticWeakRV::threePoint(sample.dW(eigenK), h);
            Ikk(eigenK) = (_dW(eigenK) * _dW(eigenK) / xi - xi) / 2.0;
        }
        auto Ikl = [&](size_t k, size_t l) -> double {
            const Eigen::Index eigenL = static_cast<Eigen::Index>(l);
            if (k < l) return 0.5 * (_dW(eigenL) - eta2 * _dW(eigenL));
            return 0.5 * (_dW(eigenL) + eta2 * _dW(eigenL)); // k > l
        };

        // Stage function evaluations. f_H0[i] = f(H_i^(0)); g_Hk[k][i] = g_k(H_i^(k)).
        std::vector<ExtendedStateVector> f_H0(s);
        std::vector<std::vector<ExtendedStateVector>> g_Hk(m, std::vector<ExtendedStateVector>(s));

        // Stage 0: H_0^(0) == H_0^(k) == x_n, so evaluate f and every g at the current state.
        f_H0.at(0) = this->computeDerivatives(currentTime, timeStep);
        {
            std::vector<ExtendedStateVector> diffs = this->computeDiffusions(currentTime, timeStep, maps);
            for (size_t k = 0; k < m; k++) g_Hk.at(k).at(0) = diffs.at(k);
        }

        // Stages i = 1..s-1.
        for (size_t i = 1; i < s; i++) {
            // H_i^(0) = x_n + h*sum_{j<i} A0[i][j] f(H0[j]) + sum_k Ihat_k*sum_{j<i} B0[i][j] g_k(Hk[j])
            currentState.setStates(this->dynPtrs);
            this->scaledSum(c.A0.at(i), f_H0, i).setDerivatives(this->dynPtrs);
            for (size_t k = 0; k < m; k++) {
                this->scaledSum(c.B0.at(i), g_Hk.at(k), i).setDiffusions(this->dynPtrs, maps.at(k));
            }
            this->propagateStateWithCachedNoise(timeStep, _dW);
            f_H0.at(i) = this->computeDerivatives(currentTime + c.c0(i) * timeStep, timeStep);

            // H_i^(k) = x_n + h*sum_{j<i} A1[i][j] f(H0[j]) + xi*sum_{j<i} B1[i][j] g_k(Hk[j])
            //               + sum_{l!=k} Ihat_(k,l)*sum_{j<i} B2[i][j] g_l(Hl[j])
            for (size_t k = 0; k < m; k++) {
                currentState.setStates(this->dynPtrs);
                this->scaledSum(c.A1.at(i), f_H0, i).setDerivatives(this->dynPtrs);
                this->scaledSum(c.B1.at(i), g_Hk.at(k), i).setDiffusions(this->dynPtrs, maps.at(k));
                for (size_t l = 0; l < m; l++) {
                    if (l == k) continue;
                    this->scaledSum(c.B2.at(i), g_Hk.at(l), i).setDiffusions(this->dynPtrs, maps.at(l));
                }
                Eigen::VectorXd step(noiseCount);
                for (size_t l = 0; l < m; l++) {
                    step(static_cast<Eigen::Index>(l)) = (l == k) ? xi : Ikl(k, l);
                }
                this->propagateStateWithCachedNoise(timeStep, step);
                g_Hk.at(k).at(i) =
                    this->computeDiffusion(currentTime + c.c1(i) * timeStep, timeStep, maps.at(k));
            }
        }

        // State update (paper eq. 3.1):
        //   y_{n+1} = x_n + h*sum_i alpha[i] f(H0[i])
        //                 + sum_k Ihat_k    * sum_i beta0[i] g_k(Hk[i])
        //                 + sum_k Ihat_(k,k)* sum_i beta1[i] g_k(Hk[i])
        currentState.setStates(this->dynPtrs);
        this->scaledSum(c.alpha, f_H0, s).setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) {
            this->scaledSum(c.beta0, g_Hk.at(k), s).setDiffusions(this->dynPtrs, maps.at(k));
        }
        this->propagateStateWithCachedNoise(timeStep, _dW);
        for (size_t k = 0; k < m; k++) {
            this->scaledSum(c.beta1, g_Hk.at(k), s).setDiffusions(this->dynPtrs, maps.at(k));
        }
        this->propagateStateWithCachedNoise(0, Ikk);

        // The dynPtrs now hold x_{n+1}.
    }
};

/** @brief Retained DRI1 recurrence using independent owning snapshots. */
template<class Integrator>
class DRI1 : public Integrator {
public:
    using Integrator::Integrator;
    /** @brief Advance the test dynamics using the retained reference recurrence.
     * @param currentTime Start time of the integration step in seconds.
     * @param timeStep Integration step duration in seconds; zero leaves the state
     * unchanged and consumes no noise sample.
     */
    void integrate(double currentTime, double timeStep) override
    {
        if (timeStep == 0) return;

        const DRI1Coefficients& c = this->coefficients;
        const ExtendedStateVector currentState = ExtendedStateVector::fromStates(this->dynPtrs);
        const std::vector<StateIdToIndexMap>& maps = this->noiseIndexMaps();
        const size_t m = maps.size();
        const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

        const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
        const double h = timeStep;
        const double sqh = std::sqrt(h);

        // Discrete random variables (deterministic functions of the Gaussian dW/dZ):
        //   _dW : three-point in {-sqrt(3h), 0, +sqrt(3h)}
        //   chi1: (_dW^2 - h)/2      (diagonal of Ihat2)
        //   _dZ : two-point in {-sqrt(h), +sqrt(h)}  (only used for cross-noise, m>1)
        Eigen::VectorXd _dW(noiseCount), chi1(noiseCount), _dZ(noiseCount);
        for (size_t k = 0; k < m; k++) {
            const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
            _dW(eigenK) = stochasticWeakRV::threePoint(sample.dW(eigenK), h);
            chi1(eigenK) = (_dW(eigenK) * _dW(eigenK) - h) / 2.0;
            _dZ(eigenK) = stochasticWeakRV::twoPoint(sample.dZ(eigenK), sqh);
        }

        // ---- Drift stages (shared across noise sources) ----
        // k1 = f(x_n); g1 = g(x_n)
        ExtendedStateVector k1 = this->computeDerivatives(currentTime, timeStep);
        std::vector<ExtendedStateVector> g1 = this->computeDiffusions(currentTime, timeStep, maps);

        // H02 = x_n + a021*k1*h + b021*g1*_dW ; k2 = f(H02, t+c02*h)
        currentState.setStates(this->dynPtrs);
        (k1 * c.a021).setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) (g1.at(k) * c.b021).setDiffusions(this->dynPtrs, maps.at(k));
        this->propagateStateWithCachedNoise(timeStep, _dW);
        ExtendedStateVector k2 = this->computeDerivatives(currentTime + c.c02 * timeStep, timeStep);

        // H03 = x_n + (a031*k1 + a032*k2)*h + b031*g1*_dW ; k3 = f(H03, t+c03*h)
        currentState.setStates(this->dynPtrs);
        {
            ExtendedStateVector d = k1 * c.a031;
            d += k2 * c.a032;
            d.setDerivatives(this->dynPtrs);
        }
        for (size_t k = 0; k < m; k++) (g1.at(k) * c.b031).setDiffusions(this->dynPtrs, maps.at(k));
        this->propagateStateWithCachedNoise(timeStep, _dW);
        ExtendedStateVector k3 = this->computeDerivatives(currentTime + c.c03 * timeStep, timeStep);

        // ---- Diffusion stages, per noise source k ----
        // H12[k] = x_n + a121*k1*h + b121*g1[k]*sqrt(h)*e_k
        // H13[k] = x_n + a131*k1*h + b131*g1[k]*sqrt(h)*e_k
        // g2[k] = g(H12[k]); g3[k] = g(H13[k])   (only component k used)
        std::vector<ExtendedStateVector> g2(m), g3(m);
        for (size_t k = 0; k < m; k++) {
            Eigen::VectorXd stepK = Eigen::VectorXd::Zero(noiseCount);
            const Eigen::Index eigenK = static_cast<Eigen::Index>(k);

            currentState.setStates(this->dynPtrs);
            (k1 * c.a121).setDerivatives(this->dynPtrs);
            g1.at(k).setDiffusions(this->dynPtrs, maps.at(k));
            stepK(eigenK) = c.b121 * sqh;
            this->propagateStateWithCachedNoise(timeStep, stepK);
            g2.at(k) = this->computeDiffusion(currentTime + c.c12 * timeStep, timeStep, maps.at(k));

            currentState.setStates(this->dynPtrs);
            (k1 * c.a131).setDerivatives(this->dynPtrs);
            g1.at(k).setDiffusions(this->dynPtrs, maps.at(k));
            stepK(eigenK) = c.b131 * sqh;
            this->propagateStateWithCachedNoise(timeStep, stepK);
            g3.at(k) = this->computeDiffusion(currentTime + c.c13 * timeStep, timeStep, maps.at(k));
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
                Eigen::VectorXd stepL = Eigen::VectorXd::Zero(noiseCount);
                const Eigen::Index eigenL = static_cast<Eigen::Index>(l);

                // Hhat2[l]
                currentState.setStates(this->dynPtrs);
                {
                    ExtendedStateVector d = g1.at(l) * c.b221;
                    d += g2.at(l) * c.b222;
                    d += g3.at(l) * c.b223;
                    d.setDiffusions(this->dynPtrs, maps.at(l));
                }
                stepL(eigenL) = sqh;
                this->propagateStateWithCachedNoise(0, stepL);
                gHat2Full.at(l) = this->computeDiffusions(currentTime, timeStep, maps);

                // Hhat3[l]
                currentState.setStates(this->dynPtrs);
                {
                    ExtendedStateVector d = g1.at(l) * c.b231;
                    d += g2.at(l) * c.b232;
                    d += g3.at(l) * c.b233;
                    d.setDiffusions(this->dynPtrs, maps.at(l));
                }
                stepL(eigenL) = sqh;
                this->propagateStateWithCachedNoise(0, stepL);
                gHat3Full.at(l) = this->computeDiffusions(currentTime, timeStep, maps);
            }
        }

        // ---- State update ----
        // Drift: x_n + (alpha1*k1 + alpha2*k2 + alpha3*k3)*h
        currentState.setStates(this->dynPtrs);
        {
            ExtendedStateVector d = k1 * c.alpha1;
            d += k2 * c.alpha2;
            d += k3 * c.alpha3;
            d.setDerivatives(this->dynPtrs);
        }
        // Noise line 1: beta11 * g1 * _dW  (plus the (m-1)*beta31 g1 self term that
        // accompanies the cross-noise contribution; omitted in the non-mixing variant).
        for (size_t k = 0; k < m; k++) {
            double self31 = doCrossNoise ? (double)(m - 1) * c.beta31 : 0.0;
            (g1.at(k) * (c.beta11 + self31)).setDiffusions(this->dynPtrs, maps.at(k));
        }
        this->propagateStateWithCachedNoise(timeStep, _dW);

        // Noise from g2/g3: (_dW*beta12 + chi1*beta22/sqrt(h)) g2 + (_dW*beta13 + chi1*beta23/sqrt(h)) g3
        for (size_t k = 0; k < m; k++) g2.at(k).setDiffusions(this->dynPtrs, maps.at(k));
        {
            Eigen::VectorXd step(noiseCount);
            for (Eigen::Index k = 0; k < noiseCount; k++) {
                step(k) = _dW(k) * c.beta12 + chi1(k) * c.beta22 / sqh;
            }
            this->propagateStateWithCachedNoise(0, step);
        }
        for (size_t k = 0; k < m; k++) g3.at(k).setDiffusions(this->dynPtrs, maps.at(k));
        {
            Eigen::VectorXd step(noiseCount);
            for (Eigen::Index k = 0; k < noiseCount; k++) {
                step(k) = _dW(k) * c.beta13 + chi1(k) * c.beta23 / sqh;
            }
            this->propagateStateWithCachedNoise(0, step);
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

                    gHat2Full.at(l).at(k).setDiffusions(this->dynPtrs, maps.at(k));
                    {
                        Eigen::VectorXd step = Eigen::VectorXd::Zero(noiseCount);
                        step(eigenK) = w2;
                        this->propagateStateWithCachedNoise(0, step);
                    }
                    gHat3Full.at(l).at(k).setDiffusions(this->dynPtrs, maps.at(k));
                    {
                        Eigen::VectorXd step = Eigen::VectorXd::Zero(noiseCount);
                        step(eigenK) = w3;
                        this->propagateStateWithCachedNoise(0, step);
                    }
                }
            }
        }

        // The dynPtrs now hold x_{n+1}.
    }
};

/** @brief Retained RS recurrence using independent owning snapshots. */
template<class Integrator>
class RS : public Integrator {
public:
    using Integrator::Integrator;
    /** @brief Advance the test dynamics using the retained reference recurrence.
     * @param currentTime Start time of the integration step in seconds.
     * @param timeStep Integration step duration in seconds; zero leaves the state
     * unchanged and consumes no noise sample.
     */
    void integrate(double currentTime, double timeStep) override
    {
        if (timeStep == 0) return;

        const RSCoefficients& c = this->coefficients;
        const ExtendedStateVector currentState = ExtendedStateVector::fromStates(this->dynPtrs);
        const std::vector<StateIdToIndexMap>& maps = this->noiseIndexMaps();
        const size_t m = maps.size();
        const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

        const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
        const double h = timeStep;
        const double sqh = std::sqrt(h);

        // Random variables of Roessler (2007). Ihat[k], k=0..m-1, is three-point distributed
        // ({+-sqrt(3h) w.p. 1/6, 0 w.p. 2/3}); Itilde[k], k=0..m-2, is two-point ({+-sqrt(h)
        // w.p. 1/2}). Only 2m-1 independent variables are used. Both are deterministic
        // functions of the Gaussian dW/dZ, so the prescribed-noise test harness drives them.
        Eigen::VectorXd Ihat(noiseCount);
        for (size_t k = 0; k < m; k++) {
            const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
            Ihat(eigenK) = stochasticWeakRV::threePoint(sample.dW(eigenK), h);
        }
        Eigen::VectorXd Itilde = Eigen::VectorXd::Zero(noiseCount); // index m-1 unused
        for (size_t k = 0; k + 1 < m; k++) {
            const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
            Itilde(eigenK) = stochasticWeakRV::twoPoint(sample.dZ(eigenK), sqh);
        }
        // Mixed iterated integral, eq. (5.2): Ihat2(k,l) = Ihat[k] Itilde[l] if l<k,
        //                                                 -Ihat[l] Itilde[k] if k<l.
        auto Ihat2 = [&](size_t k, size_t l) -> double {
            const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
            const Eigen::Index eigenL = static_cast<Eigen::Index>(l);
            if (l < k) return Ihat(eigenK) * Itilde(eigenL);
            return -Ihat(eigenL) * Itilde(eigenK); // k < l
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

        f_H0.at(0) = this->computeDerivatives(currentTime, timeStep);
        {
            std::vector<ExtendedStateVector> g0 = this->computeDiffusions(currentTime, timeStep, maps);
            for (size_t k = 0; k < m; k++) b_Hk.at(k).at(0) = g0.at(k);
        }

        // A0/A1/A2 and B0/B1/B2/B3 rows as std::array for this->scaledSum helpers.
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
            currentState.setStates(this->dynPtrs);
            scaledSum4(A0.at(i), f_H0, i).setDerivatives(this->dynPtrs);
            for (size_t l = 0; l < m; l++) {
                scaledSum4(B0.at(i), b_Hk.at(l), i).setDiffusions(this->dynPtrs, maps.at(l));
            }
            this->propagateStateWithCachedNoise(timeStep, Ihat);
            f_H0.at(i) = this->computeDerivatives(currentTime + c0nodes.at(i) * timeStep, timeStep);

            // H^(k)_i = x_n + h sum_j A1[i][j] f(H0[j])
            //               + Ihat[k] sum_j B1[i][j] b^k(H^(k)_j)
            //               + sum_{l!=k} Ihat[l] sum_j B3[i][j] b^l(H^(l)_j)
            for (size_t k = 0; k < m; k++) {
                currentState.setStates(this->dynPtrs);
                scaledSum4(A1.at(i), f_H0, i).setDerivatives(this->dynPtrs);
                for (size_t l = 0; l < m; l++) {
                    const std::array<double, 4>& brow = (l == k) ? B1.at(i) : B3.at(i);
                    scaledSum4(brow, b_Hk.at(l), i).setDiffusions(this->dynPtrs, maps.at(l));
                }
                this->propagateStateWithCachedNoise(timeStep, Ihat);
                b_Hk.at(k).at(i) =
                    this->computeDiffusion(currentTime + c1nodes.at(i) * timeStep, timeStep, maps.at(k));
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
                    currentState.setStates(this->dynPtrs);
                    Eigen::VectorXd step = Eigen::VectorXd::Zero(noiseCount);
                    for (size_t l = 0; l < m; l++) {
                        if (l == k) continue;
                        scaledSum4(B2.at(i), b_Hk.at(l), i).setDiffusions(this->dynPtrs, maps.at(l));
                        step(static_cast<Eigen::Index>(l)) = Ihat2(k, l) / sqh;
                    }
                    this->propagateStateWithCachedNoise(0, step);
                    b_Hhat.at(k).at(i) = this->computeDiffusion(currentTime, timeStep, maps.at(k));
                }
            }
        }

        // ---- State update, eq. (5.1) ----
        // u = x_n + h sum_i (alpha[i]) f(H0[i])        [note alpha uses k1 for i=0 and i=3 in RS1]
        //         + sum_i sum_k beta1[i] b^k(H^(k)_i) Ihat[k]
        //         + sum_i sum_k beta2[i] b^k(Hhat^(k)_i) sqrt(h)
        // Roessler's alpha already encodes the k4 = k1 reuse via the alpha vector below.
        currentState.setStates(this->dynPtrs);
        {
            const std::array<double, 4> alpha = {c.alpha1, c.alpha2, c.alpha3, c.alpha4};
            // f_H0[3] for RS1 has c0node 0 so equals f(x_n) = f_H0[0]; the alpha4 weight is
            // applied to f_H0[3] which was computed at node c0[3]=0, matching k4 = k1.
            scaledSum4(alpha, f_H0, 4).setDerivatives(this->dynPtrs);
        }
        // beta1 diffusion term (weighted by Ihat[k]).
        {
            const std::array<double, 4> beta1 = {c.beta11, c.beta12, c.beta13, c.beta14};
            for (size_t k = 0; k < m; k++) {
                scaledSum4(beta1, b_Hk.at(k), 4).setDiffusions(this->dynPtrs, maps.at(k));
            }
            this->propagateStateWithCachedNoise(timeStep, Ihat);
        }
        // beta2 diffusion term (weighted by sqrt(h)), using the cross-noise stage diffusions.
        if (m > 1) {
            const std::array<double, 4> beta2 = {0.0, c.beta22, c.beta23, 0.0};
            for (size_t k = 0; k < m; k++) {
                scaledSum4(beta2, b_Hhat.at(k), 4).setDiffusions(this->dynPtrs, maps.at(k));
            }
            Eigen::VectorXd step(noiseCount);
            for (Eigen::Index k = 0; k < noiseCount; k++) step(k) = sqh;
            this->propagateStateWithCachedNoise(0, step);
        }

        // The dynPtrs now hold x_{n+1}.
    }
};

/** @brief Retained SIESME recurrence using independent owning snapshots. */
template<class Integrator>
class SIESME : public Integrator {
public:
    using Integrator::Integrator;
    /** @brief Advance the test dynamics using the retained reference recurrence.
     * @param currentTime Start time of the integration step in seconds.
     * @param timeStep Integration step duration in seconds; zero leaves the state
     * unchanged and consumes no noise sample.
     */
    void integrate(double currentTime, double timeStep) override
    {
        // A zero-duration step advances nothing and must not consume a noise sample.
        if (timeStep == 0) return;

        const SIESMECoefficients& c = this->coefficients;
        const ExtendedStateVector currentState = ExtendedStateVector::fromStates(this->dynPtrs);

        const std::vector<StateIdToIndexMap>& maps = this->noiseIndexMaps();
        const size_t m = maps.size();
        const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

        const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
        const Eigen::VectorXd& dW = sample.dW;

        const double h = timeStep;
        const double sqh = std::sqrt(h);

        // Polynomial moments of the Gaussian increment (per noise source).
        Eigen::VectorXd W2(noiseCount); // dW^2 / sqrt(h)
        Eigen::VectorXd W3(noiseCount); // nu2 * dW^3 / h
        for (size_t k = 0; k < m; k++) {
            const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
            W2(eigenK) = dW(eigenK) * dW(eigenK) / sqh;
            W3(eigenK) = c.nu2 * dW(eigenK) * dW(eigenK) * dW(eigenK) / h;
        }

        // --- Stage 0: k0 = f(x_n), g0 = g(x_n) ---
        ExtendedStateVector k0 = this->computeDerivatives(currentTime, timeStep);
        std::vector<ExtendedStateVector> g0 = this->computeDiffusions(currentTime, timeStep, maps);

        // --- k1 stage: state = x_n + lambda0*k0*h + g0.*(nu1*dW + W3); k1 = f(state, t+mu0*h) ---
        currentState.setStates(this->dynPtrs);
        (k0 * c.lambda0).setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) {
            g0.at(k).setDiffusions(this->dynPtrs, maps.at(k));
        }
        {
            Eigen::VectorXd step(noiseCount);
            for (Eigen::Index k = 0; k < noiseCount; k++) step(k) = c.nu1 * dW(k) + W3(k);
            this->propagateStateWithCachedNoise(timeStep, step);
        }
        ExtendedStateVector k1 = this->computeDerivatives(currentTime + c.mu0 * timeStep, timeStep);

        // --- g1 stage: state = x_n + lambdabar0*k0*h + g0.*(beta2*sqrt(h) + beta3*W2) ---
        currentState.setStates(this->dynPtrs);
        (k0 * c.lambdabar0).setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) {
            g0.at(k).setDiffusions(this->dynPtrs, maps.at(k));
        }
        {
            Eigen::VectorXd step(noiseCount);
            for (Eigen::Index k = 0; k < noiseCount; k++) step(k) = c.beta2 * sqh + c.beta3 * W2(k);
            this->propagateStateWithCachedNoise(timeStep, step);
        }
        std::vector<ExtendedStateVector> g1 =
            this->computeDiffusions(currentTime + c.mubar0 * timeStep, timeStep, maps);

        // --- g2 stage: state = x_n + lambdabar0*k0*h + g0.*(delta2*sqrt(h) + delta3*W2) ---
        currentState.setStates(this->dynPtrs);
        (k0 * c.lambdabar0).setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) {
            g0.at(k).setDiffusions(this->dynPtrs, maps.at(k));
        }
        {
            Eigen::VectorXd step(noiseCount);
            for (Eigen::Index k = 0; k < noiseCount; k++) step(k) = c.delta2 * sqh + c.delta3 * W2(k);
            this->propagateStateWithCachedNoise(timeStep, step);
        }
        std::vector<ExtendedStateVector> g2 =
            this->computeDiffusions(currentTime + c.mubar0 * timeStep, timeStep, maps);

        // --- State update ---
        // x_{n+1} = x_n + (alpha1*k0 + alpha2*k1)*h
        //               + gamma1*g0.*dW
        //               + (lambda1*dW + lambda2*sqrt(h) + lambda3*W2).*g1
        //               + (mu1*dW + mu2*sqrt(h) + mu3*W2).*g2
        currentState.setStates(this->dynPtrs);
        ExtendedStateVector drift = k0 * c.alpha1;
        drift += k1 * c.alpha2;
        drift.setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) {
            g0.at(k).setDiffusions(this->dynPtrs, maps.at(k));
        }
        {
            Eigen::VectorXd step(noiseCount);
            for (Eigen::Index k = 0; k < noiseCount; k++) step(k) = c.gamma1 * dW(k);
            this->propagateStateWithCachedNoise(timeStep, step);
        }
        for (size_t k = 0; k < m; k++) {
            g1.at(k).setDiffusions(this->dynPtrs, maps.at(k));
        }
        {
            Eigen::VectorXd step(noiseCount);
            for (Eigen::Index k = 0; k < noiseCount; k++) {
                step(k) = c.lambda1 * dW(k) + c.lambda2 * sqh + c.lambda3 * W2(k);
            }
            this->propagateStateWithCachedNoise(0, step);
        }
        for (size_t k = 0; k < m; k++) {
            g2.at(k).setDiffusions(this->dynPtrs, maps.at(k));
        }
        {
            Eigen::VectorXd step(noiseCount);
            for (Eigen::Index k = 0; k < noiseCount; k++) {
                step(k) = c.mu1 * dW(k) + c.mu2 * sqh + c.mu3 * W2(k);
            }
            this->propagateStateWithCachedNoise(0, step);
        }

        // The dynPtrs now hold x_{n+1}.
    }
};

/** @brief Retained RDI1WM recurrence using independent owning snapshots. */
template<class Integrator>
class RDI1WM : public Integrator {
public:
    using Integrator::Integrator;
    /** @brief Advance the test dynamics using the retained reference recurrence.
     * @param currentTime Start time of the integration step in seconds.
     * @param timeStep Integration step duration in seconds; zero leaves the state
     * unchanged and consumes no noise sample.
     */
    void integrate(double currentTime, double timeStep) override
    {
        constexpr double a021 = 2.0 / 3.0;
        constexpr double b021 = 2.0 / 3.0;
        constexpr double alpha1 = 1.0 / 4.0;
        constexpr double alpha2 = 3.0 / 4.0;
        constexpr double c02 = 2.0 / 3.0;
        constexpr double beta11 = 1.0;

        if (timeStep == 0) return;

        const ExtendedStateVector currentState = ExtendedStateVector::fromStates(this->dynPtrs);
        const std::vector<StateIdToIndexMap>& maps = this->noiseIndexMaps();
        const size_t m = maps.size();
        const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

        const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);

        // Three-point distributed increment per noise source.
        Eigen::VectorXd Ihat(noiseCount);
        for (size_t k = 0; k < m; k++) {
            const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
            Ihat(eigenK) = stochasticWeakRV::threePoint(sample.dW(eigenK), timeStep);
        }

        // Stage 0.
        ExtendedStateVector k1 = this->computeDerivatives(currentTime, timeStep);
        std::vector<ExtendedStateVector> g1 = this->computeDiffusions(currentTime, timeStep, maps);

        // H02 = x_n + a021*k1*h + b021*g1*Ihat
        currentState.setStates(this->dynPtrs);
        (k1 * a021).setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) {
            (g1.at(k) * b021).setDiffusions(this->dynPtrs, maps.at(k));
        }
        this->propagateStateWithCachedNoise(timeStep, Ihat);
        ExtendedStateVector k2 = this->computeDerivatives(currentTime + c02 * timeStep, timeStep);

        // x_{n+1} = x_n + (alpha1*k1 + alpha2*k2)*h + beta11*g1*Ihat
        currentState.setStates(this->dynPtrs);
        ExtendedStateVector drift = k1 * alpha1;
        drift += k2 * alpha2;
        drift.setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) {
            (g1.at(k) * beta11).setDiffusions(this->dynPtrs, maps.at(k));
        }
        this->propagateStateWithCachedNoise(timeStep, Ihat);

        // The dynPtrs now hold x_{n+1}.
    }
};

} // namespace stochasticWeakReference
#endif // stochasticWeakReference_h
