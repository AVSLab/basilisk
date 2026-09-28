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
// Retained step-1 recurrences for stage-by-stage numerical regression checks.
// Keep these independent of the reusable-stage helpers under test.
#ifndef stochasticStrongReference_h
#define stochasticStrongReference_h
#include "simulation/dynamics/Integrators/svStochasticIntegratorEulerHeun.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorRKMil.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSRIW1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSOSRI.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSRA1.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorSOSRA.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include "simulation/dynamics/_GeneralModuleFiles/extendedStateVector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stochasticNoiseGenerator.h"
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace stochasticReference {
/** @brief Retained EulerHeun recurrence using owning map snapshots. */
class EulerHeun : public svStochasticIntegratorEulerHeun {
public:
    using svStochasticIntegratorEulerHeun::svStochasticIntegratorEulerHeun;
    /** @brief Advance the test dynamics using the retained reference recurrence.
     * @param currentTime Start time of the integration step in seconds.
     * @param timeStep Integration step duration in seconds; zero leaves the state
     * unchanged and consumes no noise sample.
     */
    void integrate(double currentTime, double timeStep) override
{
    // A zero-duration step advances nothing and must not consume a noise sample.
    // (Basilisk issues an integrate() call with timeStep == 0 at initialization.)
    if (timeStep == 0) return;

    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);

    const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps = noiseIndexMaps();
    const size_t m = stateIdToNoiseIndexMaps.size();

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const Eigen::VectorXd& dW = sample.dW;

    // --- Predictor at (t_n, x_n): f1, g1 ---
    ExtendedStateVector f1 = computeDerivatives(currentTime, timeStep);
    std::vector<ExtendedStateVector> g1 =
        computeDiffusions(currentTime, timeStep, stateIdToNoiseIndexMaps);

    // xBar = x_n + h * f1 + sum_k g1_k * dW_k
    currentState.setStates(dynPtrs);
    f1.setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        g1.at(k).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateStateWithCachedNoise(timeStep, dW);

    // --- Corrector evaluations at (t_{n+1}, xBar): f2, g2 ---
    ExtendedStateVector f2 = computeDerivatives(currentTime + timeStep, timeStep);
    std::vector<ExtendedStateVector> g2 =
        computeDiffusions(currentTime + timeStep, timeStep, stateIdToNoiseIndexMaps);

    // x_{n+1} = x_n + (h/2)(f1+f2) + sum_k (dW_k/2)(g1_k+g2_k)
    currentState.setStates(dynPtrs);
    ((f1 += f2) * 0.5).setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        ((g1.at(k) += g2.at(k)) * 0.5).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateStateWithCachedNoise(timeStep, dW);

    // The dynPtrs now hold x_{n+1}.
}
};
/** @brief Retained RKMil recurrence using owning map snapshots. */
class RKMil : public svStochasticIntegratorRKMil {
public:
    using svStochasticIntegratorRKMil::svStochasticIntegratorRKMil;
    /** @brief Advance the test dynamics using the retained reference recurrence.
     * @param currentTime Start time of the integration step in seconds.
     * @param timeStep Integration step duration in seconds; zero leaves the state
     * unchanged and consumes no noise sample.
     */
    void integrate(double currentTime, double timeStep) override
{
    // A zero-duration step advances nothing and must not consume a noise sample.
    // (Basilisk issues an integrate() call with timeStep == 0 at initialization.)
    if (timeStep == 0) return;

    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);

    const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps = noiseIndexMaps();
    const size_t m = stateIdToNoiseIndexMaps.size();
    const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

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
    propagateStateWithCachedNoise(timeStep, Eigen::VectorXd::Zero(noiseCount));
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
    propagateStateWithCachedNoise(0.0, sqh * Eigen::VectorXd::Ones(noiseCount));

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
    Eigen::VectorXd milStep(noiseCount);
    for (size_t k = 0; k < m; k++) {
        const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
        milStep(eigenK) = (dW(eigenK) * dW(eigenK) - h) / 2.0;
    }

    // Start from K, add the L*dW term (drift set to zero so it is not double-counted).
    K.setStates(dynPtrs);
    zeroDeriv.setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        L.at(k).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateStateWithCachedNoise(0.0, dW);

    // Add the Milstein correction term.
    for (size_t k = 0; k < m; k++) {
        ggprime.at(k).setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateStateWithCachedNoise(0.0, milStep);

    // The dynPtrs now hold x_{n+1}.
}
};
/** @brief Retained SRI recurrence using owning map snapshots. */
template<class Integrator, size_t numberStages>
class SRI : public Integrator {
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
    // (Basilisk issues an integrate() call with timeStep == 0 at initialization.)
    if (timeStep == 0) return;

    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(this->dynPtrs);

    // Map (ExtendedStateId -> local noise index) for each of the m noise sources (cached).
    const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps = this->noiseIndexMaps();
    const size_t m = stateIdToNoiseIndexMaps.size();
    const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

    // Draw the random variables for this step (needs dW and dZ per noise source).
    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const Eigen::VectorXd& dW = sample.dW;
    const Eigen::VectorXd& dZ = sample.dZ;

    const double h = timeStep;
    const double sqh = std::sqrt(h);
    const double sqrt3 = std::sqrt(3.0);

    // Iterated-integral approximations, one entry per noise source.
    Eigen::VectorXd chi1(noiseCount); // I_(1,1)/sqrt(h)
    Eigen::VectorXd chi2(noiseCount); // I_(1,0)/h
    Eigen::VectorXd chi3(noiseCount); // I_(1,1,1)/h
    for (size_t k = 0; k < m; k++) {
        const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
        chi1(eigenK) = (dW(eigenK) * dW(eigenK) - h) / (2.0 * sqh);
        chi2(eigenK) = (dW(eigenK) + dZ(eigenK) / sqrt3) / 2.0;
        chi3(eigenK) =
            (dW(eigenK) * dW(eigenK) * dW(eigenK) - 3.0 * dW(eigenK) * h) /
            (6.0 * h);
    }

    // f_H0[i]      = f(t_n + c0[i] h, H0[i])                  for i = 0..s-1
    // g_Hk[k][i]   = g_k(t_n + c1[i] h, H1[i] for source k)   for i = 0..s-1; k = 0..m-1
    std::array<ExtendedStateVector, numberStages> f_H0;
    std::vector<std::array<ExtendedStateVector, numberStages>> g_Hk(m);

    // i = 0: H0[0] == H1[0] == y_n (all A/B rows are strictly lower triangular).
    f_H0.at(0) = this->computeDerivatives(currentTime, timeStep);
    {
        std::vector<ExtendedStateVector> diffs =
            this->computeDiffusions(currentTime, timeStep, stateIdToNoiseIndexMaps);
        for (size_t k = 0; k < m; k++) {
            g_Hk.at(k).at(0) = std::move(diffs.at(k));
        }
    }

    // Remaining stages.
    for (size_t i = 1; i < numberStages; i++) {
        // --- H0[i] (drift stage, shared across noise sources) ---
        // H0[i] = y_n + h sum_j A0[i][j] f(H0[j]) + sum_k chi2[k] sum_j B0[i][j] g_k(H1[j])
        currentState.setStates(this->dynPtrs);
        this->scaledSum(this->coefficients.A0.at(i), f_H0, i).setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) {
            this->scaledSum(this->coefficients.B0.at(i), g_Hk.at(k), i)
                .setDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps.at(k));
        }
        // pseudo time step for the diffusion term is chi2[k]
        this->propagateStateWithCachedNoise(timeStep, chi2);
        f_H0.at(i) = this->computeDerivatives(currentTime + this->coefficients.c0.at(i) * timeStep, timeStep);

        // --- H1[i] (diffusion stage, one per noise source k) ---
        // H1[i] = y_n + h sum_j A1[i][j] f(H0[j]) + sqrt(h) sum_j B1[i][j] g_k(H1[j])
        for (size_t k = 0; k < m; k++) {
            currentState.setStates(this->dynPtrs);
            this->scaledSum(this->coefficients.A1.at(i), f_H0, i).setDerivatives(this->dynPtrs);
            this->scaledSum(this->coefficients.B1.at(i), g_Hk.at(k), i)
                .setDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps.at(k));

            // Only noise source k participates (pseudo step sqrt(h)); all others zero.
            Eigen::VectorXd pseudoTimeStep = Eigen::VectorXd::Zero(noiseCount);
            pseudoTimeStep(static_cast<Eigen::Index>(k)) = sqh;
            this->propagateStateWithCachedNoise(timeStep, pseudoTimeStep);

            g_Hk.at(k).at(i) =
                this->computeDiffusion(currentTime + this->coefficients.c1.at(i) * timeStep, timeStep,
                                 stateIdToNoiseIndexMaps.at(k));  // single-source diffusion
        }
    }

    // --- State update ---
    // y_{n+1} = y_n + h sum_i alpha[i] f(H0[i])
    //               + sum_k [ (beta1 . g_Hk[k]) dW[k] + (beta2 . g_Hk[k]) chi1[k]
    //                        + (beta3 . g_Hk[k]) chi2[k] + (beta4 . g_Hk[k]) chi3[k] ]
    // We accumulate the four diffusion contributions with four propagateState calls
    // (each with h = 0 except the first, so the drift is only counted once).
    currentState.setStates(this->dynPtrs);
    this->scaledSum(this->coefficients.alpha, f_H0, numberStages).setDerivatives(this->dynPtrs);
    for (size_t k = 0; k < m; k++) {
        this->scaledSum(this->coefficients.beta1, g_Hk.at(k), numberStages)
            .setDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    this->propagateStateWithCachedNoise(timeStep, dW);

    for (size_t k = 0; k < m; k++) {
        this->scaledSum(this->coefficients.beta2, g_Hk.at(k), numberStages)
            .setDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    this->propagateStateWithCachedNoise(0, chi1);

    for (size_t k = 0; k < m; k++) {
        this->scaledSum(this->coefficients.beta3, g_Hk.at(k), numberStages)
            .setDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    this->propagateStateWithCachedNoise(0, chi2);

    for (size_t k = 0; k < m; k++) {
        this->scaledSum(this->coefficients.beta4, g_Hk.at(k), numberStages)
            .setDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    this->propagateStateWithCachedNoise(0, chi3);

    // The this->dynPtrs now hold y_{n+1}.
}
};
/** @brief Retained SRA recurrence using owning map snapshots. */
template<class Integrator, size_t numberStages>
class SRA : public Integrator {
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
    // (Basilisk issues an integrate() call with timeStep == 0 at initialization.)
    if (timeStep == 0) return;

    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(this->dynPtrs);

    const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps = this->noiseIndexMaps();
    const size_t m = stateIdToNoiseIndexMaps.size();
    const Eigen::Index noiseCount = static_cast<Eigen::Index>(m);

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const Eigen::VectorXd& dW = sample.dW;
    const Eigen::VectorXd& dZ = sample.dZ;

    const double sqrt3 = std::sqrt(3.0);
    Eigen::VectorXd chi2(noiseCount); // I_(1,0)/h
    for (size_t k = 0; k < m; k++) {
        const Eigen::Index eigenK = static_cast<Eigen::Index>(k);
        chi2(eigenK) = (dW(eigenK) + dZ(eigenK) / sqrt3) / 2.0;
    }

    // f_H0[i]     = f(t_n + c0[i] h, H0[i])
    // g_Hk[k][i]  = g_k(t_n + c1[i] h)   (state independent, but evaluated at the stage time)
    std::array<ExtendedStateVector, numberStages> f_H0;
    std::vector<std::array<ExtendedStateVector, numberStages>> g_Hk(m);

    // i = 0: H0[0] == y_n
    f_H0.at(0) = this->computeDerivatives(currentTime + this->coefficients.c0.at(0) * timeStep, timeStep);
    {
        std::vector<ExtendedStateVector> diffs = this->computeDiffusions(
            currentTime + this->coefficients.c1.at(0) * timeStep, timeStep, stateIdToNoiseIndexMaps);
        for (size_t k = 0; k < m; k++) {
            g_Hk.at(k).at(0) = std::move(diffs.at(k));
        }
    }

    for (size_t i = 1; i < numberStages; i++) {
        // H0[i] = y_n + h sum_j A0[i][j] f(H0[j]) + sum_k chi2[k] sum_j B0[i][j] g_k(t_n + c1[j] h)
        currentState.setStates(this->dynPtrs);
        this->scaledSum(this->coefficients.A0.at(i), f_H0, i).setDerivatives(this->dynPtrs);
        for (size_t k = 0; k < m; k++) {
            this->scaledSum(this->coefficients.B0.at(i), g_Hk.at(k), i)
                .setDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps.at(k));
        }
        this->propagateStateWithCachedNoise(timeStep, chi2);

        f_H0.at(i) = this->computeDerivatives(currentTime + this->coefficients.c0.at(i) * timeStep, timeStep);
        // Diffusion is state-independent, but must be sampled at the stage time c1[i].
        {
            std::vector<ExtendedStateVector> diffs = this->computeDiffusions(
                currentTime + this->coefficients.c1.at(i) * timeStep, timeStep, stateIdToNoiseIndexMaps);
            for (size_t k = 0; k < m; k++) {
                g_Hk.at(k).at(i) = std::move(diffs.at(k));
            }
        }
    }

    // y_{n+1} = y_n + h sum_i alpha[i] f(H0[i])
    //               + sum_k [ (beta1 . g_k) dW[k] + (beta2 . g_k) chi2[k] ]
    currentState.setStates(this->dynPtrs);
    this->scaledSum(this->coefficients.alpha, f_H0, numberStages).setDerivatives(this->dynPtrs);
    for (size_t k = 0; k < m; k++) {
        this->scaledSum(this->coefficients.beta1, g_Hk.at(k), numberStages)
            .setDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    this->propagateStateWithCachedNoise(timeStep, dW);

    for (size_t k = 0; k < m; k++) {
        this->scaledSum(this->coefficients.beta2, g_Hk.at(k), numberStages)
            .setDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    this->propagateStateWithCachedNoise(0, chi2);

    // The this->dynPtrs now hold y_{n+1}.
}
};
} // namespace stochasticReference
#endif
