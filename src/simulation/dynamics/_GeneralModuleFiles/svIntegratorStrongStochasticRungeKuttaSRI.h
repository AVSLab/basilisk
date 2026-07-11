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

#ifndef svIntegratorStrongStochasticRungeKuttaSRI_h
#define svIntegratorStrongStochasticRungeKuttaSRI_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "extendedStateVector.h"

#include <array>
#include <memory>
#include <cmath>
#include <stdint.h>
#include <vector>

/**
 * Stores the coefficients for a Roessler Stochastic Runge-Kutta method for the
 * strong approximation of Ito SDEs with diagonal or scalar noise (an "SRI" method).
 *
 * The Butcher tableau of an ``s``-stage SRI method has the form (Roessler 2010):
 *
 * \f[
 *   \begin{array}{c|cc}
 *      c^{(0)} & A^{(0)} & B^{(0)} \\
 *      c^{(1)} & A^{(1)} & B^{(1)} \\ \hline
 *              & \alpha  & \beta^{(1)}\ \beta^{(2)}\ \beta^{(3)}\ \beta^{(4)}
 *   \end{array}
 * \f]
 *
 * where \f$A^{(0)}, A^{(1)}, B^{(0)}, B^{(1)}\f$ are strictly-lower-triangular
 * matrices (so the method is explicit) and the \f$\beta^{(\cdot)}\f$ are the weight
 * vectors that multiply the different random variables. See
 * ``svIntegratorStrongStochasticRungeKuttaSRI`` for the exact recurrence.
 *
 * See more in the description of svIntegratorStrongStochasticRungeKuttaSRI.
 */
template <size_t numberStages> struct SRICoefficients {

    /** Array with size = numberStages */
    using StageSizedArray = std::array<double, numberStages>;

    /** Square matrix with size = numberStages */
    using StageSizedMatrix = std::array<StageSizedArray, numberStages>;

    // = {} performs zero-initialization
    StageSizedMatrix A0 = {}; /**< drift coefficient matrix for the H0 stages */
    StageSizedMatrix A1 = {}; /**< drift coefficient matrix for the H1 stages */
    StageSizedMatrix B0 = {}; /**< diffusion coefficient matrix for the H0 stages */
    StageSizedMatrix B1 = {}; /**< diffusion coefficient matrix for the H1 stages */

    StageSizedArray alpha = {}; /**< drift weights */
    StageSizedArray beta1 = {}; /**< diffusion weights multiplying dW */
    StageSizedArray beta2 = {}; /**< diffusion weights multiplying I(1,1)/sqrt(h) */
    StageSizedArray beta3 = {}; /**< diffusion weights multiplying I(1,0)/h */
    StageSizedArray beta4 = {}; /**< diffusion weights multiplying I(1,1,1)/h */

    /** "c0" node array; the row sums of A0: \f$c^{(0)}_i = \sum_j A^{(0)}_{ij}\f$ */
    StageSizedArray c0 = {};

    /** "c1" node array; the row sums of A1: \f$c^{(1)}_i = \sum_j A^{(1)}_{ij}\f$ */
    StageSizedArray c1 = {};
};

/**
 * The svIntegratorStrongStochasticRungeKuttaSRI class implements a state integrator
 * that provides strong (order 1.5) solutions to problems with stochastic dynamics
 * (SDEs) that have diagonal or scalar noise.
 *
 * The method is the Roessler "SRI" family, described in:
 *
 *     Roessler A., "Runge-Kutta Methods for the Strong Approximation of Solutions of
 *     Stochastic Differential Equations", SIAM J. Numer. Anal., 48 (3), pp. 922-952.
 *     https://doi.org/10.1137/09076636X
 *
 * This is an implementation of the ``SRIW1``/``SOSRI`` family of Roessler SRI methods.
 * The recurrence below is the (unrolled) SRIW1/four-stage-SRI step.
 *
 * As with the other Basilisk stochastic integrators, the method is written for
 * autonomous systems (where f and g do not depend explicitly on time). Basilisk
 * supports non-autonomous systems by treating time as an extra state (with drift 1
 * and diffusion 0), which is why the drift stages are evaluated at the shifted times
 * \f$t_n + c^{(0)}_i h\f$ and the diffusion stages at \f$t_n + c^{(1)}_i h\f$.
 *
 * With \f$s\f$ stages, \f$m\f$ (diagonal) noise sources, time step \f$h\f$ and
 * per-noise-source random variables:
 *
 * \f[
 *   \hat{I}_{(1)} = \Delta W, \quad
 *   \chi_1 = \frac{\Delta W^2 - h}{2\sqrt h}, \quad
 *   \chi_2 = \frac{\Delta W + \Delta Z/\sqrt 3}{2}, \quad
 *   \chi_3 = \frac{\Delta W^3 - 3 \Delta W\,h}{6 h},
 * \f]
 *
 * (where \f$\Delta W \sim N(0,h)\f$ and \f$\Delta Z \sim N(0,h)\f$ is an independent
 * Wiener increment), the method computes, for each noise source \f$k\f$:
 *
 * \code{.txt}
 *  --- stage definitions (explicit; sums run over j = 1..i-1) ---
 *  for i = 1..s:
 *      H0[i] = y_n + h  * sum_j A0[i][j] f(t_n + c0[j] h, H0[j])
 *                  +      sum_j B0[i][j] chi2[k] g_k(t_n + c1[j] h, H1[j])
 *      H1[i] = y_n + h  * sum_j A1[i][j] f(t_n + c0[j] h, H0[j])
 *                  + sqrt(h) sum_j B1[i][j] g_k(t_n + c1[j] h, H1[j])
 *
 *  --- state update ---
 *  y_{n+1} = y_n + h * sum_i alpha[i] f(t_n + c0[i] h, H0[i])
 *                +     sum_i ( beta1[i] dW[k] + beta2[i] chi1[k] ) g_k(t_n + c1[i] h, H1[i])
 *                +     sum_i ( beta3[i] chi2[k] + beta4[i] chi3[k] ) g_k(t_n + c1[i] h, H1[i])
 * \endcode
 *
 * Note that the H0 (drift) stages are shared across noise sources, while the H1
 * (diffusion) stages are computed independently for each noise source \f$k\f$ (this
 * is what restricts the method to diagonal/scalar noise: each state is driven by its
 * own noise source through its own diffusion stage).
 *
 * @warning Stochastic integration is in beta.
 */
template <size_t numberStages>
class svIntegratorStrongStochasticRungeKuttaSRI : public StochasticRKIntegratorBase {
public:
    static_assert(numberStages > 0, "One cannot declare Runge Kutta integrators of stage 0");

    /** Creates an SRI integrator for the given DynamicObject using the passed coefficients. */
    svIntegratorStrongStochasticRungeKuttaSRI(DynamicObject* dynIn,
                                              const SRICoefficients<numberStages>& coefficients);

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;

protected:
    /** Coefficients to be used in the method */
    const SRICoefficients<numberStages> coefficients;

    /** Utility: result = sum_{i=0}^{length-1} factors[i] * vectors[i]. */
    ExtendedStateVector scaledSum(const std::array<double, numberStages>& factors,
                                  const std::array<ExtendedStateVector, numberStages>& vectors,
                                  size_t length);
};

template <size_t numberStages>
svIntegratorStrongStochasticRungeKuttaSRI<numberStages>::svIntegratorStrongStochasticRungeKuttaSRI(
    DynamicObject* dynIn, const SRICoefficients<numberStages>& coefficients)
    : StochasticRKIntegratorBase(dynIn), coefficients(coefficients)
{
}

template <size_t numberStages>
void svIntegratorStrongStochasticRungeKuttaSRI<numberStages>::integrate(double currentTime,
                                                                        double timeStep)
{
    // A zero-duration step advances nothing and must not consume a noise sample.
    // (Basilisk issues an integrate() call with timeStep == 0 at initialization.)
    if (timeStep == 0) return;

    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);

    // Map (ExtendedStateId -> local noise index) for each of the m noise sources (cached).
    const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps = noiseIndexMaps();
    const size_t m = stateIdToNoiseIndexMaps.size();

    // Draw the random variables for this step (needs dW and dZ per noise source).
    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const Eigen::VectorXd& dW = sample.dW;
    const Eigen::VectorXd& dZ = sample.dZ;

    const double h = timeStep;
    const double sqh = std::sqrt(h);
    const double sqrt3 = std::sqrt(3.0);

    // Iterated-integral approximations, one entry per noise source.
    Eigen::VectorXd chi1(m); // I_(1,1)/sqrt(h)
    Eigen::VectorXd chi2(m); // I_(1,0)/h
    Eigen::VectorXd chi3(m); // I_(1,1,1)/h
    for (size_t k = 0; k < m; k++) {
        chi1(k) = (dW(k) * dW(k) - h) / (2.0 * sqh);
        chi2(k) = (dW(k) + dZ(k) / sqrt3) / 2.0;
        chi3(k) = (dW(k) * dW(k) * dW(k) - 3.0 * dW(k) * h) / (6.0 * h);
    }

    // f_H0[i]      = f(t_n + c0[i] h, H0[i])                  for i = 0..s-1
    // g_Hk[k][i]   = g_k(t_n + c1[i] h, H1[i] for source k)   for i = 0..s-1; k = 0..m-1
    std::array<ExtendedStateVector, numberStages> f_H0;
    std::vector<std::array<ExtendedStateVector, numberStages>> g_Hk(m);

    // i = 0: H0[0] == H1[0] == y_n (all A/B rows are strictly lower triangular).
    f_H0.at(0) = computeDerivatives(currentTime, timeStep);
    {
        std::vector<ExtendedStateVector> diffs =
            computeDiffusions(currentTime, timeStep, stateIdToNoiseIndexMaps);
        for (size_t k = 0; k < m; k++) {
            g_Hk.at(k).at(0) = std::move(diffs.at(k));
        }
    }

    // Remaining stages.
    for (size_t i = 1; i < numberStages; i++) {
        // --- H0[i] (drift stage, shared across noise sources) ---
        // H0[i] = y_n + h sum_j A0[i][j] f(H0[j]) + sum_k chi2[k] sum_j B0[i][j] g_k(H1[j])
        currentState.setStates(dynPtrs);
        scaledSum(coefficients.A0.at(i), f_H0, i).setDerivatives(dynPtrs);
        for (size_t k = 0; k < m; k++) {
            scaledSum(coefficients.B0.at(i), g_Hk.at(k), i)
                .setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
        }
        // pseudo time step for the diffusion term is chi2[k]
        propagateState(timeStep, chi2, stateIdToNoiseIndexMaps);
        f_H0.at(i) = computeDerivatives(currentTime + coefficients.c0.at(i) * timeStep, timeStep);

        // --- H1[i] (diffusion stage, one per noise source k) ---
        // H1[i] = y_n + h sum_j A1[i][j] f(H0[j]) + sqrt(h) sum_j B1[i][j] g_k(H1[j])
        for (size_t k = 0; k < m; k++) {
            currentState.setStates(dynPtrs);
            scaledSum(coefficients.A1.at(i), f_H0, i).setDerivatives(dynPtrs);
            scaledSum(coefficients.B1.at(i), g_Hk.at(k), i)
                .setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));

            // Only noise source k participates (pseudo step sqrt(h)); all others zero.
            Eigen::VectorXd pseudoTimeStep = Eigen::VectorXd::Zero(m);
            pseudoTimeStep(k) = sqh;
            propagateState(timeStep, pseudoTimeStep, stateIdToNoiseIndexMaps);

            g_Hk.at(k).at(i) =
                computeDiffusion(currentTime + coefficients.c1.at(i) * timeStep, timeStep,
                                 stateIdToNoiseIndexMaps.at(k));  // single-source diffusion
        }
    }

    // --- State update ---
    // y_{n+1} = y_n + h sum_i alpha[i] f(H0[i])
    //               + sum_k [ (beta1 . g_Hk[k]) dW[k] + (beta2 . g_Hk[k]) chi1[k]
    //                        + (beta3 . g_Hk[k]) chi2[k] + (beta4 . g_Hk[k]) chi3[k] ]
    // We accumulate the four diffusion contributions with four propagateState calls
    // (each with h = 0 except the first, so the drift is only counted once).
    currentState.setStates(dynPtrs);
    scaledSum(coefficients.alpha, f_H0, numberStages).setDerivatives(dynPtrs);
    for (size_t k = 0; k < m; k++) {
        scaledSum(coefficients.beta1, g_Hk.at(k), numberStages)
            .setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateState(timeStep, dW, stateIdToNoiseIndexMaps);

    for (size_t k = 0; k < m; k++) {
        scaledSum(coefficients.beta2, g_Hk.at(k), numberStages)
            .setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateState(0, chi1, stateIdToNoiseIndexMaps);

    for (size_t k = 0; k < m; k++) {
        scaledSum(coefficients.beta3, g_Hk.at(k), numberStages)
            .setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateState(0, chi2, stateIdToNoiseIndexMaps);

    for (size_t k = 0; k < m; k++) {
        scaledSum(coefficients.beta4, g_Hk.at(k), numberStages)
            .setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
    }
    propagateState(0, chi3, stateIdToNoiseIndexMaps);

    // The dynPtrs now hold y_{n+1}.
}

template <size_t numberStages>
inline ExtendedStateVector svIntegratorStrongStochasticRungeKuttaSRI<numberStages>::scaledSum(
    const std::array<double, numberStages>& factors,
    const std::array<ExtendedStateVector, numberStages>& vectors, size_t length)
{
    ExtendedStateVector result = vectors.at(0) * factors.at(0);
    for (size_t i = 1; i < length; i++) {
        if (factors.at(i) == 0) continue;
        result += vectors.at(i) * factors.at(i);
    }
    return result;
}

#endif /* svIntegratorStrongStochasticRungeKuttaSRI_h */
