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

#ifndef svIntegratorStrongStochasticRungeKuttaSRA_h
#define svIntegratorStrongStochasticRungeKuttaSRA_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "extendedStateVector.h"

#include <array>
#include <cmath>
#include <memory>
#include <stdint.h>
#include <vector>

/**
 * Stores the coefficients for a Roessler Stochastic Runge-Kutta method for the
 * strong approximation of SDEs with additive noise (an "SRA" method).
 *
 * "Additive noise" means the diffusion \f$g(t)\f$ does not depend on the state.
 * The Butcher tableau of an ``s``-stage SRA method has the form (Roessler 2010):
 *
 * \f[
 *   \begin{array}{c|cc}
 *      c^{(0)} & A^{(0)} & B^{(0)} \\
 *      c^{(1)} & & \\ \hline
 *              & \alpha  & \beta^{(1)}\ \beta^{(2)}
 *   \end{array}
 * \f]
 *
 * where \f$A^{(0)}, B^{(0)}\f$ are strictly-lower-triangular (so the method is
 * explicit). Because the noise is additive there is only a single family of
 * stages. See ``svIntegratorStrongStochasticRungeKuttaSRA`` for the recurrence.
 */
template <size_t numberStages> struct SRACoefficients {

    /** Array with size = numberStages */
    using StageSizedArray = std::array<double, numberStages>;

    /** Square matrix with size = numberStages */
    using StageSizedMatrix = std::array<StageSizedArray, numberStages>;

    // = {} performs zero-initialization
    StageSizedMatrix A0 = {}; /**< drift coefficient matrix */
    StageSizedMatrix B0 = {}; /**< diffusion coefficient matrix (multiplies chi2 = I(1,0)/h) */

    StageSizedArray alpha = {}; /**< drift weights */
    StageSizedArray beta1 = {}; /**< diffusion weights multiplying dW */
    StageSizedArray beta2 = {}; /**< diffusion weights multiplying chi2 = I(1,0)/h */

    /** "c0" node array; the row sums of A0: \f$c^{(0)}_i = \sum_j A^{(0)}_{ij}\f$ */
    StageSizedArray c0 = {};

    /** "c1" node array, used to evaluate the (state-independent) diffusion at the
     * correct times: \f$g(t_n + c^{(1)}_i h)\f$. */
    StageSizedArray c1 = {};
};

/**
 * The svIntegratorStrongStochasticRungeKuttaSRA class implements a state integrator
 * that provides strong (order up to 1.5) solutions to problems with stochastic
 * dynamics (SDEs) that have additive noise (diffusion independent of the state).
 *
 * The method is the Roessler "SRA" family, described in:
 *
 *     Roessler A., "Runge-Kutta Methods for the Strong Approximation of Solutions of
 *     Stochastic Differential Equations", SIAM J. Numer. Anal., 48 (3), pp. 922-952.
 *     https://doi.org/10.1137/09076636X
 *
 * This is an implementation of the Roessler ``SRA1``/``SOSRA`` family.
 * The recurrence below follows the generic ``SRA`` step.
 *
 * With \f$s\f$ stages, \f$m\f$ noise sources, time step \f$h\f$ and per-noise-source
 * random variables \f$\Delta W \sim N(0,h)\f$ and \f$\chi_2 = (\Delta W + \Delta
 * Z/\sqrt 3)/2\f$ (with \f$\Delta Z \sim N(0,h)\f$ independent):
 *
 * \code{.txt}
 *  --- stage definitions (explicit; sums run over j = 1..i-1) ---
 *  for i = 1..s:
 *      H0[i] = y_n + h * sum_j A0[i][j] f(t_n + c0[j] h, H0[j])
 *                  +     sum_j B0[i][j] chi2[k] g_k(t_n + c1[j] h)
 *
 *  --- state update ---
 *  y_{n+1} = y_n + h * sum_i alpha[i] f(t_n + c0[i] h, H0[i])
 *                +     sum_k [ (beta1 . g_k) dW[k] + (beta2 . g_k) chi2[k] ]
 * \endcode
 *
 * where g_k is the (state-independent) diffusion for noise source k, evaluated at
 * the stage times t_n + c1[i] h.
 *
 * @warning Stochastic integration is in beta.
 */
template <size_t numberStages>
class svIntegratorStrongStochasticRungeKuttaSRA : public StochasticRKIntegratorBase {
public:
    static_assert(numberStages > 0, "One cannot declare Runge Kutta integrators of stage 0");

    /** Creates an SRA integrator for the given DynamicObject using the passed coefficients. */
    svIntegratorStrongStochasticRungeKuttaSRA(DynamicObject* dynIn,
                                              const SRACoefficients<numberStages>& coefficients);

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;

protected:
    /** Coefficients to be used in the method */
    const SRACoefficients<numberStages> coefficients;

    /** Utility: result = sum_{i=0}^{length-1} factors[i] * vectors[i]. */
    ExtendedStateVector scaledSum(const std::array<double, numberStages>& factors,
                                  const std::array<ExtendedStateVector, numberStages>& vectors,
                                  size_t length);
};

template <size_t numberStages>
svIntegratorStrongStochasticRungeKuttaSRA<numberStages>::svIntegratorStrongStochasticRungeKuttaSRA(
    DynamicObject* dynIn, const SRACoefficients<numberStages>& coefficients)
    : StochasticRKIntegratorBase(dynIn), coefficients(coefficients)
{
}

template <size_t numberStages>
void svIntegratorStrongStochasticRungeKuttaSRA<numberStages>::integrate(double currentTime,
                                                                        double timeStep)
{
    // A zero-duration step advances nothing and must not consume a noise sample.
    // (Basilisk issues an integrate() call with timeStep == 0 at initialization.)
    if (timeStep == 0) return;

    const ExtendedStateVector currentState = ExtendedStateVector::fromStates(dynPtrs);

    const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps = noiseIndexMaps();
    const size_t m = stateIdToNoiseIndexMaps.size();

    const GaussianNoiseSample sample = this->rvGenerator->generate(m, timeStep);
    const Eigen::VectorXd& dW = sample.dW;
    const Eigen::VectorXd& dZ = sample.dZ;

    const double sqrt3 = std::sqrt(3.0);
    Eigen::VectorXd chi2(m); // I_(1,0)/h
    for (size_t k = 0; k < m; k++) {
        chi2(k) = (dW(k) + dZ(k) / sqrt3) / 2.0;
    }

    // f_H0[i]     = f(t_n + c0[i] h, H0[i])
    // g_Hk[k][i]  = g_k(t_n + c1[i] h)   (state independent, but evaluated at the stage time)
    std::array<ExtendedStateVector, numberStages> f_H0;
    std::vector<std::array<ExtendedStateVector, numberStages>> g_Hk(m);

    // i = 0: H0[0] == y_n
    f_H0.at(0) = computeDerivatives(currentTime + coefficients.c0.at(0) * timeStep, timeStep);
    {
        std::vector<ExtendedStateVector> diffs = computeDiffusions(
            currentTime + coefficients.c1.at(0) * timeStep, timeStep, stateIdToNoiseIndexMaps);
        for (size_t k = 0; k < m; k++) {
            g_Hk.at(k).at(0) = std::move(diffs.at(k));
        }
    }

    for (size_t i = 1; i < numberStages; i++) {
        // H0[i] = y_n + h sum_j A0[i][j] f(H0[j]) + sum_k chi2[k] sum_j B0[i][j] g_k(t_n + c1[j] h)
        currentState.setStates(dynPtrs);
        scaledSum(coefficients.A0.at(i), f_H0, i).setDerivatives(dynPtrs);
        for (size_t k = 0; k < m; k++) {
            scaledSum(coefficients.B0.at(i), g_Hk.at(k), i)
                .setDiffusions(dynPtrs, stateIdToNoiseIndexMaps.at(k));
        }
        propagateState(timeStep, chi2, stateIdToNoiseIndexMaps);

        f_H0.at(i) = computeDerivatives(currentTime + coefficients.c0.at(i) * timeStep, timeStep);
        // Diffusion is state-independent, but must be sampled at the stage time c1[i].
        {
            std::vector<ExtendedStateVector> diffs = computeDiffusions(
                currentTime + coefficients.c1.at(i) * timeStep, timeStep, stateIdToNoiseIndexMaps);
            for (size_t k = 0; k < m; k++) {
                g_Hk.at(k).at(i) = std::move(diffs.at(k));
            }
        }
    }

    // y_{n+1} = y_n + h sum_i alpha[i] f(H0[i])
    //               + sum_k [ (beta1 . g_k) dW[k] + (beta2 . g_k) chi2[k] ]
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
    propagateState(0, chi2, stateIdToNoiseIndexMaps);

    // The dynPtrs now hold y_{n+1}.
}

template <size_t numberStages>
inline ExtendedStateVector svIntegratorStrongStochasticRungeKuttaSRA<numberStages>::scaledSum(
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

#endif /* svIntegratorStrongStochasticRungeKuttaSRA_h */
