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
#include "../_GeneralModuleFiles/flatStochasticWorkspace.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "../_GeneralModuleFiles/stochasticTableauValidation.h"

#include <array>
#include <cmath>
#include <stdint.h>

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

  protected:
    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    void integrateImpl(double currentTime, double timeStep) override;

    void bindStochasticMethodStorage() override
    {
        this->flatWorkspace.bind(this->stochasticObjectDescriptors(),
                                 this->stochasticNoiseBindings(),
                                 this->stochasticNoiseSlots(),
                                 numberStages,
                                 numberStages,
                                 1);
    }

    /** Coefficients to be used in the method */
    const SRACoefficients<numberStages> coefficients;
    FlatStochasticWorkspace flatWorkspace; ///< Owned stage and candidate buffers reused across steps.
};

template <size_t numberStages>
svIntegratorStrongStochasticRungeKuttaSRA<numberStages>::svIntegratorStrongStochasticRungeKuttaSRA(
    DynamicObject* dynIn, const SRACoefficients<numberStages>& coefficients)
    : StochasticRKIntegratorBase(dynIn), coefficients(coefficients)
{
    stochastic_tableau::validateExplicitMatrix(this->coefficients.A0);
    stochastic_tableau::validateExplicitMatrix(this->coefficients.B0);
    stochastic_tableau::validateFinite(this->coefficients.alpha);
    stochastic_tableau::validateFinite(this->coefficients.beta1);
    stochastic_tableau::validateFinite(this->coefficients.beta2);
    stochastic_tableau::validateFinite(this->coefficients.c0);
    stochastic_tableau::validateFinite(this->coefficients.c1);
}

template<size_t numberStages>
void
svIntegratorStrongStochasticRungeKuttaSRA<numberStages>::integrateImpl(double currentTime, double timeStep)
{
    if (timeStep == 0.0) {
        return;
    }

    this->gatherStochasticStates();
    this->generateNoise(timeStep);

    auto& workspace = this->flatWorkspace; ///< Owned stage and candidate buffers reused across steps.
    const auto& c = this->coefficients;
    const size_t noiseCount = this->globalNoiseCount();
    const double squareRootThree = std::sqrt(3.0);
    Eigen::VectorXd& chi2 = workspace.vector(0);
    for (size_t noise = 0; noise < noiseCount; ++noise) {
        const Eigen::Index index = static_cast<Eigen::Index>(noise);
        chi2(index) = (this->flatDW()(index) + this->flatDZ()(index) / squareRootThree) / 2.0;
    }

    auto evaluateDrift = [&](double time, size_t stage) {
        this->evaluateDerivatives(time, timeStep);
        workspace.captureDrift(stage);
    };
    auto evaluateDiffusions = [&](double time, size_t stage) {
        this->evaluateDiffusions(time, timeStep);
        workspace.captureAllDiffusions(stage);
    };
    auto buildCandidate = [&](double driftStep, const Eigen::VectorXd& diffusionSteps) {
        this->buildStochasticCandidate(
          this->stochasticAcceptedState(), workspace.drift(), driftStep, workspace.diffusions(), diffusionSteps);
    };

    try {
        evaluateDrift(currentTime + c.c0.at(0) * timeStep, 0);
        evaluateDiffusions(currentTime + c.c1.at(0) * timeStep, 0);

        for (size_t stage = 1; stage < numberStages; ++stage) {
            workspace.writeDrift(c.A0.at(stage), stage);
            for (size_t noise = 0; noise < noiseCount; ++noise) {
                workspace.writeDiffusion(noise, c.B0.at(stage), stage);
            }
            buildCandidate(timeStep, chi2);
            evaluateDrift(currentTime + c.c0.at(stage) * timeStep, stage);
            evaluateDiffusions(currentTime + c.c1.at(stage) * timeStep, stage);
        }

        workspace.writeDrift(c.alpha, numberStages);
        for (size_t noise = 0; noise < noiseCount; ++noise) {
            workspace.writeDiffusion(noise, c.beta1, numberStages);
        }
        const bool coalesceFinalUpdate = this->stochasticUpdatesAreAllEuclidean();
        if (coalesceFinalUpdate) {
            this->beginAllEuclideanFinalCandidate(
              this->stochasticAcceptedState(), workspace.drift(), timeStep, workspace.diffusions(), this->flatDW());
        } else {
            buildCandidate(timeStep, this->flatDW());
        }

        if (!coalesceFinalUpdate) {
            this->acceptStochasticCandidate();
        }
        for (size_t noise = 0; noise < noiseCount; ++noise) {
            workspace.writeDiffusion(noise, c.beta2, numberStages);
        }
        if (coalesceFinalUpdate) {
            this->appendAllEuclideanFinalCandidate(workspace.drift(), 0.0, workspace.diffusions(), chi2);
            this->commitAllEuclideanFinalCandidate();
        } else {
            buildCandidate(0.0, chi2);
        }
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}

#endif /* svIntegratorStrongStochasticRungeKuttaSRA_h */
