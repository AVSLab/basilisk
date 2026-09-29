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
#include "../_GeneralModuleFiles/flatStochasticWorkspace.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "../_GeneralModuleFiles/stochasticTableauValidation.h"

#include <array>
#include <cmath>
#include <stdint.h>

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
                                 3);
    }

    /** Coefficients to be used in the method */
    const SRICoefficients<numberStages> coefficients;
    FlatStochasticWorkspace flatWorkspace; ///< Owned stage and candidate buffers reused across steps.
};

template <size_t numberStages>
svIntegratorStrongStochasticRungeKuttaSRI<numberStages>::svIntegratorStrongStochasticRungeKuttaSRI(
    DynamicObject* dynIn, const SRICoefficients<numberStages>& coefficients)
    : StochasticRKIntegratorBase(dynIn), coefficients(coefficients)
{
    stochastic_tableau::validateExplicitMatrix(this->coefficients.A0);
    stochastic_tableau::validateExplicitMatrix(this->coefficients.A1);
    stochastic_tableau::validateExplicitMatrix(this->coefficients.B0);
    stochastic_tableau::validateExplicitMatrix(this->coefficients.B1);
    stochastic_tableau::validateFinite(this->coefficients.alpha);
    stochastic_tableau::validateFinite(this->coefficients.beta1);
    stochastic_tableau::validateFinite(this->coefficients.beta2);
    stochastic_tableau::validateFinite(this->coefficients.beta3);
    stochastic_tableau::validateFinite(this->coefficients.beta4);
    stochastic_tableau::validateFinite(this->coefficients.c0);
    stochastic_tableau::validateFinite(this->coefficients.c1);
}

template<size_t numberStages>
void
svIntegratorStrongStochasticRungeKuttaSRI<numberStages>::integrateImpl(double currentTime, double timeStep)
{
    if (timeStep == 0.0) {
        return;
    }

    this->gatherStochasticStates();
    this->generateNoise(timeStep);

    auto& workspace = this->flatWorkspace; ///< Owned stage and candidate buffers reused across steps.
    const auto& c = this->coefficients;
    const size_t noiseCount = this->globalNoiseCount();
    const double squareRootStep = std::sqrt(timeStep);
    const double squareRootThree = std::sqrt(3.0);
    Eigen::VectorXd& chi1 = workspace.vector(0);
    Eigen::VectorXd& chi2 = workspace.vector(1);
    Eigen::VectorXd& chi3 = workspace.vector(2);

    for (size_t noise = 0; noise < noiseCount; ++noise) {
        const Eigen::Index index = static_cast<Eigen::Index>(noise);
        const double dW = this->flatDW()(index);
        chi1(index) = (dW * dW - timeStep) / (2.0 * squareRootStep);
        chi2(index) = (dW + this->flatDZ()(index) / squareRootThree) / 2.0;
        chi3(index) = (dW * dW * dW - 3.0 * dW * timeStep) / (6.0 * timeStep);
    }

    auto evaluateDrift = [&](double time, size_t stage) {
        this->evaluateDerivatives(time, timeStep);
        workspace.captureDrift(stage);
    };
    auto evaluateAllDiffusions = [&](double time, size_t stage) {
        this->evaluateDiffusions(time, timeStep);
        workspace.captureAllDiffusions(stage);
    };
    auto evaluateDiffusion = [&](double time, size_t slot, size_t stage) {
        this->evaluateDiffusions(time, timeStep);
        workspace.captureDiffusion(slot, stage);
    };
    auto buildCandidate =
      [&](double driftStep, const Eigen::VectorXd& diffusionSteps, size_t slotBegin, size_t slotEnd) {
          this->buildStochasticCandidate(this->stochasticAcceptedState(),
                                         workspace.drift(),
                                         driftStep,
                                         workspace.diffusions(),
                                         diffusionSteps,
                                         slotBegin,
                                         slotEnd);
      };

    try {
        evaluateDrift(currentTime, 0);
        evaluateAllDiffusions(currentTime, 0);

        for (size_t stage = 1; stage < numberStages; ++stage) {
            workspace.writeDrift(c.A0.at(stage), stage);
            for (size_t noise = 0; noise < noiseCount; ++noise) {
                workspace.writeDiffusion(noise, c.B0.at(stage), stage);
            }
            buildCandidate(timeStep, chi2, 0, noiseCount);
            evaluateDrift(currentTime + c.c0.at(stage) * timeStep, stage);

            workspace.writeDrift(c.A1.at(stage), stage);
            this->beginStochasticSparseCandidates(this->stochasticAcceptedState(), workspace.drift(), timeStep);
            for (size_t noise = 0; noise < noiseCount; ++noise) {
                workspace.writeDiffusion(noise, c.B1.at(stage), stage);
                this->buildStochasticNoiseSlotCandidate(workspace.diffusions(), squareRootStep, noise);
                evaluateDiffusion(currentTime + c.c1.at(stage) * timeStep, noise, stage);
            }
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
            buildCandidate(timeStep, this->flatDW(), 0, noiseCount);
        }

        if (!coalesceFinalUpdate) {
            this->acceptStochasticCandidate();
        }
        for (size_t noise = 0; noise < noiseCount; ++noise) {
            workspace.writeDiffusion(noise, c.beta2, numberStages);
        }
        if (coalesceFinalUpdate) {
            this->appendAllEuclideanFinalCandidate(workspace.drift(), 0.0, workspace.diffusions(), chi1);
        } else {
            buildCandidate(0.0, chi1, 0, noiseCount);
        }

        if (!coalesceFinalUpdate) {
            this->acceptStochasticCandidate();
        }
        for (size_t noise = 0; noise < noiseCount; ++noise) {
            workspace.writeDiffusion(noise, c.beta3, numberStages);
        }
        if (coalesceFinalUpdate) {
            this->appendAllEuclideanFinalCandidate(workspace.drift(), 0.0, workspace.diffusions(), chi2);
        } else {
            buildCandidate(0.0, chi2, 0, noiseCount);
        }

        if (!coalesceFinalUpdate) {
            this->acceptStochasticCandidate();
        }
        for (size_t noise = 0; noise < noiseCount; ++noise) {
            workspace.writeDiffusion(noise, c.beta4, numberStages);
        }
        if (coalesceFinalUpdate) {
            this->appendAllEuclideanFinalCandidate(workspace.drift(), 0.0, workspace.diffusions(), chi3);
            this->commitAllEuclideanFinalCandidate();
        } else {
            buildCandidate(0.0, chi3, 0, noiseCount);
        }
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}

#endif /* svIntegratorStrongStochasticRungeKuttaSRI_h */
