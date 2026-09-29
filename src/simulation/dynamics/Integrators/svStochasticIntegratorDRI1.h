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

#ifndef svStochasticIntegratorDRI1_h
#define svStochasticIntegratorDRI1_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"

#include <vector>

/** Coefficients for the Debrabant & Roessler DRI1 weak-order-2 method. Names follow
 * the DRI1 tableau. */
struct DRI1Coefficients {
    double a021;   //!< Drift-stage A-matrix coefficient (A0)
    double a031;   //!< Drift-stage A-matrix coefficient (A0)
    double a032;   //!< Drift-stage A-matrix coefficient (A0)
    double a121;   //!< Drift-stage A-matrix coefficient (A1)
    double a131;   //!< Drift-stage A-matrix coefficient (A1)
    double b021;   //!< Diffusion-stage B-matrix coefficient (B0)
    double b031;   //!< Diffusion-stage B-matrix coefficient (B0)
    double b121;   //!< Diffusion-stage B-matrix coefficient (B1)
    double b131;   //!< Diffusion-stage B-matrix coefficient (B1)
    double b221;   //!< Second diffusion-stage B-matrix coefficient (B2)
    double b222;   //!< Second diffusion-stage B-matrix coefficient (B2)
    double b223;   //!< Second diffusion-stage B-matrix coefficient (B2)
    double b231;   //!< Second diffusion-stage B-matrix coefficient (B2)
    double b232;   //!< Second diffusion-stage B-matrix coefficient (B2)
    double b233;   //!< Second diffusion-stage B-matrix coefficient (B2)
    double alpha1; //!< Drift weight vector entry
    double alpha2; //!< Drift weight vector entry
    double alpha3; //!< Drift weight vector entry
    double c02;    //!< Stage time node (drift c0)
    double c03;    //!< Stage time node (drift c0)
    double c12;    //!< Stage time node (diffusion c1)
    double c13;    //!< Stage time node (diffusion c1)
    double beta11; //!< Diffusion weight vector beta^(1) entry
    double beta12; //!< Diffusion weight vector beta^(1) entry
    double beta13; //!< Diffusion weight vector beta^(1) entry
    double beta22; //!< Diffusion weight vector beta^(2) entry
    double beta23; //!< Diffusion weight vector beta^(2) entry
    double beta31; //!< Diffusion weight vector beta^(3) entry
    double beta32; //!< Diffusion weight vector beta^(3) entry
    double beta33; //!< Diffusion weight vector beta^(3) entry
    double beta42; //!< Diffusion weight vector beta^(4) entry
    double beta43; //!< Diffusion weight vector beta^(4) entry
};

/**
 * The svStochasticIntegratorDRI1 class implements the Debrabant-Roessler DRI1 method,
 * an efficient weak second-order (deterministic order 3) Runge-Kutta method for Ito
 * SDEs. It is a good general-purpose weak-order-2 method, handling non-commutative
 * noise via the mixed iterated integral
 * \f$\hat I_{(k,l)} = \tfrac12(\Delta\hat W_k \Delta\hat W_l \mp \sqrt h\,\Delta\hat Z)\f$.
 *
 * This implements the DRI1 method. The random variables
 * are a three-point increment \f$\Delta\hat W\f$ (from the Gaussian \f$\Delta W\f$),
 * its diagonal \f$\chi_1 = (\Delta\hat W^2 - h)/2\f$, and a two-point increment
 * \f$\Delta\hat Z\f$ (from the Gaussian \f$\Delta Z\f$, scale \f$\sqrt h\f$) used only
 * for the cross-noise terms when there is more than one noise source.
 *
 * The step is fully explicit and derivative-free: three drift stages
 * (\f$H_0^{(0)}=x_n, H_2^{(0)}, H_3^{(0)}\f$) and, per noise source, diffusion stages
 * (\f$H_2^{(k)}, H_3^{(k)}\f$ and, for the coupled case, \f$\hat H_2^{(k)},
 * \hat H_3^{(k)}\f$), all evaluated pointwise. See the .cpp for the exact recurrence.
 *
 * @warning Stochastic integration is in beta.
 */
class svStochasticIntegratorDRI1 : public StochasticRKIntegratorBase {
public:
    svStochasticIntegratorDRI1(DynamicObject* dyn); //!< Constructor

    /** When true, the cross-noise (mixed iterated integral) terms are skipped. This
     * is the "non-mixing" DRI1NM variant, which is exact for diagonal noise and more
     * efficient because it avoids the extra stage-state diffusion evaluations. For
     * diagonal noise it produces the same result as the full DRI1. Defaults to false
     * (full DRI1). */
    void setNonMixing(bool enabled);
    /** @brief Return whether mixed noise-source terms are omitted.
     * @return True for the diagonal-noise variant.
     */
    bool getNonMixing() const noexcept { return this->nonMixing; }

  protected:
    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    void integrateImpl(double currentTime, double timeStep) override;

    /** Constructor used by the sibling Roessler weak-order-2 methods (RI1/RI3/RI5/RI6),
     * which share DRI1's step and differ only in their coefficient tableau. */
    svStochasticIntegratorDRI1(DynamicObject* dyn, const DRI1Coefficients& coefficients);

    /** DRI1 coefficients. */
    const DRI1Coefficients coefficients;

    /** Returns the DRI1 coefficient tableau. */
    static DRI1Coefficients getCoefficients();

    /** @brief Allocate the scratch required by this numerical method.
     */
    void bindStochasticMethodStorage() override { this->bindMethodStorage(); }

    /** @brief Size method buffers from the bound state and noise dimensions once.
     */
    void bindMethodStorage();
    /** @brief Copy live drift values into the selected stage column.
     * @param stageIndex Zero-based stage column to read or write.
     */
    void gatherDerivativeStage(size_t stageIndex);
    /** @brief Capture every independent noise source in the selected stage column.
     * @param stageIndex Zero-based stage column to read or write.
     */
    void gatherAllDiffusionStages(size_t stageIndex);
    /** @brief Capture the selected noise source in its packed rows of a stage column.
     * @param globalNoiseIndex Independent noise-source index in the bound topology.
     * @param stageIndex Zero-based stage column to read or write.
     */
    void gatherDiffusionStage(size_t globalNoiseIndex, size_t stageIndex);
    /** @brief Capture live diffusion values for a mixed-integral support stage.
     * @param hatStageIndex Zero-based mixed-integral support stage.
     * @param secondHat Select the second support-stage family when true.
     */
    void gatherHatDiffusions(size_t hatStageIndex, bool secondHat);
    /** @brief Combine two drift stages into combinedDerivative in the given order.
     * @param firstStage First drift stage to read.
     * @param firstFactor Multiplier for the first contribution.
     * @param secondStage Second drift stage to read.
     * @param secondFactor Multiplier for the second contribution.
     */
    void combineTwoDrifts(size_t firstStage, double firstFactor, size_t secondStage, double secondFactor);
    /** @brief Combine drift stages zero, one, and two into combinedDerivative.
     * @param firstFactor Multiplier for the first contribution.
     * @param secondFactor Multiplier for the second contribution.
     * @param thirdFactor Multiplier for the third contribution.
     */
    void combineThreeDrifts(double firstFactor, double secondFactor, double thirdFactor);
    /** @brief Multiply one drift stage into the supplied buffer.
     * @param stageIndex Zero-based stage column to read or write.
     * @param factor Multiplier for the selected stage.
     * @param output Already sized destination buffer.
     */
    void scaleDrift(size_t stageIndex, double factor, Eigen::VectorXd& output);
    /** @brief Scale one source and stage into its rows of combinedDiffusion.
     * @param globalNoiseIndex Independent noise-source index in the bound topology.
     * @param stageIndex Zero-based stage column to read or write.
     * @param factor Multiplier for the selected stage.
     */
    void scaleDiffusion(size_t globalNoiseIndex, size_t stageIndex, double factor);
    /** @brief Combine stages zero, one, and two for one source into combinedDiffusion.
     * @param globalNoiseIndex Independent noise-source index in the bound topology.
     * @param firstFactor Multiplier for the first contribution.
     * @param secondFactor Multiplier for the second contribution.
     * @param thirdFactor Multiplier for the third contribution.
     */
    void combineThreeDiffusions(size_t globalNoiseIndex, double firstFactor, double secondFactor, double thirdFactor);
    /** @brief Copy one source from a mixed-integral support stage into combinedDiffusion.
     * @param globalNoiseIndex Independent noise-source index in the bound topology.
     * @param hatStageIndex Zero-based mixed-integral support stage.
     * @param secondHat Select the second support-stage family when true.
     */
    void copyHatDiffusion(size_t globalNoiseIndex, size_t hatStageIndex, bool secondHat);

    bool methodStorageBound = false; ///< True after all method buffers have been sized.
    Eigen::VectorXd combinedDerivative; ///< Weighted drift combination used to build a candidate.
    Eigen::VectorXd firstDiffusionDrift; ///< Drift combination for the first diffusion-support candidate.
    Eigen::VectorXd secondDiffusionDrift; ///< Drift combination for the second diffusion-support candidate.
    Eigen::VectorXd combinedDiffusion; ///< Weighted diffusion contributions in packed noise-source order.
    Eigen::MatrixXd derivativeStages; ///< Drift scalars by stage; each stage occupies one contiguous column.
    Eigen::MatrixXd diffusionStages; ///< Packed diffusion scalars by stage, grouped by independent source.
    Eigen::MatrixXd firstHatDiffusions; ///< Diffusions sampled at the first family of mixed-integral support states.
    Eigen::MatrixXd secondHatDiffusions; ///< Diffusions sampled at the second family of mixed-integral support states.
    Eigen::VectorXd weakDW; ///< Discrete weak-method increments indexed by independent source.
    Eigen::VectorXd diagonalIntegral; ///< Diagonal iterated-integral terms indexed by independent source.
    Eigen::VectorXd weakDZ; ///< Auxiliary discrete increments used for mixed noise-source terms.
    Eigen::VectorXd pseudoSteps; ///< Temporary per-source noise weights for candidate construction.

  private:
    bool nonMixing = false;
};

/** @brief DRI1NM: the "non-mixing" (diagonal-noise) variant of DRI1.
 *
 * @warning Stochastic integration is in beta.
 *
 * Implementation of the DRI1NM method. Identical to DRI1 but skips
 * the cross-noise mixed-integral terms, which is exact (and more efficient) for
 * diagonal/scalar-noise problems.
 */
class svStochasticIntegratorDRI1NM : public svStochasticIntegratorDRI1 {
public:
    /** Constructor. */
    explicit svStochasticIntegratorDRI1NM(DynamicObject* dyn) : svStochasticIntegratorDRI1(dyn)
    {
        this->setNonMixing(true);
    }
};

/** @brief RI1: Roessler weak-order-2 Ito method (Roessler 2009), any noise.
 *
 * @warning Stochastic integration is in beta.
 *
 * Weak order 2, deterministic order 3, for Ito SDEs of any noise structure (diagonal,
 * non-diagonal, non-commutative, scalar-additive). Shares DRI1's step; differs only in
 * its coefficient tableau (the RI1 tableau).
 */
class svStochasticIntegratorRI1 : public svStochasticIntegratorDRI1 {
public:
    /** Constructor. */
    svStochasticIntegratorRI1(DynamicObject* dyn);
private:
    /** Returns the RI1 coefficient tableau. */
    static DRI1Coefficients getCoefficients();
};

/** @brief RI3: Roessler weak-order-2 Ito method (Roessler 2009), any noise. */
class svStochasticIntegratorRI3 : public svStochasticIntegratorDRI1 {
public:
    /** Constructor. */
    svStochasticIntegratorRI3(DynamicObject* dyn);
private:
    /** Returns the RI3 coefficient tableau. */
    static DRI1Coefficients getCoefficients();
};

/** @brief RI5: Roessler weak-order-2 Ito method (Roessler 2009), any noise. */
class svStochasticIntegratorRI5 : public svStochasticIntegratorDRI1 {
public:
    /** Constructor. */
    svStochasticIntegratorRI5(DynamicObject* dyn);
private:
    /** Returns the RI5 coefficient tableau. */
    static DRI1Coefficients getCoefficients();
};

/** @brief RI6: Roessler weak-order-2 Ito method (Roessler 2009), any noise. */
class svStochasticIntegratorRI6 : public svStochasticIntegratorDRI1 {
public:
    /** Constructor. */
    svStochasticIntegratorRI6(DynamicObject* dyn);
private:
    /** Returns the RI6 coefficient tableau. */
    static DRI1Coefficients getCoefficients();
};

#endif /* svStochasticIntegratorDRI1_h */
