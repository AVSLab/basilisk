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

#ifndef svStochasticIntegratorW2Ito_h
#define svStochasticIntegratorW2Ito_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "../_GeneralModuleFiles/extendedStateVector.h"

#include <memory>
#include <vector>

/** @brief Extended Butcher tableau for the Tang & Xiao weak-order-2 SRK family.
 *
 * The coefficients characterise a specific member of the general weak-order-2
 * stochastic Runge-Kutta scheme for Ito SDEs. The number of stages \f$s\f$ is inferred
 * from the length of ``alpha``. The tableau is
 *
 * \f[
 *   \begin{array}{c|c|c}
 *      & A^{(0)} & B^{(0)} \\ \hline
 *      & A^{(1)} & B^{(1)}\; B^{(2)} \\ \hline
 *      & \alpha & \beta^{(0)}\; \beta^{(1)}
 *   \end{array}
 * \f]
 *
 * with \f$A^{(0)}, A^{(1)}, B^{(0)}, B^{(1)}, B^{(2)}\f$ strictly lower-triangular
 * \f$s\times s\f$ matrices (explicit method) and \f$\alpha, \beta^{(0)}, \beta^{(1)}\f$
 * length-\f$s\f$ weight vectors. */
struct W2ItoCoefficients {
    std::vector<double> alpha;              //!< drift weights, length s
    std::vector<double> beta0;              //!< diffusion weights on the three-point increment, length s
    std::vector<double> beta1;              //!< diffusion weights on the diagonal iterated integral, length s
    std::vector<std::vector<double>> A0;    //!< drift-to-drift stage matrix, s x s
    std::vector<std::vector<double>> B0;    //!< diffusion-to-drift stage matrix, s x s
    std::vector<std::vector<double>> A1;    //!< drift-to-diffusion stage matrix, s x s
    std::vector<std::vector<double>> B1;    //!< self-noise diffusion-to-diffusion stage matrix, s x s
    std::vector<std::vector<double>> B2;    //!< cross-noise diffusion-to-diffusion stage matrix, s x s

    /** Number of stages, inferred from the length of ``alpha``. */
    size_t numStages() const { return this->alpha.size(); }

    /** Drift stage node \f$c^{(0)}_i = \sum_j A^{(0)}_{ij}\f$ (row sum of A0). */
    double c0(size_t i) const
    {
        double sum = 0.0;
        for (double v : this->A0.at(i)) sum += v;
        return sum;
    }

    /** Diffusion stage node \f$c^{(1)}_i = \sum_j A^{(1)}_{ij}\f$ (row sum of A1). */
    double c1(size_t i) const
    {
        double sum = 0.0;
        for (double v : this->A1.at(i)) sum += v;
        return sum;
    }
};

/**
 * The svStochasticIntegratorW2Ito class implements the Tang & Xiao efficient weak
 * second-order (deterministic order 3+) Runge-Kutta family for Ito SDEs.
 *
 *     Tang, X., Xiao, A. "Efficient weak second-order stochastic Runge-Kutta methods
 *     for Ito stochastic differential equations", BIT Numer. Math. 57, 241-260 (2017).
 *     https://doi.org/10.1007/s10543-016-0618-9
 *
 * This is the shared base for the concrete W2Ito1 and W2Ito2 methods, which differ only
 * in their coefficient tableau (see svStochasticIntegratorW2Ito1 and
 * svStochasticIntegratorW2Ito2). It shares the pluggable noise-generator architecture of
 * the other native stochastic integrators, so the same prescribed-noise test harness
 * drives every method unchanged.
 *
 * For an \f$s\f$-stage tableau the step (paper eq. 3.1) is
 *
 * \f[
 *   y_{n+1} = y_n + \sum_i \alpha_i f(H^{(0)}_i)\,h
 *           + \sum_i \sum_k \beta^{(0)}_i g_k(H^{(k)}_i)\,\hat I_k
 *           + \sum_i \sum_k \beta^{(1)}_i g_k(H^{(k)}_i)\,\hat I_{(k,k)}
 * \f]
 *
 * with drift stages \f$H^{(0)}_i\f$ and per-noise-source diffusion stages
 * \f$H^{(k)}_i\f$. The driving random variables are a three-point increment
 * \f$\hat I_k \in \{\pm\sqrt{3h}, 0\}\f$ (from the Gaussian \f$\Delta W_k\f$), a
 * two-point scale \f$\xi = \eta_1\sqrt h\f$ (from the Gaussian \f$\Delta Z_0\f$), and the
 * mixed iterated integrals
 * \f$\hat I_{(k,l)} = \tfrac12(\hat I_l \mp \eta_2 \hat I_l)\f$ for \f$k\neq l\f$ (using a
 * second two-point sign \f$\eta_2\f$ from \f$\Delta Z_1\f$) and
 * \f$\hat I_{(k,k)} = \tfrac12(\hat I_k^2/\xi - \xi)\f$ on the diagonal. The step is
 * explicit and derivative-free.
 *
 * @warning Stochastic integration is in beta.
 */
class svStochasticIntegratorW2Ito : public StochasticRKIntegratorBase {
public:
    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;

protected:
    /** Constructor used by the concrete W2Ito methods, which supply their tableau. */
    svStochasticIntegratorW2Ito(DynamicObject* dyn, const W2ItoCoefficients& coefficients);

    /** The extended Butcher tableau of the specific W2Ito method. */
    const W2ItoCoefficients coefficients;

    /** Returns sum_{j<length} factors[j]*vectors[j] (skipping zero factors). */
    ExtendedStateVector scaledSum(const std::vector<double>& factors,
                                  const std::vector<ExtendedStateVector>& vectors,
                                  size_t length);
};

#endif /* svStochasticIntegratorW2Ito_h */
