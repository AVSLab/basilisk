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

#ifndef svIntegratorWeakSIESME_h
#define svIntegratorWeakSIESME_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "extendedStateVector.h"

#include <cmath>
#include <memory>
#include <vector>

/**
 * Coefficients for the Tocino & Vigo-Aguiar weak second-order stochastic integrators
 * SIEA / SMEA / SIEB / SMEB (the "improved/modified Euler" family).
 *
 * The naming follows the SIEA/SMEA/SIEB/SMEB tableau.
 */
struct SIESMECoefficients {
    double alpha1 = 0;      //!< drift-stage weight on k0
    double alpha2 = 0;      //!< drift-stage weight on k1
    double gamma1 = 0;      //!< g0 noise weight
    double lambda1 = 0;     //!< g1 noise weight on dW
    double lambda2 = 0;     //!< g1 noise weight on sqrt(h)
    double lambda3 = 0;     //!< g1 noise weight on W2
    double mu1 = 0;         //!< g2 noise weight on dW
    double mu2 = 0;         //!< g2 noise weight on sqrt(h)
    double mu3 = 0;         //!< g2 noise weight on W2
    double mu0 = 0;         //!< drift stage node time
    double mubar0 = 0;      //!< diffusion stage node time
    double lambda0 = 0;     //!< k1 stage drift scaling
    double lambdabar0 = 0;  //!< g1/g2 stage drift scaling
    double nu1 = 0;         //!< k1 stage noise scaling on dW
    double nu2 = 0;         //!< k1 stage noise scaling on the dW^3 term (W3)
    double beta2 = 0;       //!< g1 stage noise scaling on sqrt(h)
    double beta3 = 0;       //!< g1 stage noise scaling on W2
    double delta2 = 0;      //!< g2 stage noise scaling on sqrt(h)
    double delta3 = 0;      //!< g2 stage noise scaling on W2
};

/**
 * The svIntegratorWeakSIESME class implements the Tocino & Vigo-Aguiar family of weak
 * second-order stochastic Runge-Kutta methods (SIEA, SMEA, SIEB, SMEB) for Ito SDEs
 * with diagonal or scalar noise.
 *
 * Implementation of the ``SIEA``/``SMEA``/``SIEB``/``SMEB`` methods. Unlike the
 * Roessler weak methods, this family uses only the
 * raw Gaussian Wiener increment and its polynomial moments (no three-point or
 * two-point discrete random variables):
 *
 * \f[
 *   W_2 = \frac{\Delta W^2}{\sqrt h}, \qquad W_3 = \nu_2\,\frac{\Delta W^3}{h}.
 * \f]
 *
 * The two-stage predictor-corrector step (per noise source \f$k\f$, diagonal noise)
 * is, writing \f$k_0 = f(t_n,x_n)\f$ and \f$g_0 = g(t_n,x_n)\f$:
 *
 * \code{.txt}
 *  k1 = f( t_n + mu0*h,    x_n + lambda0*k0*h + nu1*g0*dW + g0*W3 )
 *  g1 = g( t_n + mubar0*h, x_n + lambdabar0*k0*h + beta2*g0*sqrt(h) + beta3*g0*W2 )
 *  g2 = g( t_n + mubar0*h, x_n + lambdabar0*k0*h + delta2*g0*sqrt(h) + delta3*g0*W2 )
 *
 *  x_{n+1} = x_n + (alpha1*k0 + alpha2*k1)*h
 *                + gamma1*g0*dW
 *                + (lambda1*dW + lambda2*sqrt(h) + lambda3*W2)*g1
 *                + (mu1*dW     + mu2*sqrt(h)     + mu3*W2)*g2
 * \endcode
 *
 * @warning Stochastic integration is in beta.
 */
class svIntegratorWeakSIESME : public StochasticRKIntegratorBase {
public:
    /** Constructs the integrator for the given DynamicObject with the passed coefficient tableau. */
    svIntegratorWeakSIESME(DynamicObject* dynIn, const SIESMECoefficients& coefficients);

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;

protected:
    /** Coefficients to be used in the method */
    const SIESMECoefficients coefficients;
};

#endif /* svIntegratorWeakSIESME_h */
