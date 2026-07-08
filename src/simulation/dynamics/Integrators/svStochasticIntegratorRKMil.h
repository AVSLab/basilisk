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

#ifndef svStochasticIntegratorRKMil_h
#define svStochasticIntegratorRKMil_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "../_GeneralModuleFiles/extendedStateVector.h"

#include <memory>

/** The Runge-Kutta Milstein stochastic integrator (strong order 1.0) for Ito SDEs
 * with diagonal or scalar noise.
 *
 * For an Ito SDE of the form:
 *
 * \f[
 *   dx = f(t,x)\,dt + \sum_i g_i(t,x)\,dW_i
 * \f]
 *
 * with time step \f$h\f$ and Wiener increments \f$\Delta W_i \sim N(0,h)\f$, the
 * integrator computes:
 *
 * \f[
 *   K = x_n + f(t_n,x_n)\,h, \quad L_i = g_i(t_n,x_n)
 * \f]
 * \f[
 *   \tilde{x} = K + \sum_i L_i\sqrt{h}, \quad
 *   (gg')_i = \frac{g_i(t_n,\tilde{x}) - L_i}{\sqrt{h}}
 * \f]
 * \f[
 *   x_{n+1} = K + \sum_i L_i\,\Delta W_i + \sum_i (gg')_i\,\frac{\Delta W_i^2 - h}{2}
 * \f]
 *
 * The final term is a derivative-free (Runge-Kutta) approximation of the Milstein
 * correction \f$\tfrac12 g_i\,\partial_x g_i\,(\Delta W_i^2 - h)\f$, which is why this
 * method needs no user-supplied Jacobian of the diffusion.
 *
 * This is an implementation of the ``RKMil`` method (Ito interpretation).
 * It is intended for diagonal or scalar noise.
 *
 * @warning Stochastic integration is in beta.
 */
class svStochasticIntegratorRKMil : public StochasticRKIntegratorBase {
public:
    using StochasticRKIntegratorBase::StochasticRKIntegratorBase;

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;
};

#endif /* svStochasticIntegratorRKMil_h */
