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

#ifndef svStochasticIntegratorEulerHeun_h
#define svStochasticIntegratorEulerHeun_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "../_GeneralModuleFiles/extendedStateVector.h"

#include <memory>

/** The Euler-Heun stochastic integrator (strong order 0.5), the Stratonovich
 * analogue of the Euler-Maruyama method.
 *
 * For a Stratonovich SDE of the form:
 *
 * \f[
 *   dx = f(t,x)\,dt + \sum_i g_i(t,x)\circ dW_i
 * \f]
 *
 * with time step \f$h\f$ and Wiener increments \f$\Delta W_i \sim N(0,h)\f$, the
 * integrator computes a predictor and corrector:
 *
 * \f[
 *   \bar{x} = x_n + f(t_n,x_n)\,h + \sum_i g_i(t_n,x_n)\,\Delta W_i
 * \f]
 * \f[
 *   x_{n+1} = x_n + \tfrac{h}{2}\big(f(t_n,x_n) + f(t_{n+1},\bar{x})\big)
 *                 + \sum_i \tfrac{1}{2}\big(g_i(t_n,x_n) + g_i(t_{n+1},\bar{x})\big)\Delta W_i
 * \f]
 *
 * This is an implementation of the Euler-Heun method.
 *
 * @warning Stochastic integration is in beta.
 */
class svStochasticIntegratorEulerHeun : public StochasticRKIntegratorBase {
public:
    using StochasticRKIntegratorBase::StochasticRKIntegratorBase;

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;
};

#endif /* svStochasticIntegratorEulerHeun_h */
