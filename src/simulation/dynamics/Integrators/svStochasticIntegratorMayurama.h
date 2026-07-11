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

#ifndef svStochasticIntegratorMayurama_h
#define svStochasticIntegratorMayurama_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "../_GeneralModuleFiles/extendedStateVector.h"

/** The 1-weak 1-strong order Euler-Mayurama stochastic integrator.
 *
 * For an SDE of the form:
 *
 * \f[
 *   dx = f(t,x)\,dt + \sum_i g_i(t,x)\,dW_i
 * \f]
 *
 * The integrator, with timestep h, computes:
 *
 * \f[
 *   x_{n+1} = x_n + f(t,x)\,h + \sum_i g_i(t,x)\,\Delta W_i
 * \f]
 *
 * where the Wiener increments \f$\Delta W_i \sim N(0,h)\f$ are supplied by the shared
 * pluggable noise generator (see StochasticRKIntegratorBase). Euler-Mayurama uses only
 * the Wiener increment ``dW``; the second increment ``dZ`` drawn for the higher-order
 * methods is ignored.
 *
 * @warning Stochastic integration is in beta.
 */
class svStochasticIntegratorMayurama : public StochasticRKIntegratorBase {
public:

    /** Uses same constructor as StochasticRKIntegratorBase */
    using StochasticRKIntegratorBase::StochasticRKIntegratorBase;

    /** Performs the integration of the associated dynamic objects up to time currentTime+timeStep */
    virtual void integrate(double currentTime, double timeStep) override;
};

#endif // svStochasticIntegratorMayurama_h
