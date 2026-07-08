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

#ifndef svStochasticIntegratorSOSRI_h
#define svStochasticIntegratorSOSRI_h

#include "../_GeneralModuleFiles/svIntegratorStrongStochasticRungeKuttaSRI.h"

/** @brief Stability-optimized strong-order 1.5 stochastic integrator for diagonal/scalar Ito SDEs.
 *
 * @warning Stochastic integration is in beta.
 *
 * This is the ``SOSRI`` method (stability-optimized Roessler SRI family), a
 * good general-purpose method for diagonal/scalar Ito SDEs. The coefficients
 * are those of the ``SOSRI`` tableau.
 *
 *     Roessler A., "Runge-Kutta Methods for the Strong Approximation of Solutions of
 *     Stochastic Differential Equations", SIAM J. Numer. Anal., 48 (3), pp. 922-952.
 *     https://doi.org/10.1137/09076636X
 */
class svStochasticIntegratorSOSRI : public svIntegratorStrongStochasticRungeKuttaSRI<4> {
public:
    svStochasticIntegratorSOSRI(DynamicObject* dyn); //!< Constructor
private:
    static SRICoefficients<4> getCoefficients();
};

#endif /* svStochasticIntegratorSOSRI_h */
