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

#ifndef svStochasticIntegratorSIESME_h
#define svStochasticIntegratorSIESME_h

#include "../_GeneralModuleFiles/svIntegratorWeakSIESME.h"

/** @brief SIEA: Tocino & Vigo-Aguiar weak-order-2 stochastic integrator (Ito, diagonal noise).
 *
 * @warning Stochastic integration is in beta.
 *
 * Implementation of the ``SIEA`` method (stochastic improved Euler).
 */
class svStochasticIntegratorSIEA : public svIntegratorWeakSIESME {
public:
    svStochasticIntegratorSIEA(DynamicObject* dyn); //!< Constructor
private:
    static SIESMECoefficients getCoefficients();
};

/** @brief SMEA: Tocino & Vigo-Aguiar weak-order-2 stochastic integrator (Ito, diagonal noise).
 *
 * @warning Stochastic integration is in beta.
 *
 * Implementation of the ``SMEA`` method (stochastic modified Euler).
 */
class svStochasticIntegratorSMEA : public svIntegratorWeakSIESME {
public:
    svStochasticIntegratorSMEA(DynamicObject* dyn); //!< Constructor
private:
    static SIESMECoefficients getCoefficients();
};

/** @brief SIEB: Tocino & Vigo-Aguiar weak-order-2 stochastic integrator (Ito, diagonal noise).
 *
 * @warning Stochastic integration is in beta.
 *
 * Implementation of the ``SIEB`` method.
 */
class svStochasticIntegratorSIEB : public svIntegratorWeakSIESME {
public:
    svStochasticIntegratorSIEB(DynamicObject* dyn); //!< Constructor
private:
    static SIESMECoefficients getCoefficients();
};

/** @brief SMEB: Tocino & Vigo-Aguiar weak-order-2 stochastic integrator (Ito, diagonal noise).
 *
 * @warning Stochastic integration is in beta.
 *
 * Implementation of the ``SMEB`` method.
 */
class svStochasticIntegratorSMEB : public svIntegratorWeakSIESME {
public:
    svStochasticIntegratorSMEB(DynamicObject* dyn); //!< Constructor
private:
    static SIESMECoefficients getCoefficients();
};

#endif /* svStochasticIntegratorSIESME_h */
