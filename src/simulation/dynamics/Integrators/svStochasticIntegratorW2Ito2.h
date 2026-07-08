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

#ifndef svStochasticIntegratorW2Ito2_h
#define svStochasticIntegratorW2Ito2_h

#include "svStochasticIntegratorW2Ito.h"

/**
 * @brief W2Ito2: Tang & Xiao efficient weak-order-2 method for Ito SDEs (4-stage tableau).
 *
 * Weak order 2, deterministic order 4, for Ito SDEs of any noise structure (scalar,
 * diagonal, non-diagonal / non-commutative). Shares the Tang & Xiao step of
 * svStochasticIntegratorW2Ito; differs only in its coefficient tableau (Table 3 of the
 * reference below).
 *
 *     Tang, X., Xiao, A. "Efficient weak second-order stochastic Runge-Kutta methods
 *     for Ito stochastic differential equations", BIT Numer. Math. 57, 241-260 (2017).
 *     https://doi.org/10.1007/s10543-016-0618-9
 *
 * @warning Stochastic integration is in beta.
 */
class svStochasticIntegratorW2Ito2 : public svStochasticIntegratorW2Ito {
public:
    svStochasticIntegratorW2Ito2(DynamicObject* dyn); //!< Constructor
private:
    static W2ItoCoefficients getCoefficients();
};

#endif /* svStochasticIntegratorW2Ito2_h */
