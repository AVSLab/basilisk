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
#include "svStochasticIntegratorW2Ito1.h"

#include <cmath>

svStochasticIntegratorW2Ito1::svStochasticIntegratorW2Ito1(DynamicObject* dyn)
    : svStochasticIntegratorW2Ito(dyn, svStochasticIntegratorW2Ito1::getCoefficients())
{
}

// Coefficients for the W2Ito1 tableau (Tang & Xiao, Table 2). 3-stage explicit method.
W2ItoCoefficients svStochasticIntegratorW2Ito1::getCoefficients()
{
    const double sqrt6 = std::sqrt(6.0);
    W2ItoCoefficients c;

    c.alpha = {1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0};
    c.beta0 = {-1.0, 1.0, 1.0};
    c.beta1 = {2.0, 0.0, -2.0};

    c.A0 = {{0.0, 0.0, 0.0},
            {1.0 / 2.0, 0.0, 0.0},
            {-1.0, 2.0, 0.0}};

    c.B0 = {{0.0, 0.0, 0.0},
            {(6.0 - sqrt6) / 10.0, 0.0, 0.0},
            {(3.0 + 2.0 * sqrt6) / 5.0, 0.0, 0.0}};

    c.A1 = {{0.0, 0.0, 0.0},
            {1.0 / 4.0, 0.0, 0.0},
            {1.0 / 4.0, 0.0, 0.0}};

    c.B1 = {{0.0, 0.0, 0.0},
            {1.0 / 2.0, 0.0, 0.0},
            {-1.0 / 2.0, 0.0, 0.0}};

    c.B2 = {{0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}};

    return c;
}
