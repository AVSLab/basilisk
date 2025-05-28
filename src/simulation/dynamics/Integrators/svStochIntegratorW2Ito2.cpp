/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "svStochIntegratorW2Ito2.h"

svStochIntegratorW2Ito2::svStochIntegratorW2Ito2(DynamicObject* dyn)
    : svIntegratorWeakStochasticRungeKutta(dyn, svStochIntegratorW2Ito2::getCoefficients())
{}

SRKCoefficients<4> svStochIntegratorW2Ito2::getCoefficients()
{
    SRKCoefficients<4> coefficients;

    coefficients.A0[1][0] = .5;
    coefficients.A0[2][1] = .5;
    coefficients.A0[3][2] = 1;

    coefficients.A1[1][0] = .5;
    coefficients.A1[2][0] = .5;
    coefficients.A1[3][0] = .5;

    coefficients.B0[2][1] = 1;
    coefficients.B0[3][1] = 1;

    coefficients.B1[2][1] = .5;
    coefficients.B1[3][1] = -.5;

    coefficients.B2[2][1] = 1;

    coefficients.alpha = {1./6., 1./3., 1./3., 1./6.};
    coefficients.beta0 = {0, -1, 1, 1};
    coefficients.beta1 = {0,  2, 0, -2};

    // c0[j] = sum_{p=1..s}( a0[j][p] )
    coefficients.c0 = {0, .5, .5, 1};

    // c1[j] = sum_{p=1..s}( a1[j][p] )
    coefficients.c1 = {0, .5, .5, .5};

    return coefficients;
}
