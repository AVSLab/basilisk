/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "svIntegratorRK4.h"

svIntegratorRK4::svIntegratorRK4(DynamicObject* dyn)
    : svIntegratorRungeKutta(dyn, svIntegratorRK4::getCoefficients())
{
}

RKCoefficients<4> svIntegratorRK4::getCoefficients()
{
    RKCoefficients<4> coefficients;
    coefficients.aMatrix.at(1).at(0) = 0.5;
    coefficients.aMatrix.at(2).at(1) = 0.5;
    coefficients.aMatrix.at(3).at(2) = 1.0;

    coefficients.bArray = {1. / 6., 1. / 3., 1. / 3., 1. / 6.};

    coefficients.cArray = {0., 1. / 2., 1. / 2., 1.};

    return coefficients;
}
