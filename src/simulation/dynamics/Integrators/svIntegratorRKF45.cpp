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

#include "svIntegratorRKF45.h"

svIntegratorRKF45::svIntegratorRKF45(DynamicObject* dyn)
    : svIntegratorAdaptiveRungeKutta(dyn, svIntegratorRKF45::getCoefficients(), 5.)
{
}

RKAdaptiveCoefficients<6> svIntegratorRKF45::getCoefficients()
{
    RKAdaptiveCoefficients<6> coefficients;

    coefficients.aMatrix.at(1).at(0) = 1.0 / 4.0;

    coefficients.aMatrix.at(2).at(0) = 3.0 / 32.0;
    coefficients.aMatrix.at(2).at(1) = 9.0 / 32.0;

    coefficients.aMatrix.at(3).at(0) = 1932.0 / 2197.0;
    coefficients.aMatrix.at(3).at(1) = -7200.0 / 2197.0;
    coefficients.aMatrix.at(3).at(2) = 7296.0 / 2197.0;

    coefficients.aMatrix.at(4).at(0) = 439.0 / 216.0;
    coefficients.aMatrix.at(4).at(1) = -8.0;
    coefficients.aMatrix.at(4).at(2) = 3680.0 / 513.0;
    coefficients.aMatrix.at(4).at(3) = -845.0 / 4104.0;

    coefficients.aMatrix.at(5).at(0) = -8.0 / 27.0;
    coefficients.aMatrix.at(5).at(1) = 2.0;
    coefficients.aMatrix.at(5).at(2) = -3544.0 / 2565.0;
    coefficients.aMatrix.at(5).at(3) = 1859.0 / 4104.0;
    coefficients.aMatrix.at(5).at(4) = -11.0 / 40.0;

    coefficients.cArray.at(1) = 1.0 / 4.0;
    coefficients.cArray.at(2) = 3.0 / 8.0;
    coefficients.cArray.at(3) = 12.0 / 13.0;
    coefficients.cArray.at(4) = 1.0;
    coefficients.cArray.at(5) = 1.0 / 2.0;

    coefficients.bStarArray.at(0) = 25.0 / 216.0;
    coefficients.bStarArray.at(2) = 1408.0 / 2565.0;
    coefficients.bStarArray.at(3) = 2197.0 / 4104.0;
    coefficients.bStarArray.at(4) = -1.0 / 5.0;

    coefficients.bArray.at(0) = 16.0 / 135.0;
    coefficients.bArray.at(2) = 6656.0 / 12825.0;
    coefficients.bArray.at(3) = 28561.0 / 56430.0;
    coefficients.bArray.at(4) = -9.0 / 50.0;
    coefficients.bArray.at(5) = 2.0 / 55.0;

    return coefficients;
}
