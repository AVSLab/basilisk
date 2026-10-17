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

#include "svIntegratorRKF78.h"

svIntegratorRKF78::svIntegratorRKF78(DynamicObject* dyn)
    : svIntegratorAdaptiveRungeKutta(dyn, svIntegratorRKF78::getCoefficients(), 8.)
{
}

RKAdaptiveCoefficients<13> svIntegratorRKF78::getCoefficients()
{
    RKAdaptiveCoefficients<13> coefficients;

    // From Erwin Fehlberg (1968) Classical fifth-, sixth-, seventh-, and eighth-order
    // Runge-Kutta formulas with stepsize control. NASA Technical Report 287

    coefficients.aMatrix.at(1).at(0) = 2.0 / 27.0;

    coefficients.aMatrix.at(2).at(0) = 1.0 / 36.0;
    coefficients.aMatrix.at(2).at(1) = 1.0 / 12.0;

    coefficients.aMatrix.at(3).at(0) = 1.0 / 24.0;
    coefficients.aMatrix.at(3).at(2) = 1.0 / 8.0;

    coefficients.aMatrix.at(4).at(0) = 5.0 / 12.0;
    coefficients.aMatrix.at(4).at(2) = -25.0 / 16.0;
    coefficients.aMatrix.at(4).at(3) = 25.0 / 16.0;

    coefficients.aMatrix.at(5).at(0) = 1.0 / 20.0;
    coefficients.aMatrix.at(5).at(3) = 1.0 / 4.0;
    coefficients.aMatrix.at(5).at(4) = 1.0 / 5.0;

    coefficients.aMatrix.at(6).at(0) = -25.0 / 108.0;
    coefficients.aMatrix.at(6).at(3) = 125.0 / 108.0;
    coefficients.aMatrix.at(6).at(4) = -65.0 / 27.0;
    coefficients.aMatrix.at(6).at(5) = 125.0 / 54.0;

    coefficients.aMatrix.at(7).at(0) = 31.0 / 300.0;
    coefficients.aMatrix.at(7).at(4) = 61.0 / 225.0;
    coefficients.aMatrix.at(7).at(5) = -2.0 / 9.0;
    coefficients.aMatrix.at(7).at(6) = 13.0 / 900.0;

    coefficients.aMatrix.at(8).at(0) = 2.0;
    coefficients.aMatrix.at(8).at(3) = -53.0 / 6.0;
    coefficients.aMatrix.at(8).at(4) = 704.0 / 45.0;
    coefficients.aMatrix.at(8).at(5) = -107.0 / 9.0;
    coefficients.aMatrix.at(8).at(6) = 67.0 / 90.0;
    coefficients.aMatrix.at(8).at(7) = 3.0;

    coefficients.aMatrix.at(9).at(0) = -91.0 / 108.0;
    coefficients.aMatrix.at(9).at(3) = 23.0 / 108.0;
    coefficients.aMatrix.at(9).at(4) = -976.0 / 135.0;
    coefficients.aMatrix.at(9).at(5) = 311.0 / 54.0;
    coefficients.aMatrix.at(9).at(6) = -19.0 / 60.0;
    coefficients.aMatrix.at(9).at(7) = 17.0 / 6.0;
    coefficients.aMatrix.at(9).at(8) = -1.0 / 12.0;

    coefficients.aMatrix.at(10).at(0) = 2383.0 / 4100.0;
    coefficients.aMatrix.at(10).at(3) = -341.0 / 164.0;
    coefficients.aMatrix.at(10).at(4) = 4496.0 / 1025.0;
    coefficients.aMatrix.at(10).at(5) = -301.0 / 82.0;
    coefficients.aMatrix.at(10).at(6) = 2133.0 / 4100.0;
    coefficients.aMatrix.at(10).at(7) = 45.0 / 82.0;
    coefficients.aMatrix.at(10).at(8) = 45.0 / 164.0;
    coefficients.aMatrix.at(10).at(9) = 18.0 / 41.0;

    coefficients.aMatrix.at(11).at(0) = 3.0 / 205.0;
    coefficients.aMatrix.at(11).at(5) = -6.0 / 41.0;
    coefficients.aMatrix.at(11).at(6) = -3.0 / 205.0;
    coefficients.aMatrix.at(11).at(7) = -3.0 / 41.0;
    coefficients.aMatrix.at(11).at(8) = 3.0 / 41.0;
    coefficients.aMatrix.at(11).at(9) = 6.0 / 41.0;

    coefficients.aMatrix.at(12).at(0) = -1777.0 / 4100.0;
    coefficients.aMatrix.at(12).at(3) = -341.0 / 164.0;
    coefficients.aMatrix.at(12).at(4) = 4496.0 / 1025.0;
    coefficients.aMatrix.at(12).at(5) = -289.0 / 82.0;
    coefficients.aMatrix.at(12).at(6) = 2193.0 / 4100.0;
    coefficients.aMatrix.at(12).at(7) = 51.0 / 82.0;
    coefficients.aMatrix.at(12).at(8) = 33.0 / 164.0;
    coefficients.aMatrix.at(12).at(9) = 12.0 / 41.0;
    coefficients.aMatrix.at(12).at(11) = 1.0;

    coefficients.cArray.at(0) = 0.0;
    coefficients.cArray.at(1) = 2.0 / 27.0;
    coefficients.cArray.at(2) = 1.0 / 9.0;
    coefficients.cArray.at(3) = 1.0 / 6.0;
    coefficients.cArray.at(4) = 5.0 / 12.0;
    coefficients.cArray.at(5) = 1.0 / 2.0;
    coefficients.cArray.at(6) = 5.0 / 6.0;
    coefficients.cArray.at(7) = 1.0 / 6.0;
    coefficients.cArray.at(8) = 2.0 / 3.0;
    coefficients.cArray.at(9) = 1.0 / 3.0;
    coefficients.cArray.at(10) = 1.0;
    coefficients.cArray.at(11) = 0.0;
    coefficients.cArray.at(12) = 1.0;

    coefficients.bStarArray.at(0) = 41.0 / 840.0;
    coefficients.bStarArray.at(5) = 34.0 / 105.0;
    coefficients.bStarArray.at(6) = 9.0 / 35.0;
    coefficients.bStarArray.at(7) = 9.0 / 35.0;
    coefficients.bStarArray.at(8) = 9.0 / 280.0;
    coefficients.bStarArray.at(9) = 9.0 / 280.0;
    coefficients.bStarArray.at(10) = 41.0 / 840.0;

    coefficients.bArray.at(5) = 34.0 / 105.0;
    coefficients.bArray.at(6) = 9.0 / 35.0;
    coefficients.bArray.at(7) = 9.0 / 35.0;
    coefficients.bArray.at(8) = 9.0 / 280.0;
    coefficients.bArray.at(9) = 9.0 / 280.0;
    coefficients.bArray.at(11) = 41.0 / 840.0;
    coefficients.bArray.at(12) = 41.0 / 840.0;

    return coefficients;
}
