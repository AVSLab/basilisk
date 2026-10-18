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

#include "architecture/utilities/orbitalMotion.h"

#define ZONAL_TEST_POSITION (double[]){ 7000.0, 20.0, 100.0 } /* [km] */

/**
 * @brief Compile and evaluate compound-literal source calls as standard C.
 * @param acceleration Four output vectors [km/s^2], alternating Earth and Mars.
 * @note Exercise literal and macro-expanded positions in both supported call forms.
 */
void zonalCompoundLiteralsFromC(double acceleration[4][3])
{
    jPerturb((double[]){ 7000.0, 20.0, 100.0 }, 2, acceleration[0]);                 /* [km] */
    jPerturb((double[]){ 7000.0, 20.0, 100.0 }, 2, acceleration[1], CELESTIAL_MARS); /* [km] */
    jPerturb(ZONAL_TEST_POSITION, 2, acceleration[2]);
    jPerturb(ZONAL_TEST_POSITION, 2, acceleration[3], CELESTIAL_MARS);
}
