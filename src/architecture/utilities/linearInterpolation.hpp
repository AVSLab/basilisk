/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef LINEARINTERPOLATION_H
#define LINEARINTERPOLATION_H

#include <cassert>

/*! This function uses linear interpolation to solve for the value of an unknown function of a single variables f(x)
 at the point x.
@return double
@param x1 Data point x1
@param x2 Data point x2
@param y1 Function value at point x1
@param y2 Function value at point x2
@param x Function x coordinate for interpolation
*/
double linearInterpolation(double x1, double x2, double y1, double y2, double x) {

    assert(x1 <  x && x < x2);

    return y1 * (x2 - x) / (x2 - x1) + y2 * (x - x1) / (x2 - x1);
}

#endif // LINEARINTERPOLATION_H
