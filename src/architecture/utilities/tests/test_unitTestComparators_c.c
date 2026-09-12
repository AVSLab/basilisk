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

#include "unitTestComparators.h"

/**
 * @brief Exercise the shared absolute comparator from a C translation unit.
 * @param first First scalar.
 * @param second Second scalar, in the same units as first.
 * @param tolerance Absolute tolerance, in the same units as first.
 * @return The absolute comparison result.
 */
int
compareAbsoluteFromC(double first, double second, double tolerance)
{
    return isEqual(first, second, tolerance);
}

/**
 * @brief Exercise the shared relative comparator from a C translation unit.
 * @param first Reference scalar.
 * @param second Second scalar, in the same units as first.
 * @param tolerance Dimensionless relative tolerance.
 * @return The relative comparison result.
 */
int
compareRelativeFromC(double first, double second, double tolerance)
{
    return isEqualRel(first, second, tolerance);
}
