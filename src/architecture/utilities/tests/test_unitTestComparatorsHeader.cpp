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

// Include the header first in a second C++ translation unit to check self-containment and linkage.
#include "unitTestComparators.h"
#include <limits>

extern "C" int
compareAbsoluteFromC(double first, double second, double tolerance);
extern "C" int
compareRelativeFromC(double first, double second, double tolerance);

/** @brief Verify C and C++ consumers share valid, finite comparison behavior. */
TEST(UnitTestComparators, cAndCppHeaderCompatibility)
{
    EXPECT_TRUE(isEqual(8.0, 9.0, 1.0));
    EXPECT_FALSE(isEqual(8.0, 9.0, 0.5));
    EXPECT_NEAR_REL(8.0, 9.0, 0.125);
    EXPECT_TRUE(compareAbsoluteFromC(8.0, 9.0, 1.0));
    EXPECT_FALSE(compareAbsoluteFromC(8.0, 9.0, 0.5));
    EXPECT_TRUE(compareRelativeFromC(8.0, 9.0, 0.125));
    EXPECT_FALSE(compareRelativeFromC(8.0, 9.0, 0.1));
    EXPECT_TRUE(compareRelativeFromC(0.0, 0.0, 0.0));
    EXPECT_FALSE(compareRelativeFromC(0.0, 1.0, 10.0));
    for (double invalid : { std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity() }) {
        EXPECT_FALSE(isEqual(invalid, 1.0, 1.0));
        EXPECT_FALSE(compareAbsoluteFromC(invalid, 1.0, 1.0));
        EXPECT_FALSE(compareAbsoluteFromC(1.0, invalid, 1.0));
        EXPECT_FALSE(compareRelativeFromC(invalid, 1.0, 1.0));
        EXPECT_FALSE(compareRelativeFromC(1.0, invalid, 1.0));
        EXPECT_FALSE(compareAbsoluteFromC(1.0, 1.0, invalid));
        EXPECT_FALSE(compareRelativeFromC(1.0, 1.0, invalid));
    }
    EXPECT_FALSE(compareAbsoluteFromC(1.0, 1.0, -1.0));
    EXPECT_FALSE(compareRelativeFromC(1.0, 1.0, -1.0));
}
