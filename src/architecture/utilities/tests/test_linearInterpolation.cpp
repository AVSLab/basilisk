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

#include "architecture/utilities/linearInterpolation.hpp"
#include <gtest/gtest.h>
#include <random>

std::random_device rd;
std::default_random_engine generator(rd());
std::uniform_real_distribution<double> valueDistribution(-10, 10);
std::uniform_real_distribution<double> boundDistribution(0, 2);

TEST(LinearInterpolationTest, HandlesNormalInputs) {
    double x = valueDistribution(generator);
    double x1 = x - boundDistribution(generator);
    double x2 = x + boundDistribution(generator);

    double yUnused = valueDistribution(generator);
    double y1 = yUnused - boundDistribution(generator);
    double y2 = yUnused + boundDistribution(generator);

    // Linearly interpolate to solve for y
    double y = y1 * (x2 - x) / (x2 - x1) + y2 * (x - x1) / (x2 - x1);

    EXPECT_EQ(linearInterpolation(x1, x2, y1, y2, x), y);
}
