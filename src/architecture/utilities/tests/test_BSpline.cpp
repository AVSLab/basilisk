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

#include "architecture/utilities/BSpline.h"
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <new>

namespace {
template<typename... Args>
void expectZeroOptionalValues(const Args&... args)
{
    alignas(InputDataSet) unsigned char storage[sizeof(InputDataSet)];
    // Keep the nonzero prefill observable even in optimized builds.
    for (volatile unsigned char& byte : storage) {
        byte = 0x55;
    }
    auto* input = new (storage) InputDataSet(args...);
    EXPECT_EQ(input->AvgXDot, 0.0);
    EXPECT_EQ(input->XDot_0, Eigen::Vector3d::Zero());
    EXPECT_EQ(input->XDot_N, Eigen::Vector3d::Zero());
    EXPECT_EQ(input->XDDot_0, Eigen::Vector3d::Zero());
    EXPECT_EQ(input->XDDot_N, Eigen::Vector3d::Zero());
    input->~InputDataSet();
}
} // namespace

TEST(BSpline, DefaultConstructorZerosOptionalValues)
{
    expectZeroOptionalValues();
}

TEST(BSpline, CoordinateConstructorZerosOptionalValues)
{
    const Eigen::VectorXd coordinates = Eigen::VectorXd::Zero(2);
    expectZeroOptionalValues(coordinates, coordinates, coordinates);
}
