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
#include <gtest/gtest-spi.h>
#include <limits>
#include <string>

namespace {
constexpr double notANumber = std::numeric_limits<double>::quiet_NaN();
constexpr double infinity = std::numeric_limits<double>::infinity();
}

/** @brief Verify inclusive scalar bounds, signs, and use of the first argument as reference. */
TEST(UnitTestComparators, scalarRelativeTolerance)
{
    EXPECT_NEAR_REL(8.0, 9.0, 0.125);
    EXPECT_NEAR_REL(-8.0, -9.0, 0.125);
    EXPECT_NEAR_REL(9.0, 8.0, 0.12);
    EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(8.0, 9.0, 0.12), "Relative comparison failed");
    EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(8.0, -8.0, 1.0), "Relative comparison failed");
}

/** @brief Require exact equality for zero references and zero tolerances. */
TEST(UnitTestComparators, scalarZeros)
{
    EXPECT_NEAR_REL(0.0, -0.0, 0.0);
    EXPECT_NEAR_REL(8.0, 8.0, 0.0);
    EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(0.0, 1.0, 10.0), "zero reference");
    EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(1.0, 0.0, 0.5), "Relative comparison failed");
    EXPECT_NEAR_REL(1.0, 0.0, 1.0);
    EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(8.0, 9.0, 0.0), "zero tolerance");
}

/** @brief Reject non-finite scalar values and invalid tolerances, even for equal operands. */
TEST(UnitTestComparators, scalarInvalidInputs)
{
    for (double invalid : { notANumber, infinity, -infinity }) {
        EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(invalid, 1.0, 0.1), "finite");
        EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(1.0, invalid, 0.1), "finite");
        EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(invalid, invalid, 0.1), "finite");
    }
    for (double invalid : { -1.0, notANumber, infinity, -infinity }) {
        EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(1.0, 1.0, invalid), "nonnegative");
    }
}

/** @brief Check componentwise absolute and norm-scaled relative vector tolerances. */
TEST(UnitTestComparators, vectorTolerance)
{
    const double reference[3] = { 3.0, 4.0, 0.0 };
    const double boundary[3] = { 3.5, 3.5, 0.5 };
    EXPECT_VECTOR3_NEAR(reference, boundary, 0.5);
    EXPECT_VECTOR3_NEAR_REL(reference, boundary, 0.1);
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(reference, boundary, 0.49), "component 0");
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(reference, boundary, 0.09), "component 0");

    for (int i = 0; i < 3; ++i) {
        double different[3] = { 3.0, 4.0, 0.0 };
        different[i] += 1.0;
        const std::string component = "component " + std::to_string(i);
        EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(reference, different, 0.5), component);
        EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(reference, different, 0.1), component);
    }
    const double smaller[3] = { 4.0, 0.0, 0.0 };
    const double larger[3] = { 5.0, 0.0, 0.0 };
    EXPECT_VECTOR3_NEAR_REL(larger, smaller, 0.2);
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(smaller, larger, 0.2), "component 0");
}

/** @brief Test zero-vector references, exact comparisons, and absolute tolerance around zero. */
TEST(UnitTestComparators, vectorZeros)
{
    const double zero[3] = {};
    const double nonzero[3] = { 0.0, 0.0, 1.0 };
    EXPECT_VECTOR3_NEAR(zero, zero, 0.0);
    EXPECT_VECTOR3_NEAR_REL(zero, zero, 0.0);
    EXPECT_VECTOR3_NEAR_REL(nonzero, nonzero, 0.0);
    EXPECT_VECTOR3_NEAR(zero, nonzero, 1.0);
    EXPECT_VECTOR3_NEAR_REL(nonzero, zero, 1.0);
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(zero, nonzero, 10.0), "zero reference");
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(nonzero, zero, 0.0), "component 2");
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(nonzero, zero, 0.0), "component 2");
}

/** @brief Reject invalid vector components in either operand and invalid tolerances. */
TEST(UnitTestComparators, vectorInvalidInputs)
{
    const double reference[3] = { 3.0, 4.0, 0.0 };
    for (double invalid : { notANumber, infinity, -infinity }) {
        for (int i = 0; i < 3; ++i) {
            double different[3] = { 3.0, 4.0, 0.0 };
            different[i] = invalid;
            EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(reference, different, 0.5), "Non-finite");
            EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(different, reference, 0.5), "Non-finite");
            EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(reference, different, 0.1), "Non-finite");
            EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(different, reference, 0.1), "Non-finite");
            EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(different, different, 0.1), "Non-finite");
        }
    }
    for (double invalid : { -1.0, notANumber, infinity, -infinity }) {
        EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(reference, reference, invalid), "nonnegative");
        EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(reference, reference, invalid), "nonnegative");
    }
}

/** @brief Avoid intermediate overflow and preserve relative comparisons of very small values. */
TEST(UnitTestComparators, extremeFiniteValues)
{
    const double largest = std::numeric_limits<double>::max();
    const double smallest = std::numeric_limits<double>::denorm_min();
    EXPECT_NEAR_REL(largest, -largest, 2.0);
    EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(largest, -largest, 1.0), "Relative comparison failed");
    EXPECT_NEAR_REL(smallest, 2.0 * smallest, 1.0);
    EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(smallest, 2.0 * smallest, 0.5), "Relative comparison failed");

    const double huge[3] = { largest, largest, largest };
    const double opposite[3] = { -largest, -largest, -largest };
    EXPECT_VECTOR3_NEAR_REL(huge, opposite, 1.2);
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(huge, opposite, 1.0), "component 0");
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(huge, opposite, largest), "component 0");
    const double tiny[3] = { smallest, 0.0, 0.0 };
    const double twiceTiny[3] = { 2.0 * smallest, 0.0, 0.0 };
    EXPECT_VECTOR3_NEAR_REL(tiny, twiceTiny, 1.0);
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(tiny, twiceTiny, 0.5), "component 0");
}

/** @brief Preserve relative tolerance decisions when multi-component references become subnormal. */
TEST(UnitTestComparators, subnormalVectorReferences)
{
    for (double magnitude : { 1.0, std::numeric_limits<double>::denorm_min() }) {
        const double reference[3] = { 2.0 * magnitude, 2.0 * magnitude, 0.0 };
        const double other[3] = { 3.0 * magnitude, 2.0 * magnitude, 0.0 };
        // The relative error is 1 / sqrt(8), approximately 0.353553.
        EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(reference, other, 0.34), "component 0");
        EXPECT_VECTOR3_NEAR_REL(reference, other, 0.36);

        const double diagonal[3] = { magnitude, magnitude, magnitude };
        const double displaced[3] = { 2.0 * magnitude, magnitude, magnitude };
        // The relative error is 1 / sqrt(3), approximately 0.577350.
        EXPECT_VECTOR3_NEAR_REL(diagonal, displaced, 0.75);
        EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(diagonal, displaced, 0.5), "component 0");
    }
}

/** @brief Keep a representable relative error finite when normalization would overflow midway. */
TEST(UnitTestComparators, subnormalReferenceWithLargeRelativeError)
{
    const double smallest = std::numeric_limits<double>::denorm_min();
    const double largest = std::numeric_limits<double>::max();
    const double reference[3] = { smallest, smallest, smallest };
    const double other[3] = { 1.5 * (smallest * largest), smallest, smallest };
    // The first component error is approximately sqrt(3) / 2 times the largest double.
    EXPECT_VECTOR3_NEAR_REL(reference, other, 0.9 * largest);
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(reference, other, 0.8 * largest), "component 0");
}

/** @brief Evaluate every macro argument once on both success and failure. */
TEST(UnitTestComparators, argumentsEvaluatedOnce)
{
    for (bool fail : { false, true }) {
        int firstCalls = 0;
        int secondCalls = 0;
        int toleranceCalls = 0;
        auto first = [&]() {
            ++firstCalls;
            return 8.0;
        };
        auto second = [&]() {
            ++secondCalls;
            return fail ? 10.0 : 8.0;
        };
        auto tolerance = [&]() {
            ++toleranceCalls;
            return 0.125;
        };
        if (fail) {
            EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(first(), second(), tolerance()), "Relative comparison failed");
        } else {
            EXPECT_NEAR_REL(first(), second(), tolerance());
        }
        EXPECT_EQ(firstCalls, 1);
        EXPECT_EQ(secondCalls, 1);
        EXPECT_EQ(toleranceCalls, 1);

        const double vector[3] = { 3.0, 4.0, 0.0 };
        const double other[3] = { 3.0, 4.0, 1.0 };
        auto firstVector = [&]() {
            ++firstCalls;
            return vector;
        };
        auto secondVector = [&]() {
            ++secondCalls;
            return fail ? other : vector;
        };
        if (fail) {
            EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(firstVector(), secondVector(), tolerance()), "component 2");
            EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(firstVector(), secondVector(), tolerance()), "component 2");
        } else {
            EXPECT_VECTOR3_NEAR(firstVector(), secondVector(), tolerance());
            EXPECT_VECTOR3_NEAR_REL(firstVector(), secondVector(), tolerance());
        }
        EXPECT_EQ(firstCalls, 3);
        EXPECT_EQ(secondCalls, 3);
        EXPECT_EQ(toleranceCalls, 3);
    }
}

/** @brief Preserve expression names, values, streamed context, and surrounding if/else behavior. */
TEST(UnitTestComparators, diagnosticsAndControlFlow)
{
    const double reference[3] = { 3.0, 4.0, 0.0 };
    const double other[3] = { 3.0, 4.0, 1.0 };
    EXPECT_NONFATAL_FAILURE(EXPECT_NEAR_REL(8.0, 9.0, 0.0) << "scalar context", "scalar context");
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(reference, other, 0.0) << "vector context", "vector context");
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(reference, other, 0.0) << "relative context", "relative context");
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR(reference, other, 0.0), "reference = [3, 4, 0]");
    EXPECT_NONFATAL_FAILURE(EXPECT_VECTOR3_NEAR_REL(reference, other, 0.0), "other = [3, 4, 1]");
    if (true)
        EXPECT_NEAR_REL(1.0, 1.0, 0.0);
    else
        ADD_FAILURE();
    if (true)
        EXPECT_VECTOR3_NEAR(reference, reference, 0.0);
    else
        ADD_FAILURE();
    if (true)
        EXPECT_VECTOR3_NEAR_REL(reference, reference, 0.0);
    else
        ADD_FAILURE();
}
