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

#include "architecture/utilities/linearAlgebra.h"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

namespace {
constexpr double notANumber = std::numeric_limits<double>::quiet_NaN();
constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr double largest = std::numeric_limits<double>::max();
constexpr double smallest = std::numeric_limits<double>::denorm_min();
constexpr double invalidValues[] = { notANumber, infinity, -infinity };
constexpr double invalidTolerances[] = { notANumber, infinity, -infinity, -1.0, -smallest };

// Give the fixed-size matrix APIs real two-dimensional arrays rather than casting vector storage.
template<size_t dimension, int (*compare)(double (*)[dimension], double (*)[dimension], double)>
int compareSquareMatrices(double* first, double* second, double accuracy)
{
    double firstMatrix[dimension][dimension];
    double secondMatrix[dimension][dimension];
    for (size_t row = 0; row < dimension; row++) {
        for (size_t column = 0; column < dimension; column++) {
            firstMatrix[row][column] = first[row * dimension + column];
            secondMatrix[row][column] = second[row * dimension + column];
        }
    }
    return compare(firstMatrix, secondMatrix, accuracy);
}

template<size_t dimension, int (*compare)(double (*)[dimension], double)>
int compareSquareMatrixToZero(double* values, double accuracy)
{
    double matrix[dimension][dimension];
    for (size_t row = 0; row < dimension; row++) {
        for (size_t column = 0; column < dimension; column++) {
            matrix[row][column] = values[row * dimension + column];
        }
    }
    return compare(matrix, accuracy);
}

struct EqualityCase
{
    const char* name;
    size_t size;
    int (*compare)(double*, double*, double);
};

const EqualityCase equalityCases[] = {
    { "vIsEqual",
      5,
      [](double* first, double* second, double accuracy) { return vIsEqual(first, 5, second, accuracy); } },
    { "v2IsEqual", 2, v2IsEqual },
    { "v3IsEqual", 3, v3IsEqual },
    { "v4IsEqual", 4, v4IsEqual },
    { "v6IsEqual", 6, v6IsEqual },
    { "mIsEqual2x3",
      6,
      [](double* first, double* second, double accuracy) { return mIsEqual(first, 2, 3, second, accuracy); } },
    { "mIsEqual3x2",
      6,
      [](double* first, double* second, double accuracy) { return mIsEqual(first, 3, 2, second, accuracy); } },
    { "m22IsEqual", 4, compareSquareMatrices<2, m22IsEqual> },
    { "m33IsEqual", 9, compareSquareMatrices<3, m33IsEqual> },
    { "m44IsEqual", 16, compareSquareMatrices<4, m44IsEqual> },
    { "m66IsEqual", 36, compareSquareMatrices<6, m66IsEqual> },
};

struct ZeroCase
{
    const char* name;
    size_t size;
    int (*compare)(double*, double);
};

const ZeroCase zeroCases[] = {
    { "vIsZero", 5, [](double* values, double accuracy) { return vIsZero(values, 5, accuracy); } },
    { "v2IsZero", 2, v2IsZero },
    { "v3IsZero", 3, v3IsZero },
    { "v4IsZero", 4, v4IsZero },
    { "mIsZero2x3", 6, [](double* values, double accuracy) { return mIsZero(values, 2, 3, accuracy); } },
    { "mIsZero3x2", 6, [](double* values, double accuracy) { return mIsZero(values, 3, 2, accuracy); } },
    { "m22IsZero", 4, compareSquareMatrixToZero<2, m22IsZero> },
    { "m33IsZero", 9, compareSquareMatrixToZero<3, m33IsZero> },
    { "m66IsZero", 36, compareSquareMatrixToZero<6, m66IsZero> },
};
}

/** @brief Preserve componentwise inclusive bounds, exact equality, and signed-zero behavior. */
TEST(AbsoluteComparisons, equalityFiniteValues)
{
    for (const auto& test : equalityCases) {
        SCOPED_TRACE(test.name);
        std::vector<double> first(test.size, 2.0);
        std::vector<double> second(test.size, 2.0);
        EXPECT_EQ(test.compare(first.data(), second.data(), 0.0), 1);
        EXPECT_EQ(test.compare(first.data(), first.data(), 0.0), 1);
        for (size_t index = 0; index < test.size; index++) {
            SCOPED_TRACE(index);
            second[index] = 2.5;
            EXPECT_EQ(test.compare(first.data(), second.data(), 0.5), 1);
            EXPECT_EQ(test.compare(second.data(), first.data(), 0.5), 1);
            EXPECT_EQ(test.compare(first.data(), second.data(), std::nextafter(0.5, 0.0)), 0);
            EXPECT_EQ(test.compare(first.data(), second.data(), 0.0), 0);
            second[index] = 2.0;
        }
        std::fill(second.begin(), second.end(), 2.5);
        EXPECT_EQ(test.compare(first.data(), second.data(), 0.5), 1);
        std::fill(first.begin(), first.end(), 0.0);
        std::fill(second.begin(), second.end(), -0.0);
        EXPECT_EQ(test.compare(first.data(), second.data(), 0.0), 1);
        EXPECT_EQ(test.compare(first.data(), second.data(), -0.0), 1);
    }
}

/** @brief Reject NaN and either infinity in every position of either operand, even when equal. */
TEST(AbsoluteComparisons, equalityRejectsNonFiniteValues)
{
    for (const auto& test : equalityCases) {
        SCOPED_TRACE(test.name);
        std::vector<double> first(test.size, 0.0);
        std::vector<double> second(test.size, 0.0);
        for (size_t index = 0; index < test.size; index++) {
            SCOPED_TRACE(index);
            for (double invalid : invalidValues) {
                SCOPED_TRACE(invalid);
                first[index] = invalid;
                EXPECT_EQ(test.compare(first.data(), second.data(), 1.0), 0);
                EXPECT_EQ(test.compare(second.data(), first.data(), 1.0), 0);
                EXPECT_EQ(test.compare(first.data(), first.data(), 1.0), 0);
                second[index] = invalid;
                EXPECT_EQ(test.compare(first.data(), second.data(), 1.0), 0);
                second[index] = -invalid;
                EXPECT_EQ(test.compare(first.data(), second.data(), 1.0), 0);
                first[index] = 0.0;
                second[index] = 0.0;
            }
        }
    }
}

/** @brief Reject negative and non-finite tolerances for equal and unequal finite operands. */
TEST(AbsoluteComparisons, equalityRejectsInvalidTolerances)
{
    for (const auto& test : equalityCases) {
        SCOPED_TRACE(test.name);
        std::vector<double> first(test.size, 0.0);
        std::vector<double> second(test.size, 1.0);
        for (double invalid : invalidTolerances) {
            SCOPED_TRACE(invalid);
            EXPECT_EQ(test.compare(first.data(), first.data(), invalid), 0);
            EXPECT_EQ(test.compare(first.data(), second.data(), invalid), 0);
        }
    }
}

/** @brief Preserve finite comparisons at the largest and smallest representable magnitudes. */
TEST(AbsoluteComparisons, equalityExtremeFiniteValues)
{
    for (const auto& test : equalityCases) {
        SCOPED_TRACE(test.name);
        std::vector<double> first(test.size, 0.0);
        std::vector<double> second(test.size, 0.0);
        first[0] = largest;
        second[0] = largest;
        EXPECT_EQ(test.compare(first.data(), second.data(), 0.0), 1);
        second[0] = -largest;
        EXPECT_EQ(test.compare(first.data(), second.data(), largest), 0);
        second[0] = 0.0;
        EXPECT_EQ(test.compare(first.data(), second.data(), largest), 1);
        EXPECT_EQ(test.compare(first.data(), second.data(), std::nextafter(largest, 0.0)), 0);
        first[0] = smallest;
        EXPECT_EQ(test.compare(first.data(), second.data(), 0.0), 0);
        EXPECT_EQ(test.compare(first.data(), second.data(), smallest), 1);
        first[0] = -smallest;
        EXPECT_EQ(test.compare(first.data(), second.data(), smallest), 1);
    }
}

/** @brief Preserve zero checks with signed zeros and positive and negative tolerance-boundary values. */
TEST(AbsoluteComparisons, zeroFiniteValues)
{
    const double boundaryValues[] = { 0.5, -0.5 };
    for (const auto& test : zeroCases) {
        SCOPED_TRACE(test.name);
        std::vector<double> values(test.size, 0.0);
        EXPECT_EQ(test.compare(values.data(), 0.0), 1);
        for (size_t index = 0; index < test.size; index++) {
            SCOPED_TRACE(index);
            for (double boundary : boundaryValues) {
                values[index] = boundary;
                EXPECT_EQ(test.compare(values.data(), 0.5), 1);
                EXPECT_EQ(test.compare(values.data(), std::nextafter(0.5, 0.0)), 0);
                EXPECT_EQ(test.compare(values.data(), 0.0), 0);
            }
            values[index] = 0.0;
        }
        std::fill(values.begin(), values.end(), 0.5);
        EXPECT_EQ(test.compare(values.data(), 0.5), 1);
        std::fill(values.begin(), values.end(), -0.0);
        EXPECT_EQ(test.compare(values.data(), 0.0), 1);
        EXPECT_EQ(test.compare(values.data(), -0.0), 1);
    }
}

/** @brief Reject NaN and either infinity at every position in a zero comparison. */
TEST(AbsoluteComparisons, zeroRejectsNonFiniteValues)
{
    for (const auto& test : zeroCases) {
        SCOPED_TRACE(test.name);
        std::vector<double> values(test.size, 0.0);
        for (size_t index = 0; index < test.size; index++) {
            SCOPED_TRACE(index);
            for (double invalid : invalidValues) {
                SCOPED_TRACE(invalid);
                values[index] = invalid;
                EXPECT_EQ(test.compare(values.data(), 1.0), 0);
            }
            values[index] = 0.0;
        }
    }
}

/** @brief Reject negative and non-finite tolerances when checking zero and nonzero inputs. */
TEST(AbsoluteComparisons, zeroRejectsInvalidTolerances)
{
    for (const auto& test : zeroCases) {
        SCOPED_TRACE(test.name);
        std::vector<double> zero(test.size, 0.0);
        std::vector<double> nonzero(test.size, 1.0);
        for (double invalid : invalidTolerances) {
            SCOPED_TRACE(invalid);
            EXPECT_EQ(test.compare(zero.data(), invalid), 0);
            EXPECT_EQ(test.compare(nonzero.data(), invalid), 0);
        }
    }
}

/** @brief Preserve zero comparisons at the largest and smallest finite magnitudes. */
TEST(AbsoluteComparisons, zeroExtremeFiniteValues)
{
    for (const auto& test : zeroCases) {
        SCOPED_TRACE(test.name);
        std::vector<double> values(test.size, 0.0);
        values[0] = largest;
        EXPECT_EQ(test.compare(values.data(), largest), 1);
        EXPECT_EQ(test.compare(values.data(), std::nextafter(largest, 0.0)), 0);
        values[0] = -largest;
        EXPECT_EQ(test.compare(values.data(), largest), 1);
        values[0] = smallest;
        EXPECT_EQ(test.compare(values.data(), 0.0), 0);
        EXPECT_EQ(test.compare(values.data(), smallest), 1);
        values[0] = -smallest;
        EXPECT_EQ(test.compare(values.data(), smallest), 1);
    }
}

/** @brief Empty vectors and matrices compare true only when the tolerance is valid. */
TEST(AbsoluteComparisons, emptyInputs)
{
    const double validTolerances[] = { 0.0, -0.0, 0.5, largest };
    for (double accuracy : validTolerances) {
        SCOPED_TRACE(accuracy);
        EXPECT_EQ(vIsEqual(nullptr, 0, nullptr, accuracy), 1);
        EXPECT_EQ(vIsZero(nullptr, 0, accuracy), 1);
        EXPECT_EQ(mIsEqual(nullptr, 0, 3, nullptr, accuracy), 1);
        EXPECT_EQ(mIsEqual(nullptr, 3, 0, nullptr, accuracy), 1);
        EXPECT_EQ(mIsZero(nullptr, 0, 3, accuracy), 1);
        EXPECT_EQ(mIsZero(nullptr, 3, 0, accuracy), 1);
    }
    for (double invalid : invalidTolerances) {
        SCOPED_TRACE(invalid);
        EXPECT_EQ(vIsEqual(nullptr, 0, nullptr, invalid), 0);
        EXPECT_EQ(vIsZero(nullptr, 0, invalid), 0);
        EXPECT_EQ(mIsEqual(nullptr, 0, 3, nullptr, invalid), 0);
        EXPECT_EQ(mIsEqual(nullptr, 3, 0, nullptr, invalid), 0);
        EXPECT_EQ(mIsZero(nullptr, 0, 3, invalid), 0);
        EXPECT_EQ(mIsZero(nullptr, 3, 0, invalid), 0);
    }
}

/** @brief Preserve inclusive componentwise bounds relative to the first vector's norm. */
TEST(RelativeComparisons, finiteValues)
{
    double first[3] = { 3.0, 4.0, 0.0 };
    double second[3] = { 3.0, 4.0, 0.0 };
    EXPECT_EQ(v3IsEqualRel(first, second, 0.0), 1);
    EXPECT_EQ(v3IsEqualRel(first, first, -0.0), 1);
    for (size_t index = 0; index < 3; index++) {
        SCOPED_TRACE(index);
        for (double difference : { -2.5, 2.5 }) {
            second[index] = first[index] + difference;
            EXPECT_EQ(v3IsEqualRel(first, second, 0.5), 1);
            EXPECT_EQ(v3IsEqualRel(first, second, std::nextafter(0.5, 0.0)), 0);
            EXPECT_EQ(v3IsEqualRel(first, second, 0.0), 0);
        }
        second[index] = first[index];
    }
    double displaced[3] = { 5.5, 6.5, 2.5 };
    EXPECT_EQ(v3IsEqualRel(first, displaced, 0.5), 1);

    double larger[3] = { 4.0, 0.0, 0.0 };
    double smaller[3] = { 2.0, 0.0, 0.0 };
    EXPECT_EQ(v3IsEqualRel(larger, smaller, 0.75), 1);
    EXPECT_EQ(v3IsEqualRel(smaller, larger, 0.75), 0);
}

/** @brief Reject non-finite components in either operand, including self-comparisons. */
TEST(RelativeComparisons, rejectsNonFiniteValues)
{
    for (size_t index = 0; index < 3; index++) {
        SCOPED_TRACE(index);
        for (double invalid : invalidValues) {
            SCOPED_TRACE(invalid);
            double finite[3] = { 1.0, 2.0, 3.0 };
            double nonFinite[3] = { 1.0, 2.0, 3.0 };
            nonFinite[index] = invalid;
            EXPECT_EQ(v3IsEqualRel(nonFinite, finite, 1.0), 0);
            EXPECT_EQ(v3IsEqualRel(finite, nonFinite, 1.0), 0);
            EXPECT_EQ(v3IsEqualRel(nonFinite, nonFinite, 1.0), 0);
        }
    }
}

/** @brief Reject negative and non-finite tolerances for equal, unequal, and zero vectors. */
TEST(RelativeComparisons, rejectsInvalidTolerances)
{
    double zero[3] = { 0.0, 0.0, 0.0 };
    double nonzero[3] = { 1.0, 2.0, 3.0 };
    for (double invalid : invalidTolerances) {
        SCOPED_TRACE(invalid);
        EXPECT_EQ(v3IsEqualRel(zero, zero, invalid), 0);
        EXPECT_EQ(v3IsEqualRel(nonzero, nonzero, invalid), 0);
        EXPECT_EQ(v3IsEqualRel(nonzero, zero, invalid), 0);
    }
}

/** @brief Require exact equality for a zero reference, including signed zeros. */
TEST(RelativeComparisons, zeroReference)
{
    double zero[3] = { 0.0, 0.0, 0.0 };
    double signedZero[3] = { -0.0, 0.0, -0.0 };
    for (double accuracy : { 0.0, -0.0, 0.5, largest }) {
        SCOPED_TRACE(accuracy);
        EXPECT_EQ(v3IsEqualRel(zero, signedZero, accuracy), 1);
        EXPECT_EQ(v3IsEqualRel(signedZero, zero, accuracy), 1);
        for (size_t index = 0; index < 3; index++) {
            double nonzero[3] = { 0.0, 0.0, 0.0 };
            nonzero[index] = smallest;
            EXPECT_EQ(v3IsEqualRel(zero, nonzero, accuracy), 0);
        }
    }
}

/** @brief A zero tolerance must not accept differences that underflow during normalization. */
TEST(RelativeComparisons, zeroTolerance)
{
    double first[3] = { largest, 0.0, -smallest };
    double second[3] = { largest, smallest, -smallest };
    EXPECT_EQ(v3IsEqualRel(first, first, 0.0), 1);
    EXPECT_EQ(v3IsEqualRel(first, second, 0.0), 0);
    EXPECT_EQ(v3IsEqualRel(first, second, -0.0), 0);
    EXPECT_EQ(v3IsEqualRel(first, second, smallest), 1);
}

/** @brief Preserve relative decisions when squared norms would overflow or underflow. */
TEST(RelativeComparisons, extremeFiniteScales)
{
    for (double magnitude : { 1.0, 1e200, 1e-200, smallest, largest / 2.0 }) {
        SCOPED_TRACE(magnitude);
        for (size_t index = 0; index < 3; index++) {
            SCOPED_TRACE(index);
            double first[3] = { 0.0, 0.0, 0.0 };
            double second[3] = { 0.0, 0.0, 0.0 };
            first[index] = magnitude;
            second[index] = 2.0 * magnitude;
            EXPECT_EQ(v3IsEqualRel(first, second, 1.0), 1);
            EXPECT_EQ(v3IsEqualRel(first, second, std::nextafter(1.0, 0.0)), 0);
            EXPECT_EQ(v3IsEqualRel(second, first, 0.5), 1);
            EXPECT_EQ(v3IsEqualRel(second, first, std::nextafter(0.5, 0.0)), 0);
        }
    }
    double first[3] = { 1e-200, 0.0, 0.0 };
    double second[3] = { 1.1e-200, 0.0, 0.0 };
    EXPECT_EQ(v3IsEqualRel(first, second, 0.2), 1);
}

/** @brief Handle finite vectors whose norm or component differences exceed the largest double. */
TEST(RelativeComparisons, overflowingNormAndDifference)
{
    double first[3] = { largest, largest, largest };
    double opposite[3] = { -largest, -largest, -largest };
    double zero[3] = { 0.0, 0.0, 0.0 };
    // The component errors are 2 / sqrt(3) and 1 / sqrt(3), respectively.
    EXPECT_EQ(v3IsEqualRel(first, opposite, 1.2), 1);
    EXPECT_EQ(v3IsEqualRel(first, opposite, 1.0), 0);
    EXPECT_EQ(v3IsEqualRel(first, zero, 0.6), 1);
    EXPECT_EQ(v3IsEqualRel(first, zero, 0.5), 0);

    double positiveAxis[3] = { largest, 0.0, 0.0 };
    double negativeAxis[3] = { -largest, 0.0, 0.0 };
    EXPECT_EQ(v3IsEqualRel(positiveAxis, negativeAxis, 2.0), 1);
    EXPECT_EQ(v3IsEqualRel(positiveAxis, negativeAxis, std::nextafter(2.0, 0.0)), 0);
}

/** @brief Avoid rounding the reference norm into the subnormal range before comparing. */
TEST(RelativeComparisons, subnormalReferences)
{
    for (double magnitude : { 1.0, smallest }) {
        SCOPED_TRACE(magnitude);
        double first[3] = { 2.0 * magnitude, 2.0 * magnitude, 0.0 };
        double second[3] = { 3.0 * magnitude, 2.0 * magnitude, 0.0 };
        // The first component error is 1 / sqrt(8).
        EXPECT_EQ(v3IsEqualRel(first, second, 0.36), 1);
        EXPECT_EQ(v3IsEqualRel(first, second, 0.34), 0);
    }
}

/** @brief Avoid intermediate division overflow when the final relative error is representable. */
TEST(RelativeComparisons, largeRelativeError)
{
    double first[3] = { smallest, smallest, smallest };
    double second[3] = { 1.5 * (smallest * largest), smallest, smallest };
    // The first component error is approximately sqrt(3) / 2 times the largest double.
    EXPECT_EQ(v3IsEqualRel(first, second, 0.9 * largest), 1);
    EXPECT_EQ(v3IsEqualRel(first, second, 0.8 * largest), 0);
    second[0] = largest;
    EXPECT_EQ(v3IsEqualRel(first, second, largest), 0);
}

/** @brief Preserve small differences between nearby finite values before scaling. */
TEST(RelativeComparisons, nearbyValues)
{
    double first[3] = { 7.0, 0.0, 0.0 };
    double second[3] = { std::nextafter(7.0, 8.0), 0.0, 0.0 };
    const double accuracy = (second[0] - first[0]) / first[0];
    EXPECT_EQ(v3IsEqualRel(first, second, accuracy), 1);
    EXPECT_EQ(v3IsEqualRel(first, second, std::nextafter(accuracy, 0.0)), 0);
}
