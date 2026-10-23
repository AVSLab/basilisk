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
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

namespace {
constexpr double accuracy = 1e-14;

void expectSingularValues(const double actual[3], const double expected[3])
{
    for (int i = 0; i < 3; i++) {
        SCOPED_TRACE(i);
        EXPECT_TRUE(std::isfinite(actual[i]));
        EXPECT_GE(actual[i], 0.0);
        if (expected[i] == 0.0) {
            EXPECT_NEAR(actual[i], 0.0, accuracy * expected[0]);
        } else {
            EXPECT_NEAR(actual[i] / expected[i], 1.0, accuracy);
        }
    }
    EXPECT_GE(actual[0], actual[1]);
    EXPECT_GE(actual[1], actual[2]);
}
}

/** @brief Return nonnegative singular values in descending order for signed, unsorted diagonals. */
TEST(SingularValues, signedDiagonal)
{
    double matrix[3][3] = { { -2.0, 0.0, 0.0 }, { 0.0, 3.0, 0.0 }, { 0.0, 0.0, -1.0 } };
    const double expected[3] = { 3.0, 2.0, 1.0 };
    double result[3] = {};
    m33SingularValues(matrix, result);
    expectSingularValues(result, expected);
    EXPECT_NEAR(m33ConditionNumber(matrix), 3.0, accuracy);
}

/** @brief Preserve equal singular values and unit condition number across finite scales. */
TEST(SingularValues, scaledIdentity)
{
    const double scales[] = { 0.1,
                              0.01,
                              1.0,
                              1e100,
                              1e-100,
                              1e300,
                              1e-300,
                              std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::min(),
                              std::numeric_limits<double>::denorm_min() };
    for (double scale : scales) {
        SCOPED_TRACE(scale);
        double matrix[3][3] = { { scale, 0.0, 0.0 }, { 0.0, -scale, 0.0 }, { 0.0, 0.0, scale } };
        const double expected[3] = { scale, scale, scale };
        double result[3] = {};
        m33SingularValues(matrix, result);
        expectSingularValues(result, expected);
        EXPECT_NEAR(m33ConditionNumber(matrix), 1.0, accuracy);
    }
}

/** @brief Retain small singular values that the characteristic-polynomial calculation loses. */
TEST(SingularValues, widelySeparatedValues)
{
    const double spectra[][3] = {
        { 1.0, 1e-8, 1e-12 }, { 1.0, 1e-100, 1e-200 }, { 1e100, 1.0, 1e-100 }, { 1e300, 1.0, 1e-300 }
    };
    for (const auto& expected : spectra) {
        SCOPED_TRACE(::testing::PrintToString(expected));
        double matrix[3][3] = { { expected[0], 0.0, 0.0 }, { 0.0, expected[1], 0.0 }, { 0.0, 0.0, expected[2] } };
        double result[3] = {};
        m33SingularValues(matrix, result);
        expectSingularValues(result, expected);
        double expectedCondition = expected[0] / expected[2];
        if (std::isfinite(expectedCondition)) {
            EXPECT_NEAR(m33ConditionNumber(matrix) / expectedCondition, 1.0, accuracy);
        } else {
            EXPECT_EQ(m33ConditionNumber(matrix), expectedCondition);
        }
    }
}

/** @brief Orthogonalize tiny columns even when their squared norms or rotation angles underflow. */
TEST(SingularValues, gradedColumns)
{
    const double scales[][3] = { { 1.0, 1e-100, 1e-200 }, { 1e300, 1.0, 1e-300 } };
    for (const auto& expected : scales) {
        SCOPED_TRACE(::testing::PrintToString(expected));
        for (int offset = 0; offset < 3; offset++) {
            double matrix[3][3] = {};
            matrix[0][offset] = expected[0];
            matrix[0][(offset + 1) % 3] = expected[1];
            matrix[1][(offset + 1) % 3] = expected[1];
            matrix[1][(offset + 2) % 3] = expected[2];
            matrix[2][(offset + 2) % 3] = expected[2];
            double result[3] = {};
            m33SingularValues(matrix, result);
            expectSingularValues(result, expected);
        }
    }
}

/** @brief Preserve the singular values of a dense matrix across large and small scales. */
TEST(SingularValues, scaledDenseMatrix)
{
    const double scales[] = { 1e300, 1e-300, std::numeric_limits<double>::max() / 6.0 };
    for (double scale : scales) {
        SCOPED_TRACE(scale);
        double matrix[3][3] = { { 3.0 * scale, -12.0 / 13.0 * scale, -144.0 / 65.0 * scale },
                                { 4.0 * scale, 9.0 / 13.0 * scale, 108.0 / 65.0 * scale },
                                { 0.0, -12.0 / 13.0 * scale, 5.0 / 13.0 * scale } };
        const double expected[3] = { 5.0 * scale, 3.0 * scale, scale };
        double result[3] = {};
        m33SingularValues(matrix, result);
        expectSingularValues(result, expected);
        EXPECT_NEAR(m33ConditionNumber(matrix), 5.0, accuracy * 5.0);
    }
}

/** @brief Recover known singular values of a nonsymmetric matrix without modifying the input. */
TEST(SingularValues, nonsymmetricMatrix)
{
    // Orthogonal rotations on the left and right of diag(5, 3, 1) preserve its singular values.
    const double original[3][3] = { { 3.0, -12.0 / 13.0, -144.0 / 65.0 },
                                    { 4.0, 9.0 / 13.0, 108.0 / 65.0 },
                                    { 0.0, -12.0 / 13.0, 5.0 / 13.0 } };
    const double expected[3] = { 5.0, 3.0, 1.0 };
    double matrix[3][3];
    for (int row = 0; row < 3; row++) {
        for (int column = 0; column < 3; column++) {
            matrix[row][column] = original[row][column];
        }
    }
    double result[3] = {};
    m33SingularValues(matrix, result);
    expectSingularValues(result, expected);
    EXPECT_NEAR(m33ConditionNumber(matrix), 5.0, accuracy * 5.0);
    for (int row = 0; row < 3; row++) {
        for (int column = 0; column < 3; column++) {
            EXPECT_DOUBLE_EQ(matrix[row][column], original[row][column]);
        }
    }
}

/** @brief Handle repeated singular values in a matrix with nonzero off-diagonal entries. */
TEST(SingularValues, repeatedValues)
{
    double matrix[3][3] = { { 2.0, 0.0, 0.0 }, { 0.0, 1.5, 0.5 }, { 0.0, 0.5, 1.5 } };
    const double expected[3] = { 2.0, 2.0, 1.0 };
    double result[3] = {};
    m33SingularValues(matrix, result);
    expectSingularValues(result, expected);
    EXPECT_NEAR(m33ConditionNumber(matrix), 2.0, accuracy);
}

/** @brief Preserve zero singular values and the undefined condition number of the zero matrix. */
TEST(SingularValues, zeroMatrix)
{
    double matrix[3][3] = {};
    const double expected[3] = {};
    double result[3] = { 1.0, 2.0, 3.0 };
    m33SingularValues(matrix, result);
    expectSingularValues(result, expected);
    EXPECT_TRUE(std::isnan(m33ConditionNumber(matrix)));
}

/** @brief Handle rank-one and rank-two matrices without taking square roots of negative roundoff. */
TEST(SingularValues, rankDeficientMatrices)
{
    double rankOne[3][3] = { { 1.0, 2.0, 2.0 }, { 2.0, 4.0, 4.0 }, { 2.0, 4.0, 4.0 } };
    const double rankOneExpected[3] = { 9.0, 0.0, 0.0 };
    double result[3] = {};
    m33SingularValues(rankOne, result);
    expectSingularValues(result, rankOneExpected);

    double rankTwo[3][3] = { { 3.0, 0.0, 0.0 }, { 0.0, -2.0, 0.0 }, { 0.0, 0.0, 0.0 } };
    const double rankTwoExpected[3] = { 3.0, 2.0, 0.0 };
    m33SingularValues(rankTwo, result);
    expectSingularValues(result, rankTwoExpected);
    EXPECT_EQ(m33ConditionNumber(rankTwo), std::numeric_limits<double>::infinity());
}

/** @brief Preserve finite singular values when their condition number exceeds the double range. */
TEST(SingularValues, overflowingConditionNumber)
{
    const double smallest = std::numeric_limits<double>::denorm_min();
    double matrix[3][3] = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, smallest } };
    const double expected[3] = { 1.0, 1.0, smallest };
    double result[3] = {};
    m33SingularValues(matrix, result);
    expectSingularValues(result, expected);
    EXPECT_EQ(m33ConditionNumber(matrix), std::numeric_limits<double>::infinity());
}

/** @brief Allow the result to overwrite any row of the input matrix without corrupting unread data. */
TEST(SingularValues, outputAliasesInput)
{
    const double original[3][3] = { { 3.0, -12.0 / 13.0, -144.0 / 65.0 },
                                    { 4.0, 9.0 / 13.0, 108.0 / 65.0 },
                                    { 0.0, -12.0 / 13.0, 5.0 / 13.0 } };
    const double expected[3] = { 5.0, 3.0, 1.0 };
    for (int outputRow = 0; outputRow < 3; outputRow++) {
        SCOPED_TRACE(outputRow);
        double matrix[3][3];
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                matrix[row][column] = original[row][column];
            }
        }
        m33SingularValues(matrix, matrix[outputRow]);
        expectSingularValues(matrix[outputRow], expected);
        for (int row = 0; row < 3; row++) {
            if (row != outputRow) {
                for (int column = 0; column < 3; column++) {
                    EXPECT_DOUBLE_EQ(matrix[row][column], original[row][column]);
                }
            }
        }
    }
}

/** @brief Return NaNs for a non-finite entry at any position rather than exposing uninitialized values. */
TEST(SingularValues, nonFiniteInput)
{
    const double invalidValues[] = { std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::infinity(),
                                     -std::numeric_limits<double>::infinity() };
    for (double invalid : invalidValues) {
        SCOPED_TRACE(invalid);
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                SCOPED_TRACE(row);
                SCOPED_TRACE(column);
                double matrix[3][3] = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
                matrix[row][column] = invalid;
                double result[3] = { 1.0, 2.0, 3.0 };
                m33SingularValues(matrix, result);
                for (double value : result) {
                    EXPECT_TRUE(std::isnan(value));
                }
                EXPECT_TRUE(std::isnan(m33ConditionNumber(matrix)));
            }
        }
    }
}
