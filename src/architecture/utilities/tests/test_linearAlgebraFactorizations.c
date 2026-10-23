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

#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Keep checks active in Release builds. */
#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);                                            \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

/** @brief Compare finite arrays against an absolute tolerance. */
static int closeArray(const double* actual, const double* expected, size_t count, double tolerance)
{
    for (size_t i = 0; i < count; i++) {
        if (!isfinite(actual[i]) || fabs(actual[i] - expected[i]) > tolerance) {
            return 0;
        }
    }
    return 1;
}

/** @brief Check both inverse identities with independent multiplication loops. */
static int inverseIdentities(const double* matrix, const double* inverse, size_t dim, double tolerance)
{
    for (size_t row = 0; row < dim; row++) {
        for (size_t col = 0; col < dim; col++) {
            double left = 0.0;
            double right = 0.0;
            for (size_t inner = 0; inner < dim; inner++) {
                left += matrix[row * dim + inner] * inverse[inner * dim + col];
                right += inverse[row * dim + inner] * matrix[inner * dim + col];
            }
            double expected = row == col ? 1.0 : 0.0;
            CHECK(isfinite(left) && fabs(left - expected) <= tolerance);
            CHECK(isfinite(right) && fabs(right - expected) <= tolerance);
        }
    }
    return 0;
}

/** @brief Check the left inverse and orthogonal projection for a tall matrix. */
static int leastSquaresIdentities(const double* matrix,
                                  const double* inverse,
                                  size_t rows,
                                  size_t columns,
                                  double tolerance)
{
    for (size_t row = 0; row < columns; row++) {
        for (size_t col = 0; col < columns; col++) {
            double value = 0.0;
            for (size_t inner = 0; inner < rows; inner++) {
                value += inverse[row * rows + inner] * matrix[inner * columns + col];
            }
            CHECK(isfinite(value) && fabs(value - (row == col ? 1.0 : 0.0)) <= tolerance);
        }
    }
    for (size_t row = 0; row < rows; row++) {
        for (size_t col = 0; col < rows; col++) {
            double forward = 0.0;
            double reverse = 0.0;
            for (size_t inner = 0; inner < columns; inner++) {
                forward += matrix[row * columns + inner] * inverse[inner * rows + col];
                reverse += matrix[col * columns + inner] * inverse[inner * rows + row];
            }
            CHECK(isfinite(forward) && isfinite(reverse) && fabs(forward - reverse) <= tolerance);
        }
    }
    return 0;
}

/** @brief Check determinant signs, small cases, and preservation of the input. */
static int determinantKnown(void)
{
    double scalar = -3.0;
    double pair[] = { 0, 2, 4, 6 };
    double matrix[] = { 0, 0, 2, 3, 0, 1, 0, 4, 5 };
    double saved[9];
    memcpy(saved, matrix, sizeof(matrix));
    CHECK(mDeterminant(NULL, 0) == 0.0);
    CHECK(mDeterminant(&scalar, 1) == -3.0);
    CHECK(mDeterminant(pair, 2) == -8.0);
    CHECK(mDeterminant(matrix, 3) == 24.0);
    CHECK(memcmp(saved, matrix, sizeof(matrix)) == 0);
    for (size_t col = 0; col < 3; col++) {
        matrix[col] = saved[3 + col];
        matrix[3 + col] = saved[col];
    }
    CHECK(mDeterminant(matrix, 3) == -24.0);
    memcpy(matrix + 6, matrix, 3 * sizeof(double));
    CHECK(mDeterminant(matrix, 3) == 0.0);
    return 0;
}

/** @brief Retain the determinant's existing 65-by-65 boundary without recursive work. */
static int determinantBoundary(void)
{
    double matrix[65 * 65] = { 0 };
    for (size_t i = 0; i < 65; i++) {
        matrix[i * 65 + i] = 1.0;
        if (i + 1 < 65) {
            matrix[i * 65 + i + 1] = 0.5;
        }
    }
    CHECK(mDeterminant(matrix, 65) == 1.0);
    matrix[64 * 65 + 64] = 0.0;
    CHECK(mDeterminant(matrix, 65) == 0.0);
    return 0;
}

/** @brief Preserve finite determinants when an unscaled pivot product loses range. */
static int determinantRange(void)
{
    const double diagonals[][3] = {
        { 1e160, 1e160, 1e-160 },        { 1e-200, 1e-200, 1e200 },       { 0x1p700, 0x1p700, 0x1p-700 },
        { 0x1p-700, 0x1p-700, 0x1p700 }, { 0x1p-800, 0x1p-800, 0x1p526 }, { 0x1p800, 0x1p800, 0x1p-578 },
    };
    const double expected[] = { 1e160, 1e-200, 0x1p700, 0x1p-700, 0x1p-1074, 0x1p1022 };
    for (size_t sample = 0; sample < sizeof(expected) / sizeof(expected[0]); sample++) {
        for (size_t order = 0; order < 3; order++) {
            for (int negative = 0; negative < 2; negative++) {
                double matrix[9] = { 0 };
                for (size_t row = 0; row < 3; row++) {
                    matrix[row * 3 + row] = diagonals[sample][(row + order) % 3];
                }
                if (negative) {
                    matrix[0] = -matrix[0];
                }
                double target = negative ? -expected[sample] : expected[sample];
                double saved[9];
                memcpy(saved, matrix, sizeof(matrix));
                double determinant = mDeterminant(matrix, 3);
                CHECK(isfinite(determinant) && fabs(determinant / target - 1.0) <= 8.0 * DBL_EPSILON);
                CHECK(memcmp(matrix, saved, sizeof(matrix)) == 0);
                /* A row exchange must reverse the sign without losing the scale. */
                memcpy(matrix, saved + 3, 3 * sizeof(double));
                memcpy(matrix + 3, saved, 3 * sizeof(double));
                determinant = mDeterminant(matrix, 3);
                CHECK(isfinite(determinant) && fabs(determinant / -target - 1.0) <= 8.0 * DBL_EPSILON);
            }
        }
    }

    double large[65 * 65] = { 0 };
    for (size_t row = 0; row < 65; row++) {
        large[row * 65 + row] = row < 32 ? 0x1p1000 : (row < 64 ? 0x1p-1000 : 1.0);
    }
    CHECK(mDeterminant(large, 65) == 1.0);

    double overflow[] = { -1e200, 0, 0, 0, 1e200, 0, 0, 0, 1 };
    double underflow[] = { 1e-200, 0, 0, 0, 1e-200, 0, 0, 0, 1 };
    CHECK(mDeterminant(overflow, 3) == -(double)INFINITY);
    CHECK(mDeterminant(underflow, 3) == 0.0);
    overflow[8] = 0.0;
    CHECK(mDeterminant(overflow, 3) == 0.0);
    return 0;
}

/** @brief Preserve representable cofactor minors across widely differing diagonal scales. */
static int cofactorRange(void)
{
    const double diagonals[][4] = {
        { 1e160, 1e160, 1e-160, 1e-160 },
        { 1e-200, 1e-200, 1e200, 1e200 },
    };
    const double expected[][4] = {
        { 1e-160, 1e-160, 1e160, 1e160 },
        { 1e200, 1e200, 1e-200, 1e-200 },
    };
    for (size_t sample = 0; sample < sizeof(diagonals) / sizeof(diagonals[0]); sample++) {
        double matrix[16] = { 0 };
        double cofactors[16];
        for (size_t row = 0; row < 4; row++) {
            matrix[row * 4 + row] = diagonals[sample][row];
        }
        mCofactor(matrix, 4, cofactors);
        mCofactor(matrix, 4, matrix);
        CHECK(memcmp(matrix, cofactors, sizeof(matrix)) == 0);
        for (size_t row = 0; row < 4; row++) {
            for (size_t col = 0; col < 4; col++) {
                double value = cofactors[row * 4 + col];
                if (row == col) {
                    CHECK(isfinite(value) && fabs(value / expected[sample][row] - 1.0) <= 8.0 * DBL_EPSILON);
                } else {
                    CHECK(value == 0.0);
                }
            }
        }
    }
    return 0;
}

/** @brief Detect cancellation in cofactor expansion on an ordinary nearly dependent matrix. */
static int nearlyDependentSquare(void)
{
    const double delta = 0x1p-26;
    double matrix[] = { 1, 1, 1, 1, 1 + delta, 1, 1, 1, 1 + delta };
    double expected[] = { 1 + 2 / delta, -1 / delta, -1 / delta, -1 / delta, 1 / delta, 0, -1 / delta, 0, 1 / delta };
    double inverse[9];
    double saved[9];
    memcpy(saved, matrix, sizeof(matrix));
    CHECK(mDeterminant(matrix, 3) == delta * delta);
    CHECK(mInverse(matrix, 3, inverse) == 0);
    CHECK(closeArray(inverse, expected, 9, 0.0));
    CHECK(inverseIdentities(matrix, inverse, 3, 0.0) == 0);
    CHECK(memcmp(saved, matrix, sizeof(matrix)) == 0);
    return 0;
}

/** @brief Exercise row permutations at several stages of LU substitution. */
static int inversePivoting(void)
{
    double matrix[] = { 0, 0, 2, 3, 0, 4, 1, 2, 5, 2, 1, 0, 1, 2, 0, 1 };
    double inverse[16];
    double saved[16];
    memcpy(saved, matrix, sizeof(matrix));
    CHECK(mInverse(matrix, 4, inverse) == 0);
    CHECK(inverseIdentities(matrix, inverse, 4, 1e-14) == 0);
    CHECK(memcmp(saved, matrix, sizeof(matrix)) == 0);
    return 0;
}

/** @brief Validate the maximum supported square inverse, including a nontrivial dense input. */
static int inverseBoundary(void)
{
    double matrix[64 * 64];
    double inverse[64 * 64];
    for (size_t row = 0; row < 64; row++) {
        for (size_t col = 0; col < 64; col++) {
            matrix[row * 64 + col] = row == col ? 80.0 : (double)((row + 3 * col) % 7) - 3.0;
        }
    }
    CHECK(mInverse(matrix, 64, inverse) == 0);
    CHECK(inverseIdentities(matrix, inverse, 64, 2e-14) == 0);
    return 0;
}

/** @brief Preserve exact and shifted overlapping buffers, and their surrounding storage. */
static int inverseOverlap(void)
{
    const double matrix[] = { 0, 0, 2, 3, 0, 1, 0, 4, 5 };
    for (size_t offset = 1; offset <= 7; offset++) {
        double storage[20];
        double saved[20];
        for (size_t i = 0; i < 20; i++) {
            storage[i] = -91.0;
        }
        memcpy(storage + 4, matrix, sizeof(matrix));
        memcpy(saved, storage, sizeof(storage));
        CHECK(mInverse(storage + 4, 3, storage + offset) == 0);
        CHECK(inverseIdentities(matrix, storage + offset, 3, 1e-14) == 0);
        for (size_t i = 0; i < 20; i++) {
            CHECK((i >= offset && i < offset + 9) || storage[i] == saved[i]);
        }
    }
    return 0;
}

/** @brief Retain the absolute determinant cutoff, status, and zero output on failure. */
static int inverseFailures(void)
{
    const double values[] = { 0, nextafter(DB0_EPS, 0), DB0_EPS, -DB0_EPS };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        double matrix[] = { values[i], 0, 0, 1 };
        double output[4] = { 9, 9, 9, 9 };
        CHECK(mInverse(matrix, 2, output) == 1);
        CHECK(mIsZero(output, 2, 2, 0.0));
    }
    double matrix[] = { nextafter(DB0_EPS, 1), 0, 0, 1 };
    double inverse[16];
    CHECK(mInverse(matrix, 2, inverse) == 0);
    CHECK(inverseIdentities(matrix, inverse, 2, 1e-15) == 0);
    double smallIdentity[16] = { 0 };
    for (size_t i = 0; i < 4; i++) {
        smallIdentity[5 * i] = 1e-8;
    }
    CHECK(mInverse(smallIdentity, 4, inverse) == 1);
    CHECK(mIsZero(inverse, 4, 4, 0.0));
    double singular[] = { 1, 2, 3, 1, 2, 3, 0, 1, 1 };
    feclearexcept(FE_ALL_EXCEPT);
    CHECK(mInverse(singular, 3, singular) == 1);
    CHECK(mIsZero(singular, 3, 3, 0.0));
    CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
    return 0;
}

/** @brief Keep cofactors meaningful for rank-deficient matrices, without invoking an inverse. */
static int cofactorSingular(void)
{
    double matrix[] = { 1, 2, 3, 2, 4, 6, 0, 1, 1 };
    const double expected[] = { -2, -2, 2, 1, 1, -1, 0, 0, 0 };
    double output[9];
    mCofactor(matrix, 3, output);
    CHECK(closeArray(output, expected, 9, 0.0));
    double lowRank[] = { 1, 2, 3, 2, 4, 6, 3, 6, 9 };
    mCofactor(lowRank, 3, output);
    CHECK(mIsZero(output, 3, 3, 0.0));
    /* A 12-by-12 matrix used to require impractical factorial work. */
    double diagonal[12 * 12] = { 0 };
    double cofactors[12 * 12];
    for (size_t i = 0; i < 11; i++) {
        diagonal[i * 12 + i] = 1.0;
    }
    mCofactor(diagonal, 12, cofactors);
    for (size_t i = 0; i < 12 * 12; i++) {
        CHECK(cofactors[i] == (i == 143 ? 1.0 : 0.0));
    }
    return 0;
}

/** @brief Verify shifted cofactor output against independent exact values. */
static int cofactorOverlap(void)
{
    const double matrix[] = { 1, 2, 3, 2, 4, 6, 0, 1, 1 };
    const double expected[] = { -2, -2, 2, 1, 1, -1, 0, 0, 0 };
    for (size_t offset = 0; offset <= 6; offset++) {
        double storage[18];
        memcpy(storage + 3, matrix, sizeof(matrix));
        mCofactor(storage + 3, 3, storage + offset);
        CHECK(closeArray(storage + offset, expected, 9, 0.0));
    }
    return 0;
}

/** @brief Check an analytic least-squares inverse and the orthogonal projection it defines. */
static int leastSquaresKnown(void)
{
    double matrix[] = { 1, 0, 1, 1, 1, 2, 1, 3 };
    const double expected[] = { 0.7, 0.4, 0.1, -0.2, -0.3, -0.1, 0.1, 0.3 };
    double inverse[8];
    double saved[8];
    memcpy(saved, matrix, sizeof(matrix));
    mLeastSquaresInverse(matrix, 4, 2, inverse);
    CHECK(closeArray(inverse, expected, 8, 1e-15));
    CHECK(leastSquaresIdentities(matrix, inverse, 4, 2, 2e-15) == 0);
    CHECK(memcmp(saved, matrix, sizeof(matrix)) == 0);
    return 0;
}

/** @brief Check the transpose relation and minimum-norm property against exact values. */
static int minimumNormKnown(void)
{
    double matrix[] = { 1, 1, 1, 1, 0, 1, 2, 3 };
    const double expected[] = { 0.7, -0.3, 0.4, -0.1, 0.1, 0.1, -0.2, 0.3 };
    double inverse[8];
    double saved[8];
    memcpy(saved, matrix, sizeof(matrix));
    mMinimumNormInverse(matrix, 2, 4, inverse);
    CHECK(closeArray(inverse, expected, 8, 1e-15));
    CHECK(memcmp(saved, matrix, sizeof(matrix)) == 0);
    /* The solution columns must be orthogonal to both independent null-space directions. */
    for (size_t col = 0; col < 2; col++) {
        CHECK(fabs(inverse[col] - 2 * inverse[2 + col] + inverse[4 + col]) < 1e-15);
        CHECK(fabs(inverse[2 + col] - 2 * inverse[4 + col] + inverse[6 + col]) < 1e-15);
    }
    return 0;
}

/** @brief Recover a full-rank inverse whose normal equations lose the small column difference. */
static int rectangularNearDependent(void)
{
    const double delta = 0x1p-26;
    double matrix[] = { 1, 1, 1, 1 + delta, 1, 1 - delta };
    double expected[] = { 1.0 / 3, 1.0 / 3 - 0.5 / delta, 1.0 / 3 + 0.5 / delta, 0, 0.5 / delta, -0.5 / delta };
    double inverse[6];
    mLeastSquaresInverse(matrix, 3, 2, inverse);
    CHECK(closeArray(inverse, expected, 6, 2e-8 / delta));
    CHECK(leastSquaresIdentities(matrix, inverse, 3, 2, 5e-8) == 0);
    double wide[] = { 1, 1, 1, 1, 1 + delta, 1 - delta };
    double minimumNorm[6];
    mMinimumNormInverse(wide, 2, 3, minimumNorm);
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 2; col++) {
            CHECK(fabs(minimumNorm[2 * row + col] - expected[3 * col + row]) < 2e-8 / delta);
        }
    }
    return 0;
}

/** @brief Force column permutations and check that both output orientations undo them. */
static int rectangularPivoting(void)
{
    double matrix[] = { 1, 0, 3, -1, 2, 0, 1, 0, -3, -1, -2, 0 };
    const double normsSquared[] = { 4, 8, 18 };
    double inverse[12];
    mLeastSquaresInverse(matrix, 4, 3, inverse);
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 4; col++) {
            CHECK(fabs(inverse[4 * row + col] - matrix[3 * col + row] / normsSquared[row]) < 1e-15);
        }
    }
    CHECK(leastSquaresIdentities(matrix, inverse, 4, 3, 2e-15) == 0);
    double wide[12];
    double minimumNorm[12];
    mTranspose(matrix, 4, 3, wide);
    mMinimumNormInverse(wide, 3, 4, minimumNorm);
    for (size_t row = 0; row < 4; row++) {
        for (size_t col = 0; col < 3; col++) {
            CHECK(fabs(minimumNorm[3 * row + col] - matrix[3 * row + col] / normsSquared[col]) < 1e-15);
        }
    }
    return 0;
}

/** @brief Ensure square inputs remain supported by both rectangular inverse APIs. */
static int rectangularSquare(void)
{
    double matrix[] = { 0, 0, 2, 3, 0, 1, 0, 4, 5 };
    double inverse[9];
    mLeastSquaresInverse(matrix, 3, 3, inverse);
    CHECK(inverseIdentities(matrix, inverse, 3, 5e-15) == 0);
    mMinimumNormInverse(matrix, 3, 3, inverse);
    CHECK(inverseIdentities(matrix, inverse, 3, 5e-15) == 0);
    return 0;
}

/** @brief Exercise in-place and partially overlapping rectangular input and output. */
static int rectangularOverlap(void)
{
    const double matrix[] = { 1, 0, 1, 1, 1, 2, 1, 3 };
    const double wide[] = { 1, 1, 1, 1, 0, 1, 2, 3 };
    const double expected[] = { 0.7, 0.4, 0.1, -0.2, -0.3, -0.1, 0.1, 0.3 };
    const double expectedWide[] = { 0.7, -0.3, 0.4, -0.1, 0.1, 0.1, -0.2, 0.3 };
    for (size_t mode = 0; mode < 2; mode++) {
        for (size_t offset = 1; offset <= 7; offset++) {
            double storage[20];
            double saved[20];
            for (size_t i = 0; i < 20; i++) {
                storage[i] = -91.0;
            }
            memcpy(storage + 4, mode == 0 ? matrix : wide, sizeof(matrix));
            memcpy(saved, storage, sizeof(storage));
            if (mode == 0) {
                mLeastSquaresInverse(storage + 4, 4, 2, storage + offset);
            } else {
                mMinimumNormInverse(storage + 4, 2, 4, storage + offset);
            }
            CHECK(closeArray(storage + offset, mode == 0 ? expected : expectedWide, 8, 1e-15));
            for (size_t i = 0; i < 20; i++) {
                CHECK((i >= offset && i < offset + 8) || storage[i] == saved[i]);
            }
        }
    }
    return 0;
}

/** @brief Reject zero and numerically dependent columns, including differently scaled duplicates. */
static int rectangularRankFailure(void)
{
    double cases[][6] = {
        { 0, 0, 0, 0, 0, 0 }, { 1, 1, 2, 2, 3, 3 }, { 1, 10, 2, 20, 3, 30 }, { 1, 1, 1, 1 + DBL_EPSILON, 1, 1 }
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        double inverse[6];
        double wide[6];
        mTranspose(cases[i], 3, 2, wide);
        feclearexcept(FE_ALL_EXCEPT);
        mLeastSquaresInverse(cases[i], 3, 2, inverse);
        CHECK(mIsZero(inverse, 2, 3, 0.0));
        mMinimumNormInverse(wide, 2, 3, wide);
        CHECK(mIsZero(wide, 3, 2, 0.0));
        CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
    }
    return 0;
}

/** @brief Retain the Gram-determinant threshold and document invalid-shape no-access behavior. */
static int rectangularPolicies(void)
{
    double output = 9.0;
    mLeastSquaresInverse(NULL, 2, 3, &output);
    mMinimumNormInverse(NULL, 3, 2, &output);
    CHECK(output == 9.0);
    const double values[] = { 0.5 * sqrt(DB0_EPS), 2.0 * sqrt(DB0_EPS) };
    for (size_t i = 0; i < 2; i++) {
        double matrix[] = { values[i], 0 };
        double inverse[2];
        mLeastSquaresInverse(matrix, 2, 1, inverse);
        CHECK(i == 0 ? mIsZero(inverse, 1, 2, 0.0) : fabs(inverse[0] * values[i] - 1) < 1e-15);
        mMinimumNormInverse(matrix, 1, 2, inverse);
        CHECK(i == 0 ? mIsZero(inverse, 2, 1, 0.0) : fabs(inverse[0] * values[i] - 1) < 1e-15);
    }
    double smallIdentity[] = { 1e-8, 0, 0, 1e-8 };
    double inverse[4];
    mLeastSquaresInverse(smallIdentity, 2, 2, inverse);
    CHECK(mIsZero(inverse, 2, 2, 0.0));
    mMinimumNormInverse(smallIdentity, 2, 2, inverse);
    CHECK(mIsZero(inverse, 2, 2, 0.0));
    return 0;
}

/** @brief Verify long, narrow inputs and square inputs at the full workspace capacity. */
static int rectangularBoundary(void)
{
    double matrix[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    double inverse[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    for (size_t i = 0; i < LINEAR_ALGEBRA_MAX_ARRAY_SIZE; i++) {
        matrix[i] = i % 2 == 0 ? 0.5 : -0.5;
    }
    mLeastSquaresInverse(matrix, LINEAR_ALGEBRA_MAX_ARRAY_SIZE, 1, inverse);
    for (size_t i = 0; i < LINEAR_ALGEBRA_MAX_ARRAY_SIZE; i++) {
        CHECK(fabs(inverse[i] - matrix[i] / 1024.0) < 1e-16);
    }
    mMinimumNormInverse(matrix, 1, LINEAR_ALGEBRA_MAX_ARRAY_SIZE, inverse);
    for (size_t i = 0; i < LINEAR_ALGEBRA_MAX_ARRAY_SIZE; i++) {
        CHECK(fabs(inverse[i] - matrix[i] / 1024.0) < 1e-16);
    }
    mSetZero(matrix, 64, 64);
    for (size_t row = 0; row < 64; row++) {
        matrix[row * 64 + (row + 1) % 64] = 2.0;
    }
    mLeastSquaresInverse(matrix, 64, 64, inverse);
    CHECK(inverseIdentities(matrix, inverse, 64, 1e-14) == 0);
    mMinimumNormInverse(matrix, 64, 64, inverse);
    CHECK(inverseIdentities(matrix, inverse, 64, 1e-14) == 0);
    return 0;
}

/** @brief Prevent successful inverses containing NaNs or infinities. */
static int nonfiniteInverse(void)
{
    const double values[] = { (double)NAN, (double)INFINITY, -(double)INFINITY };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        for (size_t entry = 0; entry < 4; entry++) {
            double matrix[] = { 1, 0, 0, 1 };
            double inverse[4];
            matrix[entry] = values[i];
            CHECK(mInverse(matrix, 2, inverse) == 1);
            CHECK(mIsZero(inverse, 2, 2, 0.0));
            mLeastSquaresInverse(matrix, 2, 2, inverse);
            CHECK(mIsZero(inverse, 2, 2, 0.0));
            mMinimumNormInverse(matrix, 2, 2, inverse);
            CHECK(mIsZero(inverse, 2, 2, 0.0));
        }
    }
    return 0;
}

/** @brief Run a named C regression, or all regressions if no name is supplied. */
int main(int argc, char** argv)
{
    const struct
    {
        const char* name;
        int (*run)(void);
    } tests[] = {
        { "determinantKnown", determinantKnown },
        { "determinantBoundary", determinantBoundary },
        { "determinantRange", determinantRange },
        { "cofactorRange", cofactorRange },
        { "nearlyDependentSquare", nearlyDependentSquare },
        { "inversePivoting", inversePivoting },
        { "inverseBoundary", inverseBoundary },
        { "inverseOverlap", inverseOverlap },
        { "inverseFailures", inverseFailures },
        { "cofactorSingular", cofactorSingular },
        { "cofactorOverlap", cofactorOverlap },
        { "leastSquaresKnown", leastSquaresKnown },
        { "minimumNormKnown", minimumNormKnown },
        { "rectangularNearDependent", rectangularNearDependent },
        { "rectangularPivoting", rectangularPivoting },
        { "rectangularSquare", rectangularSquare },
        { "rectangularOverlap", rectangularOverlap },
        { "rectangularRankFailure", rectangularRankFailure },
        { "rectangularPolicies", rectangularPolicies },
        { "rectangularBoundary", rectangularBoundary },
        { "nonfiniteInverse", nonfiniteInverse },
    };
    int failures = 0;
    int selected = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        if (argc == 1 || (argc == 2 && strcmp(argv[1], tests[i].name) == 0)) {
            int status = tests[i].run();
            printf("%s: %s\n", tests[i].name, status == 0 ? "PASS" : "FAIL");
            failures += status;
            selected++;
        }
    }
    return selected != 0 && failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
