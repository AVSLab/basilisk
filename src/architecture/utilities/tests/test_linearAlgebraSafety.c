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
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_MSC_VER)
#pragma fenv_access(on)
#elif defined(__clang__)
#pragma STDC FENV_ACCESS ON
#endif

/* Keep checks active in Release builds, where assert would be disabled. */
#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);                                            \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

/** @brief Check no-access behavior for empty output arrays, including null pointers. */
static int emptyOperations(void)
{
    vCopy(NULL, 0, NULL);
    vSort(NULL, NULL, 0);
    vOuterProduct(NULL, 0, NULL, SIZE_MAX, NULL);
    vOuterProduct(NULL, SIZE_MAX, NULL, 0, NULL);
    vtMultM(NULL, NULL, SIZE_MAX, 0, NULL);
    vtMultMt(NULL, NULL, 0, SIZE_MAX, NULL);
    mMultV(NULL, 0, SIZE_MAX, NULL, NULL);
    mtMultV(NULL, SIZE_MAX, 0, NULL, NULL);
    mCopy(NULL, SIZE_MAX, 0, NULL);
    mCopy(NULL, 0, SIZE_MAX, NULL);
    mDiag(NULL, 0, NULL);
    mTranspose(NULL, SIZE_MAX, 0, NULL);
    mTranspose(NULL, 0, SIZE_MAX, NULL);
    mMultM(NULL, 0, 3, NULL, 3, 4, NULL);
    mtMultM(NULL, 3, 0, NULL, 3, 4, NULL);
    mMultMt(NULL, 4, 3, NULL, 0, 3, NULL);
    mtMultMt(NULL, 3, 4, NULL, 0, 3, NULL);
    mGetSubMatrix(NULL, 0, 0, 0, 0, 0, SIZE_MAX, NULL);
    mGetSubMatrix(NULL, 0, 0, 0, 0, SIZE_MAX, 0, NULL);
    mSetSubMatrix(NULL, 0, SIZE_MAX, NULL, 0, 0, 0, 0);
    mSetSubMatrix(NULL, SIZE_MAX, 0, NULL, 0, 0, 0, 0);
    return 0;
}

/** @brief Check the explicit empty-reduction policy and preserve nonempty maxima. */
static int emptyReductions(void)
{
    CHECK(isnan(vMax(NULL, 0)));
    CHECK(isnan(vMaxAbs(NULL, 0)));
    double values[] = { -4.0, -2.0, -7.0 };
    CHECK(vMax(values, 3) == -2.0);
    CHECK(vMaxAbs(values, 3) == 7.0);
    CHECK(vMax(values, 1) == -4.0);
    CHECK(vMaxAbs(values, 1) == 4.0);
    return 0;
}

/** @brief Clamp negatives and propagate quiet NaNs without invalid floating-point operations. */
static int safeSquareRoot(void)
{
    const double values[] = { -1.0, -(double)INFINITY, -0.0, 0.0, 4.0, (double)INFINITY, (double)NAN, -(double)NAN };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        volatile double input = values[i];
        feclearexcept(FE_ALL_EXCEPT);
        double result = safeSqrt(input);
        int exceptions = fetestexcept(FE_INVALID | FE_DIVBYZERO);
        if (exceptions != 0) {
            fprintf(stderr, "safeSqrt(%g) raised floating-point exceptions %d\n", (double)input, exceptions);
        }
        CHECK(exceptions == 0);
        if (isnan(values[i])) {
            CHECK(isnan(result));
        } else if (values[i] < 0.0) {
            CHECK(result == 0.0 && !signbit(result));
        } else if (values[i] == 0.0) {
            CHECK(result == 0.0 && !!signbit(result) == !!signbit(values[i]));
        } else {
            CHECK(result == sqrt(values[i]));
        }
    }
    return 0;
}

/** @brief Reject oversized scratch buffers before accessing even a null input. */
static int scratchLimits(void)
{
    const size_t tooMany = LINEAR_ALGEBRA_MAX_ARRAY_SIZE + 1;
    double output = 19.0;
    vtMultM(NULL, NULL, 1, tooMany, &output);
    vtMultMt(NULL, NULL, tooMany, 1, &output);
    mMultV(NULL, tooMany, 1, NULL, &output);
    mtMultV(NULL, 1, tooMany, NULL, &output);
    mTranspose(NULL, 65, 65, &output);
    mMultM(NULL, 65, 1, NULL, 1, 65, &output);
    mtMultM(NULL, 1, 65, NULL, 1, 65, &output);
    mMultMt(NULL, 65, 1, NULL, 65, 1, &output);
    mtMultMt(NULL, 1, 65, NULL, 65, 1, &output);
    CHECK(output == 19.0);
    return 0;
}

/** @brief Check both element-count and byte-count overflow without large allocations. */
static int dimensionOverflow(void)
{
    const size_t tooManyBytes = SIZE_MAX / sizeof(double) + 1;
    double output = 23.0;
    vCopy(NULL, tooManyBytes, &output);
    vSort(NULL, &output, tooManyBytes);
    vOuterProduct(NULL, SIZE_MAX, NULL, 2, &output);
    mCopy(NULL, SIZE_MAX, 2, &output);
    mCopy(NULL, 1, tooManyBytes, &output);
    mDiag(NULL, SIZE_MAX, &output);
    mTranspose(NULL, SIZE_MAX, 2, &output);
    /* Small output does not make unrepresentable input shapes valid. */
    vtMultM(NULL, NULL, tooManyBytes, 1, &output);
    vtMultMt(NULL, NULL, 1, tooManyBytes, &output);
    mMultV(NULL, 1, tooManyBytes, NULL, &output);
    mtMultV(NULL, tooManyBytes, 1, NULL, &output);
    mMultM(NULL, 1, tooManyBytes, NULL, tooManyBytes, 1, &output);
    mtMultM(NULL, tooManyBytes, 1, NULL, tooManyBytes, 1, &output);
    mMultMt(NULL, 1, tooManyBytes, NULL, 1, tooManyBytes, &output);
    mtMultMt(NULL, tooManyBytes, 1, NULL, 1, tooManyBytes, &output);
    /* These products wrap to zero if dimensions are multiplied before checking. */
    mMultM(NULL, SIZE_MAX / 2 + 1, 1, NULL, 1, 2, &output);
    mtMultM(NULL, 1, SIZE_MAX / 2 + 1, NULL, 1, 2, &output);
    mMultMt(NULL, SIZE_MAX / 2 + 1, 1, NULL, 2, 1, &output);
    mtMultMt(NULL, 1, SIZE_MAX / 2 + 1, NULL, 2, 1, &output);
    CHECK(output == 23.0);
    return 0;
}

/** @brief Reject submatrix bounds errors and overflowing parent shapes before access. */
static int submatrixBounds(void)
{
    double output = 29.0;
    const size_t invalid[][4] = { { 4, 0, 1, 1 },        { 0, 5, 1, 1 },        { 3, 0, 1, 1 },
                                  { 0, 4, 1, 1 },        { 2, 3, 2, 1 },        { 2, 3, 1, 2 },
                                  { SIZE_MAX, 0, 2, 1 }, { 0, SIZE_MAX, 1, 2 }, { 1, 1, SIZE_MAX, 1 },
                                  { 1, 1, 1, SIZE_MAX } };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
        mGetSubMatrix(NULL, 3, 4, invalid[i][0], invalid[i][1], invalid[i][2], invalid[i][3], &output);
        mSetSubMatrix(NULL, invalid[i][2], invalid[i][3], &output, 3, 4, invalid[i][0], invalid[i][1]);
    }
    mGetSubMatrix(NULL, SIZE_MAX, 2, 0, 0, 1, 1, &output);
    mSetSubMatrix(NULL, 1, 1, &output, SIZE_MAX, 2, 0, 0);
    CHECK(output == 29.0);
    return 0;
}

/** @brief Reject fixed block indices before reading or writing either matrix. */
static int blockIndices(void)
{
    double matrix[6][6];
    double block[3][3];
    for (size_t row = 0; row < 6; row++) {
        for (size_t col = 0; col < 6; col++) {
            matrix[row][col] = 31.0;
        }
    }
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 3; col++) {
            block[row][col] = 37.0;
        }
    }
    const size_t invalid[][2] = { { 2, 0 }, { 0, 2 }, { SIZE_MAX, 1 }, { 1, SIZE_MAX } };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
        m66Get33Matrix(invalid[i][0], invalid[i][1], NULL, block);
        m66Set33Matrix(invalid[i][0], invalid[i][1], NULL, matrix);
    }
    for (size_t row = 0; row < 6; row++) {
        for (size_t col = 0; col < 6; col++) {
            CHECK(matrix[row][col] == 31.0);
        }
    }
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 3; col++) {
            CHECK(block[row][col] == 37.0);
        }
    }
    return 0;
}

/** @brief Preserve zero-inner-dimension products and dimension-mismatch behavior. */
static int emptyInnerProducts(void)
{
    for (size_t variant = 0; variant < 8; variant++) {
        double result[6] = { 1, 2, 3, 4, 5, 6 };
        switch (variant) {
            case 0:
                mMultM(NULL, 2, 0, NULL, 0, 3, result);
                break;
            case 1:
                mtMultM(NULL, 0, 2, NULL, 0, 3, result);
                break;
            case 2:
                mMultMt(NULL, 2, 0, NULL, 3, 0, result);
                break;
            case 3:
                mtMultMt(NULL, 0, 2, NULL, 3, 0, result);
                break;
            case 4:
                vtMultM(NULL, NULL, 0, 6, result);
                break;
            case 5:
                vtMultMt(NULL, NULL, 6, 0, result);
                break;
            case 6:
                mMultV(NULL, 6, 0, NULL, result);
                break;
            case 7:
                mtMultV(NULL, 0, 6, NULL, result);
                break;
        }
        for (size_t i = 0; i < 6; i++) {
            CHECK(result[i] == 0.0);
        }
    }
    double output = 41.0;
    mMultM(NULL, 1, 2, NULL, 3, 1, &output);
    mtMultM(NULL, 2, 1, NULL, 3, 1, &output);
    mMultMt(NULL, 1, 2, NULL, 1, 3, &output);
    mtMultMt(NULL, 2, 1, NULL, 1, 3, &output);
    CHECK(output == 41.0);
    return 0;
}

/** @brief Exercise the full scratch capacity for transpose and all product variants. */
static int capacityBoundary(void)
{
    double input[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    double output[LINEAR_ALGEBRA_MAX_ARRAY_SIZE + 2];
    double left[64];
    double right[64];
    for (size_t i = 0; i < LINEAR_ALGEBRA_MAX_ARRAY_SIZE; i++) {
        input[i] = (double)i - 100.0;
    }
    output[0] = 43.0;
    output[LINEAR_ALGEBRA_MAX_ARRAY_SIZE + 1] = 47.0;
    mTranspose(input, 64, 64, output + 1);
    for (size_t row = 0; row < 64; row++) {
        for (size_t col = 0; col < 64; col++) {
            CHECK(output[1 + row * 64 + col] == input[col * 64 + row]);
        }
        left[row] = (double)row - 10.0;
        right[row] = (double)row + 1.0;
    }
    for (size_t variant = 0; variant < 4; variant++) {
        switch (variant) {
            case 0:
                mMultM(left, 64, 1, right, 1, 64, output + 1);
                break;
            case 1:
                mtMultM(left, 1, 64, right, 1, 64, output + 1);
                break;
            case 2:
                mMultMt(left, 64, 1, right, 64, 1, output + 1);
                break;
            case 3:
                mtMultMt(left, 1, 64, right, 64, 1, output + 1);
                break;
        }
        for (size_t row = 0; row < 64; row++) {
            for (size_t col = 0; col < 64; col++) {
                CHECK(output[1 + row * 64 + col] == left[row] * right[col]);
            }
        }
    }
    double scalar = 2.0;
    for (size_t variant = 0; variant < 4; variant++) {
        switch (variant) {
            case 0:
                vtMultM(&scalar, input, 1, LINEAR_ALGEBRA_MAX_ARRAY_SIZE, output + 1);
                break;
            case 1:
                vtMultMt(&scalar, input, LINEAR_ALGEBRA_MAX_ARRAY_SIZE, 1, output + 1);
                break;
            case 2:
                mMultV(input, LINEAR_ALGEBRA_MAX_ARRAY_SIZE, 1, &scalar, output + 1);
                break;
            case 3:
                mtMultV(input, 1, LINEAR_ALGEBRA_MAX_ARRAY_SIZE, &scalar, output + 1);
                break;
        }
        for (size_t i = 0; i < LINEAR_ALGEBRA_MAX_ARRAY_SIZE; i++) {
            CHECK(output[1 + i] == 2.0 * input[i]);
        }
    }
    CHECK(output[0] == 43.0);
    CHECK(output[LINEAR_ALGEBRA_MAX_ARRAY_SIZE + 1] == 47.0);
    return 0;
}

/** @brief Input arrays may exceed the scratch limit when their output still fits. */
static int largeProductInputs(void)
{
    enum
    {
        inner = LINEAR_ALGEBRA_MAX_ARRAY_SIZE + 1
    };
    double left[inner];
    double right[inner];
    for (size_t i = 0; i < inner; i++) {
        left[i] = 2.0;
        right[i] = 3.0;
    }
    for (size_t variant = 0; variant < 8; variant++) {
        double output = -1.0;
        switch (variant) {
            case 0:
                mMultM(left, 1, inner, right, inner, 1, &output);
                break;
            case 1:
                mtMultM(left, inner, 1, right, inner, 1, &output);
                break;
            case 2:
                mMultMt(left, 1, inner, right, 1, inner, &output);
                break;
            case 3:
                mtMultMt(left, inner, 1, right, 1, inner, &output);
                break;
            case 4:
                vtMultM(left, right, inner, 1, &output);
                break;
            case 5:
                vtMultMt(left, right, 1, inner, &output);
                break;
            case 6:
                mMultV(left, 1, inner, right, &output);
                break;
            case 7:
                mtMultV(left, inner, 1, right, &output);
                break;
        }
        CHECK(output == 6.0 * (double)inner);
    }
    return 0;
}

/** @brief Snapshot both outer-product inputs across overlapping offsets and rectangular shapes. */
static int outerProductOverlap(void)
{
    const size_t shapes[][2] = { { 1, 1 }, { 1, 4 }, { 4, 1 }, { 2, 3 }, { 3, 2 }, { 4, 4 } };
    double original[64];
    for (size_t i = 0; i < 64; i++) {
        original[i] = (double)i - 17.0;
    }
    for (size_t shape = 0; shape < sizeof(shapes) / sizeof(shapes[0]); shape++) {
        size_t rows = shapes[shape][0];
        size_t columns = shapes[shape][1];
        for (size_t first = 1; first < 40; first++) {
            for (size_t second = 1; second < 40; second++) {
                double storage[64];
                double expected[64];
                memcpy(storage, original, sizeof(storage));
                memcpy(expected, original, sizeof(expected));
                for (size_t row = 0; row < rows; row++) {
                    for (size_t col = 0; col < columns; col++) {
                        expected[16 + row * columns + col] = original[first + row] * original[second + col];
                    }
                }
                vOuterProduct(storage + first, rows, storage + second, columns, storage + 16);
                for (size_t i = 0; i < 64; i++) {
                    CHECK(storage[i] == expected[i]);
                }
            }
        }
    }
    return 0;
}

/** @brief Generic outer products retain sizes beyond the fixed scratch capacity. */
static int largeOuterProduct(void)
{
    double storage[65 * 65];
    double first[65];
    double second[65];
    for (size_t i = 0; i < 65; i++) {
        first[i] = (double)i - 20.0;
        second[i] = (double)i + 1.0;
    }
    for (size_t overlap = 0; overlap < 2; overlap++) {
        memcpy(storage, first, sizeof(first));
        memcpy(storage + 100, second, sizeof(second));
        vOuterProduct(overlap ? storage : first, 65, overlap ? storage + 100 : second, 65, storage);
        for (size_t row = 0; row < 65; row++) {
            for (size_t col = 0; col < 65; col++) {
                CHECK(storage[row * 65 + col] == first[row] * second[col]);
            }
        }
    }
    return 0;
}

/** @brief Exercise packed/strided block overlap in both directions and preserve untouched entries. */
static int submatrixOverlap(void)
{
    double original[96];
    for (size_t i = 0; i < 96; i++) {
        original[i] = (double)i - 12.0;
    }
    for (size_t extract = 0; extract < 2; extract++) {
        for (size_t rows = 1; rows <= 4; rows++) {
            for (size_t columns = 1; columns <= 5; columns++) {
                for (size_t startRow = 0; startRow + rows <= 4; startRow++) {
                    for (size_t startCol = 0; startCol + columns <= 5; startCol++) {
                        for (size_t block = 1; block < 64; block++) {
                            double storage[96];
                            double expected[96];
                            memcpy(storage, original, sizeof(storage));
                            memcpy(expected, original, sizeof(expected));
                            for (size_t row = 0; row < rows; row++) {
                                for (size_t col = 0; col < columns; col++) {
                                    size_t parentIndex = 24 + (startRow + row) * 5 + startCol + col;
                                    size_t blockIndex = block + row * columns + col;
                                    expected[extract ? blockIndex : parentIndex] =
                                        original[extract ? parentIndex : blockIndex];
                                }
                            }
                            if (extract) {
                                mGetSubMatrix(storage + 24, 4, 5, startRow, startCol, rows, columns, storage + block);
                            } else {
                                mSetSubMatrix(storage + block, rows, columns, storage + 24, 4, 5, startRow, startCol);
                            }
                            for (size_t i = 0; i < 96; i++) {
                                CHECK(storage[i] == expected[i]);
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}

/** @brief Submatrix overlap remains supported above the fixed scratch capacity. */
static int largeSubmatrix(void)
{
    double storage[65 * 66 + 2];
    double original[65 * 66 + 2];
    for (size_t i = 0; i < sizeof(storage) / sizeof(storage[0]); i++) {
        original[i] = (double)i;
    }
    memcpy(storage, original, sizeof(storage));
    mGetSubMatrix(storage, 65, 66, 0, 1, 65, 65, storage + 2);
    for (size_t row = 0; row < 65; row++) {
        for (size_t col = 0; col < 65; col++) {
            CHECK(storage[2 + row * 65 + col] == original[row * 66 + 1 + col]);
        }
    }
    CHECK(storage[0] == original[0] && storage[1] == original[1]);
    memcpy(storage, original, sizeof(storage));
    mSetSubMatrix(storage + 2, 65, 65, storage, 65, 66, 0, 1);
    for (size_t row = 0; row < 65; row++) {
        CHECK(storage[row * 66] == original[row * 66]);
        for (size_t col = 0; col < 65; col++) {
            CHECK(storage[row * 66 + 1 + col] == original[2 + row * 65 + col]);
        }
    }
    return 0;
}

/** @brief Run one named regression, or all regressions when no name is supplied. */
int main(int argc, char** argv)
{
    const struct
    {
        const char* name;
        int (*run)(void);
    } tests[] = {
        { "emptyOperations", emptyOperations },
        { "emptyReductions", emptyReductions },
        { "safeSquareRoot", safeSquareRoot },
        { "scratchLimits", scratchLimits },
        { "dimensionOverflow", dimensionOverflow },
        { "submatrixBounds", submatrixBounds },
        { "blockIndices", blockIndices },
        { "emptyInnerProducts", emptyInnerProducts },
        { "capacityBoundary", capacityBoundary },
        { "largeProductInputs", largeProductInputs },
        { "outerProductOverlap", outerProductOverlap },
        { "largeOuterProduct", largeOuterProduct },
        { "submatrixOverlap", submatrixOverlap },
        { "largeSubmatrix", largeSubmatrix },

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
