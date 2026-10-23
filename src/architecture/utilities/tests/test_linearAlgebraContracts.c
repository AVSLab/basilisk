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
#include <string.h>

/* Keep checks active in Release builds. */
#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);                                            \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

/** @brief Reject singular integer matrices even when elimination leaves a nonzero pivot. */
static int singularInverse(void)
{
    const double cases[][9] = {
        { 10, 11, -5, 11, 13, -7, -5, -7, 5 }, /* A positive-semidefinite rank-two Gram matrix. */
        { 13, 9, 4, 9, 10, 6, 4, 6, 4 },
        { -5, -5, 1, -1, 4, 4, -6, -1, 5 }, /* Last row is the sum of the first two. */
        { 1, 1, 1, 3, 3, 3, 0, 1, 2 }
    };
    const size_t permutations[][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
    const size_t offsets[] = { 0, 4, 5, 6, 14 };
    for (size_t test = 0; test < sizeof(cases) / sizeof(cases[0]); test++) {
        for (size_t permutation = 0; permutation < 6; permutation++) {
            for (size_t output = 0; output < sizeof(offsets) / sizeof(offsets[0]); output++) {
                double storage[24], original[24];
                for (size_t i = 0; i < 24; i++) {
                    storage[i] = 123.0;
                }
                for (size_t row = 0; row < 3; row++) {
                    for (size_t col = 0; col < 3; col++) {
                        storage[5 + row * 3 + col] = cases[test][permutations[permutation][row] * 3 + col];
                    }
                }
                memcpy(original, storage, sizeof(storage));
                feclearexcept(FE_ALL_EXCEPT);
                CHECK(mInverse(storage + 5, 3, storage + offsets[output]) == 1);
                CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
                for (size_t i = 0; i < 24; i++) {
                    double expected = i >= offsets[output] && i < offsets[output] + 9 ? 0.0 : original[i];
                    CHECK(storage[i] == expected);
                }
            }
        }
    }
    /* Fused subtraction can leave a nonzero residual even for these proportional rows. */
    double pair[] = { 1, 1, 3, 3 };
    double output[4];
    CHECK(mInverse(pair, 2, output) == 1);
    for (size_t i = 0; i < 4; i++) {
        CHECK(output[i] == 0.0);
    }
    return 0;
}

/** @brief Include elimination growth and propagated roundoff in singularity checks. */
static int singularInverseGrowth(void)
{
    double matrix[] = {
        7, 7,  -3, -5, 2,  5, -3, 3, -8, 8,  6,  4, 5, 8,  -7,  2,  6,  -7,
        3, -1, 6,  -5, -2, 5, -5, 3, -7, -4, -2, 6, 1, 13, -19, 11, 14, 13
    }; /* Last row is the first row plus twice the second row. */
    double inverse[36];
    CHECK(mInverse(matrix, 6, inverse) == 1);
    for (size_t i = 0; i < 36; i++) {
        CHECK(inverse[i] == 0.0);
    }
    double scaled[] = { -2, -1, -8, -3, -1.75, 1, -26, -15, 0 };
    const double rowScales[] = { 1, 1, 8192 };
    const double columnScales[] = { 0x1p23, 32, 24576 };
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 3; col++) {
            scaled[row * 3 + col] *= rowScales[row] * columnScales[col];
        }
    }
    CHECK(mInverse(scaled, 3, inverse) == 1);
    for (size_t i = 0; i < 9; i++) {
        CHECK(inverse[i] == 0.0);
    }
    return 0;
}

/** @brief Retain valid small pivots in graded diagonal and triangular matrices. */
static int inversePivotScales(void)
{
    const double small = 0x1p-60;
    const double large = 0x1p60;
    const double offDiagonal = 0x1p30;
    double cases[][9] = { { small, 0, 0, 0, large, 0, 0, 0, 1 },
                          { 1, offDiagonal, 0, 0, 1, 0, 0, 0, 1 },
                          { offDiagonal, 1, 0, offDiagonal, 2, 0, 0, 0, 1 },
                          { 0, 0, small, large, 0, 0, 0, 1, 0 } };
    for (size_t test = 0; test < sizeof(cases) / sizeof(cases[0]); test++) {
        double inverse[9];
        CHECK(mInverse(cases[test], 3, inverse) == 0);
        for (size_t row = 0; row < 3; row++) {
            for (size_t col = 0; col < 3; col++) {
                double left = 0.0, right = 0.0;
                for (size_t inner = 0; inner < 3; inner++) {
                    left += cases[test][row * 3 + inner] * inverse[inner * 3 + col];
                    right += inverse[row * 3 + inner] * cases[test][inner * 3 + col];
                }
                CHECK(left == (row == col ? 1.0 : 0.0));
                CHECK(right == (row == col ? 1.0 : 0.0));
            }
        }
    }
    return 0;
}

/** @brief Distinguish cancellation at roundoff from resolvable pivots without truncating determinants. */
static int inverseRankBoundary(void)
{
    double matrix[] = { 1, 1, 1, 1 + DBL_EPSILON };
    double inverse[4], cofactors[4];
    CHECK(mInverse(matrix, 2, inverse) == 1);
    CHECK(mDeterminant(matrix, 2) == DBL_EPSILON);
    mCofactor(matrix, 2, cofactors);
    CHECK(cofactors[0] == matrix[3] && cofactors[1] == -1.0 && cofactors[2] == -1.0 && cofactors[3] == 1.0);
    double delta = 64.0 * DBL_EPSILON;
    matrix[3] = 1.0 + delta;
    CHECK(mInverse(matrix, 2, inverse) == 0);
    CHECK(inverse[0] == 1.0 + 1.0 / delta);
    CHECK(inverse[1] == -1.0 / delta && inverse[2] == -1.0 / delta && inverse[3] == 1.0 / delta);
    return 0;
}

/** @brief Check inverse status, both identities, and storage boundaries for overlapping small matrices. */
static int checkConditionInverse(const double* matrix, size_t dim, int expectedStatus)
{
    const size_t offsets[] = { 0, 16, 17, 18, 34 };
    for (size_t layout = 0; layout < sizeof(offsets) / sizeof(offsets[0]); layout++) {
        double storage[52], saved[52];
        for (size_t i = 0; i < 52; i++) {
            storage[i] = 123.0;
        }
        memcpy(storage + 17, matrix, dim * dim * sizeof(double));
        memcpy(saved, storage, sizeof(storage));
        double* inverse = storage + offsets[layout];
        feclearexcept(FE_ALL_EXCEPT);
        CHECK(mInverse(storage + 17, dim, inverse) == expectedStatus);
        CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
        for (size_t i = 0; i < 52; i++) {
            CHECK((i >= offsets[layout] && i < offsets[layout] + dim * dim) || storage[i] == saved[i]);
        }
        for (size_t row = 0; row < dim; row++) {
            for (size_t col = 0; col < dim; col++) {
                if (expectedStatus) {
                    CHECK(inverse[row * dim + col] == 0.0);
                } else {
                    double left = 0.0, right = 0.0;
                    for (size_t inner = 0; inner < dim; inner++) {
                        left += matrix[row * dim + inner] * inverse[inner * dim + col];
                        right += inverse[row * dim + inner] * matrix[inner * dim + col];
                    }
                    CHECK(isfinite(left) && fabs(left - (row == col ? 1.0 : 0.0)) < 2e-13);
                    CHECK(isfinite(right) && fabs(right - (row == col ? 1.0 : 0.0)) < 2e-13);
                }
            }
        }
    }
    return 0;
}

/** @brief Reject exactly dependent integer rows and columns that can pass the LU pivot guard. */
static int inverseConditionSingular(void)
{
    const double original[16] = { 3, -7, 10, -8, -4, -10, 4, 3, -6, -4, -4, 6, -5, -27, 18, -2 };
    /* The last row equals the first row plus twice the second row. Different scales
     * expose the regression with and without fused multiply-add instructions. */
    const double scales[] = { 7, 10, 1, 0.125, 1024 };
    for (size_t scale = 0; scale < sizeof(scales) / sizeof(scales[0]); scale++) {
        for (size_t shift = 0; shift < 4; shift++) {
            for (size_t transpose = 0; transpose < 2; transpose++) {
                double matrix[16];
                for (size_t row = 0; row < 4; row++) {
                    for (size_t col = 0; col < 4; col++) {
                        matrix[transpose ? col * 4 + row : row * 4 + col] =
                            scales[scale] * original[((row + shift) % 4) * 4 + col];
                    }
                }
                CHECK(checkConditionInverse(matrix, 4, 1) == 0);
            }
        }
    }
    return 0;
}

/** @brief Reject poor scaled conditioning even when individual pivots and the determinant are large. */
static int inverseConditionBoundary(void)
{
    for (int exponent = 23; exponent <= 25; exponent++) {
        double matrix[16];
        mSetIdentity(matrix, 4, 4);
        for (size_t row = 0; row < 3; row++) {
            matrix[row * 4 + row + 1] = ldexp(1.0, exponent);
        }
        /* After row and column scaling, rcond = 1 / (2 * (2*q*q + q + 1)),
         * where q = 2^exponent. It crosses 4*DBL_EPSILON at exponent 24. */
        CHECK(checkConditionInverse(matrix, 4, exponent >= 24) == 0);
    }
    const size_t dimensions[] = { 2, 4, 16, 64 };
    for (size_t test = 0; test < sizeof(dimensions) / sizeof(dimensions[0]); test++) {
        size_t dim = dimensions[test];
        for (size_t accepts = 0; accepts < 2; accepts++) {
            double matrix[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
            mSetIdentity(matrix, dim, dim);
            double delta = (accepts ? 8.0 : 3.0) * (double)dim * DBL_EPSILON;
            matrix[1] = matrix[dim] = 1.0;
            matrix[dim + 1] = 1.0 + delta;
            CHECK(mInverse(matrix, dim, matrix) == (accepts ? 0 : 1));
            if (accepts) {
                CHECK(matrix[0] == 1.0 + 1.0 / delta);
                CHECK(matrix[1] == -1.0 / delta && matrix[dim] == -1.0 / delta);
                CHECK(matrix[dim + 1] == 1.0 / delta);
            } else {
                CHECK(mIsZero(matrix, dim, dim, 0.0));
            }
        }
    }
    return 0;
}

/** @brief Preserve valid inverses under independent row and column scaling, including overlapping buffers. */
static int inverseConditionScales(void)
{
    const double small = 0x1p-60, large = 0x1p60;
    const double cases[][4] = { { small, 0, 0, large },
                                { 0, small, large, 0 },
                                { 1, 0x1p30, 0, 1 },
                                { 0x1p30, 1, 0x1p30, 2 },
                                { 0x1p-500, 0, 0, 0x1p500 } };
    for (size_t test = 0; test < sizeof(cases) / sizeof(cases[0]); test++) {
        CHECK(checkConditionInverse(cases[test], 2, 0) == 0);
    }
    const double rowScales[] = { 0x1p-20, 0x1p20 };
    const double columnScales[] = { 0x1p15, 0x1p-15 };
    const double scales[] = { 0.125, 1, 1024 };
    for (size_t test = 0; test < sizeof(scales) / sizeof(scales[0]); test++) {
        double matrix[4];
        for (size_t row = 0; row < 2; row++) {
            for (size_t col = 0; col < 2; col++) {
                matrix[row * 2 + col] = (row == col ? 2.0 : 1.0) * rowScales[row] * columnScales[col] * scales[test];
            }
        }
        CHECK(checkConditionInverse(matrix, 2, 0) == 0);
    }
    const double scalars[] = { -3, nextafter(DB0_EPS, 1.0), 1e308 };
    for (size_t test = 0; test < sizeof(scalars) / sizeof(scalars[0]); test++) {
        CHECK(checkConditionInverse(scalars + test, 1, 0) == 0);
    }
    return 0;
}

/** @brief Exercise every generic binary arithmetic entry point. */
static void binaryOperation(size_t operation, double* first, double* second, size_t count, double* result)
{
    switch (operation) {
        case 0:
            vAdd(first, count, second, result);
            break;
        case 1:
            vSubtract(first, count, second, result);
            break;
        case 2:
            vElementwiseMult(first, count, second, result);
            break;
        case 3:
            mAdd(first, 1, count, second, result);
            break;
        case 4:
            mSubtract(first, 1, count, second, result);
            break;
    }
}

/** @brief Bound conflicting overlap while preserving unrestricted directional and exact in-place arithmetic. */
static int arithmeticStorage(void)
{
    enum
    {
        capacity = LINEAR_ALGEBRA_MAX_ARRAY_SIZE,
        count = 2 * capacity + 2,
        storageCount = 3 * count + 4
    };
    static double storage[storageCount], saved[storageCount];
    const struct
    {
        size_t first, second, output, size;
        int succeeds;
    } layouts[] = { { 0, count - 1, capacity, count - 1, 1 }, /* Exactly capacity endangered values. */
                    { 0, count - 1, capacity + 1, count - 1, 1 },
                    { count - 1, 0, capacity, count - 1, 1 }, /* Subtraction must retain operand order. */
                    { count - 1, 0, capacity + 1, count - 1, 1 },
                    { 0, count, capacity + 1, count, 0 }, /* Both endangered overlaps exceed capacity. */
                    { count, 0, capacity + 1, count, 0 },
                    { 0, count + 2, 1, count, 1 }, /* Backward traversal, no scratch limit. */
                    { 1, count + 2, 0, count, 1 }, /* Forward traversal, no scratch limit. */
                    { 1, 2, 1, count, 1 },         /* Previously safe exact in-place plus shifted second input. */
                    { 2, 1, 2, count, 1 },
                    { 0, count, 2 * count, count, 1 } };
    for (size_t operation = 0; operation < 5; operation++) {
        for (size_t layout = 0; layout < sizeof(layouts) / sizeof(layouts[0]); layout++) {
            for (size_t i = 0; i < storageCount; i++) {
                storage[i] = (double)(i % 17) - 8.0;
            }
            memcpy(saved, storage, sizeof(storage));
            size_t first = layouts[layout].first, second = layouts[layout].second;
            size_t output = layouts[layout].output, size = layouts[layout].size;
            binaryOperation(operation, storage + first, storage + second, size, storage + output);
            for (size_t i = 0; i < storageCount; i++) {
                double expected = saved[i];
                if (layouts[layout].succeeds && i >= output && i < output + size) {
                    double a = saved[first + i - output], b = saved[second + i - output];
                    expected = operation == 2 ? a * b : (operation == 0 || operation == 3 ? a + b : a - b);
                }
                CHECK(storage[i] == expected);
            }
        }
    }
    return 0;
}

/** @brief Save only outer-product inputs, check the scratch boundary, and retain long vector products. */
static int outerProductStorage(void)
{
    enum
    {
        capacity = LINEAR_ALGEBRA_MAX_ARRAY_SIZE,
        storageCount = 3 * capacity + 16
    };
    static double storage[storageCount], saved[storageCount], separate[capacity + 3];
    double first[2] = { 2, -3 };
    for (size_t i = 0; i < capacity + 3; i++) {
        separate[i] = (double)(i % 19) - 9.0;
    }
    for (size_t test = 0; test < 6; test++) {
        for (size_t i = 0; i < storageCount; i++) {
            storage[i] = (double)(i % 13) - 6.0;
        }
        memcpy(saved, storage, sizeof(storage));
        size_t columns = test < 2 ? capacity - 2 + test : capacity + (test == 3 ? 1 : 0);
        int firstOverlaps = test < 2 || test == 5;
        int succeeds = test != 1 && test != 3;
        if (test >= 4) {
            columns = capacity + 3;
        }
        double* second = test >= 4 ? separate : storage + 8;
        vOuterProduct(firstOverlaps ? storage + 1 : first, 2, second, columns, storage);
        for (size_t i = 0; i < storageCount; i++) {
            double expected = saved[i];
            if (succeeds && i < 2 * columns) {
                double a = firstOverlaps ? saved[1 + i / columns] : first[i / columns];
                double b = test >= 4 ? separate[i % columns] : saved[8 + i % columns];
                expected = a * b;
            }
            CHECK(storage[i] == expected);
        }
    }
    for (size_t transpose = 0; transpose < 2; transpose++) {
        for (size_t i = 0; i < storageCount; i++) {
            storage[i] = (double)(i % 13) - 6.0;
        }
        memcpy(saved, storage, sizeof(storage));
        size_t size = 2 * capacity + 1;
        if (transpose) {
            vOuterProduct(storage + 4, size, storage + 3, 1, storage + 2);
        } else {
            vOuterProduct(storage + 3, 1, storage + 4, size, storage + 2);
        }
        for (size_t i = 0; i < storageCount; i++) {
            double expected = i >= 2 && i < 2 + size ? saved[3] * saved[4 + i - 2] : saved[i];
            CHECK(storage[i] == expected);
        }
    }
    return 0;
}

/** @brief Copy large overlapping blocks when the safe row direction changes within the block. */
static int submatrixTraversal(void)
{
    enum
    {
        rows = 32,
        columns = 300,
        parentColumns = 307,
        storageCount = 24000,
        parent = 1000
    };
    static double storage[storageCount], expected[storageCount], original[storageCount];
    const size_t blocks[] = { 0, parent + 7, parent + 10, parent + 107, parent + 300, parent + 12000 };
    for (size_t i = 0; i < storageCount; i++) {
        original[i] = (double)i;
    }
    for (size_t extract = 0; extract < 2; extract++) {
        for (size_t layout = 0; layout < sizeof(blocks) / sizeof(blocks[0]); layout++) {
            memcpy(storage, original, sizeof(storage));
            memcpy(expected, original, sizeof(expected));
            size_t block = blocks[layout];
            for (size_t row = 0; row < rows; row++) {
                for (size_t col = 0; col < columns; col++) {
                    size_t source = parent + row * parentColumns + 7 + col;
                    size_t packed = block + row * columns + col;
                    expected[extract ? packed : source] = original[extract ? source : packed];
                }
            }
            if (extract) {
                mGetSubMatrix(storage + parent, rows, parentColumns, 0, 7, rows, columns, storage + block);
            } else {
                mSetSubMatrix(storage + block, rows, columns, storage + parent, rows, parentColumns, 0, 7);
            }
            CHECK(memcmp(storage, expected, sizeof(storage)) == 0);
        }
    }
    return 0;
}

/** @brief Verify supported small inputs at the documented perpendicular branch boundaries. */
static int perpendicularRange(void)
{
    const double cases[][3] = { { nextafter(DB0_EPS, 1.0), 0, 0 },
                                { 0, nextafter(DB0_EPS, 1.0), 0 },
                                { DB0_EPS, DB0_EPS, 1e-31 },
                                { 0, 0, 1e-31 },
                                { 0, 0, 0 } };
    for (size_t test = 0; test < sizeof(cases) / sizeof(cases[0]); test++) {
        for (size_t inPlace = 0; inPlace < 2; inPlace++) {
            double input[3], output[3];
            memcpy(input, cases[test], sizeof(input));
            double* result = inPlace ? input : output;
            feclearexcept(FE_ALL_EXCEPT);
            v3Perpendicular(input, result);
            CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
            double normSquared = 0.0, dot = 0.0;
            for (size_t i = 0; i < 3; i++) {
                CHECK(isfinite(result[i]));
                normSquared += result[i] * result[i];
                dot += cases[test][i] * result[i];
            }
            CHECK(fabs(dot) < 1e-45);
            CHECK(fabs(normSquared - (test == 4 ? 0.0 : 1.0)) <= 4.0 * DBL_EPSILON);
            if (!inPlace) {
                CHECK(memcmp(input, cases[test], sizeof(input)) == 0);
            }
        }
    }
    return 0;
}

/** @brief Run a named contract regression, or all regressions if no name is supplied. */
int main(int argc, char** argv)
{
    const struct
    {
        const char* name;
        int (*run)(void);
    } tests[] = { { "singularInverse", singularInverse },
                  { "singularInverseGrowth", singularInverseGrowth },
                  { "inversePivotScales", inversePivotScales },
                  { "inverseRankBoundary", inverseRankBoundary },
                  { "inverseConditionSingular", inverseConditionSingular },
                  { "inverseConditionBoundary", inverseConditionBoundary },
                  { "inverseConditionScales", inverseConditionScales },
                  { "arithmeticStorage", arithmeticStorage },
                  { "outerProductStorage", outerProductStorage },
                  { "submatrixTraversal", submatrixTraversal },
                  { "perpendicularRange", perpendicularRange } };
    int found = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        if (argc == 1 || strcmp(argv[1], tests[i].name) == 0) {
            found = 1;
            int status = tests[i].run();
            printf("%s: %s\n", tests[i].name, status ? "FAIL" : "PASS");
            if (status) {
                return status;
            }
        }
    }
    return found ? 0 : 1;
}
