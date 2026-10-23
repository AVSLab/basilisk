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
    } tests[] = {

        { "arithmeticStorage", arithmeticStorage },
        { "outerProductStorage", outerProductStorage },
        { "submatrixTraversal", submatrixTraversal },
        { "perpendicularRange", perpendicularRange }
    };
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
