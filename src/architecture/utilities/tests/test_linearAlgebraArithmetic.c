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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Keep checks active in optimized builds. */
#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);                                            \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

enum Operation
{
    add,
    subtract,
    multiply,
    scale,
    normalize
};
static const struct ArithmeticCase
{
    const char* name;
    size_t rows;
    size_t columns;
    enum Operation operation;
} arithmeticCases[] = {
    { "vElementwiseMultOverlap", 5, 1, multiply },
    { "vAddOverlap", 5, 1, add },
    { "vSubtractOverlap", 5, 1, subtract },
    { "vScaleOverlap", 5, 1, scale },
    { "vNormalizeOverlap", 5, 1, normalize },
    { "v2AddOverlap", 2, 1, add },
    { "v2SubtractOverlap", 2, 1, subtract },
    { "v2ScaleOverlap", 2, 1, scale },
    { "v2NormalizeOverlap", 2, 1, normalize },
    { "v3AddOverlap", 3, 1, add },
    { "v3SubtractOverlap", 3, 1, subtract },
    { "v3ScaleOverlap", 3, 1, scale },
    { "v3NormalizeOverlap", 3, 1, normalize },
    { "v6ScaleOverlap", 6, 1, scale },
    { "mAddOverlap", 2, 3, add },
    { "mSubtractOverlap", 2, 3, subtract },
    { "mScaleOverlap", 2, 3, scale },
    { "m22AddOverlap", 2, 2, add },
    { "m22SubtractOverlap", 2, 2, subtract },
    { "m22ScaleOverlap", 2, 2, scale },
    { "m33AddOverlap", 3, 3, add },
    { "m33SubtractOverlap", 3, 3, subtract },
    { "m33ScaleOverlap", 3, 3, scale },
    { "m66AddOverlap", 6, 6, add },
    { "m66SubtractOverlap", 6, 6, subtract },
    { "m66ScaleOverlap", 6, 6, scale },
};

/** @brief Dispatch generic operations separately so dimension tests need no fixed-size buffers. */
static void genericArithmetic(size_t test, double* first, double* second, double* output, size_t rows, size_t columns)
{
    switch (test) {
        case 0:
            vElementwiseMult(first, rows * columns, second, output);
            break;
        case 1:
            vAdd(first, rows * columns, second, output);
            break;
        case 2:
            vSubtract(first, rows * columns, second, output);
            break;
        case 3:
            vScale(-2.5, first, rows * columns, output);
            break;
        case 4:
            vNormalize(first, rows * columns, output);
            break;
        case 14:
            mAdd(first, rows, columns, second, output);
            break;
        case 15:
            mSubtract(first, rows, columns, second, output);
            break;
        case 16:
            mScale(-2.5, first, rows, columns, output);
            break;
    }
}

/** @brief Adapt the public arithmetic signatures without casting function pointers. */
static void arithmetic(size_t test, double* first, double* second, double* output, size_t rows, size_t columns)
{
    switch (test) {
        case 5:
            v2Add(first, second, output);
            break;
        case 6:
            v2Subtract(first, second, output);
            break;
        case 7:
            v2Scale(-2.5, first, output);
            break;
        case 8:
            v2Normalize(first, output);
            break;
        case 9:
            v3Add(first, second, output);
            break;
        case 10:
            v3Subtract(first, second, output);
            break;
        case 11:
            v3Scale(-2.5, first, output);
            break;
        case 12:
            v3Normalize(first, output);
            break;
        case 13:
            v6Scale(-2.5, first, output);
            break;
        case 17:
            m22Add((double (*)[2])first, (double (*)[2])second, (double (*)[2])output);
            break;
        case 18:
            m22Subtract((double (*)[2])first, (double (*)[2])second, (double (*)[2])output);
            break;
        case 19:
            m22Scale(-2.5, (double (*)[2])first, (double (*)[2])output);
            break;
        case 20:
            m33Add((double (*)[3])first, (double (*)[3])second, (double (*)[3])output);
            break;
        case 21:
            m33Subtract((double (*)[3])first, (double (*)[3])second, (double (*)[3])output);
            break;
        case 22:
            m33Scale(-2.5, (double (*)[3])first, (double (*)[3])output);
            break;
        case 23:
            m66Add((double (*)[6])first, (double (*)[6])second, (double (*)[6])output);
            break;
        case 24:
            m66Subtract((double (*)[6])first, (double (*)[6])second, (double (*)[6])output);
            break;
        case 25:
            m66Scale(-2.5, (double (*)[6])first, (double (*)[6])output);
            break;
        default:
            genericArithmetic(test, first, second, output, rows, columns);
            break;
    }
}

/** @brief Compare against untouched inputs for disjoint, exact, and shifted overlap layouts. */
static int arithmeticOverlap(size_t test)
{
    const struct ArithmeticCase* item = &arithmeticCases[test];
    size_t count = item->rows * item->columns;
    const size_t outputs[] = { 0, 20, 39, 40, 41, 60, 79, 80, 81, 120 };
    for (size_t sameInput = 0; sameInput < 2; sameInput++) {
        size_t second = sameInput ? 40 : 80;
        for (size_t layout = 0; layout < sizeof(outputs) / sizeof(outputs[0]); layout++) {
            double storage[160], saved[160];
            for (size_t i = 0; i < 160; i++) {
                storage[i] = (double)(i % 23) - 11.25;
            }
            memcpy(saved, storage, sizeof(storage));
            size_t output = outputs[layout];
            arithmetic(test, storage + 40, storage + second, storage + output, item->rows, item->columns);
            long double normSquared = 0.0L;
            for (size_t i = 0; i < count; i++) {
                normSquared += (long double)saved[40 + i] * (long double)saved[40 + i];
            }
            for (size_t i = 0; i < count; i++) {
                double a = saved[40 + i], b = saved[second + i];
                double expected = 0.0;
                switch (item->operation) {
                    case add:
                        expected = a + b;
                        break;
                    case subtract:
                        expected = a - b;
                        break;
                    case multiply:
                        expected = a * b;
                        break;
                    case scale:
                        expected = -2.5 * a;
                        break;
                    case normalize:
                        expected = a / (double)sqrtl(normSquared);
                        break;
                }
                CHECK(isfinite(storage[output + i]));
                CHECK(fabs(storage[output + i] - expected) <= 4e-15 * fmax(1.0, fabs(expected)));
            }
            for (size_t i = 0; i < 160; i++) {
                CHECK((i >= output && i < output + count) || storage[i] == saved[i]);
            }
        }
    }
    return 0;
}

/** @brief Keep generic arithmetic unrestricted by the fixed scratch-buffer capacity. */
static int largeArithmetic(void)
{
    enum
    {
        count = 5001,
        storageCount = 3 * count + 2
    };
    double* storage = malloc(storageCount * sizeof(double));
    double* saved = malloc(storageCount * sizeof(double));
    CHECK(storage != NULL && saved != NULL);
    const size_t tests[] = { 0, 1, 2, 3, 4, 14, 15, 16 };
    for (size_t entry = 0; entry < sizeof(tests) / sizeof(tests[0]); entry++) {
        size_t test = tests[entry];
        for (size_t i = 0; i < storageCount; i++) {
            storage[i] = (double)(i % 11) - 4.0;
        }
        memcpy(saved, storage, storageCount * sizeof(double));
        /* The output overlaps both binary inputs in opposite directions. */
        genericArithmetic(test, storage, storage + count, storage + count / 2, 3, count / 3);
        double normSquared = 0.0;
        for (size_t i = 0; i < count; i++)
            normSquared += saved[i] * saved[i];
        for (size_t i = 0; i < count; i++) {
            double expected = 0.0;
            switch (arithmeticCases[test].operation) {
                case add:
                    expected = saved[i] + saved[count + i];
                    break;
                case subtract:
                    expected = saved[i] - saved[count + i];
                    break;
                case multiply:
                    expected = saved[i] * saved[count + i];
                    break;
                case scale:
                    expected = -2.5 * saved[i];
                    break;
                case normalize:
                    expected = saved[i] / sqrt(normSquared);
                    break;
            }
            CHECK(isfinite(storage[count / 2 + i]));
            CHECK(fabs(storage[count / 2 + i] - expected) <= 4e-15 * fmax(1.0, fabs(expected)));
        }
        for (size_t i = 0; i < storageCount; i++) {
            CHECK((i >= count / 2 && i < count / 2 + count) || storage[i] == saved[i]);
        }
    }
    free(saved);
    free(storage);
    return 0;
}

/** @brief Reject unrepresentable dimensions before accessing memory, and allow empty inputs. */
static int arithmeticDimensions(void)
{
    const size_t tests[] = { 0, 1, 2, 3, 4, 14, 15, 16 };
    double output = 17.0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        genericArithmetic(tests[i], NULL, NULL, NULL, 0, 0);
        genericArithmetic(tests[i], NULL, NULL, &output, SIZE_MAX / sizeof(double) + 1, 1);
        if (tests[i] >= 14) {
            genericArithmetic(tests[i], NULL, NULL, NULL, SIZE_MAX, 0);
            genericArithmetic(tests[i], NULL, NULL, NULL, 0, SIZE_MAX);
            genericArithmetic(tests[i], NULL, NULL, &output, SIZE_MAX, 2);
        }
    }
    CHECK(output == 17.0);
    return 0;
}

/** @brief Propagate any NaN in a maximum, independent of its position or infinity elsewhere. */
static int maximumNonfinite(void)
{
    for (size_t position = 0; position < 5; position++) {
        double values[] = { -7, (double)INFINITY, 4, -(double)INFINITY, -9 };
        values[position] = nan("");
        CHECK(isnan(vMax(values, 5)));
        CHECK(isnan(vMaxAbs(values, 5)));
    }
    double values[] = { -7, 4, -9 };
    CHECK(vMax(values, 3) == 4.0 && vMaxAbs(values, 3) == 9.0);
    double negativeInfinity[] = { -(double)INFINITY, -(double)INFINITY };
    CHECK(vMax(negativeInfinity, 2) == -(double)INFINITY);
    CHECK(vMaxAbs(negativeInfinity, 2) == (double)INFINITY);
    double zeros[] = { -0.0, 0.0 };
    CHECK(signbit(vMax(zeros, 2)));
    CHECK(!signbit(vMaxAbs(zeros, 2)));
    return 0;
}

/** @brief Normalize valid inputs while preserving the small-norm cutoff and flagging invalid norms. */
static int normalizationPolicy(void)
{
    const size_t tests[] = { 4, 8, 12 };
    const double nonfinite[] = { nan(""), (double)INFINITY, -(double)INFINITY };
    for (size_t entry = 0; entry < 3; entry++) {
        size_t test = tests[entry];
        size_t count = arithmeticCases[test].rows;
        for (size_t invalid = 0; invalid < 3; invalid++) {
            for (size_t position = 0; position < count; position++) {
                for (size_t offset = 0; offset < 3; offset++) {
                    double storage[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
                    storage[position + 1] = nonfinite[invalid];
                    feclearexcept(FE_ALL_EXCEPT);
                    arithmetic(test, storage + 1, NULL, storage + offset, count, 1);
                    for (size_t i = 0; i < count; i++)
                        CHECK(isnan(storage[offset + i]));
                    CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
                }
            }
        }
        const double small[] = { 0, 0.5 * DB0_EPS, 2 * DB0_EPS };
        for (size_t i = 0; i < 3; i++) {
            double input[5] = { 0 }, output[5];
            input[0] = small[i];
            arithmetic(test, input, NULL, output, count, 1);
            CHECK(output[0] == (i < 2 ? 0.0 : 1.0));
            for (size_t j = 1; j < count; j++)
                CHECK(output[j] == 0.0);
        }
        double input[5] = { 3, 4, 0, 0, 0 }, output[5];
        arithmetic(test, input, NULL, output, count, 1);
        CHECK(fabs(output[0] - 0.6) < 2e-16 && fabs(output[1] - 0.8) < 2e-16);
        /* Scaling the norm to handle extreme finite values remains outside this batch. */
        input[0] = DBL_MAX;
        arithmetic(test, input, NULL, output, count, 1);
        for (size_t i = 0; i < count; i++)
            CHECK(isnan(output[i]));
    }
    return 0;
}

/** @brief Sort ordinary values and infinities stably, leaving all NaNs at the end. */
static int sortNonfinite(void)
{
    double input[] = { nan("1"), 2, 0.0, -(double)INFINITY, nan("2"), -0.0, (double)INFINITY, -2 };
    const double expected[] = { -(double)INFINITY, -2, 0.0, -0.0, 2, (double)INFINITY };
    for (size_t offset = 0; offset < 3; offset++) {
        double storage[12];
        memcpy(storage + 1, input, sizeof(input));
        vSort(storage + 1, storage + offset, 8);
        CHECK(memcmp(storage + offset, expected, sizeof(expected)) == 0);
        CHECK(memcmp(storage + offset + 6, input, sizeof(double)) == 0);
        CHECK(memcmp(storage + offset + 7, input + 4, sizeof(double)) == 0);
    }
    double allNaNs[] = { nan("3"), nan("4"), nan("5") }, result[3];
    vSort(allNaNs, result, 3);
    CHECK(memcmp(allNaNs, result, sizeof(result)) == 0);
    return 0;
}

/** @brief Verify descending order for every arrangement of two numbers and a NaN. */
static int sortThreeNonfinite(void)
{
    const size_t permutations[][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
    const double values[] = { -(double)INFINITY, (double)INFINITY, nan("1") };
    for (size_t permutation = 0; permutation < 6; permutation++) {
        for (size_t offset = 0; offset < 3; offset++) {
            double storage[5];
            for (size_t i = 0; i < 3; i++)
                storage[i + 1] = values[permutations[permutation][i]];
            v3Sort(storage + 1, storage + offset);
            CHECK(storage[offset] == (double)INFINITY && storage[offset + 1] == -(double)INFINITY);
            CHECK(isnan(storage[offset + 2]));
        }
    }
    double input[] = { nan("1"), 2, nan("2") }, result[3];
    v3Sort(input, result);
    CHECK(result[0] == 2 && memcmp(result + 1, input, sizeof(double)) == 0);
    CHECK(memcmp(result + 2, input + 2, sizeof(double)) == 0);
    double zeros[] = { -0.0, 0.0, -0.0 };
    v3Sort(zeros, result);
    CHECK(memcmp(result, zeros, sizeof(zeros)) == 0);
    return 0;
}

/** @brief Cover sorted, nearly sorted, reversed, and equal values without timing-dependent assertions. */
static int sortPatterns(void)
{
    enum
    {
        count = 20000
    };
    double* values = malloc(count * sizeof(double));
    CHECK(values != NULL);
    for (size_t i = 0; i < count; i++)
        values[i] = (double)i;
    vSort(values, values, count);
    for (size_t i = 0; i < count; i++)
        CHECK(values[i] == (double)i);
    values[count - 2] = count - 1;
    values[count - 1] = count - 2;
    vSort(values, values, count);
    for (size_t i = 0; i < count; i++)
        CHECK(values[i] == (double)i);
    for (size_t i = 0; i < count; i++)
        values[i] = 1.0;
    vSort(values, values, count);
    for (size_t i = 0; i < count; i++)
        CHECK(values[i] == 1.0);
    for (size_t i = 0; i < 32; i++)
        values[i] = 31.0 - (double)i;
    vSort(values, values, 32);
    for (size_t i = 0; i < 32; i++)
        CHECK(values[i] == (double)i);
    free(values);
    return 0;
}

/** @brief Check all four matrix products against independent multiplication, including overlap. */
static int multiplicationFamilies(void)
{
    const size_t shapes[][3] = { { 1, 1, 1 },   { 2, 3, 4 },    { 3, 8, 12 },   { 12, 8, 3 }, { 8, 8, 8 },
                                 { 16, 12, 8 }, { 64, 64, 64 }, { 2, 2048, 2 }, { 3, 0, 9 } };
    double a[4096], b[4096], expected[4096], storage[12290];
    for (size_t shape = 0; shape < sizeof(shapes) / sizeof(shapes[0]); shape++) {
        size_t rows = shapes[shape][0], inner = shapes[shape][1], columns = shapes[shape][2];
        for (size_t i = 0; i < rows * inner; i++)
            a[i] = (double)(i % 7) - 3.0;
        for (size_t i = 0; i < inner * columns; i++)
            b[i] = (double)(i % 11) - 5.0;
        for (size_t row = 0; row < rows; row++) {
            for (size_t col = 0; col < columns; col++) {
                double value = 0.0;
                for (size_t k = 0; k < inner; k++)
                    value += a[row * inner + k] * b[k * columns + col];
                expected[row * columns + col] = value;
            }
        }
        for (size_t mode = 0; mode < 4; mode++) {
            for (size_t layout = 0; layout < 5; layout++) {
                double* first = storage + 1;
                double* second = storage + 4097;
                const size_t offsets[] = { 8194, 1, 2, 4097, 4096 };
                double* output = storage + offsets[layout];
                for (size_t row = 0; row < rows; row++) {
                    for (size_t k = 0; k < inner; k++) {
                        first[(mode & 1) ? k * rows + row : row * inner + k] = a[row * inner + k];
                    }
                }
                for (size_t k = 0; k < inner; k++) {
                    for (size_t col = 0; col < columns; col++) {
                        second[(mode & 2) ? col * inner + k : k * columns + col] = b[k * columns + col];
                    }
                }
                switch (mode) {
                    case 0:
                        mMultM(first, rows, inner, second, inner, columns, output);
                        break;
                    case 1:
                        mtMultM(first, inner, rows, second, inner, columns, output);
                        break;
                    case 2:
                        mMultMt(first, rows, inner, second, columns, inner, output);
                        break;
                    case 3:
                        mtMultMt(first, inner, rows, second, columns, inner, output);
                        break;
                }
                CHECK(memcmp(output, expected, rows * columns * sizeof(double)) == 0);
            }
        }
    }
    return 0;
}

/** @brief Run a named C regression, or all regressions if no name is supplied. */
int main(int argc, char** argv)
{
    int failures = 0, selected = 0;
    for (size_t i = 0; i < sizeof(arithmeticCases) / sizeof(arithmeticCases[0]); i++) {
        if (argc == 1 || (argc == 2 && strcmp(argv[1], arithmeticCases[i].name) == 0)) {
            int status = arithmeticOverlap(i);
            printf("%s: %s\n", arithmeticCases[i].name, status == 0 ? "PASS" : "FAIL");
            failures += status;
            selected++;
        }
    }
    const struct
    {
        const char* name;
        int (*run)(void);
    } tests[] = {
        { "largeArithmetic", largeArithmetic },
        { "arithmeticDimensions", arithmeticDimensions },
        { "maximumNonfinite", maximumNonfinite },
        { "normalizationPolicy", normalizationPolicy },
        { "sortNonfinite", sortNonfinite },
        { "sortThreeNonfinite", sortThreeNonfinite },
        { "sortPatterns", sortPatterns },

        { "multiplicationFamilies", multiplicationFamilies },
    };
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
