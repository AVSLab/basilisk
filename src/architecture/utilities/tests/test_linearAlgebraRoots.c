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

/* Keep assertions active in optimized builds. */
#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);                                            \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

/** @brief Retain the small Cardano term when R and sqrt(D) nearly cancel. */
static int cubicSmallTerm(void)
{
    const double linearTerms[] = { -1e-4, -1e-6, -1e-8, 1e-8, 1e-6, 1e-4 };
    for (size_t i = 0; i < sizeof(linearTerms) / sizeof(linearTerms[0]); i++) {
        for (int sign = -1; sign <= 1; sign += 2) {
            double coefficients[3] = { (double)sign, linearTerms[i], 0.0 };
            double original[3];
            double result[3];
            memcpy(original, coefficients, sizeof(original));
            feclearexcept(FE_ALL_EXCEPT);
            cubicRoots(coefficients, result);
            CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
            CHECK(memcmp(original, coefficients, sizeof(original)) == 0);
            double root = result[0];
            double residual = (root * root + coefficients[1]) * root + coefficients[0];
            CHECK(fabs(residual) <= 8.0 * DBL_EPSILON);
            CHECK(fabs(root + (double)sign) <= fabs(linearTerms[i]));
            CHECK(result[1] == result[2]);
            CHECK(result[1] == -0.5 * root);
        }
    }
    return 0;
}

/** @brief Avoid cancellation between opposite cube roots when the real root is small. */
static int cubicSmallRealRoot(void)
{
    const double roots[] = { -1e-12, -1e-8, 0.0, 1e-8, 1e-12 };
    const double linearTerms[] = { 0.25, 1.0, 4.0 };
    for (size_t i = 0; i < sizeof(roots) / sizeof(roots[0]); i++) {
        for (size_t j = 0; j < sizeof(linearTerms) / sizeof(linearTerms[0]); j++) {
            double root = roots[i];
            double linear = linearTerms[j];
            double coefficients[3] = { -root * (root * root + linear), linear, 0.0 };
            double result[3];
            feclearexcept(FE_ALL_EXCEPT);
            cubicRoots(coefficients, result);
            CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
            CHECK(fabs(result[0] - root) <= 8.0 * DBL_EPSILON * fabs(root));
            CHECK(fabs(result[1] + 0.5 * root) <= 8.0 * DBL_EPSILON * fabs(root));
            CHECK(result[1] == result[2]);
        }
    }
    return 0;
}

/** @brief Preserve real-root ordering, repeated roots, and exception-free zero cases. */
static int cubicKnownRoots(void)
{
    const struct
    {
        double coefficients[3];
        double expected[3];
    } cases[] = { { { 0, 0, 0 }, { 0, 0, 0 } },     { { -1, 3, -3 }, { 1, 1, 1 } },    { { 8, 12, 6 }, { -2, -2, -2 } },
                  { { -6, 11, -6 }, { 3, 1, 2 } },  { { 0, -4, 0 }, { 2, -2, 0 } },    { { -2, 5, -4 }, { 2, 1, 1 } },
                  { { -8, 17, -10 }, { 8, 1, 1 } }, { { 8, 17, 10 }, { -8, -1, -1 } }, { { 2, -3, 0 }, { -2, 1, 1 } } };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        double coefficients[3];
        double result[3];
        memcpy(coefficients, cases[i].coefficients, sizeof(coefficients));
        feclearexcept(FE_ALL_EXCEPT);
        cubicRoots(coefficients, result);
        CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
        for (size_t j = 0; j < 3; j++) {
            CHECK(fabs(result[j] - cases[i].expected[j]) <= 1e-14);
            double residual =
                ((result[j] + coefficients[2]) * result[j] + coefficients[1]) * result[j] + coefficients[0];
            CHECK(fabs(residual) <= 1e-12);
        }
    }
    return 0;
}

/** @brief Preserve the real root followed by the real parts of a complex-conjugate pair. */
static int cubicComplexConvention(void)
{
    const struct
    {
        double coefficients[3];
        double expected[3];
    } cases[] = {
        { { -1, 0, 0 }, { 1, -0.5, -0.5 } }, { { 1, 0, 0 }, { -1, 0.5, 0.5 } }, { { 0, 1, 0 }, { 0, 0, 0 } },
        { { 4, -2, 0 }, { -2, 1, 1 } },      { { -4, 4, -1 }, { 1, 0, 0 } }, /* (x - 1)(x^2 + 4) */
        { { -2, 4, -3 }, { 1, 1, 1 } }                                       /* (x - 1)((x - 1)^2 + 1) */
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        double coefficients[3];
        double result[3];
        memcpy(coefficients, cases[i].coefficients, sizeof(coefficients));
        cubicRoots(coefficients, result);
        for (size_t j = 0; j < 3; j++) {
            CHECK(fabs(result[j] - cases[i].expected[j]) <= 1e-14);
        }
        CHECK(result[1] == result[2]);
    }
    return 0;
}

/** @brief Preserve unread cubic coefficients for every overlapping output offset. */
static int cubicOverlap(void)
{
    const double cases[][3] = { { -6, 11, -6 }, { -8, 17, -10 }, { 1, 1e-6, 0 }, { 1e-12, 1, 0 } };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        double coefficients[3];
        double reference[3];
        memcpy(coefficients, cases[i], sizeof(coefficients));
        cubicRoots(coefficients, reference);
        for (size_t offset = 0; offset < 8; offset++) {
            double storage[12];
            double expected[12];
            for (size_t j = 0; j < 12; j++) {
                storage[j] = (double)j + 10.0;
            }
            memcpy(storage + 3, coefficients, sizeof(coefficients));
            memcpy(expected, storage, sizeof(expected));
            memcpy(expected + offset, reference, sizeof(reference));
            cubicRoots(storage + 3, storage + offset);
            CHECK(memcmp(storage, expected, sizeof(storage)) == 0);
        }
    }
    return 0;
}

/** @brief Return exact sorted diagonal entries, including repeated and clustered values. */
static int eigenDiagonal(void)
{
    const double spectra[][3] = { { 0.1, 0.1, 0.1 }, { 1.000001, 1, 1 }, { 1.000000000001, 1, 1 }, { 0, 0, 0 },
                                  { 0, -0.1, -0.1 }, { 3, 0, -2 },       { 0.001, 0.001, 0.001 } };
    const size_t permutations[][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
    for (size_t i = 0; i < sizeof(spectra) / sizeof(spectra[0]); i++) {
        for (size_t permutation = 0; permutation < 6; permutation++) {
            double matrix[3][3] = { { 0 } };
            double original[3][3];
            double result[3];
            for (size_t j = 0; j < 3; j++) {
                matrix[j][j] = spectra[i][permutations[permutation][j]];
            }
            memcpy(original, matrix, sizeof(original));
            feclearexcept(FE_ALL_EXCEPT);
            m33EigenValues(matrix, result);
            CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
            CHECK(memcmp(matrix, original, sizeof(matrix)) == 0);
            for (size_t j = 0; j < 3; j++) {
                CHECK(result[j] == spectra[i][j]);
            }
        }
    }
    return 0;
}

/** @brief Exercise equal-diagonal rotations for every off-diagonal pair and both signs. */
static int eigenJacobiPairs(void)
{
    for (size_t first = 0; first < 2; first++) {
        for (size_t second = first + 1; second < 3; second++) {
            for (int sign = -1; sign <= 1; sign += 2) {
                double matrix[3][3] = { { 0 } };
                double result[3];
                matrix[first][first] = matrix[second][second] = 2.0;
                matrix[first][second] = matrix[second][first] = (double)sign;
                feclearexcept(FE_ALL_EXCEPT);
                m33EigenValues(matrix, result);
                CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
                CHECK(fabs(result[0] - 3.0) <= 4.0 * DBL_EPSILON);
                CHECK(fabs(result[1] - 1.0) <= 4.0 * DBL_EPSILON);
                CHECK(result[2] == 0.0);
            }
        }
    }
    return 0;
}

/** @brief Recover known spectra after orthogonal rotations at ordinary matrix scales. */
static int eigenRotatedSpectra(void)
{
    /* Dividing this integer matrix by three gives an orthogonal matrix. */
    const double rotation[3][3] = { { 1, 2, 2 }, { 2, 1, -2 }, { -2, 2, -1 } };
    const double spectra[][3] = { { 0.1, 0.1, 0.1 }, { 1.000001, 1, 1 }, { 1.000000000001, 1, 1 },
                                  { 4, 2, 1 },       { 3, 0, -2 },       { 5, 0, 0 },
                                  { -1, -1, -4 } };
    const double scales[] = { 0.001, 0.1, 1.0, 10.0, 1000.0 };
    for (size_t i = 0; i < sizeof(spectra) / sizeof(spectra[0]); i++) {
        for (size_t j = 0; j < sizeof(scales) / sizeof(scales[0]); j++) {
            double matrix[3][3];
            double original[3][3];
            double result[3];
            double norm = fmax(fabs(spectra[i][0]), fabs(spectra[i][2])) * scales[j];
            for (size_t row = 0; row < 3; row++) {
                for (size_t col = row; col < 3; col++) {
                    double value = 0.0;
                    for (size_t k = 0; k < 3; k++) {
                        value += rotation[row][k] * spectra[i][k] * rotation[col][k];
                    }
                    matrix[row][col] = matrix[col][row] = value * (scales[j] / 9.0);
                }
            }
            memcpy(original, matrix, sizeof(original));
            feclearexcept(FE_ALL_EXCEPT);
            m33EigenValues(matrix, result);
            CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
            CHECK(memcmp(matrix, original, sizeof(matrix)) == 0);
            CHECK(result[0] >= result[1] && result[1] >= result[2]);
            for (size_t k = 0; k < 3; k++) {
                CHECK(fabs(result[k] - spectra[i][k] * scales[j]) <= 16.0 * DBL_EPSILON * norm);
            }
            CHECK(fabs(result[0] + result[1] + result[2] - matrix[0][0] - matrix[1][1] - matrix[2][2]) <=
                  32.0 * DBL_EPSILON * norm);
        }
    }
    return 0;
}

/** @brief Preserve nonsymmetric real spectra and the real-part convention for complex pairs. */
static int eigenNonsymmetric(void)
{
    const struct
    {
        double matrix[3][3];
        double expected[3];
    } cases[] = { { { { 2, 1, 3 }, { 0, 3, 4 }, { 0, 0, 1 } }, { 3, 2, 1 } },
                  { { { 1, -2, 0 }, { 2, 1, 0 }, { 0, 0, 3 } }, { 3, 1, 1 } },
                  { { { 0, 0, -1 }, { 1, 0, 0 }, { 0, 1, 0 } }, { 0.5, 0.5, -1 } } };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        double matrix[3][3];
        double result[3];
        memcpy(matrix, cases[i].matrix, sizeof(matrix));
        feclearexcept(FE_ALL_EXCEPT);
        m33EigenValues(matrix, result);
        CHECK(fetestexcept(FE_INVALID | FE_DIVBYZERO) == 0);
        CHECK(memcmp(matrix, cases[i].matrix, sizeof(matrix)) == 0);
        for (size_t j = 0; j < 3; j++) {
            CHECK(fabs(result[j] - cases[i].expected[j]) <= 1e-14);
        }
    }
    return 0;
}

/** @brief Preserve input reads when eigenvalue output overlaps any part of the matrix. */
static int eigenOverlap(void)
{
    const double matrices[][3][3] = { { { 0.1, 0, 0 }, { 0, 0.1, 0 }, { 0, 0, 0.1 } },
                                      { { 2, 1, 1 }, { 1, 2, 1 }, { 1, 1, 2 } },
                                      { { 1, -2, 0 }, { 2, 1, 0 }, { 0, 0, 3 } } };
    const double spectra[][3] = { { 0.1, 0.1, 0.1 }, { 4, 1, 1 }, { 3, 1, 1 } };
    for (size_t i = 0; i < sizeof(matrices) / sizeof(matrices[0]); i++) {
        for (size_t offset = 0; offset < 16; offset++) {
            double storage[20];
            double original[20];
            for (size_t j = 0; j < 20; j++) {
                storage[j] = (double)j + 10.0;
            }
            memcpy(storage + 4, matrices[i], sizeof(matrices[i]));
            memcpy(original, storage, sizeof(original));
            m33EigenValues((double (*)[3])(storage + 4), storage + offset);
            for (size_t j = 0; j < 20; j++) {
                if (j >= offset && j < offset + 3) {
                    CHECK(fabs(storage[j] - spectra[i][j - offset]) <= 1e-14);
                } else {
                    CHECK(storage[j] == original[j]);
                }
            }
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
    } tests[] = { { "cubicSmallTerm", cubicSmallTerm },       { "cubicSmallRealRoot", cubicSmallRealRoot },
                  { "cubicKnownRoots", cubicKnownRoots },     { "cubicComplexConvention", cubicComplexConvention },
                  { "cubicOverlap", cubicOverlap },           { "eigenDiagonal", eigenDiagonal },
                  { "eigenJacobiPairs", eigenJacobiPairs },   { "eigenRotatedSpectra", eigenRotatedSpectra },
                  { "eigenNonsymmetric", eigenNonsymmetric }, { "eigenOverlap", eigenOverlap } };
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
