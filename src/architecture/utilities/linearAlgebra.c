/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "linearAlgebra.h"
#include "architecture/utilities/bsk_Print.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define MOVE_DOUBLE(source, dim, destination) (memmove((void*)(destination), (void*)(source), sizeof(double) * (dim)))

/* Check before multiplying dimensions, including the byte count used by memory operations. */
static int dimensionsFit(size_t rows, size_t columns, size_t capacity)
{
    if (rows != 0 && columns > capacity / rows) {
        BSK_PRINT(MSG_ERROR, "%s", "Linear Algebra library array dimensions exceed the available capacity.");
        return 0;
    }
    return 1;
}

/* Counts must already have passed dimensionsFit. Subtract addresses to avoid overflowing an end address. */
static int buffersOverlap(const void* first, size_t firstCount, const void* second, size_t secondCount)
{
    if (firstCount == 0 || secondCount == 0) {
        return 0;
    }
    uintptr_t firstAddress = (uintptr_t)first;
    uintptr_t secondAddress = (uintptr_t)second;
    if (firstAddress <= secondAddress) {
        return secondAddress - firstAddress < firstCount * sizeof(double);
    }
    return firstAddress - secondAddress < secondCount * sizeof(double);
}

/* Return a safe traversal direction, or zero when the inputs require opposite directions. */
static int arithmeticDirection(const double* first, const double* second, size_t count, double* result)
{
    int firstOverlaps = buffersOverlap(first, count, result, count);
    int secondOverlaps = buffersOverlap(second, count, result, count);
    int forward = (!firstOverlaps || (uintptr_t)result <= (uintptr_t)first) &&
                  (!secondOverlaps || (uintptr_t)result <= (uintptr_t)second);
    int backward = (!firstOverlaps || (uintptr_t)result >= (uintptr_t)first) &&
                   (!secondOverlaps || (uintptr_t)result >= (uintptr_t)second);
    return forward ? 1 : (backward ? -1 : 0);
}

enum VectorOperation
{
    VECTOR_ADD,
    VECTOR_SUBTRACT,
    VECTOR_MULTIPLY
};

/* Only conflicting overlap needs scratch space. Save the smaller endangered input portion,
 * then traverse in the direction that preserves the other input. Check capacity before writing.
 */
static void conflictingArithmetic(const double* first,
                                  const double* second,
                                  size_t count,
                                  double* result,
                                  enum VectorOperation operation)
{
    const double* lower = (uintptr_t)first < (uintptr_t)result ? first : second;
    const double* upper = lower == first ? second : first;
    size_t lowerOffset = ((uintptr_t)result - (uintptr_t)lower) / sizeof(double);
    size_t upperOffset = ((uintptr_t)upper - (uintptr_t)result) / sizeof(double);
    int forward = lowerOffset >= upperOffset;
    const double* savedInput = forward ? lower : upper;
    size_t savedStart = forward ? lowerOffset : 0;
    size_t savedCount = count - (forward ? lowerOffset : upperOffset);
    if (!dimensionsFit(1, savedCount, LINEAR_ALGEBRA_MAX_ARRAY_SIZE)) {
        return;
    }
    double saved[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    MOVE_DOUBLE(savedInput + savedStart, savedCount, saved);
    for (size_t step = 0; step < count; step++) {
        size_t i = forward ? step : count - 1 - step;
        int useSaved = i >= savedStart && i - savedStart < savedCount;
        double a = first == savedInput && useSaved ? saved[i - savedStart] : first[i];
        double b = second == savedInput && useSaved ? saved[i - savedStart] : second[i];
        switch (operation) {
            case VECTOR_ADD:
                result[i] = a + b;
                break;
            case VECTOR_SUBTRACT:
                result[i] = a - b;
                break;
            case VECTOR_MULTIPLY:
                result[i] = a * b;
                break;
        }
    }
}

/* Both row strides are at least the copied width. Rows moving left are safe in forward
 * order; rows moving right are safe in reverse order, even when the strides differ.
 */
static void copyMatrixBlock(const double* source,
                            size_t sourceStride,
                            double* result,
                            size_t resultStride,
                            size_t rows,
                            size_t columns)
{
    for (size_t row = 0; row < rows; row++) {
        if ((uintptr_t)(result + row * resultStride) <= (uintptr_t)(source + row * sourceStride)) {
            MOVE_DOUBLE(source + row * sourceStride, columns, result + row * resultStride);
        }
    }
    for (size_t row = rows; row-- > 0;) {
        if ((uintptr_t)(result + row * resultStride) > (uintptr_t)(source + row * sourceStride)) {
            MOVE_DOUBLE(source + row * sourceStride, columns, result + row * resultStride);
        }
    }
}

/* NaNs sort last in either direction; equal values retain their input order. */
static int sortOutOfOrder(double first, double second, int descending)
{
    return !isnan(second) && (isnan(first) || (descending ? first < second : first > second));
}

void vElementwiseMult(double* v1, size_t dim, double* v2, double* result)
{
    if (dim == 0 || !dimensionsFit(1, dim, SIZE_MAX / sizeof(double))) {
        return;
    }
    int direction = arithmeticDirection(v1, v2, dim, result);
    if (direction == 0) {
        conflictingArithmetic(v1, v2, dim, result, VECTOR_MULTIPLY);
        return;
    }
    for (size_t step = 0; step < dim; step++) {
        size_t i = direction > 0 ? step : dim - 1 - step;
        result[i] = v1[i] * v2[i];
    }
}

void vCopy(double* v, size_t dim, double* result)
{
    if (!dimensionsFit(1, dim, SIZE_MAX / sizeof(double))) {
        return;
    }
    if (dim > 0) {
        MOVE_DOUBLE(v, dim, result);
    }
}

void vSetZero(double* v, size_t dim)
{
    size_t i;
    for (i = 0; i < dim; i++) {
        v[i] = 0.0;
    }
}

void vSetOnes(double* v, size_t dim)
{
    size_t i;
    for (i = 0; i < dim; i++) {
        v[i] = 1.0;
    }
}

void vAdd(double* v1, size_t dim, double* v2, double* result)
{
    if (dim == 0 || !dimensionsFit(1, dim, SIZE_MAX / sizeof(double))) {
        return;
    }
    int direction = arithmeticDirection(v1, v2, dim, result);
    if (direction == 0) {
        conflictingArithmetic(v1, v2, dim, result, VECTOR_ADD);
        return;
    }
    for (size_t step = 0; step < dim; step++) {
        size_t i = direction > 0 ? step : dim - 1 - step;
        result[i] = v1[i] + v2[i];
    }
}

void vSubtract(double* v1, size_t dim, double* v2, double* result)
{
    if (dim == 0 || !dimensionsFit(1, dim, SIZE_MAX / sizeof(double))) {
        return;
    }
    int direction = arithmeticDirection(v1, v2, dim, result);
    if (direction == 0) {
        conflictingArithmetic(v1, v2, dim, result, VECTOR_SUBTRACT);
        return;
    }
    for (size_t step = 0; step < dim; step++) {
        size_t i = direction > 0 ? step : dim - 1 - step;
        result[i] = v1[i] - v2[i];
    }
}

void vScale(double scaleFactor, double* v, size_t dim, double* result)
{
    if (dim == 0 || !dimensionsFit(1, dim, SIZE_MAX / sizeof(double))) {
        return;
    }
    /* memmove handles either overlap direction without imposing a temporary-buffer limit. */
    if (v != result && buffersOverlap(v, dim, result, dim)) {
        MOVE_DOUBLE(v, dim, result);
        v = result;
    }
    for (size_t i = 0; i < dim; i++) {
        result[i] = v[i] * scaleFactor;
    }
}

double vDot(double* v1, size_t dim, double* v2)
{
    size_t i;
    double result = 0.0;
    for (i = 0; i < dim; i++) {
        result += v1[i] * v2[i];
    }

    return result;
}

void vOuterProduct(double* v1, size_t dim1, double* v2, size_t dim2, void* result)
{
    if (dim1 == 0 || dim2 == 0 || !dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    if (dim1 == 1) {
        vScale(v1[0], v2, dim2, result);
        return;
    }
    if (dim2 == 1) {
        vScale(v2[0], v1, dim1, result);
        return;
    }
    size_t firstCount = buffersOverlap(v1, dim1, result, dim1 * dim2) ? dim1 : 0;
    size_t secondCount = buffersOverlap(v2, dim2, result, dim1 * dim2) ? dim2 : 0;
    if (!dimensionsFit(1, firstCount, LINEAR_ALGEBRA_MAX_ARRAY_SIZE) ||
        !dimensionsFit(1, secondCount, LINEAR_ALGEBRA_MAX_ARRAY_SIZE - firstCount)) {
        return;
    }
    double saved[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (firstCount != 0) {
        MOVE_DOUBLE(v1, firstCount, saved);
        v1 = saved;
    }
    if (secondCount != 0) {
        MOVE_DOUBLE(v2, secondCount, saved + firstCount);
        v2 = saved + firstCount;
    }
    double* m_result = (double*)result;

    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = v1[i] * v2[j];
        }
    }
}

void vtMultM(double* v, void* mx, size_t dim1, size_t dim2, void* result)
{
    if (dim2 == 0 || !dimensionsFit(1, dim2, LINEAR_ALGEBRA_MAX_ARRAY_SIZE) ||
        !dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    size_t dim11 = 1;
    size_t dim12 = dim1;
    size_t dim22 = dim2;
    double* m_mx1 = (double*)v;
    double* m_mx2 = (double*)mx;
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    size_t i;
    size_t j;
    size_t k;
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim22, k, j)];
            }
        }
    }

    MOVE_DOUBLE(m_result, dim11 * dim22, result);
}

void vtMultMt(double* v, void* mx, size_t dim1, size_t dim2, void* result)
{
    if (dim1 == 0 || !dimensionsFit(1, dim1, LINEAR_ALGEBRA_MAX_ARRAY_SIZE) ||
        !dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    size_t dim11 = 1;
    size_t dim12 = dim2;
    size_t dim22 = dim1;
    double* m_mx1 = (double*)v;
    double* m_mx2 = (double*)mx;
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    size_t i;
    size_t j;
    size_t k;
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim12, j, k)];
            }
        }
    }

    MOVE_DOUBLE(m_result, dim11 * dim22, result);
}

double vNorm(double* v, size_t dim)
{
    return sqrt(vDot(v, dim, v));
}

double vMax(double* array, size_t dim)
{
    if (dim == 0) {
        BSK_PRINT(MSG_ERROR, "%s", "Cannot find the maximum of an empty vector.");
        return (double)NAN;
    }
    size_t i;
    double result;

    result = array[0];
    for (i = 0; i < dim; i++) {
        if (isnan(array[i])) {
            return (double)NAN;
        }
        if (array[i] > result) {
            result = array[i];
        }
    }
    return result;
}

double vMaxAbs(double* array, size_t dim)
{
    if (dim == 0) {
        BSK_PRINT(MSG_ERROR, "%s", "Cannot find the maximum absolute value of an empty vector.");
        return (double)NAN;
    }
    size_t i;
    double result;

    result = fabs(array[0]);
    for (i = 0; i < dim; i++) {
        if (isnan(array[i])) {
            return (double)NAN;
        }
        if (fabs(array[i]) > result) {
            result = fabs(array[i]);
        }
    }
    return result;
}

void vNormalize(double* v, size_t dim, double* result)
{
    if (dim == 0 || !dimensionsFit(1, dim, SIZE_MAX / sizeof(double))) {
        return;
    }
    double norm = vNorm(v, dim);

    if (!isfinite(norm)) {
        for (size_t i = 0; i < dim; i++) {
            result[i] = (double)NAN;
        }
    } else if (norm > DB0_EPS) {
        vScale(1.0 / norm, v, dim, result);
    } else {
        vSetZero(result, dim);
    }
}

int vIsEqual(double* v1, size_t dim, double* v2, double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t i;
    for (i = 0; i < dim; i++) {
        /* Require the inclusive bound to hold; NaN differences must not compare equal. */
        if (!(fabs(v1[i] - v2[i]) <= accuracy)) {
            return 0;
        }
    }
    return 1;
}

int vIsZero(double* v, size_t dim, double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t i;
    int result = 1;
    for (i = 0; i < dim; i++) {
        if (!(fabs(v[i]) <= accuracy)) {
            result = 0;
            break;
        }
    }

    return result;
}

void vPrint(FILE* pFile, const char* name, double* v, size_t dim)
{
    size_t i;
    fprintf(pFile, "%s = [", name);
    for (i = 0; i < dim; i++) {
        fprintf(pFile, "%20.15g", v[i]);
        if (i != dim - 1) {
            fprintf(pFile, ", ");
        }
    }
    fprintf(pFile, "];\n");
}

void vSort(double* Input, double* Output, size_t dim)
{
    if (dim == 0 || !dimensionsFit(1, dim, SIZE_MAX / sizeof(double))) {
        return;
    }
    MOVE_DOUBLE(Input, dim, Output);
    /* Entries after the last swap are already sorted; a pass with no swaps finishes early. */
    size_t end = dim;
    while (end > 1) {
        size_t lastSwap = 0;
        for (size_t i = 1; i < end; i++) {
            if (sortOutOfOrder(Output[i - 1], Output[i], 0)) {
                double temporary = Output[i];
                Output[i] = Output[i - 1];
                Output[i - 1] = temporary;
                lastSwap = i;
            }
        }
        end = lastSwap;
    }
}

void v2Set(double v0, double v1, double result[2])
{
    result[0] = v0;
    result[1] = v1;
}

void v2SetZero(double v[2])
{
    size_t dim = 2;
    size_t i;
    for (i = 0; i < dim; i++) {
        v[i] = 0.0;
    }
}

void v2Copy(double v[2], double result[2])
{
    MOVE_DOUBLE(v, 2, result);
}

void v2Scale(double scaleFactor, double v[2], double result[2])
{
    double temporary[2];
    size_t dim = 2;
    size_t i;
    for (i = 0; i < dim; i++) {
        temporary[i] = v[i] * scaleFactor;
    }
    MOVE_DOUBLE(temporary, 2, result);
}

double v2Dot(double v1[2], double v2[2])
{
    size_t dim = 2;
    size_t i;
    double result = 0.0;
    for (i = 0; i < dim; i++) {
        result += v1[i] * v2[i];
    }
    return result;
}

int v2IsEqual(double v1[2], double v2[2], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim = 2;
    size_t i;
    for (i = 0; i < dim; i++) {
        if (!(fabs(v1[i] - v2[i]) <= accuracy)) {
            return 0;
        }
    }
    return 1;
}

int v2IsZero(double v[2], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim = 2;
    size_t i;
    for (i = 0; i < dim; i++) {
        if (!(fabs(v[i]) <= accuracy)) {
            return 0;
        }
    }
    return 1;
}

void v2Add(double v1[2], double v2[2], double result[2])
{
    double temporary[2];
    size_t dim = 2;
    size_t i;
    for (i = 0; i < dim; i++) {
        temporary[i] = v1[i] + v2[i];
    }
    MOVE_DOUBLE(temporary, 2, result);
}

void v2Subtract(double v1[2], double v2[2], double result[2])
{
    double temporary[2];
    size_t dim = 2;
    size_t i;
    for (i = 0; i < dim; i++) {
        temporary[i] = v1[i] - v2[i];
    }
    MOVE_DOUBLE(temporary, 2, result);
}

double v2Norm(double v[2])
{
    return sqrt(v2Dot(v, v));
}

void v2Normalize(double v[2], double result[2])
{
    double norm = v2Norm(v);
    if (!isfinite(norm)) {
        for (size_t i = 0; i < 2; i++) {
            result[i] = (double)NAN;
        }
    } else if (norm > DB0_EPS) {
        v2Scale(1. / norm, v, result);
    } else {
        v2SetZero(result);
    }
}

void v3Set(double v0, double v1, double v2, double result[3])
{
    result[0] = v0;
    result[1] = v1;
    result[2] = v2;
}

void v3Copy(double v[3], double result[3])
{
    MOVE_DOUBLE(v, 3, result);
}

void v3SetZero(double v[3])
{
    size_t dim = 3;
    size_t i;
    for (i = 0; i < dim; i++) {
        v[i] = 0.0;
    }
}

void v3Add(double v1[3], double v2[3], double result[3])
{
    double temporary[3];
    size_t dim = 3;
    size_t i;
    for (i = 0; i < dim; i++) {
        temporary[i] = v1[i] + v2[i];
    }
    MOVE_DOUBLE(temporary, 3, result);
}

void v3Subtract(double v1[3], double v2[3], double result[3])
{
    double temporary[3];
    size_t dim = 3;
    size_t i;
    for (i = 0; i < dim; i++) {
        temporary[i] = v1[i] - v2[i];
    }
    MOVE_DOUBLE(temporary, 3, result);
}

void v3Scale(double scaleFactor, double v[3], double result[3])
{
    double temporary[3];
    size_t dim = 3;
    size_t i;
    for (i = 0; i < dim; i++) {
        temporary[i] = v[i] * scaleFactor;
    }
    MOVE_DOUBLE(temporary, 3, result);
}

double v3Dot(double v1[3], double v2[3])
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void v3OuterProduct(double v1[3], double v2[3], double result[3][3])
{
    /* Preserve both inputs before writing output that may overlap either vector. */
    double v1c[3];
    double v2c[3];
    v3Copy(v1, v1c);
    v3Copy(v2, v2c);

    size_t dim = 3;
    size_t i;
    size_t j;
    for (i = 0; i < dim; i++) {
        for (j = 0; j < dim; j++) {
            result[i][j] = v1c[i] * v2c[j];
        }
    }
}

void v3tMultM33(double v[3], double mx[3][3], double result[3])
{
    size_t dim11 = 1;
    size_t dim12 = 3;
    size_t dim22 = 3;
    size_t i;
    size_t j;
    size_t k;
    double m_result[3];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[j] += v[k] * mx[k][j];
            }
        }
    }
    v3Copy(m_result, result);
}

void v3tMultM33t(double v[3], double mx[3][3], double result[3])
{
    size_t dim11 = 1;
    size_t dim12 = 3;
    size_t dim22 = 3;
    size_t i;
    size_t j;
    size_t k;
    double m_result[3];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[j] += v[k] * mx[j][k];
            }
        }
    }
    v3Copy(m_result, result);
}

double v3Norm(double v[3])
{
    return sqrt(v3Dot(v, v));
}

void v3Normalize(double v[3], double result[3])
{
    double norm = v3Norm(v);
    if (!isfinite(norm)) {
        for (size_t i = 0; i < 3; i++) {
            result[i] = (double)NAN;
        }
    } else if (norm > DB0_EPS) {
        v3Scale(1. / norm, v, result);
    } else {
        v3SetZero(result);
    }
}

int v3IsEqual(double v1[3], double v2[3], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim = 3;
    size_t i;
    for (i = 0; i < dim; i++) {
        if (!(fabs(v1[i] - v2[i]) <= accuracy)) {
            return 0;
        }
    }
    return 1;
}

int v3IsEqualRel(double v1[3], double v2[3], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim = 3;
    size_t i;
    for (i = 0; i < dim; i++) {
        if (!isfinite(v1[i]) || !isfinite(v2[i])) {
            return 0;
        }
    }

    double scale = hypot(hypot(v1[0], v1[1]), v1[2]);
    if (scale == 0.0 || accuracy == 0.0) {
        return v1[0] == v2[0] && v1[1] == v2[1] && v1[2] == v2[2];
    }

    double scaledNorm = 1.0;
    if (!isfinite(scale) || fpclassify(scale) == FP_SUBNORMAL) {
        /* Keep the norm factored to avoid overflow and rounding in the subnormal range. */
        scale = fmax(fmax(fabs(v1[0]), fabs(v1[1])), fabs(v1[2]));
        scaledNorm = hypot(hypot(v1[0] / scale, v1[1] / scale), v1[2] / scale);
    }
    for (i = 0; i < dim; i++) {
        double difference = fabs(v1[i] - v2[i]);
        /* Subtract before scaling to preserve nearby values, unless subtraction overflows. */
        double error = (isfinite(difference) ? difference / scale : fabs(v1[i] / scale - v2[i] / scale)) / scaledNorm;
        if (isinf(error) && isfinite(difference) && scaledNorm > 1.0) {
            /* A tiny scale can overflow the first division even when the final error is finite. */
            error = (difference / scaledNorm) / scale;
        }
        if (!(error <= accuracy)) {
            return 0;
        }
    }
    return 1;
}

int v3IsZero(double v[3], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim = 3;
    size_t i;
    for (i = 0; i < dim; i++) {
        if (!(fabs(v[i]) <= accuracy)) {
            return 0;
        }
    }
    return 1;
}

void v3Print(FILE* pFile, const char* name, double v[3])
{
    size_t dim = 3;
    size_t i;
    fprintf(pFile, "%s = [", name);
    for (i = 0; i < dim; i++) {
        fprintf(pFile, "%20.15g", v[i]);
        if (i != dim - 1) {
            fprintf(pFile, ", ");
        }
    }
    fprintf(pFile, "]\n");
}

void v3Cross(double v1[3], double v2[3], double result[3])
{
    double v1c[3];
    double v2c[3];
    v3Copy(v1, v1c);
    v3Copy(v2, v2c);
    result[0] = v1c[1] * v2c[2] - v1c[2] * v2c[1];
    result[1] = v1c[2] * v2c[0] - v1c[0] * v2c[2];
    result[2] = v1c[0] * v2c[1] - v1c[1] * v2c[0];
}

void v3Perpendicular(double v[3], double result[3])
{
    /* Preserve the zero-vector result without evaluating 0/0. */
    if (v[0] == 0.0 && v[1] == 0.0 && v[2] == 0.0) {
        v3SetZero(result);
        return;
    }

    /* Preserve unread input components when result aliases v. */
    double perpendicular[3];
    if (fabs(v[0]) > DB0_EPS) {
        perpendicular[0] = -(v[1] + v[2]) / v[0];
        perpendicular[1] = 1;
        perpendicular[2] = 1;
    } else if (fabs(v[1]) > DB0_EPS) {
        perpendicular[0] = 1;
        perpendicular[1] = -(v[0] + v[2]) / v[1];
        perpendicular[2] = 1;
    } else {
        perpendicular[0] = 1;
        perpendicular[1] = 1;
        perpendicular[2] = -(v[0] + v[1]) / v[2];
    }
    v3Normalize(perpendicular, result);
}

void v3Tilde(double v[3], double result[3][3])
{
    /* Preserve the input before writing output that may overlap it. */
    const double v0 = v[0];
    const double v1 = v[1];
    const double v2 = v[2];

    result[0][0] = 0.0;
    result[0][1] = -v2;
    result[0][2] = v1;
    result[1][0] = v2;
    result[1][1] = 0.0;
    result[1][2] = -v0;
    result[2][0] = -v1;
    result[2][1] = v0;
    result[2][2] = 0.0;
}

void v3Sort(double v[3], double result[3])
{
    double temp;
    v3Copy(v, result);
    if (sortOutOfOrder(result[0], result[1], 1)) {
        temp = result[0];
        result[0] = result[1];
        result[1] = temp;
    }
    if (sortOutOfOrder(result[1], result[2], 1)) {
        if (sortOutOfOrder(result[0], result[2], 1)) {
            temp = result[2];
            result[2] = result[1];
            result[1] = result[0];
            result[0] = temp;
        } else {
            temp = result[1];
            result[1] = result[2];
            result[2] = temp;
        }
    }
}

void v3PrintScreen(const char* name, double vec[3])
{
    printf("%s (%20.15g, %20.15g, %20.15g)\n", name, vec[0], vec[1], vec[2]);
}

void v4Set(double v0, double v1, double v2, double v3, double result[4])
{
    result[0] = v0;
    result[1] = v1;
    result[2] = v2;
    result[3] = v3;
}

void v4Copy(double v[4], double result[4])
{
    MOVE_DOUBLE(v, 4, result);
}

void v4SetZero(double v[4])
{
    size_t dim = 4;
    size_t i;
    for (i = 0; i < dim; i++) {
        v[i] = 0.0;
    }
}

double v4Dot(double v1[4], double v2[4])
{
    size_t dim = 4;
    size_t i;
    double result = 0.0;
    for (i = 0; i < dim; i++) {
        result += v1[i] * v2[i];
    }
    return result;
}

double v4Norm(double v[4])
{
    return sqrt(v4Dot(v, v));
}

int v4IsEqual(double v1[4], double v2[4], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim = 4;
    size_t i;
    for (i = 0; i < dim; i++) {
        if (!(fabs(v1[i] - v2[i]) <= accuracy)) {
            return 0;
        }
    }
    return 1;
}

int v4IsZero(double v[4], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim = 4;
    size_t i;
    for (i = 0; i < dim; i++) {
        if (!(fabs(v[i]) <= accuracy)) {
            return 0;
        }
    }
    return 1;
}

void v6Set(double v0, double v1, double v2, double v3, double v4, double v5, double result[6])
{
    result[0] = v0;
    result[1] = v1;
    result[2] = v2;
    result[3] = v3;
    result[4] = v4;
    result[5] = v5;
}

void v6Copy(double v[6], double result[6])
{
    MOVE_DOUBLE(v, 6, result);
}

double v6Dot(double v1[6], double v2[6])
{
    size_t dim = 6;
    size_t i;
    double result = 0.0;
    for (i = 0; i < dim; i++) {
        result += v1[i] * v2[i];
    }
    return result;
}

void v6Scale(double scaleFactor, double v[6], double result[6])
{
    double temporary[6];
    size_t dim = 6;
    size_t i;
    for (i = 0; i < dim; i++) {
        temporary[i] = v[i] * scaleFactor;
    }
    MOVE_DOUBLE(temporary, 6, result);
}

void v6OuterProduct(double v1[6], double v2[6], double result[6][6])
{
    /* Preserve both inputs before writing output that may overlap either vector. */
    double v1c[6];
    double v2c[6];
    v6Copy(v1, v1c);
    v6Copy(v2, v2c);

    size_t dim = 6;
    size_t i;
    size_t j;
    for (i = 0; i < dim; i++) {
        for (j = 0; j < dim; j++) {
            result[i][j] = v1c[i] * v2c[j];
        }
    }
}

int v6IsEqual(double v1[6], double v2[6], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim = 6;
    size_t i;
    for (i = 0; i < dim; i++) {
        if (!(fabs(v1[i] - v2[i]) <= accuracy)) {
            return 0;
        }
    }
    return 1;
}

void mLeastSquaresInverse(void* mx, size_t dim1, size_t dim2, void* result)
{
    /*
     * Computes the least squares inverse.
     */
    double* m_result = (double*)result;
    double mxTranspose[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    double mxGrammian[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    double mxGrammianInverse[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    mTranspose(mx, dim1, dim2, mxTranspose);
    mMultM(mxTranspose, dim2, dim1, mx, dim1, dim2, mxGrammian);
    mInverse(mxGrammian, dim2, mxGrammianInverse);
    mMultM(mxGrammianInverse, dim2, dim2, mxTranspose, dim2, dim1, m_result);
}

void mMinimumNormInverse(void* mx, size_t dim1, size_t dim2, void* result)
{
    /*
     * Computes the minumum norm inverse.
     */
    double* m_mx = (double*)mx;
    double* m_result = (double*)result;
    double mxTranspose[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    double mxMxTranspose[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    double mxMxTransposeInverse[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    mTranspose(m_mx, dim1, dim2, mxTranspose);
    mMultM(m_mx, dim1, dim2, mxTranspose, dim2, dim1, mxMxTranspose);
    mInverse(mxMxTranspose, dim1, mxMxTransposeInverse);
    mMultM(mxTranspose, dim2, dim1, mxMxTransposeInverse, dim1, dim1, m_result);
}

void mCopy(void* mx, size_t dim1, size_t dim2, void* result)
{
    if (!dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    if (dim1 > 0 && dim2 > 0) {
        MOVE_DOUBLE(mx, dim1 * dim2, result);
    }
}

void mSetZero(void* result, size_t dim1, size_t dim2)
{
    double* m_result = (double*)result;

    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = 0.0;
        }
    }
}

void mSetIdentity(void* result, size_t dim1, size_t dim2)
{
    double* m_result = (double*)result;

    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim2, i, j)] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void mDiag(void* v, size_t dim, void* result)
{
    if (!dimensionsFit(dim, dim, SIZE_MAX / sizeof(double))) {
        return;
    }
    double* m_v = (double*)v;
    double* m_result = (double*)result;

    /* Preserve the input in the first output row. Fill rows backwards to avoid overwriting unread values. */
    vCopy(m_v, dim, m_result);

    size_t i;
    size_t j;
    for (i = dim; i > 0; i--) {
        size_t row = i - 1;
        double diagonal = m_result[row];
        for (j = 0; j < dim; j++) {
            m_result[MXINDEX(dim, row, j)] = (row == j) ? diagonal : 0.0;
        }
    }
}

void mTranspose(void* mx, size_t dim1, size_t dim2, void* result)
{
    if (dim1 == 0 || dim2 == 0 || !dimensionsFit(dim1, dim2, LINEAR_ALGEBRA_MAX_ARRAY_SIZE)) {
        return;
    }
    double* m_mx = (double*)mx;
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            m_result[MXINDEX(dim1, j, i)] = m_mx[MXINDEX(dim2, i, j)];
        }
    }

    MOVE_DOUBLE(m_result, dim2 * dim1, result);
}

void mAdd(void* mx1, size_t dim1, size_t dim2, void* mx2, void* result)
{
    if (dim1 == 0 || dim2 == 0 || !dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    vAdd(mx1, dim1 * dim2, mx2, result);
}

void mSubtract(void* mx1, size_t dim1, size_t dim2, void* mx2, void* result)
{
    if (dim1 == 0 || dim2 == 0 || !dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    vSubtract(mx1, dim1 * dim2, mx2, result);
}

void mScale(double scaleFactor, void* mx, size_t dim1, size_t dim2, void* result)
{
    if (dim1 == 0 || dim2 == 0 || !dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    vScale(scaleFactor, mx, dim1 * dim2, result);
}

void mMultM(void* mx1, size_t dim11, size_t dim12, void* mx2, size_t dim21, size_t dim22, void* result)
{
    if (dim11 == 0 || dim22 == 0 || !dimensionsFit(dim11, dim22, LINEAR_ALGEBRA_MAX_ARRAY_SIZE) ||
        !dimensionsFit(dim11, dim12, SIZE_MAX / sizeof(double)) ||
        !dimensionsFit(dim21, dim22, SIZE_MAX / sizeof(double))) {
        return;
    }
    double* m_mx1 = (double*)mx1;
    double* m_mx2 = (double*)mx2;
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    size_t i;
    size_t j;
    size_t k;
    if (dim12 != dim21) {
        BSK_PRINT(MSG_ERROR, "%s", "Error: mMultM dimensions don't match.");
        return;
    }
    /* Contiguous row updates help wider products; retain the small-matrix dot-product loop. */
    if (dim12 >= 8 && dim22 >= 8) {
        for (i = 0; i < dim11; i++) {
            for (j = 0; j < dim22; j++) {
                m_result[MXINDEX(dim22, i, j)] = 0.0;
            }
            for (k = 0; k < dim12; k++) {
                double value = m_mx1[MXINDEX(dim12, i, k)];
                for (j = 0; j < dim22; j++) {
                    m_result[MXINDEX(dim22, i, j)] += value * m_mx2[MXINDEX(dim22, k, j)];
                }
            }
        }
    } else {
        for (i = 0; i < dim11; i++) {
            for (j = 0; j < dim22; j++) {
                m_result[MXINDEX(dim22, i, j)] = 0.0;
                for (k = 0; k < dim12; k++) {
                    m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim22, k, j)];
                }
            }
        }
    }

    MOVE_DOUBLE(m_result, dim11 * dim22, result);
}

void mtMultM(void* mx1, size_t dim11, size_t dim12, void* mx2, size_t dim21, size_t dim22, void* result)
{
    if (dim12 == 0 || dim22 == 0 || !dimensionsFit(dim12, dim22, LINEAR_ALGEBRA_MAX_ARRAY_SIZE) ||
        !dimensionsFit(dim11, dim12, SIZE_MAX / sizeof(double)) ||
        !dimensionsFit(dim21, dim22, SIZE_MAX / sizeof(double))) {
        return;
    }
    double* m_mx1 = (double*)mx1;
    double* m_mx2 = (double*)mx2;
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    size_t i;
    size_t j;
    size_t k;
    if (dim11 != dim21) {
        BSK_PRINT(MSG_ERROR, "%s", "Error: mtMultM dimensions don't match.");
        return;
    }
    /* Contiguous row updates help wider products; retain the small-matrix dot-product loop. */
    if (dim11 >= 8 && dim22 >= 8) {
        for (i = 0; i < dim12; i++) {
            for (j = 0; j < dim22; j++) {
                m_result[MXINDEX(dim22, i, j)] = 0.0;
            }
            for (k = 0; k < dim11; k++) {
                double value = m_mx1[MXINDEX(dim12, k, i)];
                for (j = 0; j < dim22; j++) {
                    m_result[MXINDEX(dim22, i, j)] += value * m_mx2[MXINDEX(dim22, k, j)];
                }
            }
        }
    } else {
        for (i = 0; i < dim12; i++) {
            for (j = 0; j < dim22; j++) {
                m_result[MXINDEX(dim22, i, j)] = 0.0;
                for (k = 0; k < dim11; k++) {
                    m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, k, i)] * m_mx2[MXINDEX(dim22, k, j)];
                }
            }
        }
    }

    MOVE_DOUBLE(m_result, dim12 * dim22, result);
}

void mMultMt(void* mx1, size_t dim11, size_t dim12, void* mx2, size_t dim21, size_t dim22, void* result)
{
    if (dim11 == 0 || dim21 == 0 || !dimensionsFit(dim11, dim21, LINEAR_ALGEBRA_MAX_ARRAY_SIZE) ||
        !dimensionsFit(dim11, dim12, SIZE_MAX / sizeof(double)) ||
        !dimensionsFit(dim21, dim22, SIZE_MAX / sizeof(double))) {
        return;
    }
    double* m_mx1 = (double*)mx1;
    double* m_mx2 = (double*)mx2;
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    size_t i;
    size_t j;
    size_t k;
    if (dim12 != dim22) {
        BSK_PRINT(MSG_ERROR, "%s", "Error: mMultMt dimensions don't match.");
        return;
    }
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim21; j++) {
            m_result[MXINDEX(dim21, i, j)] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[MXINDEX(dim21, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim22, j, k)];
            }
        }
    }

    MOVE_DOUBLE(m_result, dim11 * dim21, result);
}

void mtMultMt(void* mx1, size_t dim11, size_t dim12, void* mx2, size_t dim21, size_t dim22, void* result)
{
    if (dim12 == 0 || dim21 == 0 || !dimensionsFit(dim12, dim21, LINEAR_ALGEBRA_MAX_ARRAY_SIZE) ||
        !dimensionsFit(dim11, dim12, SIZE_MAX / sizeof(double)) ||
        !dimensionsFit(dim21, dim22, SIZE_MAX / sizeof(double))) {
        return;
    }
    double* m_mx1 = (double*)mx1;
    double* m_mx2 = (double*)mx2;
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    size_t i;
    size_t j;
    size_t k;
    if (dim11 != dim22) {
        BSK_PRINT(MSG_ERROR, "%s", "Error: mtMultMt dimensions don't match.");
        return;
    }
    for (i = 0; i < dim12; i++) {
        for (j = 0; j < dim21; j++) {
            m_result[MXINDEX(dim21, i, j)] = 0.0;
            for (k = 0; k < dim11; k++) {
                m_result[MXINDEX(dim21, i, j)] += m_mx1[MXINDEX(dim12, k, i)] * m_mx2[MXINDEX(dim22, j, k)];
            }
        }
    }

    MOVE_DOUBLE(m_result, dim12 * dim21, result);
}

void mMultV(void* mx, size_t dim1, size_t dim2, void* v, void* result)
{
    if (dim1 == 0 || !dimensionsFit(1, dim1, LINEAR_ALGEBRA_MAX_ARRAY_SIZE) ||
        !dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    size_t dim11 = dim1;
    size_t dim12 = dim2;
    size_t dim22 = 1;
    double* m_mx1 = (double*)mx;
    double* m_mx2 = (double*)v;
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    size_t i;
    size_t j;
    size_t k;
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, i, k)] * m_mx2[MXINDEX(dim22, k, j)];
            }
        }
    }

    MOVE_DOUBLE(m_result, dim11 * dim22, result);
}

void mtMultV(void* mx, size_t dim1, size_t dim2, void* v, void* result)
{
    if (dim2 == 0 || !dimensionsFit(1, dim2, LINEAR_ALGEBRA_MAX_ARRAY_SIZE) ||
        !dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    size_t dim11 = dim1;
    size_t dim12 = dim2;
    size_t dim22 = 1;
    double* m_mx1 = (double*)mx;
    double* m_mx2 = (double*)v;
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];

    size_t i;
    size_t j;
    size_t k;
    for (i = 0; i < dim12; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[MXINDEX(dim22, i, j)] = 0.0;
            for (k = 0; k < dim11; k++) {
                m_result[MXINDEX(dim22, i, j)] += m_mx1[MXINDEX(dim12, k, i)] * m_mx2[MXINDEX(dim22, k, j)];
            }
        }
    }

    MOVE_DOUBLE(m_result, dim12 * dim22, result);
}

double mTrace(void* mx, size_t dim)
{
    double* m_mx = (double*)mx;

    size_t i;
    double result = 0.0;
    for (i = 0; i < dim; i++) {
        result += m_mx[MXINDEX(dim, i, i)];
    }

    return result;
}

double mDeterminant(void* mx, size_t dim)
{
    double* m_mx = (double*)mx;

    size_t i;
    size_t j;
    size_t k;
    size_t ii;
    double result = 0;
    double mxTemp[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if ((dim - 1) * (dim - 1) > LINEAR_ALGEBRA_MAX_ARRAY_SIZE) {
        BSK_PRINT(MSG_ERROR, "%s", "Linear Algebra library array dimension input is too large.");
    }

    if (dim < 1) {
        return 0;
    } else if (dim == 1) {
        result = m_mx[MXINDEX(dim, 0, 0)];
    } else if (dim == 2) {
        result =
            m_mx[MXINDEX(dim, 0, 0)] * m_mx[MXINDEX(dim, 1, 1)] - m_mx[MXINDEX(dim, 1, 0)] * m_mx[MXINDEX(dim, 0, 1)];
    } else {
        for (k = 0; k < dim; k++) {
            for (i = 1; i < dim; i++) {
                ii = 0;
                for (j = 0; j < dim; j++) {
                    if (j == k) {
                        continue;
                    }
                    mxTemp[MXINDEX(dim - 1, i - 1, ii)] = m_mx[MXINDEX(dim, i, j)];
                    ii++;
                }
            }
            double cofactorSign = k % 2 == 0 ? 1.0 : -1.0;
            result += cofactorSign * m_mx[MXINDEX(dim, 0, k)] * mDeterminant(mxTemp, dim - 1);
        }
    }
    return (result);
}

void mCofactor(void* mx, size_t dim, void* result)
{
    /* The (j,i)th cofactor of A is defined as (-1)^(i + j)*det(A_(i,j))
       where A_(i,j) is the submatrix of A obtained from A by removing the ith row and jth column */
    size_t i;
    size_t i0;
    size_t i1;
    size_t j;
    size_t j0;
    size_t j1;
    double* m_mx = (double*)mx;
    double m_mxij[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    double det;
    if (dim * dim > LINEAR_ALGEBRA_MAX_ARRAY_SIZE) {
        BSK_PRINT(MSG_ERROR, "%s", "Linear Algebra library array dimension input is too large.");
    }

    for (i = 0; i < dim; i++) {
        for (j = 0; j < dim; j++) {
            /* Form mx_(i,j) */
            i1 = 0;
            for (i0 = 0; i0 < dim; i0++) {
                if (i0 == i) {
                    continue;
                }
                j1 = 0;
                for (j0 = 0; j0 < dim; j0++) {
                    if (j0 == j) {
                        continue;
                    }
                    m_mxij[MXINDEX(dim - 1, i1, j1)] = m_mx[MXINDEX(dim, i0, j0)];
                    j1++;
                }
                i1++;
            }

            /* Calculate the determinant */
            det = mDeterminant(m_mxij, dim - 1);

            /* Fill in the elements of the cofactor */
            double cofactorSign = (i + j) % 2 == 0 ? 1.0 : -1.0;
            m_result[MXINDEX(dim, i, j)] = cofactorSign * det;
        }
    }

    MOVE_DOUBLE(m_result, dim * dim, result);
}

int mInverse(void* mx, size_t dim, void* result)
{
    /* Inverse of a square matrix A with non zero determinant is adjoint matrix divided by determinant */
    /* The adjoint matrix is the square matrix X such that the (i,j)th entry of X is the (j,i)th cofactor of A */

    size_t i;
    size_t j;
    int status = 0;
    double det = mDeterminant(mx, dim);
    double m_result[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
    if (dim * dim > LINEAR_ALGEBRA_MAX_ARRAY_SIZE) {
        BSK_PRINT(MSG_ERROR, "%s", "Linear Algebra library array dimension input is too large.");
    }

    if (fabs(det) > DB0_EPS) {
        /* Find adjoint matrix */
        double m_adjoint[LINEAR_ALGEBRA_MAX_ARRAY_SIZE];
        mCofactor(mx, dim, m_adjoint);
        mTranspose(m_adjoint, dim, dim, m_adjoint);
        /* Find inverse */
        mScale(1.0 / det, m_adjoint, dim, dim, m_result);
    } else {
        BSK_PRINT(MSG_ERROR, "%s", "Error: cannot invert singular matrix");
        for (i = 0; i < dim; i++) {
            for (j = 0; j < dim; j++) {
                m_result[MXINDEX(dim, i, j)] = 0.0;
            }
        }
        status = 1;
    }

    MOVE_DOUBLE(m_result, dim * dim, result);
    return status;
}

int mIsEqual(void* mx1, size_t dim1, size_t dim2, void* mx2, double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    double* m_mx1 = (double*)mx1;
    double* m_mx2 = (double*)mx2;

    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            if (!(fabs(m_mx1[MXINDEX(dim2, i, j)] - m_mx2[MXINDEX(dim2, i, j)]) <= accuracy)) {
                return 0;
            }
        }
    }
    return 1;
}

int mIsZero(void* mx, size_t dim1, size_t dim2, double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    double* m_mx = (double*)mx;

    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            if (!(fabs(m_mx[MXINDEX(dim2, i, j)]) <= accuracy)) {
                return 0;
            }
        }
    }
    return 1;
}

void mPrintScreen(const char* name, void* mx, size_t dim1, size_t dim2)
{
    double* m_mx = (double*)mx;

    size_t i;
    size_t j;
    printf("%s = [", name);
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            printf("%20.15g", m_mx[MXINDEX(dim2, i, j)]);
            if (j != dim2 - 1) {
                printf(", ");
            }
        }
        if (i != dim1 - 1) {
            printf(";\n");
        }
    }
    printf("];\n");
}

void mPrint(FILE* pFile, const char* name, void* mx, size_t dim1, size_t dim2)
{
    double* m_mx = (double*)mx;

    size_t i;
    size_t j;
    fprintf(pFile, "%s = [", name);
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            fprintf(pFile, "%20.15g", m_mx[MXINDEX(dim2, i, j)]);
            if (j != dim2 - 1) {
                fprintf(pFile, ", ");
            }
        }
        if (i != dim1 - 1) {
            fprintf(pFile, ";\n");
        }
    }
    fprintf(pFile, "];\n");
}

void mGetSubMatrix(void* mx,
                   size_t dim1,
                   size_t dim2,
                   size_t dim1Start,
                   size_t dim2Start,
                   size_t dim1Result,
                   size_t dim2Result,
                   void* result)
{
    if (dim1Result == 0 || dim2Result == 0) {
        return;
    }
    if (dim1Start > dim1 || dim2Start > dim2 || dim1Result > dim1 - dim1Start || dim2Result > dim2 - dim2Start) {
        BSK_PRINT(MSG_ERROR, "%s", "Requested submatrix lies outside the input matrix.");
        return;
    }
    if (!dimensionsFit(dim1, dim2, SIZE_MAX / sizeof(double))) {
        return;
    }
    const double* source = (double*)mx + MXINDEX(dim2, dim1Start, dim2Start);
    copyMatrixBlock(source, dim2, result, dim2Result, dim1Result, dim2Result);
}

void mSetSubMatrix(void* mx,
                   size_t dim1,
                   size_t dim2,
                   void* result,
                   size_t dim1Result,
                   size_t dim2Result,
                   size_t dim1Start,
                   size_t dim2Start)
{
    if (dim1 == 0 || dim2 == 0) {
        return;
    }
    if (dim1Start > dim1Result || dim2Start > dim2Result || dim1 > dim1Result - dim1Start ||
        dim2 > dim2Result - dim2Start) {
        BSK_PRINT(MSG_ERROR, "%s", "Requested submatrix lies outside the output matrix.");
        return;
    }
    if (!dimensionsFit(dim1Result, dim2Result, SIZE_MAX / sizeof(double))) {
        return;
    }
    double* destination = (double*)result + MXINDEX(dim2Result, dim1Start, dim2Start);
    copyMatrixBlock(mx, dim2, destination, dim2Result, dim1, dim2);
}

void m22Set(double m00, double m01, double m10, double m11, double m[2][2])
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[1][0] = m10;
    m[1][1] = m11;
}

void m22Copy(double mx[2][2], double result[2][2])
{
    MOVE_DOUBLE(mx, 2 * 2, result);
}

void m22SetZero(double result[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void m22SetIdentity(double result[2][2])
{
    size_t dim = 2;
    size_t i;
    size_t j;
    for (i = 0; i < dim; i++) {
        for (j = 0; j < dim; j++) {
            result[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void m22Transpose(double mx[2][2], double result[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    double m_result[2][2];
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            m_result[j][i] = mx[i][j];
        }
    }
    m22Copy(m_result, result);
}

void m22Add(double mx1[2][2], double mx2[2][2], double result[2][2])
{
    double temporary[2][2];
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            temporary[i][j] = mx1[i][j] + mx2[i][j];
        }
    }
    MOVE_DOUBLE(temporary, 4, result);
}

void m22Subtract(double mx1[2][2], double mx2[2][2], double result[2][2])
{
    double temporary[2][2];
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            temporary[i][j] = mx1[i][j] - mx2[i][j];
        }
    }
    MOVE_DOUBLE(temporary, 4, result);
}

void m22Scale(double scaleFactor, double mx[2][2], double result[2][2])
{
    double temporary[2][2];
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            temporary[i][j] = scaleFactor * mx[i][j];
        }
    }
    MOVE_DOUBLE(temporary, 4, result);
}

void m22MultM22(double mx1[2][2], double mx2[2][2], double result[2][2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim22 = 2;
    size_t i;
    size_t j;
    size_t k;
    double m_result[2][2];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[k][j];
            }
        }
    }
    m22Copy(m_result, result);
}

void m22tMultM22(double mx1[2][2], double mx2[2][2], double result[2][2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim22 = 2;
    size_t i;
    size_t j;
    size_t k;
    double m_result[2][2];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[k][i] * mx2[k][j];
            }
        }
    }
    m22Copy(m_result, result);
}

void m22MultM22t(double mx1[2][2], double mx2[2][2], double result[2][2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim21 = 2;
    size_t i;
    size_t j;
    size_t k;
    double m_result[2][2];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim21; j++) {
            m_result[i][j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[j][k];
            }
        }
    }
    m22Copy(m_result, result);
}

void m22MultV2(double mx[2][2], double v[2], double result[2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    double m_result[2];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i] += mx[i][k] * v[k];
            }
        }
    }
    v2Copy(m_result, result);
}

void m22tMultV2(double mx[2][2], double v[2], double result[2])
{
    size_t dim11 = 2;
    size_t dim12 = 2;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    double m_result[2];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i] += mx[k][i] * v[k];
            }
        }
    }
    v2Copy(m_result, result);
}

double m22Trace(double mx[2][2])
{
    size_t dim = 2;
    size_t i;
    double result = 0.0;
    for (i = 0; i < dim; i++) {
        result += mx[i][i];
    }

    return result;
}

double m22Determinant(double mx[2][2])
{
    double value;
    value = mx[0][0] * mx[1][1] - mx[1][0] * mx[0][1];
    return value;
}

int m22IsEqual(double mx1[2][2], double mx2[2][2], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            if (!(fabs(mx1[i][j] - mx2[i][j]) <= accuracy)) {
                return 0;
            }
        }
    }
    return 1;
}

int m22IsZero(double mx[2][2], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            if (!(fabs(mx[i][j]) <= accuracy)) {
                return 0;
            }
        }
    }
    return 1;
}

void m22Print(FILE* pFile, const char* name, double mx[2][2])
{
    size_t dim1 = 2;
    size_t dim2 = 2;
    size_t i;
    size_t j;
    fprintf(pFile, "%s = [", name);
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            fprintf(pFile, "%20.15g", mx[i][j]);
            if (j != dim2 - 1) {
                fprintf(pFile, ", ");
            }
        }
        if (i != dim1 - 1) {
            fprintf(pFile, ";\n");
        }
    }
    fprintf(pFile, "]\n");
}

int m22Inverse(double mx[2][2], double result[2][2])
{
    double det = m22Determinant(mx);
    double detInv;
    double m_result[2][2];
    int status = 0;

    if (fabs(det) > DB0_EPS) {
        detInv = 1.0 / det;
        m_result[0][0] = mx[1][1] * detInv;
        m_result[0][1] = -mx[0][1] * detInv;
        m_result[1][0] = -mx[1][0] * detInv;
        m_result[1][1] = mx[0][0] * detInv;
    } else {
        BSK_PRINT(MSG_ERROR, "%s", "Error: singular 2x2 matrix inverse");
        m22Set(0.0, 0.0, 0.0, 0.0, m_result);
        status = 1;
    }
    m22Copy(m_result, result);
    return status;
}

void m22PrintScreen(const char* name, double mx[2][2])
{
    int i;
    printf("%s:\n", name);
    for (i = 0; i < 2; i++) {
        printf("%20.15g, %20.15g\n", mx[i][0], mx[i][1]);
    }
}

void m33Set(double m00,
            double m01,
            double m02,
            double m10,
            double m11,
            double m12,
            double m20,
            double m21,
            double m22,
            double m[3][3])
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[0][2] = m02;
    m[1][0] = m10;
    m[1][1] = m11;
    m[1][2] = m12;
    m[2][0] = m20;
    m[2][1] = m21;
    m[2][2] = m22;
}

void m33Copy(double mx[3][3], double result[3][3])
{
    MOVE_DOUBLE(mx, 3 * 3, result);
}

void m33SetZero(double result[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void m33SetIdentity(double result[3][3])
{
    size_t dim = 3;
    size_t i;
    size_t j;
    for (i = 0; i < dim; i++) {
        for (j = 0; j < dim; j++) {
            result[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void m33Transpose(double mx[3][3], double result[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    double m_result[3][3];
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            m_result[j][i] = mx[i][j];
        }
    }
    m33Copy(m_result, result);
}

void m33Add(double mx1[3][3], double mx2[3][3], double result[3][3])
{
    double temporary[3][3];
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            temporary[i][j] = mx1[i][j] + mx2[i][j];
        }
    }
    MOVE_DOUBLE(temporary, 9, result);
}

void m33Subtract(double mx1[3][3], double mx2[3][3], double result[3][3])
{
    double temporary[3][3];
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            temporary[i][j] = mx1[i][j] - mx2[i][j];
        }
    }
    MOVE_DOUBLE(temporary, 9, result);
}

void m33Scale(double scaleFactor, double mx[3][3], double result[3][3])
{
    double temporary[3][3];
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            temporary[i][j] = scaleFactor * mx[i][j];
        }
    }
    MOVE_DOUBLE(temporary, 9, result);
}

void m33MultM33(double mx1[3][3], double mx2[3][3], double result[3][3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim22 = 3;
    size_t i;
    size_t j;
    size_t k;
    double m_result[3][3];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[k][j];
            }
        }
    }
    m33Copy(m_result, result);
}

void m33tMultM33(double mx1[3][3], double mx2[3][3], double result[3][3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim22 = 3;
    size_t i;
    size_t j;
    size_t k;
    double m_result[3][3];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[k][i] * mx2[k][j];
            }
        }
    }
    m33Copy(m_result, result);
}

void m33MultM33t(double mx1[3][3], double mx2[3][3], double result[3][3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim21 = 3;
    size_t i;
    size_t j;
    size_t k;
    double m_result[3][3];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim21; j++) {
            m_result[i][j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[j][k];
            }
        }
    }
    m33Copy(m_result, result);
}

void m33MultV3(double mx[3][3], double v[3], double result[3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    double m_result[3];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i] += mx[i][k] * v[k];
            }
        }
    }
    v3Copy(m_result, result);
}

void m33tMultV3(double mx[3][3], double v[3], double result[3])
{
    size_t dim11 = 3;
    size_t dim12 = 3;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    double m_result[3];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i] += mx[k][i] * v[k];
            }
        }
    }
    v3Copy(m_result, result);
}

double m33Trace(double mx[3][3])
{
    size_t dim = 3;
    size_t i;
    double result = 0.0;
    for (i = 0; i < dim; i++) {
        result += mx[i][i];
    }

    return result;
}

double m33Determinant(double mx[3][3])
{
    double value;
    value = mx[0][0] * mx[1][1] * mx[2][2] + mx[0][1] * mx[1][2] * mx[2][0] + mx[0][2] * mx[1][0] * mx[2][1] -
            mx[0][0] * mx[1][2] * mx[2][1] - mx[0][1] * mx[1][0] * mx[2][2] - mx[0][2] * mx[1][1] * mx[2][0];
    return value;
}

int m33IsEqual(double mx1[3][3], double mx2[3][3], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            if (!(fabs(mx1[i][j] - mx2[i][j]) <= accuracy)) {
                return 0;
            }
        }
    }
    return 1;
}

int m33IsZero(double mx[3][3], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            if (!(fabs(mx[i][j]) <= accuracy)) {
                return 0;
            }
        }
    }
    return 1;
}

void m33Print(FILE* pFile, const char* name, double mx[3][3])
{
    size_t dim1 = 3;
    size_t dim2 = 3;
    size_t i;
    size_t j;
    fprintf(pFile, "%s = [", name);
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            fprintf(pFile, "%20.15g", mx[i][j]);
            if (j != dim2 - 1) {
                fprintf(pFile, ", ");
            }
        }
        if (i != dim1 - 1) {
            fprintf(pFile, ";\n");
        }
    }
    fprintf(pFile, "]\n");
}

int m33Inverse(double mx[3][3], double result[3][3])
{
    double det = m33Determinant(mx);
    double detInv;
    double m_result[3][3];
    int status = 0;

    if (fabs(det) > DB0_EPS) {
        detInv = 1.0 / det;
        m_result[0][0] = (mx[1][1] * mx[2][2] - mx[1][2] * mx[2][1]) * detInv;
        m_result[0][1] = -(mx[0][1] * mx[2][2] - mx[0][2] * mx[2][1]) * detInv;
        m_result[0][2] = (mx[0][1] * mx[1][2] - mx[0][2] * mx[1][1]) * detInv;
        m_result[1][0] = -(mx[1][0] * mx[2][2] - mx[1][2] * mx[2][0]) * detInv;
        m_result[1][1] = (mx[0][0] * mx[2][2] - mx[0][2] * mx[2][0]) * detInv;
        m_result[1][2] = -(mx[0][0] * mx[1][2] - mx[0][2] * mx[1][0]) * detInv;
        m_result[2][0] = (mx[1][0] * mx[2][1] - mx[1][1] * mx[2][0]) * detInv;
        m_result[2][1] = -(mx[0][0] * mx[2][1] - mx[0][1] * mx[2][0]) * detInv;
        m_result[2][2] = (mx[0][0] * mx[1][1] - mx[0][1] * mx[1][0]) * detInv;
    } else {
        BSK_PRINT(MSG_ERROR, "%s", "Error: singular 3x3 matrix inverse");
        m33Set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m_result);
        status = 1;
    }
    m33Copy(m_result, result);
    return status;
}

void m33SingularValues(double mx[3][3], double result[3])
{
    double sv[3];
    double a[3];
    double mxtmx[3][3];
    int i;

    m33tMultM33(mx, mx, mxtmx);

    /* Compute characteristic polynomial */
    a[0] = -m33Determinant(mxtmx);
    a[1] = mxtmx[0][0] * mxtmx[1][1] - mxtmx[0][1] * mxtmx[1][0] + mxtmx[0][0] * mxtmx[2][2] -
           mxtmx[0][2] * mxtmx[2][0] + mxtmx[1][1] * mxtmx[2][2] - mxtmx[1][2] * mxtmx[2][1];
    a[2] = -mxtmx[0][0] - mxtmx[1][1] - mxtmx[2][2];

    /* Solve cubic equation */
    cubicRoots(a, sv);

    /* take square roots */
    for (i = 0; i < 3; i++) {
        sv[i] = sqrt(sv[i]);
    }

    /* order roots */
    v3Sort(sv, result);
}

void m33EigenValues(double mx[3][3], double result[3])
{
    double sv[3];
    double a[3];

    /* Compute characteristic polynomial */
    a[0] = -m33Determinant(mx);
    a[1] = mx[0][0] * mx[1][1] - mx[0][1] * mx[1][0] + mx[0][0] * mx[2][2] - mx[0][2] * mx[2][0] + mx[1][1] * mx[2][2] -
           mx[1][2] * mx[2][1];
    a[2] = -mx[0][0] - mx[1][1] - mx[2][2];

    /* Solve cubic equation */
    cubicRoots(a, sv);

    /* order roots */
    v3Sort(sv, result);
}

double m33ConditionNumber(double mx[3][3])
{
    double sv[3];
    m33SingularValues(mx, sv);
    return (sv[0] / sv[2]);
}

void m33PrintScreen(const char* name, double mx[3][3])
{
    int i;
    printf("%s:\n", name);
    for (i = 0; i < 3; i++) {
        printf("%20.15g, %20.15g, %20.15g\n", mx[i][0], mx[i][1], mx[i][2]);
    }
}

void m44Set(double m00,
            double m01,
            double m02,
            double m03,
            double m10,
            double m11,
            double m12,
            double m13,
            double m20,
            double m21,
            double m22,
            double m23,
            double m30,
            double m31,
            double m32,
            double m33,
            double m[4][4])
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[0][2] = m02;
    m[0][3] = m03;
    m[1][0] = m10;
    m[1][1] = m11;
    m[1][2] = m12;
    m[1][3] = m13;
    m[2][0] = m20;
    m[2][1] = m21;
    m[2][2] = m22;
    m[2][3] = m23;
    m[3][0] = m30;
    m[3][1] = m31;
    m[3][2] = m32;
    m[3][3] = m33;
}

void m44Copy(double mx[4][4], double result[4][4])
{
    MOVE_DOUBLE(mx, 4 * 4, result);
}

void m44SetZero(double result[4][4])
{
    size_t dim1 = 4;
    size_t dim2 = 4;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void m44MultV4(double mx[4][4], double v[4], double result[4])
{
    size_t dim11 = 4;
    size_t dim12 = 4;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    double m_result[4];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i] += mx[i][k] * v[k];
            }
        }
    }
    v4Copy(m_result, result);
}

double m44Determinant(double mx[4][4])
{
    double value;
    value = mx[0][3] * mx[1][2] * mx[2][1] * mx[3][0] - mx[0][2] * mx[1][3] * mx[2][1] * mx[3][0] -
            mx[0][3] * mx[1][1] * mx[2][2] * mx[3][0] + mx[0][1] * mx[1][3] * mx[2][2] * mx[3][0] +
            mx[0][2] * mx[1][1] * mx[2][3] * mx[3][0] - mx[0][1] * mx[1][2] * mx[2][3] * mx[3][0] -
            mx[0][3] * mx[1][2] * mx[2][0] * mx[3][1] + mx[0][2] * mx[1][3] * mx[2][0] * mx[3][1] +
            mx[0][3] * mx[1][0] * mx[2][2] * mx[3][1] - mx[0][0] * mx[1][3] * mx[2][2] * mx[3][1] -
            mx[0][2] * mx[1][0] * mx[2][3] * mx[3][1] + mx[0][0] * mx[1][2] * mx[2][3] * mx[3][1] +
            mx[0][3] * mx[1][1] * mx[2][0] * mx[3][2] - mx[0][1] * mx[1][3] * mx[2][0] * mx[3][2] -
            mx[0][3] * mx[1][0] * mx[2][1] * mx[3][2] + mx[0][0] * mx[1][3] * mx[2][1] * mx[3][2] +
            mx[0][1] * mx[1][0] * mx[2][3] * mx[3][2] - mx[0][0] * mx[1][1] * mx[2][3] * mx[3][2] -
            mx[0][2] * mx[1][1] * mx[2][0] * mx[3][3] + mx[0][1] * mx[1][2] * mx[2][0] * mx[3][3] +
            mx[0][2] * mx[1][0] * mx[2][1] * mx[3][3] - mx[0][0] * mx[1][2] * mx[2][1] * mx[3][3] -
            mx[0][1] * mx[1][0] * mx[2][2] * mx[3][3] + mx[0][0] * mx[1][1] * mx[2][2] * mx[3][3];
    return value;
}

int m44IsEqual(double mx1[4][4], double mx2[4][4], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim1 = 4;
    size_t dim2 = 4;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            if (!(fabs(mx1[i][j] - mx2[i][j]) <= accuracy)) {
                return 0;
            }
        }
    }
    return 1;
}

int m44Inverse(double mx[4][4], double result[4][4])
{
    double det = m44Determinant(mx);
    double detInv;
    double m_result[4][4];
    int status = 0;

    if (fabs(det) > DB0_EPS) {
        detInv = 1.0 / det;
        m_result[0][0] =
            (mx[1][2] * mx[2][3] * mx[3][1] - mx[1][3] * mx[2][2] * mx[3][1] + mx[1][3] * mx[2][1] * mx[3][2] -
             mx[1][1] * mx[2][3] * mx[3][2] - mx[1][2] * mx[2][1] * mx[3][3] + mx[1][1] * mx[2][2] * mx[3][3]) *
            detInv;
        m_result[0][1] =
            (mx[0][3] * mx[2][2] * mx[3][1] - mx[0][2] * mx[2][3] * mx[3][1] - mx[0][3] * mx[2][1] * mx[3][2] +
             mx[0][1] * mx[2][3] * mx[3][2] + mx[0][2] * mx[2][1] * mx[3][3] - mx[0][1] * mx[2][2] * mx[3][3]) *
            detInv;
        m_result[0][2] =
            (mx[0][2] * mx[1][3] * mx[3][1] - mx[0][3] * mx[1][2] * mx[3][1] + mx[0][3] * mx[1][1] * mx[3][2] -
             mx[0][1] * mx[1][3] * mx[3][2] - mx[0][2] * mx[1][1] * mx[3][3] + mx[0][1] * mx[1][2] * mx[3][3]) *
            detInv;
        m_result[0][3] =
            (mx[0][3] * mx[1][2] * mx[2][1] - mx[0][2] * mx[1][3] * mx[2][1] - mx[0][3] * mx[1][1] * mx[2][2] +
             mx[0][1] * mx[1][3] * mx[2][2] + mx[0][2] * mx[1][1] * mx[2][3] - mx[0][1] * mx[1][2] * mx[2][3]) *
            detInv;
        m_result[1][0] =
            (mx[1][3] * mx[2][2] * mx[3][0] - mx[1][2] * mx[2][3] * mx[3][0] - mx[1][3] * mx[2][0] * mx[3][2] +
             mx[1][0] * mx[2][3] * mx[3][2] + mx[1][2] * mx[2][0] * mx[3][3] - mx[1][0] * mx[2][2] * mx[3][3]) *
            detInv;
        m_result[1][1] =
            (mx[0][2] * mx[2][3] * mx[3][0] - mx[0][3] * mx[2][2] * mx[3][0] + mx[0][3] * mx[2][0] * mx[3][2] -
             mx[0][0] * mx[2][3] * mx[3][2] - mx[0][2] * mx[2][0] * mx[3][3] + mx[0][0] * mx[2][2] * mx[3][3]) *
            detInv;
        m_result[1][2] =
            (mx[0][3] * mx[1][2] * mx[3][0] - mx[0][2] * mx[1][3] * mx[3][0] - mx[0][3] * mx[1][0] * mx[3][2] +
             mx[0][0] * mx[1][3] * mx[3][2] + mx[0][2] * mx[1][0] * mx[3][3] - mx[0][0] * mx[1][2] * mx[3][3]) *
            detInv;
        m_result[1][3] =
            (mx[0][2] * mx[1][3] * mx[2][0] - mx[0][3] * mx[1][2] * mx[2][0] + mx[0][3] * mx[1][0] * mx[2][2] -
             mx[0][0] * mx[1][3] * mx[2][2] - mx[0][2] * mx[1][0] * mx[2][3] + mx[0][0] * mx[1][2] * mx[2][3]) *
            detInv;
        m_result[2][0] =
            (mx[1][1] * mx[2][3] * mx[3][0] - mx[1][3] * mx[2][1] * mx[3][0] + mx[1][3] * mx[2][0] * mx[3][1] -
             mx[1][0] * mx[2][3] * mx[3][1] - mx[1][1] * mx[2][0] * mx[3][3] + mx[1][0] * mx[2][1] * mx[3][3]) *
            detInv;
        m_result[2][1] =
            (mx[0][3] * mx[2][1] * mx[3][0] - mx[0][1] * mx[2][3] * mx[3][0] - mx[0][3] * mx[2][0] * mx[3][1] +
             mx[0][0] * mx[2][3] * mx[3][1] + mx[0][1] * mx[2][0] * mx[3][3] - mx[0][0] * mx[2][1] * mx[3][3]) *
            detInv;
        m_result[2][2] =
            (mx[0][1] * mx[1][3] * mx[3][0] - mx[0][3] * mx[1][1] * mx[3][0] + mx[0][3] * mx[1][0] * mx[3][1] -
             mx[0][0] * mx[1][3] * mx[3][1] - mx[0][1] * mx[1][0] * mx[3][3] + mx[0][0] * mx[1][1] * mx[3][3]) *
            detInv;
        m_result[2][3] =
            (mx[0][3] * mx[1][1] * mx[2][0] - mx[0][1] * mx[1][3] * mx[2][0] - mx[0][3] * mx[1][0] * mx[2][1] +
             mx[0][0] * mx[1][3] * mx[2][1] + mx[0][1] * mx[1][0] * mx[2][3] - mx[0][0] * mx[1][1] * mx[2][3]) *
            detInv;
        m_result[3][0] =
            (mx[1][2] * mx[2][1] * mx[3][0] - mx[1][1] * mx[2][2] * mx[3][0] - mx[1][2] * mx[2][0] * mx[3][1] +
             mx[1][0] * mx[2][2] * mx[3][1] + mx[1][1] * mx[2][0] * mx[3][2] - mx[1][0] * mx[2][1] * mx[3][2]) *
            detInv;
        m_result[3][1] =
            (mx[0][1] * mx[2][2] * mx[3][0] - mx[0][2] * mx[2][1] * mx[3][0] + mx[0][2] * mx[2][0] * mx[3][1] -
             mx[0][0] * mx[2][2] * mx[3][1] - mx[0][1] * mx[2][0] * mx[3][2] + mx[0][0] * mx[2][1] * mx[3][2]) *
            detInv;
        m_result[3][2] =
            (mx[0][2] * mx[1][1] * mx[3][0] - mx[0][1] * mx[1][2] * mx[3][0] - mx[0][2] * mx[1][0] * mx[3][1] +
             mx[0][0] * mx[1][2] * mx[3][1] + mx[0][1] * mx[1][0] * mx[3][2] - mx[0][0] * mx[1][1] * mx[3][2]) *
            detInv;
        m_result[3][3] =
            (mx[0][1] * mx[1][2] * mx[2][0] - mx[0][2] * mx[1][1] * mx[2][0] + mx[0][2] * mx[1][0] * mx[2][1] -
             mx[0][0] * mx[1][2] * mx[2][1] - mx[0][1] * mx[1][0] * mx[2][2] + mx[0][0] * mx[1][1] * mx[2][2]) *
            detInv;
    } else {
        BSK_PRINT(MSG_ERROR, "%s", "Error: singular 4x4 matrix inverse");
        m44Set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m_result);
        status = 1;
    }
    m44Copy(m_result, result);
    return status;
}

void m66Set(double m00,
            double m01,
            double m02,
            double m03,
            double m04,
            double m05,
            double m10,
            double m11,
            double m12,
            double m13,
            double m14,
            double m15,
            double m20,
            double m21,
            double m22,
            double m23,
            double m24,
            double m25,
            double m30,
            double m31,
            double m32,
            double m33,
            double m34,
            double m35,
            double m40,
            double m41,
            double m42,
            double m43,
            double m44,
            double m45,
            double m50,
            double m51,
            double m52,
            double m53,
            double m54,
            double m55,
            double m[6][6])
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[0][2] = m02;
    m[0][3] = m03;
    m[0][4] = m04;
    m[0][5] = m05;
    m[1][0] = m10;
    m[1][1] = m11;
    m[1][2] = m12;
    m[1][3] = m13;
    m[1][4] = m14;
    m[1][5] = m15;
    m[2][0] = m20;
    m[2][1] = m21;
    m[2][2] = m22;
    m[2][3] = m23;
    m[2][4] = m24;
    m[2][5] = m25;
    m[3][0] = m30;
    m[3][1] = m31;
    m[3][2] = m32;
    m[3][3] = m33;
    m[3][4] = m34;
    m[3][5] = m35;
    m[4][0] = m40;
    m[4][1] = m41;
    m[4][2] = m42;
    m[4][3] = m43;
    m[4][4] = m44;
    m[4][5] = m45;
    m[5][0] = m50;
    m[5][1] = m51;
    m[5][2] = m52;
    m[5][3] = m53;
    m[5][4] = m54;
    m[5][5] = m55;
}

void m66Copy(double mx[6][6], double result[6][6])
{
    MOVE_DOUBLE(mx, 6 * 6, result);
}

void m66SetZero(double result[6][6])
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void m66SetIdentity(double result[6][6])
{
    size_t dim = 6;
    size_t i;
    size_t j;
    for (i = 0; i < dim; i++) {
        for (j = 0; j < dim; j++) {
            result[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void m66Transpose(double mx[6][6], double result[6][6])
{
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    double m_result[6][6];
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            m_result[j][i] = mx[i][j];
        }
    }
    m66Copy(m_result, result);
}

void m66Get33Matrix(size_t row, size_t col, double m[6][6], double mij[3][3])
{
    if (row > 1 || col > 1) {
        BSK_PRINT(MSG_ERROR, "%s", "A 6x6 matrix block index must be 0 or 1.");
        return;
    }
    double block[3][3];
    size_t i;
    size_t j;
    /* Read the entire block before writing output that may overlap m. */
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            block[i][j] = m[row * 3 + i][col * 3 + j];
        }
    }
    m33Copy(block, mij);
}

void m66Set33Matrix(size_t row, size_t col, double mij[3][3], double m[6][6])
{
    if (row > 1 || col > 1) {
        BSK_PRINT(MSG_ERROR, "%s", "A 6x6 matrix block index must be 0 or 1.");
        return;
    }
    double block[3][3];
    size_t i;
    size_t j;
    /* Preserve the input before writing matrix entries that may overlap it. */
    m33Copy(mij, block);
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            m[row * 3 + i][col * 3 + j] = block[i][j];
        }
    }
}

void m66Scale(double scaleFactor, double mx[6][6], double result[6][6])
{
    double temporary[6][6];
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            temporary[i][j] = scaleFactor * mx[i][j];
        }
    }
    MOVE_DOUBLE(temporary, 36, result);
}

void m66Add(double mx1[6][6], double mx2[6][6], double result[6][6])
{
    double temporary[6][6];
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            temporary[i][j] = mx1[i][j] + mx2[i][j];
        }
    }
    MOVE_DOUBLE(temporary, 36, result);
}

void m66Subtract(double mx1[6][6], double mx2[6][6], double result[6][6])
{
    double temporary[6][6];
    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            temporary[i][j] = mx1[i][j] - mx2[i][j];
        }
    }
    MOVE_DOUBLE(temporary, 36, result);
}

void m66MultM66(double mx1[6][6], double mx2[6][6], double result[6][6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim22 = 6;
    size_t i;
    size_t j;
    size_t k;
    double m_result[6][6];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[k][j];
            }
        }
    }
    m66Copy(m_result, result);
}

void m66tMultM66(double mx1[6][6], double mx2[6][6], double result[6][6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim22 = 6;
    size_t i;
    size_t j;
    size_t k;
    double m_result[6][6];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i][j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[k][i] * mx2[k][j];
            }
        }
    }
    m66Copy(m_result, result);
}

void m66MultM66t(double mx1[6][6], double mx2[6][6], double result[6][6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim21 = 6;
    size_t i;
    size_t j;
    size_t k;
    double m_result[6][6];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim21; j++) {
            m_result[i][j] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i][j] += mx1[i][k] * mx2[j][k];
            }
        }
    }
    m66Copy(m_result, result);
}

void m66MultV6(double mx[6][6], double v[6], double result[6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    double m_result[6];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i] += mx[i][k] * v[k];
            }
        }
    }
    v6Copy(m_result, result);
}

void m66tMultV6(double mx[6][6], double v[6], double result[6])
{
    size_t dim11 = 6;
    size_t dim12 = 6;
    size_t dim22 = 1;
    size_t i;
    size_t j;
    size_t k;
    double m_result[6];
    for (i = 0; i < dim11; i++) {
        for (j = 0; j < dim22; j++) {
            m_result[i] = 0.0;
            for (k = 0; k < dim12; k++) {
                m_result[i] += mx[k][i] * v[k];
            }
        }
    }
    v6Copy(m_result, result);
}

int m66IsEqual(double mx1[6][6], double mx2[6][6], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            if (!(fabs(mx1[i][j] - mx2[i][j]) <= accuracy)) {
                return 0;
            }
        }
    }
    return 1;
}

int m66IsZero(double mx[6][6], double accuracy)
{
    if (!isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }

    size_t dim1 = 6;
    size_t dim2 = 6;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            if (!(fabs(mx[i][j]) <= accuracy)) {
                return 0;
            }
        }
    }
    return 1;
}

void m99SetZero(double result[9][9])
{
    size_t dim1 = 9;
    size_t dim2 = 9;
    size_t i;
    size_t j;
    for (i = 0; i < dim1; i++) {
        for (j = 0; j < dim2; j++) {
            result[i][j] = 0.0;
        }
    }
}

void cubicRoots(double a[3], double result[3])
{
    /* Solve cubic formula for x^3 + a[2]*x^2 + a[1]*x + a[0] = 0 */
    /* see http://mathworld.wolfram.com/CubicFormula.html */
    double a2sq = a[2] * a[2];
    double a2d3 = a[2] / 3.0;
    double Q = (3.0 * a[1] - a2sq) / 9.0;
    double R = (a[2] * (9 * a[1] - 2 * a2sq) - 27 * a[0]) / 54.0;
    double RdsqrtnQ3 = R / sqrt(-Q * Q * Q);

    if (Q < 0.0 && fabs(RdsqrtnQ3) < 1.0) {
        /* A = 2*sqrt(-Q)
         * B = a[2]/3
         * result[0] = Acos(t) - B
         * result[1] = Acos(t + 2pi/3) - B = A*(cos(t)*-0.5 - sin(t)*sqrt(3)*0.5) - B
         * result[1] = Acos(t + 4pi/3) - B = A*(cos(t)*-0.5 + sin(t)*sqrt(3)*0.5) - B */
        double A = 2.0 * sqrt(-Q);
        double td3 = safeAcos(RdsqrtnQ3) / 3.0;
        double costd3 = cos(td3);
        double sintd3 = sin(td3);
        double sqrt3d2 = sqrt(3) * 0.5;
        double temp1 = -0.5 * costd3;
        double temp2 = sqrt3d2 * sintd3;

        result[0] = A * costd3 - a2d3;
        result[1] = A * (temp1 - temp2) - a2d3;
        result[2] = A * (temp1 + temp2) - a2d3;
    } else {
        double D = Q * Q * Q + R * R;
        double sqrtD = sqrt(D);
        double S = cbrt(R + sqrtD);
        double T = cbrt(R - sqrtD);

        result[0] = -a2d3 + (S + T);
        result[1] = -a2d3 - 0.5 * (S + T);
        result[2] = result[1];
    }
}

double safeAcos(double x)
{
    if (x < -1.0)
        return acos(-1);
    else if (x > 1.0)
        return acos(1);
    return acos(x);
}

double safeAsin(double x)
{
    if (x < -1.0)
        return asin(-1);
    else if (x > 1.0)
        return asin(1);
    return asin(x);
}

double safeSqrt(double x)
{
    return sqrt(isless(x, 0.0) ? 0.0 : x);
}
