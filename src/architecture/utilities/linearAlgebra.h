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

#ifndef _LINEARALGEBRA_H_
#define _LINEARALGEBRA_H_

#include <architecture/utilities/bskLogging.h>
#include <stddef.h>
#include <stdio.h>

/* Divide by zero epsilon value */
#define DB0_EPS 1e-30

/* Maximum number of doubles in a local scratch array. */
#define LINEAR_ALGEBRA_MAX_ARRAY_SIZE (64 * 64)

#define MXINDEX(dim2, row, col) ((row) * (dim2) + (col))

/* General vectors */
#ifdef __cplusplus
extern "C"
{
#endif

    /* N element vectors */
    /**
     * @brief Multiply corresponding elements, allowing overlap with either input.
     * @param v1 First input vector.
     * @param dim Number of elements.
     * @param v2 Second input vector.
     * @param result Output of the same shape; may overlap either or both inputs.
     * @note Empty inputs cause no memory access; pointers may be null.
     * @note If overlapping inputs require opposite traversal directions, the smaller input
     * overlap with result must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Other layouts have no
     * fixed size limit. Unrepresentable byte counts or excessive scratch requirements are
     * logged and leave result unchanged. No heap memory is used.
     */
    void vElementwiseMult(double* v1, size_t dim, double* v2, double* result);
    /**
     * @brief Copy a vector, allowing overlapping input and output storage.
     * @param v Input vector containing dim elements.
     * @param dim Number of elements to copy.
     * @param result Output vector containing dim elements; may overlap v.
     * @note If dim is zero, no memory is accessed and either pointer may be null.
     * @note Unrepresentable byte counts are logged and leave result unchanged.
     */
    void vCopy(double* v, size_t dim, double* result);
    void vSetZero(double* v, size_t dim);
    void vSetOnes(double* v, size_t dim);
    /**
     * @brief Add corresponding elements, allowing overlap with either input.
     * @param v1 First input vector.
     * @param dim Number of elements.
     * @param v2 Second input vector.
     * @param result Output of the same shape; may overlap either or both inputs.
     * @note Empty inputs cause no memory access; pointers may be null.
     * @note If overlapping inputs require opposite traversal directions, the smaller input
     * overlap with result must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Other layouts have no
     * fixed size limit. Unrepresentable byte counts or excessive scratch requirements are
     * logged and leave result unchanged. No heap memory is used.
     */
    void vAdd(double* v1, size_t dim, double* v2, double* result);
    /**
     * @brief Subtract corresponding elements, allowing overlap with either input.
     * @param v1 First input vector.
     * @param dim Number of elements.
     * @param v2 Second input vector.
     * @param result Output of the same shape; may overlap either or both inputs.
     * @note Empty inputs cause no memory access; pointers may be null.
     * @note If overlapping inputs require opposite traversal directions, the smaller input
     * overlap with result must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Other layouts have no
     * fixed size limit. Unrepresentable byte counts or excessive scratch requirements are
     * logged and leave result unchanged. No heap memory is used.
     */
    void vSubtract(double* v1, size_t dim, double* v2, double* result);
    /**
     * @brief Scale every element, allowing overlapping input and output.
     * @param scaleFactor Multiplier.
     * @param v Input vector.
     * @param dim Number of elements.
     * @param result Output of the same shape; may overlap the input.
     * @note Empty input causes no memory access; pointers may be null. Unrepresentable byte
     * counts are logged and leave result unchanged. No heap storage or fixed scratch limit is used.
     */
    void vScale(double scaleFactor, double* v, size_t dim, double* result);
    double vDot(double* v1, size_t dim, double* v2);
    /**
     * @brief Form a row-major outer product, allowing either input to overlap the output.
     * @param v1 First vector with dim1 elements.
     * @param dim1 Number of output rows.
     * @param v2 Second vector with dim2 elements.
     * @param dim2 Number of output columns.
     * @param result Output matrix with dim1 * dim2 elements.
     * @note A zero dimension causes no memory access; pointers may be null.
     * @note Overlapping input vectors are saved in local scratch storage; their combined
     * lengths must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Disjoint inputs and one-row or
     * one-column outputs have no fixed size limit. Unrepresentable sizes or excessive
     * scratch requirements are logged and leave result unchanged. No heap memory is used.
     */
    void vOuterProduct(double* v1, size_t dim1, double* v2, size_t dim2, void* result);
    /**
     * @brief Compute v transpose times mx.
     * @param v Input vector with dim1 elements.
     * @param mx Input matrix in contiguous row-major storage.
     * @param dim1 Number of matrix rows.
     * @param dim2 Number of matrix columns.
     * @param result Output vector with dim2 elements; may overlap either input.
     * @note dim2 must not exceed LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Invalid sizes are logged
     * and leave result unchanged. An empty output causes no memory access. A zero inner
     * dimension produces zeros. Pointers to empty arrays may be null.
     */
    void vtMultM(double* v, void* mx, size_t dim1, size_t dim2, void* result);
    /**
     * @brief Compute v transpose times mx transpose.
     * @param v Input vector with dim2 elements.
     * @param mx Input matrix in contiguous row-major storage.
     * @param dim1 Number of matrix rows.
     * @param dim2 Number of matrix columns.
     * @param result Output vector with dim1 elements; may overlap either input.
     * @note dim1 must not exceed LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Invalid sizes are logged
     * and leave result unchanged. An empty output causes no memory access. A zero inner
     * dimension produces zeros. Pointers to empty arrays may be null.
     */
    void vtMultMt(double* v, void* mx, size_t dim1, size_t dim2, void* result);
    double vNorm(double* v, size_t dim);
    /**
     * @brief Find the maximum element of a nonempty vector.
     * @param v Input vector.
     * @param dim Number of elements.
     * @return Maximum element; any NaN element gives NaN, regardless of its position.
     * Infinities retain their usual ordering. An empty input is logged and returns NaN
     * without accessing v.
     */
    double vMax(double* v, size_t dim);
    /**
     * @brief Find the maximum absolute element of a nonempty vector.
     * @param v Input vector.
     * @param dim Number of elements.
     * @return Maximum absolute element; any NaN element gives NaN, regardless of its position.
     * Either infinity gives positive infinity when no NaN is present. An empty input is
     * logged and returns NaN without accessing v.
     */
    double vMaxAbs(double* v, size_t dim);
    /**
     * @brief Normalize a vector, allowing overlapping input and output.
     * @param v Input vector.
     * @param dim Number of elements.
     * @param result Output vector; may overlap the input.
     * @note A finite norm greater than DB0_EPS gives a unit vector; smaller norms give zeros.
     * A nonfinite computed norm, including from NaN or infinite input, produces NaN in every
     * output component. The norm computation does not scale extreme finite magnitudes.
     * @note Empty input causes no memory access; pointers may be null. Unrepresentable byte
     * counts are logged and leave result unchanged. No heap storage or fixed scratch limit is used.
     */
    void vNormalize(double* v, size_t dim, double* result);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param v1 Input array.
     * @param dim Number of elements.
     * @param v2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     * @note An empty array compares equal for a valid tolerance, without accessing its storage.
     */
    int vIsEqual(double* v1, size_t dim, double* v2, double accuracy);
    /**
     * @brief Compare every element with zero using an absolute tolerance.
     * @param v Input array.
     * @param dim Number of elements.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     * @note An empty array compares equal for a valid tolerance, without accessing its storage.
     */
    int vIsZero(double* v, size_t dim, double accuracy);
    void vPrint(FILE* pFile, const char* name, double* v, size_t dim);
    /**
     * @brief Stably sort a vector in ascending order, with NaNs last.
     * @param Input Input vector containing dim elements.
     * @param Output Output vector containing dim elements; may overlap Input.
     * @param dim Number of elements to sort.
     * @note An empty input causes no memory access; pointers may be null.
     * Unrepresentable byte counts are logged and leave Output unchanged.
     * Equal values, including signed zeros and NaNs, retain their input order.
     * Already sorted input takes linear time; worst-case work remains quadratic.
     */
    void vSort(double* Input, double* Output, size_t dim);

    /* 2 element vectors */
    void v2Set(double v0, double v1, double result[2]);
    /**
     * @brief Copy a two-component vector.
     * @param v Input vector.
     * @param result Output vector, which may overlap v.
     */
    void v2Copy(double v[2], double result[2]);
    /**
     * @brief Scale every element.
     * @param scaleFactor Multiplier.
     * @param v Input array.
     * @param result Output of the same shape; may overlap the input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void v2Scale(double scaleFactor, double v[2], double result[2]);
    void v2SetZero(double v[2]);
    double v2Dot(double v1[2], double v2[2]);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param v1 Input array.
     * @param v2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int v2IsEqual(double v1[2], double v2[2], double accuracy);
    /**
     * @brief Compare every element with zero using an absolute tolerance.
     * @param v Input array.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int v2IsZero(double v[2], double accuracy);
    /**
     * @brief Add corresponding elements.
     * @param v1 First input.
     * @param v2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void v2Add(double v1[2], double v2[2], double result[2]);
    /**
     * @brief Subtract corresponding elements.
     * @param v1 First input.
     * @param v2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void v2Subtract(double v1[2], double v2[2], double result[2]);
    double v2Norm(double v1[2]);
    /**
     * @brief Normalize a vector, allowing overlapping input and output.
     * @param v1 Input vector.
     * @param result Output vector; may overlap the input.
     * @note A finite norm greater than DB0_EPS gives a unit vector; smaller norms give zeros.
     * A nonfinite computed norm, including from NaN or infinite input, produces NaN in every
     * output component. The norm computation does not scale extreme finite magnitudes.
     */
    void v2Normalize(double v1[2], double result[2]);

    /* 3 element vectors */
    void v3Set(double v0, double v1, double v2, double result[3]);
    /**
     * @brief Copy a three-component vector.
     * @param v Input vector.
     * @param result Output vector, which may overlap v.
     */
    void v3Copy(double v[3], double result[3]);
    void v3SetZero(double v[3]);
    /**
     * @brief Add corresponding elements.
     * @param v1 First input.
     * @param v2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void v3Add(double v1[3], double v2[3], double result[3]);
    /**
     * @brief Subtract corresponding elements.
     * @param v1 First input.
     * @param v2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void v3Subtract(double v1[3], double v2[3], double result[3]);
    /**
     * @brief Scale every element.
     * @param scaleFactor Multiplier.
     * @param v Input array.
     * @param result Output of the same shape; may overlap the input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void v3Scale(double scaleFactor, double v[3], double result[3]);
    double v3Dot(double v1[3], double v2[3]);
    /**
     * @brief Form the outer product of two three-component vectors.
     * @param v1 First input vector, supplying the row factors.
     * @param v2 Second input vector, supplying the column factors.
     * @param result Output matrix with result[i][j] = v1[i] * v2[j]; may overlap either or both inputs.
     */
    void v3OuterProduct(double v1[3], double v2[3], double result[3][3]);
    void v3tMultM33(double v[3], double mx[3][3], double result[3]);
    void v3tMultM33t(double v[3], double mx[3][3], double result[3]);
    double v3Norm(double v[3]);
    /**
     * @brief Normalize a vector, allowing overlapping input and output.
     * @param v Input vector.
     * @param result Output vector; may overlap the input.
     * @note A finite norm greater than DB0_EPS gives a unit vector; smaller norms give zeros.
     * A nonfinite computed norm, including from NaN or infinite input, produces NaN in every
     * output component. The norm computation does not scale extreme finite magnitudes.
     */
    void v3Normalize(double v[3], double result[3]);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param v1 Input array.
     * @param v2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int v3IsEqual(double v1[3], double v2[3], double accuracy);
    /**
     * @brief Compare finite vectors relative to the Euclidean norm of the first vector.
     * @param v1 Reference vector with three components.
     * @param v2 Vector to compare, in the same units as v1.
     * @param accuracy Finite, nonnegative relative tolerance (dimensionless).
     * @return 1 if every component satisfies abs(v1[i] - v2[i]) / norm(v1) <= accuracy, otherwise 0.
     * @note A zero reference vector or zero tolerance requires exact equality. NaN and infinity
     * are rejected in either vector or the tolerance, including comparisons of equal infinities.
     */
    int v3IsEqualRel(double v1[3], double v2[3], double accuracy);
    /**
     * @brief Compare every element with zero using an absolute tolerance.
     * @param v Input array.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int v3IsZero(double v[3], double accuracy);
    void v3Print(FILE* pFile, const char* name, double v[3]);
    void v3Cross(double v1[3], double v2[3], double result[3]);
    /**
     * @brief Compute a normalized vector perpendicular to the input.
     * @param v Input vector.
     * @param result Output vector; may overlap any input.
     * @note Zero input returns zero. Nonzero inputs must be finite and satisfy at least one
     * of abs(v[0]) > DB0_EPS, abs(v[1]) > DB0_EPS, or v[2] != 0. The intermediate
     * perpendicular components and their sum of squares must also remain finite.
     * These restrictions apply even when the input norm is representable: for example,
     * {1e-31, 0, 0} is unsupported and can raise a divide-by-zero exception and produce NaNs.
     */
    void v3Perpendicular(double v[3], double result[3]);
    /**
     * @brief Form the skew-symmetric matrix whose product with a vector w is v cross w.
     * @param v Input vector with three components.
     * @param result Output matrix, which may overlap the input vector.
     */
    void v3Tilde(double v[3], double result[3][3]);
    /**
     * @brief Stably sort three values in descending order, with NaNs last.
     * @param v Input vector.
     * @param result Output vector; may overlap v.
     * @note Equal values, including signed zeros and NaNs, retain their input order.
     */
    void v3Sort(double v[3], double result[3]);
    void v3PrintScreen(const char* name, double v[3]);

    /* 4 element vectors */
    void v4Set(double v0, double v1, double v2, double v3, double result[4]);
    /**
     * @brief Copy a four-component vector.
     * @param v Input vector.
     * @param result Output vector, which may overlap v.
     */
    void v4Copy(double v[4], double result[4]);
    void v4SetZero(double v[4]);
    double v4Dot(double v1[4], double v2[4]);
    double v4Norm(double v[4]);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param v1 Input array.
     * @param v2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int v4IsEqual(double v1[4], double v2[4], double accuracy);
    /**
     * @brief Compare every element with zero using an absolute tolerance.
     * @param v Input array.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int v4IsZero(double v[4], double accuracy);

    /* 6 element vectors */
    void v6Set(double v0, double v1, double v2, double v3, double v4, double v5, double result[6]);
    /**
     * @brief Copy a six-component vector.
     * @param v Input vector.
     * @param result Output vector, which may overlap v.
     */
    void v6Copy(double v[6], double result[6]);
    double v6Dot(double v1[6], double v2[6]);
    /**
     * @brief Scale every element.
     * @param scaleFactor Multiplier.
     * @param v Input array.
     * @param result Output of the same shape; may overlap the input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void v6Scale(double scaleFactor, double v[6], double result[6]);
    /**
     * @brief Form the outer product of two six-component vectors.
     * @param v1 First input vector, supplying the row factors.
     * @param v2 Second input vector, supplying the column factors.
     * @param result Output matrix with result[i][j] = v1[i] * v2[j]; may overlap either or both inputs.
     */
    void v6OuterProduct(double v1[6], double v2[6], double result[6][6]);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param v1 Input array.
     * @param v2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int v6IsEqual(double v1[6], double v2[6], double accuracy);

    /* NxM matrices */
    /**
     * @brief Compute the least-squares inverse using column-pivoted Householder QR.
     * @param mx Input matrix in contiguous row-major storage.
     * @param dim1 Number of input rows.
     * @param dim2 Number of input columns.
     * @param result Output matrix with dim2 rows and dim1 columns; may overlap mx.
     * @note Requires dim1 >= dim2 and a finite, numerically full-column-rank matrix.
     * Both dim1 * dim2 and dim2 * dim2 must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE.
     * Invalid dimensions are logged and leave result unchanged. An empty input causes
     * no memory access; pointers may be null.
     * @note Rank is rejected when a pivot column norm is at most DBL_EPSILON * dim1
     * times the largest original column norm. The squared product of the R diagonal
     * must also exceed DB0_EPS, preserving the absolute Gram-determinant cutoff.
     * Failure of these checks, nonfinite input or pivot norms, or nonfinite inverse
     * entries is logged and zeros result. Overflow of the determinant-related product
     * alone does not imply failure if the remaining checks pass.
     * This routine does not compute a rank-deficient pseudoinverse.
     */
    void mLeastSquaresInverse(void* mx, size_t dim1, size_t dim2, void* result);
    /**
     * @brief Compute the minimum-norm inverse using column-pivoted Householder QR of the transpose.
     * @param mx Input matrix in contiguous row-major storage.
     * @param dim1 Number of input rows.
     * @param dim2 Number of input columns.
     * @param result Output matrix with dim2 rows and dim1 columns; may overlap mx.
     * @note Requires dim1 <= dim2 and a finite, numerically full-row-rank matrix.
     * Both dim1 * dim2 and dim1 * dim1 must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE.
     * Invalid dimensions are logged and leave result unchanged. An empty input causes
     * no memory access; pointers may be null.
     * @note The rank and determinant checks are those of mLeastSquaresInverse applied
     * to the transpose, using dim2 for the rank tolerance's dimension factor.
     * Failure of these checks, nonfinite input or pivot norms, or nonfinite inverse
     * entries is logged and zeros result. Overflow of the determinant-related product
     * alone does not imply failure if the remaining checks pass.
     * This routine does not compute a rank-deficient pseudoinverse.
     */
    void mMinimumNormInverse(void* mx, size_t dim1, size_t dim2, void* result);
    /**
     * @brief Copy a matrix, allowing overlapping input and output storage.
     * @param mx Input matrix stored contiguously in row-major order.
     * @param dim1 Number of rows.
     * @param dim2 Number of columns.
     * @param result Output matrix of the same shape and layout; may overlap mx.
     * @note If either dimension is zero, no memory is accessed and either pointer may be null.
     * @note Unrepresentable byte counts are logged and leave result unchanged.
     */
    void mCopy(void* mx, size_t dim1, size_t dim2, void* result);
    void mSetZero(void* result, size_t dim1, size_t dim2);
    void mSetIdentity(void* result, size_t dim1, size_t dim2);
    /**
     * @brief Form a diagonal matrix from a vector, setting all off-diagonal entries to zero.
     * @param v Input vector containing dim elements.
     * @param dim Number of input elements and number of rows and columns in the output.
     * @param result Output matrix with dim * dim elements in row-major order; may overlap v.
     * @note If dim is zero, no memory is accessed and either pointer may be null.
     * @note Unrepresentable byte counts are logged and leave result unchanged.
     */
    void mDiag(void* v, size_t dim, void* result);
    /**
     * @brief Transpose a row-major matrix, allowing overlapping input and output.
     * @param mx Input matrix.
     * @param dim1 Number of input rows.
     * @param dim2 Number of input columns.
     * @param result Output matrix with dim2 rows and dim1 columns.
     * @note dim1 * dim2 must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Invalid sizes are logged
     * and leave result unchanged. An empty input causes no memory access; pointers may be null.
     */
    void mTranspose(void* mx, size_t dim1, size_t dim2, void* result);
    /**
     * @brief Add corresponding elements, allowing overlap with either input.
     * @param mx1 First input matrix.
     * @param dim1 Number of rows.
     * @param dim2 Number of columns.
     * @param mx2 Second input matrix.
     * @param result Output of the same shape; may overlap either or both inputs.
     * @note Empty inputs cause no memory access; pointers may be null.
     * @note If overlapping inputs require opposite traversal directions, the smaller input
     * overlap with result must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Other layouts have no
     * fixed size limit. Unrepresentable byte counts or excessive scratch requirements are
     * logged and leave result unchanged. No heap memory is used.
     */
    void mAdd(void* mx1, size_t dim1, size_t dim2, void* mx2, void* result);
    /**
     * @brief Subtract corresponding elements, allowing overlap with either input.
     * @param mx1 First input matrix.
     * @param dim1 Number of rows.
     * @param dim2 Number of columns.
     * @param mx2 Second input matrix.
     * @param result Output of the same shape; may overlap either or both inputs.
     * @note Empty inputs cause no memory access; pointers may be null.
     * @note If overlapping inputs require opposite traversal directions, the smaller input
     * overlap with result must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Other layouts have no
     * fixed size limit. Unrepresentable byte counts or excessive scratch requirements are
     * logged and leave result unchanged. No heap memory is used.
     */
    void mSubtract(void* mx1, size_t dim1, size_t dim2, void* mx2, void* result);
    /**
     * @brief Scale every element, allowing overlapping input and output.
     * @param scaleFactor Multiplier.
     * @param mx Input matrix.
     * @param dim1 Number of rows.
     * @param dim2 Number of columns.
     * @param result Output of the same shape; may overlap the input.
     * @note Empty input causes no memory access; pointers may be null. Unrepresentable byte
     * counts are logged and leave result unchanged. No heap storage or fixed scratch limit is used.
     */
    void mScale(double scaleFactor, void* mx, size_t dim1, size_t dim2, void* result);
    /**
     * @brief Compute mx1 times mx2.
     * @param mx1 First input matrix in contiguous row-major storage.
     * @param dim11 Number of rows of mx1.
     * @param dim12 Number of columns of mx1.
     * @param mx2 Second input matrix in contiguous row-major storage.
     * @param dim21 Number of rows of mx2.
     * @param dim22 Number of columns of mx2.
     * @param result Row-major output matrix, which may overlap either input.
     * @note Requires dim12 == dim21, and dim11 * dim22 must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE.
     * Invalid sizes are logged and leave result unchanged. An empty output causes no memory
     * access. A zero inner dimension produces zeros. Pointers to empty arrays may be null.
     */
    void mMultM(void* mx1, size_t dim11, size_t dim12, void* mx2, size_t dim21, size_t dim22, void* result);
    /**
     * @brief Compute mx1 transpose times mx2.
     * @param mx1 First input matrix in contiguous row-major storage.
     * @param dim11 Number of rows of mx1.
     * @param dim12 Number of columns of mx1.
     * @param mx2 Second input matrix in contiguous row-major storage.
     * @param dim21 Number of rows of mx2.
     * @param dim22 Number of columns of mx2.
     * @param result Row-major output matrix, which may overlap either input.
     * @note Requires dim11 == dim21, and dim12 * dim22 must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE.
     * Invalid sizes are logged and leave result unchanged. An empty output causes no memory
     * access. A zero inner dimension produces zeros. Pointers to empty arrays may be null.
     */
    void mtMultM(void* mx1, size_t dim11, size_t dim12, void* mx2, size_t dim21, size_t dim22, void* result);
    /**
     * @brief Compute mx1 times mx2 transpose.
     * @param mx1 First input matrix in contiguous row-major storage.
     * @param dim11 Number of rows of mx1.
     * @param dim12 Number of columns of mx1.
     * @param mx2 Second input matrix in contiguous row-major storage.
     * @param dim21 Number of rows of mx2.
     * @param dim22 Number of columns of mx2.
     * @param result Row-major output matrix, which may overlap either input.
     * @note Requires dim12 == dim22, and dim11 * dim21 must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE.
     * Invalid sizes are logged and leave result unchanged. An empty output causes no memory
     * access. A zero inner dimension produces zeros. Pointers to empty arrays may be null.
     */
    void mMultMt(void* mx1, size_t dim11, size_t dim12, void* mx2, size_t dim21, size_t dim22, void* result);
    /**
     * @brief Compute mx1 transpose times mx2 transpose.
     * @param mx1 First input matrix in contiguous row-major storage.
     * @param dim11 Number of rows of mx1.
     * @param dim12 Number of columns of mx1.
     * @param mx2 Second input matrix in contiguous row-major storage.
     * @param dim21 Number of rows of mx2.
     * @param dim22 Number of columns of mx2.
     * @param result Row-major output matrix, which may overlap either input.
     * @note Requires dim11 == dim22, and dim12 * dim21 must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE.
     * Invalid sizes are logged and leave result unchanged. An empty output causes no memory
     * access. A zero inner dimension produces zeros. Pointers to empty arrays may be null.
     */
    void mtMultMt(void* mx1, size_t dim11, size_t dim12, void* mx2, size_t dim21, size_t dim22, void* result);
    /**
     * @brief Compute mx times v.
     * @param v Input vector with dim2 elements.
     * @param mx Input matrix in contiguous row-major storage.
     * @param dim1 Number of matrix rows.
     * @param dim2 Number of matrix columns.
     * @param result Output vector with dim1 elements; may overlap either input.
     * @note dim1 must not exceed LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Invalid sizes are logged
     * and leave result unchanged. An empty output causes no memory access. A zero inner
     * dimension produces zeros. Pointers to empty arrays may be null.
     */
    void mMultV(void* mx, size_t dim1, size_t dim2, void* v, void* result);
    /**
     * @brief Compute mx transpose times v.
     * @param v Input vector with dim1 elements.
     * @param mx Input matrix in contiguous row-major storage.
     * @param dim1 Number of matrix rows.
     * @param dim2 Number of matrix columns.
     * @param result Output vector with dim2 elements; may overlap either input.
     * @note dim2 must not exceed LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Invalid sizes are logged
     * and leave result unchanged. An empty output causes no memory access. A zero inner
     * dimension produces zeros. Pointers to empty arrays may be null.
     */
    void mtMultV(void* mx, size_t dim1, size_t dim2, void* v, void* result);
    double mTrace(void* mx, size_t dim);
    /**
     * @brief Compute the determinant of a square row-major matrix using pivoted LU.
     * @param mx Input matrix.
     * @param dim Number of rows and columns.
     * @return Determinant, or NaN after logging invalid dimensions.
     * @note The (dim - 1) squared minor must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE.
     * For dim == 0, returns zero without accessing mx, preserving the existing convention.
     * Scalar and 2-by-2 inputs use direct formulas. No singularity tolerance is applied.
     * The LU pivot product uses a separate binary exponent to avoid intermediate range loss.
     * Elimination arithmetic and a final determinant outside the double range can still lose range.
     */
    double mDeterminant(void* mx, size_t dim);
    /**
     * @brief Compute the cofactor matrix, allowing overlapping input and output.
     * @param mx Input square row-major matrix.
     * @param dim Number of rows and columns.
     * @param result Output square row-major matrix.
     * @note dim * dim must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Invalid dimensions are logged
     * and leave result unchanged. For dim == 0, no memory is accessed and pointers may be null.
     * Minors are evaluated independently, including for singular input matrices.
     */
    void mCofactor(void* mx, size_t dim, void* result);
    /**
     * @brief Invert a square row-major matrix using pivoted LU, allowing overlapping input and output.
     * @param mx Input matrix.
     * @param dim Number of rows and columns.
     * @param result Output inverse; zeroed on numerical failure.
     * @return Zero on success, one for numerical failure or invalid dimensions.
     * @note dim * dim must fit LINEAR_ALGEBRA_MAX_ARRAY_SIZE. Invalid dimensions, including
     * dim == 0, are logged and leave result unchanged without accessing either pointer.
     * @note The absolute determinant must exceed DB0_EPS; this existing cutoff is not
     * scale invariant. A pivot is also rejected when its magnitude is at most
     * DBL_EPSILON * dim times its row's rounding scale. For P * mx = L * U, that scale
     * is the sum of absolute entries in the original permuted row, plus
     * sum(k < i, abs(L[i,k]) * sum(j >= k, abs(U[k,j]))) for pivot U[i,i]. This includes
     * elimination growth while preserving graded diagonal matrices.
     * @note A final condition check uses B[i,j] = mx[i,j] / r[i] / c[j], where r[i]
     * is the largest absolute entry in row i and c[j] is the largest absolute entry
     * in column j after row scaling. With X the computed inverse, the scaled inverse
     * estimate is Y[i,j] = c[i] * X[i,j] * r[j]. The reciprocal condition estimate
     * 1 / (norm_inf(B) * norm_inf(Y)) must exceed dim * DBL_EPSILON, where norm_inf
     * is the maximum absolute row sum. Scaling is used only for this check; it does
     * not change accepted inverse values. This policy can reject mathematically
     * invertible matrices near numerical singularity and is not an exact rank test.
     * @note Failed numerical checks, nonfinite input or inverse entries, and nonfinite
     * condition-check norms are logged and zero result. Overflow of the determinant
     * alone does not imply failure if the remaining checks pass.
     */
    int mInverse(void* mx, size_t dim, void* result);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param mx1 Input array.
     * @param dim1 Number of rows.
     * @param dim2 Number of columns.
     * @param mx2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     * @note An empty array compares equal for a valid tolerance, without accessing its storage.
     */
    int mIsEqual(void* mx1, size_t dim1, size_t dim2, void* mx2, double accuracy);
    /**
     * @brief Compare every element with zero using an absolute tolerance.
     * @param mx Input array.
     * @param dim1 Number of rows.
     * @param dim2 Number of columns.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     * @note An empty array compares equal for a valid tolerance, without accessing its storage.
     */
    int mIsZero(void* mx, size_t dim1, size_t dim2, double accuracy);
    void mPrintScreen(const char* name, void* mx, size_t dim1, size_t dim2);
    void mPrint(FILE* pFile, const char* name, void* mx, size_t dim1, size_t dim2);
    /**
     * @brief Extract a packed submatrix, allowing overlapping input and output.
     * @param mx Input matrix in contiguous row-major storage.
     * @param dim1 Number of input rows.
     * @param dim2 Number of input columns.
     * @param dim1Start First input row of the selected block.
     * @param dim2Start First input column of the selected block.
     * @param dim1Result Number of output rows.
     * @param dim2Result Number of output columns.
     * @param result Output matrix in contiguous row-major storage.
     * @note Invalid bounds or unrepresentable sizes are logged and leave result unchanged.
     * An empty block causes no memory access; pointers may be null. Overlap is handled by
     * directional row copies, without heap memory or a fixed scratch-buffer size limit.
     */
    void mGetSubMatrix(void* mx,
                       size_t dim1,
                       size_t dim2,
                       size_t dim1Start,
                       size_t dim2Start,
                       size_t dim1Result,
                       size_t dim2Result,
                       void* result);
    /**
     * @brief Insert a packed submatrix, allowing overlapping input and output.
     * @param mx Input matrix in contiguous row-major storage.
     * @param dim1 Number of input rows.
     * @param dim2 Number of input columns.
     * @param dim1Start First output row of the selected block.
     * @param dim2Start First output column of the selected block.
     * @param dim1Result Number of output rows.
     * @param dim2Result Number of output columns.
     * @param result Output matrix in contiguous row-major storage; entries outside the block are preserved.
     * @note Invalid bounds or unrepresentable sizes are logged and leave result unchanged.
     * An empty block causes no memory access; pointers may be null. Overlap is handled by
     * directional row copies, without heap memory or a fixed scratch-buffer size limit.
     */
    void mSetSubMatrix(void* mx,
                       size_t dim1,
                       size_t dim2,
                       void* result,
                       size_t dim1Result,
                       size_t dim2Result,
                       size_t dim1Start,
                       size_t dim2Start);

    /* 2x2 matrices */
    void m22Set(double m00, double m01, double m10, double m11, double m[2][2]);
    /**
     * @brief Copy a 2x2 matrix.
     * @param mx Input matrix.
     * @param result Output matrix, which may overlap mx.
     */
    void m22Copy(double mx[2][2], double result[2][2]);
    void m22SetZero(double result[2][2]);
    void m22SetIdentity(double result[2][2]);
    void m22Transpose(double mx[2][2], double result[2][2]);
    /**
     * @brief Add corresponding elements.
     * @param mx1 First input.
     * @param mx2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void m22Add(double mx1[2][2], double mx2[2][2], double result[2][2]);
    /**
     * @brief Subtract corresponding elements.
     * @param mx1 First input.
     * @param mx2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void m22Subtract(double mx1[2][2], double mx2[2][2], double result[2][2]);
    /**
     * @brief Scale every element.
     * @param scaleFactor Multiplier.
     * @param mx Input array.
     * @param result Output of the same shape; may overlap the input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void m22Scale(double scaleFactor, double mx[2][2], double result[2][2]);
    void m22MultM22(double mx1[2][2], double mx2[2][2], double result[2][2]);
    void m22tMultM22(double mx1[2][2], double mx2[2][2], double result[2][2]);
    void m22MultM22t(double mx1[2][2], double mx2[2][2], double result[2][2]);
    void m22MultV2(double mx[2][2], double v[2], double result[2]);
    void m22tMultV2(double mx[2][2], double v[2], double result[2]);
    double m22Trace(double mx[2][2]);
    double m22Determinant(double mx[2][2]);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param mx1 Input array.
     * @param mx2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int m22IsEqual(double mx1[2][2], double mx2[2][2], double accuracy);
    /**
     * @brief Compare every element with zero using an absolute tolerance.
     * @param mx Input array.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int m22IsZero(double mx[2][2], double accuracy);
    void m22Print(FILE* pFile, const char* name, double mx[2][2]);
    /**
     * @brief Invert a fixed-size square matrix, allowing overlapping input and output.
     * @param mx Input matrix.
     * @param result Output inverse; zeroed on failure and allowed to overlap mx.
     * @return Zero on success, one on failure.
     * @note The absolute determinant must exceed both DB0_EPS and a roundoff bound:
     * DBL_EPSILON times the matrix dimension squared times the sum of absolute determinant
     * expansion terms. This rejects cancellation at roundoff while retaining graded diagonal
     * matrices. Numerically singular matrices and nonfinite input, determinant, roundoff bound,
     * or inverse entries are logged as failures. The direct inverse formula is retained.
     */
    int m22Inverse(double mx[2][2], double result[2][2]);
    void m22PrintScreen(const char* name, double mx[2][2]);

    /* 3x3 matrices */
    void m33Set(double m00,
                double m01,
                double m02,
                double m10,
                double m11,
                double m12,
                double m20,
                double m21,
                double m22,
                double m[3][3]);
    /**
     * @brief Copy a 3x3 matrix.
     * @param mx Input matrix.
     * @param result Output matrix, which may overlap mx.
     */
    void m33Copy(double mx[3][3], double result[3][3]);
    void m33SetZero(double result[3][3]);
    void m33SetIdentity(double result[3][3]);
    void m33Transpose(double mx[3][3], double result[3][3]);
    /**
     * @brief Add corresponding elements.
     * @param mx1 First input.
     * @param mx2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void m33Add(double mx1[3][3], double mx2[3][3], double result[3][3]);
    /**
     * @brief Subtract corresponding elements.
     * @param mx1 First input.
     * @param mx2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void m33Subtract(double mx1[3][3], double mx2[3][3], double result[3][3]);
    /**
     * @brief Scale every element.
     * @param scaleFactor Multiplier.
     * @param mx Input array.
     * @param result Output of the same shape; may overlap the input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void m33Scale(double scaleFactor, double mx[3][3], double result[3][3]);
    void m33MultM33(double mx1[3][3], double mx2[3][3], double result[3][3]);
    void m33tMultM33(double mx1[3][3], double mx2[3][3], double result[3][3]);
    void m33MultM33t(double mx1[3][3], double mx2[3][3], double result[3][3]);
    void m33MultV3(double mx[3][3], double v[3], double result[3]);
    void m33tMultV3(double mx[3][3], double v[3], double result[3]);
    double m33Trace(double mx[3][3]);
    double m33Determinant(double mx[3][3]);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param mx1 Input array.
     * @param mx2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int m33IsEqual(double mx1[3][3], double mx2[3][3], double accuracy);
    /**
     * @brief Compare every element with zero using an absolute tolerance.
     * @param mx Input array.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int m33IsZero(double mx[3][3], double accuracy);
    void m33Print(FILE* pfile, const char* name, double mx[3][3]);
    /**
     * @brief Invert a fixed-size square matrix, allowing overlapping input and output.
     * @param mx Input matrix.
     * @param result Output inverse; zeroed on failure and allowed to overlap mx.
     * @return Zero on success, one on failure.
     * @note The absolute determinant must exceed both DB0_EPS and a roundoff bound:
     * DBL_EPSILON times the matrix dimension squared times the sum of absolute determinant
     * expansion terms. This rejects cancellation at roundoff while retaining graded diagonal
     * matrices. Numerically singular matrices and nonfinite input, determinant, roundoff bound,
     * or inverse entries are logged as failures. The direct inverse formula is retained.
     */
    int m33Inverse(double mx[3][3], double result[3][3]);
    /**
     * @brief Compute the three singular values in descending order.
     * @param mx Input matrix, preserved unless result overlaps its storage.
     * @param result Three nonnegative singular values, in the same units as mx.
     * @note Non-finite input produces three NaNs. The output may alias a row of mx.
     */
    void m33SingularValues(double mx[3][3], double result[3]);
    /**
     * @brief Compute the real parts of the three eigenvalues in descending order.
     * @param mx Finite input 3x3 matrix, preserved unless result overlaps its storage.
     * @param result Three real parts, in the same units as mx; may overlap mx.
     * @note Exactly symmetric matrices use Jacobi rotations to retain accuracy
     * near repeated eigenvalues. Other matrices use their characteristic polynomial.
     * A complex-conjugate pair contributes its repeated real part; imaginary parts
     * are not returned. Symmetry is checked exactly, without a tolerance.
     */
    void m33EigenValues(double mx[3][3], double result[3]);
    /**
     * @brief Compute the condition number from the largest and smallest singular values.
     * @param mx Input matrix, preserved by this function.
     * @return The largest singular value divided by the smallest (dimensionless).
     * @note A zero smallest singular value gives infinity when the largest is nonzero.
     * The zero matrix and non-finite input give NaN.
     */
    double m33ConditionNumber(double mx[3][3]);
    void m33PrintScreen(const char* name, double mx[3][3]);

    /* 4x4 matrices */
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
                double m[4][4]);
    /**
     * @brief Copy a 4x4 matrix.
     * @param mx Input matrix.
     * @param result Output matrix, which may overlap mx.
     */
    void m44Copy(double mx[4][4], double result[4][4]);
    void m44SetZero(double result[4][4]);
    void m44MultV4(double mx[4][4], double v[4], double result[4]);
    double m44Determinant(double mx[4][4]);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param mx1 Input array.
     * @param mx2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int m44IsEqual(double mx1[4][4], double mx2[4][4], double accuracy);
    /**
     * @brief Invert a fixed-size square matrix, allowing overlapping input and output.
     * @param mx Input matrix.
     * @param result Output inverse; zeroed on failure and allowed to overlap mx.
     * @return Zero on success, one on failure.
     * @note The absolute determinant must exceed both DB0_EPS and a roundoff bound:
     * DBL_EPSILON times the matrix dimension squared times the sum of absolute determinant
     * expansion terms. This rejects cancellation at roundoff while retaining graded diagonal
     * matrices. Numerically singular matrices and nonfinite input, determinant, roundoff bound,
     * or inverse entries are logged as failures. The direct inverse formula is retained.
     */
    int m44Inverse(double mx[4][4], double result[4][4]);

    /* 6x6 matrices */
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
                double m[6][6]);
    /**
     * @brief Copy a 6x6 matrix.
     * @param mx Input matrix.
     * @param result Output matrix, which may overlap mx.
     */
    void m66Copy(double mx[6][6], double result[6][6]);
    void m66SetZero(double result[6][6]);
    void m66SetIdentity(double result[6][6]);
    void m66Transpose(double mx[6][6], double result[6][6]);
    /**
     * @brief Extract a 3x3 block from a 6x6 matrix.
     * @param row Block row index, either 0 or 1.
     * @param col Block column index, either 0 or 1.
     * @param m Input 6x6 matrix.
     * @param mij Output packed 3x3 matrix; may overlap m.
     * @note Invalid indices are logged; no input or output memory is accessed.
     */
    void m66Get33Matrix(size_t row, size_t col, double m[6][6], double mij[3][3]);
    /**
     * @brief Insert a 3x3 block into a 6x6 matrix, preserving entries outside the selected block.
     * @param row Block row index, either 0 or 1.
     * @param col Block column index, either 0 or 1.
     * @param mij Input packed 3x3 matrix; may overlap m.
     * @param m Output 6x6 matrix.
     * @note Invalid indices are logged; no input or output memory is accessed.
     */
    void m66Set33Matrix(size_t row, size_t col, double mij[3][3], double m[6][6]);
    /**
     * @brief Scale every element.
     * @param scaleFactor Multiplier.
     * @param mx Input array.
     * @param result Output of the same shape; may overlap the input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void m66Scale(double scaleFactor, double mx[6][6], double result[6][6]);
    /**
     * @brief Add corresponding elements.
     * @param mx1 First input.
     * @param mx2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void m66Add(double mx1[6][6], double mx2[6][6], double result[6][6]);
    /**
     * @brief Subtract corresponding elements.
     * @param mx1 First input.
     * @param mx2 Second input.
     * @param result Output of the same shape; may overlap either input.
     * @note Uses fixed local storage; no heap allocation is required.
     */
    void m66Subtract(double mx1[6][6], double mx2[6][6], double result[6][6]);
    void m66MultM66(double mx1[6][6], double mx2[6][6], double result[6][6]);
    void m66tMultM66(double mx1[6][6], double mx2[6][6], double result[6][6]);
    void m66MultM66t(double mx1[6][6], double mx2[6][6], double result[6][6]);
    void m66MultV6(double mx[6][6], double v[6], double result[6]);
    void m66tMultV6(double mx[6][6], double v[6], double result[6]);
    /**
     * @brief Compare corresponding elements using an absolute tolerance.
     * @param mx1 Input array.
     * @param mx2 Second input.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int m66IsEqual(double mx1[6][6], double mx2[6][6], double accuracy);
    /**
     * @brief Compare every element with zero using an absolute tolerance.
     * @param mx Input array.
     * @param accuracy Finite, nonnegative absolute tolerance, in the same units as the input.
     * @return One if every absolute difference is at most accuracy, otherwise zero.
     * @note NaN and infinity are rejected in the input or tolerance.
     */
    int m66IsZero(double mx[6][6], double accuracy);

    /* 9x9 matrices */
    void m99SetZero(double result[9][9]);

    /* additional routines */
    /**
     * @brief Compute real parts of the roots of x^3 + a[2]*x^2 + a[1]*x + a[0] = 0.
     * @param a Three finite coefficients in ascending order, excluding the monic leading coefficient.
     * @param result Three real parts; may overlap a. The output is not sorted.
     * @note For one real root and a complex-conjugate pair, result[0] is the real root
     * and result[1] and result[2] are the pair's real part. Imaginary parts are not returned.
     * All three outputs are roots when all roots are real, including repeated roots.
     */
    void cubicRoots(double a[3], double result[3]);

    double safeAcos(double x);
    double safeAsin(double x);
    /**
     * @brief Take a square root after clamping negative inputs to zero.
     * @param x Input value.
     * @return Zero for negative input, otherwise sqrt(x), preserving NaN and signed zero.
     * @note Finite negative inputs and quiet NaNs do not raise the invalid floating-point exception.
     */
    double safeSqrt(double x);

#ifdef __cplusplus
}
#endif

#endif
