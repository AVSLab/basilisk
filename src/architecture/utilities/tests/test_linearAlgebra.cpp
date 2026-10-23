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
#include <cfenv>
#include <cmath>
#include <gtest/gtest.h>

namespace {
void checkM66BlockOverlap(bool extract)
{
    const size_t matrixStart = 10;
    const size_t storageSize = matrixStart + 36 + 10;
    double original[storageSize];
    for (size_t i = 0; i < storageSize; i++) {
        original[i] = static_cast<double>(i) - 20.0;
    }

    for (size_t blockRow = 0; blockRow < 2; blockRow++) {
        for (size_t blockCol = 0; blockCol < 2; blockCol++) {
            // Include partial overlap at either end, every internal offset, and adjacent disjoint storage.
            for (size_t blockStart = 1; blockStart + 9 < storageSize; blockStart++) {
                SCOPED_TRACE(blockRow);
                SCOPED_TRACE(blockCol);
                SCOPED_TRACE(blockStart);
                double storage[storageSize];
                double expected[storageSize];
                for (size_t i = 0; i < storageSize; i++) {
                    storage[i] = original[i];
                    expected[i] = original[i];
                }
                for (size_t row = 0; row < 3; row++) {
                    for (size_t col = 0; col < 3; col++) {
                        const size_t matrixIndex = matrixStart + (3 * blockRow + row) * 6 + 3 * blockCol + col;
                        const size_t blockIndex = blockStart + row * 3 + col;
                        if (extract) {
                            expected[blockIndex] = original[matrixIndex];
                        } else {
                            expected[matrixIndex] = original[blockIndex];
                        }
                    }
                }

                auto matrix = reinterpret_cast<double (*)[6]>(storage + matrixStart);
                auto block = reinterpret_cast<double (*)[3]>(storage + blockStart);
                if (extract) {
                    m66Get33Matrix(blockRow, blockCol, matrix, block);
                } else {
                    m66Set33Matrix(blockRow, blockCol, block, matrix);
                }
                for (size_t i = 0; i < storageSize; i++) {
                    EXPECT_DOUBLE_EQ(storage[i], expected[i]);
                }
            }
        }
    }
}

template<size_t dim>
void checkDiagOverlap()
{
    const size_t outputStart = dim + 1;
    const size_t storageSize = dim * dim + 2 * dim + 2;
    double original[storageSize];
    double storage[storageSize];
    for (size_t i = 0; i < storageSize; i++) {
        original[i] = static_cast<double>(i) - static_cast<double>(dim + 2);
    }
    for (size_t inputStart = 1; inputStart < outputStart + dim * dim; inputStart++) {
        SCOPED_TRACE(dim);
        SCOPED_TRACE(inputStart);
        for (size_t i = 0; i < storageSize; i++) {
            storage[i] = original[i];
        }
        mDiag(storage + inputStart, dim, storage + outputStart);
        for (size_t i = 0; i < storageSize; i++) {
            double expected = original[i];
            if (i >= outputStart && i < outputStart + dim * dim) {
                const size_t row = (i - outputStart) / dim;
                const size_t col = (i - outputStart) % dim;
                expected = row == col ? original[inputStart + row] : 0.0;
            }
            EXPECT_DOUBLE_EQ(storage[i], expected);
        }
    }
}

template<size_t dim, typename Copy>
void checkVectorCopy(Copy copy)
{
    const size_t storageSize = 2 * dim + 2;
    double original[storageSize];
    double input[storageSize];
    double output[storageSize];
    for (size_t i = 0; i < storageSize; i++) {
        original[i] = static_cast<double>(i) - static_cast<double>(dim);
        input[i] = original[i];
        output[i] = 1234.0;
    }

    copy(input + 1, output + 1);
    for (size_t i = 0; i < storageSize; i++) {
        EXPECT_DOUBLE_EQ(input[i], original[i]);
        EXPECT_DOUBLE_EQ(output[i], i >= 1 && i <= dim ? original[i] : 1234.0);
    }

    /* Exercise self-copy and every partial overlap in both directions. */
    for (size_t inputStart = 1; inputStart <= dim; inputStart++) {
        for (size_t outputStart = 1; outputStart <= dim; outputStart++) {
            SCOPED_TRACE(inputStart);
            SCOPED_TRACE(outputStart);
            for (size_t i = 0; i < storageSize; i++) {
                input[i] = original[i];
            }
            copy(input + inputStart, input + outputStart);
            for (size_t i = 0; i < storageSize; i++) {
                const double expected =
                    i >= outputStart && i < outputStart + dim ? original[inputStart + i - outputStart] : original[i];
                EXPECT_DOUBLE_EQ(input[i], expected);
            }
        }
    }
}

template<size_t dim, typename Copy>
void checkMatrixCopy(Copy copy)
{
    const size_t storageRows = 2 * dim + 2;
    double original[storageRows][dim];
    double input[storageRows][dim];
    double output[storageRows][dim];
    for (size_t row = 0; row < storageRows; row++) {
        for (size_t col = 0; col < dim; col++) {
            original[row][col] = static_cast<double>(row * dim + col) - static_cast<double>(dim * dim);
            input[row][col] = original[row][col];
            output[row][col] = 1234.0;
        }
    }

    copy(input + 1, output + 1);
    for (size_t row = 0; row < storageRows; row++) {
        for (size_t col = 0; col < dim; col++) {
            EXPECT_DOUBLE_EQ(input[row][col], original[row][col]);
            EXPECT_DOUBLE_EQ(output[row][col], row >= 1 && row <= dim ? original[row][col] : 1234.0);
        }
    }

    /* Keep the declared row type while testing each overlapping row displacement. */
    for (size_t inputStart = 1; inputStart <= dim; inputStart++) {
        for (size_t outputStart = 1; outputStart <= dim; outputStart++) {
            SCOPED_TRACE(inputStart);
            SCOPED_TRACE(outputStart);
            for (size_t row = 0; row < storageRows; row++) {
                for (size_t col = 0; col < dim; col++) {
                    input[row][col] = original[row][col];
                }
            }
            copy(input + inputStart, input + outputStart);
            for (size_t row = 0; row < storageRows; row++) {
                for (size_t col = 0; col < dim; col++) {
                    const double expected = row >= outputStart && row < outputStart + dim
                                                ? original[inputStart + row - outputStart][col]
                                                : original[row][col];
                    EXPECT_DOUBLE_EQ(input[row][col], expected);
                }
            }
        }
    }
}

struct PerpendicularCase
{
    double input[3];
    double direction[3];
};

void checkPerpendicular(const PerpendicularCase& test)
{
    SCOPED_TRACE(::testing::PrintToString(test.input));
    const double accuracy = 1e-14;
    double input[3];
    double inPlace[3];
    double result[3] = {};
    for (size_t i = 0; i < 3; i++) {
        input[i] = test.input[i];
        inPlace[i] = test.input[i];
    }

    v3Perpendicular(input, result);
    v3Perpendicular(inPlace, inPlace);

    const double directionNorm = std::hypot(std::hypot(test.direction[0], test.direction[1]), test.direction[2]);
    double dot = 0.0;
    for (size_t i = 0; i < 3; i++) {
        EXPECT_NEAR(result[i], test.direction[i] / directionNorm, accuracy);
        EXPECT_DOUBLE_EQ(inPlace[i], result[i]);
        EXPECT_DOUBLE_EQ(input[i], test.input[i]);
        dot += test.input[i] * inPlace[i];
    }
    EXPECT_NEAR(dot, 0.0, accuracy);
    EXPECT_NEAR(std::hypot(std::hypot(inPlace[0], inPlace[1]), inPlace[2]), 1.0, accuracy);
}

void expectOuterProduct(const double actual[3][3], const double expected[3][3])
{
    for (int row = 0; row < 3; row++) {
        for (int column = 0; column < 3; column++) {
            EXPECT_DOUBLE_EQ(actual[row][column], expected[row][column]);
        }
    }
}

void expectOuterProduct(const double actual[6][6], const double expected[6][6])
{
    for (int row = 0; row < 6; row++) {
        for (int column = 0; column < 6; column++) {
            EXPECT_DOUBLE_EQ(actual[row][column], expected[row][column]);
        }
    }
}

void checkTilde(const double input[3], const double expected[3][3])
{
    double vector[3] = { input[0], input[1], input[2] };
    double separate[3][3];
    v3Tilde(vector, separate);
    for (int row = 0; row < 3; row++) {
        EXPECT_DOUBLE_EQ(vector[row], input[row]);
        for (int column = 0; column < 3; column++) {
            EXPECT_DOUBLE_EQ(separate[row][column], expected[row][column]);
        }
    }

    for (int inputRow = 0; inputRow < 3; inputRow++) {
        SCOPED_TRACE(inputRow);
        double matrix[3][3];
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                matrix[row][column] = 1234.0;
            }
        }
        for (int column = 0; column < 3; column++) {
            matrix[inputRow][column] = input[column];
        }
        v3Tilde(matrix[inputRow], matrix);
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                EXPECT_DOUBLE_EQ(matrix[row][column], expected[row][column]);
            }
        }
    }
}
}

/** @brief Extract each 3x3 block into separate storage without changing the input matrix. */
TEST(LinearAlgebra, m66Get33MatrixSeparateStorage)
{
    double original[6][6];
    for (size_t row = 0; row < 6; row++) {
        for (size_t col = 0; col < 6; col++) {
            original[row][col] = static_cast<double>(row * 6 + col) - 18.0;
        }
    }
    for (size_t blockRow = 0; blockRow < 2; blockRow++) {
        for (size_t blockCol = 0; blockCol < 2; blockCol++) {
            SCOPED_TRACE(blockRow);
            SCOPED_TRACE(blockCol);
            double matrix[6][6];
            double block[3][3] = {};
            m66Copy(original, matrix);
            m66Get33Matrix(blockRow, blockCol, matrix, block);
            for (size_t row = 0; row < 3; row++) {
                for (size_t col = 0; col < 3; col++) {
                    EXPECT_DOUBLE_EQ(block[row][col], original[3 * blockRow + row][3 * blockCol + col]);
                }
            }
            for (size_t row = 0; row < 6; row++) {
                for (size_t col = 0; col < 6; col++) {
                    EXPECT_DOUBLE_EQ(matrix[row][col], original[row][col]);
                }
            }
        }
    }
}

/** @brief Insert each 3x3 block from separate storage, preserving the input and other matrix entries. */
TEST(LinearAlgebra, m66Set33MatrixSeparateStorage)
{
    const double original[3][3] = { { -4.0, -3.0, -2.0 }, { -1.0, 0.0, 1.0 }, { 2.0, 3.0, 4.0 } };
    for (size_t blockRow = 0; blockRow < 2; blockRow++) {
        for (size_t blockCol = 0; blockCol < 2; blockCol++) {
            SCOPED_TRACE(blockRow);
            SCOPED_TRACE(blockCol);
            double matrix[6][6];
            double block[3][3];
            for (size_t row = 0; row < 6; row++) {
                for (size_t col = 0; col < 6; col++) {
                    matrix[row][col] = static_cast<double>(row * 6 + col) + 10.0;
                }
            }
            for (size_t row = 0; row < 3; row++) {
                for (size_t col = 0; col < 3; col++) {
                    block[row][col] = original[row][col];
                }
            }
            m66Set33Matrix(blockRow, blockCol, block, matrix);
            for (size_t row = 0; row < 6; row++) {
                for (size_t col = 0; col < 6; col++) {
                    const double expected = row / 3 == blockRow && col / 3 == blockCol
                                                ? original[row % 3][col % 3]
                                                : static_cast<double>(row * 6 + col) + 10.0;
                    EXPECT_DOUBLE_EQ(matrix[row][col], expected);
                }
            }
            for (size_t row = 0; row < 3; row++) {
                for (size_t col = 0; col < 3; col++) {
                    EXPECT_DOUBLE_EQ(block[row][col], original[row][col]);
                }
            }
        }
    }
}

/** @brief Extract all block positions safely into overlapping storage and preserve neighboring entries. */
TEST(LinearAlgebra, m66Get33MatrixOverlappingStorage)
{
    checkM66BlockOverlap(true);
}

/** @brief Insert all block positions safely from overlapping storage and preserve neighboring entries. */
TEST(LinearAlgebra, m66Set33MatrixOverlappingStorage)
{
    checkM66BlockOverlap(false);
}

/** @brief Preserve separate input and write all diagonal and off-diagonal entries. */
TEST(LinearAlgebra, mDiagSeparateOutput)
{
    double input[3] = { -2.0, 0.0, 4.0 };
    const double original[3] = { -2.0, 0.0, 4.0 };
    const double expected[9] = { -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0 };
    double output[11];
    for (double& value : output) {
        value = 1234.0;
    }
    mDiag(input, 3, output + 1);
    for (size_t i = 0; i < 3; i++) {
        EXPECT_DOUBLE_EQ(input[i], original[i]);
    }
    for (size_t i = 0; i < 9; i++) {
        EXPECT_DOUBLE_EQ(output[i + 1], expected[i]);
    }
    EXPECT_DOUBLE_EQ(output[0], 1234.0);
    EXPECT_DOUBLE_EQ(output[10], 1234.0);
}

/** @brief Form a diagonal matrix when the input occupies its first row. */
TEST(LinearAlgebra, mDiagInputInFirstRow)
{
    double matrix[3][3] = { { 2.0, 3.0, 4.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
    const double expected[3][3] = { { 2.0, 0.0, 0.0 }, { 0.0, 3.0, 0.0 }, { 0.0, 0.0, 4.0 } };
    mDiag(matrix[0], 3, matrix);
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 3; col++) {
            EXPECT_DOUBLE_EQ(matrix[row][col], expected[row][col]);
        }
    }
}

/** @brief Handle input anywhere within output or partially overlapping either end, preserving neighbors. */
TEST(LinearAlgebra, mDiagOverlappingStorage)
{
    checkDiagOverlap<2>();
    checkDiagOverlap<3>();
    checkDiagOverlap<6>();
}

/** @brief Preserve scalar values with separate output and exact in-place use. */
TEST(LinearAlgebra, mDiagSingleElement)
{
    const double values[3] = { -3.0, 0.0, 2.0 };
    for (double value : values) {
        SCOPED_TRACE(value);
        double input = value;
        double output[3] = { 1234.0, 0.0, 5678.0 };
        mDiag(&input, 1, output + 1);
        mDiag(&input, 1, &input);
        EXPECT_DOUBLE_EQ(input, value);
        EXPECT_DOUBLE_EQ(output[0], 1234.0);
        EXPECT_DOUBLE_EQ(output[1], value);
        EXPECT_DOUBLE_EQ(output[2], 5678.0);
    }
}

/** @brief Leave storage untouched and accept null pointers when the dimension is zero. */
TEST(LinearAlgebra, mDiagEmpty)
{
    double input = 1234.0;
    double output = 5678.0;
    mDiag(&input, 0, &output);
    mDiag(&input, 0, &input);
    mDiag(nullptr, 0, nullptr);
    EXPECT_DOUBLE_EQ(input, 1234.0);
    EXPECT_DOUBLE_EQ(output, 5678.0);
}

/** @brief Copy generic vectors with separate storage, self-copy, and overlapping storage. */
TEST(LinearAlgebra, vCopyBuffers)
{
    checkVectorCopy<1>([](double* input, double* output) { vCopy(input, 1, output); });
    checkVectorCopy<7>([](double* input, double* output) { vCopy(input, 7, output); });
}

/** @brief Leave storage untouched and accept null pointers for an empty vector. */
TEST(LinearAlgebra, vCopyEmpty)
{
    double input = 1234.0;
    double output = 5678.0;
    vCopy(&input, 0, &output);
    vCopy(&input, 0, &input);
    vCopy(nullptr, 0, nullptr);
    EXPECT_DOUBLE_EQ(input, 1234.0);
    EXPECT_DOUBLE_EQ(output, 5678.0);
}

/** @brief Copy two-component vectors without corrupting overlapping storage. */
TEST(LinearAlgebra, v2CopyBuffers)
{
    checkVectorCopy<2>(v2Copy);
}

/** @brief Copy three-component vectors without corrupting overlapping storage. */
TEST(LinearAlgebra, v3CopyBuffers)
{
    checkVectorCopy<3>(v3Copy);
}

/** @brief Copy four-component vectors without corrupting overlapping storage. */
TEST(LinearAlgebra, v4CopyBuffers)
{
    checkVectorCopy<4>(v4Copy);
}

/** @brief Copy six-component vectors without corrupting overlapping storage. */
TEST(LinearAlgebra, v6CopyBuffers)
{
    checkVectorCopy<6>(v6Copy);
}

/** @brief Copy rectangular, row, column, and scalar matrices with arbitrary element overlap. */
TEST(LinearAlgebra, mCopyBuffers)
{
    checkVectorCopy<6>([](double* input, double* output) { mCopy(input, 2, 3, output); });
    checkVectorCopy<6>([](double* input, double* output) { mCopy(input, 3, 2, output); });
    checkVectorCopy<4>([](double* input, double* output) { mCopy(input, 1, 4, output); });
    checkVectorCopy<4>([](double* input, double* output) { mCopy(input, 4, 1, output); });
    checkVectorCopy<1>([](double* input, double* output) { mCopy(input, 1, 1, output); });
}

/** @brief Leave storage untouched and accept null pointers when either matrix dimension is zero. */
TEST(LinearAlgebra, mCopyEmpty)
{
    const size_t shapes[3][2] = { { 0, 3 }, { 3, 0 }, { 0, 0 } };
    for (const auto& shape : shapes) {
        SCOPED_TRACE(::testing::PrintToString(shape));
        double input = 1234.0;
        double output = 5678.0;
        mCopy(&input, shape[0], shape[1], &output);
        mCopy(&input, shape[0], shape[1], &input);
        mCopy(nullptr, shape[0], shape[1], nullptr);
        EXPECT_DOUBLE_EQ(input, 1234.0);
        EXPECT_DOUBLE_EQ(output, 5678.0);
    }
}

/** @brief Copy 2x2 matrices without corrupting overlapping rows. */
TEST(LinearAlgebra, m22CopyBuffers)
{
    checkMatrixCopy<2>(m22Copy);
}

/** @brief Copy 3x3 matrices without corrupting overlapping rows. */
TEST(LinearAlgebra, m33CopyBuffers)
{
    checkMatrixCopy<3>(m33Copy);
}

/** @brief Copy 4x4 matrices without corrupting overlapping rows. */
TEST(LinearAlgebra, m44CopyBuffers)
{
    checkMatrixCopy<4>(m44Copy);
}

/** @brief Copy 6x6 matrices without corrupting overlapping rows. */
TEST(LinearAlgebra, m66CopyBuffers)
{
    checkMatrixCopy<6>(m66Copy);
}

/** @brief Sort mixed values and duplicates without changing separate input or surrounding output. */
TEST(LinearAlgebra, vSortSeparateOutput)
{
    double input[6] = { 3.0, -1.0, 2.0, 0.0, -1.0, 3.0 };
    const double original[6] = { 3.0, -1.0, 2.0, 0.0, -1.0, 3.0 };
    const double expected[6] = { -1.0, -1.0, 0.0, 2.0, 3.0, 3.0 };
    double output[8] = { 1234.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5678.0 };
    vSort(input, output + 1, 6);
    for (size_t i = 0; i < 6; i++) {
        EXPECT_DOUBLE_EQ(output[i + 1], expected[i]);
        EXPECT_DOUBLE_EQ(input[i], original[i]);
    }
    EXPECT_DOUBLE_EQ(output[0], 1234.0);
    EXPECT_DOUBLE_EQ(output[7], 5678.0);
}

/** @brief Sort mixed values and duplicates with identical input and output pointers. */
TEST(LinearAlgebra, vSortInPlace)
{
    double values[6] = { 3.0, -1.0, 2.0, 0.0, -1.0, 3.0 };
    const double expected[6] = { -1.0, -1.0, 0.0, 2.0, 3.0, 3.0 };
    vSort(values, values, 6);
    for (size_t i = 0; i < 6; i++) {
        EXPECT_DOUBLE_EQ(values[i], expected[i]);
    }
}

/** @brief Preserve all input values when overlapping output starts after input. */
TEST(LinearAlgebra, vSortOutputAfterInput)
{
    double values[9] = { 1234.0, 3.0, -1.0, 2.0, 0.0, -1.0, 3.0, 0.0, 5678.0 };
    const double expected[6] = { -1.0, -1.0, 0.0, 2.0, 3.0, 3.0 };
    vSort(values + 1, values + 2, 6);
    for (size_t i = 0; i < 6; i++) {
        EXPECT_DOUBLE_EQ(values[i + 2], expected[i]);
    }
    EXPECT_DOUBLE_EQ(values[0], 1234.0);
    EXPECT_DOUBLE_EQ(values[1], 3.0);
    EXPECT_DOUBLE_EQ(values[8], 5678.0);
}

/** @brief Preserve all input values when overlapping output starts before input. */
TEST(LinearAlgebra, vSortOutputBeforeInput)
{
    double values[9] = { 1234.0, 0.0, 3.0, -1.0, 2.0, 0.0, -1.0, 3.0, 5678.0 };
    const double expected[6] = { -1.0, -1.0, 0.0, 2.0, 3.0, 3.0 };
    vSort(values + 2, values + 1, 6);
    for (size_t i = 0; i < 6; i++) {
        EXPECT_DOUBLE_EQ(values[i + 1], expected[i]);
    }
    EXPECT_DOUBLE_EQ(values[0], 1234.0);
    EXPECT_DOUBLE_EQ(values[7], 3.0);
    EXPECT_DOUBLE_EQ(values[8], 5678.0);
}

/** @brief Handle already sorted, reverse sorted, and equal values. */
TEST(LinearAlgebra, vSortOrderedInputs)
{
    double inputs[3][4] = { { -2.0, 0.0, 1.0, 3.0 }, { 3.0, 1.0, 0.0, -2.0 }, { 2.0, 2.0, 2.0, 2.0 } };
    const double expected[3][4] = { { -2.0, 0.0, 1.0, 3.0 }, { -2.0, 0.0, 1.0, 3.0 }, { 2.0, 2.0, 2.0, 2.0 } };
    for (size_t test = 0; test < 3; test++) {
        SCOPED_TRACE(test);
        vSort(inputs[test], inputs[test], 4);
        for (size_t i = 0; i < 4; i++) {
            EXPECT_DOUBLE_EQ(inputs[test][i], expected[test][i]);
        }
    }
}

/** @brief Leave valid input and output storage untouched when the vector is empty. */
TEST(LinearAlgebra, vSortEmpty)
{
    double input = 1234.0;
    double output = 5678.0;
    vSort(&input, &output, 0);
    vSort(&input, &input, 0);
    EXPECT_DOUBLE_EQ(input, 1234.0);
    EXPECT_DOUBLE_EQ(output, 5678.0);
}

/** @brief Copy or retain one element without touching adjacent storage. */
TEST(LinearAlgebra, vSortSingleElement)
{
    double input = -2.0;
    double output[3] = { 1234.0, 0.0, 5678.0 };
    vSort(&input, output + 1, 1);
    vSort(output + 1, output + 1, 1);
    EXPECT_DOUBLE_EQ(input, -2.0);
    EXPECT_DOUBLE_EQ(output[0], 1234.0);
    EXPECT_DOUBLE_EQ(output[1], -2.0);
    EXPECT_DOUBLE_EQ(output[2], 5678.0);
}

/** @brief Preserve separate inputs and compute every entry, including negative and zero products. */
TEST(LinearAlgebra, v3OuterProductSeparateOutput)
{
    double left[3] = { 2.0, -3.0, 0.0 };
    double right[3] = { 4.0, 0.0, -5.0 };
    const double originalLeft[3] = { 2.0, -3.0, 0.0 };
    const double originalRight[3] = { 4.0, 0.0, -5.0 };
    const double expected[3][3] = { { 8.0, 0.0, -10.0 }, { -12.0, 0.0, 15.0 }, { 0.0, 0.0, 0.0 } };
    double result[3][3] = { { 1234.0, 1234.0, 1234.0 }, { 1234.0, 1234.0, 1234.0 }, { 1234.0, 1234.0, 1234.0 } };
    v3OuterProduct(left, right, result);
    expectOuterProduct(result, expected);
    for (int i = 0; i < 3; i++) {
        EXPECT_DOUBLE_EQ(left[i], originalLeft[i]);
        EXPECT_DOUBLE_EQ(right[i], originalRight[i]);
    }
}

/** @brief Preserve the first input when it occupies any row of the output matrix. */
TEST(LinearAlgebra, v3OuterProductFirstInputInOutput)
{
    const double expected[3][3] = { { 10.0, 12.0, 14.0 }, { 15.0, 18.0, 21.0 }, { 20.0, 24.0, 28.0 } };
    for (int inputRow = 0; inputRow < 3; inputRow++) {
        SCOPED_TRACE(inputRow);
        double right[3] = { 5.0, 6.0, 7.0 };
        double result[3][3] = {};
        v3Set(2.0, 3.0, 4.0, result[inputRow]);
        v3OuterProduct(result[inputRow], right, result);
        expectOuterProduct(result, expected);
    }
}

/** @brief Preserve the second input when it occupies any row of the output matrix. */
TEST(LinearAlgebra, v3OuterProductSecondInputInOutput)
{
    const double expected[3][3] = { { 10.0, 12.0, 14.0 }, { 15.0, 18.0, 21.0 }, { 20.0, 24.0, 28.0 } };
    for (int inputRow = 0; inputRow < 3; inputRow++) {
        SCOPED_TRACE(inputRow);
        double left[3] = { 2.0, 3.0, 4.0 };
        double result[3][3] = {};
        v3Set(5.0, 6.0, 7.0, result[inputRow]);
        v3OuterProduct(left, result[inputRow], result);
        expectOuterProduct(result, expected);
    }
}

/** @brief Preserve both inputs when they occupy distinct rows of the output matrix. */
TEST(LinearAlgebra, v3OuterProductBothInputsInOutput)
{
    const double expected[3][3] = { { 8.0, 0.0, -10.0 }, { -12.0, 0.0, 15.0 }, { 0.0, 0.0, 0.0 } };
    for (int leftRow = 0; leftRow < 3; leftRow++) {
        for (int rightRow = 0; rightRow < 3; rightRow++) {
            if (leftRow == rightRow) {
                continue;
            }
            SCOPED_TRACE(leftRow);
            SCOPED_TRACE(rightRow);
            double result[3][3] = {};
            v3Set(2.0, -3.0, 0.0, result[leftRow]);
            v3Set(4.0, 0.0, -5.0, result[rightRow]);
            v3OuterProduct(result[leftRow], result[rightRow], result);
            expectOuterProduct(result, expected);
        }
    }
}

/** @brief Accept the same vector for both inputs, with separate or overlapping output. */
TEST(LinearAlgebra, v3OuterProductSameInput)
{
    double vector[3] = { 2.0, -3.0, 0.0 };
    const double expected[3][3] = { { 4.0, -6.0, 0.0 }, { -6.0, 9.0, 0.0 }, { 0.0, 0.0, 0.0 } };
    double separate[3][3];
    v3OuterProduct(vector, vector, separate);
    expectOuterProduct(separate, expected);
    for (int inputRow = 0; inputRow < 3; inputRow++) {
        SCOPED_TRACE(inputRow);
        double result[3][3] = {};
        v3Set(2.0, -3.0, 0.0, result[inputRow]);
        v3OuterProduct(result[inputRow], result[inputRow], result);
        expectOuterProduct(result, expected);
    }
}

/** @brief Preserve separate six-component inputs and write negative and zero products correctly. */
TEST(LinearAlgebra, v6OuterProductSeparateOutput)
{
    double left[6] = { 2.0, -3.0, 0.0, 4.0, -1.0, 5.0 };
    double right[6] = { -2.0, 0.0, 3.0, 1.0, -4.0, 2.0 };
    const double originalLeft[6] = { 2.0, -3.0, 0.0, 4.0, -1.0, 5.0 };
    const double originalRight[6] = { -2.0, 0.0, 3.0, 1.0, -4.0, 2.0 };
    const double expected[6][6] = { { -4.0, 0.0, 6.0, 2.0, -8.0, 4.0 },  { 6.0, 0.0, -9.0, -3.0, 12.0, -6.0 },
                                    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },    { -8.0, 0.0, 12.0, 4.0, -16.0, 8.0 },
                                    { 2.0, 0.0, -3.0, -1.0, 4.0, -2.0 }, { -10.0, 0.0, 15.0, 5.0, -20.0, 10.0 } };
    double result[6][6];
    for (int row = 0; row < 6; row++) {
        for (int column = 0; column < 6; column++) {
            result[row][column] = 1234.0;
        }
    }
    v6OuterProduct(left, right, result);
    expectOuterProduct(result, expected);
    for (int i = 0; i < 6; i++) {
        EXPECT_DOUBLE_EQ(left[i], originalLeft[i]);
        EXPECT_DOUBLE_EQ(right[i], originalRight[i]);
    }
}

/** @brief Preserve the first six-component input in each output row. */
TEST(LinearAlgebra, v6OuterProductFirstInputInOutput)
{
    const double expected[6][6] = { { 6.0, 8.0, 10.0, 12.0, 14.0, 16.0 },   { 9.0, 12.0, 15.0, 18.0, 21.0, 24.0 },
                                    { 12.0, 16.0, 20.0, 24.0, 28.0, 32.0 }, { 15.0, 20.0, 25.0, 30.0, 35.0, 40.0 },
                                    { 18.0, 24.0, 30.0, 36.0, 42.0, 48.0 }, { 21.0, 28.0, 35.0, 42.0, 49.0, 56.0 } };
    for (int inputRow = 0; inputRow < 6; inputRow++) {
        SCOPED_TRACE(inputRow);
        double right[6] = { 3.0, 4.0, 5.0, 6.0, 7.0, 8.0 };
        double result[6][6] = {};
        v6Set(2.0, 3.0, 4.0, 5.0, 6.0, 7.0, result[inputRow]);
        v6OuterProduct(result[inputRow], right, result);
        expectOuterProduct(result, expected);
    }
}

/** @brief Preserve the second six-component input in each output row. */
TEST(LinearAlgebra, v6OuterProductSecondInputInOutput)
{
    const double expected[6][6] = { { 6.0, 8.0, 10.0, 12.0, 14.0, 16.0 },   { 9.0, 12.0, 15.0, 18.0, 21.0, 24.0 },
                                    { 12.0, 16.0, 20.0, 24.0, 28.0, 32.0 }, { 15.0, 20.0, 25.0, 30.0, 35.0, 40.0 },
                                    { 18.0, 24.0, 30.0, 36.0, 42.0, 48.0 }, { 21.0, 28.0, 35.0, 42.0, 49.0, 56.0 } };
    for (int inputRow = 0; inputRow < 6; inputRow++) {
        SCOPED_TRACE(inputRow);
        double left[6] = { 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 };
        double result[6][6] = {};
        v6Set(3.0, 4.0, 5.0, 6.0, 7.0, 8.0, result[inputRow]);
        v6OuterProduct(left, result[inputRow], result);
        expectOuterProduct(result, expected);
    }
}

/** @brief Preserve both six-component inputs in every pair of distinct output rows. */
TEST(LinearAlgebra, v6OuterProductBothInputsInOutput)
{
    const double expected[6][6] = { { -4.0, 0.0, 6.0, 2.0, -8.0, 4.0 },  { 6.0, 0.0, -9.0, -3.0, 12.0, -6.0 },
                                    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },    { -8.0, 0.0, 12.0, 4.0, -16.0, 8.0 },
                                    { 2.0, 0.0, -3.0, -1.0, 4.0, -2.0 }, { -10.0, 0.0, 15.0, 5.0, -20.0, 10.0 } };
    for (int leftRow = 0; leftRow < 6; leftRow++) {
        for (int rightRow = 0; rightRow < 6; rightRow++) {
            if (leftRow == rightRow) {
                continue;
            }
            SCOPED_TRACE(leftRow);
            SCOPED_TRACE(rightRow);
            double result[6][6] = {};
            v6Set(2.0, -3.0, 0.0, 4.0, -1.0, 5.0, result[leftRow]);
            v6Set(-2.0, 0.0, 3.0, 1.0, -4.0, 2.0, result[rightRow]);
            v6OuterProduct(result[leftRow], result[rightRow], result);
            expectOuterProduct(result, expected);
        }
    }
}

/** @brief Accept one six-component vector for both inputs, with separate or overlapping output. */
TEST(LinearAlgebra, v6OuterProductSameInput)
{
    double vector[6] = { 2.0, -3.0, 0.0, 4.0, -1.0, 5.0 };
    const double expected[6][6] = { { 4.0, -6.0, 0.0, 8.0, -2.0, 10.0 }, { -6.0, 9.0, 0.0, -12.0, 3.0, -15.0 },
                                    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },    { 8.0, -12.0, 0.0, 16.0, -4.0, 20.0 },
                                    { -2.0, 3.0, 0.0, -4.0, 1.0, -5.0 }, { 10.0, -15.0, 0.0, 20.0, -5.0, 25.0 } };
    double separate[6][6];
    v6OuterProduct(vector, vector, separate);
    expectOuterProduct(separate, expected);
    for (int inputRow = 0; inputRow < 6; inputRow++) {
        SCOPED_TRACE(inputRow);
        double result[6][6] = {};
        v6Set(2.0, -3.0, 0.0, 4.0, -1.0, 5.0, result[inputRow]);
        v6OuterProduct(result[inputRow], result[inputRow], result);
        expectOuterProduct(result, expected);
    }
}

/** @brief Form the cross-product matrix with separate output or input in any output row. */
TEST(LinearAlgebra, v3TildePositiveComponents)
{
    const double input[3] = { 1.0, 2.0, 3.0 };
    const double expected[3][3] = { { 0.0, -3.0, 2.0 }, { 3.0, 0.0, -1.0 }, { -2.0, 1.0, 0.0 } };
    checkTilde(input, expected);
}

/** @brief Preserve signs and zero components when the output overlaps the input. */
TEST(LinearAlgebra, v3TildeMixedComponents)
{
    const double input[3] = { -2.0, 0.0, 3.0 };
    const double expected[3][3] = { { 0.0, -3.0, 0.0 }, { 3.0, 0.0, 2.0 }, { 0.0, -2.0, 0.0 } };
    checkTilde(input, expected);
}

/** @brief Overwrite every matrix entry for a zero vector, including overlapping storage. */
TEST(LinearAlgebra, v3TildeZeroVector)
{
    const double input[3] = {};
    const double expected[3][3] = {};
    checkTilde(input, expected);
}

/** @brief Return zero for every signed-zero input without raising floating-point exceptions. */
TEST(LinearAlgebra, v3PerpendicularZeroInput)
{
    const double cases[][3] = {
        { 0.0, 0.0, 0.0 },   { -0.0, 0.0, 0.0 },  { 0.0, -0.0, 0.0 },  { 0.0, 0.0, -0.0 },
        { -0.0, -0.0, 0.0 }, { -0.0, 0.0, -0.0 }, { 0.0, -0.0, -0.0 }, { -0.0, -0.0, -0.0 },
    };

    for (const auto& test : cases) {
        SCOPED_TRACE(::testing::PrintToString(test));
        for (int inPlace = 0; inPlace < 2; inPlace++) {
            SCOPED_TRACE(inPlace);
            double input[3] = { test[0], test[1], test[2] };
            double output[3] = { 1.0, 2.0, 3.0 };
            double* result = inPlace ? input : output;
            std::fenv_t environment;
            ASSERT_EQ(std::feholdexcept(&environment), 0);
            v3Perpendicular(input, result);
            const int exceptions = std::fetestexcept(FE_INVALID | FE_DIVBYZERO);
            ASSERT_EQ(std::fesetenv(&environment), 0);

            EXPECT_EQ(exceptions, 0);
            for (size_t i = 0; i < 3; i++) {
                EXPECT_DOUBLE_EQ(result[i], 0.0);
                if (!inPlace) {
                    EXPECT_DOUBLE_EQ(input[i], test[i]);
                    EXPECT_EQ(std::signbit(input[i]), std::signbit(test[i]));
                }
            }
        }
    }
}

/** @brief Preserve the first branch's perpendicular direction with separate and in-place output. */
TEST(LinearAlgebra, v3PerpendicularFirstComponent)
{
    const PerpendicularCase cases[] = {
        { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 1.0 } },
        { { -1.0, 0.0, 0.0 }, { 0.0, 1.0, 1.0 } },
        { { 2.0, 1.0, 1.0 }, { -1.0, 1.0, 1.0 } },
        { { -2.0, 1.0, -3.0 }, { -1.0, 1.0, 1.0 } },
    };
    for (const auto& test : cases) {
        checkPerpendicular(test);
    }
}

/** @brief Preserve unread input components when the second branch writes in place. */
TEST(LinearAlgebra, v3PerpendicularSecondComponent)
{
    const PerpendicularCase cases[] = {
        { { 0.0, 1.0, 0.0 }, { 1.0, 0.0, 1.0 } },          { { 0.0, -1.0, 0.0 }, { 1.0, 0.0, 1.0 } },
        { { 0.0, 2.0, 3.0 }, { 2.0, -3.0, 2.0 } },         { { 0.0, -2.0, 3.0 }, { 2.0, 3.0, 2.0 } },
        { { DB0_EPS, 1.0, 0.0 }, { 1.0, -DB0_EPS, 1.0 } },
    };
    for (const auto& test : cases) {
        checkPerpendicular(test);
    }
}

/** @brief Preserve unread input components when the third branch writes in place. */
TEST(LinearAlgebra, v3PerpendicularThirdComponent)
{
    const PerpendicularCase cases[] = {
        { { 0.0, 0.0, 1.0 }, { 1.0, 1.0, 0.0 } },
        { { 0.0, 0.0, -1.0 }, { 1.0, 1.0, 0.0 } },
        { { DB0_EPS, DB0_EPS, 1.0 }, { 1.0, 1.0, -2.0 * DB0_EPS } },
        { { -DB0_EPS, 0.0, -1.0 }, { 1.0, 1.0, -DB0_EPS } },
    };
    for (const auto& test : cases) {
        checkPerpendicular(test);
    }
}

/** @brief Use the column count as the row stride for a wide matrix. */
TEST(LinearAlgebra, vtMultMtWideMatrix)
{
    double vector[3] = { 2.0, -1.0, 3.0 };
    double matrix[2][3] = { { 1.0, -2.0, 4.0 }, { -3.0, 5.0, 2.0 } };
    double result[2] = {};

    vtMultMt(vector, matrix, 2, 3, result);

    EXPECT_DOUBLE_EQ(result[0], 16.0);
    EXPECT_DOUBLE_EQ(result[1], -5.0);
}

/** @brief Multiply a tall matrix without reading beyond its final row. */
TEST(LinearAlgebra, vtMultMtTallMatrix)
{
    double vector[2] = { 2.0, -1.0 };
    double matrix[3][2] = { { 1.0, -2.0 }, { -3.0, 5.0 }, { 4.0, 2.0 } };
    double result[3] = {};

    vtMultMt(vector, matrix, 3, 2, result);

    EXPECT_DOUBLE_EQ(result[0], 4.0);
    EXPECT_DOUBLE_EQ(result[1], -11.0);
    EXPECT_DOUBLE_EQ(result[2], 6.0);
}

/** @brief Allow a wide product to overwrite the input vector without writing past the result. */
TEST(LinearAlgebra, vtMultMtWideVectorInPlace)
{
    double vector[3] = { 2.0, -1.0, 3.0 };
    double matrix[2][3] = { { 1.0, -2.0, 4.0 }, { -3.0, 5.0, 2.0 } };

    vtMultMt(vector, matrix, 2, 3, vector);

    EXPECT_DOUBLE_EQ(vector[0], 16.0);
    EXPECT_DOUBLE_EQ(vector[1], -5.0);
    EXPECT_DOUBLE_EQ(vector[2], 3.0);
}

/** @brief Allow a tall product to overwrite a vector buffer sized for the larger result. */
TEST(LinearAlgebra, vtMultMtTallVectorInPlace)
{
    // Only the first two elements are input; the buffer must also hold three output elements.
    double vector[3] = { 2.0, -1.0, 1234.0 };
    double matrix[3][2] = { { 1.0, -2.0 }, { -3.0, 5.0 }, { 4.0, 2.0 } };

    vtMultMt(vector, matrix, 3, 2, vector);

    EXPECT_DOUBLE_EQ(vector[0], 4.0);
    EXPECT_DOUBLE_EQ(vector[1], -11.0);
    EXPECT_DOUBLE_EQ(vector[2], 6.0);
}

/** @brief Preserve unread matrix elements when a wide product overwrites the matrix. */
TEST(LinearAlgebra, vtMultMtWideMatrixInPlace)
{
    double vector[3] = { 2.0, -1.0, 3.0 };
    double matrix[2][3] = { { 1.0, -2.0, 4.0 }, { -3.0, 5.0, 2.0 } };

    vtMultMt(vector, matrix, 2, 3, matrix);

    EXPECT_DOUBLE_EQ(matrix[0][0], 16.0);
    EXPECT_DOUBLE_EQ(matrix[0][1], -5.0);
    EXPECT_DOUBLE_EQ(matrix[0][2], 4.0);
}

/** @brief Allow a tall product to overwrite matrix storage across a row boundary. */
TEST(LinearAlgebra, vtMultMtTallMatrixInPlace)
{
    double vector[2] = { 2.0, -1.0 };
    double matrix[3][2] = { { 1.0, -2.0 }, { -3.0, 5.0 }, { 4.0, 2.0 } };

    vtMultMt(vector, matrix, 3, 2, matrix);

    EXPECT_DOUBLE_EQ(matrix[0][0], 4.0);
    EXPECT_DOUBLE_EQ(matrix[0][1], -11.0);
    EXPECT_DOUBLE_EQ(matrix[1][0], 6.0);
    EXPECT_DOUBLE_EQ(matrix[1][1], 5.0);
}
