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

#include "architecture/utilities/rigidBodyKinematics.h"
#include <array>
#include <cstddef>
#include <cmath>
#include <limits>
#include <ostream>
#include <string>
#include <gtest/gtest.h>
#include "architecture/utilities/linearAlgebra.h"

namespace {

const double pi = std::acos(-1.0);  // [rad]
const double dcmTolerance = 5e-14;  // [-]

struct EulerSequence {
    const char* name;
    std::array<int, 3> axes;
    void (*fromDcm)(double[3][3], double*);
    void (*fromEp)(double*, double*);
    void (*add)(double*, double*, double*);
    void (*subtract)(double*, double*, double*);
};

const EulerSequence sequences[] = {
    {"121", {0, 1, 0}, C2Euler121, EP2Euler121, addEuler121, subEuler121},
    {"123", {0, 1, 2}, C2Euler123, EP2Euler123, addEuler123, subEuler123},
    {"131", {0, 2, 0}, C2Euler131, EP2Euler131, addEuler131, subEuler131},
    {"132", {0, 2, 1}, C2Euler132, EP2Euler132, addEuler132, subEuler132},
    {"212", {1, 0, 1}, C2Euler212, EP2Euler212, addEuler212, subEuler212},
    {"213", {1, 0, 2}, C2Euler213, EP2Euler213, addEuler213, subEuler213},
    {"231", {1, 2, 0}, C2Euler231, EP2Euler231, addEuler231, subEuler231},
    {"232", {1, 2, 1}, C2Euler232, EP2Euler232, addEuler232, subEuler232},
    {"312", {2, 0, 1}, C2Euler312, EP2Euler312, addEuler312, subEuler312},
    {"313", {2, 0, 2}, C2Euler313, EP2Euler313, addEuler313, subEuler313},
    {"321", {2, 1, 0}, C2Euler321, EP2Euler321, addEuler321, subEuler321},
    {"323", {2, 1, 2}, C2Euler323, EP2Euler323, addEuler323, subEuler323}
};

/** @brief Identify a parameterized test by its Euler sequence. */
void PrintTo(const EulerSequence& sequence, std::ostream* stream)
{
    *stream << sequence.name;
}

/** @brief Build a passive DCM independently from elementary rotations. */
void elementaryProduct(const EulerSequence& sequence, const double angles[3], double matrix[3][3], bool exact = false)
{
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            matrix[row][column] = row == column ? 1.0 : 0.0;
        }
    }
    for (std::size_t step = 0; step < sequence.axes.size(); ++step) {
        int row = (sequence.axes[step] + 1) % 3;
        int column = (sequence.axes[step] + 2) % 3;
        double cosine = std::cos(angles[step]);
        double sine = std::sin(angles[step]);
        if (exact) {
            cosine = std::abs(cosine) < 1e-15 ? 0.0 : cosine;
            sine = std::abs(sine) < 1e-15 ? 0.0 : sine;
        }
        double rotation[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
        rotation[row][row] = rotation[column][column] = cosine;
        rotation[row][column] = sine;
        rotation[column][row] = -sine;
        double product[3][3] = {};
        for (int first = 0; first < 3; ++first) {
            for (int second = 0; second < 3; ++second) {
                for (int inner = 0; inner < 3; ++inner) {
                    product[first][second] += rotation[first][inner] * matrix[inner][second];
                }
            }
        }
        for (int first = 0; first < 3; ++first) {
            for (int second = 0; second < 3; ++second) {
                matrix[first][second] = product[first][second];
            }
        }
    }
}

/** @brief Verify finite Euler angles, their middle-angle range, and the recovered rotation. */
void checkRotation(const EulerSequence& sequence, double matrix[3][3], double angles[3], bool singular)
{
    for (int index = 0; index < 3; ++index) {
        ASSERT_TRUE(std::isfinite(angles[index]));
    }
    if (singular) {
        EXPECT_DOUBLE_EQ(angles[2], 0.0);
    }
    bool repeated = sequence.axes[0] == sequence.axes[2];
    EXPECT_GE(angles[1], repeated ? 0.0 : -pi / 2.0);
    EXPECT_LE(angles[1], repeated ? pi : pi / 2.0);
    double recovered[3][3];
    elementaryProduct(sequence, angles, recovered);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            EXPECT_NEAR(recovered[row][column], matrix[row][column], dcmTolerance);
        }
    }
}

/** @brief Check the DCM and both signs of the equivalent unit quaternion. */
void checkConversions(const EulerSequence& sequence, double matrix[3][3], bool singular = false)
{
    double angles[3];  // [rad]
    sequence.fromDcm(matrix, angles);
    checkRotation(sequence, matrix, angles, singular);
    double quaternion[4];  // [-]
    C2EP(matrix, quaternion);
    for (int sign = 0; sign < 2; ++sign) {
        sequence.fromEp(quaternion, angles);
        checkRotation(sequence, matrix, angles, singular);
        for (double& value : quaternion) {
            value = -value;
        }
    }
}

class EulerConversions : public ::testing::TestWithParam<EulerSequence> {};

/** @brief Preserve rotations at both singular endpoints with positive or negative zeros. */
TEST_P(EulerConversions, Singularities)
{
    const auto& sequence = GetParam();
    SCOPED_TRACE(sequence.name);
    bool repeated = sequence.axes[0] == sequence.axes[2];
    const double limits[] = {repeated ? 0.0 : -pi / 2.0, repeated ? pi : pi / 2.0};  // [rad]
    for (double middle : limits) {
        double angles[] = {0.7, middle, -1.1};  // [rad]
        double matrix[3][3];
        elementaryProduct(sequence, angles, matrix, true);
        for (double zero : {0.0, -0.0}) {
            for (auto& row : matrix) {
                for (double& value : row) {
                    if (value == 0.0) {
                        value = zero;
                    }
                }
            }
            checkConversions(sequence, matrix, true);
        }
    }
}

/** @brief Preserve identity and pure first-axis rotations. */
TEST_P(EulerConversions, FirstAxisRotations)
{
    const auto& sequence = GetParam();
    SCOPED_TRACE(sequence.name);
    for (double angle : {0.0, 0.5}) {  // [rad]
        double angles[] = {angle, 0.0, 0.0};  // [rad]
        double matrix[3][3];
        elementaryProduct(sequence, angles, matrix);
        checkConversions(sequence, matrix, sequence.axes[0] == sequence.axes[2]);
    }
}

/** @brief Preserve rotations on both sides of each singularity. */
TEST_P(EulerConversions, NearSingularities)
{
    const auto& sequence = GetParam();
    SCOPED_TRACE(sequence.name);
    bool repeated = sequence.axes[0] == sequence.axes[2];
    const double limits[] = {repeated ? 0.0 : -pi / 2.0, repeated ? pi : pi / 2.0};  // [rad]
    for (double middle : limits) {
        for (double offset : {1e-4, 1e-8, 1e-12, 1e-15}) {  // [rad]
            for (double direction : {-1.0, 1.0}) {
                double angles[] = {0.7, middle + direction * offset, -1.1};  // [rad]
                double matrix[3][3];
                elementaryProduct(sequence, angles, matrix);
                checkConversions(sequence, matrix);
            }
        }
    }
}

/** @brief Preserve nonsingular rotations across outer-angle quadrants. */
TEST_P(EulerConversions, OrdinaryRotations)
{
    const auto& sequence = GetParam();
    SCOPED_TRACE(sequence.name);
    const double cases[][3] = {{0.7, 0.8, -1.1}, {-2.8, 0.8, 2.6}, {2.9, 0.8, 2.7}};  // [rad]
    for (const auto& angles : cases) {
        double matrix[3][3];
        elementaryProduct(sequence, angles, matrix);
        checkConversions(sequence, matrix);
    }
}

/** @brief Check composition against independent DCM products at regular and singular attitudes. */
TEST_P(EulerConversions, Composition)
{
    const auto& sequence = GetParam();
    double cases[][2][3] = {
        {{0.2, 0.5, 0.7}, {0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}, {0.2, 0.5, 0.7}},
        {{0.2, 0.5, 0.7}, {0.2, 0.5, 0.7}},
        {{0.7, 0.0, -1.1}, {-0.3, 0.8, 0.9}},
        {{0.7, pi, -1.1}, {-0.3, 0.8, 0.9}},
        {{0.7, 1e-12, -1.1}, {-0.3, pi - 1e-12, 0.9}},
        {{0.7, 0.0, 0.0}, {-0.7, 0.0, 0.0}},
        {{-2.8, 0.8, 2.6}, {2.9, -0.6, 2.7}}
    };  // [rad]
    for (auto& inputs : cases) {
        double first[3][3], second[3][3];
        elementaryProduct(sequence, inputs[0], first);
        elementaryProduct(sequence, inputs[1], second);
        for (bool subtract : {false, true}) {
            double expected[3][3] = {};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    for (int inner = 0; inner < 3; ++inner) {
                        expected[row][column] += subtract ? first[row][inner] * second[column][inner]
                                                         : second[row][inner] * first[inner][column];
                    }
                }
            }
            double result[3];  // [rad]
            (subtract ? sequence.subtract : sequence.add)(inputs[0], inputs[1], result);
            checkRotation(sequence, expected, result, false);
        }
    }
}

/** @brief Verify PRV rate matrices against the integrated matrix-exponential series. */
TEST(PrvKinematics, RateMatrices)
{
    for (double angle : {0.0, 1e-16, 1e-12, 1e-9, 1e-5, 0.009999, 0.01, 0.010001, 0.2, 2.0, 5.0}) {  // [rad]
        SCOPED_TRACE(angle);
        double vector[] = {angle * 2.0 / 7.0, -angle * 3.0 / 7.0, angle * 6.0 / 7.0};  // [rad]
        double negativeSkew[3][3] = {{0.0, vector[2], -vector[1]},
                                    {-vector[2], 0.0, vector[0]},
                                    {vector[1], -vector[0], 0.0}};
        double expected[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
        double term[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
        for (int order = 1; order <= 48; ++order) {
            double next[3][3] = {};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    for (int inner = 0; inner < 3; ++inner) {
                        next[row][column] += term[row][inner] * negativeSkew[inner][column] / (order + 1);
                    }
                }
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    term[row][column] = next[row][column];
                    expected[row][column] += next[row][column];
                }
            }
        }
        double matrix[3][3], inverse[3][3];
        BmatPRV(vector, matrix);
        BinvPRV(vector, inverse);
        double omega[] = {0.1, -0.2, 0.3};  // [rad/s]
        double derivative[3];  // [rad/s]
        dPRV(vector, omega, derivative);
        for (int row = 0; row < 3; ++row) {
            double recoveredRate = 0.0;  // [rad/s]
            for (int column = 0; column < 3; ++column) {
                ASSERT_TRUE(std::isfinite(matrix[row][column]));
                EXPECT_NEAR(inverse[row][column], expected[row][column],
                            3e-14 * std::abs(expected[row][column]) + 1e-22);
                double product = 0.0;
                for (int inner = 0; inner < 3; ++inner) {
                    product += expected[row][inner] * matrix[inner][column];
                }
                EXPECT_NEAR(product, row == column ? 1.0 : 0.0, 3e-14);
                recoveredRate += expected[row][column] * derivative[column];
                if (angle == 0.0) {
                    EXPECT_DOUBLE_EQ(matrix[row][column], row == column ? 1.0 : 0.0);
                    EXPECT_DOUBLE_EQ(inverse[row][column], row == column ? 1.0 : 0.0);
                }
            }
            EXPECT_NEAR(recoveredRate, omega[row], 3e-14);  // [rad/s]
            if (angle == 0.0) {
                EXPECT_DOUBLE_EQ(derivative[row], omega[row]);
            }
        }
    }
}

/** @brief Preserve small rotations and both quaternion angle branches. */
TEST(PrvKinematics, SmallConversions)
{
    const double axes[][3] = {{1.0, 0.0, 0.0}, {2.0 / 7.0, -3.0 / 7.0, 6.0 / 7.0}};
    for (double angle : {0.0, 1e-16, 1e-13, 1e-9, 1e-8, 1e-7, 1e-3, 0.4}) {  // [rad]
        SCOPED_TRACE(angle);
        for (const auto& axis : axes) {
            double vector[3], gibbs[3], mrp[3], result[3];
            double quaternion[] = {std::cos(angle / 2.0), axis[0] * std::sin(angle / 2.0),
                                   axis[1] * std::sin(angle / 2.0), axis[2] * std::sin(angle / 2.0)};
            for (int index = 0; index < 3; ++index) {
                vector[index] = angle * axis[index];  // [rad]
            }
            double elements[4], converted[4];
            PRV2elem(vector, elements);
            PRV2EP(vector, converted);
            EXPECT_NEAR(elements[0], angle, angle * 5e-15);
            for (int index = 0; index < 4; ++index) {
                EXPECT_NEAR(converted[index], quaternion[index], std::abs(quaternion[index]) * 5e-15);
                if (index > 0) {
                    EXPECT_NEAR(elements[index], angle == 0.0 ? 0.0 : axis[index - 1], 5e-15);
                }
            }
            PRV2Gibbs(vector, gibbs);
            PRV2MRP(vector, mrp);
            for (int index = 0; index < 3; ++index) {
                EXPECT_NEAR(gibbs[index], axis[index] * std::tan(angle / 2.0), angle * 5e-15);
                EXPECT_NEAR(mrp[index], axis[index] * std::tan(angle / 4.0), angle * 5e-15);
            }
            double matrix[3][3], expected[3][3];
            PRV2C(vector, matrix);
            EP2C(quaternion, expected);
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    EXPECT_NEAR(matrix[row][column], expected[row][column], std::abs(expected[row][column]) * 5e-15);
                }
            }
            C2PRV(expected, result);
            for (int index = 0; index < 3; ++index) {
                EXPECT_NEAR(result[index], vector[index], angle * 5e-15);
            }
            Gibbs2PRV(gibbs, result);
            for (int index = 0; index < 3; ++index) {
                EXPECT_NEAR(result[index], vector[index], angle * 5e-15);
            }
            MRP2PRV(mrp, result);
            for (int index = 0; index < 3; ++index) {
                EXPECT_NEAR(result[index], vector[index], angle * 5e-15);
            }
            EP2PRV(quaternion, result);
            for (int index = 0; index < 3; ++index) {
                EXPECT_NEAR(result[index], vector[index], angle * 5e-15);
            }
            for (double& element : quaternion) {
                element = -element;
            }
            EP2PRV(quaternion, result);
            for (int index = 0; index < 3; ++index) {
                double expectedAngle = angle == 0.0 ? 0.0 : (angle - 2.0 * pi) * axis[index];  // [rad]
                EXPECT_NEAR(result[index], expectedAngle, 1e-14);  // [rad]
            }
        }
    }
}

/** @brief Independently evaluate a passive rotation matrix using the exponential series. */
void rotationVectorMatrix(const double vector[3], double matrix[3][3])
{
    double negativeSkew[3][3] = {{0.0, vector[2], -vector[1]},
                                {-vector[2], 0.0, vector[0]},
                                {vector[1], -vector[0], 0.0}};
    double term[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            matrix[row][column] = term[row][column];
        }
    }
    for (int order = 1; order <= 64; ++order) {
        double next[3][3] = {};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                for (int inner = 0; inner < 3; ++inner) {
                    next[row][column] += term[row][inner] * negativeSkew[inner][column] / order;
                }
            }
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                term[row][column] = next[row][column];
                matrix[row][column] += next[row][column];
            }
        }
    }
}

/** @brief Preserve PRV composition for cancelling rotations and tiny noncommuting increments. */
TEST(PrvKinematics, Composition)
{
    double cases[][2][3] = {
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
        {{1.0, -0.5, 0.3}, {0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}, {1.0, -0.5, 0.3}},
        {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}},
        {{1.0, -0.5, 0.3}, {-1.0, 0.5, -0.3}},
        {{1.0, -0.5, 0.3}, {1.0, -0.5, 0.3}},
        {{1.0, 0.0, 0.0}, {-1.0 + 1e-12, 0.0, 0.0}},
        {{1.0, 0.0, 0.0}, {1.0 + 1e-12, 0.0, 0.0}},
        {{2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
        {{1.5, 0.5, 0.5}, {-0.5, 0.25, 0.15}},
        {{1e-9, 2e-9, 0.0}, {0.0, 0.7, -0.3}},
        {{1e-13, 2e-13, 0.0}, {0.0, -3e-13, 4e-13}}
    };  // [rad]
    for (auto& inputs : cases) {
        double first[3][3], second[3][3];
        rotationVectorMatrix(inputs[0], first);
        rotationVectorMatrix(inputs[1], second);
        for (bool subtract : {false, true}) {
            double result[3];  // [rad]
            (subtract ? subPRV : addPRV)(inputs[0], inputs[1], result);
            double recovered[3][3];
            rotationVectorMatrix(result, recovered);
            bool cancels = true;
            for (int index = 0; index < 3; ++index) {
                ASSERT_TRUE(std::isfinite(result[index]));
                cancels = cancels && inputs[0][index] == (subtract ? inputs[1][index] : -inputs[1][index]);
            }
            for (int row = 0; row < 3; ++row) {
                if (cancels) {
                    EXPECT_DOUBLE_EQ(result[row], 0.0);
                }
                for (int column = 0; column < 3; ++column) {
                    double expected = 0.0;
                    for (int inner = 0; inner < 3; ++inner) {
                        expected += subtract ? first[row][inner] * second[column][inner]
                                             : second[row][inner] * first[inner][column];
                    }
                    EXPECT_NEAR(recovered[row][column], expected, 2e-14);
                }
            }
        }
    }
    double vector[] = {2.0, 0.0, 0.0};  // [rad]
    double result[3];  // [rad]
    addPRV(vector, vector, result);
    EXPECT_NEAR(result[0], 4.0, 1e-15);  // [rad]
    EXPECT_DOUBLE_EQ(result[1], 0.0);
    EXPECT_DOUBLE_EQ(result[2], 0.0);
}

/** @brief Verify ordinary MRP shadow and composition branches, including a full-turn result. */
TEST(MrpKinematics, ShadowAndComposition)
{
    double first[3] = {1.0, 0.0, 0.0};  // [-] half-turn MRP set
    for (double transverse : {0.0, 1e-4}) {  // [-]
        double second[3] = {1.0, transverse, 0.0};  // [-]
        double result[3];
        double firstMatrix[3][3];
        double secondMatrix[3][3];
        double expected[3][3];
        double actual[3][3];
        MRP2C(first, firstMatrix);
        for (bool subtract : {false, true}) {
            if (subtract) {
                v3Scale(-1.0, second, second);
            }
            MRP2C(second, secondMatrix);
            if (subtract) {
                m33MultM33t(firstMatrix, secondMatrix, expected);
            } else {
                m33MultM33(secondMatrix, firstMatrix, expected);
            }
            (subtract ? subMRP : addMRP)(first, second, result);
            MRP2C(result, actual);
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    EXPECT_NEAR(actual[row][column], expected[row][column], 5e-15);
                }
            }
            EXPECT_LE(v3Norm(result), 1.0);
        }
    }
    double sigma[3] = {2.0, 0.0, 0.0};  // [-] outside the principal set
    double quaternion[4];
    double shadow[3];
    MRP2EP(sigma, quaternion);
    EXPECT_NEAR(quaternion[0], -0.6, 1e-15);
    EXPECT_NEAR(quaternion[1], 0.8, 1e-15);
    MRPswitch(sigma, 1.0, shadow);  // [-] norm threshold
    EXPECT_DOUBLE_EQ(shadow[0], -0.5);
    MRPshadow(sigma, sigma);  // In-place C calls remain supported.
    EXPECT_DOUBLE_EQ(sigma[0], shadow[0]);
}


/** @brief Preserve quaternion conversion when input and output share a four-double buffer. */
TEST(AttitudeConversions, InPlaceQuaternion)
{
    for (auto convert : {MRP2EP, Gibbs2EP}) {
        double buffer[4] = {0.1, 0.2, 0.3, 0.0};  // [-]
        double expected[4];
        convert(buffer, expected);
        convert(buffer, buffer);
        for (int index = 0; index < 4; ++index) {
            EXPECT_DOUBLE_EQ(buffer[index], expected[index]);
        }
    }
}


/** @brief Remove multiple revolutions without losing the known fractional angle. */
TEST(AngleWrapping, MultipleRevolutions)
{
    for (int turns : {-1000000, -100, -10, -3, -2, -1, 0, 1, 2, 3, 10, 100, 1000000}) {
        for (double offset : {-2.9, -0.2, 0.0, 0.2, 2.9}) {  // [rad]
            double angle = turns * (2.0 * pi) + offset;  // [rad]
            double result = wrapToPi(angle);
            double scale = std::abs(angle) > 1.0 ? std::abs(angle) : 1.0;  // [rad]
            double tolerance = 4.0 * std::numeric_limits<double>::epsilon() * scale;  // [rad]
            EXPECT_GE(result, -pi);
            EXPECT_LE(result, pi);
            EXPECT_NEAR(result, offset, tolerance);
            EXPECT_DOUBLE_EQ(wrapToPi(result), result);
        }
    }
}

/** @brief Preserve the closed interval, signed zeros, and both half-turn endpoints. */
TEST(AngleWrapping, Boundaries)
{
    for (double angle : {0.0, -0.0, 1e-12, -1e-12, 0.2, -0.2, pi, -pi,
                         std::nextafter(pi, 0.0), std::nextafter(-pi, 0.0)}) {  // [rad]
        double result = wrapToPi(angle);
        EXPECT_DOUBLE_EQ(result, angle);
        EXPECT_EQ(std::signbit(result), std::signbit(angle));
    }
    for (int multiple : {1, 3, 5}) {
        for (double sign : {-1.0, 1.0}) {
            double boundary = sign * (multiple * pi);  // [rad]
            EXPECT_DOUBLE_EQ(wrapToPi(boundary), sign * pi);
            double inside = wrapToPi(std::nextafter(boundary, 0.0));
            double outside = wrapToPi(std::nextafter(boundary, sign * std::numeric_limits<double>::infinity()));
            EXPECT_GT(sign * inside, 0.0);
            EXPECT_LT(sign * inside, pi);
            EXPECT_GT(sign * outside, -pi);
            EXPECT_LT(sign * outside, 0.0);
        }
    }
    for (int multiple : {2, 4, 8}) {
        for (double sign : {-1.0, 1.0}) {
            double result = wrapToPi(sign * multiple * pi);
            EXPECT_DOUBLE_EQ(result, 0.0);
            EXPECT_EQ(std::signbit(result), sign < 0.0);
        }
    }
}

/** @brief Return NaN for inputs with no finite wrapped angle. */
TEST(AngleWrapping, Nonfinite)
{
    EXPECT_TRUE(std::isnan(wrapToPi(std::numeric_limits<double>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(wrapToPi(std::numeric_limits<double>::infinity())));
    EXPECT_TRUE(std::isnan(wrapToPi(-std::numeric_limits<double>::infinity())));
}

INSTANTIATE_TEST_SUITE_P(RigidBodyKinematics, EulerConversions, ::testing::ValuesIn(sequences),
                        [](const ::testing::TestParamInfo<EulerSequence>& info) {
                            return std::string("Sequence") + info.param.name;
                        });

}  // namespace
