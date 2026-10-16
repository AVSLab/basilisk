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
#include <ostream>
#include <string>
#include <gtest/gtest.h>

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

INSTANTIATE_TEST_SUITE_P(RigidBodyKinematics, EulerConversions, ::testing::ValuesIn(sequences),
                        [](const ::testing::TestParamInfo<EulerSequence>& info) {
                            return std::string("Sequence") + info.param.name;
                        });

}  // namespace
