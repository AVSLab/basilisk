/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include <gtest/gtest.h>
#include "architecture/utilities/avsEigenSupport.h"
#include <cmath>

// Test Vector3d conversions
TEST(avsEigenSupport, Vector3dConversions) {
    Eigen::Vector3d vec(1, 2, 3);
    double array[3];

    eigenVector3d2CArray(vec, array);
    Eigen::Vector3d result = cArray2EigenVector3d(array);

    EXPECT_TRUE(vec.isApprox(result));
}

// Test Matrix3d conversions
TEST(avsEigenSupport, Matrix3dConversions) {
    Eigen::Matrix3d mat;
    mat << 1, 2, 3,
           4, 5, 6,
           7, 8, 9;

    double array[9];
    eigenMatrix3d2CArray(mat, array);

    Eigen::Matrix3d result = cArray2EigenMatrix3d(array);
    EXPECT_TRUE(mat.isApprox(result));
}

// Test 2D array to Matrix3d conversion
TEST(avsEigenSupport, C2DArrayToMatrix3d) {
    double array2d[3][3] = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };

    Eigen::Matrix3d expected;
    expected << 1, 2, 3,
               4, 5, 6,
               7, 8, 9;

    Eigen::Matrix3d result = c2DArray2EigenMatrix3d(array2d);
    EXPECT_TRUE(expected.isApprox(result));
}

// Test rotation matrices
TEST(avsEigenSupport, RotationMatrices) {
    double angle = M_PI / 4.0; // 45 degrees

    // Test M1 (rotation about x-axis)
    Eigen::Matrix3d m1 = eigenM1(angle);
    EXPECT_NEAR(m1(1,1), cos(angle), 1e-10);
    EXPECT_NEAR(m1(1,2), sin(angle), 1e-10);
    EXPECT_NEAR(m1(2,1), -sin(angle), 1e-10);

    // Test M2 (rotation about y-axis)
    Eigen::Matrix3d m2 = eigenM2(angle);
    EXPECT_NEAR(m2(0,0), cos(angle), 1e-10);
    EXPECT_NEAR(m2(0,2), -sin(angle), 1e-10);
    EXPECT_NEAR(m2(2,0), sin(angle), 1e-10);

    // Test M3 (rotation about z-axis)
    Eigen::Matrix3d m3 = eigenM3(angle);
    EXPECT_NEAR(m3(0,0), cos(angle), 1e-10);
    EXPECT_NEAR(m3(0,1), sin(angle), 1e-10);
    EXPECT_NEAR(m3(1,0), -sin(angle), 1e-10);
}

// Test tilde matrix
TEST(avsEigenSupport, TildeMatrix) {
    Eigen::Vector3d vec(1, 2, 3);
    Eigen::Matrix3d tilde = eigenTilde(vec);

    // Test properties of skew-symmetric matrix
    EXPECT_NEAR(tilde(0,0), 0, 1e-10);
    EXPECT_NEAR(tilde(1,1), 0, 1e-10);
    EXPECT_NEAR(tilde(2,2), 0, 1e-10);
    EXPECT_NEAR(tilde(0,1), -vec(2), 1e-10);
    EXPECT_NEAR(tilde(1,2), -vec(0), 1e-10);
    EXPECT_NEAR(tilde(2,0), -vec(1), 1e-10);
}

// Test MRP conversions
TEST(avsEigenSupport, MRPConversions) {
    Eigen::Vector3d mrp(0.1, 0.2, 0.3);
    double array[3];

    eigenMRPd2CArray(mrp, array);
    Eigen::MRPd result = cArray2EigenMRPd(array);

    EXPECT_NEAR(mrp[0], result.x(), 1e-10);
    EXPECT_NEAR(mrp[1], result.y(), 1e-10);
    EXPECT_NEAR(mrp[2], result.z(), 1e-10);
}

// Test Newton-Raphson solver
TEST(avsEigenSupport, NewtonRaphson) {
    // Test with a simple function f(x) = x^2 - 4
    auto f = [](double x) { return x*x - 4; };
    auto fPrime = [](double x) { return 2*x; };

    double result = newtonRaphsonSolve(3.0, 1e-10, f, fPrime);
    EXPECT_NEAR(result, 2.0, 1e-10);
}

TEST(avsEigenSupport, MatrixXiConversions) {
    Eigen::MatrixXi mat(2, 3);
    mat << 1, 2, 3,
           4, 5, 6;

    int array[6];
    eigenMatrixXi2CArray(mat, array);

    // Verify the conversion
    for(int i = 0; i < 6; i++) {
        EXPECT_EQ(array[i], mat(i/3, i%3));
    }
}

TEST(avsEigenSupport, DCMtoMRP) {
    // Create a simple rotation DCM (90 degrees about 3rd axis)
    Eigen::Matrix3d dcm;
    dcm << 0, 1, 0,
           -1,  0, 0,
           0,  0, 1;

    Eigen::MRPd mrp = eigenC2MRP(dcm);

    // MRP conversion values validated with software package
    // from Analytical Mechanics of Space Systems
    // by Schaub and Junkins
    EXPECT_NEAR(mrp.x(), 0.0, 1e-6);
    EXPECT_NEAR(mrp.y(), 0.0, 1e-6);
    EXPECT_NEAR(mrp.z(), 0.41421356, 1e-6);
}

TEST(avsEigenSupport, MRPdToVector3d) {
    Eigen::MRPd mrp(0.1, 0.2, 0.3);
    Eigen::Vector3d vec = eigenMRPd2Vector3d(mrp);

    EXPECT_NEAR(mrp.x(), vec[0], 1e-10);
    EXPECT_NEAR(mrp.y(), vec[1], 1e-10);
    EXPECT_NEAR(mrp.z(), vec[2], 1e-10);
}
