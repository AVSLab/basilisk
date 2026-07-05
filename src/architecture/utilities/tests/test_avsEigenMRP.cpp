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

#include "architecture/utilities/avsEigenMRP.h"

#include <cmath>
#include <gtest/gtest.h>

namespace {

constexpr double kTolerance = 1e-12;  // [-]
constexpr double kLooseTolerance = 1e-6;  // [-]
constexpr double kPi = 3.14159265358979323846;  // [rad]

void expectVectorNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected, double tolerance)
{
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(actual(i), expected(i), tolerance);
    }
}

void expectMatrixNear(const Eigen::Matrix3d& actual, const Eigen::Matrix3d& expected, double tolerance)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_NEAR(actual(i, j), expected(i, j), tolerance);
        }
    }
}

void expectFiniteMRP(const Eigen::MRPd& sigma)
{
    for (int i = 0; i < 3; i++) {
        EXPECT_TRUE(std::isfinite(sigma.coeffs()(i)));
    }
}

Eigen::Matrix3d expectedBmat(const Eigen::MRPd& sigma)
{
    const double sigmaSquared = sigma.squaredNorm();  // [-]
    Eigen::Matrix3d expected = (1.0 - sigmaSquared)*Eigen::Matrix3d::Identity();  // [-]
    expected += 2.0*sigma.coeffs()*sigma.coeffs().transpose();

    Eigen::Matrix3d sigmaTilde;
    sigmaTilde << 0.0, -sigma.z(), sigma.y(),
                  sigma.z(), 0.0, -sigma.x(),
                  -sigma.y(), sigma.x(), 0.0;
    expected += 2.0*sigmaTilde;
    return expected;
}

}

TEST(eigenMRP, testIdentity) {
    Eigen::MRPd sigma;
    sigma = sigma.Identity();

    EXPECT_TRUE(sigma.norm() < kTolerance);
}

TEST(eigenMRP, testCoefficientConstructorsAndAccessors) {
    const double rawCoefficients[3] = {0.10, -0.20, 0.30};  // [-]

    Eigen::MRPd fromScalars(0.10, -0.20, 0.30);
    Eigen::MRPd fromPointer(rawCoefficients);
    Eigen::MRPd fromVector(Eigen::Vector3d(0.10, -0.20, 0.30));

    expectVectorNear(fromScalars.coeffs(), Eigen::Vector3d(0.10, -0.20, 0.30), kTolerance);
    expectVectorNear(fromPointer.coeffs(), fromScalars.coeffs(), kTolerance);
    expectVectorNear(fromVector.vec(), fromScalars.coeffs(), kTolerance);

    fromScalars.x() = -0.40;
    fromScalars.y() = 0.50;
    fromScalars.z() = -0.60;

    EXPECT_NEAR(fromScalars.x(), -0.40, kTolerance);
    EXPECT_NEAR(fromScalars.y(), 0.50, kTolerance);
    EXPECT_NEAR(fromScalars.z(), -0.60, kTolerance);
    expectVectorNear(fromScalars.coeffs(), Eigen::Vector3d(-0.40, 0.50, -0.60), kTolerance);
}

TEST(eigenMRP, testSetIdentity) {
    Eigen::MRPd sigma(0.20, -0.10, 0.40);

    sigma.setIdentity();

    expectVectorNear(sigma.coeffs(), Eigen::Vector3d::Zero(), kTolerance);
    expectMatrixNear(sigma.toRotationMatrix(), Eigen::Matrix3d::Identity(), kTolerance);
}

TEST(eigenMRP, testAngleAxisConversionUsesTangentQuarterAngle) {
    const double angle = kPi;  // [rad]
    Eigen::AngleAxisd angleAxis(angle, Eigen::Vector3d::UnitY());

    Eigen::MRPd sigma(angleAxis);

    EXPECT_NEAR(sigma.norm(), std::tan(angle/4.0), kTolerance);
    expectMatrixNear(sigma.toRotationMatrix(), angleAxis.toRotationMatrix(), kTolerance);
}

TEST(eigenMRP, testAngleAxisConversionChoosesEquivalentInnerSet) {
    const double angle = 1.5*kPi;  // [rad]
    Eigen::AngleAxisd angleAxis(angle, Eigen::Vector3d::UnitZ());

    Eigen::MRPd sigma(angleAxis);

    EXPECT_LT(sigma.norm(), 1.0);
    expectMatrixNear(sigma.toRotationMatrix(), angleAxis.toRotationMatrix(), kTolerance);
}

TEST(eigenMRP, testRotationMatrixConversionChoosesEquivalentInnerSet) {
    const double angle = 1.5*kPi;  // [rad]
    Eigen::AngleAxisd angleAxis(angle, Eigen::Vector3d::UnitZ());

    Eigen::MRPd sigma(angleAxis.toRotationMatrix());

    EXPECT_LT(sigma.norm(), 1.0);
    expectMatrixNear(sigma.toRotationMatrix(), angleAxis.toRotationMatrix(), kTolerance);
}

TEST(eigenMRP, testCompositionMatchesRotationMatrixProduct) {
    Eigen::MRPd sigmaOne(0.15, -0.20, 0.10);
    Eigen::MRPd sigmaTwo(-0.05, 0.08, 0.12);

    Eigen::MRPd sigmaCombined = sigmaOne*sigmaTwo;

    expectMatrixNear(sigmaCombined.toRotationMatrix(),
                     sigmaOne.toRotationMatrix()*sigmaTwo.toRotationMatrix(),
                     kTolerance);
}

TEST(eigenMRP, testCompositionUsesShadowSetNearSingularity) {
    Eigen::MRPd sigmaOne(1.0, 0.0, 0.0);
    Eigen::MRPd sigmaTwo(1.0, 0.0, 0.0);

    Eigen::MRPd sigmaCombined = sigmaOne*sigmaTwo;

    expectFiniteMRP(sigmaCombined);
    EXPECT_NEAR(sigmaCombined.norm(), 0.0, kTolerance);
    expectMatrixNear(sigmaCombined.toRotationMatrix(), Eigen::Matrix3d::Identity(), kTolerance);
}

TEST(eigenMRP, testCompoundCompositionMatchesBinaryComposition) {
    Eigen::MRPd sigmaOne(0.25, 0.10, -0.05);
    Eigen::MRPd sigmaTwo(-0.15, 0.05, 0.20);
    Eigen::MRPd sigmaProduct = sigmaOne;

    sigmaProduct *= sigmaTwo;

    expectVectorNear(sigmaProduct.coeffs(), (sigmaOne*sigmaTwo).coeffs(), kTolerance);
    expectMatrixNear(sigmaProduct.toRotationMatrix(),
                     sigmaOne.toRotationMatrix()*sigmaTwo.toRotationMatrix(),
                     kTolerance);
}

TEST(eigenMRP, testCoefficientAdditionAndSubtractionOperators) {
    Eigen::MRPd sigma(0.20, -0.30, 0.10);
    Eigen::MRPd delta(-0.05, 0.10, 0.20);

    sigma += delta;
    expectVectorNear(sigma.coeffs(), Eigen::Vector3d(0.15, -0.20, 0.30), kTolerance);

    sigma -= delta;
    expectVectorNear(sigma.coeffs(), Eigen::Vector3d(0.20, -0.30, 0.10), kTolerance);
}

TEST(eigenMRP, testVectorTransformUsesRotationMatrix) {
    const double angle = kPi/2.0;  // [rad]
    Eigen::MRPd sigma(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d vector(1.0, -2.0, 0.5);

    expectVectorNear(sigma*vector, sigma.toRotationMatrix()*vector, kTolerance);
}

TEST(eigenMRP, testFromTwoVectorsHandlesGeneralParallelAntiparallelAndZeroVectors) {
    Eigen::Vector3d vectorOne(2.0, 0.0, 0.0);
    Eigen::Vector3d vectorTwo(0.0, -3.0, 0.0);
    Eigen::Vector3d smallVectorOne(1.0e-7, 0.0, 0.0);
    Eigen::Vector3d smallVectorTwo(0.0, -2.0e-7, 0.0);
    Eigen::MRPd sigmaMember;
    Eigen::MRPd sigmaGeneral = Eigen::MRPd::FromTwoVectors(vectorOne, vectorTwo);
    Eigen::MRPd sigmaParallel = Eigen::MRPd::FromTwoVectors(vectorOne, vectorOne);
    Eigen::MRPd sigmaAntiparallel = Eigen::MRPd::FromTwoVectors(vectorOne, -vectorOne);
    Eigen::MRPd sigmaSmall = Eigen::MRPd::FromTwoVectors(smallVectorOne, smallVectorTwo);
    Eigen::MRPd sigmaZero = Eigen::MRPd::FromTwoVectors(Eigen::Vector3d::Zero(), vectorTwo);

    sigmaMember.setFromTwoVectors(vectorOne, vectorTwo);

    expectFiniteMRP(sigmaGeneral);
    expectFiniteMRP(sigmaParallel);
    expectFiniteMRP(sigmaAntiparallel);
    expectFiniteMRP(sigmaSmall);
    expectFiniteMRP(sigmaZero);
    expectVectorNear(sigmaMember.coeffs(), sigmaGeneral.coeffs(), kTolerance);
    expectVectorNear((sigmaGeneral*vectorOne).normalized(), vectorTwo.normalized(), kTolerance);
    expectVectorNear(sigmaParallel*vectorOne, vectorOne, kTolerance);
    expectVectorNear(sigmaAntiparallel*vectorOne, -vectorOne, kTolerance);
    expectVectorNear((sigmaSmall*smallVectorOne).normalized(), smallVectorTwo.normalized(), kTolerance);
    expectMatrixNear(sigmaZero.toRotationMatrix(), Eigen::Matrix3d::Identity(), kTolerance);
}

TEST(eigenMRP, testNormSquaredNormDotAndIsApprox) {
    Eigen::MRPd sigma(0.20, -0.30, 0.40);
    Eigen::MRPd other(-0.10, 0.05, 0.20);

    EXPECT_NEAR(sigma.squaredNorm(), 0.29, kTolerance);
    EXPECT_NEAR(sigma.norm(), std::sqrt(0.29), kTolerance);
    EXPECT_NEAR(sigma.dot(other), 0.045, kTolerance);
    EXPECT_TRUE(sigma.isApprox(Eigen::MRPd(0.20 + 1e-13, -0.30, 0.40), 1e-12));
    EXPECT_FALSE(sigma.isApprox(Eigen::MRPd(0.21, -0.30, 0.40), 1e-12));
}

TEST(eigenMRP, testNormalizeInPlaceReturnsPrincipalAxisDirection) {
    Eigen::MRPd sigma(3.0, 4.0, 0.0);

    sigma.normalize();

    EXPECT_NEAR(sigma.norm(), 1.0, kTolerance);
    expectVectorNear(sigma.coeffs(), Eigen::Vector3d(0.6, 0.8, 0.0), kTolerance);
}

TEST(eigenMRP, testShadowSet) {
    Eigen::MRPd sigma(2.0, -1.0, 0.0);
    Eigen::MRPd zero = Eigen::MRPd::Identity();

    Eigen::MRPd shadow = sigma.shadow();

    expectVectorNear(shadow.coeffs(), Eigen::Vector3d(-0.4, 0.2, 0.0), kTolerance);
    expectMatrixNear(shadow.toRotationMatrix(), sigma.toRotationMatrix(), kTolerance);
    expectVectorNear(zero.shadow().coeffs(), Eigen::Vector3d::Zero(), kTolerance);
}

TEST(eigenMRP, testBmatMatchesMRPKinematicsDefinition) {
    Eigen::MRPd sigma(0.20, -0.30, 0.10);

    expectMatrixNear(sigma.Bmat(), expectedBmat(sigma), kTolerance);
}

TEST(eigenMRP, testInverseAndConjugateRepresentOppositeRotation) {
    Eigen::MRPd sigma(0.20, -0.10, 0.05);
    Eigen::MRPd inverse = sigma.inverse();
    Eigen::MRPd conjugate = sigma.conjugate();
    Eigen::MRPd identityProduct = sigma*inverse;

    expectVectorNear(inverse.coeffs(), -sigma.coeffs(), kTolerance);
    expectVectorNear(conjugate.coeffs(), -sigma.coeffs(), kTolerance);
    expectMatrixNear(inverse.toRotationMatrix(), sigma.toRotationMatrix().transpose(), kTolerance);
    expectMatrixNear(identityProduct.toRotationMatrix(), Eigen::Matrix3d::Identity(), kTolerance);
}

TEST(eigenMRP, testAngularDistance) {
    const double angle = kPi/3.0;  // [rad]
    Eigen::MRPd identity = Eigen::MRPd::Identity();
    Eigen::MRPd sigma(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));

    EXPECT_NEAR(identity.angularDistance(sigma), angle, kTolerance);
    EXPECT_NEAR(sigma.angularDistance(identity), angle, kTolerance);
    EXPECT_NEAR(sigma.angularDistance(sigma), 0.0, kTolerance);
}

TEST(eigenMRP, testCastConvertsScalarType) {
    Eigen::MRPd sigma(0.20, -0.30, 0.10);

    const Eigen::MRPd& sigmaDouble = sigma.cast<double>();
    Eigen::MRPf sigmaFloat = sigma.cast<float>();
    Eigen::MRPf constructedFloat(sigma);

    EXPECT_EQ(&sigmaDouble, &sigma);
    EXPECT_NEAR(sigmaFloat.x(), 0.20F, kLooseTolerance);
    EXPECT_NEAR(sigmaFloat.y(), -0.30F, kLooseTolerance);
    EXPECT_NEAR(sigmaFloat.z(), 0.10F, kLooseTolerance);
    EXPECT_NEAR(constructedFloat.x(), 0.20F, kLooseTolerance);
    EXPECT_NEAR(constructedFloat.y(), -0.30F, kLooseTolerance);
    EXPECT_NEAR(constructedFloat.z(), 0.10F, kLooseTolerance);
}

TEST(eigenMRP, testNormalizedReturnsPrincipalAxisDirection) {
    Eigen::MRPd sigma(0.0, -2.0, 0.0);

    Eigen::MRPd axis = sigma.normalized();

    EXPECT_NEAR(axis.norm(), 1.0, kTolerance);
    expectVectorNear(axis.coeffs(), Eigen::Vector3d(0.0, -1.0, 0.0), kTolerance);
}

TEST(eigenMRP, testAssignmentFromVectorMatrixAndMRPBase) {
    Eigen::MRPd sigma;
    Eigen::Vector3d coeffs(0.10, -0.20, 0.30);
    const double angle = kPi/4.0;  // [rad]
    Eigen::AngleAxisd angleAxis(angle, Eigen::Vector3d::UnitZ());

    sigma = coeffs;
    expectVectorNear(sigma.coeffs(), coeffs, kTolerance);

    sigma = angleAxis;
    expectMatrixNear(sigma.toRotationMatrix(), angleAxis.toRotationMatrix(), kTolerance);

    sigma = angleAxis.toRotationMatrix();
    expectMatrixNear(sigma.toRotationMatrix(), angleAxis.toRotationMatrix(), kTolerance);

    Eigen::MRPd copied;
    copied = sigma;
    expectVectorNear(copied.coeffs(), sigma.coeffs(), kTolerance);
}

TEST(eigenMRP, testMapReadWrite) {
    double coeffs[3] = {0.1, -0.2, 0.3};
    Eigen::Map<Eigen::MRPd> sigmaMap(coeffs);

    sigmaMap = Eigen::MRPd(0.4, -0.5, 0.6);

    EXPECT_NEAR(coeffs[0], 0.4, kTolerance);
    EXPECT_NEAR(coeffs[1], -0.5, kTolerance);
    EXPECT_NEAR(coeffs[2], 0.6, kTolerance);

    Eigen::Map<const Eigen::MRPd> constSigmaMap(coeffs);
    expectVectorNear(constSigmaMap.coeffs(), Eigen::Vector3d(0.4, -0.5, 0.6), kTolerance);
}

TEST(eigenMRP, testMapAliasesAndMappedOperations) {
    double coeffs[3] = {0.2, 0.0, 0.0};
    Eigen::MRPMapd sigmaMap(coeffs);

    sigmaMap.x() = 0.1;
    sigmaMap.y() = -0.2;
    sigmaMap.z() = 0.3;
    sigmaMap += Eigen::MRPd(0.1, 0.1, -0.1);

    EXPECT_NEAR(coeffs[0], 0.2, kTolerance);
    EXPECT_NEAR(coeffs[1], -0.1, kTolerance);
    EXPECT_NEAR(coeffs[2], 0.2, kTolerance);
    expectMatrixNear((sigmaMap*sigmaMap.inverse()).toRotationMatrix(), Eigen::Matrix3d::Identity(), kTolerance);
}

TEST(eigenMRP, testAlignedMapAlias) {
    EIGEN_ALIGN16 double coeffs[4] = {0.1, -0.2, 0.3, 0.0};
    Eigen::MRPMapAlignedd sigmaMap(coeffs);

    sigmaMap = Eigen::MRPd(-0.4, 0.5, -0.6);

    EXPECT_NEAR(coeffs[0], -0.4, kTolerance);
    EXPECT_NEAR(coeffs[1], 0.5, kTolerance);
    EXPECT_NEAR(coeffs[2], -0.6, kTolerance);
}
