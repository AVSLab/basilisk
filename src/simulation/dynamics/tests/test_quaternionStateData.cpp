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

#include "simulation/dynamics/_GeneralModuleFiles/quaternionStateData.h"

#include <gtest/gtest.h>

#include <cmath>

namespace {
constexpr double TOL = 1e-12;

// Make a QuaternionStateData with the given (w, x, y, z) and angular velocity.
QuaternionStateData
makeQ(double w,
      double x,
      double y,
      double z,
      double wx = 0.0, // [rad/s]
      double wy = 0.0, // [rad/s]
      double wz = 0.0) // [rad/s]
{
    QuaternionStateData q("test", Eigen::MatrixXd::Zero(4, 1));
    q.state(0) = w;
    q.state(1) = x;
    q.state(2) = y;
    q.state(3) = z;
    q.stateDeriv(0) = wx;
    q.stateDeriv(1) = wy;
    q.stateDeriv(2) = wz;
    return q;
}
}

TEST(QuaternionStateDataTest, ConstructsAtIdentity)
{
    QuaternionStateData q("default", Eigen::MatrixXd::Zero(4, 1));
    EXPECT_EQ(q.state.rows(), 4);
    EXPECT_EQ(q.state.cols(), 1);
    EXPECT_EQ(q.stateDeriv.rows(), 3);
    EXPECT_EQ(q.stateDeriv.cols(), 1);

    EXPECT_DOUBLE_EQ(q.state(0), 1.0);
    EXPECT_DOUBLE_EQ(q.state(1), 0.0);
    EXPECT_DOUBLE_EQ(q.state(2), 0.0);
    EXPECT_DOUBLE_EQ(q.state(3), 0.0);

    EXPECT_DOUBLE_EQ(q.stateDeriv(0), 0.0);
    EXPECT_DOUBLE_EQ(q.stateDeriv(1), 0.0);
    EXPECT_DOUBLE_EQ(q.stateDeriv(2), 0.0);
}

TEST(QuaternionStateDataTest, ConstructsAtIdentityFromEmptyState)
{
    QuaternionStateData q("default", Eigen::MatrixXd());

    EXPECT_DOUBLE_EQ(q.state(0), 1.0);
    EXPECT_DOUBLE_EQ(q.state(1), 0.0);
    EXPECT_DOUBLE_EQ(q.state(2), 0.0);
    EXPECT_DOUBLE_EQ(q.state(3), 0.0);
}

TEST(QuaternionStateDataTest, ConstructsFromInitialQuaternion)
{
    Eigen::MatrixXd initial(4, 1);
    initial << 0.5, -0.5, 0.5, -0.5;

    QuaternionStateData q("initial", initial);

    EXPECT_DOUBLE_EQ(q.state(0), initial(0));
    EXPECT_DOUBLE_EQ(q.state(1), initial(1));
    EXPECT_DOUBLE_EQ(q.state(2), initial(2));
    EXPECT_DOUBLE_EQ(q.state(3), initial(3));
}

TEST(QuaternionStateDataTest, NormalizesInitialQuaternion)
{
    Eigen::MatrixXd initial(4, 1);
    initial << 1.0, 1.0, 1.0, 1.0;

    QuaternionStateData q("initial", initial);

    EXPECT_NEAR(q.state.norm(), 1.0, TOL);
    EXPECT_NEAR(q.state(0), 0.5, TOL);
    EXPECT_NEAR(q.state(1), 0.5, TOL);
    EXPECT_NEAR(q.state(2), 0.5, TOL);
    EXPECT_NEAR(q.state(3), 0.5, TOL);
}

TEST(QuaternionStateDataTest, RejectsInitialStateWithWrongSize)
{
    EXPECT_THROW(QuaternionStateData("invalid", Eigen::MatrixXd::Zero(3, 1)), BasiliskError);
}

TEST(QuaternionStateDataTest, ZeroAngularVelocityIsFixedPoint)
{
    // Arbitrary unit quaternion. Propagating with zero omega should not move it.
    double n = std::sqrt(1 + 4 + 9 + 16);
    auto q = makeQ(1.0 / n, 2.0 / n, 3.0 / n, 4.0 / n);
    auto qw = q.state(0), qx = q.state(1), qy = q.state(2), qz = q.state(3);

    q.propagateState(0.1);

    EXPECT_NEAR(q.state(0), qw, TOL);
    EXPECT_NEAR(q.state(1), qx, TOL);
    EXPECT_NEAR(q.state(2), qy, TOL);
    EXPECT_NEAR(q.state(3), qz, TOL);
}

TEST(QuaternionStateDataTest, SingleAxisRotationMatchesAnalytical)
{
    // Rotate identity about +z at 1 rad/s for pi/2 s -> q = (cos(pi/4), 0, 0, sin(pi/4)).
    auto q = makeQ(1.0, 0.0, 0.0, 0.0, /*omega*/ 0.0, 0.0, 1.0);
    q.propagateState(M_PI_2);

    const double expectedW = std::cos(M_PI_4);
    const double expectedZ = std::sin(M_PI_4);
    EXPECT_NEAR(q.state(0), expectedW, TOL);
    EXPECT_NEAR(q.state(1), 0.0, TOL);
    EXPECT_NEAR(q.state(2), 0.0, TOL);
    EXPECT_NEAR(q.state(3), expectedZ, TOL);
}

TEST(QuaternionStateDataTest, PreservesUnitNormAfterManySteps)
{
    // Drift in |q| over many steps would corrupt downstream rotations, so the
    // integrator must renormalize even with a non-axis-aligned omega.
    auto q = makeQ(1.0, 0.0, 0.0, 0.0, /*omega*/ 0.3, -0.2, 0.7);
    const double dt = 0.05; // [s]
    for (int i = 0; i < 1000; ++i) {
        q.propagateState(dt);
    }
    double norm =
      std::sqrt(q.state(0) * q.state(0) + q.state(1) * q.state(1) + q.state(2) * q.state(2) + q.state(3) * q.state(3));
    EXPECT_NEAR(norm, 1.0, 1e-12);
}

TEST(QuaternionStateDataTest, SmallAngleBranchIsStable)
{
    // |omega * dt| below the 1e-12 branch threshold must not divide by zero
    // and should yield the identity update.
    auto q = makeQ(1.0, 0.0, 0.0, 0.0, /*omega*/ 1e-20, 1e-20, 1e-20);
    q.propagateState(1e-20);

    EXPECT_NEAR(q.state(0), 1.0, TOL);
    EXPECT_NEAR(q.state(1), 0.0, TOL);
    EXPECT_NEAR(q.state(2), 0.0, TOL);
    EXPECT_NEAR(q.state(3), 0.0, TOL);
}

TEST(QuaternionStateDataTest, AppliesRotationalDiffusion)
{
    auto q = makeQ(1.0, 0.0, 0.0, 0.0);
    q.setNumNoiseSources(1);

    Eigen::Matrix<double, 3, 1> diffusion = Eigen::Matrix<double, 3, 1>::Zero();
    diffusion(2) = 1.0; // [-]
    q.setDiffusion(diffusion, 0);

    const double rotationAngle = M_PI_2; // [rad]
    q.propagateState(0.0, {rotationAngle});

    EXPECT_NEAR(q.state(0), std::cos(rotationAngle / 2.0), TOL);
    EXPECT_NEAR(q.state(1), 0.0, TOL);
    EXPECT_NEAR(q.state(2), 0.0, TOL);
    EXPECT_NEAR(q.state(3), std::sin(rotationAngle / 2.0), TOL);
}

TEST(QuaternionStateDataTest, RejectsMissingPseudoStepsForStochasticState)
{
    auto q = makeQ(1.0, 0.0, 0.0, 0.0);
    q.setNumNoiseSources(1);

    EXPECT_THROW(q.propagateState(0.0), BasiliskError);
}

TEST(QuaternionStateDataTest, RejectsMismatchedPseudoStepCount)
{
    auto q = makeQ(1.0, 0.0, 0.0, 0.0);
    q.setNumNoiseSources(1);

    EXPECT_THROW(q.propagateState(0.0, {1.0, 2.0}), BasiliskError);
}

TEST(QuaternionStateDataTest, CloneDeepCopiesAndIsIndependent)
{
    auto q = makeQ(0.5, 0.5, 0.5, 0.5, /*omega*/ 0.1, 0.2, 0.3);

    auto cloned = q.clone();
    ASSERT_NE(cloned, nullptr);
    ASSERT_EQ(cloned->state.rows(), 4);
    ASSERT_EQ(cloned->stateDeriv.rows(), 3);

    EXPECT_DOUBLE_EQ(cloned->state(0), q.state(0));
    EXPECT_DOUBLE_EQ(cloned->state(3), q.state(3));
    EXPECT_DOUBLE_EQ(cloned->stateDeriv(1), q.stateDeriv(1));

    // Mutating the original must not touch the clone.
    q.state(0) = 99.0;
    q.stateDeriv(0) = -99.0;
    EXPECT_DOUBLE_EQ(cloned->state(0), 0.5);
    EXPECT_DOUBLE_EQ(cloned->stateDeriv(0), 0.1);
}
