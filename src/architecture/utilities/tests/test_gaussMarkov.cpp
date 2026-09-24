/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "architecture/utilities/gauss_markov.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <random>
#include <stdint.h>


Eigen::Vector2d calculateSD(const Eigen::MatrixXd& dat, int64_t numPts)
{
    // Calculate mean properly
    Eigen::Vector2d means = dat.rowwise().mean();

    // Calculate variance using numerically stable algorithm
    Eigen::Vector2d variance = Eigen::Vector2d::Zero();
    for(int64_t i = 0; i < numPts; i++) {
        Eigen::Vector2d diff = dat.col(i) - means;
        variance += diff.cwiseProduct(diff);
    }
    variance /= static_cast<double>(numPts - 1);  // Use n-1 for sample standard deviation

    return variance.cwiseSqrt();
}

TEST(GaussMarkov, stdDeviationIsExpected) {
    uint64_t seedIn = 1000;

    // Use proper Gauss-Markov propagation matrix (with decay)
    Eigen::Matrix2d propIn;
    propIn << 0.9,0,0,0.9;  // Decay factor of 0.9

    // Set noise matrix for std=1.0 in steady state
    // For an autoregressive process of order 1 with coefficient a,
    // steady state variance is: sigma^2/(1-a^2)
    // So for a=0.9, noise variance should be (1-0.81) = 0.19 to get std=1.0
    Eigen::Matrix2d noiseMatrix;
    noiseMatrix << sqrt(0.19),0,0,sqrt(0.19);

    Eigen::Vector2d bounds;
    bounds << 100.0, 100.0;  // Large bounds to avoid affecting distribution

    GaussMarkov errorModel = GaussMarkov(2);
    errorModel.setRNGSeed(seedIn);
    errorModel.setPropMatrix(propIn);
    errorModel.setNoiseMatrix(noiseMatrix);
    errorModel.setUpperBounds(bounds);

    int64_t numPts = 20000;
    Eigen::MatrixXd noiseOut;
    noiseOut.resize(2, numPts);

    // Warm up for many times the process correlation time.
    for(int64_t i = 0; i < 500; i++) {
        errorModel.computeNextState();
    }

    // Collect samples
    for(int64_t i = 0; i < numPts; i++) {
        errorModel.computeNextState();
        noiseOut.block(0, i, 2, 1) = errorModel.getCurrentState();
    }

    Eigen::Vector2d stds = calculateSD(noiseOut, numPts);

    // Test with appropriate statistical tolerances
    EXPECT_NEAR(stds(0), 1.0, 0.1);  // Within 10% of expected std=1.0
    EXPECT_NEAR(stds(1), 1.0, 0.1);
}

TEST(GaussMarkov, meanIsZero) {
    uint64_t seedIn = 1000;
    Eigen::Matrix2d propIn;
    propIn << 1,0,0,1;

    // Set noise matrix as sqrt of covariance for std=1.0
    Eigen::Matrix2d noiseMatrix;
    noiseMatrix << 1.0,0,0,1.0;  // Square root of unit covariance

    Eigen::Vector2d bounds;
    bounds << 1e-15, 1e-15;

    GaussMarkov errorModel = GaussMarkov(2);
    errorModel.setRNGSeed(seedIn);
    errorModel.setPropMatrix(propIn);
    errorModel.setNoiseMatrix(noiseMatrix);
    errorModel.setUpperBounds(bounds);

    int64_t numPts = 10000;
    Eigen::MatrixXd noiseOut;
    noiseOut.resize(2, numPts);

    // Warm up
    for(int64_t i = 0; i < 1000; i++) {
        errorModel.computeNextState();
    }

    // Collect samples
    for(int64_t i = 0; i < numPts; i++) {
        errorModel.computeNextState();
        noiseOut.block(0, i, 2, 1) = errorModel.getCurrentState();
    }

    Eigen::Vector2d means = noiseOut.rowwise().mean();

    // Test with appropriate statistical tolerances
    double tol = 4.0 / sqrt(numPts);  // 4-sigma confidence interval
    EXPECT_LT(fabs(means(0)), tol);
    EXPECT_LT(fabs(means(1)), tol);
}

TEST(GaussMarkov, boundsAreRespected) {
    uint64_t seedIn = 1500;
    Eigen::Matrix2d propIn;
    propIn << 1,0,0,1;

    // Adjust covariance matrix for std=1.0
    Eigen::Matrix2d covar;
    covar << 0.5,0,0,0.005;

    Eigen::Vector2d bounds;
    bounds << 10., 0.1;
    GaussMarkov errorModel = GaussMarkov(2);
    errorModel.setRNGSeed(seedIn);
    errorModel.setPropMatrix(propIn);
    errorModel.setNoiseMatrix(covar);
    errorModel.setUpperBounds(bounds);

    int64_t numPts = 10000;
    Eigen::MatrixXd noiseOut;
    noiseOut.resize(2, numPts);

    Eigen::Vector2d maxOut;
    maxOut.fill(0.0);
    Eigen::Vector2d minOut;
    minOut.fill(0.0);

    for(int64_t i = 0; i < numPts; i++){
        errorModel.computeNextState();
        noiseOut.block(0, i, 2, 1) = errorModel.getCurrentState();
        if (noiseOut(0,i) > maxOut(0)){
            maxOut(0) = noiseOut(0,i);
        }
        if (noiseOut(0,i) < minOut(0)){
            minOut(0) = noiseOut(0,i);
        }
        if (noiseOut(1,i) > maxOut(1)){
            maxOut(1) = noiseOut(1,i);
        }
        if (noiseOut(1,i) < minOut(1)){
            minOut(1) = noiseOut(1,i);
        }
    }

    // Adjust expected bounds for std=1.0
    EXPECT_LT(fabs(11.0 - maxOut(0)) / 11.0, 5e-1);
    EXPECT_LT(fabs(0.11 - maxOut(1)) / 0.11, 5e-1);
    EXPECT_LT(fabs(-11.0 - minOut(0)) / -11.0, 5e-1);
    EXPECT_LT(fabs(-0.11 - minOut(1)) / -0.11, 5e-1);
}

TEST(GaussMarkov, gaussianOnlyMode) {
    uint64_t seedIn = 1000;

    // Setup matrices for pure Gaussian noise
    Eigen::Matrix2d propIn;
    propIn << 0.0,0,0,0.0;  // No state propagation

    Eigen::Matrix2d noiseMatrix;
    noiseMatrix << 1.0,0,0,1.0;  // Unit variance

    Eigen::Vector2d bounds;
    bounds << -1.0, -1.0;  // Disable bounds/random walk

    GaussMarkov errorModel = GaussMarkov(2);
    errorModel.setRNGSeed(seedIn);
    errorModel.setPropMatrix(propIn);
    errorModel.setNoiseMatrix(noiseMatrix);
    errorModel.setUpperBounds(bounds);

    // Collect samples and verify standard normal distribution properties
    int64_t numPts = 10000;
    Eigen::MatrixXd samples;
    samples.resize(2, numPts);

    for(int64_t i = 0; i < numPts; i++) {
        errorModel.computeNextState();
        samples.col(i) = errorModel.getCurrentState();
    }

    // Calculate statistics
    Eigen::Vector2d means = samples.rowwise().mean();
    Eigen::Vector2d stds = calculateSD(samples, numPts);

    // Test mean is ~0 and std is ~1
    EXPECT_NEAR(means(0), 0.0, 0.1);
    EXPECT_NEAR(means(1), 0.0, 0.1);
    EXPECT_NEAR(stds(0), 1.0, 0.1);
    EXPECT_NEAR(stds(1), 1.0, 0.1);
}

TEST(GaussMarkov, reseedingRestartsSequence)
{
    constexpr uint64_t seed = 1000;
    // An odd state count leaves one normal variate cached between updates.
    GaussMarkov errorModel(3);
    errorModel.setNoiseMatrix(Eigen::Matrix3d::Identity());
    errorModel.setPropMatrix(Eigen::Matrix3d::Identity());
    errorModel.setRNGSeed(seed);

    errorModel.computeNextState();
    const Eigen::Vector3d firstSample = errorModel.getCurrentState();

    errorModel.setRNGSeed(seed);
    errorModel.computeNextState();
    const Eigen::Vector3d replayedSample = errorModel.getCurrentState();

    for (Eigen::Index i = 0; i < firstSample.size(); i++) {
        EXPECT_DOUBLE_EQ(replayedSample[i], firstSample[i]);
    }
}

TEST(GaussMarkov, secondarySeedDoesNotAliasZero)
{
    constexpr uint64_t baseSeed = 0;
    const uint64_t secondarySeed = GaussMarkov::deriveSecondarySeed(baseSeed);
    std::minstd_rand baseGenerator(static_cast<std::minstd_rand::result_type>(baseSeed));
    std::minstd_rand secondaryGenerator(static_cast<std::minstd_rand::result_type>(secondarySeed));

    EXPECT_NE(baseGenerator(), secondaryGenerator());
}

TEST(GaussMarkov, secondarySeedIsPlatformIndependent)
{
    //! - std::minstd_rand::result_type is std::uint_fast32_t, whose width is implementation
    //!   defined.  A derived seed carrying bits above 32 would therefore seed the secondary
    //!   engine differently depending on the standard library Basilisk was built against.
    constexpr uint64_t defaultSeed = 0x1badcad1;
    constexpr uint64_t expectedSecondarySeed = 0x64e7b6c4;
    constexpr uint64_t expectedFirstDraw = 128424993;
    constexpr uint64_t seedWidthMask = 0xFFFFFFFFULL;

    const uint64_t secondarySeed = GaussMarkov::deriveSecondarySeed(defaultSeed);
    EXPECT_EQ(secondarySeed, expectedSecondarySeed);

    std::minstd_rand secondaryGenerator(static_cast<std::minstd_rand::result_type>(secondarySeed));
    EXPECT_EQ(static_cast<uint64_t>(secondaryGenerator()), expectedFirstDraw);

    //! - The normalization must hold for any base seed, including one that fills all 64 bits.
    const uint64_t probeSeeds[] = {0, 1, defaultSeed, 0x7FFFFFFFULL, 0xFFFFFFFFFFFFFFFFULL};
    for (const uint64_t probeSeed : probeSeeds) {
        EXPECT_LE(GaussMarkov::deriveSecondarySeed(probeSeed), seedWidthMask);
    }
}

TEST(GaussMarkov, secondarySeedSeparatesBothEngineWidths)
{
    // Exercise both platform seed widths even when the native engine uses only one.
    using Engine32 = std::linear_congruential_engine<uint32_t, 48271, 0, 2147483647>;
    using Engine64 = std::linear_congruential_engine<uint64_t, 48271, 0, 2147483647>;
    const uint64_t probeSeeds[] = {
        0, 1, 0x1badcad1, 0x7FFFFFFFULL, 0x80000000ULL, 0xFFFFFFFFULL,
        0x100000000ULL, 0xFFFFFFFFFFFFFFFFULL,
        0x7fa53e0900000001ULL,  // The review reproducer aliases the full-width primary state.
        0x005ac1f57f4a7c15ULL,  // Both full-width primary and candidate seeds reduce to zero.
        0x005ac1f67f4a7c14ULL,  // Both full-width primary and candidate seeds reduce to one.
    };
    for (const uint64_t baseSeed : probeSeeds) {
        SCOPED_TRACE(::testing::Message() << "baseSeed = " << baseSeed);
        const uint64_t secondarySeed = GaussMarkov::deriveSecondarySeed(baseSeed);
        ASSERT_LE(secondarySeed, 0xFFFFFFFFULL);
        EXPECT_EQ(secondarySeed, GaussMarkov::deriveSecondarySeed(baseSeed));

        const Engine32 primary32(static_cast<uint32_t>(baseSeed));
        const Engine64 primary64(baseSeed);
        Engine32 secondary32(static_cast<uint32_t>(secondarySeed));
        Engine64 secondary64(secondarySeed);
        EXPECT_NE(primary32, secondary32);
        EXPECT_NE(primary64, secondary64);
        EXPECT_EQ(secondary32(), secondary64());
    }

    // Pin the collision fallback so every platform selects the same secondary stream.
    EXPECT_EQ(GaussMarkov::deriveSecondarySeed(0x7fa53e0900000001ULL), 1304083119ULL);
}
