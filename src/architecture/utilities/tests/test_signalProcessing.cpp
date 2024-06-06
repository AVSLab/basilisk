/*
 ISC License

 Copyright (c) 2024, University of Colorado at Boulder

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

#include "architecture/utilities/signalProcessing.h"
#include <numeric>
#include <gtest/gtest.h>
#include "architecture/utilities/tests/random_normal.h"

const double testAccuracy = 1e-8;

double calculateStandardDeviation(const std::vector<double>& input)
{
    double sum = 0.;
    double mean = 0.;
    double standardDeviation = 0.;

    for (double i : input) {
        sum += input[i];
    }

    mean = sum / input.size();
    for (int i = 0; i < input.size(); ++i) {
        standardDeviation += pow(input[i] - mean, 2);
    }

    return std::sqrt(standardDeviation / input.size());
}

// Test low pass filter performance with set of parameters
class GeneralPerformance: public testing::TestWithParam<std::tuple<double, double>> {};

TEST_P(GeneralPerformance, lowPassFilterProperties)
{
    double pi = std::atan(1)*4;
    double noise_bound = 0.1;
    int multipleOfSteps = 10;

    auto [timeStep, omegaCutOff] = GetParam();

    auto testLowPassFilter = LowPassFilter();
    testLowPassFilter.setFilterStep(timeStep);
    testLowPassFilter.setFilterCutoff(omegaCutOff*2*pi);

    EXPECT_TRUE(std::abs(timeStep - testLowPassFilter.getFilterStep()) < testAccuracy);
    EXPECT_TRUE(std::abs(omegaCutOff*2*pi - testLowPassFilter.getFilterCutoff()) < testAccuracy);

    std::vector<double> timeVector;
    std::vector<double> cosValues;
    std::vector<Eigen::Vector3d> residuals;
    for (int step=0; step<multipleOfSteps*int(pi/timeStep); ++step){
        timeVector.push_back(timeStep*step/multipleOfSteps);
        cosValues.push_back(std::cos(2*pi*timeVector.back()) + noise[step]);

        Eigen::Vector3d measurement(cosValues.back(), cosValues.back(), cosValues.back());
        testLowPassFilter.processMeasurement(measurement);

        Eigen::Vector3d state = testLowPassFilter.getCurrentState();
        Eigen::Vector3d truth(std::cos(2*pi*timeVector.back()),
                              std::cos(2*pi*timeVector.back()),
                              std::cos(2*pi*timeVector.back()));
        residuals.push_back(state - truth);
    }

    // Ignoring the first part of the signal with a transient, compare the standard deviation of the remaining
    // noise to the input noise. Despite some low pass signal, it should be below the input noise
    std::vector<Eigen::Vector3d> residualsWithoutTransient = std::vector<Eigen::Vector3d>(residuals.begin() + 100,
                                                                                     residuals.end());
    std::vector<double> normOfResiduals;
    for (auto const& vector : residualsWithoutTransient){
        normOfResiduals.push_back(vector.norm());
    }

    double residualsStandardDev = calculateStandardDeviation(normOfResiduals);
    EXPECT_TRUE(residualsStandardDev < noise_bound);

}

INSTANTIATE_TEST_SUITE_P(
  LowPassFilter,
  GeneralPerformance,
  /* Testing parameters:
   * -timeStep : filter time step
   * -omegaCutOff : rad/s cut off of the filter
   * */
  ::testing::Values(
        std::make_tuple(0.01, 2.5),
        std::make_tuple(0.05, 5),
        std::make_tuple(0.05, 10))
        );
