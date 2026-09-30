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

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <random>

#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorRungeKutta.h"

namespace {
class AccuracyDynamics final : public DynamicObject
{
  public:
    explicit AccuracyDynamics(unsigned int scalarCount)
    {
        this->dynManager.registerState(scalarCount, 1, "accuracyState");
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}
    void equationsOfMotion(double, double) override {}
};

template<size_t numberStages>
class AccuracyProbe final : public svIntegratorRungeKutta<numberStages>
{
  public:
    explicit AccuracyProbe(DynamicObject* dynamics)
      : svIntegratorRungeKutta<numberStages>(dynamics, RKCoefficients<numberStages>{})
    {
        this->bindFlatStorage();
    }

    void checkCandidates(std::mt19937_64& generator, bool cancellation)
    {
        std::uniform_real_distribution<double> distribution(-2.0, 2.0);
        for (Eigen::Index index = 0; index < this->baseState.size(); ++index) {
            this->baseState(index) = index % 2 == 0 ? 0.0 : distribution(generator);
            for (size_t stage = 0; stage < numberStages; ++stage) {
                this->kStorage(index, static_cast<Eigen::Index>(stage)) =
                  std::ldexp(distribution(generator), static_cast<int>(index % 9) - 4);
            }
        }
        std::array<double, numberStages> weights{};
        for (size_t stage = 0; stage < numberStages; ++stage) {
            weights[stage] = stage == 2 ? 0.0 : (stage % 2 == 0 ? 1.0 : -1.0) / static_cast<double>(stage + 3);
        }
        if (cancellation) {
            weights[1] = -weights[0];
            this->kStorage.col(1) = this->kStorage.col(0);
        }

        const double timeStep = 0.137; // [s]
        for (size_t maxStage : std::array<size_t, 4>{ { 0, 1, 2, numberStages } }) {
            SCOPED_TRACE(maxStage);
            this->buildFlatCandidate(timeStep, weights, maxStage, this->candidateState);
            for (Eigen::Index index = 0; index < this->baseState.size(); ++index) {
                long double combined = 0.0L;
                long double magnitude = 0.0L;
                for (size_t stage = 0; stage < maxStage; ++stage) {
                    const long double term =
                      static_cast<long double>(this->kStorage(index, static_cast<Eigen::Index>(stage))) *
                      static_cast<long double>(weights[stage]);
                    combined += term;
                    magnitude += std::abs(term);
                }
                const long double base = this->baseState(index);
                const long double expected = base + timeStep * combined;
                const long double scale = std::abs(base) + timeStep * magnitude;
                // Scale by the absolute terms, so cancellation cannot hide a
                // legitimate roundoff difference behind a near-zero result.
                // The bound covers products, sums, the final update, and the
                // reference calculation where long double has double precision.
                const long double tolerance =
                  8.0L * static_cast<long double>(maxStage + 1) * std::numeric_limits<double>::epsilon() * scale;
                EXPECT_LE(std::abs(static_cast<long double>(this->candidateState(index)) - expected), tolerance)
                  << "scalar " << index;
            }
        }
    }
};

template<size_t numberStages>
void
checkAccuracy()
{
    std::mt19937_64 generator(123);
    for (unsigned int scalarCount : { 1U, 3U, 18U, 65U }) {
        SCOPED_TRACE(scalarCount);
        AccuracyDynamics dynamics(scalarCount);
        AccuracyProbe<numberStages> integrator(&dynamics);
        for (size_t trial = 0; trial < 16; ++trial) {
            SCOPED_TRACE(trial);
            integrator.checkCandidates(generator, trial % 2 != 0);
        }
    }
}
}

TEST(RungeKuttaAccuracy, FourStageCandidatesSatisfyRoundoffBound)
{
    checkAccuracy<4>();
}

TEST(RungeKuttaAccuracy, ThirteenStageCandidatesSatisfyRoundoffBound)
{
    checkAccuracy<13>();
}
