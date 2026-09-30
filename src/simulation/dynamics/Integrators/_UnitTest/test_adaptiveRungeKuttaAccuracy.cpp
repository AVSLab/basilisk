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

#include "integratorTestAccess.h"
#include "simulation/dynamics/Integrators/svIntegratorRKF45.h"
#include "simulation/dynamics/Integrators/svIntegratorRKF78.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

namespace adaptive_runge_kutta_accuracy_test {

using integrator_step_test::stepIntegrator;

/** @brief Record the evaluation time and trial step size for one RK stage. */
struct StageTime
{
    double time;     //!< [s] Stage evaluation time.
    double timeStep; //!< [s] Trial step size supplied to the dynamics.
};

/** @brief Evolve dimensionless oscillator coordinates in two coupled dynamic objects. */
class OscillatorDynamics : public DynamicObject
{
  public:
    /** @brief Register and finalize the dimensionless oscillator state. */
    OscillatorDynamics()
    {
        this->value = this->dynManager.registerState(2, 3, "oscillator");
        this->dynManager.finalizeStates();
    }

    /** @brief Unused scheduler callback; tests advance the integrator directly. */
    void UpdateState(uint64_t) override {}
    /** @brief No preparation is needed before an integration interval. */
    void preIntegration(uint64_t) override {}
    /** @brief No processing is needed after an integration interval. */
    void postIntegration(uint64_t) override {}

    /** @brief Record the stage and evaluate the coupled oscillator derivative.
     * @param time Stage evaluation time [s].
     * @param timeStep Trial step size [s].
     */
    void equationsOfMotion(double time, double timeStep) override
    {
        this->stages.push_back({ time, timeStep });
        this->value->setDerivative(this->rate * this->partner->value->stateView());
    }

    StateData* value;                      //!< Borrowed handle to the oscillator coordinates.
    OscillatorDynamics* partner = nullptr; //!< Borrowed model providing the coupled coordinates.
    double rate = 1.0;                     //!< [1/s] Signed coupling rate.
    std::vector<StageTime> stages;         //!< Evaluation times and trial step sizes.
};

/** @brief Inspect the real tableau without reproducing its stage times in the test. */
template<typename Integrator>
class AccuracyIntegrator : public Integrator
{
  public:
    using Integrator::Integrator;

    /** @brief Return the stage abscissae from the method's Butcher tableau. */
    const auto& stageTimes() const { return this->coefficients.cArray; }
};

/** @brief Share accuracy checks across the supported adaptive RK methods. */
template<typename Integrator>
class AdaptiveRungeKuttaAccuracy : public testing::Test
{};

/** @brief Adaptive methods exercised by the typed accuracy tests. */
using AdaptiveMethods = testing::Types<svIntegratorRKF45, svIntegratorRKF78>;
/** @brief Provide readable integrator names for typed test results. */
struct AdaptiveMethodNames
{
    /** @brief Identify the adaptive method independently of its type-list index.
     * @return The integrator name used in the test result.
     */
    template<typename Integrator>
    static std::string GetName(int)
    {
        return std::is_same_v<Integrator, svIntegratorRKF45> ? "RKF45" : "RKF78";
    }
};
TYPED_TEST_SUITE(AdaptiveRungeKuttaAccuracy, AdaptiveMethods, AdaptiveMethodNames);

/**
 * @brief Check each trial's timing and rejection behavior independently of another solver's schedule.
 * @param stages Dynamics calls made during one integration interval.
 * @param stageTimes Abscissae of the method's Butcher tableau.
 * @param start Start of the requested integration interval [s].
 * @param duration Length of the requested integration interval [s].
 */
template<typename Abscissae>
void
expectValidTrials(const std::vector<StageTime>& stages, const Abscissae& stageTimes, double start, double duration)
{
    const std::size_t stageCount = stageTimes.size();
    ASSERT_FALSE(stages.empty());
    ASSERT_EQ(stages.size() % stageCount, 0U);
    const double end = start + duration;
    const double timeTolerance = 32.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(end)); // [s]
    EXPECT_NEAR(stages.front().time, start, timeTolerance);
    std::size_t rejected = 0;
    std::size_t accepted = 0;
    for (std::size_t trial = 0; trial < stages.size(); trial += stageCount) {
        const auto& first = stages[trial];
        ASSERT_TRUE(std::isfinite(first.time));
        ASSERT_TRUE(std::isfinite(first.timeStep));
        ASSERT_GT(first.timeStep, 0.0);
        EXPECT_LE(first.time + first.timeStep, end + timeTolerance);
        for (std::size_t stage = 0; stage < stageCount; ++stage) {
            const auto& sample = stages[trial + stage];
            EXPECT_EQ(sample.timeStep, first.timeStep);
            EXPECT_NEAR(sample.time, first.time + stageTimes[stage] * first.timeStep, timeTolerance);
        }
        if (trial + stageCount < stages.size()) {
            const auto& next = stages[trial + stageCount];
            if (next.time == first.time) {
                ++rejected;
                EXPECT_LT(next.timeStep, first.timeStep);
            } else {
                ++accepted;
                EXPECT_NEAR(next.time, first.time + first.timeStep, timeTolerance);
            }
        } else {
            ++accepted;
            EXPECT_NEAR(first.time + first.timeStep, end, timeTolerance);
        }
    }
    EXPECT_GT(rejected, 0U);
    EXPECT_GT(accepted, 1U);
}

/** @brief Tightening tolerances must improve analytic accuracy even when rounding changes the trial schedule. */
TYPED_TEST(AdaptiveRungeKuttaAccuracy, CoupledOscillatorConvergesWithTighterTolerance)
{
    const double duration = 8.0;                                                   // [s]
    const double frequency = 1.0;                                                  // [rad/s]
    const Eigen::MatrixXd initial = Eigen::MatrixXd::Constant(2, 3, 1.0);          // [-]
    const Eigen::MatrixXd exactFirst = initial * std::cos(frequency * duration);   // [-]
    const Eigen::MatrixXd exactSecond = -initial * std::sin(frequency * duration); // [-]
    double previousError = std::numeric_limits<double>::infinity();                // [-]

    for (const double tolerance : { 1e-4, 1e-8 }) { // [-]
        SCOPED_TRACE(tolerance);
        OscillatorDynamics first;
        OscillatorDynamics second;
        first.partner = &second;
        second.partner = &first;
        first.rate = frequency;
        second.rate = -frequency;
        first.value->setState(initial);
        second.value->setState(Eigen::MatrixXd::Zero(2, 3));
        auto* integrator = new AccuracyIntegrator<TypeParam>(&first);
        first.setIntegrator(integrator);
        second.setIntegrator(new TypeParam(&second));
        first.syncDynamicsIntegration(&second);
        integrator->setRelativeTolerance(tolerance);
        integrator->setAbsoluteTolerance(tolerance * 0.01); // [-]
        stepIntegrator(integrator, 0.0, duration);          // [s]

        const auto finalFirst = first.value->stateView();
        const auto finalSecond = second.value->stateView();
        ASSERT_TRUE(finalFirst.allFinite());
        ASSERT_TRUE(finalSecond.allFinite());
        const double error =
          std::max((finalFirst - exactFirst).cwiseAbs().maxCoeff(), (finalSecond - exactSecond).cwiseAbs().maxCoeff());
        if (tolerance == 1e-8) {
            // These are global-error bounds for this eight-second trajectory,
            // rather than an assertion that local tolerances bound global error.
            EXPECT_LT(error, 1e-6); // [-]
            EXPECT_LT(error, previousError * 0.01);
        }
        previousError = error;
        expectValidTrials(first.stages, integrator->stageTimes(), 0.0, duration);  // [s]
        expectValidTrials(second.stages, integrator->stageTimes(), 0.0, duration); // [s]
    }
}

/** @brief Bundle a constant large component with a decaying small component. */
class MixedScaleDynamics : public DynamicObject
{
  public:
    /** @brief Register a mixed-scale state with per-component error control. */
    MixedScaleDynamics()
    {
        StateSpec spec;
        spec.state = { 2, 1 };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        spec.errorControl = ErrorControlMode::PerComponent;
        this->value = this->dynManager.registerState("mixed", spec);
        const Eigen::Vector2d initial{ 1e9, 1.0 }; // [-]
        this->value->setState(initial);
        this->dynManager.finalizeStates();
    }

    /** @brief Unused scheduler callback; tests advance the integrator directly. */
    void UpdateState(uint64_t) override {}
    /** @brief No preparation is needed before an integration interval. */
    void preIntegration(uint64_t) override {}
    /** @brief No processing is needed after an integration interval. */
    void postIntegration(uint64_t) override {}

    /** @brief Record the stage and decay only the smaller state component.
     * @param time Stage evaluation time [s].
     * @param timeStep Trial step size [s].
     */
    void equationsOfMotion(double time, double timeStep) override
    {
        this->stages.push_back({ time, timeStep });
        const double decayRate = -1.0;                                                    // [1/s]
        const Eigen::Vector2d derivative{ 0.0, decayRate * this->value->stateView()(1) }; // [1/s]
        this->value->setDerivative(derivative);
    }

    StateData* value;              //!< Borrowed handle to the mixed-scale coordinates.
    std::vector<StageTime> stages; //!< Evaluation times and trial step sizes.
};

/** @brief Componentwise tolerances must preserve the accuracy of a small state under optimized arithmetic. */
TYPED_TEST(AdaptiveRungeKuttaAccuracy, ComponentwiseControlPreservesSmallStateAccuracy)
{
    MixedScaleDynamics model;
    AccuracyIntegrator<TypeParam> integrator(&model);
    integrator.setRelativeTolerance(1e-8);               // [-]
    integrator.setAbsoluteTolerance(1e-10);              // [-]
    const double duration = 4.0;                         // [s]
    const double decayRate = -1.0;                       // [1/s]
    const double exact = std::exp(decayRate * duration); // [-]
    stepIntegrator(integrator, 0.0, duration);           // [s]
    const auto final = model.value->stateView();
    ASSERT_TRUE(final.allFinite());
    EXPECT_DOUBLE_EQ(final(0), 1e9);
    EXPECT_NEAR(final(1), exact, 1e-6 * exact);
    expectValidTrials(model.stages, integrator.stageTimes(), 0.0, duration); // [s]
}

} // namespace adaptive_runge_kutta_accuracy_test
