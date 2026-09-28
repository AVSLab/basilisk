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

#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include "simulation/dynamics/_GeneralModuleFiles/extendedStateVector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorRungeKutta.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorAdaptiveRungeKutta.h"
#include "simulation/dynamics/Integrators/svIntegratorRKF45.h"
#include "simulation/dynamics/Integrators/svIntegratorRKF78.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#if !defined(EIGEN_RUNTIME_NO_MALLOC) || defined(EIGEN_NO_DEBUG)
#error "Cache reuse tests require Eigen runtime allocation checks with assertions enabled."
#endif

namespace {

/** @brief Return the classical four-stage RK4 tableau. */
RKCoefficients<4>
rk4Coefficients()
{
    RKCoefficients<4> coefficients;
    coefficients.aMatrix[1][0] = 0.5;
    coefficients.aMatrix[2][1] = 0.5;
    coefficients.aMatrix[3][2] = 1.0;
    coefficients.bArray = { 1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0 };
    coefficients.cArray = { 0.0, 0.5, 0.5, 1.0 };
    return coefficients;
}

/** @brief Restore the pre-cache integration sequence using the retained map-based helpers. */
template<std::size_t stages>
class ReferenceIntegrator : public svIntegratorRungeKutta<stages>
{
  public:
    using svIntegratorRungeKutta<stages>::svIntegratorRungeKutta;

    void integrate(double currentTime, double timeStep) override
    {
        const auto initial = ExtendedStateVector::fromStates(this->dynPtrs);
        const auto derivatives = this->computeKCoefficients(currentTime, timeStep, initial);
        const auto final =
          this->propagateStateWithKVectors(timeStep, initial, derivatives, this->coefficients->bArray, stages);
        final.setStates(this->dynPtrs);
    }
};

/**
 * @brief Test a redundant two-component state advanced by a one-component derivative.
 *
 * This exercises the StateData extension contract used by MuJoCo, where state
 * and derivative dimensions can differ and propagation must use virtual methods.
 * It requires no optional dynamics engine. Both components are dimensionless.
 */
class MappedState : public StateData
{
  public:
    MappedState(const std::string& name, const Eigen::MatrixXd& initial)
      : StateData(name, initial)
    {
        this->stateDeriv = Eigen::MatrixXd::Zero(1, 1);
    }

    void setDerivative(const Eigen::MatrixXd& derivative) override
    {
        ASSERT_EQ(derivative.rows(), 1);
        ASSERT_EQ(derivative.cols(), 1);
        ++this->derivativeCalls;
        StateData::setDerivative(derivative);
    }

    void propagateState(double timeStep, std::vector<double> pseudoStep = {}) override
    {
        ASSERT_TRUE(pseudoStep.empty());
        ASSERT_EQ(this->state.rows(), 2);
        ASSERT_EQ(this->state.cols(), 1);
        ASSERT_EQ(this->stateDeriv.rows(), 1);
        ++this->propagationCalls;
        this->state(0, 0) += timeStep * this->stateDeriv(0, 0);
        this->state(1, 0) += 2.0 * timeStep * this->stateDeriv(0, 0);
    }

    std::size_t derivativeCalls = 0;
    std::size_t propagationCalls = 0;
};

struct StageSample
{
    double time;                                   //!< [s] Time passed to the dynamics evaluation.
    double timeStep;                               //!< [s] Step size passed to the dynamics evaluation.
    std::map<std::string, Eigen::MatrixXd> states; //!< [-] Values seen by the dynamics at this stage.
};

/** @brief Unscheduled test double with matrix states and time-dependent, coupled dynamics. */
class TestDynamics : public DynamicObject
{
  public:
    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double time, double step) override
    {
        StageSample sample{ time, step, {} };
        double coupledValue = 0.0; // [-]
        if (this->partner) {
            for (const auto& entry : this->partner->dynManager.stateContainer.stateMap) {
                coupledValue += entry.second->getStateReference().sum();
            }
        }
        const double decayRate = -0.25;     // [1/s]
        const double couplingRate = 0.0625; // [1/s]
        const double rampRate = 0.03125;    // [1/s^2]
        const double bias = 0.125;          // [1/s]
        const double forcing = bias + rampRate * time + couplingRate * coupledValue;
        for (auto& entry : this->dynManager.stateContainer.stateMap) {
            StateData& data = *entry.second;
            const auto& value = data.getStateReference();
            sample.states.emplace(entry.first, value);
            Eigen::MatrixXd derivative;
            if (dynamic_cast<MappedState*>(&data)) {
                derivative = Eigen::MatrixXd::Constant(1, 1, decayRate * value(0, 0) + forcing);
            } else {
                derivative = (decayRate * value.array() + forcing).matrix();
            }
            if (this->poisonFirstStage && this->samples.empty()) {
                derivative.setConstant(std::numeric_limits<double>::quiet_NaN());
            }
            data.setDerivative(derivative);
        }
        this->samples.push_back(std::move(sample));
    }

    TestDynamics* partner = nullptr;
    bool poisonFirstStage = false;
    std::vector<StageSample> samples;
};

/** @brief Build a dimensionless matrix whose entries distinguish both rows and columns. */
Eigen::MatrixXd
makeValues(Eigen::Index rows, Eigen::Index columns, double offset)
{
    Eigen::MatrixXd values(rows, columns);
    for (Eigen::Index column = 0; column < columns; ++column) {
        for (Eigen::Index row = 0; row < rows; ++row) {
            values(row, column) = offset + 0.125 * static_cast<double>(row) - 0.0625 * static_cast<double>(column);
        }
    }
    return values;
}

/** @brief Forbid Eigen matrix allocations within a scope and restore the previous allocation policy. */
class EigenAllocationGuard
{
  public:
    EigenAllocationGuard()
      : previouslyAllowed(Eigen::internal::is_malloc_allowed())
    {
        Eigen::internal::set_is_malloc_allowed(false);
    }

    ~EigenAllocationGuard() { Eigen::internal::set_is_malloc_allowed(this->previouslyAllowed); }

    EigenAllocationGuard(const EigenAllocationGuard&) = delete;
    EigenAllocationGuard& operator=(const EigenAllocationGuard&) = delete;

  private:
    bool previouslyAllowed;
};

/** @brief Evaluate dimensionless matrix dynamics using a preallocated derivative buffer and no stage recording. */
class AllocationFreeDynamics : public DynamicObject
{
  public:
    AllocationFreeDynamics(Eigen::Index rows, Eigen::Index columns)
      : derivative(rows, columns)
    {
        this->value =
          this->dynManager.registerState(static_cast<uint32_t>(rows), static_cast<uint32_t>(columns), "value");
        this->value->setState(makeValues(rows, columns, 0.375)); // [-]
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double time, double) override
    {
        const double decayRate = -0.25;  // [1/s]
        const double rampRate = 0.03125; // [1/s^2]
        this->derivative = decayRate * this->value->getStateReference();
        this->derivative.array() += rampRate * time;
        this->value->setDerivative(this->derivative);
    }

    StateData* value;

  private:
    Eigen::MatrixXd derivative;
};

/** @brief Register a dimensionless matrix state with a deterministic initial value. */
void
addState(TestDynamics& model, const std::string& name, const Eigen::MatrixXd& values)
{
    auto* state =
      model.dynManager.registerState(static_cast<uint32_t>(values.rows()), static_cast<uint32_t>(values.cols()), name);
    state->setState(values);
}

/** @brief Require matching shapes and agreement to floating-point roundoff, rejecting non-finite results. */
void
expectMatrixMatches(const Eigen::MatrixXd& actual, const Eigen::MatrixXd& expected)
{
    ASSERT_EQ(actual.rows(), expected.rows());
    ASSERT_EQ(actual.cols(), expected.cols());
    ASSERT_TRUE(actual.allFinite());
    ASSERT_TRUE(expected.allFinite());
    for (Eigen::Index index = 0; index < actual.size(); ++index) {
        // Allow compiler-dependent contraction while remaining sensitive to incorrect stage combinations.
        const double tolerance =
          32.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(expected(index)));
        EXPECT_NEAR(actual(index), expected(index), tolerance) << "coefficient " << index;
    }
}

/** @brief Compare every stage input as well as final values and derivatives. */
void
expectDynamicsMatch(const TestDynamics& actual, const TestDynamics& expected)
{
    const auto& actualStates = actual.dynManager.stateContainer.stateMap;
    const auto& expectedStates = expected.dynManager.stateContainer.stateMap;
    ASSERT_EQ(actualStates.size(), expectedStates.size());
    for (const auto& entry : expectedStates) {
        SCOPED_TRACE(entry.first);
        ASSERT_EQ(actualStates.count(entry.first), 1U);
        expectMatrixMatches(actualStates.at(entry.first)->getStateReference(), entry.second->getStateReference());
        expectMatrixMatches(actualStates.at(entry.first)->getStateDerivReference(),
                            entry.second->getStateDerivReference());
    }
    ASSERT_EQ(actual.samples.size(), expected.samples.size());
    for (std::size_t stage = 0; stage < expected.samples.size(); ++stage) {
        SCOPED_TRACE(stage);
        const auto& actualSample = actual.samples.at(stage);
        const auto& expectedSample = expected.samples.at(stage);
        EXPECT_EQ(actualSample.time, expectedSample.time);
        EXPECT_EQ(actualSample.timeStep, expectedSample.timeStep);
        ASSERT_EQ(actualSample.states.size(), expectedSample.states.size());
        for (const auto& entry : expectedSample.states) {
            SCOPED_TRACE(entry.first);
            ASSERT_EQ(actualSample.states.count(entry.first), 1U);
            expectMatrixMatches(actualSample.states.at(entry.first), entry.second);
        }
    }
}

/** @brief Own independent model pairs so the cached and reference integrators cannot share state storage. */
template<std::size_t stages>
struct Comparison
{
    explicit Comparison(const RKCoefficients<stages>& coefficients)
      : cached(&cachedPrimary, coefficients)
      , reference(&referencePrimary, coefficients)
    {
        for (auto* primary : { &cachedPrimary, &referencePrimary }) {
            addState(*primary, "scalar", makeValues(1, 1, 0.375)); // [-]
            addState(*primary, "matrix", makeValues(2, 3, -0.5));  // [-]
        }
        for (auto* secondary : { &cachedSecondary, &referenceSecondary }) {
            // Reuse names across objects, with different values and matrix shapes.
            addState(*secondary, "scalar", makeValues(1, 1, -0.75)); // [-]
            addState(*secondary, "matrix", makeValues(3, 2, 0.625)); // [-]
        }
        cachedPrimary.partner = &cachedSecondary;
        cachedSecondary.partner = &cachedPrimary;
        referencePrimary.partner = &referenceSecondary;
        referenceSecondary.partner = &referencePrimary;
        cached.dynPtrs.push_back(&cachedSecondary);
        reference.dynPtrs.push_back(&referenceSecondary);
    }

    void step(double time, double stepSize)
    {
        cachedPrimary.samples.clear();
        cachedSecondary.samples.clear();
        referencePrimary.samples.clear();
        referenceSecondary.samples.clear();
        cached.integrate(time, stepSize);
        reference.integrate(time, stepSize);
        for (auto* model : { &cachedPrimary, &cachedSecondary }) {
            const bool active = std::find(cached.dynPtrs.begin(), cached.dynPtrs.end(), model) != cached.dynPtrs.end();
            EXPECT_EQ(model->samples.size(), active ? stages : 0U);
        }
        expectDynamicsMatch(cachedPrimary, referencePrimary);
        expectDynamicsMatch(cachedSecondary, referenceSecondary);
    }

    void repeatedSteps()
    {
        double time = 0.25; // [s]
        for (int iteration = 0; iteration < 8; ++iteration) {
            SCOPED_TRACE(iteration);
            const double stepSize = iteration % 2 == 0 ? 0.125 : 0.0625; // [s]
            this->step(time, stepSize);
            time += stepSize;
        }
    }

    TestDynamics cachedPrimary;
    TestDynamics cachedSecondary;
    TestDynamics referencePrimary;
    TestDynamics referenceSecondary;
    svIntegratorRungeKutta<stages> cached;
    ReferenceIntegrator<stages> reference;
};

/** @brief Compare Euler across repeated steps, coupled objects, and heterogeneous state shapes. */
TEST(RungeKuttaCache, EulerMatchesReference)
{
    RKCoefficients<1> coefficients;
    coefficients.bArray = { 1.0 };
    Comparison<1> comparison(coefficients);
    comparison.repeatedSteps();
}

/** @brief Compare the trapezoidal RK2 tableau used by svIntegratorRK2. */
TEST(RungeKuttaCache, RK2MatchesReference)
{
    RKCoefficients<2> coefficients;
    coefficients.aMatrix[1] = { 1.0, 1.0 };
    coefficients.bArray = { 0.5, 0.5 };
    coefficients.cArray = { 0.0, 1.0 };
    Comparison<2> comparison(coefficients);
    comparison.repeatedSteps();
}

/** @brief Compare classical RK4, including cache reuse and changing step sizes. */
TEST(RungeKuttaCache, RK4MatchesReference)
{
    Comparison<4> comparison(rk4Coefficients());
    comparison.repeatedSteps();
}

/** @brief Warm integrations must reuse scratch matrices, including after a live layout change. */
TEST(RungeKuttaCache, ReusesScratchMatricesAfterWarmup)
{
    AllocationFreeDynamics primary(2, 3);
    AllocationFreeDynamics secondary(3, 2);
    svIntegratorRungeKutta<4> integrator(&primary, rk4Coefficients());
    double time = 0.0;           // [s]
    const double warmup = 0.125; // [s]

    const auto integrateWithoutAllocations = [&]() {
        // This guard covers Eigen allocations in the header-defined integrator;
        // separately compiled dynamics libraries are not instrumented by this target.
        EigenAllocationGuard guard;
        for (int iteration = 0; iteration < 8; ++iteration) {
            const double stepSize = iteration % 2 == 0 ? 0.125 : 0.0625; // [s]
            integrator.integrate(time, stepSize);
            time += stepSize;
        }
    };

    integrator.integrate(time, warmup);
    time += warmup;
    integrateWithoutAllocations();

    // Permit cache rebuilding for the new object, then require allocation-free reuse again.
    integrator.dynPtrs.push_back(&secondary);
    integrator.integrate(time, warmup);
    time += warmup;
    integrateWithoutAllocations();

    EXPECT_TRUE(primary.value->getStateReference().allFinite());
    EXPECT_TRUE(secondary.value->getStateReference().allFinite());
}

/** @brief Exercise negative intermediate and final weights, with zero weights between nonzero terms. */
TEST(RungeKuttaCache, NegativeAndSparseCoefficientsMatchReference)
{
    RKCoefficients<3> coefficients;
    coefficients.aMatrix[1] = { 0.5, 0.0, 0.0 };
    coefficients.aMatrix[2] = { -1.0, 2.0, 0.0 };
    coefficients.bArray = { -0.25, 0.0, 1.25 };
    coefficients.cArray = { 0.0, 0.5, 1.0 };
    Comparison<3> comparison(coefficients);
    comparison.repeatedSteps();
}

/** @brief A zero final combination restores the initial state even after nontrivial intermediate stages. */
TEST(RungeKuttaCache, ZeroFinalWeightsRestoreInitialState)
{
    auto coefficients = rk4Coefficients();
    coefficients.bArray = {};
    Comparison<4> comparison(coefficients);
    const auto initial = ExtendedStateVector::fromStates(comparison.cached.dynPtrs);
    comparison.repeatedSteps();
    const auto final = ExtendedStateVector::fromStates(comparison.cached.dynPtrs);
    for (const auto& entry : initial) {
        expectMatrixMatches(final.at(entry.first), entry.second);
    }
}

/** @brief Skipped zero weights must not introduce NaNs from an unused stage, even as the first final term. */
TEST(RungeKuttaCache, ZeroWeightsSkipNonFiniteDerivative)
{
    RKCoefficients<2> coefficients;
    coefficients.bArray = { 0.0, 1.0 };
    coefficients.cArray = { 0.0, 0.5 };
    Comparison<2> comparison(coefficients);
    for (auto* model : { &comparison.cachedPrimary,
                         &comparison.cachedSecondary,
                         &comparison.referencePrimary,
                         &comparison.referenceSecondary }) {
        model->poisonFirstStage = true;
    }
    comparison.repeatedSteps();
}

enum class CacheChange
{
    insertState,
    appendState,
    removeState,
    replaceState,
    resizeState,
    reshapeState,
    clearStates,
    resetValues,
    addObject,
    removeObject,
    reorderObjects
};

class RungeKuttaCacheChange : public testing::TestWithParam<CacheChange>
{};

/** @brief Reuse a warm integrator after changes to live states or dynamic objects, then reuse the refreshed cache. */
TEST_P(RungeKuttaCacheChange, MatchesReferenceAfterChange)
{
    Comparison<4> comparison(rk4Coefficients());
    const auto change = GetParam();
    if (change == CacheChange::addObject) {
        comparison.cached.dynPtrs.pop_back();
        comparison.reference.dynPtrs.pop_back();
    }
    comparison.repeatedSteps();

    // Keep removed objects alive so stale cache entries fail comparisons rather
    // than dereferencing freed storage. Replacement cannot reuse the old address.
    std::vector<std::unique_ptr<StateData>> retired;
    for (auto* model : { &comparison.cachedPrimary, &comparison.referencePrimary }) {
        auto& states = model->dynManager.stateContainer.stateMap;
        switch (change) {
            case CacheChange::insertState:
                addState(*model, "added", makeValues(4, 1, 0.25)); // [-]
                break;
            case CacheChange::appendState:
                addState(*model, "zzAdded", makeValues(1, 4, -0.25)); // [-]
                break;
            case CacheChange::removeState:
                retired.push_back(std::move(states.at("scalar")));
                states.erase("scalar");
                break;
            case CacheChange::replaceState:
                retired.push_back(std::move(states.at("matrix")));
                states.at("matrix") = std::make_unique<StateData>("matrix", makeValues(2, 3, 0.875)); // [-]
                break;
            case CacheChange::resizeState:
                states.at("matrix")->setState(makeValues(4, 1, -0.125)); // [-]
                break;
            case CacheChange::reshapeState:
                states.at("matrix")->setState(makeValues(3, 2, 0.125)); // [-]
                break;
            case CacheChange::clearStates:
                for (auto& entry : states) {
                    retired.push_back(std::move(entry.second));
                }
                states.clear();
                break;
            case CacheChange::resetValues:
                states.at("matrix")->setState(makeValues(2, 3, -0.875)); // [-]
                break;
            default:
                break;
        }
    }
    switch (change) {
        case CacheChange::addObject:
            comparison.cached.dynPtrs.push_back(&comparison.cachedSecondary);
            comparison.reference.dynPtrs.push_back(&comparison.referenceSecondary);
            break;
        case CacheChange::removeObject:
            comparison.cached.dynPtrs.pop_back();
            comparison.reference.dynPtrs.pop_back();
            break;
        case CacheChange::reorderObjects:
            std::reverse(comparison.cached.dynPtrs.begin(), comparison.cached.dynPtrs.end());
            std::reverse(comparison.reference.dynPtrs.begin(), comparison.reference.dynPtrs.end());
            break;
        default:
            break;
    }
    comparison.repeatedSteps();
}

INSTANTIATE_TEST_SUITE_P(LiveLayout,
                         RungeKuttaCacheChange,
                         testing::Values(CacheChange::insertState,
                                         CacheChange::appendState,
                                         CacheChange::removeState,
                                         CacheChange::replaceState,
                                         CacheChange::resizeState,
                                         CacheChange::reshapeState,
                                         CacheChange::clearStates,
                                         CacheChange::resetValues,
                                         CacheChange::addObject,
                                         CacheChange::removeObject,
                                         CacheChange::reorderObjects),
                         [](const testing::TestParamInfo<CacheChange>& info) {
                             switch (info.param) {
                                 case CacheChange::insertState:
                                     return "InsertState";
                                 case CacheChange::appendState:
                                     return "AppendState";
                                 case CacheChange::removeState:
                                     return "RemoveState";
                                 case CacheChange::replaceState:
                                     return "ReplaceState";
                                 case CacheChange::resizeState:
                                     return "ResizeState";
                                 case CacheChange::reshapeState:
                                     return "ReshapeState";
                                 case CacheChange::clearStates:
                                     return "ClearStates";
                                 case CacheChange::resetValues:
                                     return "ResetValues";
                                 case CacheChange::addObject:
                                     return "AddObject";
                                 case CacheChange::removeObject:
                                     return "RemoveObject";
                                 case CacheChange::reorderObjects:
                                     return "ReorderObjects";
                             }
                             return "Unknown";
                         });

/** @brief Preserve polymorphic derivative assignment and propagation when state and derivative shapes differ. */
TEST(RungeKuttaCache, DifferentDerivativeDimensionsUseVirtualPropagation)
{
    Comparison<4> comparison(rk4Coefficients());
    comparison.repeatedSteps();
    for (auto* model : { &comparison.cachedPrimary, &comparison.referencePrimary }) {
        Eigen::MatrixXd initial(2, 1);
        initial << 0.25, 0.5; // [-]
        model->dynManager.stateContainer.stateMap.emplace("mapped", std::make_unique<MappedState>("mapped", initial));
    }
    comparison.repeatedSteps();
    const auto& cached =
      dynamic_cast<const MappedState&>(*comparison.cachedPrimary.dynManager.stateContainer.stateMap.at("mapped"));
    const auto& reference =
      dynamic_cast<const MappedState&>(*comparison.referencePrimary.dynManager.stateContainer.stateMap.at("mapped"));
    EXPECT_EQ(cached.state.rows(), 2);
    EXPECT_EQ(cached.stateDeriv.rows(), 1);
    EXPECT_EQ(cached.propagationCalls, 8U * 4U);
    EXPECT_EQ(cached.derivativeCalls, 8U * 8U);
    EXPECT_EQ(cached.propagationCalls, reference.propagationCalls);
    EXPECT_EQ(cached.derivativeCalls, reference.derivativeCalls);
    EXPECT_DOUBLE_EQ(cached.state(1, 0), 2.0 * cached.state(0, 0));
}

/** @brief Integrating dynamic objects without registered states must still evaluate every stage. */
TEST(RungeKuttaCache, EmptyStateMapsMatchReference)
{
    Comparison<4> comparison(rk4Coefficients());
    for (auto* model : { &comparison.cachedPrimary,
                         &comparison.cachedSecondary,
                         &comparison.referencePrimary,
                         &comparison.referenceSecondary }) {
        model->dynManager.stateContainer.stateMap.clear();
    }
    comparison.repeatedSteps();
}


/** @brief Retain the pre-cache adaptive loop as an independent trial/acceptance reference. */
template<typename Integrator>
class ReferenceAdaptiveIntegrator : public Integrator
{
  public:
    using Integrator::Integrator;
    static constexpr std::size_t stages = std::tuple_size<typename Integrator::KCoefficientsValues>::value;

    void integrate(double startingTime, double desiredTimeStep) override
    {
        double time = startingTime;
        double timeStep = desiredTimeStep;
        auto state = ExtendedStateVector::fromStates(this->dynPtrs);
        const auto* coefficients = static_cast<const RKAdaptiveCoefficients<stages>*>(this->coefficients.get());
        std::size_t trialCount = 0;
        while (time < startingTime + desiredTimeStep) {
            if (++trialCount > 10000) {
                throw std::runtime_error("Reference adaptive integration did not converge");
            }
            const auto kValues = this->computeKCoefficients(time, timeStep, state);
            const auto first = this->propagateStateWithKVectors(timeStep, state, kValues,
                                                                coefficients->bArray, stages);
            auto second = this->propagateStateWithKVectors(timeStep, state, kValues,
                                                           coefficients->bStarArray, stages);
            const double error = this->computeMaxRelativeError(timeStep, first, second);
            if (error <= 1.0) {
                time += timeStep;
                state = std::move(second);
            }
            double nextStep = this->safetyFactorForNextStepSize * timeStep *
                              std::pow(1.0 / error, 1.0 / this->methodLargestOrder);
            nextStep = std::min(nextStep, timeStep * this->maximumFactorIncreaseForNextStepSize);
            nextStep = std::max(nextStep, timeStep * this->minimumFactorDecreaseForNextStepSize);
            timeStep = std::min(nextStep, startingTime + desiredTimeStep - time);
        }
        state.setStates(this->dynPtrs);
    }
};

/**
 * @brief Compare adaptive stages with floating-point contraction disabled by this test target.
 *
 * The separate test_adaptiveRungeKuttaAccuracy target enables contraction and
 * checks analytic solutions without requiring identical adaptive trial sequences.
 */
template<typename Integrator>
struct AdaptiveComparison
{
    AdaptiveComparison() : cached(&cachedPrimary), reference(&referencePrimary)
    {
        for (auto* primary : {&cachedPrimary, &referencePrimary}) {
            addState(*primary, "scalar", makeValues(1, 1, 0.375)); // [-]
            addState(*primary, "matrix", makeValues(2, 3, -0.5));  // [-]
        }
        for (auto* secondary : {&cachedSecondary, &referenceSecondary}) {
            addState(*secondary, "scalar", makeValues(1, 1, -0.75)); // [-]
            addState(*secondary, "matrix", makeValues(3, 2, 0.625)); // [-]
        }
        cachedPrimary.partner = &cachedSecondary;
        cachedSecondary.partner = &cachedPrimary;
        referencePrimary.partner = &referenceSecondary;
        referenceSecondary.partner = &referencePrimary;
        cached.dynPtrs.push_back(&cachedSecondary);
        reference.dynPtrs.push_back(&referenceSecondary);
    }

    void step(double time, double duration)
    {
        for (auto* model : {&cachedPrimary, &cachedSecondary, &referencePrimary, &referenceSecondary}) {
            model->samples.clear();
        }
        cached.integrate(time, duration);
        reference.integrate(time, duration);
        expectDynamicsMatch(cachedPrimary, referencePrimary);
        expectDynamicsMatch(cachedSecondary, referenceSecondary);
    }

    TestDynamics cachedPrimary;
    TestDynamics cachedSecondary;
    TestDynamics referencePrimary;
    TestDynamics referenceSecondary;
    Integrator cached;
    ReferenceAdaptiveIntegrator<Integrator> reference;
};

template<typename Integrator>
class AdaptiveRungeKuttaCache : public testing::Test {};
using AdaptiveMethods = testing::Types<svIntegratorRKF45, svIntegratorRKF78>;
struct AdaptiveMethodNames
{
    template<typename Integrator>
    static std::string GetName(int)
    {
        return std::is_same_v<Integrator, svIntegratorRKF45> ? "RKF45" : "RKF78";
    }
};
TYPED_TEST_SUITE(AdaptiveRungeKuttaCache, AdaptiveMethods, AdaptiveMethodNames);

/** @brief Match rejected trials, accepted substeps, final clipping, and repeated integrations. */
TYPED_TEST(AdaptiveRungeKuttaCache, RejectedStepsMatchReference)
{
    AdaptiveComparison<TypeParam> comparison;
    for (TypeParam* integrator : {&comparison.cached, static_cast<TypeParam*>(&comparison.reference)}) {
        integrator->relTol = 1e-9; // [-]
        integrator->absTol = 1e-11; // [-]
    }
    const double start = 0.25; // [s]
    const double duration = 8.0; // [s]
    comparison.step(start, duration);
    constexpr auto stages = ReferenceAdaptiveIntegrator<TypeParam>::stages;
    const auto& samples = comparison.cachedPrimary.samples;
    ASSERT_GT(samples.size(), stages);
    EXPECT_EQ(samples.size() % stages, 0U);
    bool sawRejection = false;
    for (std::size_t trial = stages; trial < samples.size(); trial += stages) {
        if (samples[trial].time == samples[trial - stages].time) {
            sawRejection = true;
            EXPECT_LT(samples[trial].timeStep, samples[trial - stages].timeStep);
        }
    }
    EXPECT_TRUE(sawRejection);
    const auto& finalTrial = samples[samples.size() - stages];
    EXPECT_DOUBLE_EQ(finalTrial.time + finalTrial.timeStep, start + duration);
    comparison.step(start + duration, duration);
    comparison.step(start + 2.0 * duration, 0.125); // [s]
}

/** @brief Read updated global, state, object-state, and componentwise tolerances after warmup. */
TYPED_TEST(AdaptiveRungeKuttaCache, ToleranceChangesMatchReference)
{
    AdaptiveComparison<TypeParam> comparison;
    comparison.step(0.0, 0.125); // [s]
    for (TypeParam* integrator : {&comparison.cached, static_cast<TypeParam*>(&comparison.reference)}) {
        integrator->relTol = 1e-7; // [-]
        integrator->absTol = 1e-9; // [-]
        integrator->setRelativeTolerance("matrix", 1e-8); // [-]
        integrator->setAbsoluteTolerance("scalar", 1e-10); // [-]
        integrator->setAbsoluteTolerance(*integrator->dynPtrs[1], "matrix", 1e-12); // [-]
        integrator->setRelativeTolerance(*integrator->dynPtrs[0], "scalar", 0.0); // [-]
    }
    for (auto* model : {&comparison.cachedSecondary, &comparison.referenceSecondary}) {
        model->dynManager.stateContainer.stateMap.at("matrix")->perComponentErrorControl = true;
    }
    comparison.step(0.125, 8.0); // [s]
    // Direct public-field edits and switching back to norm control must also take effect.
    for (TypeParam* integrator : {&comparison.cached, static_cast<TypeParam*>(&comparison.reference)}) {
        integrator->relTol = 1e-5; // [-]
        integrator->setRelativeTolerance("matrix", 1e-4); // [-]
    }
    for (auto* model : {&comparison.cachedSecondary, &comparison.referenceSecondary}) {
        model->dynManager.stateContainer.stateMap.at("matrix")->perComponentErrorControl = false;
    }
    comparison.step(8.125, 8.0); // [s]
}

/** @brief Exercise cache invalidation independently for every live layout change. */
TYPED_TEST(AdaptiveRungeKuttaCache, LayoutChangesMatchReference)
{
    const std::array changes{CacheChange::insertState, CacheChange::appendState, CacheChange::removeState,
                            CacheChange::replaceState, CacheChange::resizeState, CacheChange::reshapeState,
                            CacheChange::clearStates, CacheChange::resetValues, CacheChange::addObject,
                            CacheChange::removeObject, CacheChange::reorderObjects};
    for (const auto change : changes) {
        SCOPED_TRACE(static_cast<int>(change));
        AdaptiveComparison<TypeParam> comparison;
        if (change == CacheChange::addObject) {
            comparison.cached.dynPtrs.pop_back();
            comparison.reference.dynPtrs.pop_back();
        }
        comparison.step(0.0, 0.125); // [s]
        std::vector<std::unique_ptr<StateData>> retired;
        for (auto* model : {&comparison.cachedPrimary, &comparison.referencePrimary}) {
            auto& states = model->dynManager.stateContainer.stateMap;
            switch (change) {
                case CacheChange::insertState:
                    addState(*model, "added", makeValues(4, 1, 0.25)); // [-]
                    break;
                case CacheChange::appendState:
                    addState(*model, "zzAdded", makeValues(1, 4, -0.25)); // [-]
                    break;
                case CacheChange::removeState:
                    retired.push_back(std::move(states.at("scalar")));
                    states.erase("scalar");
                    break;
                case CacheChange::replaceState:
                    retired.push_back(std::move(states.at("matrix")));
                    states.at("matrix") = std::make_unique<StateData>("matrix", makeValues(2, 3, 0.875)); // [-]
                    break;
                case CacheChange::resizeState:
                    states.at("matrix")->setState(makeValues(4, 1, -0.125)); // [-]
                    break;
                case CacheChange::reshapeState:
                    states.at("matrix")->setState(makeValues(3, 2, 0.125)); // [-]
                    break;
                case CacheChange::clearStates:
                    for (auto& entry : states) {
                        retired.push_back(std::move(entry.second));
                    }
                    states.clear();
                    break;
                case CacheChange::resetValues:
                    states.at("matrix")->setState(makeValues(2, 3, -0.875)); // [-]
                    break;
                default:
                    break;
            }
        }
        switch (change) {
            case CacheChange::addObject:
                comparison.cached.dynPtrs.push_back(&comparison.cachedSecondary);
                comparison.reference.dynPtrs.push_back(&comparison.referenceSecondary);
                break;
            case CacheChange::removeObject:
                comparison.cached.dynPtrs.pop_back();
                comparison.reference.dynPtrs.pop_back();
                break;
            case CacheChange::reorderObjects:
                std::reverse(comparison.cached.dynPtrs.begin(), comparison.cached.dynPtrs.end());
                std::reverse(comparison.reference.dynPtrs.begin(), comparison.reference.dynPtrs.end());
                break;
            default:
                break;
        }
        comparison.step(0.125, 2.0); // [s]
        comparison.step(2.125, 2.0); // [s]
    }
}

/** @brief Tolerance identities must refresh even when the ordered StateData pointer list is unchanged. */
TYPED_TEST(AdaptiveRungeKuttaCache, ChangedNamesAndObjectIndicesMatchReference)
{
    AdaptiveComparison<TypeParam> comparison;
    comparison.step(0.0, 0.125); // [s]
    TestDynamics cachedEmpty;
    TestDynamics referenceEmpty;
    comparison.cached.dynPtrs.insert(comparison.cached.dynPtrs.begin(), &cachedEmpty);
    comparison.reference.dynPtrs.insert(comparison.reference.dynPtrs.begin(), &referenceEmpty);
    for (auto* model : {&comparison.cachedPrimary, &comparison.referencePrimary}) {
        auto& states = model->dynManager.stateContainer.stateMap;
        auto renamed = states.extract("matrix");
        renamed.key() = "renamedMatrixWithLongNameToExerciseOwnedIdentityStorage";
        states.insert(std::move(renamed));
    }
    for (TypeParam* integrator : {&comparison.cached, static_cast<TypeParam*>(&comparison.reference)}) {
        integrator->setAbsoluteTolerance(*integrator->dynPtrs[1],
            "renamedMatrixWithLongNameToExerciseOwnedIdentityStorage", 1e-12); // [-]
        integrator->setRelativeTolerance(*integrator->dynPtrs[1],
            "renamedMatrixWithLongNameToExerciseOwnedIdentityStorage", 0.0); // [-]
    }
    comparison.step(0.125, 8.0); // [s]
    expectDynamicsMatch(cachedEmpty, referenceEmpty);
}

/** @brief Preserve virtual propagation and independent state/derivative buffer dimensions. */
TYPED_TEST(AdaptiveRungeKuttaCache, DifferentDerivativeDimensionsMatchReference)
{
    AdaptiveComparison<TypeParam> comparison;
    comparison.step(0.0, 0.125); // [s]
    for (auto* model : {&comparison.cachedPrimary, &comparison.referencePrimary}) {
        Eigen::MatrixXd initial(2, 1);
        initial << 0.25, 0.5; // [-]
        model->dynManager.stateContainer.stateMap.emplace("mapped", std::make_unique<MappedState>("mapped", initial));
    }
    comparison.step(0.125, 8.0); // [s]
    comparison.step(8.125, 0.25); // [s]
    const auto& cached = dynamic_cast<const MappedState&>(
        *comparison.cachedPrimary.dynManager.stateContainer.stateMap.at("mapped"));
    const auto& reference = dynamic_cast<const MappedState&>(
        *comparison.referencePrimary.dynManager.stateContainer.stateMap.at("mapped"));
    EXPECT_EQ(cached.propagationCalls, reference.propagationCalls);
    EXPECT_EQ(cached.derivativeCalls, reference.derivativeCalls);
    EXPECT_GT(cached.propagationCalls, 0U);
    EXPECT_DOUBLE_EQ(cached.state(1, 0), 2.0 * cached.state(0, 0));
}

/** @brief Empty systems and nonpositive requested intervals preserve the existing no-op behavior. */
TYPED_TEST(AdaptiveRungeKuttaCache, EmptyAndNonpositiveIntervalsMatchReference)
{
    AdaptiveComparison<TypeParam> comparison;
    comparison.step(0.25, 0.0); // [s]
    comparison.step(0.25, -0.125); // [s]
    for (auto* model : {&comparison.cachedPrimary, &comparison.cachedSecondary,
                        &comparison.referencePrimary, &comparison.referenceSecondary}) {
        model->dynManager.stateContainer.stateMap.clear();
    }
    comparison.step(0.25, 0.5); // [s]
}

/** @brief Warm adaptive trials, including rejections, must reuse Eigen buffers after layout and shape changes. */
TYPED_TEST(AdaptiveRungeKuttaCache, ReusesScratchMatricesAfterWarmup)
{
    AllocationFreeDynamics primary(2, 3);
    AllocationFreeDynamics secondary(3, 2);
    TypeParam integrator(&primary);
    integrator.relTol = 1e-9; // [-]
    integrator.absTol = 1e-11; // [-]
    const auto runWithoutAllocations = [&]() {
        EigenAllocationGuard guard;
        integrator.integrate(0.0, 8.0); // [s]
        integrator.integrate(8.0, 0.25); // [s]
    };
    integrator.integrate(0.0, 8.0); // [s]
    runWithoutAllocations();
    integrator.dynPtrs.push_back(&secondary);
    integrator.integrate(0.0, 8.0); // [s]
    runWithoutAllocations();
    primary.value->setState(makeValues(4, 2, 0.375)); // [-]
    integrator.integrate(0.0, 8.0); // [s]
    runWithoutAllocations();
    EXPECT_TRUE(primary.value->getStateReference().allFinite());
    EXPECT_TRUE(secondary.value->getStateReference().allFinite());
}

/** @brief Preserve zero embedded weights, skipped nonfinite stages, negative weights, and zero-error step growth. */
TEST(AdaptiveRungeKuttaCache, SparseAndZeroEmbeddedWeightsMatchReference)
{
    using Integrator = svIntegratorAdaptiveRungeKutta<3>;
    for (int variant = 0; variant < 4; ++variant) {
        SCOPED_TRACE(variant);
        RKAdaptiveCoefficients<3> coefficients;
        coefficients.bArray = {-0.25, 0.0, 1.25};
        coefficients.bStarArray = coefficients.bArray;
        coefficients.cArray = {0.0, 0.5, 1.0};
        if (variant == 0) coefficients.bArray = {};
        if (variant == 1) coefficients.bStarArray = {};
        if (variant == 2) coefficients.bArray = coefficients.bStarArray = {};
        if (variant == 3) coefficients.bArray = coefficients.bStarArray = {0.0, -0.25, 1.25};
        TestDynamics cachedModel;
        TestDynamics referenceModel;
        for (auto* model : {&cachedModel, &referenceModel}) {
            addState(*model, "value", makeValues(2, 3, 0.375)); // [-]
            model->poisonFirstStage = variant == 3;
        }
        Integrator cached(&cachedModel, coefficients, 2.0);
        ReferenceAdaptiveIntegrator<Integrator> reference(&referenceModel, coefficients, 2.0);
        cached.absTol = reference.absTol = 1.0; // [-]
        cached.integrate(0.0, 0.25); // [s]
        reference.integrate(0.0, 0.25); // [s]
        expectDynamicsMatch(cachedModel, referenceModel);
    }
}


/** @brief Keep one large component constant while a small component decays independently. */
class MixedScaleDynamics : public TestDynamics
{
  public:
    void equationsOfMotion(double time, double step) override
    {
        TestDynamics::equationsOfMotion(time, step);
        auto& data = *this->dynManager.stateContainer.stateMap.at("mixed");
        const double decayRate = -1.0; // [1/s]
        Eigen::Vector2d derivative{0.0, decayRate * data.getStateReference()(1)}; // [1/s]
        data.setDerivative(derivative);
    }
};

/** @brief Componentwise error control must protect small components hidden by a large vector norm. */
TYPED_TEST(AdaptiveRungeKuttaCache, ComponentwiseControlResolvesSmallComponent)
{
    std::array<std::size_t, 2> evaluations{};
    const double duration = 4.0; // [s]
    for (int componentwise = 0; componentwise < 2; ++componentwise) {
        MixedScaleDynamics cachedModel;
        MixedScaleDynamics referenceModel;
        for (auto* model : {&cachedModel, &referenceModel}) {
            Eigen::MatrixXd initial(2, 1);
            initial << 1e9, 1.0; // [-]
            addState(*model, "mixed", initial);
            model->dynManager.stateContainer.stateMap.at("mixed")->perComponentErrorControl = componentwise != 0;
        }
        TypeParam cached(&cachedModel);
        ReferenceAdaptiveIntegrator<TypeParam> reference(&referenceModel);
        cached.relTol = reference.relTol = 1e-7; // [-]
        cached.absTol = reference.absTol = 1e-10; // [-]
        cached.integrate(0.0, duration); // [s]
        reference.integrate(0.0, duration); // [s]
        expectDynamicsMatch(cachedModel, referenceModel);
        evaluations[static_cast<std::size_t>(componentwise)] = cachedModel.samples.size();
        if (componentwise) {
            const auto& final = cachedModel.dynManager.stateContainer.stateMap.at("mixed")->getStateReference();
            const double exact = std::exp(-duration); // [-], decay rate is 1/s
            EXPECT_DOUBLE_EQ(final(0), 1e9);
            EXPECT_NEAR(final(1), exact, 1e-5 * exact);
        }
    }
    EXPECT_GT(evaluations[1], evaluations[0]);
}

/** @brief Expose tolerance resolution for checking independent relative and absolute override precedence. */
template<typename Integrator>
class ToleranceProbe : public Integrator
{
  public:
    using Integrator::Integrator;
    using Integrator::getTolerance;
};

/** @brief Absolute and relative settings independently select object-state, state, then global defaults. */
TYPED_TEST(AdaptiveRungeKuttaCache, TolerancePrecedence)
{
    TestDynamics primary;
    TestDynamics secondary;
    ToleranceProbe<TypeParam> integrator(&primary);
    integrator.dynPtrs.push_back(&secondary);
    integrator.relTol = 0.25; // [-]
    integrator.absTol = 0.125; // [-]
    const double stateNorm = 2.0; // [-]
    EXPECT_DOUBLE_EQ(integrator.getTolerance({0, "value"}, stateNorm), 0.625);
    integrator.setRelativeTolerance("value", 0.0625); // [-]
    EXPECT_DOUBLE_EQ(integrator.getTolerance({0, "value"}, stateNorm), 0.25);
    integrator.setAbsoluteTolerance(primary, "value", 0.03125); // [-]
    EXPECT_DOUBLE_EQ(integrator.getTolerance({0, "value"}, stateNorm), 0.15625);
    EXPECT_DOUBLE_EQ(integrator.getTolerance({1, "value"}, stateNorm), 0.25);
    integrator.setRelativeTolerance(primary, "value", 0.0); // [-]
    integrator.setAbsoluteTolerance("value", 0.5); // [-]
    EXPECT_DOUBLE_EQ(integrator.getTolerance({0, "value"}, stateNorm), 0.03125);
    EXPECT_DOUBLE_EQ(integrator.getTolerance({1, "value"}, stateNorm), 0.625);
}

} // namespace
