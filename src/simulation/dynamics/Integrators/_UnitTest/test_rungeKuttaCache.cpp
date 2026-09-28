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

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
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

/** @brief Restore the pre-cache integration sequence using the retained adaptive-integrator helpers. */
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

} // namespace
