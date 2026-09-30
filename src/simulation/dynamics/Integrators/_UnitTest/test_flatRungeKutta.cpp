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

#include "simulation/dynamics/_GeneralModuleFiles/stateRegistry.h"
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "integratorTestAccess.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorAdaptiveRungeKutta.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorRungeKutta.h"

namespace {
using integrator_step_test::stepIntegrator;

std::atomic<size_t> allocationCount{ 0 };

static_assert(std::is_same_v<decltype(std::declval<const StateData&>().getName()), std::string>);
static_assert(std::is_same_v<decltype(std::declval<const StateData&>().getStateReference()), ConstMatrixView>);
static_assert(std::is_same_v<decltype(std::declval<const StateData&>().getStateDerivReference()), ConstMatrixView>);
static_assert(!std::is_destructible_v<StateData>);

class MissingBindingPreparation : public StateVecIntegrator
{
  public:
    using StateVecIntegrator::StateVecIntegrator;

  protected:
    void validateIntegrationBinding() const override {}
    void integrateImpl(double, double) override {}
};

class MissingBindingValidation : public StateVecIntegrator
{
  public:
    using StateVecIntegrator::StateVecIntegrator;

  protected:
    void prepareIntegrationBinding() override {}
    void integrateImpl(double, double) override {}
};

class CompleteLifecycleIntegrator final : public StateVecIntegrator
{
  public:
    using StateVecIntegrator::StateVecIntegrator;

  protected:
    void prepareIntegrationBinding() override {}
    void validateIntegrationBinding() const override {}
    void integrateImpl(double, double) override {}
};

static_assert(std::is_abstract_v<MissingBindingPreparation>);
static_assert(std::is_abstract_v<MissingBindingValidation>);
static_assert(!std::is_abstract_v<CompleteLifecycleIntegrator>);

class TestDynamics final : public DynamicObject
{
  public:
    TestDynamics()
    {
        StateSpec matrixSpec;
        matrixSpec.state = { 2, 2 };
        matrixSpec.derivative = matrixSpec.state;
        matrixSpec.diffusionTangent = matrixSpec.state;
        this->matrixState = this->dynManager.registerState("matrixStateWithANameLongEnoughToAllocate", matrixSpec);

        StateSpec vectorSpec;
        vectorSpec.state = { 3, 1 };
        vectorSpec.derivative = vectorSpec.state;
        vectorSpec.diffusionTangent = vectorSpec.state;
        this->vectorState = this->dynManager.registerState("vectorState", vectorSpec);

        Eigen::MatrixXd matrix(2, 2);
        matrix << 0.75, -1.25, 2.5, -0.375;
        this->matrixState->setState(matrix);

        Eigen::MatrixXd vector(3, 1);
        vector << -0.625, 1.75, 0.3125;
        this->vectorState->setState(vector);

        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double time, double timeStep) override
    {
        ++this->equationsOfMotionCalls;
        if (this->equationsOfMotionObserver) {
            this->equationsOfMotionObserver();
        }
        if (this->throwOnCall == this->equationsOfMotionCalls) {
            throw std::runtime_error("deliberate equations-of-motion failure");
        }

        const auto matrix = this->matrixState->stateView();
        const auto vector = this->vectorState->stateView();
        auto matrixDerivative = this->matrixState->derivativeView();
        auto vectorDerivative = this->vectorState->derivativeView();

        matrixDerivative(0, 0) = 0.125 * matrix(0, 0) - 0.75 * matrix(1, 0) + vector(0) + time;
        matrixDerivative(1, 0) = matrix(0, 1) * vector(1) - 0.375 * matrix(1, 0) + timeStep;
        matrixDerivative(0, 1) = -0.625 * matrix(0, 0) + matrix(1, 1) * vector(2) - time;
        matrixDerivative(1, 1) = matrix(1, 0) - 0.25 * matrix(0, 1) + vector(1);

        vectorDerivative(0) = matrix(0, 0) * matrix(1, 1) - 0.5 * vector(2) + timeStep;
        vectorDerivative(1) = matrix(1, 0) + 0.75 * vector(0) - time;
        vectorDerivative(2) = matrix(0, 1) - matrix(1, 1) + 0.125 * vector(1);
    }

    StateData* matrixState = nullptr;
    StateData* vectorState = nullptr;
    size_t equationsOfMotionCalls = 0;
    size_t throwOnCall = 0;
    std::function<void()> equationsOfMotionObserver;
};

class UnequalExtentPolicy final : public StateUpdatePolicy
{
  public:
    bool topologyEquals(const StateUpdatePolicy& other) const override
    {
        return dynamic_cast<const UnequalExtentPolicy*>(&other) != nullptr;
    }

    void validate(const StateSpec&) const override {}

    void buildDriftCandidate(ConstMatrixView base, ConstMatrixView, double, MutableMatrixView output) const override
    {
        output = base;
    }

    void applyNoiseIncrement(MutableMatrixView, ConstMatrixView, double) const override {}
};

class UnequalExtentDynamics final : public DynamicObject
{
  public:
    UnequalExtentDynamics()
    {
        StateSpec spec;
        spec.state = { 4, 1 };
        spec.derivative = { 3, 1 };
        spec.diffusionTangent = { 3, 1 };
        spec.updateKind = StateUpdateKind::Special;
        this->state =
          this->dynManager.registerState("unequalExtentState", spec, std::make_unique<UnequalExtentPolicy>());
        Eigen::MatrixXd initialState(4, 1);
        initialState << 1.0, -2.0, 3.0, -4.0;
        this->state->setState(initialState);
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override
    {
        auto derivative = this->state->derivativeView();
        derivative << 0.25, -0.5, 0.75;
    }

    StateData* state = nullptr;
};

class MixedUpdatePolicy final : public StateUpdatePolicy
{
  public:
    bool topologyEquals(const StateUpdatePolicy& other) const override
    {
        return dynamic_cast<const MixedUpdatePolicy*>(&other) != nullptr;
    }

    void validate(const StateSpec& spec) const override
    {
        if (spec.state != MatrixShape{ 4, 1 } || spec.derivative != MatrixShape{ 3, 1 }) {
            throw std::invalid_argument("Unexpected mixed-update topology.");
        }
    }

    void buildDriftCandidate(ConstMatrixView base,
                             ConstMatrixView combinedDrift,
                             double timeStep,
                             MutableMatrixView output) const override
    {
        output = base;
        output(0) += combinedDrift(0) * timeStep;
        output(1) -= combinedDrift(1) * timeStep;
        output(2) += combinedDrift(2) * timeStep;
        output(3) += (combinedDrift(0) + combinedDrift(2)) * timeStep;
    }

    void applyNoiseIncrement(MutableMatrixView, ConstMatrixView, double) const override {}
};

class MixedUpdateDynamics final : public DynamicObject
{
  public:
    MixedUpdateDynamics()
    {
        this->registerEuclidean("euclideanA", 2, 1, 0.25);
        this->registerEuclidean("euclideanB", 2, 2, 1.25);

        StateSpec specialSpec;
        specialSpec.state = { 4, 1 };
        specialSpec.derivative = { 3, 1 };
        specialSpec.diffusionTangent = { 3, 1 };
        specialSpec.updateKind = StateUpdateKind::Special;
        this->specialState =
          this->dynManager.registerState("special", specialSpec, std::make_unique<MixedUpdatePolicy>());
        Eigen::MatrixXd specialValue(4, 1);
        specialValue << -0.75, 1.5, -2.25, 3.0;
        this->specialState->setState(specialValue);

        this->registerEuclidean("euclideanC", 1, 3, -1.0);
        this->registerEuclidean("euclideanD", 1, 1, 2.0);
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}
    void equationsOfMotion(double, double) override {}

  private:
    void registerEuclidean(const std::string& name, uint32_t rows, uint32_t columns, double firstValue)
    {
        StateSpec spec;
        spec.state = { rows, columns };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        StateData* state = this->dynManager.registerState(name, spec);
        Eigen::MatrixXd value(rows, columns);
        for (Eigen::Index index = 0; index < value.size(); ++index) {
            value(index) = firstValue + 0.125 * static_cast<double>(index);
        }
        state->setState(value);
    }

    StateData* specialState = nullptr;
};

class StageZeroDynamics final : public DynamicObject
{
  public:
    StageZeroDynamics()
    {
        this->state = this->dynManager.registerState(1, 1, "stageZeroState");
        this->state->stateView()(0, 0) = 2.0;
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override
    {
        ++this->equationsOfMotionCalls;
        if (this->firstStageObserver) {
            this->firstStageObserver();
        }
        const double currentState = this->state->stateView()(0, 0);
        if (this->equationsOfMotionCalls == 1) {
            this->firstStateObserved = currentState;
        }
        this->state->derivativeView()(0, 0) = currentState;
    }

    StateData* state = nullptr;
    size_t equationsOfMotionCalls = 0;
    double firstStateObserved = 0.0;
    std::function<void()> firstStageObserver;
};

class ControlledScalarDynamics final : public DynamicObject
{
  public:
    ControlledScalarDynamics()
    {
        this->state = this->dynManager.registerState(1, 1, "controlledScalar");
        this->state->stateView()(0, 0) = 1.0;
        this->dynManager.finalizeStates();
    }

    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}

    void equationsOfMotion(double, double) override
    {
        ++this->equationsOfMotionCalls;
        if (this->equationsOfMotionCalls == this->throwOnCall) {
            throw std::runtime_error("deliberate adaptive-stage failure");
        }
        this->state->derivativeView()(0, 0) = this->returnNaN ? std::numeric_limits<double>::quiet_NaN() : 1.0;
    }

    StateData* state = nullptr;
    bool returnNaN = false;
    size_t equationsOfMotionCalls = 0;
    size_t throwOnCall = std::numeric_limits<size_t>::max();
};

template<size_t numberStages>
class RungeKuttaCandidateProbe final : public svIntegratorRungeKutta<numberStages>
{
  public:
    RungeKuttaCandidateProbe(DynamicObject* dynamics, const RKCoefficients<numberStages>& coefficients)
      : svIntegratorRungeKutta<numberStages>(dynamics, coefficients)
    {
    }

    void bindAndSeed()
    {
        this->bindFlatStorage();
        this->gatherStates(this->baseState);
        for (Eigen::Index stage = 0; stage < this->kStorage.cols(); ++stage) {
            for (Eigen::Index index = 0; index < this->kStorage.rows(); ++index) {
                this->kStorage(index, stage) = 0.0637 * static_cast<double>((stage + 1) * (index + 3)) - 0.3719;
            }
        }
    }

    size_t runCount() const { return this->flatStateUpdateRuns().size(); }

    void primeCandidate(double value)
    {
        this->bindFlatStorage();
        this->candidateState.setConstant(value);
    }

    double candidateFront() const { return this->candidateState(0); }

    StateUpdateKind runKind(size_t index) const { return this->flatStateUpdateRuns().at(index).updateKind; }

    Eigen::VectorXd optimizedCandidate(double timeStep,
                                       const std::array<double, numberStages>& stageCoefficients,
                                       size_t maxStage)
    {
        this->buildFlatCandidate(timeStep, stageCoefficients, maxStage, this->candidateState);
        return this->candidateState;
    }

    Eigen::VectorXd descriptorWiseCandidate(double timeStep,
                                            const std::array<double, numberStages>& stageCoefficients,
                                            size_t maxStage)
    {
        Eigen::VectorXd combined(this->combinedDerivative.size());
        Eigen::VectorXd scaled(this->combinedDerivative.size());
        bool haveTerm = false;
        for (size_t stageIndex = 0; stageIndex < maxStage; ++stageIndex) {
            const double coefficient = stageCoefficients.at(stageIndex);
            if (coefficient == 0.0) {
                continue;
            }
            for (const auto& descriptor : this->flatStateDescriptors()) {
                const auto offset = static_cast<Eigen::Index>(descriptor.derivativeOffset);
                Eigen::Map<Eigen::MatrixXd> scaledDerivative(
                  scaled.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
                Eigen::Map<Eigen::MatrixXd> combinedDerivative(
                  combined.data() + offset, descriptor.derivativeRows, descriptor.derivativeColumns);
                const Eigen::Map<const Eigen::MatrixXd> stageDerivative(
                  this->kStorage.col(static_cast<Eigen::Index>(stageIndex)).data() + offset,
                  descriptor.derivativeRows,
                  descriptor.derivativeColumns);
                scaledDerivative = stageDerivative * coefficient;
                if (!haveTerm) {
                    combinedDerivative = scaledDerivative;
                } else {
                    combinedDerivative += scaledDerivative;
                }
            }
            haveTerm = true;
        }

        if (!haveTerm) {
            return this->baseState;
        }

        Eigen::VectorXd output(this->baseState.size());
        for (const auto& descriptor : this->flatStateDescriptors()) {
            const auto stateOffset = static_cast<Eigen::Index>(descriptor.stateOffset);
            const auto derivativeOffset = static_cast<Eigen::Index>(descriptor.derivativeOffset);
            const Eigen::Map<const Eigen::MatrixXd> base(
              this->baseState.data() + stateOffset, descriptor.stateRows, descriptor.stateColumns);
            const Eigen::Map<const Eigen::MatrixXd> combinedDrift(
              combined.data() + derivativeOffset, descriptor.derivativeRows, descriptor.derivativeColumns);
            Eigen::Map<Eigen::MatrixXd> candidate(
              output.data() + stateOffset, descriptor.stateRows, descriptor.stateColumns);
            if (descriptor.updateKind == StateUpdateKind::Special) {
                descriptor.specialUpdate->buildDriftCandidate(base, combinedDrift, timeStep, candidate);
            } else {
                candidate = base;
                candidate += combinedDrift * timeStep;
            }
        }
        return output;
    }
};

template<size_t numberStages>
class RungeKuttaStorageProbe final : public svIntegratorRungeKutta<numberStages>
{
  public:
    RungeKuttaStorageProbe(DynamicObject* dynamics, const RKCoefficients<numberStages>& coefficients)
      : svIntegratorRungeKutta<numberStages>(dynamics, coefficients)
    {
    }


};

template<size_t numberStages>
class AdaptiveToleranceProbe final : public svIntegratorAdaptiveRungeKutta<numberStages>
{
  public:
    AdaptiveToleranceProbe(DynamicObject* dynamics,
                           const RKAdaptiveCoefficients<numberStages>& coefficients,
                           double methodLargestOrder)
      : svIntegratorAdaptiveRungeKutta<numberStages>(dynamics, coefficients, methodLargestOrder)
    {
    }

    void primeCandidate(double value)
    {
        this->bindFlatStorage();
        this->candidateState.setConstant(value);
    }

    double candidateFront() const { return this->candidateState(0); }

    const auto& resolvedSpans() const { return this->toleranceSpans; }

    std::pair<double, double> resolvedTolerance(size_t dynamicObjectIndex, const std::string& stateName)
    {
        this->bindFlatStorage();
        this->resolveToleranceSpans();
        const auto& states = this->flatStateDescriptors();
        for (size_t index = 0; index < states.size(); ++index) {
            const auto& descriptor = states[index];
            if (descriptor.dynamicObjectIndex == dynamicObjectIndex && descriptor.state->getName() == stateName) {
                const auto& tolerance = this->toleranceSpans[index];
                return { tolerance.relative, tolerance.absolute };
            }
        }
        throw std::invalid_argument("State is not bound to the tolerance probe.");
    }
};







RKCoefficients<1>
eulerCoefficients()
{
    RKCoefficients<1> coefficients;
    coefficients.aMatrix = { { { { 0.0 } } } };
    coefficients.bArray = { { 1.0 } };
    coefficients.cArray = { { 0.0 } };
    return coefficients;
}

RKCoefficients<4>
rk4Coefficients()
{
    RKCoefficients<4> coefficients;
    coefficients.aMatrix = { { { { 0.0, 0.0, 0.0, 0.0 } },
                               { { 0.5, 0.0, 0.0, 0.0 } },
                               { { 0.0, 0.5, 0.0, 0.0 } },
                               { { 0.0, 0.0, 1.0, 0.0 } } } };
    coefficients.bArray = { { 1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0 } };
    coefficients.cArray = { { 0.0, 0.5, 0.5, 1.0 } };
    return coefficients;
}

RKAdaptiveCoefficients<4>
bogackiShampineCoefficients()
{
    RKAdaptiveCoefficients<4> coefficients;
    coefficients.aMatrix = { { { { 0.0, 0.0, 0.0, 0.0 } },
                               { { 0.5, 0.0, 0.0, 0.0 } },
                               { { 0.0, 0.75, 0.0, 0.0 } },
                               { { 2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0, 0.0 } } } };
    coefficients.bArray = { { 7.0 / 24.0, 0.25, 1.0 / 3.0, 0.125 } };
    coefficients.bStarArray = { { 2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0, 0.0 } };
    coefficients.cArray = { { 0.0, 0.5, 0.75, 1.0 } };
    return coefficients;
}

RKAdaptiveCoefficients<1>
unequalEmbeddedEulerCoefficients()
{
    RKAdaptiveCoefficients<1> coefficients;
    coefficients.aMatrix = { { { { 0.0 } } } };
    coefficients.bArray = { { 0.0 } };
    coefficients.bStarArray = { { 1.0 } };
    coefficients.cArray = { { 0.0 } };
    return coefficients;
}

uint64_t
doubleBits(double value)
{
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

void
expectStateBitsEqual(const TestDynamics& first, const TestDynamics& second)
{
    const auto firstMatrix = first.matrixState->stateView();
    const auto secondMatrix = second.matrixState->stateView();
    ASSERT_EQ(firstMatrix.size(), secondMatrix.size());
    for (Eigen::Index index = 0; index < firstMatrix.size(); ++index) {
        EXPECT_EQ(doubleBits(firstMatrix(index)), doubleBits(secondMatrix(index)));
    }

    const auto firstVector = first.vectorState->stateView();
    const auto secondVector = second.vectorState->stateView();
    ASSERT_EQ(firstVector.size(), secondVector.size());
    for (Eigen::Index index = 0; index < firstVector.size(); ++index) {
        EXPECT_EQ(doubleBits(firstVector(index)), doubleBits(secondVector(index)));
    }
}

void
configureTolerances(svIntegratorAdaptiveRungeKutta<4>& integrator, const TestDynamics& dynamics)
{
    integrator.setRelativeTolerance(2.5e-7);
    integrator.setAbsoluteTolerance(3.0e-10);
    integrator.setRelativeTolerance("matrixStateWithANameLongEnoughToAllocate", 7.5e-8);
    integrator.setAbsoluteTolerance("matrixStateWithANameLongEnoughToAllocate", 2.0e-11);
    integrator.setRelativeTolerance(dynamics, "matrixStateWithANameLongEnoughToAllocate", 1.25e-8);
    integrator.setAbsoluteTolerance(dynamics, "matrixStateWithANameLongEnoughToAllocate", 4.0e-12);
}

} // namespace

void*
operator new(std::size_t size)
{
    allocationCount.fetch_add(1, std::memory_order_relaxed);
    if (void* memory = std::malloc(size)) {
        return memory;
    }
    throw std::bad_alloc();
}

void*
operator new[](std::size_t size)
{
    return ::operator new(size);
}

void
operator delete(void* memory) noexcept
{
    std::free(memory);
}

void
operator delete[](void* memory) noexcept
{
    std::free(memory);
}

void
operator delete(void* memory, std::size_t) noexcept
{
    std::free(memory);
}

void
operator delete[](void* memory, std::size_t) noexcept
{
    std::free(memory);
}

TEST(FlatRungeKutta, FixedStepExceptionRestoresAcceptedBase)
{
    TestDynamics expectedDynamics;
    TestDynamics failingDynamics;
    failingDynamics.throwOnCall = 2;
    svIntegratorRungeKutta<4> integrator(&failingDynamics, rk4Coefficients());

    EXPECT_THROW(stepIntegrator(integrator, 0.75, 0.125), std::runtime_error);
    expectStateBitsEqual(expectedDynamics, failingDynamics);
}

TEST(FlatRungeKutta, EulerUsesAcceptedBaseWhenCallbackMutatesStates)
{
    TestDynamics dynamics;
    const Eigen::MatrixXd matrixBase = dynamics.matrixState->stateView();
    const Eigen::MatrixXd vectorBase = dynamics.vectorState->stateView();
    dynamics.equationsOfMotionObserver = [&]() {
        dynamics.matrixState->stateView().setConstant(2.0);
        dynamics.vectorState->stateView().setConstant(3.0);
    };
    svIntegratorRungeKutta<1> integrator(&dynamics, eulerCoefficients());

    stepIntegrator(integrator, 0.75, 0.125);

    const Eigen::MatrixXd matrixExpected = matrixBase + 0.125 * dynamics.matrixState->derivativeView();
    const Eigen::MatrixXd vectorExpected = vectorBase + 0.125 * dynamics.vectorState->derivativeView();
    EXPECT_TRUE(dynamics.matrixState->stateView().isApprox(matrixExpected, 1.0e-14));
    EXPECT_TRUE(dynamics.vectorState->stateView().isApprox(vectorExpected, 1.0e-14));
    EXPECT_EQ(dynamics.equationsOfMotionCalls, 1U);
}

TEST(FlatRungeKutta, EulerExceptionRestoresStatesMutatedByCallback)
{
    TestDynamics expectedDynamics;
    TestDynamics dynamics;
    dynamics.equationsOfMotionObserver = [&]() { dynamics.matrixState->stateView().setZero(); };
    dynamics.throwOnCall = 1;
    svIntegratorRungeKutta<1> integrator(&dynamics, eulerCoefficients());

    EXPECT_THROW(stepIntegrator(integrator, 0.75, 0.125), std::runtime_error);
    expectStateBitsEqual(expectedDynamics, dynamics);
}

TEST(FlatRungeKutta, NativeConstructorsRejectNonfiniteTableausAndOrder)
{
    TestDynamics dynamics;
    RKCoefficients<1> fixedCoefficients = eulerCoefficients();
    fixedCoefficients.aMatrix[0][0] = std::numeric_limits<double>::infinity();
    EXPECT_THROW((svIntegratorRungeKutta<1>(&dynamics, fixedCoefficients)), std::invalid_argument);

    RKAdaptiveCoefficients<1> adaptiveCoefficients;
    adaptiveCoefficients.aMatrix = { { { { 0.0 } } } };
    adaptiveCoefficients.bArray = { { 1.0 } };
    adaptiveCoefficients.bStarArray = { { std::numeric_limits<double>::quiet_NaN() } };
    adaptiveCoefficients.cArray = { { 0.0 } };
    EXPECT_THROW((svIntegratorAdaptiveRungeKutta<1>(&dynamics, adaptiveCoefficients, 1.0)), std::invalid_argument);

    adaptiveCoefficients.bStarArray = { { 0.0 } };
    EXPECT_THROW((svIntegratorAdaptiveRungeKutta<1>(&dynamics, adaptiveCoefficients, 0.0)), std::invalid_argument);
    EXPECT_THROW(
      (svIntegratorAdaptiveRungeKutta<1>(&dynamics, adaptiveCoefficients, std::numeric_limits<double>::infinity())),
      std::invalid_argument);

    RKCoefficients<2> implicitCoefficients;
    implicitCoefficients.aMatrix = { { { { 0.0, 0.25 } }, { { 1.0, 0.0 } } } };
    implicitCoefficients.bArray = { { 0.5, 0.5 } };
    implicitCoefficients.cArray = { { 0.0, 1.0 } };
    EXPECT_THROW((svIntegratorRungeKutta<2>(&dynamics, implicitCoefficients)), std::invalid_argument);

    implicitCoefficients.aMatrix[0][1] = 0.0;
    implicitCoefficients.aMatrix[1][1] = 0.25;
    EXPECT_THROW((svIntegratorRungeKutta<2>(&dynamics, implicitCoefficients)), std::invalid_argument);

    RKAdaptiveCoefficients<2> implicitAdaptiveCoefficients;
    implicitAdaptiveCoefficients.aMatrix = { { { { 0.0, 0.25 } }, { { 1.0, 0.0 } } } };
    implicitAdaptiveCoefficients.bArray = { { 0.5, 0.5 } };
    implicitAdaptiveCoefficients.bStarArray = { { 1.0, 0.0 } };
    implicitAdaptiveCoefficients.cArray = { { 0.0, 1.0 } };
    EXPECT_THROW((svIntegratorAdaptiveRungeKutta<2>(&dynamics, implicitAdaptiveCoefficients, 2.0)),
                 std::invalid_argument);
}

TEST(FlatRungeKutta, FixedStepFirstZeroRowReusesGatheredBase)
{
    StageZeroDynamics dynamics;
    RungeKuttaCandidateProbe<4> integrator(&dynamics, rk4Coefficients());
    constexpr double candidateSentinel = 12345.0;
    double candidateDuringFirstStage = 0.0;
    integrator.primeCandidate(candidateSentinel);
    dynamics.firstStageObserver = [&]() {
        if (dynamics.equationsOfMotionCalls == 1) {
            candidateDuringFirstStage = integrator.candidateFront();
        }
    };

    constexpr double timeStep = 0.125;
    stepIntegrator(integrator, 0.75, timeStep);

    const double expected = 2.0 * (1.0 + timeStep + timeStep * timeStep / 2.0 + timeStep * timeStep * timeStep / 6.0 +
                                   timeStep * timeStep * timeStep * timeStep / 24.0);
    EXPECT_DOUBLE_EQ(candidateDuringFirstStage, candidateSentinel);
    EXPECT_DOUBLE_EQ(dynamics.firstStateObserved, 2.0);
    EXPECT_NEAR(dynamics.state->stateView()(0, 0), expected, 1e-15);
    EXPECT_EQ(dynamics.equationsOfMotionCalls, 4U);
}

TEST(FlatRungeKutta, AdaptiveExceptionRestoresAcceptedBase)
{
    TestDynamics expectedDynamics;
    TestDynamics failingDynamics;
    failingDynamics.throwOnCall = 3;
    svIntegratorAdaptiveRungeKutta<4> integrator(&failingDynamics, bogackiShampineCoefficients(), 3.0);

    EXPECT_THROW(stepIntegrator(integrator, 0.75, 0.125), std::runtime_error);
    expectStateBitsEqual(expectedDynamics, failingDynamics);
}

TEST(FlatRungeKutta, AdaptiveFirstZeroRowReusesGatheredBase)
{
    StageZeroDynamics dynamics;
    AdaptiveToleranceProbe<4> integrator(&dynamics, bogackiShampineCoefficients(), 3.0);
    constexpr double candidateSentinel = 12345.0;
    double candidateDuringFirstStage = 0.0;
    integrator.primeCandidate(candidateSentinel);
    integrator.relTol = 1.0;
    integrator.absTol = 1.0;
    dynamics.firstStageObserver = [&]() {
        if (dynamics.equationsOfMotionCalls == 1) {
            candidateDuringFirstStage = integrator.candidateFront();
        }
    };

    stepIntegrator(integrator, 0.75, 0.125);

    EXPECT_DOUBLE_EQ(candidateDuringFirstStage, candidateSentinel);
    EXPECT_DOUBLE_EQ(dynamics.firstStateObserved, 2.0);
}

TEST(FlatRungeKutta, AdaptiveRejectsNonfiniteCandidateAndRestoresState)
{
    ControlledScalarDynamics dynamics;
    dynamics.returnNaN = true;
    svIntegratorAdaptiveRungeKutta<1> integrator(&dynamics, unequalEmbeddedEulerCoefficients(), 1.0);

    EXPECT_THROW(stepIntegrator(integrator, 1.0, 0.125), std::runtime_error);
    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(1.0));
    EXPECT_EQ(dynamics.equationsOfMotionCalls, 1U);
}

TEST(FlatRungeKutta, AdaptiveThrowsWhenErrorControlCannotMakeProgress)
{
    ControlledScalarDynamics dynamics;
    svIntegratorAdaptiveRungeKutta<1> integrator(&dynamics, unequalEmbeddedEulerCoefficients(), 1.0);
    integrator.relTol = 0.0;
    integrator.absTol = 0.0;

    EXPECT_THROW(stepIntegrator(integrator, 1.0, 0.125), std::runtime_error);
    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(1.0));
    EXPECT_GT(dynamics.equationsOfMotionCalls, 1U);
    EXPECT_LT(dynamics.equationsOfMotionCalls, 100U);
}

TEST(FlatRungeKutta, AdaptiveIntegratesSubUlpAbsoluteTimeStep)
{
    ControlledScalarDynamics dynamics;
    svIntegratorAdaptiveRungeKutta<1> integrator(&dynamics, unequalEmbeddedEulerCoefficients(), 1.0);
    integrator.relTol = 1.0;
    integrator.absTol = 1.0;
    constexpr double startingTime = 1e10; // [s]
    constexpr double timeStep = 1e-7;     // [s]
    ASSERT_EQ(startingTime + timeStep, startingTime);

    stepIntegrator(integrator, startingTime, timeStep);

    EXPECT_DOUBLE_EQ(dynamics.state->stateView()(0, 0), 1.0 + timeStep);
    EXPECT_EQ(dynamics.equationsOfMotionCalls, 1U);
}

TEST(FlatRungeKutta, AdaptiveRejectionThenExceptionRestoresEntryState)
{
    TestDynamics expectedDynamics;
    TestDynamics failingDynamics;
    failingDynamics.throwOnCall = 5;
    svIntegratorAdaptiveRungeKutta<4> integrator(&failingDynamics, bogackiShampineCoefficients(), 3.0);
    integrator.setRelativeTolerance(0.0);
    integrator.setAbsoluteTolerance(1e-18);

    EXPECT_THROW(stepIntegrator(integrator, 0.75, 0.125), std::runtime_error);
    EXPECT_EQ(failingDynamics.equationsOfMotionCalls, 5U);
    expectStateBitsEqual(expectedDynamics, failingDynamics);
}

TEST(FlatRungeKutta, AdaptiveAcceptedSubstepThenExceptionRestoresEntryState)
{
    ControlledScalarDynamics dynamics;
    dynamics.throwOnCall = 3;
    svIntegratorAdaptiveRungeKutta<1> integrator(&dynamics, unequalEmbeddedEulerCoefficients(), 1.0);
    integrator.relTol = 0.0;
    integrator.absTol = 0.05;

    EXPECT_THROW(stepIntegrator(integrator, 1.0, 0.125), std::runtime_error);
    EXPECT_EQ(dynamics.equationsOfMotionCalls, 3U);
    EXPECT_EQ(doubleBits(dynamics.state->stateView()(0, 0)), doubleBits(1.0));
}

TEST(FlatRungeKutta, CoalescedEuclideanRunsMatchDescriptorWiseArithmetic)
{
    MixedUpdateDynamics dynamics;
    RungeKuttaCandidateProbe<4> integrator(&dynamics, rk4Coefficients());
    integrator.bindAndSeed();

    ASSERT_EQ(integrator.runCount(), 3U);
    EXPECT_EQ(integrator.runKind(0), StateUpdateKind::Euclidean);
    EXPECT_EQ(integrator.runKind(1), StateUpdateKind::Special);
    EXPECT_EQ(integrator.runKind(2), StateUpdateKind::Euclidean);

    const std::array<double, 4> coefficients = { { 1.0 / 6.0, -1.0 / 3.0, 0.0, 1.0 / 7.0 } };
    const Eigen::VectorXd optimized = integrator.optimizedCandidate(0.1875, coefficients, 4);
    const Eigen::VectorXd descriptorWise = integrator.descriptorWiseCandidate(0.1875, coefficients, 4);
    ASSERT_EQ(optimized.size(), descriptorWise.size());
    for (Eigen::Index index = 0; index < optimized.size(); ++index) {
        // This small fixture permits roundoff from contraction and coalescing.
        // The topology and policy-dispatch assertions above remain exact.
        const double scale = std::max(1.0, std::abs(descriptorWise(index)));
        const double tolerance = 32.0 * std::numeric_limits<double>::epsilon() * scale;
        EXPECT_NEAR(optimized(index), descriptorWise(index), tolerance) << "scalar " << index;
    }
}

TEST(FlatRungeKutta, BookkeepingDoesNotAllocateAfterBinding)
{
    TestDynamics fixedDynamics;
    TestDynamics adaptiveDynamics;
    svIntegratorRungeKutta<4> fixedIntegrator(&fixedDynamics, rk4Coefficients());
    svIntegratorAdaptiveRungeKutta<4> adaptiveIntegrator(&adaptiveDynamics, bogackiShampineCoefficients(), 3.0);
    adaptiveIntegrator.setRelativeTolerance(1.0);
    adaptiveIntegrator.setAbsoluteTolerance(1.0);

    stepIntegrator(fixedIntegrator, 0.0, 0.0);
    stepIntegrator(adaptiveIntegrator, 0.0, 0.0);

    allocationCount.store(0, std::memory_order_relaxed);
    stepIntegrator(fixedIntegrator, 0.0, 0.125);
    EXPECT_EQ(allocationCount.load(std::memory_order_relaxed), 0U);

    allocationCount.store(0, std::memory_order_relaxed);
    stepIntegrator(adaptiveIntegrator, 0.0, 0.125);
    EXPECT_EQ(allocationCount.load(std::memory_order_relaxed), 0U);
}

TEST(FlatRungeKutta, PostBindToleranceRefreshMatchesPreBindConfiguration)
{
    TestDynamics preBindDynamics;
    TestDynamics postBindDynamics;
    svIntegratorAdaptiveRungeKutta<4> preBindIntegrator(&preBindDynamics, bogackiShampineCoefficients(), 3.0);
    svIntegratorAdaptiveRungeKutta<4> postBindIntegrator(&postBindDynamics, bogackiShampineCoefficients(), 3.0);

    configureTolerances(preBindIntegrator, preBindDynamics);
    stepIntegrator(postBindIntegrator, 0.0, 0.0);
    configureTolerances(postBindIntegrator, postBindDynamics);

    stepIntegrator(preBindIntegrator, 0.0, 0.125);
    allocationCount.store(0, std::memory_order_relaxed);
    stepIntegrator(postBindIntegrator, 0.0, 0.125);

    EXPECT_EQ(allocationCount.load(std::memory_order_relaxed), 0U);
    EXPECT_EQ(preBindDynamics.equationsOfMotionCalls, postBindDynamics.equationsOfMotionCalls);
    expectStateBitsEqual(preBindDynamics, postBindDynamics);
}

TEST(FlatRungeKutta, ToleranceRefreshPreservesOverridePrecedence)
{
    TestDynamics dynamics;
    AdaptiveToleranceProbe<4> integrator(&dynamics, bogackiShampineCoefficients(), 3.0);
    const std::string matrixStateName = "matrixStateWithANameLongEnoughToAllocate";

    EXPECT_EQ(integrator.resolvedTolerance(0, matrixStateName), std::make_pair(1e-4, 1e-8));

    integrator.setRelativeTolerance(2.5e-5);
    integrator.setAbsoluteTolerance(3.0e-9);
    EXPECT_EQ(integrator.resolvedTolerance(0, matrixStateName), std::make_pair(2.5e-5, 3.0e-9));

    integrator.setRelativeTolerance(matrixStateName, 4.0e-6);
    integrator.setAbsoluteTolerance(matrixStateName, 5.0e-10);
    EXPECT_EQ(integrator.resolvedTolerance(0, matrixStateName), std::make_pair(4.0e-6, 5.0e-10));

    integrator.setRelativeTolerance(dynamics, matrixStateName, 6.0e-7);
    integrator.setAbsoluteTolerance(dynamics, matrixStateName, 7.0e-11);
    EXPECT_EQ(integrator.resolvedTolerance(0, matrixStateName), std::make_pair(6.0e-7, 7.0e-11));

    integrator.setRelativeTolerance(matrixStateName, 8.0e-8);
    integrator.setAbsoluteTolerance(matrixStateName, 9.0e-12);
    EXPECT_EQ(integrator.resolvedTolerance(0, matrixStateName), std::make_pair(6.0e-7, 7.0e-11));
}

TEST(FlatRungeKutta, DirectPublicToleranceWritesRefreshCachedSpans)
{
    TestDynamics dynamics;
    AdaptiveToleranceProbe<4> integrator(&dynamics, bogackiShampineCoefficients(), 3.0);

    EXPECT_EQ(integrator.resolvedTolerance(0, "vectorState"), std::make_pair(1e-4, 1e-8));

    integrator.relTol = 2.5e-6;
    integrator.absTol = 3.0e-11;
    EXPECT_EQ(integrator.resolvedTolerance(0, "vectorState"), std::make_pair(2.5e-6, 3.0e-11));

    integrator.relTol = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(integrator.resolvedTolerance(0, "vectorState"), std::invalid_argument);
}

TEST(FlatRungeKutta, StateSettersPreserveContiguousAndStridedMatrixValues)
{
    DynParamManager manager;
    StateSpec spec;
    spec.state = spec.derivative = spec.diffusionTangent = { 3, 2 };
    spec.noiseCount = 1;
    StateData* state = manager.registerState("matrix", spec);
    manager.finalizeStates();

    Eigen::MatrixXd source(5, 3);
    source << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
    const auto strided = source.block(1, 1, 3, 2);
    const Eigen::MatrixXd expected = strided;
    ASSERT_GT(strided.outerStride(), strided.rows());
    state->setState(strided);
    state->setDerivative(strided);
    state->setDiffusion(strided, 0);
    EXPECT_TRUE(state->stateView().isApprox(expected, 0.0));
    EXPECT_TRUE(state->derivativeView().isApprox(expected, 0.0));
    EXPECT_TRUE(state->diffusionView(0).isApprox(expected, 0.0));

    const Eigen::MatrixXd contiguous = -expected;
    state->setState(contiguous);
    state->setDerivative(contiguous);
    state->setDiffusion(contiguous, 0);
    state->setState(state->stateView());
    state->setDerivative(state->derivativeView());
    state->setDiffusion(state->diffusionView(0), 0);
    EXPECT_TRUE(state->stateView().isApprox(contiguous, 0.0));
    EXPECT_TRUE(state->derivativeView().isApprox(contiguous, 0.0));
    EXPECT_TRUE(state->diffusionView(0).isApprox(contiguous, 0.0));

    EXPECT_THROW(state->setState(source), std::invalid_argument);
    EXPECT_THROW(state->setDerivative(source), std::invalid_argument);
    EXPECT_THROW(state->setDiffusion(source, 0), std::invalid_argument);
    EXPECT_THROW(state->setDiffusion(contiguous, 1), std::out_of_range);
    EXPECT_TRUE(state->stateView().isApprox(contiguous, 0.0));
    EXPECT_TRUE(state->derivativeView().isApprox(contiguous, 0.0));
    EXPECT_TRUE(state->diffusionView(0).isApprox(contiguous, 0.0));
}

TEST(FlatRungeKutta, DeprecatedReferenceAccessorsReturnLiveConstViews)
{
    TestDynamics dynamics;
    const StateData& state = *dynamics.matrixState;
    const auto stateReference = state.getStateReference();
    const auto derivativeReference = state.getStateDerivReference();

    Eigen::MatrixXd replacement(2, 2);
    replacement << 4.0, 3.0, 2.0, 1.0;
    dynamics.matrixState->setState(replacement);
    Eigen::MatrixXd derivative(2, 2);
    derivative << -1.0, -2.0, -3.0, -4.0;
    dynamics.matrixState->setDerivative(derivative);

    EXPECT_EQ(stateReference, replacement);
    EXPECT_EQ(derivativeReference, derivative);
}

TEST(FlatRungeKutta, ToleranceSettersRejectInvalidValues)
{
    TestDynamics dynamics;
    svIntegratorAdaptiveRungeKutta<4> integrator(&dynamics, bogackiShampineCoefficients(), 3.0);

    EXPECT_THROW(integrator.setRelativeTolerance(-1.0), std::invalid_argument);
    EXPECT_THROW(integrator.setAbsoluteTolerance(std::numeric_limits<double>::infinity()), std::invalid_argument);
    EXPECT_THROW(integrator.setRelativeTolerance("vectorState", -1.0), std::invalid_argument);
    EXPECT_THROW(integrator.setAbsoluteTolerance(dynamics, "vectorState", -1.0), std::invalid_argument);
}

TEST(FlatRungeKutta, ObjectSpecificToleranceOverridesControlStepAcceptance)
{
    TestDynamics looseDynamics;
    TestDynamics overrideDynamics;
    TestDynamics strictDynamics;
    svIntegratorAdaptiveRungeKutta<4> looseIntegrator(&looseDynamics, bogackiShampineCoefficients(), 3.0);
    svIntegratorAdaptiveRungeKutta<4> overrideIntegrator(&overrideDynamics, bogackiShampineCoefficients(), 3.0);
    svIntegratorAdaptiveRungeKutta<4> strictIntegrator(&strictDynamics, bogackiShampineCoefficients(), 3.0);

    constexpr double looseRelative = 1e-4;
    constexpr double looseAbsolute = 1e-8;
    constexpr double strictRelative = 0.0;
    constexpr double strictAbsolute = 1e-12;
    looseIntegrator.setRelativeTolerance(looseRelative);
    looseIntegrator.setAbsoluteTolerance(looseAbsolute);
    overrideIntegrator.setRelativeTolerance(strictRelative);
    overrideIntegrator.setAbsoluteTolerance(strictAbsolute);
    strictIntegrator.setRelativeTolerance(strictRelative);
    strictIntegrator.setAbsoluteTolerance(strictAbsolute);

    for (const std::string& stateName : { "matrixStateWithANameLongEnoughToAllocate", "vectorState" }) {
        overrideIntegrator.setRelativeTolerance(overrideDynamics, stateName, looseRelative);
        overrideIntegrator.setAbsoluteTolerance(overrideDynamics, stateName, looseAbsolute);
    }

    stepIntegrator(looseIntegrator, 0.75, 0.125);
    stepIntegrator(overrideIntegrator, 0.75, 0.125);
    stepIntegrator(strictIntegrator, 0.75, 0.125);

    EXPECT_EQ(overrideDynamics.equationsOfMotionCalls, looseDynamics.equationsOfMotionCalls);
    expectStateBitsEqual(looseDynamics, overrideDynamics);
    EXPECT_GT(strictDynamics.equationsOfMotionCalls, looseDynamics.equationsOfMotionCalls);
}

TEST(FlatRungeKutta, BindingKeepsStateAndDerivativeExtentsIndependent)
{
    UnequalExtentDynamics dynamics;
    const Eigen::MatrixXd initialState = dynamics.state->getState();
    RKCoefficients<1> coefficients;
    coefficients.aMatrix = { { { { 0.0 } } } };
    coefficients.bArray = { { 0.0 } };
    coefficients.cArray = { { 0.0 } };
    svIntegratorRungeKutta<1> integrator(&dynamics, coefficients);

    EXPECT_NO_THROW(stepIntegrator(integrator, 0.0, 0.25));
    const Eigen::MatrixXd finalState = dynamics.state->getState();
    ASSERT_EQ(finalState.size(), initialState.size());
    for (Eigen::Index index = 0; index < initialState.size(); ++index) {
        EXPECT_EQ(doubleBits(finalState(index)), doubleBits(initialState(index)));
    }
    EXPECT_EQ(dynamics.state->derivativeView().size(), 3);
}


TEST(FlatRungeKutta, FinalizationAndRepeatedDeclarationsPreserveStorage)
{
    DynParamManager manager;
    StateData* alpha = manager.registerState(2, 1, "alpha");
    StateData* beta = manager.registerState(1, 2, "beta");
    alpha->stateView() << 1.0, 2.0;
    beta->stateView() << 3.0, 4.0;
    manager.finalizeStates();
    auto& registry = manager.getStateRegistry();
    const StateBufferSegment segment = registry.getStateSegment(0, 4);
    double* const live = registry.stateSegmentData(segment);
    double* const derivative = alpha->derivativeData();
    EXPECT_DOUBLE_EQ(live[0], 1.0);
    EXPECT_DOUBLE_EQ(live[3], 4.0);

    // Reset in a different declaration order; names identify the existing records.
    EXPECT_EQ(manager.registerState(1, 2, "beta"), beta);
    EXPECT_EQ(manager.registerState(2, 1, "alpha"), alpha);
    alpha->stateView() << 5.0, 6.0;
    beta->stateView() << 7.0, 8.0;
    manager.finalizeStates();
    EXPECT_EQ(registry.stateSegmentData(segment), live);
    EXPECT_EQ(alpha->derivativeData(), derivative);
    EXPECT_DOUBLE_EQ(live[0], 5.0);
    EXPECT_DOUBLE_EQ(live[3], 8.0);
    EXPECT_THROW(manager.registerState(3, 1, "alpha"), std::logic_error);
    EXPECT_THROW(manager.registerState(1, 1, "extra"), std::logic_error);

    DynParamManager other;
    other.finalizeStates();
    EXPECT_THROW(other.getStateRegistry().stateSegmentData(segment), std::logic_error);
    EXPECT_THROW(registry.derivativeSegmentData(segment), std::logic_error);
    EXPECT_THROW(registry.getStateSegment(0, 3), std::logic_error);
}

TEST(FlatRungeKutta, SurvivingDynamicsAdvanceAfterSynchronizedPeerDestruction)
{
    for (bool adaptive : { false, true }) {
        for (bool destroyPrimary : { false, true }) {
            SCOPED_TRACE(::testing::Message() << "adaptive=" << adaptive << ", destroyPrimary=" << destroyPrimary);
            auto primary = std::make_unique<ControlledScalarDynamics>();
            auto secondary = std::make_unique<ControlledScalarDynamics>();
            for (auto* object : { primary.get(), secondary.get() }) {
                if (adaptive) {
                    object->setIntegrator(
                      new svIntegratorAdaptiveRungeKutta<4>(object, bogackiShampineCoefficients(), 3.0));
                } else {
                    object->setIntegrator(new svIntegratorRungeKutta<4>(object, rk4Coefficients()));
                }
                object->timeStep = 1.0; // [s]
            }
            primary->syncDynamicsIntegration(secondary.get());
            primary->integrateState(0);
            EXPECT_DOUBLE_EQ(primary->state->stateView()(0, 0), 2.0);
            EXPECT_DOUBLE_EQ(secondary->state->stateView()(0, 0), 2.0);

            auto* survivor = destroyPrimary ? secondary.get() : primary.get();
            if (destroyPrimary) {
                primary.reset();
            } else {
                secondary.reset();
            }
            EXPECT_EQ(survivor->getIntegrationOwner(), nullptr);
            EXPECT_EQ(survivor->getIntegrator()->getDynamicsCount(), 1U);
            survivor->integrateState(0);
            EXPECT_DOUBLE_EQ(survivor->state->stateView()(0, 0), 3.0);
        }
    }
}

TEST(FlatRungeKutta, AdaptivePeerDestructionMatchesFreshBinding)
{
    // Check both shifted offsets and a removed trailing span, without changing tolerances after binding.
    for (bool removeMiddle : { false, true }) {
        SCOPED_TRACE(::testing::Message() << "removeMiddle=" << removeMiddle);
        TestDynamics primary;
        TestDynamics survivor;
        auto removed = std::make_unique<ControlledScalarDynamics>();
        auto* integrator = new AdaptiveToleranceProbe<4>(&primary, bogackiShampineCoefficients(), 3.0);
        primary.setIntegrator(integrator);
        survivor.setIntegrator(new svIntegratorRungeKutta<4>(&survivor, rk4Coefficients()));
        removed->setIntegrator(new svIntegratorRungeKutta<4>(removed.get(), rk4Coefficients()));
        if (removeMiddle) {
            primary.syncDynamicsIntegration(removed.get());
            primary.syncDynamicsIntegration(&survivor);
        } else {
            primary.syncDynamicsIntegration(&survivor);
            primary.syncDynamicsIntegration(removed.get());
        }
        configureTolerances(*integrator, survivor);
        integrator->setAbsoluteTolerance(*removed, "controlledScalar", 1.0); // [-]

        const double timeStep = 0.125; // [s]
        stepIntegrator(integrator, 0.0, timeStep);
        ASSERT_EQ(integrator->resolvedSpans().size(), 5U);
        removed.reset();

        // A fresh two-object group starts from the surviving states and has the same tolerance configuration.
        TestDynamics referencePrimary;
        TestDynamics referenceSurvivor;
        referencePrimary.matrixState->setState(primary.matrixState->stateView());
        referencePrimary.vectorState->setState(primary.vectorState->stateView());
        referenceSurvivor.matrixState->setState(survivor.matrixState->stateView());
        referenceSurvivor.vectorState->setState(survivor.vectorState->stateView());
        auto* reference =
          new AdaptiveToleranceProbe<4>(&referencePrimary, bogackiShampineCoefficients(), 3.0);
        referencePrimary.setIntegrator(reference);
        referenceSurvivor.setIntegrator(new svIntegratorRungeKutta<4>(&referenceSurvivor, rk4Coefficients()));
        referencePrimary.syncDynamicsIntegration(&referenceSurvivor);
        configureTolerances(*reference, referenceSurvivor);

        // Bind without advancing so stale spans fail deterministically before any out-of-bounds access.
        stepIntegrator(integrator, timeStep, 0.0);
        stepIntegrator(reference, timeStep, 0.0);
        const auto& actualSpans = integrator->resolvedSpans();
        const auto& expectedSpans = reference->resolvedSpans();
        ASSERT_EQ(actualSpans.size(), expectedSpans.size());
        for (size_t index = 0; index < expectedSpans.size(); ++index) {
            const auto& actual = actualSpans[index];
            const auto& expected = expectedSpans[index];
            EXPECT_EQ(actual.globalStateOffset, expected.globalStateOffset);
            EXPECT_EQ(actual.rows, expected.rows);
            EXPECT_EQ(actual.columns, expected.columns);
            EXPECT_EQ(actual.mode, expected.mode);
            EXPECT_DOUBLE_EQ(actual.relative, expected.relative);
            EXPECT_DOUBLE_EQ(actual.absolute, expected.absolute);
        }

        for (size_t step = 1; step <= 2; ++step) {
            primary.equationsOfMotionCalls = 0;
            survivor.equationsOfMotionCalls = 0;
            referencePrimary.equationsOfMotionCalls = 0;
            referenceSurvivor.equationsOfMotionCalls = 0;
            const double currentTime = static_cast<double>(step) * timeStep; // [s]
            stepIntegrator(integrator, currentTime, timeStep);
            stepIntegrator(reference, currentTime, timeStep);
            EXPECT_GT(primary.equationsOfMotionCalls, 4U);
            EXPECT_EQ(primary.equationsOfMotionCalls, referencePrimary.equationsOfMotionCalls);
            EXPECT_EQ(survivor.equationsOfMotionCalls, referenceSurvivor.equationsOfMotionCalls);
            expectStateBitsEqual(primary, referencePrimary);
            expectStateBitsEqual(survivor, referenceSurvivor);
        }
    }
}
