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

#include "simulation/dynamics/_GeneralModuleFiles/stochasticRKIntegratorBase.h"
#include "simulation/dynamics/Integrators/svStochasticIntegratorMayurama.h"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// The cache implementation is compiled into this executable so this counter
// observes its STL allocations on all platforms, including shared-library builds.
namespace allocationCounting {
thread_local bool enabled = false;
thread_local size_t count = 0;
}

void* operator new(std::size_t size)
{
    if (allocationCounting::enabled) ++allocationCounting::count;
    if (void* pointer = std::malloc(size == 0 ? 1 : size)) return pointer;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept { std::free(pointer); }

namespace {
/** @brief Count C++ allocations only inside the measured operation. */
class AllocationScope {
public:
    AllocationScope() { allocationCounting::count = 0; allocationCounting::enabled = true; }
    ~AllocationScope() { allocationCounting::enabled = false; }
};

/** @brief Expose the prepared propagation path for comparison with the public map path. */
class RoutingProbe : public StochasticRKIntegratorBase {
public:
    using StochasticRKIntegratorBase::StochasticRKIntegratorBase;
    using StochasticRKIntegratorBase::noiseIndexMaps;
    using StochasticRKIntegratorBase::propagateStateWithCachedNoise;
    void integrate(double, double) override {}
};

/** @brief Small dimensionless dynamics with configurable shapes and shared noise. */
class TestDynamics : public DynamicObject {
public:
    TestDynamics* partner = nullptr;
    std::vector<std::pair<double, double>> driftTimes;
    std::vector<std::pair<double, double>> diffusionTimes;

    StateData* add(const std::string& name, size_t sources = 1, uint32_t rows = 1, uint32_t columns = 1)
    {
        auto* state = dynManager.registerState(rows, columns, name);
        state->setState(Eigen::MatrixXd::Constant(rows, columns, 0.375)); // [-]
        state->setNumNoiseSources(sources);
        return state;
    }
    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}
    void equationsOfMotion(double time, double step) override
    {
        driftTimes.emplace_back(time, step);
        const double decay = -0.25; // [1/s]
        const double coupling = 0.0625; // [1/s]
        const double ramp = 0.03125; // [1/s^2]
        const double other = partner && !partner->dynManager.stateContainer.stateMap.empty()
            ? partner->dynManager.stateContainer.stateMap.begin()->second->state(0, 0) : 0.0; // [-]
        for (auto& [name, state] : dynManager.stateContainer.stateMap) {
            const double value = decay * state->state(0, 0) + coupling * other + ramp * time;
            state->setDerivative(Eigen::MatrixXd::Constant(state->stateDeriv.rows(), state->stateDeriv.cols(), value));
        }
    }
    void equationsOfMotionDiffusion(double time, double step) override
    {
        diffusionTimes.emplace_back(time, step);
        for (auto& [name, state] : dynManager.stateContainer.stateMap) {
            for (size_t index = 0; index < state->getNumNoiseSources(); ++index) {
                const double scale = 0.125 * static_cast<double>(index + 1); // [1/sqrt(s)]
                const auto& previous = state->stateDiffusion[index];
                state->setDiffusion(Eigen::MatrixXd::Constant(previous.rows(), previous.cols(),
                    scale * (1.0 + state->state(0, 0))), index);
            }
        }
    }
};

/** @brief Preserve the pre-optimization Euler recurrence using fresh source maps. */
class ReferenceEuler : public StochasticRKIntegratorBase {
public:
    using StochasticRKIntegratorBase::StochasticRKIntegratorBase;
    void integrate(double time, double step) override
    {
        if (step == 0.0) return;
        const auto maps = getStateIdToNoiseIndexMaps();
        const auto sample = rvGenerator->generate(maps.size(), step);
        computeDerivatives(time, step).setDerivatives(dynPtrs);
        const auto diffusions = computeDiffusions(time, step, maps);
        for (size_t source = 0; source < maps.size(); ++source) diffusions[source].setDiffusions(dynPtrs, maps[source]);
        propagateState(step, sample.dW, maps);
    }
};

/** @brief Compare all registered matrices without allowing matching NaNs. */
void expectStatesEqual(const ExtendedStateVector& expected, const ExtendedStateVector& actual)
{
    ASSERT_EQ(expected.size(), actual.size());
    for (const auto& [id, values] : expected) {
        ASSERT_EQ(values.rows(), actual.at(id).rows());
        ASSERT_EQ(values.cols(), actual.at(id).cols());
        EXPECT_TRUE(values.allFinite());
        EXPECT_TRUE(actual.at(id).allFinite());
        EXPECT_TRUE((values.array() == actual.at(id).array()).all());
    }
}

/** @brief Compare prepared routing with the uncached public method on the same live layout. */
void checkRouting(RoutingProbe& integrator)
{
    const auto maps = integrator.getStateIdToNoiseIndexMaps();
    EXPECT_EQ(integrator.noiseIndexMaps(), maps);
    Eigen::VectorXd increments(static_cast<Eigen::Index>(maps.size()));
    for (Eigen::Index source = 0; source < increments.size(); ++source)
        increments(source) = 0.125 * static_cast<double>(source % 3 - 1); // [sqrt(s)]
    const double time = 0.5; // [s]
    const double step = 0.125; // [s]
    for (auto* object : integrator.dynPtrs) object->equationsOfMotion(time, step);
    for (auto* object : integrator.dynPtrs) object->equationsOfMotionDiffusion(time, step);
    const auto initial = ExtendedStateVector::fromStates(integrator.dynPtrs);
    integrator.propagateState(step, increments, maps);
    const auto expected = ExtendedStateVector::fromStates(integrator.dynPtrs);
    initial.setStates(integrator.dynPtrs);
    integrator.propagateStateWithCachedNoise(step, increments);
    expectStatesEqual(expected, ExtendedStateVector::fromStates(integrator.dynPtrs));
}

TEST(StochasticRouting, SharedSparseNoiseAndIndependentObjects)
{
    TestDynamics first, second;
    auto* firstA = first.add("a", 2, 2, 3);
    auto* firstB = first.add("b", 1);
    first.add("deterministic", 0);
    auto* secondA = second.add("a", 2);
    auto* secondB = second.add("b", 1, 3, 1);
    first.dynManager.registerSharedNoiseSource({{*firstA, 1}, {*firstB, 0}});
    second.dynManager.registerSharedNoiseSource({{*secondA, 0}, {*secondB, 0}});
    RoutingProbe integrator(&first);
    integrator.dynPtrs.push_back(&second);
    const auto& maps = integrator.noiseIndexMaps();
    ASSERT_EQ(maps.size(), 4u);
    EXPECT_EQ(maps[0].at({0, "a"}), 0u);
    EXPECT_EQ(maps[1].at({0, "a"}), 1u);
    EXPECT_EQ(maps[1].at({0, "b"}), 0u);
    EXPECT_EQ(maps[2].at({1, "a"}), 0u);
    EXPECT_EQ(maps[2].at({1, "b"}), 0u);
    EXPECT_EQ(maps[3].at({1, "a"}), 1u);
    checkRouting(integrator);
}

TEST(StochasticRouting, AddRemoveReplaceRenameAndReshapeStates)
{
    TestDynamics model;
    model.add("a", 1);
    RoutingProbe integrator(&model);
    checkRouting(integrator);
    model.add("b", 2, 2, 3);
    checkRouting(integrator);
    model.dynManager.stateContainer.stateMap.erase("a");
    checkRouting(integrator);
    // Keep the old allocation alive to ensure replacement uses a different address.
    auto old = std::move(model.dynManager.stateContainer.stateMap.at("b"));
    auto replacement = std::make_unique<StateData>("b", Eigen::MatrixXd::Constant(3, 1, 0.5)); // [-]
    replacement->setNumNoiseSources(2);
    model.dynManager.stateContainer.stateMap.at("b") = std::move(replacement);
    checkRouting(integrator);
    auto node = model.dynManager.stateContainer.stateMap.extract("b");
    node.key() = "renamed";
    model.dynManager.stateContainer.stateMap.insert(std::move(node));
    checkRouting(integrator);
    auto& state = *model.dynManager.stateContainer.stateMap.at("renamed");
    state.setState(Eigen::MatrixXd::Constant(1, 3, 0.25)); // [-], reshape with the same element count
    state.setDerivative(Eigen::MatrixXd::Zero(1, 3));
    state.setNumNoiseSources(2);
    checkRouting(integrator);
    state.setState(Eigen::MatrixXd::Constant(4, 2, 0.25)); // [-]
    state.setDerivative(Eigen::MatrixXd::Zero(4, 2));
    state.setNumNoiseSources(2);
    checkRouting(integrator);
    model.dynManager.stateContainer.stateMap.clear();
    checkRouting(integrator);
    model.add("new", 3);
    checkRouting(integrator);
}

TEST(StochasticRouting, NoiseCountsAndSharingChangeWithoutChangingStatePointers)
{
    TestDynamics model;
    auto* first = model.add("a", 1);
    auto* second = model.add("b", 1);
    RoutingProbe integrator(&model);
    checkRouting(integrator);
    first->setNumNoiseSources(2);
    checkRouting(integrator);
    model.dynManager.registerSharedNoiseSource({{*first, 1}, {*second, 0}});
    ASSERT_EQ(integrator.noiseIndexMaps().size(), 2u);
    checkRouting(integrator);
    // Keep the source count fixed while changing which local channel is shared.
    model.dynManager.sharedNoiseMap.clear();
    model.dynManager.registerSharedNoiseSource({{*first, 0}, {*second, 0}});
    ASSERT_EQ(integrator.noiseIndexMaps().size(), 2u);
    checkRouting(integrator);
    model.dynManager.sharedNoiseMap.clear();
    checkRouting(integrator);
    first->setNumNoiseSources(0);
    second->setNumNoiseSources(0);
    checkRouting(integrator);
}

TEST(StochasticRouting, ObjectAdditionRemovalReorderingAndEmptyLayouts)
{
    TestDynamics first, second, empty;
    first.add("sameName", 1);
    second.add("sameName", 2);
    RoutingProbe integrator(&first);
    checkRouting(integrator);
    integrator.dynPtrs.push_back(&second);
    checkRouting(integrator);
    std::swap(integrator.dynPtrs[0], integrator.dynPtrs[1]);
    checkRouting(integrator);
    integrator.dynPtrs.insert(integrator.dynPtrs.begin(), &empty);
    checkRouting(integrator);
    integrator.dynPtrs.erase(integrator.dynPtrs.begin() + 1);
    checkRouting(integrator);
    integrator.dynPtrs = {&second};
    checkRouting(integrator);
    integrator.dynPtrs.clear();
    checkRouting(integrator);
    integrator.dynPtrs = {&empty};
    checkRouting(integrator);
    empty.add("afterEmpty", 1);
    checkRouting(integrator);
}

TEST(StochasticRouting, OmittedLocalChannelRetainsPublicMappingSemantics)
{
    TestDynamics model;
    auto* state = model.add("a", 2);
    model.dynManager.registerSharedNoiseSource({{*state, 0}, {*state, 1}});
    RoutingProbe integrator(&model);
    ASSERT_EQ(integrator.noiseIndexMaps().size(), 1u);
    checkRouting(integrator);
}

/** @brief Two-component state with scalar drift, scalar diffusions, and a transforming setter. */
class MappedState : public StateData {
public:
    explicit MappedState(const std::string& name) : StateData(name, Eigen::MatrixXd::Zero(2, 1))
    {
        stateDeriv = Eigen::MatrixXd::Zero(1, 1);
        setNumNoiseSources(2);
        for (auto& diffusion : stateDiffusion) diffusion = Eigen::MatrixXd::Zero(1, 1);
    }
    size_t setterCalls = 0;
    size_t propagationCalls = 0;
    std::vector<std::vector<double>> received;
    void setDerivative(const Eigen::MatrixXd& derivative) override
    {
        ++setterCalls;
        stateDeriv = 2.0 * derivative;
    }
    void propagateState(double step, std::vector<double> increments = {}) override
    {
        ++propagationCalls;
        received.push_back(increments);
        ASSERT_EQ(increments.size(), 2u);
        double change = step * stateDeriv(0, 0);
        for (size_t source = 0; source < increments.size(); ++source)
            change += stateDiffusion[source](0, 0) * increments[source];
        state(0, 0) += change;
        state(1, 0) += 2.0 * change;
    }
};

TEST(StochasticRouting, PreservesVirtualPropagationAndUnequalDimensions)
{
    TestDynamics model;
    auto value = std::make_unique<MappedState>("mapped");
    auto* state = value.get();
    model.dynManager.stateContainer.stateMap.emplace("mapped", std::move(value));
    RoutingProbe integrator(&model);
    checkRouting(integrator);
    ASSERT_EQ(state->propagationCalls, 2u);
    EXPECT_EQ(state->received[0], state->received[1]);
    EXPECT_EQ(state->state(1, 0), 2.0 * state->state(0, 0));
}

TEST(StochasticRouting, ZeroDriftStepStillAppliesSignedNoise)
{
    TestDynamics model;
    auto* state = model.add("a", 2);
    state->setState(Eigen::MatrixXd::Zero(1, 1));
    state->setDerivative(Eigen::MatrixXd::Constant(1, 1, 8.0)); // [1/s]
    state->setDiffusion(Eigen::MatrixXd::Constant(1, 1, 2.0), 0); // [1/sqrt(s)]
    state->setDiffusion(Eigen::MatrixXd::Constant(1, 1, 3.0), 1); // [1/sqrt(s)]
    RoutingProbe integrator(&model);
    integrator.noiseIndexMaps();
    Eigen::VectorXd increments(2);
    increments << 0.25, -0.5; // [sqrt(s)]
    integrator.propagateStateWithCachedNoise(0.0, increments);
    EXPECT_DOUBLE_EQ(state->state(0, 0), -1.0);
    increments.setZero();
    integrator.propagateStateWithCachedNoise(0.0, increments);
    EXPECT_DOUBLE_EQ(state->state(0, 0), -1.0);
}

TEST(StochasticRouting, PublicMappingArgumentRemainsIndependentOfPreparedRouting)
{
    TestDynamics model;
    auto* state = model.add("a", 2);
    state->setState(Eigen::MatrixXd::Zero(1, 1));
    state->setDiffusion(Eigen::MatrixXd::Constant(1, 1, 2.0), 0); // [1/sqrt(s)]
    state->setDiffusion(Eigen::MatrixXd::Constant(1, 1, 3.0), 1); // [1/sqrt(s)]
    RoutingProbe integrator(&model);
    auto maps = integrator.noiseIndexMaps();
    std::swap(maps[0], maps[1]);
    Eigen::VectorXd increments(2);
    increments << 0.25, -0.5; // [sqrt(s)]
    integrator.propagateState(0.0, increments, maps);
    EXPECT_DOUBLE_EQ(state->state(0, 0), -0.25);
}

TEST(StochasticRouting, RejectsUnpreparedOrWrongLengthBeforePropagation)
{
    TestDynamics model;
    auto* state = model.add("a", 1);
    RoutingProbe integrator(&model);
    const Eigen::VectorXd empty;
    EXPECT_THROW(integrator.propagateStateWithCachedNoise(1.0, empty), std::invalid_argument);
    integrator.noiseIndexMaps();
    const auto initial = state->getState();
    EXPECT_THROW(integrator.propagateStateWithCachedNoise(1.0, empty), std::invalid_argument);
    EXPECT_TRUE((initial.array() == state->state.array()).all());
}

TEST(StochasticRouting, WarmRoutingAllocatesOnlyLegacyByValueArguments)
{
    TestDynamics model;
    auto* first = model.add("long_state_name_for_first_channel", 2);
    auto* second = model.add("long_state_name_for_second_channel", 1);
    model.dynManager.registerSharedNoiseSource({{*first, 1}, {*second, 0}});
    model.add("deterministic", 0);
    RoutingProbe integrator(&model);
    for (int layout = 0; layout < 3; ++layout) {
        if (layout == 1) model.add("another_stochastic_state", 3);
        if (layout == 2) model.dynManager.stateContainer.stateMap.erase("long_state_name_for_first_channel");
        const auto& maps = integrator.noiseIndexMaps();
        const Eigen::VectorXd increments = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(maps.size()));
        size_t noisyStates = 0;
        for (const auto& [name, state] : model.dynManager.stateContainer.stateMap)
            if (state->getNumNoiseSources() != 0) ++noisyStates;
        {
            AllocationScope scope;
            for (int repeat = 0; repeat < 8; ++repeat) {
                integrator.noiseIndexMaps();
                integrator.propagateStateWithCachedNoise(0.0, increments);
            }
        }
        EXPECT_EQ(allocationCounting::count, 8 * noisyStates);
        // Positive control: rebuilding the public maps costs additional allocations.
        {
            AllocationScope scope;
            integrator.propagateState(0.0, increments, maps);
        }
        EXPECT_GT(allocationCounting::count, noisyStates);
    }
}

TEST(StochasticEuler, SeededCoupledObjectsMatchRetainedRecurrence)
{
    TestDynamics model, partner, referenceModel, referencePartner;
    for (auto* object : {&model, &partner, &referenceModel, &referencePartner}) {
        auto* first = object->add("a", 2, 2, 3);
        auto* second = object->add("b", 1);
        object->add("deterministic", 0);
        object->dynManager.registerSharedNoiseSource({{*first, 1}, {*second, 0}});
    }
    model.partner = &partner;
    partner.partner = &model;
    referenceModel.partner = &referencePartner;
    referencePartner.partner = &referenceModel;
    svStochasticIntegratorMayurama integrator(&model);
    ReferenceEuler reference(&referenceModel);
    integrator.dynPtrs.push_back(&partner);
    reference.dynPtrs.push_back(&referencePartner);
    integrator.setRNGSeed(713);
    reference.setRNGSeed(713);
    const double step = 0.03125; // [s]
    for (int index = 0; index < 100; ++index) {
        if (index == 50) { integrator.setRNGSeed(19); reference.setRNGSeed(19); }
        const double time = step * index;
        integrator.integrate(time, step);
        reference.integrate(time, step);
        expectStatesEqual(ExtendedStateVector::fromStates(reference.dynPtrs), ExtendedStateVector::fromStates(integrator.dynPtrs));
    }
    EXPECT_EQ(model.driftTimes, referenceModel.driftTimes);
    EXPECT_EQ(partner.driftTimes, referencePartner.driftTimes);
    EXPECT_EQ(model.diffusionTimes, referenceModel.diffusionTimes);
    EXPECT_EQ(partner.diffusionTimes, referencePartner.diffusionTimes);
}

TEST(StochasticEuler, PrescribedNoiseTopologyChangesAndGeneratorReplacement)
{
    TestDynamics model, referenceModel;
    auto* first = model.add("a", 1);
    auto* second = model.add("b", 1);
    auto* referenceFirst = referenceModel.add("a", 1);
    auto* referenceSecond = referenceModel.add("b", 1);
    svStochasticIntegratorMayurama integrator(&model);
    ReferenceEuler reference(&referenceModel);
    auto noise = std::make_shared<PrescribedGaussianNoiseGenerator>();
    auto referenceNoise = std::make_shared<PrescribedGaussianNoiseGenerator>();
    integrator.setNoiseGenerator(noise);
    reference.setNoiseGenerator(referenceNoise);
    noise->pushStep({0.125, -0.25}); // [sqrt(s)]
    referenceNoise->pushStep({0.125, -0.25}); // [sqrt(s)]
    integrator.integrate(0.0, 0.0);
    EXPECT_EQ(noise->remaining(), 1u);
    EXPECT_TRUE(model.driftTimes.empty());
    const double step = 0.125; // [s]
    integrator.integrate(0.0, step);
    reference.integrate(0.0, step);
    model.dynManager.registerSharedNoiseSource({{*first, 0}, {*second, 0}});
    referenceModel.dynManager.registerSharedNoiseSource({{*referenceFirst, 0}, {*referenceSecond, 0}});
    noise->pushStep({-0.0625}); // [sqrt(s)]
    referenceNoise->pushStep({-0.0625}); // [sqrt(s)]
    integrator.integrate(step, step);
    reference.integrate(step, step);
    expectStatesEqual(ExtendedStateVector::fromStates(reference.dynPtrs), ExtendedStateVector::fromStates(integrator.dynPtrs));
    EXPECT_EQ(noise->remaining(), 0u);
    auto replacement = std::make_shared<PrescribedGaussianNoiseGenerator>();
    replacement->pushStep({0.25}); // [sqrt(s)]
    integrator.setNoiseGenerator(replacement);
    referenceNoise->pushStep({0.25}); // [sqrt(s)]
    integrator.integrate(2.0 * step, step);
    reference.integrate(2.0 * step, step);
    expectStatesEqual(ExtendedStateVector::fromStates(reference.dynPtrs), ExtendedStateVector::fromStates(integrator.dynPtrs));
    EXPECT_EQ(replacement->remaining(), 0u);
}

TEST(StochasticEuler, PreservesCustomDerivativeSetterAndMappedPropagation)
{
    TestDynamics model, referenceModel;
    auto state = std::make_unique<MappedState>("mapped");
    auto referenceState = std::make_unique<MappedState>("mapped");
    auto* value = state.get();
    auto* referenceValue = referenceState.get();
    model.dynManager.stateContainer.stateMap.emplace("mapped", std::move(state));
    referenceModel.dynManager.stateContainer.stateMap.emplace("mapped", std::move(referenceState));
    svStochasticIntegratorMayurama integrator(&model);
    ReferenceEuler reference(&referenceModel);
    integrator.setRNGSeed(87);
    reference.setRNGSeed(87);
    const double step = 0.125; // [s]
    for (int index = 0; index < 5; ++index) {
        integrator.integrate(index * step, step);
        reference.integrate(index * step, step);
        expectStatesEqual(ExtendedStateVector::fromStates(reference.dynPtrs), ExtendedStateVector::fromStates(integrator.dynPtrs));
    }
    EXPECT_EQ(value->setterCalls, 10u);
    EXPECT_EQ(value->setterCalls, referenceValue->setterCalls);
    EXPECT_EQ(value->received, referenceValue->received);
}

TEST(StochasticEuler, DeterministicLimitMatchesAnalyticEulerStep)
{
    TestDynamics model;
    auto* state = model.add("a", 0);
    svStochasticIntegratorMayurama integrator(&model);
    const double step = 0.125; // [s]
    integrator.integrate(0.0, step);
    EXPECT_DOUBLE_EQ(state->state(0, 0), 0.375 * (1.0 - 0.25 * step));
}
} // namespace
