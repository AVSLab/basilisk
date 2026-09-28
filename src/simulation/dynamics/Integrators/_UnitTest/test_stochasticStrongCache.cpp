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

#include "stochasticStrongReference.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#if !defined(EIGEN_RUNTIME_NO_MALLOC) || defined(EIGEN_NO_DEBUG)
#error "Stage cache checks require Eigen allocation assertions."
#endif

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
/** @brief Disable Eigen allocations and count C++ allocations within one measured call. */
class AllocationScope {
public:
    AllocationScope() : previous(Eigen::internal::is_malloc_allowed()) {
        allocationCounting::count = 0;
        allocationCounting::enabled = true;
        Eigen::internal::set_is_malloc_allowed(false);
    }
    ~AllocationScope() {
        Eigen::internal::set_is_malloc_allowed(previous);
        allocationCounting::enabled = false;
    }
private:
    bool previous;
};

/** @brief Exempt only the unchanged value-returning generator from the Eigen allocation guard. */
class TestNoise : public RandomGaussianNoiseGenerator {
public:
    size_t calls = 0;
    GaussianNoiseSample generate(size_t count, double step) override {
        ++calls;
        const bool allowed = Eigen::internal::is_malloc_allowed();
        Eigen::internal::set_is_malloc_allowed(true);
        auto sample = RandomGaussianNoiseGenerator::generate(count, step);
        Eigen::internal::set_is_malloc_allowed(allowed);
        return sample;
    }
};

/** @brief Record evaluation order, times, and the state seen by each callback. */
struct Evaluation {
    bool diffusion;
    double time;
    double step;
    std::vector<double> values;
};

/** @brief Dimensionless state with a scalar derivative and nonlinear, ordered noise propagation. */
class MappedState : public StateData {
public:
    explicit MappedState(const std::string& name) : StateData(name, Eigen::MatrixXd::Zero(2, 1)) {
        stateDeriv = Eigen::MatrixXd::Zero(1, 1);
        setNumNoiseSources(2);
        for (auto& diffusion : stateDiffusion) diffusion.resize(1, 1);
    }
    size_t setterCalls = 0;
    size_t propagationCalls = 0;
    void setDerivative(const Eigen::MatrixXd& derivative) override {
        ++setterCalls;
        stateDeriv = 2.0 * derivative;
    }
    void propagateState(double step, std::vector<double> increments = {}) override {
        ++propagationCalls;
        state(0, 0) += step * stateDeriv(0, 0);
        for (size_t source = 0; source < increments.size(); ++source) {
            const double change = stateDiffusion[source](0, 0) * increments[source];
            // Separate affine transformations make source/propagation order observable.
            state(0, 0) = (1.0 + 0.125 * change) * state(0, 0) + change;
        }
        state(1, 0) = 2.0 * state(0, 0);
    }
};

/** @brief Coupled matrix dynamics with preallocated callback storage. */
class Dynamics : public DynamicObject {
public:
    struct Entry { StateData* state; Eigen::MatrixXd work; };
    std::vector<Entry> entries;
    Dynamics* partner = nullptr;
    bool additive = false;
    bool record = true;
    std::vector<Evaluation> evaluations;

    StateData* add(const std::string& name, size_t sources = 1, uint32_t rows = 1, uint32_t columns = 1) {
        auto* state = dynManager.registerState(rows, columns, name);
        state->setNumNoiseSources(sources);
        state->state.setConstant(0.375); // [-]
        entries.push_back({state, {}});
        return state;
    }
    MappedState* addMapped() {
        auto state = std::make_unique<MappedState>("mapped");
        auto* pointer = state.get();
        dynManager.stateContainer.stateMap.emplace("mapped", std::move(state));
        entries.push_back({pointer, {}});
        return pointer;
    }
    void refreshEntries() {
        entries.clear();
        for (auto& [name, state] : dynManager.stateContainer.stateMap) entries.push_back({state.get(), {}});
    }
    void trace(bool diffusion, double time, double step) {
        if (!record) return;
        Evaluation evaluation{diffusion, time, step, {}};
        for (const auto& [name, state] : dynManager.stateContainer.stateMap) {
            const auto& matrix = state->state;
            for (Eigen::Index index = 0; index < matrix.size(); ++index) evaluation.values.push_back(matrix(index));
        }
        evaluations.push_back(std::move(evaluation));
    }
    void UpdateState(uint64_t) override {}
    void preIntegration(uint64_t) override {}
    void postIntegration(uint64_t) override {}
    void equationsOfMotion(double time, double step) override {
        trace(false, time, step);
        const double rate = -0.25; // [1/s]
        const double coupling = 0.0625; // [1/s]
        const double ramp = 0.03125; // [1/s^2]
        const double other = partner && !partner->entries.empty() ? partner->entries[0].state->state(0, 0) : 0.0;
        for (auto& entry : entries) {
            auto& state = *entry.state;
            entry.work.resizeLike(state.stateDeriv);
            for (Eigen::Index index = 0; index < entry.work.size(); ++index)
                entry.work(index) = rate * state.state(index % state.state.size()) + coupling * other + ramp * time;
            state.setDerivative(entry.work);
        }
    }
    void equationsOfMotionDiffusion(double time, double step) override {
        trace(true, time, step);
        for (auto& entry : entries) {
            auto& state = *entry.state;
            for (size_t source = 0; source < state.getNumNoiseSources(); ++source) {
                entry.work.resizeLike(state.stateDiffusion[source]);
                const double scale = 0.0625 * static_cast<double>(source + 1); // [1/sqrt(s)]
                for (Eigen::Index index = 0; index < entry.work.size(); ++index)
                    entry.work(index) = scale * (1.0 + 0.125 * time +
                        (additive ? 0.0 : state.state(index % state.state.size())));
                state.setDiffusion(entry.work, source);
            }
        }
    }
};

/** @brief Pair each production method with the independently retained recurrence. */
template<class Method, class OldMethod, bool additiveNoise, size_t stageCount>
struct Case {
    using Current = Method;
    using Reference = OldMethod;
    static constexpr bool additive = additiveNoise;
    static constexpr size_t stages = stageCount;
};
using Cases = ::testing::Types<
    Case<svStochasticIntegratorEulerHeun, stochasticReference::EulerHeun, false, 0>,
    Case<svStochasticIntegratorRKMil, stochasticReference::RKMil, false, 0>,
    Case<svStochasticIntegratorSRIW1, stochasticReference::SRI<svStochasticIntegratorSRIW1, 4>, false, 4>,
    Case<svStochasticIntegratorSOSRI, stochasticReference::SRI<svStochasticIntegratorSOSRI, 4>, false, 4>,
    Case<svStochasticIntegratorSRA1, stochasticReference::SRA<svStochasticIntegratorSRA1, 2>, true, 2>,
    Case<svStochasticIntegratorSOSRA, stochasticReference::SRA<svStochasticIntegratorSOSRA, 3>, true, 3>>;

template<class Configuration>
class StrongStochasticCache : public ::testing::Test {
public:
    Dynamics currentModel, oldModel, currentPartner, oldPartner;
    typename Configuration::Current current{&currentModel};
    typename Configuration::Reference old{&oldModel};
    std::shared_ptr<TestNoise> currentNoise = std::make_shared<TestNoise>();
    std::shared_ptr<TestNoise> oldNoise = std::make_shared<TestNoise>();
    double time = 0.0; // [s]
    const double step = 0.03125; // [s]
    void SetUp() override {
        for (auto* model : {&currentModel, &oldModel, &currentPartner, &oldPartner}) model->additive = Configuration::additive;
        current.setNoiseGenerator(currentNoise);
        old.setNoiseGenerator(oldNoise);
        current.setRNGSeed(641);
        old.setRNGSeed(641);
    }
    void compare() {
        currentModel.evaluations.clear(); oldModel.evaluations.clear();
        currentPartner.evaluations.clear(); oldPartner.evaluations.clear();
        current.integrate(time, step); old.integrate(time, step);
        const auto expected = ExtendedStateVector::fromStates(old.dynPtrs);
        const auto actual = ExtendedStateVector::fromStates(current.dynPtrs);
        ASSERT_EQ(expected.size(), actual.size());
        for (const auto& [id, matrix] : expected) {
            ASSERT_EQ(matrix.rows(), actual.at(id).rows());
            ASSERT_EQ(matrix.cols(), actual.at(id).cols());
            EXPECT_TRUE(matrix.allFinite()); EXPECT_TRUE(actual.at(id).allFinite());
            EXPECT_LE((matrix - actual.at(id)).norm(), 128.0 * std::numeric_limits<double>::epsilon() * (1.0 + matrix.norm()));
        }
        for (const auto& pair : {std::pair{&currentModel, &oldModel}, std::pair{&currentPartner, &oldPartner}}) {
            const auto& first = pair.first->evaluations;
            const auto& second = pair.second->evaluations;
            ASSERT_EQ(first.size(), second.size());
            for (size_t index = 0; index < first.size(); ++index) {
                EXPECT_EQ(first[index].diffusion, second[index].diffusion);
                EXPECT_DOUBLE_EQ(first[index].time, second[index].time);
                EXPECT_DOUBLE_EQ(first[index].step, second[index].step);
                ASSERT_EQ(first[index].values.size(), second[index].values.size());
                for (size_t component = 0; component < first[index].values.size(); ++component)
                    EXPECT_NEAR(first[index].values[component], second[index].values[component],
                                128.0 * std::numeric_limits<double>::epsilon() * (1.0 + std::abs(second[index].values[component])));
            }
        }
        EXPECT_EQ(currentNoise->calls, oldNoise->calls);
        time += step;
    }
};
TYPED_TEST_SUITE(StrongStochasticCache, Cases, );

TYPED_TEST(StrongStochasticCache, StagesMatchWithCouplingSharedNoiseAndMultipleObjects)
{
    for (auto* model : {&this->currentModel, &this->oldModel, &this->currentPartner, &this->oldPartner}) {
        auto* first = model->add("a", 2, 2, 3);
        auto* second = model->add("b", 1, 3, 1);
        model->add("deterministic", 0);
        model->dynManager.registerSharedNoiseSource({{*first, 1}, {*second, 0}});
    }
    this->currentModel.partner = &this->currentPartner;
    this->oldModel.partner = &this->oldPartner;
    this->currentPartner.partner = &this->currentModel;
    this->oldPartner.partner = &this->oldModel;
    this->current.dynPtrs.push_back(&this->currentPartner);
    this->old.dynPtrs.push_back(&this->oldPartner);
    for (int step = 0; step < 20; ++step) this->compare();
}

TYPED_TEST(StrongStochasticCache, RebuildsAfterRegistrationSharingObjectAndShapeChanges)
{
    this->compare(); // Empty objects, including the zero-source path.
    for (auto* model : {&this->currentModel, &this->oldModel}) model->add("a", 1, 2, 3);
    this->compare();
    for (auto* model : {&this->currentModel, &this->oldModel}) model->add("b", 1);
    this->compare();
    for (auto* model : {&this->currentModel, &this->oldModel}) {
        auto* first = model->entries[0].state;
        first->setNumNoiseSources(2);
        model->dynManager.registerSharedNoiseSource({{*first, 1}, {*model->entries[1].state, 0}});
    }
    this->compare();
    for (auto* model : {&this->currentModel, &this->oldModel}) {
        model->dynManager.sharedNoiseMap.clear();
        model->dynManager.registerSharedNoiseSource({{*model->entries[0].state, 0}, {*model->entries[1].state, 0}});
        auto* state = model->entries[0].state;
        state->state = Eigen::MatrixXd::Constant(3, 2, 0.375); // [-], same-sized reshape
        state->stateDeriv = Eigen::MatrixXd::Zero(3, 2);
        state->setNumNoiseSources(2);
    }
    this->compare();
    for (auto* model : {&this->currentModel, &this->oldModel}) {
        auto* state = model->entries[0].state;
        state->state = Eigen::MatrixXd::Constant(4, 3, 0.375); // [-]
        state->stateDeriv = Eigen::MatrixXd::Zero(4, 3);
        state->setNumNoiseSources(2);
        model->dynManager.sharedNoiseMap.clear();
        auto node = model->dynManager.stateContainer.stateMap.extract("a");
        node.key() = "renamed";
        model->dynManager.stateContainer.stateMap.insert(std::move(node));
        model->refreshEntries();
    }
    this->compare();
    this->currentPartner.add("partner", 1);
    this->oldPartner.add("partner", 1);
    this->current.dynPtrs.insert(this->current.dynPtrs.begin(), &this->currentPartner);
    this->old.dynPtrs.insert(this->old.dynPtrs.begin(), &this->oldPartner);
    this->compare();
    for (auto* model : {&this->currentModel, &this->oldModel}) {
        auto replacement = std::make_unique<StateData>("renamed", Eigen::MatrixXd::Constant(1, 1, 0.25)); // [-]
        replacement->setNumNoiseSources(1);
        model->dynManager.stateContainer.stateMap.at("renamed") = std::move(replacement);
        model->dynManager.stateContainer.stateMap.erase("b");
        model->refreshEntries();
    }
    this->compare();
    std::reverse(this->current.dynPtrs.begin(), this->current.dynPtrs.end());
    std::reverse(this->old.dynPtrs.begin(), this->old.dynPtrs.end());
    this->compare();
    this->current.dynPtrs.clear(); this->old.dynPtrs.clear();
    this->compare();
}

TYPED_TEST(StrongStochasticCache, VirtualSettersPropagationAndUnequalDimensions)
{
    auto* current = this->currentModel.addMapped();
    auto* old = this->oldModel.addMapped();
    for (int step = 0; step < 5; ++step) this->compare();
    EXPECT_EQ(current->setterCalls, old->setterCalls);
    EXPECT_EQ(current->propagationCalls, old->propagationCalls);
    EXPECT_GT(current->propagationCalls, 5u);
}

TYPED_TEST(StrongStochasticCache, ZeroStepReseedingAndGeneratorReplacement)
{
    this->currentModel.add("a"); this->oldModel.add("a");
    this->current.integrate(0.0, 0.0); this->old.integrate(0.0, 0.0);
    EXPECT_EQ(this->currentNoise->calls, 0u);
    this->compare();
    this->current.setRNGSeed(99); this->old.setRNGSeed(99);
    this->compare();
    this->currentNoise = std::make_shared<TestNoise>(); this->oldNoise = std::make_shared<TestNoise>();
    this->current.setNoiseGenerator(this->currentNoise); this->old.setNoiseGenerator(this->oldNoise);
    this->current.setRNGSeed(51); this->old.setRNGSeed(51);
    this->compare();
}

TYPED_TEST(StrongStochasticCache, WarmStageStorageDoesNotAllocateEigenOrAdditionalCppBuffers)
{
    this->currentModel.record = false;
    this->currentModel.add("long_state_name_one", 1);
    this->currentModel.add("long_state_name_two", 1, 3, 1);
    for (int layout = 0; layout < 3; ++layout) {
        if (layout == 1) this->currentModel.add("new_noise_source", 1, 2, 3);
        if (layout == 2) {
            auto* state = this->currentModel.entries[0].state;
            state->state = Eigen::MatrixXd::Constant(3, 2, 0.375); // [-]
            state->stateDeriv = Eigen::MatrixXd::Zero(3, 2);
            state->setNumNoiseSources(1);
        }
        this->current.integrate(0.0, this->step); // Warm every stage after the layout/shape change.
        const size_t sources = this->current.getStateIdToNoiseIndexMaps().size();
        size_t propagations;
        if constexpr (std::is_same_v<typename TypeParam::Current, svStochasticIntegratorEulerHeun>) propagations = 2;
        else if constexpr (std::is_same_v<typename TypeParam::Current, svStochasticIntegratorRKMil>) propagations = 4;
        else if constexpr (TypeParam::additive) propagations = TypeParam::stages + 1;
        else propagations = (TypeParam::stages - 1) * (sources + 1) + 4;
        {
            AllocationScope scope;
            this->current.integrate(this->step, this->step);
        }
        // One legacy by-value vector copy per noisy state and propagation call remains.
        EXPECT_EQ(allocationCounting::count, this->currentModel.entries.size() * propagations);
    }
}

/** @brief Inspect sum helpers without changing any solver tableau. */
class SumProbe : public StochasticRKIntegratorBase {
public:
    using StochasticRKIntegratorBase::StochasticRKIntegratorBase;
    using StochasticRKIntegratorBase::prepareStageBuffers;
    using StochasticRKIntegratorBase::derivativeStages;
    using StochasticRKIntegratorBase::diffusionStages;
    using StochasticRKIntegratorBase::applyDerivativeSum;
    using StochasticRKIntegratorBase::applyDiffusionSum;
    void integrate(double, double) override {}
};

TEST(StrongStochasticSums, NegativeAndZeroWeightsPreserveFirstTermSemantics)
{
    Dynamics model;
    auto* state = model.add("a");
    SumProbe integrator(&model);
    integrator.prepareStageBuffers(3, 3, 1, 0);
    const std::array<double, 3> values{2.0, 3.0, std::numeric_limits<double>::quiet_NaN()};
    for (size_t stage = 0; stage < values.size(); ++stage) {
        integrator.derivativeStages[stage][0] = Eigen::MatrixXd::Constant(1, 1, values[stage]);
        integrator.diffusionStages[stage][0][0] = Eigen::MatrixXd::Constant(1, 1, values[stage]);
    }
    const std::array<double, 3> weights{0.5, -1.0, 0.0};
    integrator.applyDerivativeSum(weights.data(), weights.size());
    integrator.applyDiffusionSum(0, weights.data(), weights.size());
    EXPECT_DOUBLE_EQ(state->stateDeriv(0, 0), -2.0);
    EXPECT_DOUBLE_EQ(state->stateDiffusion[0](0, 0), -2.0);
    integrator.derivativeStages[0][0].setConstant(std::numeric_limits<double>::quiet_NaN());
    integrator.diffusionStages[0][0][0].setConstant(std::numeric_limits<double>::quiet_NaN());
    const std::array<double, 3> zero{};
    integrator.applyDerivativeSum(zero.data(), zero.size());
    integrator.applyDiffusionSum(0, zero.data(), zero.size());
    EXPECT_TRUE(std::isnan(state->stateDeriv(0, 0)));
    EXPECT_TRUE(std::isnan(state->stateDiffusion[0](0, 0)));
}

TEST(StrongStochasticBuffers, DiffusionReferenceIsLiveAndCopyRemainsIndependent)
{
    StateData state("value", Eigen::MatrixXd::Zero(1, 1));
    state.setNumNoiseSources(1);
    const auto& reference = state.getStateDiffusionReference(0);
    const auto snapshot = state.getStateDiffusion(0);
    state.setDiffusion(Eigen::MatrixXd::Constant(1, 1, 0.25), 0); // [1/sqrt(s)]
    EXPECT_DOUBLE_EQ(reference(0, 0), 0.25);
    EXPECT_DOUBLE_EQ(snapshot(0, 0), 0.0);
    EXPECT_THROW(state.getStateDiffusionReference(1), std::out_of_range);
}
} // namespace
