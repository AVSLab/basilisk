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
#include "simulation/dynamics/_GeneralModuleFiles/stateRegistry.h"

#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

#ifdef BASILISK_TEST_MUJOCO_POLICIES
#include "architecture/utilities/bskLogging.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJQuaternionStatePolicy.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJScene.h"
#include <cstdint>
#include <memory>
#endif

namespace {
static_assert(std::is_same_v<decltype(std::declval<StateRegistry&>().segmentView({})), MutableMatrixView>);
static_assert(std::is_same_v<decltype(std::declval<const StateRegistry&>().segmentView({})), ConstMatrixView>);

class StateBufferViews : public testing::Test
{
  protected:
    void SetUp() override
    {
        StateSpec spec;
        spec.state = { 2, 2 };
        spec.derivative = spec.state;
        spec.diffusionTangent = spec.state;
        spec.noiseCount = 2;
        this->matrix = this->manager.registerState("matrix", spec);
        this->quiet = this->manager.registerState(1, 1, "quiet");
        this->tail = this->manager.registerState(2, 1, "tail");
        this->tail->setNumNoiseSources(1);
        this->manager.registerSharedNoiseSource({ { *this->matrix, 1 }, { *this->tail, 0 } });
        this->manager.finalizeStates();
    }

    DynParamManager manager;
    StateData* matrix = nullptr;
    StateData* quiet = nullptr;
    StateData* tail = nullptr;
};
}

TEST_F(StateBufferViews, StateAndDerivativeViewsAliasColumnMajorRecords)
{
    auto& registry = this->manager.getStateRegistry();
    for (const auto kind : { StateBufferKind::State, StateBufferKind::Derivative }) {
        const auto segment = registry.getSegment(kind, 0, 7);
        auto view = registry.segmentView(segment);
        ASSERT_EQ(view.rows(), 7);
        ASSERT_EQ(view.cols(), 1);
        view << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;

        const auto matrixValues =
          kind == StateBufferKind::State ? this->matrix->getState() : this->matrix->getStateDeriv();
        EXPECT_DOUBLE_EQ(matrixValues(1, 0), 2.0);
        EXPECT_DOUBLE_EQ(matrixValues(0, 1), 3.0);
        const auto tailValues = kind == StateBufferKind::State ? this->tail->getState() : this->tail->getStateDeriv();
        EXPECT_DOUBLE_EQ(tailValues(0, 0), 6.0);
        EXPECT_DOUBLE_EQ(tailValues(1, 0), 7.0);

        const auto tailSegment = registry.getSegment(kind, 2, 2);
        registry.segmentView(tailSegment).setConstant(9.0);
        EXPECT_DOUBLE_EQ(view(5, 0), 9.0);
        EXPECT_DOUBLE_EQ(view(6, 0), 9.0);

        const StateRegistry& constantRegistry = registry;
        const auto constantView = constantRegistry.segmentView(segment);
        EXPECT_EQ(constantView.data(), view.data());
        EXPECT_DOUBLE_EQ(constantView(6, 0), 9.0);
    }
}

TEST_F(StateBufferViews, DiffusionViewsPreserveStateAndLocalSourceOrder)
{
    auto& registry = this->manager.getStateRegistry();
    const auto segment = registry.getDiffusionSegment(0, 10);
    auto view = registry.segmentView(segment);
    view << 1.0, 2.0, 3.0, 4.0, 11.0, 12.0, 13.0, 14.0, 21.0, 22.0;

    EXPECT_DOUBLE_EQ(this->matrix->diffusionView(0)(1, 0), 2.0);
    EXPECT_DOUBLE_EQ(this->matrix->diffusionView(1)(0, 1), 13.0);
    EXPECT_DOUBLE_EQ(this->tail->diffusionView(0)(0, 0), 21.0);
    EXPECT_EQ(registry.diffusionSegmentData(segment), this->matrix->diffusionData(0));
    EXPECT_EQ(registry.segmentView(registry.getSegment(StateBufferKind::Diffusion, 0, 10)).data(), view.data());

    // The intervening noiseless record occupies no diffusion storage.
    const auto tailSegment = registry.getDiffusionSegment(1, 2);
    EXPECT_EQ(registry.diffusionSegmentData(tailSegment), this->tail->diffusionData(0));
    this->tail->diffusionView(0)(1, 0) = 23.0;
    const StateRegistry& constantRegistry = registry;
    EXPECT_DOUBLE_EQ(constantRegistry.segmentView(tailSegment)(1, 0), 23.0);
    EXPECT_EQ(constantRegistry.diffusionSegmentData(tailSegment), view.data() + 8);
}

TEST_F(StateBufferViews, TypedAccessorsRejectAnotherBufferKind)
{
    auto& registry = this->manager.getStateRegistry();
    const auto state = registry.getStateSegment(0, 4);
    const auto derivative = registry.getDerivativeSegment(0, 4);
    const auto diffusion = registry.getDiffusionSegment(0, 8);
    EXPECT_THROW(registry.stateSegmentData(derivative), std::logic_error);
    EXPECT_THROW(registry.stateSegmentData(diffusion), std::logic_error);
    EXPECT_THROW(registry.derivativeSegmentData(state), std::logic_error);
    EXPECT_THROW(registry.derivativeSegmentData(diffusion), std::logic_error);
    EXPECT_THROW(registry.diffusionSegmentData(state), std::logic_error);
    EXPECT_THROW(registry.diffusionSegmentData(derivative), std::logic_error);
}

TEST_F(StateBufferViews, RejectsInvalidRangesWithoutWritingStorage)
{
    auto& registry = this->manager.getStateRegistry();
    DynParamManager other;
    other.registerState(2, 2, "other");
    other.finalizeStates();

    for (const auto kind : { StateBufferKind::State, StateBufferKind::Derivative, StateBufferKind::Diffusion }) {
        const size_t count = kind == StateBufferKind::Diffusion ? 8 : 4;
        const auto segment = registry.getSegment(kind, 0, count);
        EXPECT_THROW(other.getStateRegistry().segmentView(segment), std::logic_error);
        EXPECT_THROW(registry.getSegment(kind, 0, 3), std::logic_error);
        EXPECT_THROW(registry.getSegment(kind, 3, 0), std::out_of_range);
        EXPECT_THROW(registry.getSegment(kind, 0, std::numeric_limits<size_t>::max()), std::logic_error);
        auto invalid = segment;
        invalid.count = std::numeric_limits<size_t>::max();
        EXPECT_THROW(registry.segmentView(invalid), std::logic_error);
        invalid = segment;
        invalid.offset = std::numeric_limits<size_t>::max();
        EXPECT_THROW(registry.segmentView(invalid), std::logic_error);
        EXPECT_TRUE(registry.segmentView(segment).isZero());
    }
    EXPECT_THROW(registry.getDiffusionSegment(0, 4), std::logic_error);
    EXPECT_THROW(registry.getSegment(static_cast<StateBufferKind>(99), 0, 0), std::invalid_argument);
    auto invalid = registry.getStateSegment(0, 4);
    invalid.bufferKind = static_cast<StateBufferKind>(99);
    EXPECT_THROW(registry.segmentView(invalid), std::logic_error);
}

TEST(StateBufferAccess, RequiresFinalizationAndAcceptsEmptyDiffusion)
{
    DynParamManager manager;
    manager.registerState(2, 1, "quiet");
    auto& registry = manager.getStateRegistry();
    for (const auto kind : { StateBufferKind::State, StateBufferKind::Derivative, StateBufferKind::Diffusion }) {
        EXPECT_THROW(registry.getSegment(kind, 0, 0), std::logic_error);
        EXPECT_THROW(registry.segmentView((StateBufferSegment{ &registry, kind, 0, 0 })), std::logic_error);
    }
    manager.finalizeStates();
    const auto segment = registry.getDiffusionSegment(0, 0);
    EXPECT_EQ(registry.segmentView(segment).size(), 0);
    const StateRegistry& constantRegistry = registry;
    EXPECT_EQ(constantRegistry.segmentView(segment).size(), 0);
    EXPECT_THROW(registry.getDiffusionSegment(0, 1), std::logic_error);
}

TEST_F(StateBufferViews, BorrowedViewsSurviveRepeatedFinalization)
{
    auto& registry = this->manager.getStateRegistry();
    auto view = registry.segmentView(registry.getDiffusionSegment(0, 10));
    double* const originalData = view.data();
    this->manager.registerState(2, 2, "matrix");
    this->manager.finalizeStates();
    this->matrix->diffusionView(0).setConstant(3.0);
    EXPECT_DOUBLE_EQ(view(0, 0), 3.0);
    EXPECT_EQ(registry.segmentView(registry.getDiffusionSegment(0, 10)).data(), originalData);
}

#ifdef BASILISK_TEST_MUJOCO_POLICIES
namespace {
class MujocoRegistrationProbe : public MJScene
{
  public:
    using MJScene::MJScene;
    using MJScene::registerMujocoStates;
};
}

TEST(StateBufferAccess, MujocoRejectsOversizedBulkDimensionsBeforeRegistration)
{
    // An unchecked uint32_t cast would wrap this count to one and accept it.
    const mjtSize oversized = static_cast<mjtSize>(std::numeric_limits<uint32_t>::max()) + 2;
    for (auto dimension : { &mjModel::nbody, &mjModel::na }) {
        MujocoRegistrationProbe scene("<mujoco/>");
        mjModel model = *scene.getMujocoModel();
        model.*dimension = oversized;
        EXPECT_THROW(scene.registerMujocoStates(&model, false), BasiliskError);
        EXPECT_EQ(scene.dynManager.getStateRegistry().getStateCount(), 0U);
        EXPECT_NO_THROW(scene.Reset(0));
    }
}

TEST(StateBufferAccess, QuaternionPoliciesKeepIndependentBufferDimensions)
{
    DynParamManager manager;
    StateSpec native;
    native.state = { 4, 1 };
    native.derivative = { 3, 1 };
    native.diffusionTangent = { 3, 1 };
    native.noiseCount = 2;
    native.updateKind = StateUpdateKind::Special;
    manager.registerState("native", native, std::make_unique<MJNativeQuaternionStatePolicy>());
    StateSpec highOrder = native;
    highOrder.derivative = { 4, 1 };
    highOrder.noiseCount = 1;
    auto* attitude =
      manager.registerState("highOrder", highOrder, std::make_unique<MJHighOrderQuaternionStatePolicy>());
    manager.finalizeStates();
    auto& registry = manager.getStateRegistry();
    EXPECT_EQ(registry.segmentView(registry.getStateSegment(0, 8)).size(), 8);
    EXPECT_EQ(registry.segmentView(registry.getDerivativeSegment(0, 7)).size(), 7);
    EXPECT_EQ(registry.segmentView(registry.getDiffusionSegment(0, 9)).size(), 9);
    registry.segmentView(registry.getDerivativeSegment(1, 4)).setConstant(2.0);
    EXPECT_EQ(attitude->getStateDeriv().rows(), 4);
    EXPECT_DOUBLE_EQ(attitude->getStateDeriv()(3, 0), 2.0);
    registry.segmentView(registry.getDiffusionSegment(1, 3)).setConstant(3.0);
    EXPECT_EQ(attitude->getStateDiffusion(0).rows(), 3);
    EXPECT_DOUBLE_EQ(attitude->getStateDiffusion(0)(2, 0), 3.0);
}
#endif
