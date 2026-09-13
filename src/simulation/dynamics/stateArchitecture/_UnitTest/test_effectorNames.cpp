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

#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/HingedRigidBodies/hingedRigidBodyStateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

// Naming support must not enable copying or moving live SysModel modules.
static_assert(!std::is_copy_constructible_v<HingedRigidBodyStateEffector>);
static_assert(!std::is_move_constructible_v<HingedRigidBodyStateEffector>);
static_assert(!std::is_copy_assignable_v<HingedRigidBodyStateEffector>);
static_assert(!std::is_move_assignable_v<HingedRigidBodyStateEffector>);

EffectorNameGroup
panelNames()
{
    return { "hingedRigidBody",
             { { "theta", EffectorNameKind::State, "hingedRigidBodyTheta", "", std::nullopt },
               { "thetaDot", EffectorNameKind::State, "hingedRigidBodyThetaDot", "", std::nullopt },
               { "position", EffectorNameKind::Property, "hingedRigidBodyInertialPosition", "", std::nullopt } } };
}

class LegacyEffector : public StateEffector
{
  public:
    void registerStates(DynParamManager&) override {}
    void linkInStates(DynParamManager&) override {}
    void computeDerivatives(double, Eigen::Vector3d, Eigen::Vector3d, Eigen::MRPd) override {}
};

class NamingEffector : public LegacyEffector
{
  public:
    EffectorNameGroup names = panelNames();
    std::vector<StateEffector*> children;
    StateData* registeredState = nullptr;
    Eigen::MatrixXd* registeredProperty = nullptr;
    using StateEffector::getEffectorNameRequest;
    using StateEffector::getResolvedEffectorName;

    void registerStates(DynParamManager& manager) override
    {
        this->registeredState = manager.registerEffectorState(1, 1, this->getEffectorNameRequest(), "theta");
        this->registeredProperty =
          manager.createEffectorProperty(this->getEffectorNameRequest(), "position", Eigen::Vector3d::Zero());
    }

  protected:
    EffectorNameGroup describeEffectorNames() const override { return this->names; }
    std::vector<StateEffector*> getNestedStateEffectors() const override { return this->children; }
};

class EffectorNames : public testing::Test
{
  protected:
    DynParamManager manager;

    void SetUp() override { this->manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal); }
};

struct NamingModel
{
    DynParamManager manager;
    NamingEffector first;
    NamingEffector second;
};

class EffectorNameModelCopies : public testing::TestWithParam<std::tuple<bool, bool, bool>>
{};

class EffectorNameModelMoves : public testing::TestWithParam<std::tuple<bool, bool, bool>>
{};

} // namespace

TEST(EffectorNamingPolicy, legacyRemainsTheDefault)
{
    DynParamManager manager;
    LegacyEffector effector;
    EXPECT_EQ(manager.getEffectorNamingPolicy(), EffectorNamingPolicy::Legacy);
    EXPECT_NO_THROW(effector.collectEffectorNames(manager));
    EXPECT_NO_THROW(manager.resolveEffectorNames());
    EXPECT_THROW(manager.requestEffectorNames(panelNames()), BasiliskError);
    // Legacy duplicate-registration behavior is deliberately unchanged.
    auto* state = manager.registerState(1, 1, "customTheta");
    EXPECT_EQ(state, manager.registerState(1, 1, "customTheta"));
}

TEST_F(EffectorNames, rejectsUnsupportedEffectors)
{
    LegacyEffector effector;
    EXPECT_THROW(effector.collectEffectorNames(this->manager), BasiliskError);
}

TEST_F(EffectorNames, assignsWholeGroupsInRequestOrder)
{
    const auto first = this->manager.requestEffectorNames(panelNames());
    const auto second = this->manager.requestEffectorNames(panelNames());
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(first, "theta"), "hingedRigidBodyTheta1");
    EXPECT_EQ(this->manager.getEffectorName(first, "thetaDot"), "hingedRigidBodyThetaDot1");
    EXPECT_EQ(this->manager.getEffectorName(first, "position"), "hingedRigidBodyInertialPosition1");
    EXPECT_EQ(this->manager.getEffectorName(second, "theta"), "hingedRigidBodyTheta2");
    EXPECT_EQ(this->manager.getEffectorName(second, "position"), "hingedRigidBodyInertialPosition2");
    EXPECT_TRUE(this->manager.stateContainer.stateMap.empty());
    EXPECT_TRUE(this->manager.dynProperties.empty());
}

TEST_F(EffectorNames, independentManagersIgnoreOverlappingLifetimes)
{
    const auto first = this->manager.requestEffectorNames(panelNames());
    this->manager.resolveEffectorNames();
    for (int iteration = 0; iteration < 3; ++iteration) {
        DynParamManager other;
        other.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
        const auto request = other.requestEffectorNames(panelNames());
        other.resolveEffectorNames();
        EXPECT_EQ(other.getEffectorName(request, "theta"), this->manager.getEffectorName(first, "theta"));
    }
}

TEST_F(EffectorNames, constructionOrderDoesNotDetermineNames)
{
    NamingEffector constructedFirst;
    NamingEffector constructedSecond;
    constructedSecond.collectEffectorNames(this->manager);
    constructedFirst.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(constructedSecond.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta1");
    EXPECT_EQ(constructedFirst.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta2");
}

TEST_F(EffectorNames, laterCustomNamesHavePriorityOverAutomaticGroups)
{
    const auto automatic = this->manager.requestEffectorNames(panelNames());
    auto customGroup = panelNames();
    // This is explicit even though it looks exactly like an automatic name.
    customGroup.names[0].customName = "hingedRigidBodyTheta1";
    customGroup.names[1].customName = "customRate";
    customGroup.names[2].customName = "customPosition";
    const auto custom = this->manager.requestEffectorNames(customGroup);
    const auto next = this->manager.requestEffectorNames(panelNames());
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(automatic, "theta"), "hingedRigidBodyTheta2");
    EXPECT_EQ(this->manager.getEffectorName(automatic, "position"), "hingedRigidBodyInertialPosition2");
    EXPECT_EQ(this->manager.getEffectorName(custom, "theta"), "hingedRigidBodyTheta1");
    EXPECT_EQ(this->manager.getEffectorName(custom, "thetaDot"), "customRate");
    EXPECT_EQ(this->manager.getEffectorName(next, "theta"), "hingedRigidBodyTheta3");
}

TEST_F(EffectorNames, partialOverridesPreserveGroupedAutomaticNames)
{
    auto group = panelNames();
    group.names[0].customName = "leftPanelAngle";
    const auto request = this->manager.requestEffectorNames(group);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(request, "theta"), "leftPanelAngle");
    EXPECT_EQ(this->manager.getEffectorName(request, "thetaDot"), "hingedRigidBodyThetaDot1");
    EXPECT_EQ(this->manager.getEffectorName(request, "position"), "hingedRigidBodyInertialPosition1");
}

TEST_F(EffectorNames, propertiesAndOtherFamiliesCanBlockAnIndex)
{
    this->manager.createProperty("hingedRigidBodyInertialPosition1", Eigen::Vector3d::Zero());
    this->manager.registerState(1, 1, "hingedRigidBodyTheta2");
    const auto first = this->manager.requestEffectorNames(panelNames());
    auto otherFamily = panelNames();
    otherFamily.family = "externalPanel";
    const auto other = this->manager.requestEffectorNames(otherFamily);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(first, "theta"), "hingedRigidBodyTheta3");
    EXPECT_EQ(this->manager.getEffectorName(other, "thetaDot"), "hingedRigidBodyThetaDot4");
}

TEST_F(EffectorNames, independentFamiliesStartAtOne)
{
    const auto panel = this->manager.requestEffectorNames(panelNames());
    const auto spring = this->manager.requestEffectorNames(
      { "spring", { { "rho", EffectorNameKind::State, "spring", "Rho", std::nullopt } } });
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(panel, "theta"), "hingedRigidBodyTheta1");
    EXPECT_EQ(this->manager.getEffectorName(spring, "rho"), "spring1Rho");
}

TEST_F(EffectorNames, ownerPrefixesAndBodySuffixesRemainPartOfThePattern)
{
    const auto request = this->manager.requestEffectorNames(
      { "chain",
        { { "angle", EffectorNameKind::State, "spacecraftchain", "Theta", std::nullopt },
          { "body1", EffectorNameKind::Property, "chainPosition", "_1", std::nullopt },
          { "body2", EffectorNameKind::Property, "chainPosition", "_2", std::nullopt } } });
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(request, "angle"), "spacecraftchain1Theta");
    EXPECT_EQ(this->manager.getEffectorName(request, "body1"), "chainPosition1_1");
    EXPECT_EQ(this->manager.getEffectorName(request, "body2"), "chainPosition1_2");
}

TEST_F(EffectorNames, repeatedPreparationPreservesNamesAndStateData)
{
    NamingEffector effector;
    effector.collectEffectorNames(this->manager);
    effector.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    const auto name = effector.getResolvedEffectorName(this->manager, "theta");
    auto* state = this->manager.registerEffectorState(1, 1, effector.getEffectorNameRequest(), "theta");
    const Eigen::MatrixXd angle = Eigen::MatrixXd::Constant(1, 1, 0.25); // [rad]
    state->setState(angle);
    effector.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(effector.getResolvedEffectorName(this->manager, "theta"), name);
    EXPECT_EQ(this->manager.getStateObject(name), state);
    EXPECT_EQ(state->getState()(0, 0), 0.25); // [rad]
    EXPECT_THROW(this->manager.requestEffectorNames(panelNames()), BasiliskError);
}

TEST_F(EffectorNames, destroyingAnEffectorDoesNotReleaseItsReservation)
{
    {
        NamingEffector temporary;
        temporary.collectEffectorNames(this->manager);
    }
    NamingEffector survivor;
    survivor.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(survivor.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta2");
}

TEST_F(EffectorNames, copiedEffectorsObtainTheirOwnRequests)
{
    NamingEffector original;
    original.collectEffectorNames(this->manager);
    NamingEffector copy = original;
    EXPECT_THROW(copy.getResolvedEffectorName(this->manager, "theta"), BasiliskError);
    copy.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(original.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta1");
    EXPECT_EQ(copy.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta2");
}

TEST_F(EffectorNames, changedSpecificationsAreRejectedAfterResolution)
{
    NamingEffector effector;
    effector.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    effector.names.names[0].customName = "newName";
    EXPECT_THROW(effector.collectEffectorNames(this->manager), BasiliskError);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(effector.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta1");
}

TEST_F(EffectorNames, duplicateCustomNamesFailWithoutPublishingPartialResults)
{
    const auto automatic = this->manager.requestEffectorNames(panelNames());
    const EffectorNameGroup custom = { "custom", { { "angle", EffectorNameKind::State, "", "", "leftAngle" } } };
    this->manager.requestEffectorNames(custom);
    this->manager.requestEffectorNames(custom);
    EXPECT_THROW(this->manager.resolveEffectorNames(), BasiliskError);
    EXPECT_THROW(this->manager.getEffectorName(automatic, "theta"), BasiliskError);
    EXPECT_TRUE(this->manager.stateContainer.stateMap.empty());
}

TEST_F(EffectorNames, customNamesCannotOverwriteExistingData)
{
    auto* state = this->manager.registerState(1, 1, "leftAngle");
    const auto request =
      this->manager.requestEffectorNames({ "custom", { { "angle", EffectorNameKind::State, "", "", "leftAngle" } } });
    EXPECT_THROW(this->manager.resolveEffectorNames(), BasiliskError);
    EXPECT_EQ(this->manager.getStateObject("leftAngle"), state);
    EXPECT_THROW(this->manager.getEffectorName(request, "angle"), BasiliskError);
}

TEST_F(EffectorNames, statesAndPropertiesKeepTheirSeparateNamespaces)
{
    const auto request =
      this->manager.requestEffectorNames({ "custom",
                                           { { "state", EffectorNameKind::State, "", "", "sameName" },
                                             { "property", EffectorNameKind::Property, "", "", "sameName" } } });
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(request, "state"), "sameName");
    EXPECT_EQ(this->manager.getEffectorName(request, "property"), "sameName");
}

TEST_F(EffectorNames, validatesPatternsKeysAndExplicitEmptyNames)
{
    auto group = panelNames();
    group.names[0].customName = "";
    EXPECT_THROW(this->manager.requestEffectorNames(group), BasiliskError);
    group = panelNames();
    group.names[1].key = group.names[0].key;
    EXPECT_THROW(this->manager.requestEffectorNames(group), BasiliskError);
    group = panelNames();
    group.names[1].prefix = group.names[0].prefix;
    EXPECT_THROW(this->manager.requestEffectorNames(group), BasiliskError);
    group = panelNames();
    group.names[0].kind = static_cast<EffectorNameKind>(99);
    EXPECT_THROW(this->manager.requestEffectorNames(group), BasiliskError);
    group = panelNames();
    group.family.clear();
    EXPECT_THROW(this->manager.requestEffectorNames(group), BasiliskError);
    const auto request = this->manager.requestEffectorNames(panelNames());
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(request, "theta"), "hingedRigidBodyTheta1");
}

TEST_F(EffectorNames, guardsPolicyChangesAndForeignRequests)
{
    const auto request = this->manager.requestEffectorNames(panelNames());
    EXPECT_THROW(this->manager.setEffectorNamingPolicy(EffectorNamingPolicy::Legacy), BasiliskError);
    EXPECT_NO_THROW(this->manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal));
    EXPECT_THROW(this->manager.setEffectorNamingPolicy(static_cast<EffectorNamingPolicy>(99)), BasiliskError);
    DynParamManager other;
    other.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    EXPECT_THROW(other.requestEffectorNames(panelNames(), request), BasiliskError);
    EXPECT_THROW(other.getEffectorName(request, "theta"), BasiliskError);
    EXPECT_THROW(this->manager.getEffectorName(request, "theta"), BasiliskError);
    this->manager.resolveEffectorNames();
    EXPECT_THROW(this->manager.getEffectorName(request, "missingKey"), BasiliskError);
    EXPECT_THROW(this->manager.getEffectorName({}, "theta"), BasiliskError);
}

TEST_F(EffectorNames, managerCopiesDoNotShareMutableResolutionResults)
{
    const auto request = this->manager.requestEffectorNames(panelNames());
    DynParamManager copy = this->manager;
    copy.registerState(1, 1, "hingedRigidBodyTheta1");
    copy.resolveEffectorNames();
    EXPECT_THROW(this->manager.getEffectorName(request, "theta"), BasiliskError);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(request, "theta"), "hingedRigidBodyTheta1");
    EXPECT_EQ(copy.getEffectorName(request, "theta"), "hingedRigidBodyTheta2");
}

TEST(EffectorNamingPolicy, selectionMustPrecedeStateOrPropertyRegistration)
{
    DynParamManager withState;
    withState.registerState(1, 1, "existingState");
    EXPECT_THROW(withState.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal), BasiliskError);
    DynParamManager withProperty;
    withProperty.createProperty("existingProperty", Eigen::Vector3d::Zero());
    EXPECT_THROW(withProperty.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal), BasiliskError);
}

TEST_F(EffectorNames, reservedStatesRequireOwnerRegistration)
{
    const auto first = this->manager.requestEffectorNames(panelNames());
    const auto second = this->manager.requestEffectorNames(panelNames());
    this->manager.resolveEffectorNames();
    const auto name = this->manager.getEffectorName(first, "theta");
    EXPECT_THROW(this->manager.registerState(1, 1, name), BasiliskError);
    EXPECT_TRUE(this->manager.stateContainer.stateMap.empty());

    auto* state = this->manager.registerEffectorState(1, 1, first, "theta");
    const Eigen::MatrixXd angle = Eigen::MatrixXd::Constant(1, 1, 0.25); // [rad]
    state->setState(angle);
    EXPECT_THROW(this->manager.registerState(1, 1, name), BasiliskError);
    EXPECT_EQ(this->manager.registerEffectorState(1, 1, first, "theta"), state);
    EXPECT_NE(this->manager.registerEffectorState(1, 1, second, "theta"), state);
    EXPECT_THROW(this->manager.registerEffectorState(2, 1, first, "theta"), BasiliskError);

    class OtherState : public StateData
    {
      public:
        using StateData::StateData;
    };
    EXPECT_THROW(this->manager.registerEffectorState<OtherState>(1, 1, first, "theta"), BasiliskError);
    EXPECT_EQ(state->getState()(0, 0), 0.25); // [rad]
}

TEST_F(EffectorNames, reservedPropertiesCannotBeClaimedOrOverwritten)
{
    const auto request = this->manager.requestEffectorNames(panelNames());
    this->manager.resolveEffectorNames();
    const auto name = this->manager.getEffectorName(request, "position");
    const Eigen::Vector3d competingValue = Eigen::Vector3d::Constant(2.0); // [m]
    EXPECT_THROW(this->manager.createProperty(name, competingValue), BasiliskError);
    EXPECT_TRUE(this->manager.dynProperties.empty());

    auto* property = this->manager.createEffectorProperty(request, "position", Eigen::Vector3d::Zero());
    EXPECT_THROW(this->manager.createProperty(name, competingValue), BasiliskError);
    EXPECT_EQ(this->manager.createEffectorProperty(request, "position", competingValue), property);
    EXPECT_TRUE(property->isZero());
    EXPECT_THROW(this->manager.createEffectorProperty(request, "position", Eigen::Vector2d::Zero()), BasiliskError);
    EXPECT_EQ(property->rows(), 3);
    EXPECT_TRUE(property->isZero());

    // Registration preserves existing data; explicit property updates remain supported.
    this->manager.setPropertyValue(name, competingValue);
    EXPECT_EQ(this->manager.createEffectorProperty(request, "position", Eigen::Vector3d::Zero()), property);
    EXPECT_EQ((*property)(0, 0), 2.0); // [m]
}

TEST_F(EffectorNames, ownedRegistrationChecksNamespaceEvenWhenNamesMatch)
{
    const auto request =
      this->manager.requestEffectorNames({ "custom",
                                           { { "state", EffectorNameKind::State, "", "", "sameName" },
                                             { "property", EffectorNameKind::Property, "", "", "sameName" } } });
    this->manager.resolveEffectorNames();
    EXPECT_THROW(this->manager.registerEffectorState(1, 1, request, "property"), BasiliskError);
    EXPECT_THROW(this->manager.createEffectorProperty(request, "state", Eigen::Vector3d::Zero()), BasiliskError);
    EXPECT_TRUE(this->manager.stateContainer.stateMap.empty());
    EXPECT_TRUE(this->manager.dynProperties.empty());
    EXPECT_NO_THROW(this->manager.registerEffectorState(1, 1, request, "state"));
    EXPECT_NO_THROW(this->manager.createEffectorProperty(request, "property", Eigen::Vector3d::Zero()));
}

TEST_F(EffectorNames, ownedRegistrationRejectsUnresolvedForeignAndUnknownRequests)
{
    const auto request = this->manager.requestEffectorNames(panelNames());
    EXPECT_THROW(this->manager.registerEffectorState(1, 1, request, "theta"), BasiliskError);
    EXPECT_THROW(this->manager.createEffectorProperty(request, "position", Eigen::Vector3d::Zero()), BasiliskError);
    this->manager.resolveEffectorNames();
    EXPECT_THROW(this->manager.registerEffectorState(1, 1, request, "missingKey"), BasiliskError);
    EXPECT_THROW(this->manager.createEffectorProperty(request, "missingKey", Eigen::Vector3d::Zero()), BasiliskError);
    EXPECT_THROW(this->manager.registerEffectorState(1, 1, {}, "theta"), BasiliskError);
    EXPECT_THROW(this->manager.createEffectorProperty({}, "position", Eigen::Vector3d::Zero()), BasiliskError);
    DynParamManager other;
    other.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    other.resolveEffectorNames();
    EXPECT_THROW(other.registerEffectorState(1, 1, request, "theta"), BasiliskError);
    EXPECT_THROW(other.createEffectorProperty(request, "position", Eigen::Vector3d::Zero()), BasiliskError);
    EXPECT_TRUE(other.stateContainer.stateMap.empty());
    EXPECT_TRUE(other.dynProperties.empty());
}

TEST_F(EffectorNames, duplicateNameFailureCanBeCorrectedOnTheSameEffectors)
{
    NamingEffector first;
    NamingEffector second;
    first.names.names[0].customName = "duplicate";
    second.names.names[0].customName = "duplicate";
    first.collectEffectorNames(this->manager);
    second.collectEffectorNames(this->manager);
    const auto oldRequest = second.getEffectorNameRequest();
    EXPECT_THROW(this->manager.resolveEffectorNames(), BasiliskError);

    second.names.names[0].customName = "corrected";
    second.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(first.getResolvedEffectorName(this->manager, "theta"), "duplicate");
    EXPECT_EQ(second.getResolvedEffectorName(this->manager, "theta"), "corrected");
    EXPECT_EQ(first.getResolvedEffectorName(this->manager, "thetaDot"), "hingedRigidBodyThetaDot1");
    EXPECT_EQ(second.getResolvedEffectorName(this->manager, "thetaDot"), "hingedRigidBodyThetaDot2");
    EXPECT_NE(this->manager.registerEffectorState(1, 1, first.getEffectorNameRequest(), "theta"),
              this->manager.registerEffectorState(1, 1, second.getEffectorNameRequest(), "theta"));
    EXPECT_THROW(this->manager.registerEffectorState(1, 1, oldRequest, "theta"), BasiliskError);
    EXPECT_THROW(this->manager.createEffectorProperty(oldRequest, "position", Eigen::Vector3d::Zero()), BasiliskError);
}

TEST_F(EffectorNames, pendingCorrectionsPreserveCollectionOrder)
{
    auto group = panelNames();
    const auto original = this->manager.requestEffectorNames(group);
    const auto second = this->manager.requestEffectorNames(group);
    group.names[0].customName = "leftAngle";
    const auto replacement = this->manager.requestEffectorNames(group, original);
    EXPECT_NE(original, replacement);
    EXPECT_EQ(this->manager.requestEffectorNames(group, replacement), replacement);
    EXPECT_THROW(this->manager.requestEffectorNames(group, original), BasiliskError);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(replacement, "theta"), "leftAngle");
    EXPECT_EQ(this->manager.getEffectorName(replacement, "position"), "hingedRigidBodyInertialPosition1");
    EXPECT_EQ(this->manager.getEffectorName(second, "position"), "hingedRigidBodyInertialPosition2");
}

TEST_F(EffectorNames, invalidCorrectionsLeaveTheOriginalRequestIntact)
{
    auto group = panelNames();
    const auto original = this->manager.requestEffectorNames(group);
    group.names[0].customName = "";
    EXPECT_THROW(this->manager.requestEffectorNames(group, original), BasiliskError);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(original, "theta"), "hingedRigidBodyTheta1");
}

TEST_F(EffectorNames, pendingRequestsCanBeCancelledAndMovedToAnotherManager)
{
    NamingEffector first;
    NamingEffector second;
    first.names.names[0].customName = "duplicate";
    second.names.names[0].customName = "duplicate";
    first.collectEffectorNames(this->manager);
    second.collectEffectorNames(this->manager);
    const auto cancelled = second.getEffectorNameRequest();
    EXPECT_THROW(this->manager.resolveEffectorNames(), BasiliskError);
    second.cancelEffectorNames(this->manager);
    EXPECT_THROW(second.getEffectorNameRequest(), BasiliskError);
    EXPECT_THROW(this->manager.cancelEffectorNames(cancelled), BasiliskError);
    this->manager.resolveEffectorNames();

    DynParamManager replacement;
    replacement.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    second.collectEffectorNames(replacement);
    replacement.resolveEffectorNames();
    EXPECT_EQ(first.getResolvedEffectorName(this->manager, "thetaDot"), "hingedRigidBodyThetaDot1");
    EXPECT_EQ(second.getResolvedEffectorName(replacement, "thetaDot"), "hingedRigidBodyThetaDot1");
    EXPECT_NO_THROW(replacement.registerEffectorState(1, 1, second.getEffectorNameRequest(), "theta"));
    EXPECT_THROW(this->manager.registerEffectorState(1, 1, cancelled, "theta"), BasiliskError);
}

TEST_F(EffectorNames, failedCancellationDoesNotDetachTheEffector)
{
    NamingEffector effector;
    EXPECT_NO_THROW(effector.cancelEffectorNames(this->manager));
    effector.collectEffectorNames(this->manager);
    const auto request = effector.getEffectorNameRequest();
    DynParamManager other;
    other.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    EXPECT_THROW(effector.cancelEffectorNames(other), BasiliskError);
    EXPECT_EQ(effector.getEffectorNameRequest(), request);
    this->manager.resolveEffectorNames();
    EXPECT_THROW(effector.cancelEffectorNames(this->manager), BasiliskError);
    EXPECT_EQ(effector.getEffectorNameRequest(), request);
    EXPECT_EQ(effector.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta1");
}

TEST_F(EffectorNames, uncollectedParentCopyCannotCancelSourceChildren)
{
    NamingEffector parent;
    NamingEffector child;
    parent.children = {&child};
    parent.collectEffectorNames(this->manager);
    const auto parentRequest = parent.getEffectorNameRequest();
    const auto childRequest = child.getEffectorNameRequest();
    NamingEffector copy = parent;
    EXPECT_NO_THROW(copy.cancelEffectorNames(this->manager));
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(parentRequest, "theta"), "hingedRigidBodyTheta1");
    EXPECT_EQ(this->manager.getEffectorName(childRequest, "theta"), "hingedRigidBodyTheta2");
}

TEST_F(EffectorNames, staleChildRequestPreventsPartialTreeCancellation)
{
    NamingEffector parent;
    NamingEffector staleChild;
    NamingEffector validChild;
    parent.children = {&staleChild, &validChild};
    parent.collectEffectorNames(this->manager);
    const auto parentRequest = parent.getEffectorNameRequest();
    const auto validRequest = validChild.getEffectorNameRequest();
    this->manager.cancelEffectorNames(staleChild.getEffectorNameRequest());
    EXPECT_THROW(parent.cancelEffectorNames(this->manager), BasiliskError);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(parentRequest, "theta"), "hingedRigidBodyTheta1");
    EXPECT_EQ(this->manager.getEffectorName(validRequest, "theta"), "hingedRigidBodyTheta2");
}

TEST_F(EffectorNames, pendingEditsDoNotMutateManagerCopies)
{
    auto group = panelNames();
    const auto original = this->manager.requestEffectorNames(group);
    DynParamManager copy = this->manager;
    group.names[0].customName = "leftAngle";
    const auto replacement = this->manager.requestEffectorNames(group, original);
    this->manager.resolveEffectorNames();
    copy.resolveEffectorNames();
    EXPECT_EQ(this->manager.getEffectorName(replacement, "theta"), "leftAngle");
    EXPECT_EQ(copy.getEffectorName(original, "theta"), "hingedRigidBodyTheta1");
    EXPECT_THROW(copy.getEffectorName(replacement, "theta"), BasiliskError);
}

TEST_F(EffectorNames, resolvedReservationsSurviveManagerCopies)
{
    const auto request = this->manager.requestEffectorNames(panelNames());
    this->manager.resolveEffectorNames();
    auto* originalState = this->manager.registerEffectorState(1, 1, request, "theta");
    auto* originalProperty = this->manager.createEffectorProperty(request, "position", Eigen::Vector3d::Zero());
    DynParamManager copy = this->manager;
    EXPECT_THROW(copy.registerState(1, 1, "hingedRigidBodyTheta1"), BasiliskError);
    EXPECT_THROW(copy.createProperty("hingedRigidBodyInertialPosition1", Eigen::Vector3d::Zero()), BasiliskError);
    EXPECT_NE(copy.registerEffectorState(1, 1, request, "theta"), originalState);
    EXPECT_NE(copy.createEffectorProperty(request, "position", Eigen::Vector3d::Zero()), originalProperty);
}

TEST(EffectorNamingPolicy, legacyPropertiesRetainDuplicateRegistrationBehavior)
{
    DynParamManager manager;
    auto* property = manager.createProperty("position", Eigen::Vector3d::Zero());
    const Eigen::Vector3d updated = Eigen::Vector3d::Constant(2.0); // [m]
    EXPECT_EQ(manager.createProperty("position", updated), property);
    EXPECT_EQ((*property)(0, 0), 2.0); // [m]
}

TEST_P(EffectorNameModelCopies, preserveDeclarationsAndIndependentStorage)
{
    const auto [customName, resolvedBeforeCopy, useAssignment] = this->GetParam();
    NamingModel original;
    original.manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    if (customName) {
        original.first.names.names[0].customName = "leftAngle";
    }
    original.first.collectEffectorNames(original.manager);
    original.second.collectEffectorNames(original.manager);
    if (resolvedBeforeCopy) {
        original.manager.resolveEffectorNames();
        auto* state = original.manager.registerEffectorState(1, 1, original.first.getEffectorNameRequest(), "theta");
        state->setState(Eigen::MatrixXd::Constant(1, 1, 0.25)); // [rad]
        original.manager.createEffectorProperty(
          original.first.getEffectorNameRequest(), "position", Eigen::Vector3d::Zero());
    }

    auto copy = useAssignment ? std::make_unique<NamingModel>() : std::make_unique<NamingModel>(original);
    if (useAssignment) {
        *copy = original;
    }
    EXPECT_THROW(copy->first.getEffectorNameRequest(), BasiliskError);
    copy->first.collectEffectorNames(copy->manager);
    copy->second.collectEffectorNames(copy->manager);
    EXPECT_EQ(copy->first.getEffectorNameRequest(), original.first.getEffectorNameRequest());
    EXPECT_EQ(copy->second.getEffectorNameRequest(), original.second.getEffectorNameRequest());
    copy->manager.resolveEffectorNames();
    original.manager.resolveEffectorNames();
    EXPECT_EQ(copy->first.getResolvedEffectorName(copy->manager, "theta"),
              customName ? "leftAngle" : "hingedRigidBodyTheta1");
    EXPECT_EQ(copy->second.getResolvedEffectorName(copy->manager, "theta"), "hingedRigidBodyTheta2");

    auto* sourceState = original.manager.registerEffectorState(1, 1, original.first.getEffectorNameRequest(), "theta");
    auto* copiedState = copy->manager.registerEffectorState(1, 1, copy->first.getEffectorNameRequest(), "theta");
    EXPECT_NE(sourceState, copiedState);
    EXPECT_EQ(copiedState->getState()(0, 0), resolvedBeforeCopy ? 0.25 : 0.0); // [rad]
    sourceState->setState(Eigen::MatrixXd::Constant(1, 1, 0.75));              // [rad]
    EXPECT_EQ(copiedState->getState()(0, 0), resolvedBeforeCopy ? 0.25 : 0.0); // [rad]
    auto* sourceProperty = original.manager.createEffectorProperty(
      original.first.getEffectorNameRequest(), "position", Eigen::Vector3d::Zero());
    auto* copiedProperty =
      copy->manager.createEffectorProperty(copy->first.getEffectorNameRequest(), "position", Eigen::Vector3d::Zero());
    EXPECT_NE(sourceProperty, copiedProperty);
    sourceProperty->setConstant(2.0); // [m]
    EXPECT_TRUE(copiedProperty->isZero());
    EXPECT_NO_THROW(copy->first.collectEffectorNames(copy->manager));
    EXPECT_NO_THROW(copy->second.collectEffectorNames(copy->manager));
    EXPECT_NO_THROW(original.first.collectEffectorNames(original.manager));
}

INSTANTIATE_TEST_SUITE_P(EffectorNaming,
                         EffectorNameModelCopies,
                         testing::Combine(testing::Bool(), testing::Bool(), testing::Bool()));

TEST_F(EffectorNames, copiedReservationCanOnlyBeClaimedByOneEffector)
{
    NamingEffector original;
    original.names.names[0].customName = "leftAngle";
    original.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    DynParamManager copy = this->manager;
    NamingEffector firstCopy = original;
    NamingEffector secondCopy = original;
    firstCopy.collectEffectorNames(copy);
    EXPECT_THROW(secondCopy.collectEffectorNames(copy), BasiliskError);
    EXPECT_THROW(original.cancelEffectorNames(copy), BasiliskError);
    EXPECT_THROW(original.collectEffectorNames(copy), BasiliskError);
    EXPECT_EQ(firstCopy.getResolvedEffectorName(copy, "theta"), "leftAngle");
    EXPECT_EQ(original.getResolvedEffectorName(this->manager, "theta"), "leftAngle");
}

TEST_F(EffectorNames, copyingAnEffectorWithinTheSameManagerStillChecksCustomCollisions)
{
    NamingEffector original;
    original.names.names[0].customName = "leftAngle";
    original.collectEffectorNames(this->manager);
    NamingEffector copy = original;
    copy.collectEffectorNames(this->manager);
    EXPECT_NE(copy.getEffectorNameRequest(), original.getEffectorNameRequest());
    EXPECT_THROW(this->manager.resolveEffectorNames(), BasiliskError);
    copy.names.names[0].customName = "rightAngle";
    copy.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(original.getResolvedEffectorName(this->manager, "theta"), "leftAngle");
    EXPECT_EQ(copy.getResolvedEffectorName(this->manager, "theta"), "rightAngle");
}

TEST_F(EffectorNames, correctionsInCopiedModelsPreserveTheSourceAndOwnerBinding)
{
    NamingEffector original;
    original.collectEffectorNames(this->manager);
    DynParamManager copy = this->manager;
    NamingEffector copiedEffector = original;
    copiedEffector.names.names[0].customName = "copiedAngle";
    copiedEffector.collectEffectorNames(copy);
    const auto corrected = copiedEffector.getEffectorNameRequest();
    EXPECT_NE(corrected, original.getEffectorNameRequest());
    copiedEffector.collectEffectorNames(copy);
    EXPECT_EQ(corrected, copiedEffector.getEffectorNameRequest());
    copy.resolveEffectorNames();
    this->manager.resolveEffectorNames();
    EXPECT_EQ(copiedEffector.getResolvedEffectorName(copy, "theta"), "copiedAngle");
    EXPECT_EQ(copiedEffector.getResolvedEffectorName(copy, "thetaDot"), "hingedRigidBodyThetaDot1");
    EXPECT_EQ(original.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta1");
    NamingEffector secondCopy = copiedEffector;
    EXPECT_THROW(secondCopy.collectEffectorNames(copy), BasiliskError);
}

TEST_F(EffectorNames, copiedModelCancellationDoesNotAffectTheSource)
{
    NamingEffector original;
    original.collectEffectorNames(this->manager);
    DynParamManager copy = this->manager;
    NamingEffector copiedEffector = original;
    copiedEffector.collectEffectorNames(copy);
    copiedEffector.cancelEffectorNames(copy);
    EXPECT_THROW(copiedEffector.getEffectorNameRequest(), BasiliskError);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(original.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta1");
    copiedEffector.collectEffectorNames(copy);
    copy.resolveEffectorNames();
    EXPECT_EQ(copiedEffector.getResolvedEffectorName(copy, "theta"), "hingedRigidBodyTheta1");
}

TEST_F(EffectorNames, expiredManagerAllowsRecoveryEvenWhenItsAddressIsReused)
{
    NamingEffector effector;
    std::optional<DynParamManager> abandoned(std::in_place);
    abandoned->setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    effector.collectEffectorNames(*abandoned);
    const auto oldRequest = effector.getEffectorNameRequest();
    // Keeping a copy alive must not keep the original manager's lifetime alive.
    DynParamManager snapshot = *abandoned;
    abandoned.reset();
    EXPECT_THROW(effector.getEffectorNameRequest(), BasiliskError);
    abandoned.emplace();
    abandoned->setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    effector.collectEffectorNames(*abandoned);
    EXPECT_NE(effector.getEffectorNameRequest(), oldRequest);
    abandoned->resolveEffectorNames();
    snapshot.resolveEffectorNames();
    EXPECT_EQ(effector.getResolvedEffectorName(*abandoned, "theta"), "hingedRigidBodyTheta1");
    EXPECT_EQ(snapshot.getEffectorName(oldRequest, "theta"), "hingedRigidBodyTheta1");
}

TEST_F(EffectorNames, survivingEffectorCanClaimItsDeclarationInASnapshot)
{
    NamingEffector effector;
    {
        DynParamManager original;
        original.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
        effector.collectEffectorNames(original);
        original.resolveEffectorNames();
        this->manager = original;
    }
    EXPECT_THROW(effector.getEffectorNameRequest(), BasiliskError);
    effector.collectEffectorNames(this->manager);
    EXPECT_EQ(effector.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta1");
}

TEST_F(EffectorNames, reusedEffectorAddressCannotReclaimAnOldReservation)
{
    std::optional<NamingEffector> original(std::in_place);
    original->collectEffectorNames(this->manager);
    NamingEffector copy = *original;
    const auto oldRequest = original->getEffectorNameRequest();
    original.reset();
    original.emplace(copy);
    EXPECT_THROW(original->getEffectorNameRequest(), BasiliskError);
    original->collectEffectorNames(this->manager);
    EXPECT_NE(original->getEffectorNameRequest(), oldRequest);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(original->getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta2");
}

TEST_F(EffectorNames, movingAManagerPreservesItsBindings)
{
    NamingEffector effector;
    effector.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    DynParamManager moved = std::move(this->manager);
    EXPECT_NO_THROW(effector.collectEffectorNames(moved));
    EXPECT_EQ(effector.getResolvedEffectorName(moved, "theta"), "hingedRigidBodyTheta1");
}

TEST_P(EffectorNameModelMoves, preserveNamesOwnershipAndRegisteredPointers)
{
    const auto [customName, resolvedBeforeMove, useAssignment] = this->GetParam();
    auto original = std::make_unique<NamingModel>();
    original->manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    if (customName) {
        original->first.names.names[0].customName = "leftAngle";
    }
    original->first.collectEffectorNames(original->manager);
    original->second.collectEffectorNames(original->manager);
    const auto firstRequest = original->first.getEffectorNameRequest();
    const auto secondRequest = original->second.getEffectorNameRequest();
    if (resolvedBeforeMove) {
        original->manager.resolveEffectorNames();
        original->first.registerStates(original->manager);
        original->second.registerStates(original->manager);
        original->first.registeredState->setState(Eigen::MatrixXd::Constant(1, 1, 0.25)); // [rad]
        original->first.registeredProperty->setConstant(2.0);                             // [m]
    }
    const auto* originalState = original->first.registeredState;
    const auto* originalProperty = original->first.registeredProperty;

    auto moved = useAssignment ? std::make_unique<NamingModel>() : std::make_unique<NamingModel>(std::move(*original));
    if (useAssignment) {
        // Assignment must also work when replacing an already prepared destination.
        moved->manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
        moved->first.collectEffectorNames(moved->manager);
        moved->manager.resolveEffectorNames();
        moved->first.registerStates(moved->manager);
        *moved = std::move(*original);
    }
    EXPECT_THROW(original->first.getEffectorNameRequest(), BasiliskError);
    EXPECT_THROW(original->second.getEffectorNameRequest(), BasiliskError);
    original.reset();

    EXPECT_EQ(moved->first.getEffectorNameRequest(), firstRequest);
    EXPECT_EQ(moved->second.getEffectorNameRequest(), secondRequest);
    moved->first.collectEffectorNames(moved->manager);
    moved->second.collectEffectorNames(moved->manager);
    moved->manager.resolveEffectorNames();
    const auto& angleName = moved->first.getResolvedEffectorName(moved->manager, "theta");
    EXPECT_EQ(angleName, customName ? "leftAngle" : "hingedRigidBodyTheta1");
    EXPECT_EQ(moved->second.getResolvedEffectorName(moved->manager, "theta"), "hingedRigidBodyTheta2");
    if (resolvedBeforeMove) {
        auto* transferredState = moved->manager.getStateObject(angleName);
        auto* transferredProperty =
          moved->manager.getPropertyReference(moved->first.getResolvedEffectorName(moved->manager, "position"));
        ASSERT_EQ(transferredState, originalState);
        ASSERT_EQ(transferredProperty, originalProperty);
        ASSERT_EQ(moved->first.registeredState, transferredState);
        ASSERT_EQ(moved->first.registeredProperty, transferredProperty);
        EXPECT_EQ(transferredState->getState()(0, 0), 0.25); // [rad]
        EXPECT_EQ((*transferredProperty)(0, 0), 2.0);        // [m]
    }
    moved->first.registerStates(moved->manager);
    moved->second.registerStates(moved->manager);
    EXPECT_EQ(moved->manager.stateContainer.stateMap.size(), 2U);
    EXPECT_EQ(moved->manager.dynProperties.size(), 2U);
    EXPECT_THROW(moved->manager.registerState(1, 1, angleName), BasiliskError);

    // Moving must not let another effector claim the transferred reservation.
    NamingEffector duplicate = moved->first;
    EXPECT_THROW(duplicate.collectEffectorNames(moved->manager), BasiliskError);
}

INSTANTIATE_TEST_SUITE_P(EffectorNaming,
                         EffectorNameModelMoves,
                         testing::Combine(testing::Bool(), testing::Bool(), testing::Bool()));

TEST_F(EffectorNames, movingAnEffectorPreservesItsReservationInTheSameManager)
{
    auto original = std::make_unique<NamingEffector>();
    original->collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    original->registerStates(this->manager);
    const auto request = original->getEffectorNameRequest();
    const auto* state = original->registeredState;
    NamingEffector moved = std::move(*original);
    original.reset();
    EXPECT_EQ(moved.getEffectorNameRequest(), request);
    EXPECT_EQ(moved.registeredState, state);
    EXPECT_NO_THROW(moved.collectEffectorNames(this->manager));
    NamingEffector assigned;
    assigned = std::move(moved);
    EXPECT_THROW(moved.getEffectorNameRequest(), BasiliskError);
    EXPECT_EQ(assigned.getEffectorNameRequest(), request);
    EXPECT_EQ(assigned.registeredState, state);
    EXPECT_NO_THROW(assigned.collectEffectorNames(this->manager));
    EXPECT_EQ(assigned.getResolvedEffectorName(this->manager, "theta"), "hingedRigidBodyTheta1");
}

TEST_F(EffectorNames, transferredPendingRequestCanBeCancelledAndCorrected)
{
    NamingEffector original;
    original.collectEffectorNames(this->manager);
    const auto previous = original.getEffectorNameRequest();
    NamingEffector moved = std::move(original);
    moved.names.names[0].customName = "correctedAngle";
    moved.collectEffectorNames(this->manager);
    const auto corrected = moved.getEffectorNameRequest();
    EXPECT_NE(corrected, previous);
    EXPECT_NO_THROW(original.cancelEffectorNames(this->manager));
    EXPECT_NO_THROW(moved.cancelEffectorNames(this->manager));
    EXPECT_THROW(this->manager.requestEffectorNames(panelNames(), corrected), BasiliskError);
    moved.collectEffectorNames(this->manager);
    this->manager.resolveEffectorNames();
    EXPECT_EQ(moved.getResolvedEffectorName(this->manager, "theta"), "correctedAngle");
    EXPECT_EQ(moved.getResolvedEffectorName(this->manager, "thetaDot"), "hingedRigidBodyThetaDot1");
}
