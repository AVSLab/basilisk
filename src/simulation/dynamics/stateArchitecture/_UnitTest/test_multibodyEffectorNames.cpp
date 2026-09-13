/* Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 * This file is distributed under the ISC License in LICENSE.
 */

#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/NHingedRigidBodies/nHingedRigidBodyStateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/constraintEffector/constraintDynamicEffector.h"
#include "simulation/dynamics/dualHingedRigidBodies/dualHingedRigidBodyStateEffector.h"
#include "simulation/dynamics/linearTranslationalBodies/linearTranslationBodiesNDOF/linearTranslationNDOFStateEffector.h"
#include "simulation/dynamics/linearTranslationalBodies/linearTranslationBodiesOneDOF/linearTranslationOneDOFStateEffector.h"
#include "simulation/dynamics/prescribedMotion/prescribedMotionStateEffector.h"
#include "simulation/dynamics/spacecraft/spacecraft.h"
#include "simulation/dynamics/spinningBodies/spinningBodiesNDOF/spinningBodyNDOFStateEffector.h"
#include "simulation/dynamics/spinningBodies/spinningBodiesOneDOF/spinningBodyOneDOFStateEffector.h"
#include "simulation/dynamics/spinningBodies/spinningBodiesTwoDOF/spinningBodyTwoDOFStateEffector.h"
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace {

template<typename Effector>
void
configure(Effector& effector)
{
    if constexpr (std::is_same_v<Effector, SpinningBodyOneDOFStateEffector>) {
        effector.sHat_S = Eigen::Vector3d::UnitX();
    } else if constexpr (std::is_same_v<Effector, DualHingedRigidBodyStateEffector>) {
        effector.mass1 = 1.0; // [kg]
        effector.mass2 = 1.0; // [kg]
    } else if constexpr (std::is_same_v<Effector, SpinningBodyNDOFStateEffector>) {
        effector.addSpinningBody(std::make_shared<SpinningBody>());
        effector.addSpinningBody(std::make_shared<SpinningBody>());
    } else if constexpr (std::is_same_v<Effector, LinearTranslationNDOFStateEffector>) {
        for (int index = 0; index < 2; ++index) {
            auto body = std::make_shared<TranslatingBody>();
            body->setMass(1.0); // [kg]
            effector.addTranslatingBody(body);
        }
    } else if constexpr (std::is_same_v<Effector, NHingedRigidBodyStateEffector>) {
        HingedPanel panel;
        panel.IPntS_S.setIdentity(); // [kg m^2]
        effector.addHingedPanel(panel);
        effector.addHingedPanel(panel);
    }
}

template<typename Effector>
std::string
stateName(const Effector& effector)
{
    if constexpr (std::is_same_v<Effector, PrescribedMotionStateEffector>) {
        return effector.getNameOfsigma_PMState();
    } else if constexpr (std::is_same_v<Effector, LinearTranslationOneDOFStateEffector> ||
                         std::is_same_v<Effector, LinearTranslationNDOFStateEffector>) {
        return effector.getNameOfRhoState();
    } else if constexpr (std::is_same_v<Effector, DualHingedRigidBodyStateEffector> ||
                         std::is_same_v<Effector, SpinningBodyTwoDOFStateEffector>) {
        return effector.getNameOfTheta1State();
    } else {
        return effector.getNameOfThetaState();
    }
}

template<typename Effector>
void
setName(Effector& effector, const std::string& name)
{
    if constexpr (std::is_same_v<Effector, PrescribedMotionStateEffector>) {
        effector.setNameOfsigma_PMState(name);
    } else if constexpr (std::is_same_v<Effector, LinearTranslationOneDOFStateEffector> ||
                         std::is_same_v<Effector, LinearTranslationNDOFStateEffector>) {
        effector.setNameOfRhoState(name);
    } else if constexpr (std::is_same_v<Effector, DualHingedRigidBodyStateEffector> ||
                         std::is_same_v<Effector, SpinningBodyTwoDOFStateEffector>) {
        effector.setNameOfTheta1State(name);
    } else {
        effector.setNameOfThetaState(name);
    }
}

void
registerEffector(StateEffector& effector, DynParamManager& manager)
{
    effector.registerStates(manager);
}

template<typename Effector>
class MultibodyEffectorNames : public testing::Test
{};
using MultibodyEffectors = testing::Types<SpinningBodyOneDOFStateEffector,
                                          SpinningBodyTwoDOFStateEffector,
                                          SpinningBodyNDOFStateEffector,
                                          LinearTranslationOneDOFStateEffector,
                                          LinearTranslationNDOFStateEffector,
                                          DualHingedRigidBodyStateEffector,
                                          NHingedRigidBodyStateEffector,
                                          PrescribedMotionStateEffector>;
TYPED_TEST_SUITE(MultibodyEffectorNames, MultibodyEffectors, );

TYPED_TEST(MultibodyEffectorNames, resolvesEveryStateAndBodyPropertyInCollectionOrder)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam second;
    TypeParam first;
    configure(first);
    configure(second);
    first.collectEffectorNames(manager);
    second.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    registerEffector(first, manager);
    const auto statesPerEffector = manager.stateContainer.stateMap.size();
    const auto propertiesPerEffector = manager.dynProperties.size();
    ASSERT_GT(propertiesPerEffector, 0U);
    registerEffector(second, manager);
    EXPECT_EQ(manager.stateContainer.stateMap.size(), 2 * statesPerEffector);
    EXPECT_EQ(manager.dynProperties.size(), 2 * propertiesPerEffector);
    EXPECT_NE(stateName(first), stateName(second));
    EXPECT_NE(manager.getStateObject(stateName(first)), manager.getStateObject(stateName(second)));
}

TYPED_TEST(MultibodyEffectorNames, keepsLegacyNamesAndCustomAssignments)
{
    DynParamManager manager;
    TypeParam first;
    TypeParam custom;
    configure(first);
    configure(custom);
    const auto original = stateName(first);
    setName(custom, "customState");
    first.collectEffectorNames(manager);
    custom.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    registerEffector(first, manager);
    registerEffector(custom, manager);
    EXPECT_EQ(stateName(first), original);
    EXPECT_EQ(stateName(custom), "customState");
    EXPECT_NE(manager.getStateObject(original), nullptr);
    EXPECT_NE(manager.getStateObject("customState"), nullptr);
}

TYPED_TEST(MultibodyEffectorNames, treatsConstructorLookingAssignmentsAsExplicit)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam effector;
    configure(effector);
    const auto original = stateName(effector);
    setName(effector, original);
    effector.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    registerEffector(effector, manager);
    EXPECT_EQ(stateName(effector), original);
}

TYPED_TEST(MultibodyEffectorNames, rejectsRegistrationAndBindingBeforeResolution)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam effector;
    configure(effector);
    EXPECT_THROW(registerEffector(effector, manager), BasiliskError);
    EXPECT_THROW(effector.bindAttachedDynamicEffectors(manager), BasiliskError);
    EXPECT_TRUE(manager.stateContainer.stateMap.empty());
    EXPECT_TRUE(manager.dynProperties.empty());
}

TYPED_TEST(MultibodyEffectorNames, rejectsNameChangesAfterResolution)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam effector;
    configure(effector);
    effector.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    setName(effector, "changed");
    EXPECT_THROW(registerEffector(effector, manager), BasiliskError);
    EXPECT_TRUE(manager.stateContainer.stateMap.empty());
    EXPECT_TRUE(manager.dynProperties.empty());
}

TYPED_TEST(MultibodyEffectorNames, preservesStorageAndNamesAcrossRepeatedRegistration)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam effector;
    configure(effector);
    effector.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    registerEffector(effector, manager);
    const auto name = stateName(effector);
    const auto* state = manager.getStateObject(name);
    std::vector<const Eigen::MatrixXd*> properties;
    for (const auto& entry : manager.dynProperties)
        properties.push_back(&entry.second);
    registerEffector(effector, manager);
    EXPECT_EQ(manager.getStateObject(name), state);
    std::size_t index = 0;
    for (const auto& entry : manager.dynProperties)
        EXPECT_EQ(&entry.second, properties[index++]);
    EXPECT_NO_THROW(setName(effector, name));
    EXPECT_THROW(setName(effector, "changed"), BasiliskError);
}

TYPED_TEST(MultibodyEffectorNames, detectsAndRepairsDuplicateCustomNames)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam first;
    TypeParam second;
    configure(first);
    configure(second);
    setName(first, "duplicate");
    setName(second, "duplicate");
    first.collectEffectorNames(manager);
    second.collectEffectorNames(manager);
    EXPECT_THROW(manager.resolveEffectorNames(), BasiliskError);
    EXPECT_TRUE(manager.stateContainer.stateMap.empty());
    setName(second, "corrected");
    second.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    registerEffector(first, manager);
    registerEffector(second, manager);
    EXPECT_NE(manager.getStateObject("duplicate"), manager.getStateObject("corrected"));
}

TEST(MultibodyNamingCollisions, avoidsOverlappingAutomaticPatternsAcrossSpinningTypes)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    std::vector<std::unique_ptr<SpinningBodyOneDOFStateEffector>> singles;
    for (int index = 0; index < 11; ++index) {
        singles.push_back(std::make_unique<SpinningBodyOneDOFStateEffector>());
        configure(*singles.back());
        singles.back()->collectEffectorNames(manager);
    }
    SpinningBodyTwoDOFStateEffector dual;
    dual.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    for (auto& single : singles)
        single->registerStates(manager);
    dual.registerStates(manager);
    EXPECT_EQ(singles.back()->getNameOfThetaState(), "spinningBodyTheta11");
    EXPECT_EQ(dual.getNameOfTheta1State(), "spinningBodyTheta12");
    EXPECT_EQ(dual.getNameOfInertialPositionProperty1(), "spinningBodyInertialPosition12");
    EXPECT_EQ(manager.stateContainer.stateMap.size(), 26U);
    EXPECT_EQ(manager.dynProperties.size(), 52U);
}

TEST(MultibodyNamingCollisions, reservesCustomPropertiesAcrossEffectorTypes)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    SpinningBodyOneDOFStateEffector spinner;
    configure(spinner);
    LinearTranslationOneDOFStateEffector slider;
    slider.setNameOfInertialPositionProperty("spinningBodyInertialPosition1");
    spinner.collectEffectorNames(manager);
    slider.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    spinner.registerStates(manager);
    registerEffector(slider, manager);
    EXPECT_EQ(spinner.getNameOfThetaState(), "spinningBodyTheta2");
    EXPECT_NE(manager.getPropertyReference(spinner.getNameOfInertialPositionProperty()),
              manager.getPropertyReference(slider.getNameOfInertialPositionProperty()));
}

TEST(MultibodyNamingCollisions, rejectsNestedCyclesAndRepeatedChildren)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    PrescribedMotionStateEffector parent;
    parent.addStateEffector(&parent);
    EXPECT_THROW(parent.collectEffectorNames(manager), BasiliskError);
    EXPECT_NO_THROW(parent.cancelEffectorNames(manager));
    EXPECT_NO_THROW(parent.cancelEffectorNames(manager));
    DynParamManager other;
    other.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    PrescribedMotionStateEffector repeated;
    SpinningBodyOneDOFStateEffector child;
    configure(child);
    repeated.addStateEffector(&child);
    repeated.addStateEffector(&child);
    EXPECT_THROW(repeated.collectEffectorNames(other), BasiliskError);
    EXPECT_NO_THROW(repeated.cancelEffectorNames(other));
    DynParamManager replacement;
    replacement.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    EXPECT_NO_THROW(child.collectEffectorNames(replacement));
}

TEST(MultibodyNamingCollisions, cancelsTheEntireNestedTreeBeforeMovingManagers)
{
    DynParamManager first;
    DynParamManager second;
    first.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    second.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    PrescribedMotionStateEffector root;
    PrescribedMotionStateEffector branch;
    SpinningBodyOneDOFStateEffector leaf;
    configure(leaf);
    root.setNameOfsigma_PMState("rootState");
    branch.setNameOfsigma_PMState("branchState");
    leaf.setNameOfThetaState("leafState");
    root.addStateEffector(&branch);
    branch.addStateEffector(&leaf);
    root.collectEffectorNames(first);
    root.cancelEffectorNames(first);
    EXPECT_NO_THROW(root.cancelEffectorNames(first));
    ASSERT_NO_THROW(root.collectEffectorNames(second));
    ASSERT_NO_THROW(second.resolveEffectorNames());
    // This is a naming-tree test; no dynamics of nested prescribed frames are evaluated.
    ASSERT_NO_THROW(root.registerStates(second));
    EXPECT_NE(second.getStateObject("rootState"), nullptr);
    EXPECT_NE(second.getStateObject("branchState"), nullptr);
    EXPECT_NE(second.getStateObject("leafState"), nullptr);

    // Every cancelled name, including descendant names, is available in the old manager.
    PrescribedMotionStateEffector replacementRoot;
    PrescribedMotionStateEffector replacementBranch;
    SpinningBodyOneDOFStateEffector replacementLeaf;
    replacementRoot.setNameOfsigma_PMState("rootState");
    replacementBranch.setNameOfsigma_PMState("branchState");
    replacementLeaf.setNameOfThetaState("leafState");
    replacementRoot.addStateEffector(&replacementBranch);
    replacementBranch.addStateEffector(&replacementLeaf);
    replacementRoot.collectEffectorNames(first);
    EXPECT_NO_THROW(first.resolveEffectorNames());
}

TEST(MultibodyNamingCollisions, rejectsWrongManagerCancellationWithoutCancellingTheParent)
{
    DynParamManager first;
    DynParamManager second;
    first.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    second.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    PrescribedMotionStateEffector parent;
    SpinningBodyOneDOFStateEffector child;
    parent.setNameOfsigma_PMState("retainedParent");
    parent.collectEffectorNames(first);
    child.collectEffectorNames(second);
    parent.addStateEffector(&child);
    EXPECT_THROW(parent.cancelEffectorNames(first), BasiliskError);

    PrescribedMotionStateEffector competing;
    competing.setNameOfsigma_PMState("retainedParent");
    competing.collectEffectorNames(first);
    EXPECT_THROW(first.resolveEffectorNames(), BasiliskError);
    child.cancelEffectorNames(second);
    parent.cancelEffectorNames(first);
    EXPECT_NO_THROW(first.resolveEffectorNames());
}

TEST(MultibodyNamingCollisions, rejectsResolvedTreeCancellationWithoutDetachingChildren)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    PrescribedMotionStateEffector parent;
    SpinningBodyOneDOFStateEffector child;
    configure(child);
    parent.addStateEffector(&child);
    parent.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    EXPECT_THROW(parent.cancelEffectorNames(manager), BasiliskError);
    EXPECT_NO_THROW(parent.registerStates(manager));
    DynParamManager other;
    other.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    EXPECT_THROW(child.collectEffectorNames(other), BasiliskError);
}

TEST(MultibodyNamingCollisions, registersNestedChildrenBeforeBindingAndFreezesAttachments)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    PrescribedMotionStateEffector parent;
    SpinningBodyOneDOFStateEffector child;
    configure(child);
    SpinningBodyTwoDOFStateEffector sibling;
    parent.addStateEffector(&child);
    parent.collectEffectorNames(manager);
    sibling.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    parent.registerStates(manager);
    sibling.registerStates(manager);
    parent.bindAttachedDynamicEffectors(manager);
    EXPECT_NE(manager.getStateObject(child.getNameOfThetaState()), nullptr);
    EXPECT_NE(manager.getPropertyReference(child.getNameOfInertialPositionProperty()), nullptr);
    EXPECT_THROW(parent.addStateEffector(&sibling), BasiliskError);
}

TEST(MultibodyNamingCollisions, keepsConstraintBindingsDistinctForTwoSegmentsOfOneParent)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    SpinningBodyTwoDOFStateEffector parent;
    ConstraintDynamicEffector constraint;
    constraint.setAlpha(1.0); // [-]
    constraint.setBeta(1.0);  // [-]
    parent.addDynamicEffector(&constraint, 2);
    parent.addDynamicEffector(&constraint, 1);
    parent.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    parent.registerStates(manager);
    parent.bindAttachedDynamicEffectors(manager);
    const auto names = constraint.getPropName_inertialPosition();
    ASSERT_EQ(names.size(), 2U);
    EXPECT_EQ(names[0], parent.getNameOfInertialPositionProperty2());
    EXPECT_EQ(names[1], parent.getNameOfInertialPositionProperty1());
    EXPECT_NO_THROW(parent.bindAttachedDynamicEffectors(manager));
    EXPECT_EQ(constraint.getPropName_inertialPosition(), names);
    (*manager.getPropertyReference(names[0]))(0) = 1.0; // [m]
    // The first call supplies the previous force to the second body; the next computes both forces.
    constraint.computeForceTorque(0.0, 0.0);                  // [s]
    constraint.computeForceTorque(0.0, 0.0);                  // [s]
    EXPECT_DOUBLE_EQ(constraint.forceExternal_N.norm(), 1.0); // [N]
    const Eigen::Vector3d force = constraint.forceExternal_N; // [N]
    constraint.computeForceTorque(0.0, 0.0);                  // [s]
    EXPECT_TRUE(constraint.forceExternal_N.isApprox(-force));
}

TEST(MultibodyNamingDynamics, appliesConstraintTorquesToTheSelectedSpinningSegments)
{
    for (const auto policy : { EffectorNamingPolicy::Legacy, EffectorNamingPolicy::ManagerLocal }) {
        for (const bool reverse : { false, true }) {
            SCOPED_TRACE(testing::Message() << "policy=" << static_cast<int>(policy) << ", reverse=" << reverse);
            Spacecraft vehicle;
            vehicle.hub.mHub = 100.0;        // [kg]
            const double hubInertia = 4.0;   // [kg m^2]
            const double innerInertia = 2.0; // [kg m^2]
            const double outerInertia = 3.0; // [kg m^2]
            vehicle.hub.IHubPntBc_B = hubInertia * Eigen::Matrix3d::Identity();
            vehicle.dynManager.setEffectorNamingPolicy(policy);
            SpinningBodyTwoDOFStateEffector spinner;
            spinner.mass1 = 1.0; // [kg]
            spinner.mass2 = 1.0; // [kg]
            spinner.s1Hat_S1 = Eigen::Vector3d::UnitX();
            spinner.s2Hat_S2 = Eigen::Vector3d::UnitY();
            spinner.IS1PntSc1_S1 = innerInertia * Eigen::Matrix3d::Identity();
            spinner.IS2PntSc2_S2 = outerInertia * Eigen::Matrix3d::Identity();
            ConstraintDynamicEffector constraint;
            constraint.setAlpha(1.0);                                 // [-]
            constraint.setBeta(1.0);                                  // [-]
            constraint.setSigma_B2B1Init(Eigen::MRPd(0.0, 0.1, 0.0)); // [-]
            spinner.addDynamicEffector(&constraint, reverse ? 2 : 1);
            spinner.addDynamicEffector(&constraint, reverse ? 1 : 2);
            vehicle.addStateEffector(&spinner);
            vehicle.initializeDynamics();
            // Complete another force pair so both bodies receive the constraint torque.
            vehicle.equationsOfMotion(0.0, 0.01);                                  // [s]
            const double lastBodyTorque = constraint.torqueExternalPntB_B.y();     // [N m]
            EXPECT_NEAR(std::abs(lastBodyTorque), 0.1, 1e-14);                     // [N m]
            const double outerTorque = reverse ? -lastBodyTorque : lastBodyTorque; // [N m]
            // With coincident origins, the inner body and hub rotate together about Y.
            // Equal and opposite torques give the relative outer-joint acceleration below.
            const double expectedAcceleration =
              outerTorque * (1.0 / outerInertia + 1.0 / (hubInertia + innerInertia)); // [rad/s^2]
            const auto* jointRate = vehicle.dynManager.getStateObject(spinner.getNameOfTheta2DotState());
            const auto* hubRate = vehicle.dynManager.getStateObject(vehicle.hub.nameOfHubOmega);
            EXPECT_NEAR(jointRate->getStateDeriv()(0, 0), expectedAcceleration, 1e-14);                     // [rad/s^2]
            EXPECT_NEAR(hubRate->getStateDeriv()(1, 0), -outerTorque / (hubInertia + innerInertia), 1e-14); // [rad/s^2]
        }
    }
}

} // namespace
