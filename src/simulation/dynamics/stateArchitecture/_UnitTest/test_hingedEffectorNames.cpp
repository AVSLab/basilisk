/* Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 * This file is distributed under the ISC License in LICENSE.
 */

#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/HingedRigidBodies/hingedRigidBodyStateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#include "simulation/dynamics/spacecraft/spacecraft.h"
#include <Eigen/Dense>
#include <array>
#include <gtest/gtest.h>
#include <string>

namespace {

class PropertyObserver : public DynamicEffector
{
  public:
    std::string lastPanelState;
    std::string lastPanelProperty;
    std::array<Eigen::MatrixXd*, 4> properties{};
    int bindingCount = 0;

    void linkInStates(DynParamManager&) override {}
    void computeForceTorque(double, double) override {}
    void linkInProperties(DynParamManager& manager) override
    {
        // The last panel must already exist when the first attachment binds.
        if (!this->lastPanelState.empty()) {
            EXPECT_NE(manager.getStateObject(this->lastPanelState), nullptr);
            EXPECT_NE(manager.getPropertyReference(this->lastPanelProperty), nullptr);
        }
        this->properties = { manager.getPropertyReference(this->propName_inertialPosition),
                             manager.getPropertyReference(this->propName_inertialVelocity),
                             manager.getPropertyReference(this->propName_inertialAttitude),
                             manager.getPropertyReference(this->propName_inertialAngVelocity) };
        ++this->bindingCount;
    }
};

void
configureHub(Spacecraft& spacecraft)
{
    spacecraft.hub.mHub = 100.0;                                      // [kg]
    spacecraft.hub.IHubPntBc_B = 100.0 * Eigen::Matrix3d::Identity(); // [kg m^2]
}

void
preparePanel(DynParamManager& manager, HingedRigidBodyStateEffector& panel)
{
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    panel.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    panel.registerStates(manager);
    panel.bindAttachedDynamicEffectors(manager);
}

} // namespace

TEST(HingedEffectorNames, registersEveryPanelBeforeBindingAttachments)
{
    Spacecraft spacecraft;
    configureHub(spacecraft);
    spacecraft.dynManager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    HingedRigidBodyStateEffector second;
    HingedRigidBodyStateEffector first;
    PropertyObserver observer;
    observer.lastPanelState = "hingedRigidBodyTheta2";
    observer.lastPanelProperty = "hingedRigidBodyInertialAngVelocity2";
    first.addDynamicEffector(&observer);
    spacecraft.addStateEffector(&first);
    spacecraft.addStateEffector(&second);
    spacecraft.initializeDynamics();

    EXPECT_EQ(first.getNameOfThetaState(), "hingedRigidBodyTheta1");
    EXPECT_EQ(second.getNameOfThetaState(), "hingedRigidBodyTheta2");
    EXPECT_EQ(observer.bindingCount, 1);
    EXPECT_EQ(observer.properties[0],
              spacecraft.dynManager.getPropertyReference(first.getNameOfInertialPositionProperty()));
    EXPECT_NE(observer.properties[0],
              spacecraft.dynManager.getPropertyReference(second.getNameOfInertialPositionProperty()));
}

TEST(HingedEffectorNames, customPropertyReservesAnEntireAutomaticIndex)
{
    Spacecraft spacecraft;
    configureHub(spacecraft);
    spacecraft.dynManager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    HingedRigidBodyStateEffector automatic;
    HingedRigidBodyStateEffector custom;
    PropertyObserver observer;
    custom.addDynamicEffector(&observer);
    custom.setNameOfInertialPositionProperty("hingedRigidBodyInertialPosition1");
    spacecraft.addStateEffector(&automatic);
    spacecraft.addStateEffector(&custom);
    spacecraft.initializeDynamics();

    EXPECT_EQ(automatic.getNameOfThetaState(), "hingedRigidBodyTheta2");
    EXPECT_EQ(automatic.getNameOfInertialAttitudeProperty(), "hingedRigidBodyInertialAttitude2");
    EXPECT_EQ(custom.getNameOfThetaState(), "hingedRigidBodyTheta3");
    EXPECT_EQ(custom.getNameOfInertialPositionProperty(), "hingedRigidBodyInertialPosition1");
    EXPECT_EQ(observer.properties[0], spacecraft.dynManager.getPropertyReference("hingedRigidBodyInertialPosition1"));
}

TEST(HingedEffectorNames, tracksAssignmentOfTheExactConstructorName)
{
    DynParamManager manager;
    HingedRigidBodyStateEffector unused;
    HingedRigidBodyStateEffector panel;
    const std::string constructorName = panel.getNameOfThetaState();
    panel.setNameOfThetaState(constructorName);
    preparePanel(manager, panel);
    EXPECT_EQ(panel.getNameOfThetaState(), constructorName);
    EXPECT_EQ(panel.getNameOfThetaDotState(), "hingedRigidBodyThetaDot1");
    EXPECT_NE(manager.getStateObject(constructorName), nullptr);
}

TEST(HingedEffectorNames, correctsPendingConfigurationWithoutLosingCollectionOrder)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    HingedRigidBodyStateEffector first;
    HingedRigidBodyStateEffector second;
    first.setNameOfThetaState("duplicate");
    second.setNameOfThetaState("duplicate");
    first.collectEffectorNames(manager);
    second.collectEffectorNames(manager);
    EXPECT_THROW(manager.resolveEffectorNames(), BasiliskError);
    first.setNameOfThetaState("corrected");
    first.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    first.registerStates(manager);
    second.registerStates(manager);
    EXPECT_EQ(first.getNameOfThetaDotState(), "hingedRigidBodyThetaDot1");
    EXPECT_EQ(second.getNameOfThetaDotState(), "hingedRigidBodyThetaDot2");
}

TEST(HingedEffectorNames, rejectsRegistrationBeforeResolution)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    HingedRigidBodyStateEffector panel;
    EXPECT_THROW(panel.registerStates(manager), BasiliskError);
    EXPECT_TRUE(manager.stateContainer.stateMap.empty());
    EXPECT_TRUE(manager.dynProperties.empty());
}

TEST(HingedEffectorNames, rejectsBindingBeforePropertiesExist)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    HingedRigidBodyStateEffector panel;
    PropertyObserver observer;
    panel.addDynamicEffector(&observer);
    panel.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    EXPECT_THROW(panel.bindAttachedDynamicEffectors(manager), BasiliskError);
    EXPECT_EQ(observer.bindingCount, 0);
    panel.registerStates(manager);
    EXPECT_NO_THROW(panel.bindAttachedDynamicEffectors(manager));
    EXPECT_EQ(observer.bindingCount, 1);
}

TEST(HingedEffectorNames, rejectsChangesAfterResolutionBeforeRegisteringData)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    HingedRigidBodyStateEffector panel;
    panel.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    panel.setNameOfThetaState("tooLate");
    EXPECT_THROW(panel.registerStates(manager), BasiliskError);
    EXPECT_TRUE(manager.stateContainer.stateMap.empty());
}

TEST(HingedEffectorNames, repeatedPreparationPreservesNamesAndStorage)
{
    Spacecraft spacecraft;
    configureHub(spacecraft);
    spacecraft.dynManager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    HingedRigidBodyStateEffector panel;
    PropertyObserver observer;
    panel.addDynamicEffector(&observer);
    spacecraft.addStateEffector(&panel);
    spacecraft.initializeDynamics();
    const auto properties = observer.properties;
    const auto* state = spacecraft.dynManager.getStateObject(panel.getNameOfThetaState());
    panel.setNameOfThetaState(panel.getNameOfThetaState());
    spacecraft.initializeDynamics();
    EXPECT_EQ(observer.properties, properties);
    EXPECT_EQ(state, spacecraft.dynManager.getStateObject(panel.getNameOfThetaState()));
    EXPECT_THROW(panel.setNameOfThetaState("renamed"), BasiliskError);
    EXPECT_THROW(panel.setNameOfInertialPositionProperty("renamed"), BasiliskError);
}

TEST(HingedEffectorNames, keepsCustomNamesExactWithAnOwnerPrefix)
{
    DynParamManager manager;
    HingedRigidBodyStateEffector panel;
    panel.nameOfSpacecraftAttachedTo = "vehicle";
    panel.setNameOfThetaState("customAngle");
    preparePanel(manager, panel);
    EXPECT_EQ(panel.getNameOfThetaState(), "customAngle");
    EXPECT_EQ(panel.getNameOfThetaDotState(), "vehiclehingedRigidBodyThetaDot1");
}

TEST(HingedEffectorNames, detectsConflictsWithFixedHubNames)
{
    Spacecraft spacecraft;
    configureHub(spacecraft);
    spacecraft.dynManager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    HingedRigidBodyStateEffector panel;
    panel.setNameOfThetaState(spacecraft.hub.nameOfHubSigma);
    spacecraft.addStateEffector(&panel);
    EXPECT_THROW(spacecraft.initializeDynamics(), BasiliskError);
    EXPECT_EQ(spacecraft.dynManager.stateContainer.stateMap.count(panel.getNameOfThetaDotState()), 0U);
}
