/* Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 * This file is distributed under the ISC License in LICENSE.
 */

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/HingedRigidBodies/hingedRigidBodyStateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/prescribedMotion/prescribedMotionStateEffector.h"
#include "simulation/dynamics/spacecraft/spacecraft.h"
#include "simulation/dynamics/spinningBodies/spinningBodiesOneDOF/spinningBodyOneDOFStateEffector.h"
#include <Eigen/Dense>
#include <array>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace {

class PreparationObserver : public DynamicEffector
{
  public:
    std::vector<std::string> requiredStates;
    std::vector<std::string> requiredProperties;
    std::array<Eigen::MatrixXd*, 4> properties{};
    int bindingCount = 0;

    void linkInStates(DynParamManager& manager) override { this->checkRegistration(manager); }
    void computeForceTorque(double, double) override {}
    void linkInProperties(DynParamManager& manager) override
    {
        this->checkRegistration(manager);
        this->properties = { manager.getPropertyReference(this->propName_inertialPosition),
                             manager.getPropertyReference(this->propName_inertialVelocity),
                             manager.getPropertyReference(this->propName_inertialAttitude),
                             manager.getPropertyReference(this->propName_inertialAngVelocity) };
        ++this->bindingCount;
    }

  private:
    void checkRegistration(const DynParamManager& manager) const
    {
        for (const auto& name : this->requiredStates) {
            EXPECT_EQ(manager.stateContainer.stateMap.count(name), 1U) << name;
        }
        for (const auto& name : this->requiredProperties) {
            EXPECT_EQ(manager.dynProperties.count(name), 1U) << name;
        }
    }
};

class ExternalEffector : public StateEffector
{
  public:
    int registrationCount = 0;
    void registerStates(DynParamManager&) override { ++this->registrationCount; }
    void linkInStates(DynParamManager&) override {}
    void computeDerivatives(double, Eigen::Vector3d, Eigen::Vector3d, Eigen::MRPd) override {}
};

class TaggedExternalEffector
  : public ExternalEffector
  , public SysModel
{};

void
configure(Spacecraft& vehicle)
{
    vehicle.hub.mHub = 100.0;                                      // [kg]
    vehicle.hub.IHubPntBc_B = 100.0 * Eigen::Matrix3d::Identity(); // [kg m^2]
    vehicle.dynManager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
}

struct Branches
{
    // Construct the last attached leaf first, so constructor counters cannot determine order.
    SpinningBodyOneDOFStateEffector last;
    SpinningBodyOneDOFStateEffector first;
    PrescribedMotionStateEffector secondRoot;
    PrescribedMotionStateEffector firstRoot;
    PreparationObserver firstObserver;
    PreparationObserver lastObserver;

    Branches()
    {
        first.sHat_S = Eigen::Vector3d::UnitX();
        last.sHat_S = Eigen::Vector3d::UnitY();
        first.thetaInit = 0.125; // [rad]
        last.thetaInit = -0.25;  // [rad]
        // A later custom declaration must reserve the earlier automatic candidate.
        last.setNameOfThetaState("prescribedObjectspinningBodyTheta1");
        last.setNameOfInertialPositionProperty("spinningBodyInertialPosition1");
        firstRoot.setNameOfsigma_PMState("firstFrame");
        secondRoot.setNameOfsigma_PMState("lastFrame");
        firstRoot.addStateEffector(&first);
        secondRoot.addStateEffector(&last);
        first.addDynamicEffector(&firstObserver);
        last.addDynamicEffector(&lastObserver);
        for (auto* observer : { &firstObserver, &lastObserver }) {
            observer->requiredStates = {
                "firstFrame", "lastFrame", "prescribedObjectspinningBodyTheta2", "prescribedObjectspinningBodyTheta1"
            };
            observer->requiredProperties = { "spinningBodyInertialPosition2", "spinningBodyInertialPosition1" };
        }
    }

    std::vector<StateEffector*> roots() { return { &firstRoot, &secondRoot }; }
};

TEST(EffectorNamingIntegration, spacecraftRegistersEveryBranchBeforeBinding)
{
    Branches branches;
    Spacecraft vehicle;
    configure(vehicle);
    for (auto* root : branches.roots()) {
        vehicle.addStateEffector(root);
    }
    PreparationObserver hubObserver;
    hubObserver.requiredStates = branches.firstObserver.requiredStates;
    hubObserver.requiredProperties = branches.firstObserver.requiredProperties;
    vehicle.addDynamicEffector(&hubObserver);
    vehicle.initializeDynamics();
    EXPECT_EQ(branches.firstObserver.bindingCount, 1);
    EXPECT_EQ(branches.lastObserver.bindingCount, 1);
    EXPECT_EQ(branches.first.getNameOfThetaState(), "prescribedObjectspinningBodyTheta2");
    EXPECT_EQ(branches.last.getNameOfThetaState(), "prescribedObjectspinningBodyTheta1");
    EXPECT_NE(branches.firstObserver.properties[0], branches.lastObserver.properties[0]);
    const auto firstProperties = branches.firstObserver.properties;
    const auto lastProperties = branches.lastObserver.properties;
    auto* firstState = vehicle.dynManager.getStateObject(branches.first.getNameOfThetaState());
    const auto stateCount = vehicle.dynManager.stateContainer.stateMap.size();
    const auto propertyCount = vehicle.dynManager.dynProperties.size();
    vehicle.initializeDynamics();
    EXPECT_EQ(branches.firstObserver.bindingCount, 2);
    EXPECT_EQ(branches.lastObserver.bindingCount, 2);
    EXPECT_EQ(branches.firstObserver.properties, firstProperties);
    EXPECT_EQ(branches.lastObserver.properties, lastProperties);
    EXPECT_EQ(vehicle.dynManager.getStateObject(branches.first.getNameOfThetaState()), firstState);
    EXPECT_EQ(vehicle.dynManager.stateContainer.stateMap.size(), stateCount);
    EXPECT_EQ(vehicle.dynManager.dynProperties.size(), propertyCount);
    EXPECT_DOUBLE_EQ(firstState->getState()(0, 0), branches.first.thetaInit);
}

TEST(EffectorNamingIntegration, sharedManagerPreparesMultipleTreesAndAllTheirDescendants)
{
    Branches branches;
    PrescribedMotionStateEffector intermediate;
    SpinningBodyOneDOFStateEffector deepLeaf;
    deepLeaf.sHat_S = Eigen::Vector3d::UnitZ();
    deepLeaf.setNameOfThetaState("deepAngle");
    intermediate.setNameOfsigma_PMState("middleFrame");
    intermediate.addStateEffector(&deepLeaf);
    branches.firstRoot.addStateEffector(&intermediate);
    PreparationObserver deepObserver;
    deepLeaf.addDynamicEffector(&deepObserver);
    for (auto* observer : { &branches.firstObserver, &branches.lastObserver, &deepObserver }) {
        observer->requiredStates = {
            "firstFrame", "middleFrame", "deepAngle", "lastFrame", "prescribedObjectspinningBodyTheta1"
        };
        observer->requiredProperties = { "spinningBodyInertialPosition1" };
    }
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    const auto roots = branches.roots();
    StateEffector::collectEffectorNames(manager, roots);
    // Fixed names from all owners must participate before resolving the shared manager.
    manager.registerState(1, 1, "prescribedObjectspinningBodyTheta2");
    manager.createProperty("spinningBodyInertialPosition3", Eigen::Vector3d::Zero());
    manager.resolveEffectorNames();
    // This checks naming preparation at arbitrary depth, without assuming support for
    // the dynamics of a prescribed frame attached to another prescribed frame.
    for (auto* root : roots) {
        root->registerStates(manager);
    }
    for (auto* root : roots) {
        root->bindAttachedDynamicEffectors(manager);
    }
    EXPECT_EQ(branches.first.getNameOfThetaState(), "prescribedObjectspinningBodyTheta4");
    EXPECT_EQ(branches.last.getNameOfThetaState(), "prescribedObjectspinningBodyTheta1");
    EXPECT_EQ(deepObserver.bindingCount, 1);
    EXPECT_NE(deepObserver.properties[0], branches.firstObserver.properties[0]);
    const auto stateCount = manager.stateContainer.stateMap.size();
    const auto propertyCount = manager.dynProperties.size();
    const auto properties = deepObserver.properties;
    StateEffector::collectEffectorNames(manager, roots);
    manager.resolveEffectorNames();
    for (auto* root : roots) {
        root->registerStates(manager);
    }
    for (auto* root : roots) {
        root->bindAttachedDynamicEffectors(manager);
    }
    EXPECT_EQ(deepObserver.bindingCount, 2);
    EXPECT_EQ(deepObserver.properties, properties);
    EXPECT_EQ(manager.stateContainer.stateMap.size(), stateCount);
    EXPECT_EQ(manager.dynProperties.size(), propertyCount);
}

TEST(EffectorNamingIntegration, unsupportedExternalEffectorFailsBeforeAnyRegistrationOrBinding)
{
    for (const bool nested : { false, true }) {
        SCOPED_TRACE(nested);
        Spacecraft vehicle;
        configure(vehicle);
        HingedRigidBodyStateEffector supported;
        PreparationObserver observer;
        supported.addDynamicEffector(&observer);
        vehicle.addStateEffector(&supported);
        PrescribedMotionStateEffector root;
        PrescribedMotionStateEffector branch;
        TaggedExternalEffector external;
        external.ModelTag = "externalPanel%name";
        if (nested) {
            root.addStateEffector(&branch);
            branch.addStateEffector(&external);
            vehicle.addStateEffector(&root);
        } else {
            vehicle.addStateEffector(&external);
        }
        try {
            vehicle.initializeDynamics();
            FAIL() << "Unsupported external effector was accepted";
        } catch (const BasiliskError& error) {
            const std::string message = error.what();
            EXPECT_NE(message.find(external.ModelTag), std::string::npos);
            EXPECT_NE(message.find("describeEffectorNames()"), std::string::npos);
            EXPECT_NE(message.find("legacy naming"), std::string::npos);
        }
        EXPECT_TRUE(vehicle.dynManager.stateContainer.stateMap.empty());
        EXPECT_TRUE(vehicle.dynManager.dynProperties.empty());
        EXPECT_EQ(external.registrationCount, 0);
        EXPECT_EQ(observer.bindingCount, 0);
    }
}

TEST(EffectorNamingIntegration, legacyStillAcceptsExternalEffectors)
{
    Spacecraft vehicle;
    vehicle.hub.mHub = 100.0; // [kg]
    ExternalEffector external;
    vehicle.addStateEffector(&external);
    EXPECT_NO_THROW(vehicle.initializeDynamics());
    EXPECT_EQ(external.registrationCount, 1);
}

} // namespace
