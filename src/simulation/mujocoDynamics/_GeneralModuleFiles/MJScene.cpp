/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "MJScene.h"

#include "MJFwdKinematics.h"
#include "StatefulSysModel.h"

#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorRK4.h"
#include "architecture/utilities/macroDefinitions.h"

#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cmath>

MJScene::MJScene(std::string xml, const std::vector<std::string>& files)
    : spec(*this, xml, files)
{
    this->AddFwdKinematicsToDynamicsTask(MJScene::FWD_KINEMATICS_PRIORITY);
    this->integrator = new svIntegratorRK4(this);

    // Replace default MuJoCo error/warning handling with our own
    mju_user_error = MJBasilisk::detail::logMujocoError;
    mju_user_warning = MJBasilisk::detail::logMujocoWarning;
}

MJScene MJScene::fromFile(const std::string& fileName)
{
    std::stringstream os(std::stringstream::out);
    os << std::ifstream(fileName).rdbuf();
    return MJScene(os.str());
}

void MJScene::AddModelToDynamicsTask(SysModel* model, int32_t priority)
{
    this->dynamicsTask.AddNewObject(model, priority);
}

void MJScene::AddFwdKinematicsToDynamicsTask(int32_t priority)
{
    this->ownedSysModel.emplace_back(std::make_unique<MJFwdKinematics>(*this));
    this->ownedSysModel.back()->ModelTag = "FwdKinematics" + std::to_string(this->ownedSysModel.size()-1);
    this->AddModelToDynamicsTask(this->ownedSysModel.back().get(), priority);
}

void MJScene::SelfInit() { this->dynamicsTask.SelfInitTaskList(); }

void MJScene::Reset(uint64_t CurrentSimNanos)
{
    this->timeBefore = CurrentSimNanos * NANO2SEC;
    this->dynamicsTask.TaskName = "Dynamics:" + this->ModelTag;
    this->dynamicsTask.ResetTaskList(CurrentSimNanos);
    this->initializeDynamics();
    this->writeOutputStateMessages(CurrentSimNanos);
}

void MJScene::initializeDynamics()
{
    // We need to use a special StateData type for qpos
    // since it is integrated using a mujoco function
    this->qposState = this->dynManager.registerState<MJQPosStateData>(1, 1, "mujocoQpos");

    this->qvelState = this->dynManager.registerState(1, 1, "mujocoQvel");
    this->actState = this->dynManager.registerState(1, 1, "mujocoAct");

    for (auto&& body : this->spec.getBodies()) {
        body.registerStates(DynParamRegisterer(
            this->dynManager,
            "body_" + body.getName() + "_"
        ));
    }

    // Make sure the spec is compiled
    bool recompiled = this->spec.recompileIfNeeded();

    // Always call `configure`, which will reshape the states size
    if (!recompiled) {
        this->spec.configure();
    }

    // Register the states of the models in the dynamics task
    for (auto[_, sysModelPtr] : this->dynamicsTask.TaskModels)
    {
        if (auto statefulSysModelPtr = dynamic_cast<StatefulSysModel*>(sysModelPtr))
        {
            statefulSysModelPtr->registerStates(DynParamRegisterer(
                this->dynManager,
                sysModelPtr->ModelTag.empty() ? std::string("model") : sysModelPtr->ModelTag
                + "_" + std::to_string(sysModelPtr->moduleID) + "_"
            ));
        }
    }
}

void MJScene::UpdateState(uint64_t CurrentSimNanos)
{
    this->integrateState(CurrentSimNanos);
    this->writeOutputStateMessages(CurrentSimNanos);
    for (auto&& body : this->spec.getBodies()) {
        body.writeStateDependentOutputMessages(CurrentSimNanos);
    }
}

void MJScene::equationsOfMotion(double t, double timeStep)
{
    auto nanos = static_cast<uint64_t>(t * SEC2NANO);

    // Make sure the model is compiled
    this->spec.recompileIfNeeded();

    // Copy data from Basilisk state objects to MuJoCo structs
    updateMujocoArraysFromStates();

    for (auto&& body : this->spec.getBodies()) {
        // The mass of bodies is stored as a state, which may evolve in time.
        // Mujoco expects mass properties to be stored in mjModel, so we need
        // to update the mjModel with the mass properties of the bodies.
        body.updateMujocoModelFromMassProps();

        body.writeStateDependentOutputMessages(nanos);
    }

    // Mujoco models cache certain computations that depend on values that are
    // supposed to be constant during mujoco simulations (like body mass). However,
    // we need to alter some of them, in which case we need to update the 'constants'.
    if (areMujocoModelConstStale()) {
        mj_setConst(this->spec.getMujocoModel(), this->spec.getMujocoData());
    }

    // Execute the dynamics task!
    // (One of) The first module in this task should be a MJFwdKinematics
    // which will take the recently updated qpos and qvel data,
    // do the fwd kinematics, and update the state messages.
    // These messages can then be read by the rest of modules.
    this->dynamicsTask.ExecuteTaskList(nanos);

    // TODO: When allowing to prescribe the mass, remember to mj_setConst.
    // This is similar to how prescribing joint states should be followed
    // by running forward kinematics.

    // If the kinematics have become stale while running the dynamics
    // modules, we need to update them so that the accelerations
    // calculations are correct. We do not need to write the messages
    // though, since nothing needs them (only running mujoco now)
    if (areKinematicsStale()) {
        mj_fwdPosition(this->spec.getMujocoModel(), this->spec.getMujocoData());
        mj_fwdVelocity(this->spec.getMujocoModel(), this->spec.getMujocoData());
    }

    // Update the ctrl array in mjData from the inputs in the actuators
    for (auto&& actuator : this->spec.getActuators()) {
        actuator->updateCtrl(this->spec.getMujocoData());
    }

    // Update the prescribed joints equalities
    for (auto&& body : this->spec.getBodies()) {
        body.updateConstrainedEqualityJoints();
    }

    // These methods will compute the accelerations
    mj_fwdActuation(this->spec.getMujocoModel(), this->spec.getMujocoData());
    mj_fwdAcceleration(this->spec.getMujocoModel(), this->spec.getMujocoData());
    mj_fwdConstraint(this->spec.getMujocoModel(), this->spec.getMujocoData());

    // Sanity check the produced accelerations
    auto qacc = this->spec.getMujocoData()->qacc;
    if (std::any_of(qacc, qacc + this->spec.getMujocoModel()->nv, [](mjtNum v){return std::isnan(v);})) {
        logAndThrow<std::runtime_error>("Encountered NaN acceleration at time " +
                                        std::to_string(t) +
                                        "s in MJScene with ID: " + std::to_string(moduleID));
    }

    // The derivative of the position is simply the state of the velocity
    this->qposState->setDerivative(this->qvelState->getState());

    // Copy the computed accelerations into the derivative of the velocity
    auto qvelDeriv = this->qvelState->stateDeriv.data();
    std::copy_n(qacc, this->spec.getMujocoModel()->nv, qvelDeriv);

    // Also copy the derivative of the actuator states, if we have them
    if (this->spec.getMujocoModel()->na > 0) {
        auto actDeriv = this->actState->stateDeriv.data();
        std::copy_n(this->spec.getMujocoData()->act_dot, this->spec.getMujocoModel()->na, actDeriv);
    }

    // Update the derivative of the body mass property states
    for (auto&& body : this->spec.getBodies()) {
        body.updateMassPropsDerivative();
    }
}

void MJScene::preIntegration(uint64_t callTimeNanos) { this->timeStep = diffNanoToSec(callTimeNanos, this->timeBeforeNanos); }

void MJScene::postIntegration(uint64_t callTimeNanos)
{
    this->timeBefore = callTimeNanos * NANO2SEC;
    this->timeBeforeNanos = callTimeNanos;
    double callTime = callTimeNanos * NANO2SEC;

    // Copy data from Basilisk state objects to MuJoCo structs
    updateMujocoArraysFromStates();

    if (extraEoMCall)
    {
        // If asked, this will call the equations of motion one last
        // time with the final/integrated state. This also calls
        // MJFwdKinematics::fwdKinematics
        equationsOfMotion(callTime, 0);
    }
    else
    {
        // Always forward the kinematics with the final state
        MJFwdKinematics::fwdKinematics(*this, static_cast<uint64_t>(callTime * SEC2NANO));
    }
}

void MJScene::writeFwdKinematicsMessages(uint64_t CurrentSimNanos)
{
    for (auto&& body : this->spec.getBodies()) {
        body.writeFwdKinematicsMessages(this->spec.getMujocoModel(), this->spec.getMujocoData(), CurrentSimNanos);
    }
    this->forwardKinematicsStale = false;
}

void MJScene::saveToFile(std::string filename)
{
    std::string suffix = ".mjb";
    bool binary_file = filename.size() >= suffix.size() &&
        filename.compare(filename.size() - suffix.size(), suffix.size(), suffix) == 0;

    if (binary_file)
    {
        mj_saveModel(this->getMujocoModel(), filename.c_str(), NULL, 0);
    }
    else
    {
        char error[1024];
        mj_saveXML(this->spec.getMujocoSpec(), filename.c_str(), error, sizeof(error));
    }
}

MJQPosStateData* MJScene::getQposState()
{
    if (!this->qposState) {
        logAndThrow<std::runtime_error>("Tried to get qpos state before initialization.");
    }
    return this->qposState;
}

StateData* MJScene::getQvelState()
{
    if (!this->qvelState) {
        logAndThrow<std::runtime_error>("Tried to get qvel state before initialization.");
    }
    return this->qvelState;
}

StateData* MJScene::getActState()
{
    if (!this->actState) {
        logAndThrow<std::runtime_error>("Tried to get qpos state before initialization.");
    }
    return this->actState;
}

void MJScene::printMujocoModelDebugInfo(const std::string& path)
{
    mj_printModel(this->getMujocoModel(), path.c_str());
}

MJBody& MJScene::getBody(const std::string& name)
{
    auto& bodies = this->spec.getBodies();
    auto bodyPtr = std::find_if(std::begin(bodies),
                                std::end(bodies),
                                [&](auto&& obj) { return obj.getName() == name; });

    if (bodyPtr == std::end(bodies)) {
        logAndThrow("Unknown body '" + name + "' in MJScene");
    }
    return *bodyPtr;
}

MJSite& MJScene::getSite(const std::string& name)
{
    for (auto&& body : this->spec.getBodies()) {
        if (body.hasSite(name)) return body.getSite(name);
    }
    logAndThrow("Unknown site '" + name + "' in MJScene");
}

MJEquality&
MJScene::getEquality(const std::string& name)
{
    auto& equalities = this->spec.getEqualities();
    auto equalityPtr = std::find_if(std::begin(equalities),
                                std::end(equalities),
                                [&](auto&& obj) { return obj.getName() == name; });

    if (equalityPtr == std::end(equalities)) {
        logAndThrow("Unknown equality '" + name + "' in MJScene");
    }
    return *equalityPtr;
}

MJSingleActuator& MJScene::getSingleActuator(const std::string& name)
{
    return this->spec.getActuator<MJSingleActuator>(name);
}

MJForceActuator& MJScene::getForceActuator(const std::string& name)
{
    return this->spec.getActuator<MJForceActuator>(name);
}

MJTorqueActuator& MJScene::getTorqueActuator(const std::string& name)
{
    return this->spec.getActuator<MJTorqueActuator>(name);
}

MJForceTorqueActuator& MJScene::getForceTorqueActuator(const std::string& name)
{
    return this->spec.getActuator<MJForceTorqueActuator>(name);
}

MJSingleActuator& MJScene::addJointSingleActuator(const std::string& name,
                                             const std::string& joint)
{
    return this->spec.addJointSingleActuator(name, joint);
}

MJSingleActuator&
MJScene::addJointSingleActuator(const std::string& name, const MJJoint& joint)
{
    return this->addJointSingleActuator(name, joint.getName());
}

MJSingleActuator& MJScene::addSingleActuator(const std::string& name,
                                             const std::string& site,
                                             const Eigen::Vector6d& gear)
{
    return this->spec.addSingleActuator(name, site, gear);
}

MJSingleActuator&
MJScene::addSingleActuator(const std::string& name, const MJSite& site, const Eigen::Vector6d& gear)
{
    return this->addSingleActuator(name, site.getName(), gear);
}

MJForceActuator& MJScene::addForceActuator(const std::string& name, const std::string& site)
{
    return this->spec.addCompositeActuator<MJForceActuator>(name, site);
}

MJForceActuator & MJScene::addForceActuator(const std::string & name, const MJSite & site)
{
    return this->addForceActuator(name, site.getName());
}

MJTorqueActuator& MJScene::addTorqueActuator(const std::string& name, const std::string& site)
{
    return this->spec.addCompositeActuator<MJTorqueActuator>(name, site);
}

MJTorqueActuator&
MJScene::addTorqueActuator(const std::string& name, const MJSite& site)
{
    return this->addTorqueActuator(name, site.getName());
}

MJForceTorqueActuator& MJScene::addForceTorqueActuator(const std::string& name,
                                                       const std::string& site)
{
    return this->spec.addCompositeActuator<MJForceTorqueActuator>(name, site);
}

MJForceTorqueActuator&
MJScene::addForceTorqueActuator(const std::string& name, const MJSite& site)
{
    return this->addForceTorqueActuator(name, site.getName());
}

void MJScene::updateMujocoArraysFromStates()
{
    auto mujocoModel = this->getMujocoModel();
    auto mujocoData = this->getMujocoData();

    // Copy the joint positions and velocity stored in the states
    // into the mujoco arrays
    std::copy_n(this->qposState->state.data(), mujocoModel->nq, mujocoData->qpos);

    std::copy_n(this->qvelState->state.data(), mujocoModel->nv, mujocoData->qvel);

    if (mujocoModel->na > 0) {
        std::copy_n(this->actState->state.data(), mujocoModel->na, mujocoData->act);
    }

    markKinematicsAsStale();
}

void MJScene::writeOutputStateMessages(uint64_t CurrentSimNanos)
{
    MJSceneStateMsgPayload stateOutMsgPayload{this->qposState->getState(),
                                              this->qvelState->getState(),
                                              this->actState->getState()};

    stateOutMsg.write(&stateOutMsgPayload, this->moduleID, CurrentSimNanos);
}
