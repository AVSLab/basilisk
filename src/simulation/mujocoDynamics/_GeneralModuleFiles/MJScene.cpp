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

#include "architecture/utilities/macroDefinitions.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorAdaptiveRungeKutta.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorRK4.h"

#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <vector>

using MJBasilisk::detail::logAndThrow;

MJScene::MJScene(std::string xml, const std::vector<std::string>& files)
  : spec(*this, xml, files)
{
    this->AddFwdKinematicsToDynamicsTask(MJScene::FWD_KINEMATICS_PRIORITY);
    this->integrator = new svIntegratorRK4(this);

    // Replace default MuJoCo error/warning handling with our own
    mju_user_error = MJBasilisk::detail::logMujocoError;
    mju_user_warning = MJBasilisk::detail::logMujocoWarning;
}

MJScene
MJScene::fromFile(const std::string& fileName)
{
    std::stringstream os(std::stringstream::out);
    os << std::ifstream(fileName).rdbuf();
    return MJScene(os.str());
}

void
MJScene::AddModelToDynamicsTask(SysModel* model, int32_t priority)
{
    this->dynamicsTask.AddNewObject(model, priority);
}

void
MJScene::AddFwdKinematicsToDynamicsTask(int32_t priority)
{
    this->ownedSysModel.emplace_back(std::make_unique<MJFwdKinematics>(*this));
    this->ownedSysModel.back()->ModelTag = "FwdKinematics" + std::to_string(this->ownedSysModel.size() - 1);
    this->AddModelToDynamicsTask(this->ownedSysModel.back().get(), priority);
}

void
MJScene::AddModelToDiffusionDynamicsTask(SysModel* model, int32_t priority)
{
    this->dynamicsDiffusionTask.AddNewObject(model, priority);
}

void
MJScene::AddFwdKinematicsToDiffusionDynamicsTask(int32_t priority)
{
    this->ownedSysModel.emplace_back(std::make_unique<MJFwdKinematics>(*this));
    this->ownedSysModel.back()->ModelTag = "FwdKinematics" + std::to_string(this->ownedSysModel.size() - 1);
    this->AddModelToDiffusionDynamicsTask(this->ownedSysModel.back().get(), priority);
}

void
MJScene::SelfInit()
{
    this->dynamicsTask.SelfInitTaskList();
    this->dynamicsDiffusionTask.SelfInitTaskList();
}

void
MJScene::Reset(uint64_t CurrentSimNanos)
{
    this->timeBefore = CurrentSimNanos * NANO2SEC;
    this->firstDynamicsCall = true;
    this->initializeDynamics();
    this->dynamicsTask.TaskName = "Dynamics:" + this->ModelTag;
    this->dynamicsTask.ResetTaskList(CurrentSimNanos);
    this->dynamicsDiffusionTask.TaskName = "DiffusionDynamics:" + this->ModelTag;
    this->dynamicsDiffusionTask.ResetTaskList(CurrentSimNanos);
    this->writeOutputStateMessages(CurrentSimNanos);
}

void
MJScene::initializeDynamics()
{
    // A MuJoCo scene integrates a small, fixed set of bulk states regardless of
    // how many bodies or joints it contains: the whole position vector (qpos),
    // the whole velocity vector (qvel), one mass entry per body, and the actuator
    // state (act, only when the model has actuator activation states). Joints and
    // bodies address their own slices of these. States are registered at the
    // compiled model dimensions so a repeated Reset re-registers them at a
    // matching size.
    mjModel* model = this->spec.getMujocoModel();
    this->qposState = this->dynManager.registerState<MJQPosStateData>(model->nq, 1, "mujocoQpos");
    this->qvelState = this->dynManager.registerState(model->nv, 1, "mujocoQvel");
    this->massState = this->dynManager.registerState(model->nbody, 1, "mujocoMass");
    if (model->na > 0) {
        this->actState = this->dynManager.registerState(model->na, 1, "mujocoAct");
    }

    // The bulk position state advances quaternion blocks on SO(3).
    this->qposState->highOrderIntegration = this->highOrderAttitudeIntegration;

    // qpos and qvel bundle degrees of freedom of very different magnitudes
    // (e.g. orbital translation alongside attitude quaternion components), so the
    // adaptive integrator scales their truncation error per component. The
    // homogeneous act and mass states keep the default whole-vector measure.
    this->qposState->perComponentErrorControl = true;
    this->qvelState->perComponentErrorControl = true;

    bool recompiled = this->spec.recompileIfNeeded();
    if (!recompiled) {
        this->spec.configure();
    }

    // Seed the bulk states from mjData once, here rather than in configure():
    // configure() also runs on mid-simulation recompiles, where re-seeding would
    // discard state the user set on the stale model. body_mass includes the world
    // body's entry at index 0, which no MJBody owns.
    mjData* data = this->spec.getMujocoData();
    std::copy_n(data->qpos, model->nq, this->qposState->state.data());
    std::copy_n(data->qvel, model->nv, this->qvelState->state.data());
    std::copy_n(model->body_mass, model->nbody, this->massState->state.data());
    if (this->actState) {
        std::copy_n(data->act, model->na, this->actState->state.data());
    }

    // Register the states of the models in the dynamics task
    std::unordered_set<StatefulSysModel*> alreadyRegisteredModels;
    auto registerStatesOnSysModel = [this, &alreadyRegisteredModels](SysModel* sysModelPtr) {
        if (auto statefulSysModelPtr = dynamic_cast<StatefulSysModel*>(sysModelPtr)) {
            // Don't registerStates in a model twice!
            if (alreadyRegisteredModels.count(statefulSysModelPtr) > 0)
                return;

            statefulSysModelPtr->registerStates(
              DynParamRegisterer(this->dynManager,
                                 sysModelPtr->ModelTag.empty()
                                   ? std::string("model")
                                   : sysModelPtr->ModelTag + "_" + std::to_string(sysModelPtr->moduleID) + "_"));

            alreadyRegisteredModels.emplace(statefulSysModelPtr);
        }
    };

    for (auto [_, sysModelPtr] : this->dynamicsTask.TaskModels) {
        registerStatesOnSysModel(sysModelPtr);
    }
    for (auto [_, sysModelPtr] : this->dynamicsDiffusionTask.TaskModels) {
        registerStatesOnSysModel(sysModelPtr);
    }

    // If an adaptive integrator is advancing a free-joint body, zero the
    // relative tolerance on the bulk qpos/qvel states. At orbital position and
    // velocity scales, the default relative tolerance can permit absolute
    // errors large enough to destabilize stiff appendage dynamics. This holds
    // regardless of how the body's gravity is applied (e.g. NBodyGravity).
    bool hasFreeBody = false;
    for (auto&& body : this->spec.getBodies()) {
        if (body.isFree()) {
            hasFreeBody = true;
            break;
        }
    }
    auto* adaptiveIntegrator = dynamic_cast<StateVecAdaptiveIntegrator*>(this->integrator);
    if (adaptiveIntegrator && hasFreeBody) {
        adaptiveIntegrator->setRelativeTolerance("mujocoQpos", 0.0);
        adaptiveIntegrator->setRelativeTolerance("mujocoQvel", 0.0);
    }
}

void
MJScene::UpdateState(uint64_t CurrentSimNanos)
{
    this->integrateState(CurrentSimNanos);
    this->writeOutputStateMessages(CurrentSimNanos);
    for (auto&& body : this->spec.getBodies()) {
        body.writeStateDependentOutputMessages(CurrentSimNanos);
    }
}

void
MJScene::equationsOfMotion(double t, double timeStep)
{
    auto nanos = static_cast<uint64_t>(t * SEC2NANO);

    // Make sure the model is compiled
    this->spec.recompileIfNeeded();

    // Copy data from Basilisk state objects to MuJoCo structs
    updateMujocoArraysFromStates();

    // Keep MuJoCo's internal time in sync with the Basilisk simulation time so
    // diagnostics and MuJoCo warnings report the correct timestamp.
    this->spec.getMujocoData()->time = t;

    // On the first dynamics call, zero the CTRL array to prevent NaN/uninitialized
    // actuator commands from triggering instability at t=0.
    if (this->firstDynamicsCall) {
        auto m = this->spec.getMujocoModel();
        auto d = this->spec.getMujocoData();
        for (int i = 0; i < m->nu; ++i) {
            d->ctrl[i] = 0.0;
        }
        this->firstDynamicsCall = false;
    }

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

        // mj_setConst overwrites qpos with the reference pose; restore the integrator
        // state (and re-flag kinematics stale) before anything downstream reads qpos.
        updateMujocoArraysFromStates();
        this->mjModelConstStale = false;
    }

    // Execute the dynamics task!
    // (One of) The first module in this task should be a MJFwdKinematics
    // which will take the recently updated qpos and qvel data,
    // do the fwd kinematics, and update the state messages.
    // These messages can then be read by the rest of modules.
    this->dynamicsTask.ExecuteTaskList(nanos);

    // If the kinematics became stale while running dynamics modules, refresh
    // MuJoCo's cached position/velocity quantities before actuator, equality,
    // and acceleration calculations read them.
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
    if (std::any_of(qacc, qacc + this->spec.getMujocoModel()->nv, [](mjtNum v) { return std::isnan(v); })) {
        logAndThrow<std::runtime_error>("Encountered NaN acceleration at time " + std::to_string(t) +
                                        "s in MJScene with ID: " + std::to_string(moduleID));
    }

    // The derivative of the bulk position is the bulk velocity.  The
    // MJQPosStateData consumes it directly (default mode) or expands the
    // quaternion blocks into four-component rates (high-order mode).
    this->qposState->setDerivative(this->qvelState->getState());

    // The derivative of the bulk velocity is the computed acceleration.
    {
        auto qvelDeriv = this->qvelState->stateDeriv.data();
        std::copy_n(this->spec.getMujocoData()->qacc, this->spec.getMujocoModel()->nv, qvelDeriv);
    }

    // Also copy the derivative of the actuator states, if we have them
    if (this->spec.getMujocoModel()->na > 0) {
        auto actDeriv = this->actState->stateDeriv.data();
        std::copy_n(this->spec.getMujocoData()->act_dot, this->spec.getMujocoModel()->na, actDeriv);
    }

    // Update the derivative of the body mass property states (into the bulk
    // mass state, one entry per body).
    for (auto&& body : this->spec.getBodies()) {
        body.updateMassPropsDerivative();
    }
}

void
MJScene::equationsOfMotionDiffusion(double t, double timeStep)
{
    auto nanos = static_cast<uint64_t>(t * SEC2NANO);
    this->dynamicsDiffusionTask.ExecuteTaskList(nanos);
}

void
MJScene::preIntegration(uint64_t callTime)
{
    this->timeStep = diffNanoToSec(callTime, this->timeBeforeNanos);
}

void
MJScene::postIntegration(uint64_t callTimeNanos)
{
    this->timeBefore = callTimeNanos * NANO2SEC;
    this->timeBeforeNanos = callTimeNanos;
    double callTime = callTimeNanos * NANO2SEC;

    // Copy data from Basilisk state objects to MuJoCo structs
    updateMujocoArraysFromStates();

    if (extraEoMCall) {
        // If asked, this will call the equations of motion one last
        // time with the final/integrated state. This also calls
        // MJFwdKinematics::fwdKinematics
        equationsOfMotion(callTime, 0);
        equationsOfMotionDiffusion(callTime, 0);
    } else {
        // Always forward the kinematics with the final state
        MJFwdKinematics::fwdKinematics(*this, static_cast<uint64_t>(callTime * SEC2NANO));
    }
}

void
MJScene::writeFwdKinematicsMessages(uint64_t CurrentSimNanos)
{
    for (auto&& body : this->spec.getBodies()) {
        body.writeFwdKinematicsMessages(this->spec.getMujocoModel(), this->spec.getMujocoData(), CurrentSimNanos);
    }
    this->forwardKinematicsStale = false;
}

void
MJScene::saveToFile(std::string filename)
{
    std::string suffix = ".mjb";
    bool binary_file =
      filename.size() >= suffix.size() && filename.compare(filename.size() - suffix.size(), suffix.size(), suffix) == 0;

    if (binary_file) {
        mj_saveModel(this->getMujocoModel(), filename.c_str(), NULL, 0);
    } else {
        char error[1024];
        mj_saveXML(this->spec.getMujocoSpec(), filename.c_str(), error, sizeof(error));
    }
}

StateData*
MJScene::getActState()
{
    // Returns nullptr when the model has no actuator activation states (na == 0),
    // in which case no act state is created.  Callers must handle nullptr.
    return this->actState;
}

MJQPosStateData* MJScene::getQposState() { return this->qposState; }

StateData* MJScene::getQvelState() { return this->qvelState; }

StateData* MJScene::getMassState() { return this->massState; }

void
MJScene::printMujocoModelDebugInfo(const std::string& path)
{
    mj_printModel(this->getMujocoModel(), path.c_str());
}

std::vector<std::string>
MJScene::getBodyNames() const
{
    return this->spec.getBodyNames();
}

std::string
MJScene::getBodyParentName(const std::string& bodyName) const
{
    return this->spec.getBodyParentName(bodyName);
}

std::vector<MJGeomInfo>
MJScene::getGeomInfos() const
{
    return this->spec.getGeomInfos();
}

MJBody&
MJScene::getBody(const std::string& name)
{
    auto& bodies = this->spec.getBodies();
    auto bodyPtr =
      std::find_if(std::begin(bodies), std::end(bodies), [&](auto&& obj) { return obj.getName() == name; });

    if (bodyPtr == std::end(bodies)) {
        this->bskLogger.bskError("Unknown body '%s' in MJScene", name.c_str());
    }
    return *bodyPtr;
}

MJSite&
MJScene::getSite(const std::string& name)
{
    for (auto&& body : this->spec.getBodies()) {
        if (body.hasSite(name))
            return body.getSite(name);
    }
    this->bskLogger.bskError("Unknown site '%s' in MJScene", name.c_str());
}

MJEquality&
MJScene::getEquality(const std::string& name)
{
    auto& equalities = this->spec.getEqualities();
    auto equalityPtr =
      std::find_if(std::begin(equalities), std::end(equalities), [&](auto&& obj) { return obj.getName() == name; });

    if (equalityPtr == std::end(equalities)) {
        this->bskLogger.bskError("Unknown equality '%s' in MJScene", name.c_str());
    }
    return *equalityPtr;
}

MJSingleActuator&
MJScene::getSingleActuator(const std::string& name)
{
    return this->spec.getActuator<MJSingleActuator>(name);
}

MJForceActuator&
MJScene::getForceActuator(const std::string& name)
{
    return this->spec.getActuator<MJForceActuator>(name);
}

MJTorqueActuator&
MJScene::getTorqueActuator(const std::string& name)
{
    return this->spec.getActuator<MJTorqueActuator>(name);
}

MJForceTorqueActuator&
MJScene::getForceTorqueActuator(const std::string& name)
{
    return this->spec.getActuator<MJForceTorqueActuator>(name);
}

MJSingleActuator&
MJScene::addJointSingleActuator(const std::string& name, const std::string& joint)
{
    return this->spec.addJointSingleActuator(name, joint);
}

MJSingleActuator&
MJScene::addJointSingleActuator(const std::string& name, const MJJoint& joint)
{
    return this->addJointSingleActuator(name, joint.getName());
}

MJSingleActuator&
MJScene::addSingleActuator(const std::string& name, const std::string& site, const Eigen::Vector6d& gear)
{
    return this->spec.addSingleActuator(name, site, gear);
}

MJSingleActuator&
MJScene::addSingleActuator(const std::string& name, const MJSite& site, const Eigen::Vector6d& gear)
{
    return this->addSingleActuator(name, site.getName(), gear);
}

MJForceActuator&
MJScene::addForceActuator(const std::string& name, const std::string& site)
{
    return this->spec.addCompositeActuator<MJForceActuator>(name, site);
}

MJForceActuator&
MJScene::addForceActuator(const std::string& name, const MJSite& site)
{
    return this->addForceActuator(name, site.getName());
}

MJTorqueActuator&
MJScene::addTorqueActuator(const std::string& name, const std::string& site)
{
    return this->spec.addCompositeActuator<MJTorqueActuator>(name, site);
}

MJTorqueActuator&
MJScene::addTorqueActuator(const std::string& name, const MJSite& site)
{
    return this->addTorqueActuator(name, site.getName());
}

MJForceTorqueActuator&
MJScene::addForceTorqueActuator(const std::string& name, const std::string& site)
{
    return this->spec.addCompositeActuator<MJForceTorqueActuator>(name, site);
}

MJForceTorqueActuator&
MJScene::addForceTorqueActuator(const std::string& name, const MJSite& site)
{
    return this->addForceTorqueActuator(name, site.getName());
}

void
MJScene::updateMujocoArraysFromStates()
{
    auto mujocoModel = this->getMujocoModel();
    auto mujocoData  = this->getMujocoData();

    // Copy the bulk position/velocity states straight into mjData.
    std::copy_n(this->qposState->state.data(), mujocoModel->nq, mujocoData->qpos);
    std::copy_n(this->qvelState->state.data(), mujocoModel->nv, mujocoData->qvel);

    if (mujocoModel->na > 0) {
        std::copy_n(this->actState->state.data(), mujocoModel->na, mujocoData->act);
    }

    markKinematicsAsStale();
}

Eigen::VectorXd
MJScene::assembleFullQpos()
{
    // The bulk position state already mirrors the contiguous mjData::qpos layout.
    auto m = this->spec.getMujocoModel();
    return Eigen::Map<const Eigen::VectorXd>(this->qposState->state.data(), m->nq);
}

Eigen::VectorXd
MJScene::assembleFullQvel()
{
    auto m = this->spec.getMujocoModel();
    return Eigen::Map<const Eigen::VectorXd>(this->qvelState->state.data(), m->nv);
}

void
MJScene::writeOutputStateMessages(uint64_t CurrentSimNanos)
{
    // The actuator state only exists when the model has actuator activation
    // states; otherwise report an empty vector.
    Eigen::MatrixXd act = this->actState ? this->actState->getState() : Eigen::MatrixXd(0, 1);
    MJSceneStateMsgPayload stateOutMsgPayload{ this->assembleFullQpos(),
                                               this->assembleFullQvel(),
                                               act };

    stateOutMsg.write(&stateOutMsgPayload, this->moduleID, CurrentSimNanos);
}
