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

#include "simulation/dynamics/_GeneralModuleFiles/stateRegistry.h"
#include "MJScene.h"

#include "MJFwdKinematics.h"
#include "StatefulSysModel.h"

#include "architecture/utilities/macroDefinitions.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorAdaptiveRungeKutta.h"
#include "simulation/dynamics/_GeneralModuleFiles/svIntegratorRK4.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <vector>

using MJBasilisk::detail::logAndThrow;
using MJBasilisk::detail::checkedMjtSizeCast;

namespace {
struct MujocoStateSegments
{
    StateBufferSegment qposState;
    StateBufferSegment qvelState;
    StateBufferSegment qposDerivative;
    StateBufferSegment qvelDerivative;
};

class ScopedFlag
{
  public:
    explicit ScopedFlag(bool& flag) noexcept
      : flag(flag)
      , previous(flag)
    {
        flag = true;
    }

    ScopedFlag(const ScopedFlag&) = delete;
    ScopedFlag& operator=(const ScopedFlag&) = delete;

    ~ScopedFlag() noexcept { this->flag = this->previous; }

  private:
    bool& flag;
    bool previous;
};

template<typename Callback>
void
forEachUniqueTaskModel(SysModelTask& dynamicsTask, SysModelTask& diffusionTask, Callback&& callback)
{
    std::unordered_set<SysModel*> visited;
    visited.reserve(dynamicsTask.TaskModels.size() + diffusionTask.TaskModels.size());
    std::vector<SysModel*> models;
    models.reserve(dynamicsTask.TaskModels.size() + diffusionTask.TaskModels.size());
    auto collectTask = [&visited, &models](const SysModelTask& task) {
        for (const auto& modelPair : task.TaskModels) {
            if (visited.emplace(modelPair.ModelPtr).second) {
                models.push_back(modelPair.ModelPtr);
            }
        }
    };
    collectTask(dynamicsTask);
    collectTask(diffusionTask);
    for (SysModel* model : models) {
        callback(model);
    }
}
}

MJScene::MJScene(std::string xml, const std::vector<std::string>& files)
  : spec(*this, xml, files)
{
    this->AddFwdKinematicsToDynamicsTask(MJScene::FWD_KINEMATICS_PRIORITY);
    this->setIntegrator(new svIntegratorRK4(this));

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
    this->requireSceneMutationAllowed("MJScene::AddModelToDynamicsTask");
    if (dynamic_cast<StatefulSysModel*>(model) != nullptr) {
        this->requireMutableTopology("MJScene::AddModelToDynamicsTask");
    }
    this->dynamicsTask.AddNewObject(model, priority);
}

void
MJScene::AddFwdKinematicsToDynamicsTask(int32_t priority)
{
    this->requireSceneMutationAllowed("MJScene::AddFwdKinematicsToDynamicsTask");
    this->ownedSysModel.emplace_back(std::make_unique<MJFwdKinematics>(*this));
    this->ownedSysModel.back()->ModelTag = "FwdKinematics" + std::to_string(this->ownedSysModel.size() - 1);
    this->AddModelToDynamicsTask(this->ownedSysModel.back().get(), priority);
}

void
MJScene::AddModelToDiffusionDynamicsTask(SysModel* model, int32_t priority)
{
    this->requireSceneMutationAllowed("MJScene::AddModelToDiffusionDynamicsTask");
    if (dynamic_cast<StatefulSysModel*>(model) != nullptr) {
        this->requireMutableTopology("MJScene::AddModelToDiffusionDynamicsTask");
    }
    this->dynamicsDiffusionTask.AddNewObject(model, priority);
}

void
MJScene::AddFwdKinematicsToDiffusionDynamicsTask(int32_t priority)
{
    this->requireSceneMutationAllowed("MJScene::AddFwdKinematicsToDiffusionDynamicsTask");
    this->ownedSysModel.emplace_back(std::make_unique<MJFwdKinematics>(*this));
    this->ownedSysModel.back()->ModelTag = "FwdKinematics" + std::to_string(this->ownedSysModel.size() - 1);
    this->AddModelToDiffusionDynamicsTask(this->ownedSysModel.back().get(), priority);
}

void
MJScene::SelfInit()
{
    ScopedFlag mutationGuard(this->sceneMutationBlocked);
    forEachUniqueTaskModel(this->dynamicsTask, this->dynamicsDiffusionTask, [](SysModel* model) { model->SelfInit(); });
}

void
MJScene::Reset(uint64_t CurrentSimNanos)
{
    this->requireSceneMutationAllowed("MJScene::Reset");
    this->spec.configureForStateRegistration();
    ScopedFlag mutationGuard(this->sceneMutationBlocked);
    MJSpec::NoRecompileGuard recompileGuard(this->spec);
    mjModel* model = this->spec.getMujocoModel();
    mjData* data = this->spec.getMujocoData();
    const bool topologyAlreadyFinalized = this->dynManager.statesAreFinalized();
    if (!this->modelTopology.registered) {
        this->modelTopology = MujocoTopology::capture(*model, this->highOrderAttitudeIntegration);
    }
    this->prepareOutputStateMessageStorage(model->nq, model->nv, model->na);

    this->timeBefore = static_cast<double>(CurrentSimNanos) * NANO2SEC;
    this->timeBeforeNanos = CurrentSimNanos;
    this->firstDynamicsCall = true;
    this->registerAndBindDynamicsState(model, data, topologyAlreadyFinalized);
    if (topologyAlreadyFinalized) {
        this->synchronizeMujocoFromStates(model, data);
    }
    this->dynamicsTask.TaskName = "Dynamics:" + this->ModelTag;
    this->dynamicsDiffusionTask.TaskName = "DiffusionDynamics:" + this->ModelTag;
    this->resetTaskModels(CurrentSimNanos);
    if (this->spec.hasPendingModelChanges()) {
        throw std::logic_error("Configure MuJoCo specification changes before Reset.");
    }
    this->populateOutputStateMessagePayload(model->nq, model->nv, model->na);
    this->synchronizeMujocoFromStates(model, data);
    data->time = static_cast<double>(CurrentSimNanos) * NANO2SEC;
    this->publishOutputStateMessage(CurrentSimNanos);
}

void
MJScene::registerAndBindDynamicsState(mjModel* model, mjData* data, bool topologyAlreadyFinalized)
{
    const bool requestedHighOrder = this->highOrderAttitudeIntegration;
    const std::array<size_t, 2> jointSlots = this->registerMujocoStates(model, requestedHighOrder);
    this->registerTaskModelStates();

    if (!topologyAlreadyFinalized) {
        this->massState->setState(
          Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(model->body_mass, model->nbody));
        if (this->actState) {
            this->actState->setState(Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(data->act, model->na));
        }
    }
    this->massState->setDerivative(Eigen::MatrixXd::Zero(model->nbody, 1));

    this->dynManager.finalizeStates();
    this->bindMujocoStateSegments(model, data, topologyAlreadyFinalized, requestedHighOrder, jointSlots);
    this->configureAdaptiveStateTolerances();
}

std::array<size_t, 2>
MJScene::registerMujocoStates(const mjModel* model, bool highOrderAttitude)
{
    const auto nbody = checkedMjtSizeCast<uint32_t>(model->nbody, "nbody");
    const auto na = checkedMjtSizeCast<uint32_t>(model->na, "na");
    const auto nq = checkedMjtSizeCast<size_t>(model->nq, "nq");
    constexpr size_t qposSlot = 0;
    constexpr size_t qvelSlot = 1;
    std::array<size_t, 2> jointSlots{};
    auto euclideanSpec = [](uint32_t rows, ErrorControlMode errorControl) {
        StateSpec result;
        result.state = { rows, 1 };
        result.derivative = result.state;
        result.diffusionTangent = result.state;
        result.errorControl = errorControl;
        return result;
    };

    jointSlots[qposSlot] = 0;
    for (auto&& body : this->spec.getBodies()) {
        body.registerJointPositionStates(DynParamRegisterer(this->dynManager, "body_" + body.getName() + "_"),
                                         highOrderAttitude);
    }
    jointSlots[qvelSlot] = this->dynManager.getStateRegistry().getStateCount();
    if (this->dynManager.statesAreFinalized()) {
        const auto& layouts = this->dynManager.getStateRegistry().getStateLayouts();
        jointSlots[qvelSlot] = 0;
        while (jointSlots[qvelSlot] < layouts.size() && layouts[jointSlots[qvelSlot]].stateOffset < nq) {
            ++jointSlots[qvelSlot];
        }
    }
    for (auto&& body : this->spec.getBodies()) {
        body.registerJointVelocityStates(DynParamRegisterer(this->dynManager, "body_" + body.getName() + "_"));
    }
    this->massState = this->dynManager.registerState("mujocoMass", euclideanSpec(nbody, ErrorControlMode::WholeState));
    this->actState = nullptr;
    if (na > 0) {
        this->actState = this->dynManager.registerState("mujocoAct", euclideanSpec(na, ErrorControlMode::WholeState));
    }
    return jointSlots;
}

void
MJScene::registerTaskModelStates()
{
    forEachUniqueTaskModel(this->dynamicsTask, this->dynamicsDiffusionTask, [this](SysModel* model) {
        auto* statefulModel = dynamic_cast<StatefulSysModel*>(model);
        if (statefulModel == nullptr) {
            return;
        }
        const std::string prefix = (model->ModelTag.empty() ? std::string("model") : model->ModelTag) + "_" +
                                   std::to_string(model->moduleID) + "_";
        statefulModel->registerStates(DynParamRegisterer(this->dynManager, prefix));
    });
}

void
MJScene::bindMujocoStateSegments(mjModel* model,
                                 mjData* data,
                                 bool topologyAlreadyFinalized,
                                 bool highOrderAttitude,
                                 const std::array<size_t, 2>& jointSlots)
{
    constexpr size_t qposSlot = 0;
    constexpr size_t qvelSlot = 1;
    MujocoStateSegments segments;
    if (model->nq > 0) {
        const auto nq = checkedMjtSizeCast<size_t>(model->nq, "nq");
        const auto nv = checkedMjtSizeCast<size_t>(model->nv, "nv");
        const auto& layouts = this->dynManager.getStateRegistry().getStateLayouts();
        if (jointSlots[qposSlot] >= layouts.size() || jointSlots[qvelSlot] >= layouts.size()) {
            throw std::logic_error("MuJoCo joint record boundaries are outside the state layout.");
        }
        const size_t qposOffset = layouts[jointSlots[qposSlot]].stateOffset;
        if (layouts[jointSlots[qvelSlot]].stateOffset != qposOffset + nq) {
            throw std::logic_error("MuJoCo joint records do not match the captured qpos/qvel spans.");
        }
        segments.qposState = this->dynManager.getStateRegistry().getStateSegment(jointSlots[qposSlot], nq);
        segments.qvelState = this->dynManager.getStateRegistry().getStateSegment(jointSlots[qvelSlot], nv);
        segments.qvelDerivative = this->dynManager.getStateRegistry().getDerivativeSegment(jointSlots[qvelSlot], nv);
        if (!highOrderAttitude) {
            segments.qposDerivative =
              this->dynManager.getStateRegistry().getDerivativeSegment(jointSlots[qposSlot], nv);
        }
        double* qposData = this->dynManager.getStateRegistry().stateSegmentData(segments.qposState);
        double* qvelData = this->dynManager.getStateRegistry().stateSegmentData(segments.qvelState);
        if (!topologyAlreadyFinalized) {
            std::copy_n(data->qpos, model->nq, qposData);
            std::copy_n(data->qvel, model->nv, qvelData);
        }
        for (const auto& body : this->spec.getBodies()) {
            body.validateJointStateLayout(qposData, qvelData);
        }
    }
    this->jointQposStateSegment = segments.qposState;
    this->jointQvelStateSegment = segments.qvelState;
    this->jointQposDerivativeSegment = segments.qposDerivative;
    this->jointQvelDerivativeSegment = segments.qvelDerivative;
}

void
MJScene::resetTaskModels(uint64_t currentSimNanos)
{
    forEachUniqueTaskModel(this->dynamicsTask, this->dynamicsDiffusionTask, [currentSimNanos](SysModel* model) {
        model->Reset(currentSimNanos);
    });
    this->dynamicsTask.NextStartTime = currentSimNanos;
    this->dynamicsTask.NextPickupTime = currentSimNanos + this->dynamicsTask.TaskPeriod;
    this->dynamicsDiffusionTask.NextStartTime = currentSimNanos;
    this->dynamicsDiffusionTask.NextPickupTime = currentSimNanos + this->dynamicsDiffusionTask.TaskPeriod;
}

void
MJScene::configureAdaptiveStateTolerances()
{
    auto* adaptiveIntegrator = dynamic_cast<StateVecAdaptiveIntegrator*>(this->getIntegrator());
    if (adaptiveIntegrator) {
        for (auto&& body : this->spec.getBodies()) {
            if (!body.isFree()) {
                continue;
            }
            auto& joint = body.getFreeJoint();
            adaptiveIntegrator->setRelativeTolerance(joint.getTranslationPositionState()->getName(), 0.0);
            adaptiveIntegrator->setRelativeTolerance(joint.getTranslationVelocityState()->getName(), 0.0);
        }
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
MJScene::equationsOfMotion(double t, double timeStep [[maybe_unused]])
{
    auto nanos = static_cast<uint64_t>(t * SEC2NANO);

    // Make sure the model is compiled
    this->spec.recompileUntilStable();
    ScopedFlag mutationGuard(this->sceneMutationBlocked);
    MJSpec::NoRecompileGuard recompileGuard(this->spec);

    // recompileIfNeeded() above is the only thing that can invalidate these, so cache
    // them for the rest of the call rather than re-fetching through the accessors.
    mjModel* model = this->spec.getMujocoModel();
    mjData* data = this->spec.getMujocoData();

    // Copy data from Basilisk state objects to MuJoCo structs and refresh
    // constants affected by evolving mass states.
    this->synchronizeMujocoFromStates(model, data);

    // Keep MuJoCo's internal time in sync with the Basilisk simulation time so
    // diagnostics and MuJoCo warnings report the correct timestamp.
    data->time = t;

    // On the first dynamics call, zero the CTRL array to prevent NaN/uninitialized
    // actuator commands from triggering instability at t=0.
    if (this->firstDynamicsCall) {
        for (mjtSize i = 0; i < model->nu; ++i) {
            data->ctrl[i] = 0.0;
        }
        this->firstDynamicsCall = false;
    }

    for (auto&& body : this->spec.getBodies()) {
        body.writeStateDependentOutputMessages(nanos);
    }

    // Execute the dynamics task!
    // (One of) The first module in this task should be a MJFwdKinematics
    // which will take the recently updated qpos and qvel data,
    // do the fwd kinematics, and update the state messages.
    // These messages can then be read by the rest of modules.
    this->dynamicsTask.ExecuteTaskList(nanos);

    // Refresh after task callbacks, including direct StateData writes that
    // cannot mark the scene's explicit stale flag.
    this->updateForwardKinematicsFromStates(model, data);

    // Update the ctrl array in mjData from the inputs in the actuators
    for (auto&& actuator : this->spec.getActuators()) {
        actuator->updateCtrl(data);
    }

    // Update the prescribed joints equalities
    for (auto&& body : this->spec.getBodies()) {
        body.updateConstrainedEqualityJoints();
    }

    // These methods will compute the accelerations
    mj_fwdActuation(model, data);
    mj_fwdAcceleration(model, data);
    mj_fwdConstraint(model, data);

    // Sanity check the produced accelerations
    auto qacc = data->qacc;
    if (std::any_of(qacc, qacc + model->nv, [](mjtNum v) { return std::isnan(v); })) {
        logAndThrow<std::runtime_error>("Encountered NaN acceleration at time " + std::to_string(t) +
                                        "s in MJScene with ID: " + std::to_string(moduleID));
    }

    if (model->nv > 0) {
        double* qvelDerivativeData = this->dynManager.getStateRegistry().derivativeSegmentData(this->jointQvelDerivativeSegment);
        std::copy_n(data->qacc, model->nv, qvelDerivativeData);
        if (this->modelTopology.highOrderAttitude) {
            for (auto&& body : this->spec.getBodies()) {
                body.setJointPositionDerivativesFromMujoco(data);
            }
        } else {
            double* qposDerivativeData = this->dynManager.getStateRegistry().derivativeSegmentData(this->jointQposDerivativeSegment);
            std::copy_n(data->qvel, model->nv, qposDerivativeData);
        }
    }

    // Also copy the derivative of the actuator states, if we have them
    if (model->na > 0) {
        auto actDeriv = this->actState->derivativeView();
        std::copy_n(data->act_dot, model->na, actDeriv.data());
    }

    // Update the derivative of the body mass property states (into the bulk
    // mass state, one entry per body).
    for (auto&& body : this->spec.getBodies()) {
        body.updateMassPropsDerivative();
    }
}

void
MJScene::equationsOfMotionDiffusion(double t, double timeStep [[maybe_unused]])
{
    auto nanos = static_cast<uint64_t>(t * SEC2NANO);
    this->spec.recompileUntilStable();
    ScopedFlag mutationGuard(this->sceneMutationBlocked);
    MJSpec::NoRecompileGuard recompileGuard(this->spec);

    mjModel* model = this->spec.getMujocoModel();
    mjData* data = this->spec.getMujocoData();
    this->synchronizeMujocoFromStates(model, data);
    data->time = t;

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
    this->timeBefore = static_cast<double>(callTimeNanos) * NANO2SEC;
    this->timeBeforeNanos = callTimeNanos;
    double callTime = static_cast<double>(callTimeNanos) * NANO2SEC;

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

StateData*
MJScene::getMassState()
{
    return this->massState;
}

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

bool
MJScene::modelTopologyMatches(const mjModel& candidate) const
{
    return this->modelTopology.matches(candidate);
}

MJScene::MujocoTopology
MJScene::MujocoTopology::capture(const mjModel& model, bool highOrderAttitude)
{
    MujocoTopology result;
    result.registered = true;
    result.highOrderAttitude = highOrderAttitude;
    result.nq = model.nq;
    result.nv = model.nv;
    result.na = model.na;
    result.nbody = model.nbody;
    result.jointTypes.assign(model.jnt_type, model.jnt_type + model.njnt);
    return result;
}

bool
MJScene::MujocoTopology::matches(const mjModel& model) const noexcept
{
    if (!this->registered) {
        return true;
    }
    if (model.nq != this->nq || model.nv != this->nv || model.na != this->na || model.nbody != this->nbody ||
        model.njnt != static_cast<mjtSize>(this->jointTypes.size())) {
        return false;
    }
    for (mjtSize joint = 0; joint < model.njnt; ++joint) {
        const size_t index = static_cast<size_t>(joint);
        if (model.jnt_type[joint] != this->jointTypes[index]) {
            return false;
        }
    }
    return true;
}

void
MJScene::setHighOrderAttitudeIntegration(bool enabled)
{
    if (enabled == this->highOrderAttitudeIntegration) {
        return;
    }
    this->requireMutableTopology("Changing MuJoCo attitude integration mode");
    this->highOrderAttitudeIntegration = enabled;
}

bool
MJScene::getHighOrderAttitudeIntegration() const noexcept
{
    return this->highOrderAttitudeIntegration;
}

void
MJScene::updateMujocoArraysFromStates()
{
    this->copyMujocoArraysFromStates(this->getMujocoModel(), this->getMujocoData());
}

void
MJScene::copyMujocoArraysFromStates(mjModel* model, mjData* data)
{
    if (model->nq > 0) {
        const double* qposData = this->dynManager.getStateRegistry().stateSegmentData(this->jointQposStateSegment);
        const double* qvelData = this->dynManager.getStateRegistry().stateSegmentData(this->jointQvelStateSegment);
        std::copy_n(qposData, model->nq, data->qpos);
        std::copy_n(qvelData, model->nv, data->qvel);
    }

    if (model->na > 0) {
        std::copy_n(this->actState->stateView().data(), model->na, data->act);
    }

    this->forwardKinematicsStale = true;
}

bool
MJScene::updateForwardKinematicsFromStates(mjModel* model, mjData* data)
{
    const double* qposData = model->nq > 0 ? this->dynManager.getStateRegistry().stateSegmentData(this->jointQposStateSegment) : nullptr;
    const double* qvelData = model->nv > 0 ? this->dynManager.getStateRegistry().stateSegmentData(this->jointQvelStateSegment) : nullptr;
    const bool qposChanged = model->nq > 0 && !std::equal(qposData, qposData + model->nq, data->qpos);
    const bool qvelChanged = model->nv > 0 && !std::equal(qvelData, qvelData + model->nv, data->qvel);

    if (!this->forwardKinematicsStale && !qposChanged && !qvelChanged) {
        return false;
    }

    if (qposChanged) {
        std::copy_n(qposData, model->nq, data->qpos);
    }
    if (qvelChanged) {
        std::copy_n(qvelData, model->nv, data->qvel);
    }
    mj_fwdPosition(model, data);
    mj_fwdVelocity(model, data);
    this->forwardKinematicsStale = false;
    return true;
}

void
MJScene::synchronizeMujocoFromStates(mjModel* model, mjData* data)
{
    this->validateMujocoMassStates(model);
    const auto masses = this->massState->stateView();
    for (auto&& body : this->spec.getBodies()) {
        body.applyPrevalidatedMass(model, masses(static_cast<Eigen::Index>(body.getId())));
    }
    if (this->areMujocoModelConstStale()) {
        mj_setConst(model, data);
        this->mjModelConstStale = false;
    }
    // mj_setConst restores the reference pose, so publish the retained state
    // only after mass-dependent constants are current.
    this->copyMujocoArraysFromStates(model, data);
}

void
MJScene::requireSceneMutationAllowed(const char* operation) const
{
    if (this->sceneMutationBlocked) {
        throw std::logic_error(std::string(operation) + " cannot run from an MJScene reset or dynamics callback");
    }
}

void
MJScene::validateMujocoMassStates(const mjModel* model) const
{
    const auto masses = this->massState->stateView();
    if (masses.size() != model->nbody || !std::isfinite(masses(0)) || masses(0) != 0.0) {
        throw std::invalid_argument("The MuJoCo world-body mass state must remain finite and zero.");
    }

    constexpr double massEpsilon = 10.0 * std::numeric_limits<double>::epsilon();
    for (Eigen::Index bodyId = 1; bodyId < masses.size(); ++bodyId) {
        const double oldMass = model->body_mass[bodyId];
        const double newMass = masses(bodyId);
        if (!std::isfinite(newMass) || newMass < 0.0) {
            throw std::invalid_argument("MuJoCo body mass states must be finite and nonnegative.");
        }
        if (std::abs(oldMass - newMass) > massEpsilon && (oldMass <= massEpsilon || newMass <= massEpsilon)) {
            throw std::invalid_argument("Runtime MuJoCo mass updates cannot transition a body to or "
                                        "from zero mass because no reversible inertia scaling exists.");
        }
    }
}

Eigen::VectorXd
MJScene::assembleFullQpos()
{
    auto* model = this->getMujocoModel();
    if (model->nq == 0) {
        return Eigen::VectorXd(0);
    }
    return Eigen::Map<const Eigen::VectorXd>(this->dynManager.getStateRegistry().stateSegmentData(this->jointQposStateSegment), model->nq);
}

Eigen::VectorXd
MJScene::assembleFullQvel()
{
    auto* model = this->getMujocoModel();
    if (model->nv == 0) {
        return Eigen::VectorXd(0);
    }
    return Eigen::Map<const Eigen::VectorXd>(this->dynManager.getStateRegistry().stateSegmentData(this->jointQvelStateSegment), model->nv);
}

void
MJScene::writeOutputStateMessages(uint64_t CurrentSimNanos)
{
    this->writeOutputStateMessages(
      CurrentSimNanos, this->modelTopology.nq, this->modelTopology.nv, this->modelTopology.na);
}

void
MJScene::writeOutputStateMessages(uint64_t currentSimNanos, mjtSize nq, mjtSize nv, mjtSize na)
{
    this->prepareOutputStateMessageStorage(nq, nv, na);
    this->populateOutputStateMessagePayload(nq, nv, na);
    this->publishOutputStateMessage(currentSimNanos);
}

void
MJScene::prepareOutputStateMessageStorage(mjtSize nq, mjtSize nv, mjtSize na)
{
    this->outputStateMessagePayload.qpos.resize(checkedMjtSizeCast<Eigen::Index>(nq, "nq"));
    this->outputStateMessagePayload.qvel.resize(checkedMjtSizeCast<Eigen::Index>(nv, "nv"));
    this->outputStateMessagePayload.act.resize(checkedMjtSizeCast<Eigen::Index>(na, "na"));
}

void
MJScene::populateOutputStateMessagePayload(mjtSize nq, mjtSize nv, mjtSize na)
{
    if (nq > 0) {
        std::copy_n(this->dynManager.getStateRegistry().stateSegmentData(this->jointQposStateSegment),
                    nq,
                    this->outputStateMessagePayload.qpos.data());
    }
    if (nv > 0) {
        std::copy_n(this->dynManager.getStateRegistry().stateSegmentData(this->jointQvelStateSegment),
                    nv,
                    this->outputStateMessagePayload.qvel.data());
    }
    if (na > 0) {
        std::copy_n(this->actState->stateView().data(), na, this->outputStateMessagePayload.act.data());
    }
}

void
MJScene::publishOutputStateMessage(uint64_t currentSimNanos)
{
    this->stateOutMsg.write(&this->outputStateMessagePayload, this->moduleID, currentSimNanos);
}
