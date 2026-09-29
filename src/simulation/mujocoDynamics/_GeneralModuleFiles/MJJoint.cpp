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

#include "MJJoint.h"

#include "MJBody.h"
#include "MJQuaternionStatePolicy.h"
#include "MJScene.h"
#include "MJSpec.h"

#include <algorithm>
#include <memory>

namespace
{
mjsEquality* createConstrainedEquality(const std::string& jointName,
                                                   mjSpec* spec)
{
    auto mjsequality = mjs_addEquality(spec, 0);

    std::string eqName = "_basilisk_constrainedEquality_" + jointName;
    MJBasilisk::detail::setSpecObjectName(mjsequality, eqName);

    mjs_setString(mjsequality->name1, jointName.c_str());
    mjsequality->type = mjEQ_JOINT;
    mjsequality->active = false;
    for (auto i = 0; i < mjNEQDATA; i++)
    {
        mjsequality->data[i] = 0;
    }

    return mjsequality;
}

StateSpec
euclideanVectorSpec(uint32_t rows)
{
    StateSpec spec;
    spec.state = { rows, 1 };
    spec.derivative = spec.state;
    spec.diffusionTangent = spec.state;
    spec.errorControl = ErrorControlMode::PerComponent;
    return spec;
}

StateData*
registerQuaternionState(DynParamRegisterer registerer, const std::string& name, bool highOrder)
{
    StateSpec spec;
    spec.state = { 4, 1 };
    spec.derivative = { highOrder ? 4U : 3U, 1 };
    spec.diffusionTangent = { 3, 1 };
    spec.errorControl = ErrorControlMode::PerComponent;
    spec.updateKind = StateUpdateKind::Special;
    if (highOrder) {
        return registerer.registerState(name, spec, std::make_unique<MJHighOrderQuaternionStatePolicy>());
    }
    return registerer.registerState(name, spec, std::make_unique<MJNativeQuaternionStatePolicy>());
}

Eigen::Matrix<double, 4, 1>
quaternionRate(ConstMatrixView quaternion, const double* angularVelocity)
{
    const double w = quaternion(0);
    const double x = quaternion(1);
    const double y = quaternion(2);
    const double z = quaternion(3);
    const double wx = angularVelocity[0];
    const double wy = angularVelocity[1];
    const double wz = angularVelocity[2];

    Eigen::Matrix<double, 4, 1> result;
    result(0) = 0.5 * (-x * wx - y * wy - z * wz);
    result(1) = 0.5 * (w * wx + y * wz - z * wy);
    result(2) = 0.5 * (w * wy - x * wz + z * wx);
    result(3) = 0.5 * (w * wz + x * wy - y * wx);
    return result;
}

void
setQuaternionDerivative(StateData& quaternion, const double* angularVelocity)
{
    auto derivative = quaternion.derivativeView();
    if (derivative.rows() == 3) {
        std::copy_n(angularVelocity, 3, derivative.data());
    } else {
        derivative = quaternionRate(static_cast<const StateData&>(quaternion).stateView(), angularVelocity);
    }
}
} // namespace

void
MJJoint::configure(const mjModel* model)
{
    MJObject::configure(model);
    this->qposAdr = model->jnt_qposadr[this->getId()];
    this->qvelAdr = model->jnt_dofadr[this->getId()];
}

void MJJoint::checkInitialized() const
{
    if (!this->qposAdr.has_value() || !this->statesRegistered) {
        body.getSpec().getScene().bskLogger.bskError("Tried to manipulate joint state before the joint was configured.");
    }
}

// ---------------------------------------------------------------------------
// MJScalarJoint
// ---------------------------------------------------------------------------

MJScalarJoint::MJScalarJoint(mjsJoint* joint, MJBody& body)
    : MJJoint(joint, body),
      constrainedEquality(
        createConstrainedEquality(name, body.getSpec().getMujocoSpec()),
        body.getSpec()
    )
{
    body.getSpec().markAsNeedingToRecompileModel();
}

Eigen::Vector3d MJScalarJoint::getAxis() const
{
    checkInitialized();
    const auto m = this->body.getSpec().getMujocoModel();
    return Eigen::Vector3d(m->jnt_axis + (this->getId() * 3));
}

bool MJScalarJoint::isHinge() const
{
    return this->mjsObject->type == mjJNT_HINGE;
}

void
MJScalarJoint::configure(const mjModel* model)
{
    MJJoint::configure(model);
    this->constrainedEquality.configure(model);
}

void MJScalarJoint::updateConstrainedEquality()
{
    bool useConstraint = this->constrainedStateInMsg.isLinked();
    this->constrainedEquality.setActive(useConstraint);
    if (useConstraint) {
        this->constrainedEquality.setJointOffsetConstraint(this->constrainedStateInMsg().state);
    }
}

void MJScalarJoint::writeJointStateMessage(uint64_t CurrentSimNanos)
{
    checkInitialized();

    auto& scene = body.getSpec().getScene();

    ScalarJointStateMsgPayload stateOutMsgPayload;
    stateOutMsgPayload.state = this->qposState->stateView()(0);
    this->stateOutMsg.write(&stateOutMsgPayload, scene.moduleID, CurrentSimNanos);

    ScalarJointStateMsgPayload stateDotOutMsgPayload;
    stateDotOutMsgPayload.state = this->qvelState->stateView()(0);
    this->stateDotOutMsg.write(&stateDotOutMsgPayload, scene.moduleID, CurrentSimNanos);
}

void
MJScalarJoint::registerPositionStates(DynParamRegisterer registerer, bool highOrderAttitude)
{
    (void)highOrderAttitude;
    this->qposState = registerer.registerState("joint_" + this->name + "_qpos", euclideanVectorSpec(1));
}

void
MJScalarJoint::registerVelocityStates(DynParamRegisterer registerer)
{
    this->qvelState = registerer.registerState("joint_" + this->name + "_qvel", euclideanVectorSpec(1));
    this->statesRegistered = true;
}

void
MJScalarJoint::setPositionDerivativeFromMujoco(const mjData* data)
{
    this->qposState->derivativeView()(0) = data->qvel[this->qvelAdr.value()];
}

void
MJScalarJoint::validateStateLayout(const double* qposBase, const double* qvelBase) const
{
    if (this->qposState->stateData() != qposBase + this->qposAdr.value() ||
        this->qvelState->stateData() != qvelBase + this->qvelAdr.value()) {
        throw std::logic_error("Joint-bound state layout does not match MuJoCo addresses for scalar joint '" +
                               this->name + "'.");
    }
}

void MJScalarJoint::setPosition(double value)
{
    checkInitialized();
    this->qposState->stateView()(0) = value;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJScalarJoint::setVelocity(double value)
{
    checkInitialized();
    this->qvelState->stateView()(0) = value;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

MJSingleJointEquality&
MJScalarJoint::getConstrainedEquality()
{
    return this->constrainedEquality;
}

// ---------------------------------------------------------------------------
// MJBallJoint
// ---------------------------------------------------------------------------

void
MJBallJoint::registerPositionStates(DynParamRegisterer registerer, bool highOrderAttitude)
{
    this->qposState = registerQuaternionState(registerer, "joint_" + this->name + "_qpos", highOrderAttitude);
}

void
MJBallJoint::registerVelocityStates(DynParamRegisterer registerer)
{
    this->qvelState = registerer.registerState("joint_" + this->name + "_qvel", euclideanVectorSpec(3));
    this->statesRegistered = true;
}

void
MJBallJoint::setPositionDerivativeFromMujoco(const mjData* data)
{
    const auto qvelAddress = this->qvelAdr.value();
    setQuaternionDerivative(*this->qposState, data->qvel + qvelAddress);
}

void
MJBallJoint::validateStateLayout(const double* qposBase, const double* qvelBase) const
{
    if (this->qposState->stateData() != qposBase + this->qposAdr.value() ||
        this->qvelState->stateData() != qvelBase + this->qvelAdr.value()) {
        throw std::logic_error("Joint-bound state layout does not match MuJoCo addresses for ball joint '" +
                               this->name + "'.");
    }
}

// ---------------------------------------------------------------------------
// MJFreeJoint
// ---------------------------------------------------------------------------

void
MJFreeJoint::registerPositionStates(DynParamRegisterer registerer, bool highOrderAttitude)
{
    const std::string prefix = "joint_" + this->name;
    this->qposTranslationState = registerer.registerState(prefix + "_qposTranslation", euclideanVectorSpec(3));
    this->qposAttitudeState = registerQuaternionState(registerer, prefix + "_qposAttitude", highOrderAttitude);
}

void
MJFreeJoint::registerVelocityStates(DynParamRegisterer registerer)
{
    const std::string prefix = "joint_" + this->name;
    this->qvelTranslationState = registerer.registerState(prefix + "_qvelTranslation", euclideanVectorSpec(3));
    this->qvelAttitudeState = registerer.registerState(prefix + "_qvelAttitude", euclideanVectorSpec(3));
    this->statesRegistered = true;
}

void
MJFreeJoint::setPositionDerivativeFromMujoco(const mjData* data)
{
    const auto qvelAddress = this->qvelAdr.value();
    std::copy_n(data->qvel + qvelAddress, 3, this->qposTranslationState->derivativeView().data());
    setQuaternionDerivative(*this->qposAttitudeState, data->qvel + qvelAddress + 3);
}

void
MJFreeJoint::validateStateLayout(const double* qposBase, const double* qvelBase) const
{
    const auto qposAddress = this->qposAdr.value();
    const auto qvelAddress = this->qvelAdr.value();
    if (this->qposTranslationState->stateData() != qposBase + qposAddress ||
        this->qposAttitudeState->stateData() != qposBase + qposAddress + 3 ||
        this->qvelTranslationState->stateData() != qvelBase + qvelAddress ||
        this->qvelAttitudeState->stateData() != qvelBase + qvelAddress + 3) {
        throw std::logic_error("Joint-bound state layout does not match MuJoCo addresses for free joint '" +
                               this->name + "'.");
    }
}

void MJFreeJoint::setPosition(const Eigen::Vector3d& position)
{
    checkInitialized();
    this->qposTranslationState->stateView() = position;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setVelocity(const Eigen::Vector3d& velocity)
{
    checkInitialized();
    this->qvelTranslationState->stateView() = velocity;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setAttitude(const Eigen::MRPd& attitude)
{
    checkInitialized();
    auto mat  = attitude.toRotationMatrix();
    auto quat = Eigen::Quaterniond(mat);
    auto qpos = this->qposAttitudeState->stateView();
    qpos(0) = quat.w();
    qpos(1) = quat.x();
    qpos(2) = quat.y();
    qpos(3) = quat.z();
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setAttitudeRate(const Eigen::Vector3d& attitudeRate)
{
    checkInitialized();
    this->qvelAttitudeState->stateView() = attitudeRate;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

Eigen::Vector3d MJFreeJoint::getTranslationalVelocityFromData(const mjData* data) const
{
    checkInitialized();
    auto i = this->qvelAdr.value();
    return {data->qvel[i], data->qvel[i + 1], data->qvel[i + 2]};
}

Eigen::Vector3d MJFreeJoint::getTranslationalPositionFromData(const mjData* data) const
{
    checkInitialized();
    auto i = this->qposAdr.value();
    return {data->qpos[i], data->qpos[i + 1], data->qpos[i + 2]};
}

void MJFreeJoint::setTranslationalVelocityInData(mjData* data, const Eigen::Vector3d& vel)
{
    checkInitialized();
    auto i = this->qvelAdr.value();
    data->qvel[i]     = vel[0];
    data->qvel[i + 1] = vel[1];
    data->qvel[i + 2] = vel[2];
}

void MJFreeJoint::setTranslationalPositionInData(mjData* data, const Eigen::Vector3d& pos)
{
    checkInitialized();
    auto i = this->qposAdr.value();
    data->qpos[i]     = pos[0];
    data->qpos[i + 1] = pos[1];
    data->qpos[i + 2] = pos[2];
}
