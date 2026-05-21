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
#include "MJScene.h"
#include "MJSpec.h"

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
} // namespace

void MJJoint::configure(const mjModel* m)
{
    MJObject::configure(m);

    this->qposAdr = m->jnt_qposadr[this->getId()];
    this->qvelAdr = m->jnt_dofadr[this->getId()];
}

void MJJoint::checkInitialized() const
{
    if (!this->qposAdr.has_value()) {
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
{}

Eigen::Vector3d MJScalarJoint::getAxis() const
{
    checkInitialized();
    const auto m = this->body.getSpec().getMujocoModel();
    return Eigen::Vector3d(m->jnt_axis + (this->qposAdr.value() * 3));
}

bool MJScalarJoint::isHinge() const
{
    checkInitialized();
    const auto m = this->body.getSpec().getMujocoModel();
    return m->jnt_type[this->qposAdr.value()] == mjJNT_HINGE;
}

void MJScalarJoint::configure(const mjModel* m)
{
    MJJoint::configure(m);
    this->constrainedEquality.configure(m);
}

void MJScalarJoint::updateConstrainedEquality()
{
    bool useConstraint = this->constrainedStateInMsg.isLinked();
    this->constrainedEquality.setActive(useConstraint);
    if (useConstraint) {
        this->constrainedEquality.setJointOffsetConstraint(this->constrainedStateInMsg().state);
    }
}

void MJScalarJoint::registerStates(DynParamRegisterer registerer)
{
    this->qposState = registerer.registerState(1, 1, "joint_" + this->name + "_qpos");
    this->qvelState = registerer.registerState(1, 1, "joint_" + this->name + "_qvel");
}

void MJScalarJoint::setStateInMujoco(mjData* d) const
{
    d->qpos[this->qposAdr.value()] = this->qposState->state(0);
    d->qvel[this->qvelAdr.value()] = this->qvelState->state(0);
}

void MJScalarJoint::getStateFromMujoco(const mjData* d)
{
    this->qposState->state(0) = d->qpos[this->qposAdr.value()];
    this->qvelState->state(0) = d->qvel[this->qvelAdr.value()];
}

void MJScalarJoint::setDerivativesFromMujoco(const mjData* d)
{
    Eigen::Matrix<double, 1, 1> dPos;
    dPos(0) = d->qvel[this->qvelAdr.value()];
    this->qposState->setDerivative(dPos);

    Eigen::Matrix<double, 1, 1> dVel;
    dVel(0) = d->qacc[this->qvelAdr.value()];
    this->qvelState->setDerivative(dVel);
}

void MJScalarJoint::writeJointStateMessage(uint64_t CurrentSimNanos)
{
    checkInitialized();

    auto& scene = body.getSpec().getScene();

    ScalarJointStateMsgPayload stateOutMsgPayload;
    stateOutMsgPayload.state = this->qposState->state(0);
    this->stateOutMsg.write(&stateOutMsgPayload, scene.moduleID, CurrentSimNanos);

    ScalarJointStateMsgPayload stateDotOutMsgPayload;
    stateDotOutMsgPayload.state = this->qvelState->state(0);
    this->stateDotOutMsg.write(&stateDotOutMsgPayload, scene.moduleID, CurrentSimNanos);
}

void MJScalarJoint::setPosition(double value)
{
    checkInitialized();
    this->qposState->state(0) = value;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJScalarJoint::setVelocity(double value)
{
    checkInitialized();
    this->qvelState->state(0) = value;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

MJSingleJointEquality
MJScalarJoint::getConstrainedEquality()
{
    return this->constrainedEquality;
}

// ---------------------------------------------------------------------------
// MJBallJoint
// ---------------------------------------------------------------------------

void MJBallJoint::registerStates(DynParamRegisterer registerer)
{
    this->qposState = registerer.registerState<QuaternionStateData>(
        4, 1, "joint_" + this->name + "_qpos");
    this->qvelState = registerer.registerState(3, 1, "joint_" + this->name + "_qvel");
}

void MJBallJoint::setStateInMujoco(mjData* d) const
{
    auto qp = this->qposAdr.value();
    auto qv = this->qvelAdr.value();
    for (int k = 0; k < 4; ++k) d->qpos[qp + k] = this->qposState->state(k);
    for (int k = 0; k < 3; ++k) d->qvel[qv + k] = this->qvelState->state(k);
}

void MJBallJoint::getStateFromMujoco(const mjData* d)
{
    auto qp = this->qposAdr.value();
    auto qv = this->qvelAdr.value();
    for (int k = 0; k < 4; ++k) this->qposState->state(k) = d->qpos[qp + k];
    for (int k = 0; k < 3; ++k) this->qvelState->state(k) = d->qvel[qv + k];
}

void MJBallJoint::setDerivativesFromMujoco(const mjData* d)
{
    auto qv = this->qvelAdr.value();
    // qpos integrator consumes body angular velocity (3 elements).
    Eigen::Matrix<double, 3, 1> omega;
    for (int k = 0; k < 3; ++k) omega(k) = d->qvel[qv + k];
    this->qposState->setDerivative(omega);

    Eigen::Matrix<double, 3, 1> alpha;
    for (int k = 0; k < 3; ++k) alpha(k) = d->qacc[qv + k];
    this->qvelState->setDerivative(alpha);
}

// ---------------------------------------------------------------------------
// MJFreeJoint
// ---------------------------------------------------------------------------

void MJFreeJoint::registerStates(DynParamRegisterer registerer)
{
    this->qposTranslationState = registerer.registerState(
        3, 1, "joint_" + this->name + "_qposTranslation");
    this->qposAttitudeState    = registerer.registerState<QuaternionStateData>(
        4, 1, "joint_" + this->name + "_qposAttitude");
    this->qvelTranslationState = registerer.registerState(
        3, 1, "joint_" + this->name + "_qvelTranslation");
    this->qvelAttitudeState    = registerer.registerState(
        3, 1, "joint_" + this->name + "_qvelAttitude");
}

void MJFreeJoint::setStateInMujoco(mjData* d) const
{
    auto qp = this->qposAdr.value();
    auto qv = this->qvelAdr.value();
    for (int k = 0; k < 3; ++k) d->qpos[qp + k]     = this->qposTranslationState->state(k);
    for (int k = 0; k < 4; ++k) d->qpos[qp + 3 + k] = this->qposAttitudeState->state(k);
    for (int k = 0; k < 3; ++k) d->qvel[qv + k]     = this->qvelTranslationState->state(k);
    for (int k = 0; k < 3; ++k) d->qvel[qv + 3 + k] = this->qvelAttitudeState->state(k);
}

void MJFreeJoint::getStateFromMujoco(const mjData* d)
{
    auto qp = this->qposAdr.value();
    auto qv = this->qvelAdr.value();
    for (int k = 0; k < 3; ++k) this->qposTranslationState->state(k) = d->qpos[qp + k];
    for (int k = 0; k < 4; ++k) this->qposAttitudeState->state(k)    = d->qpos[qp + 3 + k];
    for (int k = 0; k < 3; ++k) this->qvelTranslationState->state(k) = d->qvel[qv + k];
    for (int k = 0; k < 3; ++k) this->qvelAttitudeState->state(k)    = d->qvel[qv + 3 + k];
}

void MJFreeJoint::setDerivativesFromMujoco(const mjData* d)
{
    auto qv = this->qvelAdr.value();

    // Translation qpos derivative is the inertial translational velocity.
    Eigen::Matrix<double, 3, 1> dPosTran;
    for (int k = 0; k < 3; ++k) dPosTran(k) = d->qvel[qv + k];
    this->qposTranslationState->setDerivative(dPosTran);

    // Attitude qpos derivative is the body angular velocity (consumed by
    // QuaternionStateData::propagateState as omega).
    Eigen::Matrix<double, 3, 1> omega;
    for (int k = 0; k < 3; ++k) omega(k) = d->qvel[qv + 3 + k];
    this->qposAttitudeState->setDerivative(omega);

    Eigen::Matrix<double, 3, 1> dVelTran;
    for (int k = 0; k < 3; ++k) dVelTran(k) = d->qacc[qv + k];
    this->qvelTranslationState->setDerivative(dVelTran);

    Eigen::Matrix<double, 3, 1> dVelAtt;
    for (int k = 0; k < 3; ++k) dVelAtt(k) = d->qacc[qv + 3 + k];
    this->qvelAttitudeState->setDerivative(dVelAtt);
}

void MJFreeJoint::setPosition(const Eigen::Vector3d& position)
{
    checkInitialized();
    for (int k = 0; k < 3; ++k) this->qposTranslationState->state(k) = position[k];
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setVelocity(const Eigen::Vector3d& velocity)
{
    checkInitialized();
    for (int k = 0; k < 3; ++k) this->qvelTranslationState->state(k) = velocity[k];
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setAttitude(const Eigen::MRPd& attitude)
{
    checkInitialized();
    auto mat  = attitude.toRotationMatrix();
    auto quat = Eigen::Quaterniond(mat);
    this->qposAttitudeState->state(0) = quat.w();
    this->qposAttitudeState->state(1) = quat.x();
    this->qposAttitudeState->state(2) = quat.y();
    this->qposAttitudeState->state(3) = quat.z();
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setAttitudeRate(const Eigen::Vector3d& attitudeRate)
{
    checkInitialized();
    for (int k = 0; k < 3; ++k) this->qvelAttitudeState->state(k) = attitudeRate[k];
    this->body.getSpec().getScene().markKinematicsAsStale();
}
