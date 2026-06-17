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

void MJScalarJoint::writeJointStateMessage(uint64_t CurrentSimNanos)
{
    checkInitialized();

    auto& scene = body.getSpec().getScene();

    ScalarJointStateMsgPayload stateOutMsgPayload;
    stateOutMsgPayload.state = scene.getQposState()->state(this->qposAdr.value());
    this->stateOutMsg.write(&stateOutMsgPayload, scene.moduleID, CurrentSimNanos);

    ScalarJointStateMsgPayload stateDotOutMsgPayload;
    stateDotOutMsgPayload.state = scene.getQvelState()->state(this->qvelAdr.value());
    this->stateDotOutMsg.write(&stateDotOutMsgPayload, scene.moduleID, CurrentSimNanos);
}

void MJScalarJoint::setPosition(double value)
{
    checkInitialized();
    this->body.getSpec().getScene().getQposState()->state(this->qposAdr.value()) = value;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJScalarJoint::setVelocity(double value)
{
    checkInitialized();
    this->body.getSpec().getScene().getQvelState()->state(this->qvelAdr.value()) = value;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

MJSingleJointEquality
MJScalarJoint::getConstrainedEquality()
{
    return this->constrainedEquality;
}

// ---------------------------------------------------------------------------
// MJFreeJoint
// ---------------------------------------------------------------------------

void MJFreeJoint::setPosition(const Eigen::Vector3d& position)
{
    checkInitialized();
    auto& qpos = this->body.getSpec().getScene().getQposState()->state;
    auto i = this->qposAdr.value();
    qpos.middleRows(i, 3) = position;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setVelocity(const Eigen::Vector3d& velocity)
{
    checkInitialized();
    auto& qvel = this->body.getSpec().getScene().getQvelState()->state;
    auto i = this->qvelAdr.value();
    qvel.middleRows(i, 3) = velocity;
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setAttitude(const Eigen::MRPd& attitude)
{
    checkInitialized();
    auto mat  = attitude.toRotationMatrix();
    auto quat = Eigen::Quaterniond(mat);
    auto& qpos = this->body.getSpec().getScene().getQposState()->state;
    auto i = this->qposAdr.value();
    // The free joint quaternion is stored three entries after the translation.
    qpos(i + 3) = quat.w();
    qpos(i + 4) = quat.x();
    qpos(i + 5) = quat.y();
    qpos(i + 6) = quat.z();
    this->body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setAttitudeRate(const Eigen::Vector3d& attitudeRate)
{
    checkInitialized();
    auto& qvel = this->body.getSpec().getScene().getQvelState()->state;
    auto i = this->qvelAdr.value();
    qvel.middleRows(i + 3, 3) = attitudeRate;
    this->body.getSpec().getScene().markKinematicsAsStale();
}
