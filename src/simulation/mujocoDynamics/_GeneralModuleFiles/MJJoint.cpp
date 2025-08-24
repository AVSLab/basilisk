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
    mjs_setString(mjsequality->name, eqName.c_str());

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

void MJJoint::checkInitialized()
{
    if (!this->qposAdr.has_value()) {
        return body.getSpec().getScene().logAndThrow<std::runtime_error>(
            "Tried to manipulate joint state before the joint was configured.");
    }
}

MJScalarJoint::MJScalarJoint(mjsJoint* joint, MJBody& body)
    : MJJoint(joint, body),
      constrainedEquality(
        createConstrainedEquality(name, body.getSpec().getMujocoSpec()),
        body.getSpec()
    )
{}

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

    ScalarJointStateMsgPayload stateOutMsgPayload;
    auto state = body.getSpec().getScene().getQposState()->getState();
    stateOutMsgPayload.state = state(this->qposAdr.value());
    this->stateOutMsg.write(&stateOutMsgPayload, body.getSpec().getScene().moduleID, 0);

    ScalarJointStateMsgPayload stateDotOutMsgPayload;
    auto stateDot = body.getSpec().getScene().getQvelState()->getState();
    stateDotOutMsgPayload.state = stateDot(this->qvelAdr.value());
    this->stateDotOutMsg.write(&stateDotOutMsgPayload, body.getSpec().getScene().moduleID, 0);
}

void MJScalarJoint::setPosition(double value)
{
    checkInitialized();

    body.getSpec().getScene().getQposState()->state(this->qposAdr.value()) = value;
    body.getSpec().getScene().markKinematicsAsStale();
}

void MJScalarJoint::setVelocity(double value)
{
    checkInitialized();

    body.getSpec().getScene().getQvelState()->state(this->qvelAdr.value()) = value;
    body.getSpec().getScene().markKinematicsAsStale();
}

MJSingleJointEquality
MJScalarJoint::getConstrainedEquality()
{
    return this->constrainedEquality;
}

void MJFreeJoint::setPosition(const Eigen::Vector3d& position)
{
    checkInitialized();

    auto& qposState = body.getSpec().getScene().getQposState()->state;
    auto i = this->qposAdr.value();
    qposState.middleRows(i, 3) = position;

    body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setVelocity(const Eigen::Vector3d& velocity)
{
    checkInitialized();

    auto& qvelState = body.getSpec().getScene().getQvelState()->state;
    auto i = this->qvelAdr.value();
    qvelState.middleRows(i, 3) = velocity;

    body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setAttitude(const Eigen::MRPd& attitude)
{
    checkInitialized();

    auto& qposState = body.getSpec().getScene().getQposState()->state;
    auto i = this->qposAdr.value();

    auto mat = attitude.toRotationMatrix();
    auto quat = Eigen::Quaterniond(mat);

    qposState.middleRows(i + 3, 4) = Eigen::Vector4d{quat.w(), quat.x(), quat.y(), quat.z()};
    body.getSpec().getScene().markKinematicsAsStale();
}

void MJFreeJoint::setAttitudeRate(const Eigen::Vector3d& attitudeRate)
{
    checkInitialized();

    auto& qvelState = body.getSpec().getScene().getQvelState()->state;
    auto i = this->qvelAdr.value();
    qvelState.middleRows(i + 3, 3) = attitudeRate;

    body.getSpec().getScene().markKinematicsAsStale();
}
