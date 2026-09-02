/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/dynamics/hingedRigidBodyMotor/hingedRigidBodyMotor.h"
#include <algorithm>

/** @brief Constructs a hinged rigid body motor controller with torque saturation disabled. */
HingedRigidBodyMotor::HingedRigidBodyMotor()
{
    this->K = 0.0;  // [N m/rad]
    this->P = 0.0;  // [N m s/rad]
    this->uMax = -1.0;  // [N m] negative values disable torque saturation
}

/*! Module Destructor */
HingedRigidBodyMotor::~HingedRigidBodyMotor()
{
}

/** @brief Checks that required input messages and controller gains are configured.
 * @param CurrentSimNanos [ns] Current simulation time; unused during reset.
 */
void HingedRigidBodyMotor::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    //! check that required input messages are connected
    if (!this->hingedBodyStateSensedInMsg.isLinked()) {
        bskLogger.bskError("HingedRigidBodyMotor.hingedBodyStateSensedInMsg was not linked.");
    }
    if (!this->hingedBodyStateReferenceInMsg.isLinked()) {
        bskLogger.bskError("HingedRigidBodyMotor.hingedBodyStateReferenceInMsg was not linked.");
    }
    if (this->K <= 0.0 || this->P <= 0.0) {
        bskLogger.bskError("HingedRigidBodyMotor K and P must be set to positive values.");
    }

}


/** @brief Calculates and limits the commanded hinge torque.
 * @param CurrentSimNanos [ns] Current simulation time.
 */
void HingedRigidBodyMotor::UpdateState(uint64_t CurrentSimNanos)
{
    //! local variables
    double sensedTheta;  // [rad]
    double sensedThetaDot;  // [rad/s]
    double refTheta;  // [rad]
    double refThetaDot;  // [rad/s]
    double torque;  // [N m]

    HingedRigidBodyMsgPayload hingedBodyStateSensedInMsgBuffer;  //!< local copy of message buffer for reference
    HingedRigidBodyMsgPayload hingedBodyStateReferenceInMsgBuffer;  //!< local copy of message buffer for measurement
    ArrayMotorTorqueMsgPayload motorTorqueOutMsgBuffer;  //!< local copy of message buffer for motor torque

    //! zero the output message buffers before assigning values
    motorTorqueOutMsgBuffer = this->motorTorqueOutMsg.zeroMsgPayload;

    //! read in the input messages
    hingedBodyStateSensedInMsgBuffer = this->hingedBodyStateSensedInMsg();
    hingedBodyStateReferenceInMsgBuffer = this->hingedBodyStateReferenceInMsg();

    sensedTheta = hingedBodyStateSensedInMsgBuffer.theta;
    sensedThetaDot = hingedBodyStateSensedInMsgBuffer.thetaDot;

    refTheta = hingedBodyStateReferenceInMsgBuffer.theta;
    refThetaDot = hingedBodyStateReferenceInMsgBuffer.thetaDot;

    //! calculate motor torque
    torque = this->K * (refTheta - sensedTheta) + this->P * (refThetaDot - sensedThetaDot);
    if (this->uMax >= 0.0) {
        torque = std::clamp(torque, -this->uMax, this->uMax);
    }
    motorTorqueOutMsgBuffer.motorTorque[0] = torque;

    //! write to the output messages
    this->motorTorqueOutMsg.write(&motorTorqueOutMsgBuffer, this->moduleID, CurrentSimNanos);
}
