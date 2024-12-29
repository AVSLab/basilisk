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
#include <iostream>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
HingedRigidBodyMotor::HingedRigidBodyMotor()
{
    //! zero values will lead to zero outputs
    this->K = 0.0;
    this->P = 0.0;
}

/*! Module Destructor */
HingedRigidBodyMotor::~HingedRigidBodyMotor()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.

*/
void HingedRigidBodyMotor::Reset(uint64_t CurrentSimNanos)
{
    //! check that required input messages are connected
    if (!this->hingedBodyStateSensedInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "HingedRigidBodyMotor.hingedBodyStateSensedInMsg was not linked.");
    }
    if (!this->hingedBodyStateReferenceInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "HingedRigidBodyMotor.hingedBodyStateReferenceInMsg was not linked.");
    }
    if (this->K <= 0.0 || this->P <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "HingedRigidBodyMotor K and P must be set to positive values.");
    }

}


/*! This is the main method that gets called every time the module is updated.  It calculates a motor torque on a hinged rigid body using a simple PD control law.

*/
void HingedRigidBodyMotor::UpdateState(uint64_t CurrentSimNanos)
{
    //! local variables
    double sensedTheta;
    double sensedThetaDot;
    double refTheta;
    double refThetaDot;
    double torque;

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
    torque = -1 * this->K * (sensedTheta - refTheta) - this->P * (sensedThetaDot - refThetaDot);
    motorTorqueOutMsgBuffer.motorTorque[0] = torque;

    //! write to the output messages
    this->motorTorqueOutMsg.write(&motorTorqueOutMsgBuffer, this->moduleID, CurrentSimNanos);
}
