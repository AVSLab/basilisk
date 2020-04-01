/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "dynamics/extForceTorque/extForceTorque.h"
#include "architecture/messaging/system_messaging.h"
#include <iostream>
#include "utilities/avsEigenSupport.h"


/*! This is the constructor.  It sets some default initializers that can be
 overriden by the user.*/
ExtForceTorque::ExtForceTorque()
{
    /* initialize the 3 output vectors to zero */
    this->extForce_N.fill(0.0);
    this->extForce_B.fill(0.0);
    this->extTorquePntB_B.fill(0.0);

    CallCounts = 0;
    return;
}

/*! The destructor.  Nothing of note is performed here*/
ExtForceTorque::~ExtForceTorque()
{
    return;
}

/*! No Action is performed in this function.
 @return void
 */
void ExtForceTorque::SelfInit()
{
    return;
}

/*! This method is used to connect the input message.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void ExtForceTorque::CrossInit()
{
    /* zero the input message vectors */
    memset(&(this->incomingCmdTorqueBuffer.torqueRequestBody), 0x0, 3*sizeof(double));
    memset(&(this->incomingCmdForceInertialBuffer.forceRequestInertial), 0x0, 3*sizeof(double));
    memset(&(this->incomingCmdForceBodyBuffer.forceRequestBody), 0x0, 3*sizeof(double));

    return;
}

void ExtForceTorque::linkInStates(DynParamManager& statesIn)
{

}


/*! This module does not write any output messages.
 @param currentClock The current time used for time-stamping the message
 @return void
 */
void ExtForceTorque::writeOutputMessages(uint64_t currentClock)
{

}

/*! This method is used to read the incoming message and set the
 associated buffer structure.
 @return void
 */
void ExtForceTorque::readInputMessages()
{
    SingleMessageHeader LocalHeader;

    if(this->readCmdTorque.linked()){
        this->incomingCmdTorqueBuffer = this->readCmdTorque();
    }
    if(this->readBodyForce.linked()){
        this->incomingCmdForceBodyBuffer = this->readBodyForce();
    }
    if(this->readInertialForce.linked()){
        this->incomingCmdForceInertialBuffer = this->readInertialForce();
    }

}

/*! This method is used to compute the RHS forces and torques.
    Note:   the module can set any of these three vecors, or a subset.  Regarding the external force, the
            matrix representations in the body (B) and inerial (N) frame components are treated as 2
            separate vectors.  Only set both if you mean to, as both vectors will be included.
 */
void ExtForceTorque::computeForceTorque(double integTime)
{
	Eigen::Vector3d cmdVec;

    /* add the cmd force in inertial frame components set via Python */
    this->forceExternal_N = this->extForce_N;
    /* add the cmd force in inertial frame components set via FSW communication */
    if(this->readInertialForce.linked()){
		cmdVec = cArray2EigenVector3d(this->incomingCmdForceInertialBuffer.forceRequestInertial);
		this->forceExternal_N += cmdVec;
    }

    /* add the cmd force in body frame components set via Python */
    this->forceExternal_B = this->extForce_B;
    /* add the cmd force in body frame components set via FSW communication */
    if(this->readBodyForce.linked()){
		cmdVec = cArray2EigenVector3d(this->incomingCmdForceBodyBuffer.forceRequestBody);
        this->forceExternal_B +=cmdVec;
    }

    /* add the cmd torque about Point B in body frame components set via Python */
    this->torqueExternalPntB_B = this->extTorquePntB_B;
    /* add the cmd torque about Point B in body frame components set via FSW communication */
    if(this->readCmdTorque.linked()){
		cmdVec = cArray2EigenVector3d(this->incomingCmdTorqueBuffer.torqueRequestBody);
        this->torqueExternalPntB_B += cmdVec;
    }

    return;
}

void ExtForceTorque::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
}
