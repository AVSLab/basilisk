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

    /* setup default input message names.  These can be over-riden by the user */
    this->cmdTorqueInMsgName = "extTorquePntB_B_cmds";
    this->cmdForceInertialInMsgName = "extForce_N_cmds";
    this->cmdForceBodyInMsgName = "extForce_B_cmds";
    this->cmdTorqueInMsgID = -1;
    this->cmdForceInertialInMsgID = -1;
    this->cmdForceBodyInMsgID = -1;
    this->goodTorqueCmdMsg = false;
    this->goodForceNCmdMsg = false;
    this->goodForceBCmdMsg = false;


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
    //! Begin method steps
    //! - Find the message ID associated with the InputCmds string.
    this->cmdTorqueInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cmdTorqueInMsgName,
                                                                                sizeof(CmdTorqueBodyIntMsg), moduleID);
    this->cmdForceInertialInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cmdForceInertialInMsgName,
                                                                                       sizeof(CmdForceInertialIntMsg), moduleID);
    this->cmdForceBodyInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cmdForceBodyInMsgName,
                                                                                   sizeof(CmdForceBodyIntMsg), moduleID);
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
 @param CurrentClock The current time used for time-stamping the message
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

    //! - If the input message ID is invalid, return without touching states
    if(this->cmdTorqueInMsgID >= 0)
    {
        memset(&(this->incomingCmdTorqueBuffer), 0x0, sizeof(CmdTorqueBodyIntMsg));
        this->goodTorqueCmdMsg =
            SystemMessaging::GetInstance()->ReadMessage(this->cmdTorqueInMsgID, &LocalHeader,
                                                     sizeof(CmdTorqueBodyIntMsg),
                                                     reinterpret_cast<uint8_t*> (&(this->incomingCmdTorqueBuffer)), moduleID);
    }

    //! - If the input message ID is invalid, return without touching states
    if(this->cmdForceInertialInMsgID >= 0)
    {
        memset(&(this->incomingCmdForceInertialBuffer), 0x0, sizeof(CmdForceInertialIntMsg));
        this->goodForceNCmdMsg =
            SystemMessaging::GetInstance()->ReadMessage(this->cmdForceInertialInMsgID, &LocalHeader,
                                                     sizeof(CmdForceInertialIntMsg),
                                                     reinterpret_cast<uint8_t*> (&(this->incomingCmdForceInertialBuffer)), moduleID);
    }

    //! - If the input message ID is invalid, return without touching states
    if(this->cmdForceBodyInMsgID >= 0)
    {
        memset(&(this->incomingCmdForceBodyBuffer), 0x0, sizeof(CmdForceBodyIntMsg));
        this->goodForceBCmdMsg =
            SystemMessaging::GetInstance()->ReadMessage(this->cmdForceBodyInMsgID, &LocalHeader,
                                                     sizeof(CmdForceBodyIntMsg),
                                                     reinterpret_cast<uint8_t*> (&(this->incomingCmdForceBodyBuffer)), moduleID);
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
    //! Begin method steps


    /* add the cmd force in inertial frame components set via Python */
    this->forceExternal_N = this->extForce_N;
    /* add the cmd force in inertial frame components set via FSW communication */
    if (this->goodForceNCmdMsg) {
		cmdVec = cArray2EigenVector3d(this->incomingCmdForceInertialBuffer.forceRequestInertial);
		this->forceExternal_N += cmdVec;
    }

    /* add the cmd force in body frame components set via Python */
    this->forceExternal_B = this->extForce_B;
    /* add the cmd force in body frame components set via FSW communication */
    if (this->goodForceBCmdMsg) {
		cmdVec = cArray2EigenVector3d(this->incomingCmdForceBodyBuffer.forceRequestBody);
        this->forceExternal_B +=cmdVec;
    }

    /* add the cmd torque about Point B in body frame components set via Python */
    this->torqueExternalPntB_B = this->extTorquePntB_B;
    /* add the cmd torque about Point B in body frame components set via FSW communication */
    if (this->goodTorqueCmdMsg) {
		cmdVec = cArray2EigenVector3d(this->incomingCmdTorqueBuffer.torqueRequestBody);
        this->torqueExternalPntB_B += cmdVec;
    }

    return;
}

void ExtForceTorque::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
}
