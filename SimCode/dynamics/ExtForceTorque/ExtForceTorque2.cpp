/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#include "dynamics/ExtForceTorque/extForceTorque.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
//#include <cstring>
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <cmath>

/*! This is the constructor.  It sets some defaul initializers that can be
 overriden by the user.*/
ExtForceTorque::ExtForceTorque()
{
    memset(this->extForce_N, 0x0, 3*sizeof(double));
    memset(this->extForce_B, 0x0, 3*sizeof(double));
    memset(this->extTorquePntB_B, 0x0, 3*sizeof(double));

    this->cmdTorqueInMsgName = "extTorquePntB_B_cmds";
    this->cmdForceInertialInMsgName = "extForce_N_cmds";
    this->cmdForceBodyInMsgName = "extForce_B_cmds";
    this->cmdTorqueInMsgID = -1;
    this->cmdForceInertialInMsgID = -1;
    this->cmdForceBodyInMsgID = -1;

    CallCounts = 0;
    return;
}

/*! The destructor.  Nothing of note is performed here*/
ExtForceTorque::~ExtForceTorque()
{
    return;
}

/*! This method is used to clear out the current thruster states and make sure
 that the overall model is ready for firings
 @return void
 */
void ExtForceTorque::SelfInit()
{

}

/*! This method is used to connect the input command message to the thrusters.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void ExtForceTorque::CrossInit()
{
    //! Begin method steps
    //! - Find the message ID associated with the InputCmds string.
    this->cmdTorqueInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cmdTorqueInMsgName,
                                                                                sizeof(extForceTorqueCmdStruct), moduleID);
    this->cmdForceInertialInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cmdForceInertialInMsgName,
                                                                                       sizeof(extForceTorqueCmdStruct), moduleID);
    this->cmdForceBodyInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cmdForceBodyInMsgName,
                                                                                   sizeof(extForceTorqueCmdStruct), moduleID);

    v3SetZero(this->incomingCmdTorqueBuffer.cmd);
    v3SetZero(this->incomingCmdForceInertialBuffer.cmd);
    v3SetZero(this->incomingCmdForceBodyBuffer.cmd);

}

void ExtForceTorque::linkInStates(DynParamManager& statesIn)
{

}


/*! This method is here to write the output message structure into the specified
 message.  It is currently blank but we will certainly have an output message
 soon.  If it is already here, bludgeon whoever added it and didn't fix the
 comment.
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

    this->goodTorqueCmdMsg = false;
    this->goodForceNCmdMsg = false;
    this->goodForceBCmdMsg = false;

    //! - If the input message ID is invalid, return without touching states
    if(this->cmdTorqueInMsgID >= 0)
    {
        memset(&(this->incomingCmdTorqueBuffer), 0x0, sizeof(extForceTorqueCmdStruct));
        this->goodTorqueCmdMsg =
            SystemMessaging::GetInstance()->ReadMessage(this->cmdTorqueInMsgID, &LocalHeader,
                                                     sizeof(extForceTorqueCmdStruct),
                                                     reinterpret_cast<uint8_t*> (&(this->incomingCmdTorqueBuffer)), moduleID);
    }

    //! - If the input message ID is invalid, return without touching states
    if(this->cmdForceInertialInMsgID >= 0)
    {
        memset(&(this->incomingCmdForceInertialBuffer), 0x0, sizeof(extForceTorqueCmdStruct));
        this->goodForceNCmdMsg =
            SystemMessaging::GetInstance()->ReadMessage(this->cmdForceInertialInMsgID, &LocalHeader,
                                                     sizeof(extForceTorqueCmdStruct),
                                                     reinterpret_cast<uint8_t*> (&(this->incomingCmdForceInertialBuffer)), moduleID);
    }

    //! - If the input message ID is invalid, return without touching states
    if(this->cmdForceBodyInMsgID >= 0)
    {
        memset(&(this->incomingCmdForceBodyBuffer), 0x0, sizeof(extForceTorqueCmdStruct));
        this->goodForceBCmdMsg =
            SystemMessaging::GetInstance()->ReadMessage(this->cmdForceBodyInMsgID, &LocalHeader,
                                                     sizeof(extForceTorqueCmdStruct),
                                                     reinterpret_cast<uint8_t*> (&(this->incomingCmdForceBodyBuffer)), moduleID);
    }
    

}

/*! This method is used to compute the RHS forces and torques.
 */
void ExtForceTorque::computeBodyForceTorque()
{
    //! Begin method steps
    Eigen::Vector3d tmpForce_B(0, 0, 0);
    Eigen::Vector3d tmpForce_N(0, 0, 0);
    Eigen::Vector3d tmpTorquePntB_B(0, 0, 0);

    double dynEffectorForce_N[3];
    double dynEffectorForce_B[3];
    double dynEffectorTorquePntB_B[3];

    /* add the cmd force in inertial frame components set via Python */
    v3Copy(this->extForce_N, dynEffectorForce_N);
    /* add the cmd force in inertial frame components set via FSW communication */
    if (this->goodForceNCmdMsg) {
        v3Add(this->incomingCmdForceInertialBuffer.cmd, dynEffectorForce_N, dynEffectorForce_N);
    }

    /* add the cmd force in body frame components set via Python */
    v3Copy(this->extForce_B, dynEffectorForce_B);
    /* add the cmd force in body frame components set via FSW communication */
    if (this->goodForceBCmdMsg) {
        v3Add(this->incomingCmdForceBodyBuffer.cmd, dynEffectorForce_B, dynEffectorForce_B);
    }

    /* add the cmd torque about Point B in body frame components set via Python */
    v3Copy(this->extTorquePntB_B, dynEffectorTorquePntB_B);
    /* add the cmd torque about Point B in body frame components set via FSW communication */
    if (this->goodTorqueCmdMsg) {
        v3Add(this->incomingCmdTorqueBuffer.cmd, dynEffectorTorquePntB_B, dynEffectorTorquePntB_B);
    }


    this->forceExternal_N = (Eigen::Vector3d) dynEffectorForce_N;
    this->forceExternal_B = (Eigen::Vector3d)dynEffectorForce_B;
    this->torqueExternalPntB_B = (Eigen::Vector3d) dynEffectorTorquePntB_B;
}

void ExtForceTorque::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
}


