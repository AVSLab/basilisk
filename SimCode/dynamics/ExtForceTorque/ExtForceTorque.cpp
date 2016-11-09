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
#include "dynamics/ExtForceTorque/ExtForceTorque.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
//#include <cstring>
#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <cmath>

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
    /* zero the input message vectors */
    this->incomingCmdTorqueBuffer.cmd.fill(0.0);
    this->incomingCmdForceInertialBuffer.cmd.fill(0.0);
    this->incomingCmdForceBodyBuffer.cmd.fill(0.0);

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

    /* default the cmdMsg states to false */
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
    Note:   the module can set any of these three vecors, or a subset.  Regarding the external force, the
            matrix represnetations in the body (B) and inerial (N) frame components are treated as 2 
            separate vectors.  Only set both if you mean to, as both vectors will be included.
 */
void ExtForceTorque::computeBodyForceTorque(uint64_t currentTime)
{
    //! Begin method steps


    /* add the cmd force in inertial frame components set via Python */
    this->forceExternal_N = this->extForce_N;
    /* add the cmd force in inertial frame components set via FSW communication */
    if (this->goodForceNCmdMsg) {
        this->forceExternal_N += this->incomingCmdForceInertialBuffer.cmd;
    }

    /* add the cmd force in body frame components set via Python */
    this->forceExternal_B = this->extForce_B;
    /* add the cmd force in body frame components set via FSW communication */
    if (this->goodForceBCmdMsg) {
        this->forceExternal_B += this->incomingCmdForceBodyBuffer.cmd;
    }

    /* add the cmd torque about Point B in body frame components set via Python */
    this->torqueExternalPntB_B = this->extTorquePntB_B;
    /* add the cmd torque about Point B in body frame components set via FSW communication */
    if (this->goodTorqueCmdMsg) {
        this->torqueExternalPntB_B += this->incomingCmdTorqueBuffer.cmd;
    }


}

void ExtForceTorque::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
}


