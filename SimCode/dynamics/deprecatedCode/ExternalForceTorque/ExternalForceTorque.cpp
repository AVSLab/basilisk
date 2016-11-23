/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "ExternalForceTorque.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include <cstring>
#include <cmath>
#include <iostream>

/*! This is the constructor.  It sets some defaul initializers that can be
overriden by the user.*/
ExternalForceTorque::ExternalForceTorque()
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

    return;
}

/*! The destructor.  Nothing of note is performed here*/
ExternalForceTorque::~ExternalForceTorque()
{
    return;
}

/*! This method is used to setup the module 
@return void
*/
void ExternalForceTorque::SelfInit()
{
    return;
}

/*! This method is used to connect the input command message to the thrusters.
It sets the message ID based on what it finds for the input string.  If the
message is not successfully linked, it will warn the user.
@return void
*/
void ExternalForceTorque::CrossInit()
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

    return;
}

/*! This method is here to write the output message structure into the specified
message.  It is currently blank but we will certainly have an output message
soon.  If it is already here, bludgeon whoever added it and didn't fix the
comment.
@param CurrentClock The current time used for time-stamping the message
@return void
*/
void ExternalForceTorque::writeOutputMessages(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to read the incoming ephmeris message and set the
associated buffer structure.
@return void
*/
void ExternalForceTorque::readInputs()
{
    SingleMessageHeader LocalHeader;

    this->goodTorqueCmdMsg = false;
    this->goodForceNCmdMsg = false;
    this->goodForceBCmdMsg = false;

    //! - If the input message ID is invalid, return without touching states
    if(this->cmdTorqueInMsgID >= 0)
    {
        memset(&(this->incomingCmdTorqueBuffer), 0x0, sizeof(extForceTorqueCmdStruct));
        this->goodTorqueCmdMsg = SystemMessaging::GetInstance()->ReadMessage(this->cmdTorqueInMsgID, &LocalHeader,
                                                    sizeof(extForceTorqueCmdStruct),
                                                    reinterpret_cast<uint8_t*> (&(this->incomingCmdTorqueBuffer)), moduleID);
    }

    //! - If the input message ID is invalid, return without touching states
    if(this->cmdForceInertialInMsgID >= 0)
    {
        memset(&(this->incomingCmdForceInertialBuffer), 0x0, sizeof(extForceTorqueCmdStruct));
        this->goodForceNCmdMsg = SystemMessaging::GetInstance()->ReadMessage(this->cmdForceInertialInMsgID, &LocalHeader,
                                                    sizeof(extForceTorqueCmdStruct),
                                                    reinterpret_cast<uint8_t*> (&(this->incomingCmdForceInertialBuffer)), moduleID);
    }

    //! - If the input message ID is invalid, return without touching states
    if(this->cmdForceBodyInMsgID >= 0)
    {
        memset(&(this->incomingCmdForceBodyBuffer), 0x0, sizeof(extForceTorqueCmdStruct));
        this->goodForceBCmdMsg = SystemMessaging::GetInstance()->ReadMessage(this->cmdForceBodyInMsgID, &LocalHeader,
                                                    sizeof(extForceTorqueCmdStruct),
                                                    reinterpret_cast<uint8_t*> (&(this->incomingCmdForceBodyBuffer)), moduleID);
    }

    return;
}

/*! This method is used to compute all the dynamical effects.
It is an inherited method from the DynEffector class and is designed to be called
by the dynamics plant for the simulation.  It uses the thruster force magnitude
computed for the current time as well as the current vehicle state and mass
properties to get the current body force/torque which serve as the API to
dynamics
@return void
@param Props Current mass properties of the vehicle (using center of mass and str2bdy transformation
@param Bstate Current state of the vehicle (not used by thruster dynamics)
@param CurrentTime Current simulation time converted to double precision
*/
void ExternalForceTorque::ComputeDynamics(MassPropsData *massPropsData, OutputStateData *bodyState, double currentTime)
{
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

}

void ExternalForceTorque::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputs();
}

