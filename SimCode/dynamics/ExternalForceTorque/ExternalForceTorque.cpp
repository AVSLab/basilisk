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
    memset(this->force_B, 0x0, 3*sizeof(double));
    memset(this->torque_B, 0x0, 3*sizeof(double));
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
    v3Copy(this->force_B, BodyForce);
    v3Copy(this->torque_B, BodyTorque);
}

void ExternalForceTorque::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputs();
}

