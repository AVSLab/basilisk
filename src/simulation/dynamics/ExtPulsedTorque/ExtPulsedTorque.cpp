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
#include "simulation/dynamics/ExtPulsedTorque/ExtPulsedTorque.h"
#include <iostream>

/*! This is the constructor.  It sets some default initializers that can be
 overriden by the user.*/
ExtPulsedTorque::ExtPulsedTorque()
{
    this->c = 0;

    CallCounts = 0;
    return;
}

/*! The destructor.  Nothing of note is performed here*/
ExtPulsedTorque::~ExtPulsedTorque()
{
    return;
}


/*! link the states */
void ExtPulsedTorque::linkInStates(DynParamManager& statesIn)
{
    return;
}


/*! This module does not write any output messages.
 @param currentClock The current time used for time-stamping the message

 */
void ExtPulsedTorque::writeOutputMessages(uint64_t currentClock)
{
    return;
}

/*! This method is used to read the incoming message and set the
 associated buffer structure.

 */
void ExtPulsedTorque::readInputMessages()
{
    return;
}

/*! This method is used to compute the RHS forces and torques.
    Note:   the module can set any of these three vecors, or a subset.  Regarding the external force, the
            matrix represnetations in the body (B) and inerial (N) frame components are treated as 2
            separate vectors.  Only set both if you mean to, as both vectors will be included.
 */
void ExtPulsedTorque::computeForceTorque(double integTime, double timeStep)
{
    /* zero the output vector */
    this->torqueExternalPntB_B.fill(0.0);

    /* check if the pulse sequence must restart */
    if (this->c >= this->countOnPulse*2 + this->countOff) {
        this->c = 0;
    }

    if (this->c < this->countOnPulse) {
        this->torqueExternalPntB_B += this->pulsedTorqueExternalPntB_B;
    } else if (this->c < this->countOnPulse*2) {
        this->torqueExternalPntB_B -= this->pulsedTorqueExternalPntB_B;
    }
    this->c++;


    return;
}

/*! Module update method
 */
void ExtPulsedTorque::UpdateState(uint64_t CurrentSimNanos)
{
    return;
}
