/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "fswAlgorithms/effectorInterfaces/thrFiringRound/thrFiringRound.h"
#include <iostream>
#include <cstring>

/*! Initialize C-wrapped output messages */
void
ThrFiringRound::SelfInit(){
    THRArrayCmdForceMsg_C_init(&this->thrForceOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
ThrFiringRound::ThrFiringRound()
{
    this->thrForce = std::vector<double>(MAX_EFF_CNT, 0.0);
    this->timeStep = 0.01;
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void ThrFiringRound::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->onTimeInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "ThrFiringRound.onTimeInMsg was not linked.");
    }

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void ThrFiringRound::UpdateState(uint64_t CurrentSimNanos)
{
    THRArrayOnTimeCmdMsgPayload onTimeIn;
    THRArrayCmdForceMsgPayload thrForceOut;

    // always zero the output message buffers before assigning values
    thrForceOut = this->thrForceOutMsg.zeroMsgPayload;

    // read in the input messages if not stale
    // calculate the number of steps to fire each thruster
    if (this->onTimeInMsg.isWritten()) {
        const uint64_t tW = this->onTimeInMsg.timeWritten();
        if (tW != this->lastWritten) {
            this->lastWritten = tW;
            // copy the input message
            onTimeIn = this->onTimeInMsg();

            // determine the number of steps to fire each thruster
            for (int i=0; i<2; i++) {
                const double timeRequest = onTimeIn.OnTimeRequest[i];
                const double stepsExact = timeRequest / this->timeStep;

                this->stepsToFire[i] = static_cast<uint32_t>(std::round(stepsExact));
                this->stepsFired[i] = 0;
            }
        }
    }

    // determine the thrust for each thruster and update the steps fired
    for (int i=0; i<2; i++) {
        if (this->stepsFired[i] < this->stepsToFire[i]) {
            thrForceOut.thrForce[i] = this->thrForce[i];
            this->stepsFired[i] += 1;
        } else {
            thrForceOut.thrForce[i] = 0.0;
        }
    }

    // write to the output messages
    this->thrForceOutMsg.write(&thrForceOut, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message
    THRArrayCmdForceMsg_C_write(&thrForceOut, &this->thrForceOutMsgC, this->moduleID, CurrentSimNanos);
}

void ThrFiringRound::setTHRForce(std::vector<double> var)
{
    std::copy(var.begin(), var.end(), this->thrForce.begin());
}

void ThrFiringRound::setTimeStep(double timeStep)
{
    if (timeStep <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "ThrFiringRound: timeStep must be positive");
    } else {
        this->timeStep = timeStep;
    }
}
