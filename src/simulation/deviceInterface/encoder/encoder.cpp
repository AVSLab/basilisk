/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "simulation/deviceInterface/encoder/encoder.h"
#include <math.h>
#include "architecture/utilities/macroDefinitions.h"

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
Encoder::Encoder()
{
    this->numRW = -1;       // set the number of reaction wheels to -1 to throw a warning if not set
    this->clicksPerRotation = -1;

    return;
}

/*! Module Destructor.  */
Encoder::~Encoder()
{
    return;
}


/*! This method is used to reset the module.

 */
void Encoder::Reset(uint64_t CurrentSimNanos)
{
    // check if input message is linked
    if (!this->rwSpeedInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "encoder.rwSpeedInMsg is not linked.");
    }

    // if the number of clicks is not greater than 0, throw a warning message
    if (this->clicksPerRotation <= 0)
    {
        bskLogger.bskLog(BSK_ERROR, "encoder: number of clicks must be a positive integer.");
    }

    // if the number of reaction wheels is not greater than 0, throw a warning message
    if (this->numRW <= 0)
    {
        bskLogger.bskLog(BSK_ERROR, "encoder: number of reaction wheels must be a positive integer. It may not have been set.");
    }

    // reset the previous time
    this->prevTime = CurrentSimNanos;

    // zero the RW wheel output message buffer //
    this->rwSpeedConverted = this->rwSpeedOutMsg.zeroMsgPayload;

    // Loop through the RW to set some internal parameters to default
    for (int i = 0; i < MAX_EFF_CNT; i++)
    {
        // set all reaction wheels signal to nominal
        this->rwSignalState[i] = SIGNAL_NOMINAL;
        // set the remaining clicks to zero
        this->remainingClicks[i] = 0.0;
    }

    return;
}

/*! This method reads the speed input message
 */
void Encoder::readInputMessages()
{
    // read the incoming power message
    this->rwSpeedBuffer = this->rwSpeedInMsg();

    return;
}

/*! This method writes encoded the wheel speed message.

 @param CurrentClock The clock time associated with the model call
 */
void Encoder::writeOutputMessages(uint64_t CurrentClock)
{
    this->rwSpeedOutMsg.write(&this->rwSpeedConverted, this->moduleID, CurrentClock);

    return;
}

/*! This method applies an encoder to the reaction wheel speeds.
*/
void Encoder::encode(uint64_t CurrentSimNanos)
{
    double timeStep;
    double numberClicks;
    double clicksPerRadian;
    double angle;

    // convert clicks per rotation to clicks per radian
    clicksPerRadian = this->clicksPerRotation / (2 * M_PI);

    // set the time step
    timeStep = diffNanoToSec(CurrentSimNanos, this->prevTime);

    // at the beginning of the simulation, the encoder simply outputs the true RW speeds
    if (timeStep == 0.0)
    {
        this->rwSpeedConverted = this->rwSpeedInMsg();
    }
    else
    {
        // loop through the RW
        for (int i = 0; i < this->numRW; i++)
        {
            // check if encoder is operational
            if (this->rwSignalState[i] == SIGNAL_NOMINAL)
            {
                // calculate the angle sweeped by the reaction wheel during the time step
                angle = this->rwSpeedBuffer.wheelSpeeds[i] * timeStep;

                // calculate the number of clicks
                numberClicks = trunc(angle * clicksPerRadian + this->remainingClicks[i]);

                // update the remaining clicks
                this->remainingClicks[i] = angle * clicksPerRadian + this->remainingClicks[i] - numberClicks;

                // calculate the discretized angular velocity
                this->rwSpeedConverted.wheelSpeeds[i] = numberClicks / (clicksPerRadian * timeStep);
            }
            // check if encoder is off
            else if (this->rwSignalState[i] == SIGNAL_OFF)
            {
                // set the outgoing reaction wheel speed to 0
                this->rwSpeedConverted.wheelSpeeds[i] = 0.0;

                // reset the remaining clicks
                this->remainingClicks[i] = 0;
            } else if (this->rwSignalState[i] == SIGNAL_STUCK) {
                // if the encoder is stuck, it will output the previous results
            } else {
                bskLogger.bskLog(BSK_ERROR, "encoder: un-modeled encoder signal mode %d selected.", this->rwSignalState[i]);
            }
        }
    }
    return;
}

/*! This method runs the encoder module in the sim.
*/
void Encoder::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->encode(CurrentSimNanos);
    this->writeOutputMessages(CurrentSimNanos);

    this->prevTime = CurrentSimNanos;

    return;
}
