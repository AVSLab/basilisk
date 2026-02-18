/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "simulation/mujocoDynamics/thrOnTimeToForce/thrOnTimeToForce.h"
#include <algorithm>
#include "architecture/utilities/macroDefinitions.h"

void ThrOnTimeToForce::Reset(uint64_t CurrentSimNanos)
{
    if (!this->onTimeInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "ThrOnTimeToForce.onTimeInMsg was not linked.");
    }

    if (this->thrMag.size() > MAX_EFF_CNT) {
        bskLogger.bskLog(BSK_ERROR,
                         "ThrOnTimeToForce configured for %zu thrusters, but THRArrayOnTimeCmdMsgPayload supports at most %d.",
                         this->thrMag.size(), MAX_EFF_CNT);
    }

    if (this->numThr != this->thrMag.size()) {
        bskLogger.bskLog(BSK_ERROR,
                         "ThrOnTimeToForce thruster count (%d) does not match size of thrMag vector (%zu).",
                         this->numThr, this->thrMag.size());
    }

    if (this->thrusterForceOutMsgs.size() != this->numThr) {
        bskLogger.bskLog(BSK_ERROR,
                         "ThrOnTimeToForce output message count (%zu) does not match thruster count (%zu).",
                         this->thrusterForceOutMsgs.size(), this->numThr);
    }

    this->firingTimeRemaining.assign(this->numThr, 0.0);
    this->previousUpdateTimeSec = CurrentSimNanos * NANO2SEC;
    this->prevCommandWriteNanos = 0xFFFFFFFFFFFFFFFF;
}

void ThrOnTimeToForce::UpdateState(uint64_t CurrentSimNanos)
{
    const double currentTimeSec = CurrentSimNanos * NANO2SEC;
    bool newOnTimeCmd = false;

    // if a new on-time command message has been written since the last update, latch the new command and reset timers
    if (this->onTimeInMsg.isLinked() && this->onTimeInMsg.isWritten() &&
        this->onTimeInMsg.timeWritten() != this->prevCommandWriteNanos)
    {
        newOnTimeCmd = true;
        THRArrayOnTimeCmdMsgPayload onTime = this->onTimeInMsg();
        this->prevCommandWriteNanos = this->onTimeInMsg.timeWritten();

        for (size_t i = 0; i < this->numThr; ++i) {
            const double cmdTime = onTime.OnTimeRequest[i];
            if (cmdTime < 0.0) {        // negative on-time commands are treated as zero with a warning
                bskLogger.bskLog(BSK_WARNING, "ThrOnTimeToForce received negative on-time command (%f s) for thruster %zu. Setting to 0.", cmdTime, i);
                this->firingTimeRemaining[i] = 0.0;
            } else {
                this->firingTimeRemaining[i] = cmdTime;
            }
        }
    }

    // if there is a new command do not decrement timers, if there is no new command, decrement timers based on time elapsed since last update
    // this is needed to avoid decrementing by large amounts if there is a gap between when the on-time command message
    // is written and the first time this module is used to process that command
    if (newOnTimeCmd || this->previousUpdateTimeSec < 0.0) {
        this->previousUpdateTimeSec = currentTimeSec;
    } else {
        const double dt = std::max(0.0, currentTimeSec - this->previousUpdateTimeSec);
        this->previousUpdateTimeSec = currentTimeSec;
        for (double &timeRemaining : this->firingTimeRemaining) {
            timeRemaining = std::max(0.0, timeRemaining - dt);
            if (timeRemaining < 1e-12) {     // to avoid numerical issues with very small remaining times, treat as zero
                timeRemaining = 0.0;
            }
        }
    }

    // write output messages based on remaining firing time
    for (size_t i = 0; i < this->numThr; ++i) {
        SingleActuatorMsgPayload forceOut = this->thrusterForceOutMsgs[i]->zeroMsgPayload;
        forceOut.input = this->firingTimeRemaining[i] > 1e-12 ? this->thrMag[i] : 0.0;
        this->thrusterForceOutMsgs[i]->write(&forceOut, this->moduleID, CurrentSimNanos);
    }
}

void ThrOnTimeToForce::setThrMag(const std::vector<double>& value)
{
    this->thrMag = value;
}

void ThrOnTimeToForce::addThruster()
{
    this->numThr++;
    this->thrusterForceOutMsgs.push_back(new Message<SingleActuatorMsgPayload>());
}
