/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "scalarJointStatesToRWSpeed.h"

ScalarJointStatesToRWSpeed::ScalarJointStatesToRWSpeed()
    : numJoints(0)
{
}

ScalarJointStatesToRWSpeed::ScalarJointStatesToRWSpeed(uint32_t numJointsIn)
    : numJoints(numJointsIn)
{
    this->ensureSizes();
}

ScalarJointStatesToRWSpeed::~ScalarJointStatesToRWSpeed() = default;

void ScalarJointStatesToRWSpeed::setNumJoints(uint32_t numJointsIn)
{
    this->numJoints = numJointsIn;
    this->ensureSizes();
}

void ScalarJointStatesToRWSpeed::ensureSizes()
{
    this->jointStateInMsgs.resize(this->numJoints);
}

void ScalarJointStatesToRWSpeed::Reset(uint64_t CurrentSimNanos)
{
    this->ensureSizes();

    // zero out output on reset
    RWSpeedMsgPayload outPayload = {};
    this->rwSpeedOutMsg.write(
        &outPayload,
        this->moduleID,
        CurrentSimNanos
    );
}

void ScalarJointStatesToRWSpeed::UpdateState(uint64_t CurrentSimNanos)
{
    RWSpeedMsgPayload outPayload = {};
    for (uint32_t i = 0; i < this->numJoints; ++i) {
        ScalarJointStateMsgPayload inPayload = this->jointStateInMsgs[i]();
        outPayload.wheelSpeeds[i] = inPayload.state;
    }

    this->rwSpeedOutMsg.write(
        &outPayload,
        this->moduleID,
        CurrentSimNanos
    );
}
