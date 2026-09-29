/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "MJActuator.h"

void MJActuatorObject::updateCtrl(mjData* data, double value) { data->ctrl[this->getId()] = value; }

void
MJActuator::configure(const mjModel* model)
{
    for (auto& sub : this->subActuators) {
        sub.configure(model);
    }
}

void
MJSingleActuator::updateCtrl(mjData* data)
{
    const double value = this->actuatorInMsg.isLinked() ? this->actuatorInMsg().input : 0.0;
    this->subActuators[0].updateCtrl(data, value);
}

void
MJForceActuator::updateCtrl(mjData* data)
{
    const auto input = this->forceInMsg.isLinked() ? this->forceInMsg() : this->forceInMsg.zeroMsgPayload;
    for (size_t index = 0; index < 3; ++index) {
        this->subActuators[index].updateCtrl(data, input.force_S[index]);
    }
}

void
MJTorqueActuator::updateCtrl(mjData* data)
{
    const auto input = this->torqueInMsg.isLinked() ? this->torqueInMsg() : this->torqueInMsg.zeroMsgPayload;
    for (size_t index = 0; index < 3; ++index) {
        this->subActuators[index].updateCtrl(data, input.torque_S[index]);
    }
}

void
MJForceTorqueActuator::updateCtrl(mjData* data)
{
    const auto force = this->forceInMsg.isLinked() ? this->forceInMsg() : this->forceInMsg.zeroMsgPayload;
    const auto torque = this->torqueInMsg.isLinked() ? this->torqueInMsg() : this->torqueInMsg.zeroMsgPayload;
    for (size_t index = 0; index < 3; ++index) {
        this->subActuators[index].updateCtrl(data, force.force_S[index]);
        this->subActuators[index + 3].updateCtrl(data, torque.torque_S[index]);
    }
}
