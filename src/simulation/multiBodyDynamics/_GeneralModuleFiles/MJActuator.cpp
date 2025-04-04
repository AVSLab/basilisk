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

void MJActuator::configure(const mjModel* model)
{
    for (auto&& sub : subActuators)
        sub.configure(model);
}

void MJActuator::updateCtrl(mjData* data)
{
    auto values = readControlMessages();
    for (size_t i = 0; i < this->subActuators.size(); i++)
    {
        this->subActuators.at(i).updateCtrl(data, values.at(i));
    }
}

std::vector<double> MJSingleActuator::readControlMessages()
{
    return actuatorInMsg.isLinked() ? std::vector{actuatorInMsg().input} : std::vector{0.};
}

std::vector<double> MJForceActuator::readControlMessages()
{
    auto fInMsg = forceInMsg.isLinked() ? forceInMsg() : forceInMsg.zeroMsgPayload;
    return {fInMsg.force_S[0], fInMsg.force_S[1], fInMsg.force_S[2]};
}

std::vector<double> MJTorqueActuator::readControlMessages()
{
    auto tInMsg = torqueInMsg.isLinked() ? torqueInMsg() : torqueInMsg.zeroMsgPayload;
    return {tInMsg.torque_S[0], tInMsg.torque_S[1], tInMsg.torque_S[2]};
}

std::vector<double> MJForceTorqueActuator::readControlMessages()
{
    auto fInMsg = forceInMsg.isLinked() ? forceInMsg() : forceInMsg.zeroMsgPayload;
    auto tInMsg = torqueInMsg.isLinked() ? torqueInMsg() : torqueInMsg.zeroMsgPayload;
    return {fInMsg.force_S[0],
            fInMsg.force_S[1],
            fInMsg.force_S[2],
            tInMsg.torque_S[0],
            tInMsg.torque_S[1],
            tInMsg.torque_S[2]};
}
