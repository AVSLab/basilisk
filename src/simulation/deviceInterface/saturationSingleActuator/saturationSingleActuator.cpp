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

#include "saturationSingleActuator.h"

#include <algorithm>

void SaturationSingleActuator::UpdateState(uint64_t currentSimNanos)
{
    SingleActuatorMsgPayload inPayload = this->actuatorInMsg();
    SingleActuatorMsgPayload outPayload = {};

    outPayload.input = this->applySaturation(inPayload.input);

    this->actuatorOutMsg.write(&outPayload, currentSimNanos, this->moduleID);
}

void SaturationSingleActuator::setSaturationEnabled(bool enabled)
{
    this->saturationEnabled = enabled;
}

void SaturationSingleActuator::setMinInput(double minInput)
{
    this->minInput = minInput;
}

void SaturationSingleActuator::setMaxInput(double maxInput)
{
    this->maxInput = maxInput;
}

double SaturationSingleActuator::applySaturation(double inputValue) const
{
    if (!this->saturationEnabled) {
        return inputValue;
    }

    return std::max(this->minInput, std::min(inputValue, this->maxInput));
}
