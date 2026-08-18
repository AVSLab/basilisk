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

#include "twoHingeDamper.h"

#include <cmath>

void TwoHingeDamper::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    if (!this->thetaInMsg.isLinked()) {
        this->bskLogger.bskError("TwoHingeDamper.thetaInMsg is not linked.");
    }
    if (!this->phiDotInMsg.isLinked()) {
        this->bskLogger.bskError("TwoHingeDamper.phiDotInMsg is not linked.");
    }
    if (!this->thetaDotInMsg.isLinked()) {
        this->bskLogger.bskError("TwoHingeDamper.thetaDotInMsg is not linked.");
    }
    if (!std::isfinite(this->dampingCoeff) || this->dampingCoeff < 0.0) {
        this->bskLogger.bskError("TwoHingeDamper.dampingCoeff must be finite and nonnegative.");
    }
}

void TwoHingeDamper::UpdateState(uint64_t CurrentSimNanos)
{
    const double theta = this->thetaInMsg().state;
    const double cosTheta = std::cos(theta);

    SingleActuatorMsgPayload phiPayload = this->phiTorqueOutMsg.zeroMsgPayload;
    phiPayload.input =
        -this->dampingCoeff*cosTheta*cosTheta*this->phiDotInMsg().state;
    this->phiTorqueOutMsg.write(&phiPayload, this->moduleID, CurrentSimNanos);

    SingleActuatorMsgPayload thetaPayload = this->thetaTorqueOutMsg.zeroMsgPayload;
    thetaPayload.input = -this->dampingCoeff*this->thetaDotInMsg().state;
    this->thetaTorqueOutMsg.write(&thetaPayload, this->moduleID, CurrentSimNanos);
}
