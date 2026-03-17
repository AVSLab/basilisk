/*
 ISC License

 Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder

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

#include "windBase.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/simDefinitions.h"

WindBase::WindBase()
{
    //! - zero the planet message and set DCM to identity
    this->planetState = this->planetPosInMsg.zeroMsgPayload;
    m33SetIdentity(this->planetState.J20002Pfix);

    //! - set default epoch (Basilisk standard epoch)
    this->epochDateTime.tm_year  = EPOCH_YEAR - 1900;
    this->epochDateTime.tm_mon   = EPOCH_MONTH - 1;
    this->epochDateTime.tm_mday  = EPOCH_DAY;
    this->epochDateTime.tm_hour  = EPOCH_HOUR;
    this->epochDateTime.tm_min   = EPOCH_MIN;
    this->epochDateTime.tm_sec   = (int) round(EPOCH_SEC);
    this->epochDateTime.tm_isdst = -1;
}

WindBase::~WindBase()
{
    for (auto* msg : this->envOutMsgs) {
        delete msg;
    }
}

void WindBase::Reset(uint64_t CurrentSimNanos)
{
    if (this->scStateInMsgs.empty()) {
        bskLogger.bskLog(BSK_ERROR, "Wind model has no spacecraft added to it.");
    }
    if (!this->planetPosInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "WindBase: planetPosInMsg is not linked.");
    }

    //! - set epoch from message if provided, otherwise let subclass set it from a variable
    if (this->epochInMsg.isLinked()) {
        if (!this->epochInMsg.isWritten()) {
            bskLogger.bskLog(BSK_ERROR, "WindBase: epochInMsg is linked but has not been written.");
        }
        EpochMsgPayload epochMsg = this->epochInMsg();
        this->epochDateTime.tm_year  = epochMsg.year - 1900;
        this->epochDateTime.tm_mon   = epochMsg.month - 1;
        this->epochDateTime.tm_mday  = epochMsg.day;
        this->epochDateTime.tm_hour  = epochMsg.hours;
        this->epochDateTime.tm_min   = epochMsg.minutes;
        this->epochDateTime.tm_sec   = (int) round(epochMsg.seconds);
        mktime(&this->epochDateTime);
    } else {
        customSetEpochFromVariable();
    }

    customReset(CurrentSimNanos);
}

void WindBase::customReset(uint64_t CurrentClock)
{
    // Read the planet message to get the current SPICE data
    if (this->planetPosInMsg.isWritten()) {
        this->planetState = this->planetPosInMsg();
        // Update planetOmega_N from SPICE data only if SPICE mode is enabled
        if (this->useSpiceOmega) {
            this->updatePlanetOmegaFromSpice();
        }
    }
}

void WindBase::customSetEpochFromVariable() {}

bool WindBase::readMessages()
{
    this->scStates.clear();
    bool scRead = std::all_of(
        this->scStateInMsgs.begin(), this->scStateInMsgs.end(),
        [](auto& msg) { return msg.isWritten(); }
    );

    if (scRead) {
        for (auto& msg : this->scStateInMsgs) {
            this->scStates.push_back(msg());
        }
    }

    bool planetRead = this->planetPosInMsg.isWritten();
    this->planetState = this->planetPosInMsg();

    // Update planetOmega_N from SPICE data only if SPICE mode is enabled and planetPosInMsg has ever been written to
    if (this->useSpiceOmega && planetRead) {
        this->updatePlanetOmegaFromSpice();
    }

    return scRead && planetRead && customReadMessages();
}

bool WindBase::customReadMessages() { return true; }

Eigen::Vector3d WindBase::computeRelativePos(const SpicePlanetStateMsgPayload& planetState,
                                             const SCStatesMsgPayload& scState) const
{
    return Eigen::Vector3d(scState.r_BN_N) - Eigen::Vector3d(planetState.PositionVector);  // r_BP_N
}

Eigen::Vector3d WindBase::getPlanetOmega_N() const
{
    return this->planetOmega_N;
}

void WindBase::setPlanetOmega_N(const Eigen::Vector3d &omega)
{
    // Validate input for NaN and infinite values
    if (std::isnan(omega(0)) || std::isnan(omega(1)) || std::isnan(omega(2)) ||
        std::isinf(omega(0)) || std::isinf(omega(1)) || std::isinf(omega(2))) {
        bskLogger.bskLog(BSK_ERROR, "Planet angular velocity contains NaN or infinite values.");
    } else {
        this->planetOmega_N = omega;
    }
    this->useSpiceOmega = false;
}

bool WindBase::getUseSpiceOmegaFlag() const
{
    return this->useSpiceOmega;
}

void WindBase::setUseSpiceOmegaFlag(bool useSpice)
{
    this->useSpiceOmega = useSpice;
}

void WindBase::updatePlanetOmegaFromSpice()
{
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> C_dot(this->planetState.J20002Pfix_dot[0]);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> C(this->planetState.J20002Pfix[0]);
    const Eigen::Matrix3d skewP = -C_dot * C.transpose();
    const Eigen::Vector3d omega_P(skewP(2, 1), skewP(0, 2), skewP(1, 0));
    this->spiceOmega_N = C.transpose() * omega_P;
}

Eigen::Vector3d WindBase::getSelectedOmega()
{
    if (this->useSpiceOmega && this->planetPosInMsg.isWritten()) {
        return this->spiceOmega_N;
    }
    return this->planetOmega_N;  // manual fallback
}

void WindBase::updateLocalWind(double currentTime)
{
    // Compute co-rotating atmosphere velocity: omega_planet_N x r_BP_N
    const Eigen::Vector3d omega_planet_N = this->getSelectedOmega();

    for (size_t c = 0; c < this->scStates.size(); c++) {
        Eigen::Vector3d r_BP_N = this->computeRelativePos(this->planetState, this->scStates[c]);
        Eigen::Vector3d v_corotatingAir_N = omega_planet_N.cross(r_BP_N);

        WindMsgPayload windMsg = this->envOutMsgs[c]->zeroMsgPayload;
        this->evaluateWindModel(&windMsg, r_BP_N, v_corotatingAir_N, currentTime);
        this->envOutBuffer.push_back(windMsg);
    }
}

void WindBase::writeMessages(uint64_t CurrentClock)
{
    for (size_t c = 0; c < this->envOutMsgs.size(); c++) {
        if (c < this->envOutBuffer.size()) {
            this->envOutMsgs[c]->write(&this->envOutBuffer[c], this->moduleID, CurrentClock);
        }
    }
    customWriteMessages(CurrentClock);
}

void WindBase::customWriteMessages(uint64_t /*CurrentClock*/) {}

void WindBase::addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg)
{
    //! - add input message
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    //! - create output message
    Message<WindMsgPayload> *msg;
    msg = new Message<WindMsgPayload>;
    this->envOutMsgs.push_back(msg);
}

void WindBase::UpdateState(uint64_t CurrentSimNanos)
{
    this->envOutBuffer.clear();

    if (this->readMessages()) {
        this->updateLocalWind(CurrentSimNanos * NANO2SEC);
    } else {
        // Zero outputs when message reads fail to avoid stale data
        for (size_t c = 0; c < this->envOutMsgs.size(); c++) {
            WindMsgPayload zeroMsg = this->envOutMsgs[c]->zeroMsgPayload;
            this->envOutBuffer.push_back(zeroMsg);
        }
    }

    this->writeMessages(CurrentSimNanos);
}
