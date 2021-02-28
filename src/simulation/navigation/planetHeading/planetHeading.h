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

#pragma once

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <Eigen/Dense>
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/BodyHeadingMsgPayload.h"
#include "architecture/messaging/messaging.h"


/*! @brief planet heading class */
class PlanetHeading: public SysModel {
public:
    PlanetHeading();
    ~PlanetHeading(){};
    
    void UpdateState(uint64_t CurrentSimNanos) override;
    void Reset(uint64_t CurrentSimNanos) override;
    void writeMessages(uint64_t CurrentSimNanos);
    void readMessages();

public:
    ReadFunctor<SpicePlanetStateMsgPayload> planetPositionInMsg;    //!< planet state input message
    ReadFunctor<SCStatesMsgPayload> spacecraftStateInMsg;       //!< spacecraft state input message
    Message<BodyHeadingMsgPayload> planetHeadingOutMsg;             //!< body heading output message

    BSKLogger bskLogger;                        //!< -- BSK Logging

private:
    Eigen::Vector3d r_PN_N;  //!< [m] planet position
    Eigen::Vector3d r_BN_N;  //!< [m] s/c position
    Eigen::Vector3d rHat_PB_B;  //!< [] planet heading in s/c body frame (unit mag)
    Eigen::MRPd sigma_BN;  //!< [] s/c body att wrt inertial
};
