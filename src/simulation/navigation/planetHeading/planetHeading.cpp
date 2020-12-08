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

#include "planetHeading.h"
#include "messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/bodyHeadingSimMsg.h"
#include <iostream>

/*! Customer constructor just sets the spacecraftSTateInMsgName by default*/
PlanetHeading::PlanetHeading() : spacecraftStateInMsgName("inertial_state_output") {}

/*! This method is used to create the body heading message for the heading to the planet
 @return void
 */
void PlanetHeading::SelfInit()
{
    auto messagingSystem = SystemMessaging::GetInstance();
    this->planetHeadingOutMsgId = messagingSystem->CreateNewMessage(this->planetHeadingOutMsgName, sizeof(BodyHeadingSimMsg), 2,
            "BodyHeadingSimMsg", this->moduleID);
}

/*! This method is used to subscribe to planet and spacecraft messages
 @return void
 */
void PlanetHeading::CrossInit() {
    /*! - subscribe to required messages */
    auto messagingSystem = SystemMessaging::GetInstance();
    this->planetPositionInMsgId = messagingSystem->subscribeToMessage(this->planetPositionInMsgName,
                                                                   sizeof(SpicePlanetStateSimMsg), this->moduleID);
    this->spacecraftStateInMsgId = messagingSystem->subscribeToMessage(this->spacecraftStateInMsgName,
                                                                       sizeof(SCPlusStatesSimMsg), this->moduleID);
}

/*! This method reads messages, calculates the planet heading, and writes out the heading message
 @return void
 */
void PlanetHeading::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages();

    /*! - evaluate planet position relative to the s/c body in body frame components */
    auto r_PB_N = this->r_PN_N - this->r_BN_N;
    /*! - normalize and convert to body frame */
    this->rHat_PB_B = (this->sigma_BN.toRotationMatrix().transpose() * r_PB_N).normalized();

    this->writeMessages(CurrentSimNanos);
}

/*! Read input messages and save data to member variables
 @return void
 */
void PlanetHeading::readMessages() {
    SingleMessageHeader tmpHeader;
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    auto messagingSystem = SystemMessaging::GetInstance();

    SpicePlanetStateSimMsg planetPositionMsgData;
    /*! - read in planet state message (required) */
    messagingSystem->ReadMessage(this->planetPositionInMsgId, &tmpHeader, sizeof(SpicePlanetStateSimMsg),
                                 reinterpret_cast<uint8_t*>(&planetPositionMsgData), this->moduleID);
    this->r_PN_N = Eigen::Vector3d(planetPositionMsgData.PositionVector);

    SCPlusStatesSimMsg scStatesMsgData;
    /*! - read in spacecraft state message (required) */
    messagingSystem->ReadMessage(this->spacecraftStateInMsgId, &tmpHeader, sizeof(SCPlusStatesSimMsg),
                                 reinterpret_cast<uint8_t*>(&scStatesMsgData), this->moduleID);
    this->r_BN_N = Eigen::Vector3d(scStatesMsgData.r_BN_N);
    this->sigma_BN = Eigen::MRPd(scStatesMsgData.sigma_BN);
}

/*! This method is used to write out the planet heading message
 @return void
 */
void PlanetHeading::writeMessages(uint64_t CurrentSimNanos) {
    BodyHeadingSimMsg planetHeadingOutMsgData;
    Eigen::VectorXd::Map(planetHeadingOutMsgData.rHat_XB_B, this->rHat_PB_B.rows()) = this->rHat_PB_B;  // eigen to c array for messaging

    /*! - write the output message */
    SystemMessaging::GetInstance()->WriteMessage(this->planetHeadingOutMsgId, CurrentSimNanos, sizeof(BodyHeadingSimMsg),
                                  reinterpret_cast<uint8_t*>(&planetHeadingOutMsgData));
}

/*! This method is used to reset the module. Currently no tasks are required.
 @return void
 */
void PlanetHeading::Reset(uint64_t CurrentSimNanos)
{
    return;
}
