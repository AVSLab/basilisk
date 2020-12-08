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

#include "solarFlux.h"
#include "../architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/solarFluxSimMsg.h"
#include "simMessages/eclipseSimMsg.h"

/*! This method is used to create messages
 @return void
 */
void SolarFlux::SelfInit()
{
    auto messagingSystem = SystemMessaging::GetInstance();
    this->solarFluxOutMsgId = messagingSystem->CreateNewMessage(this->solarFluxOutMsgName, sizeof(SolarFluxSimMsg), 2,
            "SolarFluxSimMsg", this->moduleID);
}

/*! This method is used to subscribe to modules
 @return void
 */
void SolarFlux::CrossInit() {
    auto messagingSystem = SystemMessaging::GetInstance();

    /*! - subscribe to required messages */
    this->sunPositionInMsgId = messagingSystem->subscribeToMessage(this->sunPositionInMsgName,
                                                                   sizeof(SpicePlanetStateSimMsg), this->moduleID);
    this->spacecraftStateInMsgId = messagingSystem->subscribeToMessage(this->spacecraftStateInMsgName,
                                                                       sizeof(SCPlusStatesSimMsg), this->moduleID);

    /*! - check for optional eclipse msg */
    if (this->eclipseInMsgName.length() > 0) {
        this->eclipseInMsgId = messagingSystem->subscribeToMessage(this->eclipseInMsgName, sizeof(EclipseSimMsg),
                                                                   this->moduleID);
    }
}

/*! This method is used to reset the module. Currently no tasks are required.
 @return void
 */
void SolarFlux::Reset(uint64_t CurrentSimNanos)
{
    return;
}

/*! Read Messages and scale the solar flux then write it out
 @return void
 */
void SolarFlux::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages();

    /*! - evaluate spacecraft position relative to the sun in N frame components */
    auto r_SSc_N = this->r_SN_N - this->r_ScN_N;

    /*! - compute the scalar distance to the sun.  The following math requires this to be in km. */
    double dist_SSc_N = r_SSc_N.norm() / 1000;  // to km

    /*! - compute the local solar flux value */
    this->fluxAtSpacecraft = SOLAR_FLUX_EARTH * pow(AU, 2) / pow(dist_SSc_N, 2) * this->eclipseFactor;

    this->writeMessages(CurrentSimNanos);
}

/*! This method is used to  read messages and save values to member attributes
 @return void
 */
void SolarFlux::readMessages() {
    SingleMessageHeader tmpHeader;
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    auto messagingSystem = SystemMessaging::GetInstance();

    /*! - read in planet state message (required) */
    SpicePlanetStateSimMsg sunPositionMsgData;
    messagingSystem->ReadMessage(this->sunPositionInMsgId, &tmpHeader, sizeof(SpicePlanetStateSimMsg),
                                 reinterpret_cast<uint8_t*>(&sunPositionMsgData), this->moduleID);
    this->r_SN_N = Eigen::Vector3d(sunPositionMsgData.PositionVector);

    /*! - read in spacecraft state message (required) */
    SCPlusStatesSimMsg scStatesMsgData;
    messagingSystem->ReadMessage(this->spacecraftStateInMsgId, &tmpHeader, sizeof(SCPlusStatesSimMsg),
                                 reinterpret_cast<uint8_t*>(&scStatesMsgData), this->moduleID);
    this->r_ScN_N = Eigen::Vector3d(scStatesMsgData.r_BN_N);

    /*! - read in eclipse message (optional) */
    if (this->eclipseInMsgId >= 0) {
        EclipseSimMsg eclipseInMsgData;
        messagingSystem->ReadMessage(this->eclipseInMsgId, &tmpHeader, sizeof(EclipseSimMsg),
                                     reinterpret_cast<uint8_t *>(&eclipseInMsgData), this->moduleID);
        this->eclipseFactor = eclipseInMsgData.shadowFactor;
    }

}

/*! This method is used to write the output flux message
 @return void
 */
void SolarFlux::writeMessages(uint64_t CurrentSimNanos) {
    SolarFluxSimMsg fluxMsgOutData = {this->fluxAtSpacecraft};
    SystemMessaging::GetInstance()->WriteMessage(this->solarFluxOutMsgId, CurrentSimNanos, sizeof(SolarFluxSimMsg),
                                  reinterpret_cast<uint8_t*>(&fluxMsgOutData));
}
