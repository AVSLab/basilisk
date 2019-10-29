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
#include "architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/solarFluxSimMsg.h"
#include "simMessages/eclipseSimMsg.h"

void SolarFlux::SelfInit()
{
    auto messagingSystem = SystemMessaging::GetInstance();
    this->solarFluxOutMsgId = messagingSystem->CreateNewMessage(this->solarFluxOutMsgName, sizeof(SolarFluxSimMsg), 2,
            "SolarFluxSimMsg", this->moduleID);
}

void SolarFlux::CrossInit() {
    auto messagingSystem = SystemMessaging::GetInstance();

    /*! - read in required messages */
    this->sunPositionInMsgId = messagingSystem->subscribeToMessage(this->sunPositionInMsgName,
                                                                   sizeof(SpicePlanetStateSimMsg), this->moduleID);
    this->spacecraftStateInMsgId = messagingSystem->subscribeToMessage(this->spacecraftStateInMsgName,
                                                                       sizeof(SCPlusStatesSimMsg), this->moduleID);

    /*! - check for optional eclipse msg */
    if (this->eclipseInMsgName.length() > 0) {
        this->eclipseInMsgId = messagingSystem->subscribeToMessage(this->eclipseInMsgName, sizeof(EclipseSimMsg),
                                                                   this->moduleID);
    } else {
        /* set to default values if msg is not present */
        this->eclipseInMsgId = -1;
        this->eclipseFactor = 1.0;
    }
}

/*! This method is used to reset the module. Currently no tasks are required.
 @return void
 */
void SolarFlux::Reset(uint64_t CurrentSimNanos)
{
    return;
}

void SolarFlux::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages();

    auto r_SSc_N = this->r_SN_N - this->r_ScN_N;
    double dist_SSc_N = r_SSc_N.norm() / 1000;  // to km
    this->fluxAtSpacecraft = SOLAR_FLUX_EARTH * pow(AU, 2) / pow(dist_SSc_N, 2) * this->eclipseFactor;

    this->writeMessages(CurrentSimNanos);
}

void SolarFlux::readMessages() {
    SingleMessageHeader tmpHeader;
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    auto messagingSystem = SystemMessaging::GetInstance();

    SpicePlanetStateSimMsg sunPositionMsgData;
    messagingSystem->ReadMessage(this->sunPositionInMsgId, &tmpHeader, sizeof(SpicePlanetStateSimMsg),
                                 reinterpret_cast<uint8_t*>(&sunPositionMsgData), this->moduleID);
    this->r_SN_N = Eigen::Vector3d(sunPositionMsgData.PositionVector);

    SCPlusStatesSimMsg scStatesMsgData;
    messagingSystem->ReadMessage(this->spacecraftStateInMsgId, &tmpHeader, sizeof(SCPlusStatesSimMsg),
                                 reinterpret_cast<uint8_t*>(&scStatesMsgData), this->moduleID);
    this->r_ScN_N = Eigen::Vector3d(scStatesMsgData.r_BN_N);

    if (this->eclipseInMsgId >= 0) {
        EclipseSimMsg eclipseInMsgData;
        messagingSystem->ReadMessage(this->eclipseInMsgId, &tmpHeader, sizeof(EclipseSimMsg),
                                     reinterpret_cast<uint8_t *>(&eclipseInMsgData), this->moduleID);
        this->eclipseFactor = eclipseInMsgData.shadowFactor;
    }

}

void SolarFlux::writeMessages(uint64_t CurrentSimNanos) {
    SolarFluxSimMsg fluxMsgOutData = {this->fluxAtSpacecraft};
    SystemMessaging::GetInstance()->WriteMessage(this->solarFluxOutMsgId, CurrentSimNanos, sizeof(SolarFluxSimMsg),
                                  reinterpret_cast<uint8_t*>(&fluxMsgOutData));
}
