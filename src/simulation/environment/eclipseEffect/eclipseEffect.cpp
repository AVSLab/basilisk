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
#include "eclipseEffect.h"
#include "architecture/messaging/system_messaging.h"
#include "simMessages/eclipseSimMsg.h"
#include "simMessages/solarFluxSimMsg.h"

EclipseEffect::EclipseEffect(){}

EclipseEffect::~EclipseEffect(){}

void EclipseEffect::SelfInit(){}

void EclipseEffect::CrossInit()
{
    auto messagingSystem = SystemMessaging::GetInstance();
    this->eclipseInMsgId = messagingSystem->subscribeToMessage(this->eclipseInMsgName,
                                                               sizeof(EclipseSimMsg), this->moduleID);
    this->solarFluxInOutMsgId = messagingSystem->subscribeToMessage(this->solarFluxInOutMsgName, sizeof(SolarFluxSimMsg),
                                                                    this->moduleID);
    messagingSystem->obtainWriteRights(this->solarFluxInOutMsgId, this->moduleID);

}

void EclipseEffect::readInputMessages()
{
    auto messagingSystem = SystemMessaging::GetInstance();
    SingleMessageHeader tmpHeader;
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));

    EclipseSimMsg eclipseInMsgData;
    messagingSystem->ReadMessage(this->eclipseInMsgId, &tmpHeader, sizeof(EclipseSimMsg),
                                 reinterpret_cast<uint8_t*>(&eclipseInMsgData), this->moduleID);
    this->eclipseFactor = eclipseInMsgData.shadowFactor;

    SolarFluxSimMsg solarFluxInOutMsgData;
    messagingSystem->ReadMessage(this->solarFluxInOutMsgId, &tmpHeader, sizeof(SolarFluxSimMsg),
            reinterpret_cast<uint8_t*>(&solarFluxInOutMsgData), this->moduleID);
    this->fluxIn = solarFluxInOutMsgData.flux;
}

void EclipseEffect::writeOutputMessages(uint64_t CurrentSimNanos)
{
    auto messagingSystem = SystemMessaging::GetInstance();
    SolarFluxSimMsg fluxMsgOutData = {this->fluxOut};
    messagingSystem->WriteMessage(this->solarFluxInOutMsgId, CurrentSimNanos, sizeof(SolarFluxSimMsg),
                                                 reinterpret_cast<uint8_t*>(&fluxMsgOutData));
}

void EclipseEffect::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();

    this->fluxOut = this->fluxIn * this->eclipseFactor;

    this->writeOutputMessages(CurrentSimNanos);
}
