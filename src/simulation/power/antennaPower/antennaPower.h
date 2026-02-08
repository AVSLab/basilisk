/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU, Norway

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


#ifndef BASILISK_POWERANTENNA_H
#define BASILISK_POWERANTENNA_H

#include "architecture/messaging/messaging.h"
#include "simulation/power/_GeneralModuleFiles/powerNodeBase.h"
#include "simulation/communication/_GeneralModuleFiles/AntennaDefinitions.h"

#include "architecture/msgPayloadDefC/AntennaLogMsgPayload.h"

#include "architecture/utilities/bskLogging.h"



/*! @brief Antenna power class */
class AntennaPower: public PowerNodeBase {

public:
    AntennaPower();
    ~AntennaPower();
    void customReset(uint64_t CurrentSimNanos); //!< Custom reset method
    bool customReadMessages();                  //!< Custom read method, similar to customSelfInit; returns `true' by default.

private:
    void evaluatePowerModel(PowerNodeUsageMsgPayload *powerUsageMsg);

public:
    ReadFunctor<AntennaLogMsgPayload> antennaSetStateInMsg;  //!< Antenna state input message
    double basePowerNeed;                                    //!< [W] base electrical power required to operate an antenna (positive value)
    BSKLogger bskLogger;                                     //!< -- BSK Logging

private:
    AntennaLogMsgPayload antennaStatusMsgBuffer = {};        //!< copy of the antenna status message
};

#endif //BASILISK_POWERANTENNA_H
