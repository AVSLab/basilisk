/*
 ISC License

 Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#ifndef BASILISK_POWERRW_H
#define BASILISK_POWERRW_H

#include "architecture/messaging/messaging.h"
#include "simulation/power/_GeneralModuleFiles/powerNodeBase.h"

#include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"

#include "architecture/utilities/bskLogging.h"



/*! @brief reaction wheel power class */
class ReactionWheelPower: public PowerNodeBase {

public:
    ReactionWheelPower();
    ~ReactionWheelPower();
    void customReset(uint64_t CurrentSimNanos); //!< Custom reset method
    bool customReadMessages();          //!< Custom read method, similar to customSelfInit; returns `true' by default.

private:
    void evaluatePowerModel(PowerNodeUsageMsgPayload *powerUsageMsg);

public:
    ReadFunctor<RWConfigLogMsgPayload> rwStateInMsg;  //!< Reaction wheel state input message name
    double elecToMechEfficiency;        //!< efficiency factor to convert electrical power to mechanical power
    double mechToElecEfficiency;        //!< efficiency factor to convert mechanical power to electrical power
    double basePowerNeed;               //!< [W] base electrical power required to operate RW, typically a positive value
    BSKLogger bskLogger;                //!< -- BSK Logging

private:
    RWConfigLogMsgPayload rwStatus;     //!< copy of the RW status message

};


#endif //BASILISK_POWERRW_H
