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

#include "power/_GeneralModuleFiles/powerNodeBase.h"
#include "../../simMessages/rwConfigLogSimMsg.h"
#include "utilities/bskLogging.h"




class PowerRW: public PowerNodeBase {

public:
    PowerRW();
    ~PowerRW();
    void customCrossInit();             //!< Custom message subscription method
    void customReset(uint64_t CurrentSimNanos); //!< Custom reset method
    bool customReadMessages();          //!< Custom read method, similar to customSelfInit; returns `true' by default.

private:
    void evaluatePowerModel(PowerNodeUsageSimMsg *powerUsageMsg);

public:
    std::string rwStateInMsgName;       //!< Reaction wheel state input message name
    double eta_e2m;                     //!< efficiency factor to convert electrical power to mechanical power
    double eta_m2e;                     //!< efficiency factor to convert mechanical power to electrical power
    BSKLogger bskLogger;                //!< -- BSK Logging

private:
    int64_t rwStateInMsgId;             //!< Message ID for the RW state input message
    RWConfigLogSimMsg rwStatus;         //!< copy of the RW status message

};


#endif //BASILISK_POWERRW_H
