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

#ifndef BASILISK_SIMPLEBATTERY_H
#define BASILISK_SIMPLEBATTERY_H


#include "simulation/power/_GeneralModuleFiles/powerStorageBase.h"
#include "architecture/msgPayloadDefC/PowerStorageFaultMsgPayload.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/bskLogging.h"


/*! @brief simple battery class */
class SimpleBattery: public PowerStorageBase {

public:
    SimpleBattery();
    ~SimpleBattery();
    void readInputMessage();
    ReadFunctor<PowerStorageFaultMsgPayload> batteryFaultInMsg; //!< input message to record battery status

private:
    void customReset(uint64_t CurrentClock);
    void evaluateBatteryModel(PowerStorageStatusMsgPayload *msg);
    double faultCapacityRatio; //!< Fault capacity ratio (faulted capacity / nominal capacity)

public:
    double storageCapacity; //!< [W-s] Battery capacity in Watt-seconds (Joules).
    BSKLogger bskLogger;                      //!< -- BSK Logging

};


#endif //BASILISK_SIMPLEBATTERY_H
