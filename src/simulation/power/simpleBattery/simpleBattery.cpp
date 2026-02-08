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
#include "simpleBattery.h"

/*! The constructor creates a SimpleBattery instance with zero stored charge */
SimpleBattery::SimpleBattery(){

    this->storageCapacity = -1.0;
    this->storedCharge = 0.0;
    this->faultCapacityRatio = 1.0;
    return;
}

SimpleBattery::~SimpleBattery(){

    return;
}

/*! custom reset function.
 */
void SimpleBattery::customReset(uint64_t CurrentClock) {

    if (this->storageCapacity <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "The storageCapacity variable must be set to a positive value.");
    }
    return;
}

/*! This method allows the user to set the fault status of the battery capacity
*/
void SimpleBattery::readInputMessage(){
     if(this->batteryFaultInMsg.isLinked()){
        PowerStorageFaultMsgPayload faultMsg;
        faultMsg = this->batteryFaultInMsg();
        this->faultCapacityRatio = faultMsg.faultCapacityRatio;
     }
     else{
        this->faultCapacityRatio = 1.0;
     }
}

/*! This method integrates the current net power, and checks to see whether the integrated power falls between 0 and the battery's storageCapacity.
 @param *msg:  pointer to a PowerStorageStatusMsgPayload instance

 */
void SimpleBattery::evaluateBatteryModel(PowerStorageStatusMsgPayload *msg) {

    this->storedCharge = this->storedCharge + this->currentPowerSum * (this->currentTimestep);

    if(this->storedCharge > this->storageCapacity) {
        this->storedCharge = this->storageCapacity;
    }

    if(this->storedCharge < 0)
    {
        this->storedCharge = 0;
    }

    this->readInputMessage();
    if (this->faultCapacityRatio < 0 || this->faultCapacityRatio > 1) {
        bskLogger.bskLog(BSK_ERROR, "faultCapacityRatio should be between 0 and 1!");
    } else if(this->storedCharge > this->storageCapacity * this->faultCapacityRatio) {
            this->storedCharge = this->storageCapacity * this->faultCapacityRatio;
    }

    msg->storageCapacity = this->storageCapacity;
    msg->storageLevel = this->storedCharge;
    msg->currentNetPower = this->currentPowerSum;

    return;
}
