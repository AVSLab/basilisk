//
// Created by andrew on 7/18/19.
//

#include "simpleBattery.h"

SimpleBattery::SimpleBattery(){

    this->storageCapacity = -1;
    this->storedCharge = 0;
    return;
}

SimpleBattery::~SimpleBattery(){

    return;
}

void SimpleBattery::evaluateBatteryModel(PowerStorageStatusSimMsg *msg,double currentTime) {

    this->storedCharge = this->storedCharge + this->currentPowerSum * (this->currentTimestep/3600.0); //integrate over hours


    if(this->storedCharge > this->storageCapacity) {
        this->storedCharge = this->storageCapacity;
    }

    if(this->storedCharge < 0)
    {
        this->storedCharge = 0;
    }

    msg->storageCapacity = this->storageCapacity;
    msg->storageLevel = this->storedCharge;
    msg->currentNetPower = this->currentPowerSum;

    return;
}