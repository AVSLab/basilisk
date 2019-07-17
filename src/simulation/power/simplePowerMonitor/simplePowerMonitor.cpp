//
// Created by andrew on 7/12/19.
//

#include "simplePowerMonitor.h"


SimplePowerMonitor::SimplePowerMonitor(){

    this->storageCapacity = -1;
    return;
}

SimplePowerMonitor::~SimplePowerMonitor(){

    return;
}

void SimplePowerMonitor::evaluateBatteryModel(PowerStorageStatusSimMsg *msg,double currentTime) {

    this->storedCharge = this->storedCharge + this->currentPowerSum * (currentTime - this->previousTime);
    msg->storageCapacity = this->storageCapacity;
    msg->storageLevel = this->storedCharge;

    return;
}