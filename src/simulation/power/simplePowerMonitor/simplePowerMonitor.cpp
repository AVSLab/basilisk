//
// Created by andrew on 7/12/19.
//

#include "simplePowerMonitor.h"


SimplePowerMonitor::SimplePowerMonitor(){

    this->storageCapacity = -1;
    this->storedCharge = 0;
    return;
}

SimplePowerMonitor::~SimplePowerMonitor(){

    return;
}

void SimplePowerMonitor::evaluateBatteryModel(PowerStorageStatusSimMsg *msg,double currentTime) {

    this->storedCharge = this->storedCharge + this->currentPowerSum * (this->currentTimestep);
    msg->storageCapacity = this->storageCapacity;
    msg->storageLevel = this->storedCharge;
    msg->currentNetPower = this->currentPowerSum;

    return;
}