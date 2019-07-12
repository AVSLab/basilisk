//
// Created by andrew on 7/12/19.
//

#include "simplePowerMonitor.h"


void SimplePowerMonitor::SimplePowerMonitor(){
    return;
}

void SimplePowerMonitor::~SimplePowerMonitor(){

    return;
}

void SimplePowerMonitor::evaluateBatteryModel(PowerStorageStatusSimMsg *msg) {
    msg.storageLevel = this->currentPowerSum;
    msg.storageCapacity = -1;

    return;
}