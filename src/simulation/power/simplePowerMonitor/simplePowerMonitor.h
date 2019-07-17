//
// Created by andrew on 7/12/19.
//

#ifndef BASILISK_SIMPLEPOWERMONITOR_H
#define BASILISK_SIMPLEPOWERMONITOR_H

#include "../_GeneralModuleFiles/simPowerStorageBase.h"
#include "../../simMessages/powerStorageStatusSimMsg.h"

class SimplePowerMonitor: public PowerStorageBase {

public:
    SimplePowerMonitor();
    ~SimplePowerMonitor();

private:
    void evaluateBatteryModel(PowerStorageStatusSimMsg *msg, double currentTime);

public:
    double storageCapacity;

};


#endif //BASILISK_SIMPLEPOWERMONITOR_H
