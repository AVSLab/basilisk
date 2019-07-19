//
// Created by andrew on 7/18/19.
//

#ifndef BASILISK_SIMPLEBATTERY_H
#define BASILISK_SIMPLEBATTERY_H


#include "../_GeneralModuleFiles/simPowerStorageBase.h"
#include "../../simMessages/powerStorageStatusSimMsg.h"

class SimpleBattery: public PowerStorageBase {

public:
    SimpleBattery();
    ~SimpleBattery();

private:
    void evaluateBatteryModel(PowerStorageStatusSimMsg *msg, double currentTime);

public:
    double storageCapacity;

};


#endif //BASILISK_SIMPLEBATTERY_H
