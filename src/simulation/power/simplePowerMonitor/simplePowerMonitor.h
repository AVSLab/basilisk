//
// Created by andrew on 7/12/19.
//

#ifndef BASILISK_SIMPLEPOWERMONITOR_H
#define BASILISK_SIMPLEPOWERMONITOR_H


class SimplePowerMonitor: public simPowerStorageBase {

public:
    SimplePowerMonitor::SimplePowerMonitor;
    SimplePowerMonitor::~SimplePowerMonitor;

private:
    void evaluateBatteryModel(PowerStorageStatusSimMsg *msg);

public:
    double storageCapacity;

};


#endif //BASILISK_SIMPLEPOWERMONITOR_H
