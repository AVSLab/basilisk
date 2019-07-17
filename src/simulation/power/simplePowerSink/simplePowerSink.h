//
// Created by andrew on 7/12/19.
//

#ifndef BASILISK_SIMPLEPOWERSINK_H
#define BASILISK_SIMPLEPOWERSINK_H

#include "../_GeneralModuleFiles/simPowerNodeBase.h"

class SimplePowerSink: public PowerNodeBase {

public:
    SimplePowerSink();
    ~SimplePowerSink();

private:
    void evaluatePowerModel(PowerNodeUsageSimMsg *powerUsageMsg);

};


#endif //BASILISK_SIMPLEPOWERSINK_H
