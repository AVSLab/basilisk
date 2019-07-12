//
// Created by andrew on 7/12/19.
//

#ifndef BASILISK_SIMPLEPOWERSINK_H
#define BASILISK_SIMPLEPOWERSINK_H


class simplePowerSink: public simPowerNodeBase {

public:
    SimplePowerSink::SimplePowerSink();
    SimplePowerSink::~SimplePowerSink();

private:
    void evaulatePowerModel(PowerNodeUsageSimMsg *powerUsageMsg);

};


#endif //BASILISK_SIMPLEPOWERSINK_H
