//
// Created by andrew on 7/12/19.
//

#include "simplePowerSink.h"

void SimplePowerSink::SimplePowerSink(){

    this->nodePowerOut = 0.0;
    return;

}

void SimplePowerSink::~SimplePowerSink(){

    return;
}

void SimplePowerSink::evaluatePowerModel(PowerNodeUsageSimMsg *powerUsageSimMsg){


    powerUsageSimMsg.netPower_W = this->nodePowerOut;

    return;
}