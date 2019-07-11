//
// Created by andrew on 7/9/19.
//

#ifndef BASILISK_POWERNODEUSAGESIMMSG_H
#define BASILISK_POWERNODEUSAGESIMMSG_H


typedef struct{
    double netPower_W; //Power usage by the message writer; positive for sources, negative for sinks
}PowerNodeUsageSimMsg;
#endif //BASILISK_POWERNODEUSAGESIMMSG_H
