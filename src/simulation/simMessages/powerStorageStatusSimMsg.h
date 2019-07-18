//
// Created by andrew on 7/9/19.
//

#ifndef BASILISK_POWERSTORAGESTATUSSIMMSG_H
#define BASILISK_POWERSTORAGESTATUSSIMMSG_H


typedef struct{
    double storageLevel;
    double storageCapacity;
    double currentNetPower;
}PowerStorageStatusSimMsg;

#endif //BASILISK_POWERSTORAGESTATUSSIMMSG_H
