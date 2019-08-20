/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
#include "simpleBattery.h"

SimpleBattery::SimpleBattery(){

    this->storageCapacity = -1;
    this->storedCharge = 0;
    return;
}

SimpleBattery::~SimpleBattery(){

    return;
}

/*! This method is evaluates the centered dipole magnetic field model.
 @param msg magnetic field message structure
 @return void
 */
void SimpleBattery::evaluateBatteryModel(PowerStorageStatusSimMsg *msg,double currentTime) {

    this->storedCharge = this->storedCharge + this->currentPowerSum * (this->currentTimestep/3600.0); //integrate over hours


    if(this->storedCharge > this->storageCapacity) {
        this->storedCharge = this->storageCapacity;
    }

    if(this->storedCharge < 0)
    {
        this->storedCharge = 0;
    }

    msg->storageCapacity = this->storageCapacity;
    msg->storageLevel = this->storedCharge;
    msg->currentNetPower = this->currentPowerSum;

    return;
}