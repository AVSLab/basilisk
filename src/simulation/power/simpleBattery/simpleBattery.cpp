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

/*! The constructor creates a SimpleBattery instance with zero stored charge and a capacity of -1.*/
SimpleBattery::SimpleBattery(){

    this->storageCapacity = -1;
    this->storedCharge = 0;
    return;
}

SimpleBattery::~SimpleBattery(){

    return;
}

/*! This method integrates the current net power, and checks to see whether the integrated power falls between 0 and the battery's storageCapacity.
 @param *msg:  pointer to a PowerStorageStatusSimMsg instance
 @return void
 */
void SimpleBattery::evaluateBatteryModel(PowerStorageStatusSimMsg *msg) {

    this->storedCharge = this->storedCharge + this->currentPowerSum * (this->currentTimestep*SEC2HOUR); //integrate over hours


 */

/*! @brief Power storage module that considers net power up to some capacity.

 The SimpleBattery class is a minimal model of battery functionality that considers:
1. Integrated net input power of the attached modules
2. The battery's maximum storage capacity as defined by the storageCapacity attribute.
Integration of the net input power is performed with a simple Euler method.

To set up this module, users must create a simpleBattery instance, set its storageCapacity
attribute, and attach one or more PowerNodeUsageSimMsg instances to it using the
addNodeToStorage() method.

For more information on how to set up and use this module, see the simple power system example: @ref scenarioSimplePowerDemo

 */
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