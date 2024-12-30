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

#include "simplePowerMonitor.h"


/*! The constructor creates a SimplePowerMonitor instance with zero stored charge and a capacity of -1.*/
SimplePowerMonitor::SimplePowerMonitor(){

    this->storedCharge = 0;
    return;
}

SimplePowerMonitor::~SimplePowerMonitor(){

    return;
}

/*! This method integrates the net power across all the attached devices and stores it.

 */
void SimplePowerMonitor::evaluateBatteryModel(PowerStorageStatusMsgPayload *msg) {

    this->storedCharge = this->storedCharge + this->currentPowerSum * (this->currentTimestep);
    msg->storageCapacity = -1.0;
    msg->currentNetPower = this->currentPowerSum;
    msg->storageLevel = this->storedCharge;
    return;
}
