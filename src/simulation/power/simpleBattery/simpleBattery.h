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

#ifndef BASILISK_SIMPLEBATTERY_H
#define BASILISK_SIMPLEBATTERY_H


#include "../_GeneralModuleFiles/simPowerStorageBase.h"
#include "../../simMessages/powerStorageStatusSimMsg.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/*! \addtogroup SimModelGroup
 * @{
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

class SimpleBattery: public PowerStorageBase {

public:
    SimpleBattery();
    ~SimpleBattery();

private:
    void evaluateBatteryModel(PowerStorageStatusSimMsg *msg);

public:
    double storageCapacity; //!< [W-hr] Battery capacity in Watt-Hours.

};


#endif //BASILISK_SIMPLEBATTERY_H
