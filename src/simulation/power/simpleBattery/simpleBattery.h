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


#include "power/_GeneralModuleFiles/powerStorageBase.h"
#include "../../simMessages/powerStorageStatusSimMsg.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/*! \addtogroup SimModelGroup
 * @{
 */

/*! @brief Power storage module that considers net power up to some capacity.

 ## Module Purpose
 ### Executive Summary
    The SimpleBattery class is a minimal model of battery functionality that considers:
    1. Integrated net input power of the attached modules
    2. The battery's maximum storage capacity as defined by the `storageCapacity` attribute.
    Integration of the net input power is performed with a simple Euler method.
    \f[
         W_{stored} = \dot{W}_{net} (t_{current} - t_{previous})
    \f]

 ### Module Assumptions and Limitations
    See PowerStorageBase class for inherited assumption and limitations.  The SimpleBattery class assumes that the net energy storage amount is a fixed value.

 ### Message Connection Descriptions
    This module only uses the input and output messages of the PowerStorageBase base class.

 ## User Guide
    To set up this module users must create a simpleBattery instance.
    ~~~~~~~~~~{.py}
    battery = simpleBattery.SimpleBattery()
    battery.ModelTag = "batteryModel"
    ~~~~~~~~~~

    In addition to the variables that must be set for the PowerNodeBase base class, this module requires the `storageCapacity` attribute to be specified.  The total power stored in the battery will be limited to not exceed this capacity value.
    ~~~~~~~~~~{.py}
    battery.storageCapacity = 10.0 # Given in Joules or Watt-seconds
    ~~~~~~~~~~
    The next step is to attach one or more PowerNodeUsageSimMsg instances to it using the `addNodeToStorage()` method.
    ~~~~~~~~~~{.py}
    battery.addPowerNodeToModel("msg name")
    ~~~~~~~~~~
    The final step is to specify the output message name.
    ~~~~~~~~~~{.py}
    battery.batPowerOutMsgName= "outMsgName"
    ~~~~~~~~~~

    For more information on how to set up and use this module, see the simple power system example: @ref scenarioSimplePowerDemo

 */

class SimpleBattery: public PowerStorageBase {

public:
    SimpleBattery();
    ~SimpleBattery();

private:
    void customReset(uint64_t CurrentClock);
    void evaluateBatteryModel(PowerStorageStatusSimMsg *msg);

public:
    double storageCapacity; //!< [W-s] Battery capacity in Watt-seconds (Joules).

};

/*! @} */

#endif //BASILISK_SIMPLEBATTERY_H
