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


#ifndef BASILISK_SIMPLEPOWERMONITOR_H
#define BASILISK_SIMPLEPOWERMONITOR_H

#include "power/_GeneralModuleFiles/powerStorageBase.h"
#include "../../simMessages/powerStorageStatusSimMsg.h"

/*! \addtogroup SimModelGroup
* @{
*/

/*! @brief  Simple battery model with unlimited capacity.

 ## Module Purpose
 ### Executive Summary

    The SimplePowerMonitor class is a minimal representation of the PowerStorageBase base class that could represent an electrical battery.  It tracks the integrated net power of a set of attached devices analagous to the behavior of the SimpleBattery module. The main difference is that SimplePowerMonitor does not limit the amount of energy that can be stored.  Thus, it is useful as a power monitor module that simple tracks the positive or negative net energy usage history.  The functionality includes:

    1. Compute the integrated and instantaneous net power of all attached devices
    Integration is performed with a simple Euler method.
    \f[
         W_{stored} = \dot{W}_{net} (t_{current} - t_{previous})
    \f]

 ### Module Assumptions and Limitations
    See PowerStorageBase class for inherited assumption and limitations.  The SimpleBattery class assumes that the net energy storage amount is a fixed value.

 ### Message Connection Descriptions
    This module only uses the input and output messages of the PowerStorageBase base class.  As in this module the storage capacity is not used, the output message sets the variable `storageCapacity` to -1.

 ## User Guide
    To set up this module users must create a simpleBattery instance.
    ~~~~~~~~~~{.py}
    battery = simplePowerMonitor.SimplePowerMonitor()
    battery.ModelTag = "powerMonitorModel"
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

class SimplePowerMonitor: public PowerStorageBase {

public:
    SimplePowerMonitor();
    ~SimplePowerMonitor();

private:
    void evaluateBatteryModel(PowerStorageStatusSimMsg *msg);


};

/*! @} */

#endif //BASILISK_SIMPLEPOWERMONITOR_H
