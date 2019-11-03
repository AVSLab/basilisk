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

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/powerStorageStatusSimMsg.h"
#include "simMessages/powerNodeUsageSimMsg.h"

#ifndef BASILISK_SIMPOWERSTORAGEBASE_H
#define BASILISK_SIMPOWERSTORAGEBASE_H

/*!
 
 \verbatim embed:rst

 Executive Summary
    The PowerStorageBase is a base class that is used generate a standard interface and list of features for modules that store electrical power.  This class is used by other modules as a parent class and cannot be instantiated by itself.  All Basilisk power storateg modules based on this PowerStorageBase inherit the following common properties:

    1. Writes out a :ref:`PowerStorageStatusSimMsg` containing the current stored power (in Watt-Seconds or Joules), the current net power (in Watts), and the battery storage capacity (in Watt-Seconds or Joules).
    2. Allows for multiple :ref:`PowerNodeUsageSimMsg` corresponding to individual :ref:`simPowerNodeBase` instances to be subscribed to using the addPowerNodeToModel method.
    3. Iterates through attached :ref:`PowerNodeUsageSimMsg` instances and computes the net power over all messages using ``sumAllInputs()``
    4. Computes the conversion between net power in and storage using the ``evaluateBatteryModel`` method, which must be overriden in child classes and is therefore module-specific.

    Core functionality is wrapped in the ``evaluateBatteryModel`` protected virtual void method, which is assumed to compute power storage on a module specific mathematical model.
    
    Protected methods prepended with "custom" are intended for module developers to override with additional, module-specific functionality.

    For more information on how to set up and use classes derived from this module, see the simple power system example: :ref:`scenarioPowerDemo`

 Module Assumptions and Limitations
    The base class makes no specific energy storate device related assumptions.

 Message Connection Descriptions
    The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

    .. table:: Module I/O Messages
        :widths: 25 25 100

        +-----------------------+---------------------------------+---------------------------------------------------+
        | Msg Variable Name     | Msg Type                        | Description                                       |
        +=======================+=================================+===================================================+
        | nodePowerUseMsgNames  | :ref:`powerNodeUsageSimMsg`     | Input messages. Vector of power node usage        |
        |                       |                                 | usage messages. Set using ``addPowerNodeToModel`` |
        +-----------------------+---------------------------------+---------------------------------------------------+
        | batPowerOutMsgName    | :ref:`PowerStorageStatusSimMsg` | Output message. Describes battery                 |
        |                       |                                 | capacity, charge level, net power.                |
        +-----------------------+---------------------------------+---------------------------------------------------+

 User Guide
    - The base class behavior requires the initial energy storage to be specified through ``storedCharge_Init``.
    - The integration time step is evaluated as the time between module calls.
    - The user must set the output message name variable ``batPowerOutMsgName``
    - The input message names are provided by calling the method ``addPowerNodeToModel()``

 \endverbatim
 */

class PowerStorageBase: public SysModel  {
public:
    PowerStorageBase();
    ~PowerStorageBase();
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    void addPowerNodeToModel(std::string tmpNodeMsgName); //!< Function to add additional power devices to the storage module.
    void UpdateState(uint64_t CurrentSimNanos);

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    void integratePowerStatus(double currentTime); //!< Integrates the net power given the current time using a simple Euler method.
    double sumAllInputs(); //!< Sums over the input power consumption messages.
    virtual void evaluateBatteryModel(PowerStorageStatusSimMsg *msg) = 0; //!< Virtual function to represent power storage computation or losses.
    virtual void customSelfInit(){};//!Custom SelfInit() method.  This allows a child class to add additional functionality to the SelfInit() method
    virtual void customCrossInit(){};//! Custom CrossInit() method, similar to customSelfInit.
    virtual void customReset(uint64_t CurrentClock){}; //! Custom Reset() method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t currentSimNanos){}; //! Custom Write() method, similar to customSelfInit.
    virtual bool customReadMessages(){return true;};//! Custom Read() method, similar to customSelfInit.

public:
    std::vector<std::string> nodePowerUseMsgNames;    //!< Vector of power node input message names
    std::string batPowerOutMsgName; //!< Vector of message names to be written out by the battery
    double storedCharge_Init;//!< [W-s] Initial stored charge set by the user. Defaults to 0.

protected:
    std::vector<std::int64_t> nodePowerUseMsgIds;
    int64_t batPowerOutMsgId;
    PowerStorageStatusSimMsg storageStatusMsg;
    std::vector<PowerNodeUsageSimMsg> nodeWattMsgs;
    double previousTime; //! Previous time used for integration
    double currentTimestep;//! [s] Timestep duration in seconds.
    double storedCharge; //!< [W-s] Stored charge in Watt-hours.
    double currentPowerSum;//!< [W] Current net power.
    uint64_t outputBufferCount;

};


#endif //BASILISK_SIMPOWERSTORAGEBASE_H
