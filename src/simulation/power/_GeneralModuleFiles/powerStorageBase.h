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
#include "utilities/bskLogging.h"

#ifndef BASILISK_SIMPOWERSTORAGEBASE_H
#define BASILISK_SIMPOWERSTORAGEBASE_H


/*! @brief power storage base class */
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
    virtual void customSelfInit();//!< Custom SelfInit() method.  This allows a child class to add additional functionality to the SelfInit() method
    virtual void customCrossInit();//!< Custom CrossInit() method, similar to customSelfInit.
    virtual void customReset(uint64_t CurrentClock); //!< Custom Reset() method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t currentSimNanos); //!< Custom Write() method, similar to customSelfInit.
    virtual bool customReadMessages();//!< Custom Read() method, similar to customSelfInit.

public:
    std::vector<std::string> nodePowerUseMsgNames;    //!< Vector of power node input message names
    std::string batPowerOutMsgName; //!< Vector of message names to be written out by the battery
    double storedCharge_Init;//!< [W-s] Initial stored charge set by the user. Defaults to 0.
    BSKLogger bskLogger;                      //!< -- BSK Logging

protected:
    std::vector<std::int64_t> nodePowerUseMsgIds;               //!< class variable
    int64_t batPowerOutMsgId;                                   //!< class variable
    PowerStorageStatusSimMsg storageStatusMsg;                  //!< class variable
    std::vector<PowerNodeUsageSimMsg> nodeWattMsgs;             //!< class variable
    double previousTime; //!< Previous time used for integration
    double currentTimestep;//!< [s] Timestep duration in seconds.
    double storedCharge; //!< [W-s] Stored charge in Watt-hours.
    double currentPowerSum;//!< [W] Current net power.
    uint64_t outputBufferCount; //!< class variable

};


#endif //BASILISK_SIMPOWERSTORAGEBASE_H
