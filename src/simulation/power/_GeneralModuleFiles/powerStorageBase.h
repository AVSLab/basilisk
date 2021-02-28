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
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/PowerStorageStatusMsgPayload.h"
#include "architecture/msgPayloadDefC/PowerNodeUsageMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

#ifndef BASILISK_SIMPOWERSTORAGEBASE_H
#define BASILISK_SIMPOWERSTORAGEBASE_H


/*! @brief power storage base class */
class PowerStorageBase: public SysModel  {
public:
    PowerStorageBase();
    ~PowerStorageBase();
    void Reset(uint64_t CurrentSimNanos);
    void addPowerNodeToModel(Message<PowerNodeUsageMsgPayload> *tmpNodeMsg);
    void UpdateState(uint64_t CurrentSimNanos);

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    void integratePowerStatus(double currentTime); //!< Integrates the net power given the current time using a simple Euler method.
    double sumAllInputs(); //!< Sums over the input power consumption messages.
    virtual void evaluateBatteryModel(PowerStorageStatusMsgPayload *msg) = 0; //!< Virtual function to represent power storage computation or losses.
    virtual void customReset(uint64_t CurrentClock); //!< Custom Reset() method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t currentSimNanos); //!< Custom Write() method, similar to customSelfInit.
    virtual bool customReadMessages();//!< Custom Read() method, similar to customSelfInit.

public:
    std::vector<ReadFunctor<PowerNodeUsageMsgPayload>> nodePowerUseInMsgs;    //!< Vector of power node input message names
    Message<PowerStorageStatusMsgPayload> batPowerOutMsg; //!< power storage status output message
    double storedCharge_Init;//!< [W-s] Initial stored charge set by the user. Defaults to 0.
    BSKLogger bskLogger;                      //!< -- BSK Logging

protected:
    PowerStorageStatusMsgPayload storageStatusMsg;                  //!< class variable
    std::vector<PowerNodeUsageMsgPayload> nodeWattMsgs;             //!< class variable
    double previousTime; //!< Previous time used for integration
    double currentTimestep;//!< [s] Timestep duration in seconds.
    double storedCharge; //!< [W-s] Stored charge in Watt-hours.
    double currentPowerSum;//!< [W] Current net power.
};


#endif //BASILISK_SIMPOWERSTORAGEBASE_H
