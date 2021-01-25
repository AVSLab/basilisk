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


#ifndef BASILISK_POWERNODEBASE_H
#define BASILISK_POWERNODEBASE_H



#include <Eigen/Dense>
#include <vector>
#include <string>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefC/PowerNodeUsageMsgPayload.h"
#include "architecture/msgPayloadDefC/DeviceStatusMsgPayload.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief power node base class */
class PowerNodeBase: public SysModel  {
public:
    PowerNodeBase();
    ~PowerNodeBase();
    void Reset(uint64_t CurrentSimNanos);
    void computePowerStatus(double currentTime);
    void UpdateState(uint64_t CurrentSimNanos);

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages(); 
    virtual void evaluatePowerModel(PowerNodeUsageMsgPayload *powerUsageMsg)=0; //!< Virtual void method used to compute module-wise power usage/generation.
    virtual void customReset(uint64_t CurrentClock); //!< Custom Reset method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t CurrentClock);//!< custom Write method, similar to customSelfInit.
    virtual bool customReadMessages(); //!< Custom read method, similar to customSelfInit; returns `true' by default.

public:
    Message<PowerNodeUsageMsgPayload> nodePowerOutMsg; //!< output power message
    ReadFunctor<DeviceStatusMsgPayload> nodeStatusInMsg; //!< note status input message
    double nodePowerOut; //!< [W] Power provided (+) or consumed (-).
    uint64_t powerStatus; //!< Device power mode; by default, 0 is off and 1 is on. Additional modes can fill other slots
    BSKLogger bskLogger;                      //!< -- BSK Logging

protected:
    PowerNodeUsageMsgPayload nodePowerMsg;          //!< buffer of output message
    DeviceStatusMsgPayload nodeStatusMsg;           //!< copy of input message
    double currentPowerConsumption;             //!< class variable
    double previousTime; //!< Previous time used for integration

};


#endif //BASILISK_POWERNODEBASE_H
