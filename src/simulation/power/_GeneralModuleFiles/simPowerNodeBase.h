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


#ifndef BASILISK_SIMPOWERNODEBASE_H
#define BASILISK_SIMPOWERNODEBASE_H



#include <Eigen/Dense>
#include <vector>
#include <string>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/powerNodeUsageSimMsg.h"
#include "simFswInterfaceMessages/powerNodeStatusIntMsg.h"

/*! \addtogroup SimModelGroup
 * @{
 */



//! @brief General power source/sink base class.
/*! The powerNodeBase class is used generate a standard interface and list of features for modules that consume or provide power. 
Specifically, each PowerNodeBase:

1. Writes out a PowerNodeUsageSimMsg describing its power consumption at each sim update;
2. Can be switched on or off using a PowerNodeStatusIntMsg.

Core functionality is wrapped in the evaluatePowerModel protected virtual void method, which is assumed to compute power usage based on a module specific mathematical model. 
Protected methods prepended with "custom" are intended for module developers to override with additional, module-specific functionality. */


class PowerNodeBase: public SysModel  {
public:
    PowerNodeBase();
    ~PowerNodeBase();
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    void computePowerStatus(double currentTime);
    void UpdateState(uint64_t CurrentSimNanos);

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages(); 
    virtual void evaluatePowerModel(PowerNodeUsageSimMsg *powerUsageMsg)=0; //!< Virtual void method used to compute module-wise power usage/generation.
    virtual void customSelfInit(){};//! Custom output input reading method.  This allows a child class to add additional functionality.
    virtual void customCrossInit(){}; //! Custom subscription method, similar to customSelfInit.
    virtual void customReset(uint64_t CurrentClock){}; //! Custom Reset method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t CurrentClock){};//! custom Write method, similar to customSelfInit.
    virtual bool customReadMessages(){return true;} //! Custom read method, similar to customSelfInit; returns `true' by default.

public:
    std::string nodePowerOutMsgName; //!< Message name for the node's output message
    std::string nodeStatusInMsgName; //!< String for the message name that tells the node it's status
    int64_t nodePowerOutMsgId;
    int64_t nodeStatusInMsgId;
    double nodePowerOut; //!< [W] Power provided (+) or consumed (-).
    uint64_t powerStatus; //!< Device power mode; by default, 0 is off and 1 is on. Additional modes can fill other slots

protected:
    PowerNodeUsageSimMsg nodePowerMsg;
    PowerNodeStatusIntMsg nodeStatusMsg;
    double currentPowerConsumption;
    double previousTime; //! Previous time used for integration

private:
    uint64_t outputBufferCount;


};


#endif //BASILISK_SIMPOWERNODEBASE_H
