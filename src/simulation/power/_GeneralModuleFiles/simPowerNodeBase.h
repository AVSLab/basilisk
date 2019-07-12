//
// Created by andrew on 7/12/19.
//

#ifndef BASILISK_SIMPOWERNODEBASE_H
#define BASILISK_SIMPOWERNODEBASE_H


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
#include "simMessages/powerNodeUsageSimMsg.h"
#include "simFswInterfaceMessages/powerNodeStatusIntMsg.h"

/*! \addtogroup SimModelGroup
 * @{
 */



//! @brief General power source/sink base class.


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
    void writeMessage(uint64_t CurrentClock);
    bool readMessages();
    void evaluatePowerUsage(PowerNodeUsageSimMsg *powerUsageMsg);
    virtual void customPowerModel() = 0;
    virtual void customSelfInit();
    virtual void customCrossInit();
    virtual void customReset(uint64_t CurrentClock);
    virtual void customWriteMessages(uint64_t CurrentClock);
    virtual bool customReadMessages();

public:
    std::string nodePowerOutMsgName; //!< Message name for the node's output message
    std::string nodeStatusInMsgName; //!< String for the message name that tells the node it's status
    int_64_t nodePowerOutMsgId;
    int_64_t nodeStatusInMsgId;
    double nodePowerOut; //!< [W] Power provided (+) or consumed (-).
    uint_8_t powerStatus; //!< Device power mode; by default, 0 is off and 1 is on. Additional modes can fill other slots

private:
    PowerNodeUsageSimMsg nodePowerMsg;
    PowerNodeStatusIntMsg nodeStatusMsg;
    double currentPowerConsumption;

    double previousTime; //! Previous time used for integration

};


#endif //BASILISK_SIMPOWERNODEBASE_H
