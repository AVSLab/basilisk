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

#ifndef BASILISK_DATANODEBASE_H
#define BASILISK_DATANODEBASE_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "_GeneralModuleFiles/sys_model.h"
#include <simMessages/dataNodeUsageSimMsg.h>
#include "simFswInterfaceMessages/deviceStatusIntMsg.h"
#include "simMessages/dataStorageStatusSimMsg.h"

/*! @brief data node base class */
class DataNodeBase: public SysModel {
public:
    DataNodeBase();
    ~DataNodeBase();
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    void computeDataStatus(double currentTime);
    void UpdateState(uint64_t CurrentSimNanos);

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    virtual void evaluateDataModel(DataNodeUsageSimMsg *dataUsageMsg, double currentTime)=0; //!< Virtual void method used to compute module-wise data usage/generation.
    virtual void customSelfInit();
    virtual void customCrossInit();   
    virtual void customReset(uint64_t CurrentClock); //!< Custom Reset method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t CurrentClock);//!< custom Write method, similar to customSelfInit.
    virtual bool customReadMessages(); //!< Custom read method, similar to customSelfInit; returns `true' by default.

public:
    std::string nodeDataOutMsgName; //!< Message name for the node's output message
    std::string nodeStatusInMsgName; //!< String for the message name that tells the node it's status
    double nodeBaudRate; //!< [baud] Data provided (+) or consumed (-).
    char nodeDataName[128]; //!< Name of the data node consuming or generating data.
    uint64_t dataStatus; //!< Device data mode; by default, 0 is off and 1 is on. Additional modes can fill other slots

protected:
    int64_t nodeDataOutMsgId;           //!< class variable
    int64_t nodeStatusInMsgId;          //!< class variable
    DataNodeUsageSimMsg nodeDataMsg;    //!< class variable
    DeviceStatusIntMsg nodeStatusMsg;   //!< class variable

private:
    uint64_t outputBufferCount;         //!< class variable
};

#endif //BASILISK_DATANODEBASE_H
