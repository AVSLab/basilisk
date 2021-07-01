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
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/DataNodeUsageMsgPayload.h"
#include "architecture/msgPayloadDefC/DeviceCmdMsgPayload.h"
#include "architecture/msgPayloadDefCpp/DataStorageStatusMsgPayload.h"
#include "architecture/messaging/messaging.h"


/*! @brief data node base class */
class DataNodeBase: public SysModel {
public:
    DataNodeBase();
    ~DataNodeBase();
    void Reset(uint64_t CurrentSimNanos);
    void computeDataStatus(double currentTime);
    void UpdateState(uint64_t CurrentSimNanos);

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    virtual void evaluateDataModel(DataNodeUsageMsgPayload *dataUsageMsg, double currentTime)=0; //!< Virtual void method used to compute module-wise data usage/generation.
    virtual void customReset(uint64_t CurrentClock); //!< Custom Reset method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t CurrentClock);//!< custom Write method, similar to customSelfInit.
    virtual bool customReadMessages(); //!< Custom read method, similar to customSelfInit; returns `true' by default.

public:
    Message<DataNodeUsageMsgPayload> nodeDataOutMsg; //!< Message name for the node's output message
    ReadFunctor<DeviceCmdMsgPayload> nodeStatusInMsg; //!< String for the message name that tells the node it's status
    double nodeBaudRate; //!< [baud] Data provided (+) or consumed (-).
    char nodeDataName[128]; //!< Name of the data node consuming or generating data.
    uint64_t dataStatus; //!< Device data mode; by default, 0 is off and 1 is on. Additional modes can fill other slots

protected:
    DataNodeUsageMsgPayload nodeDataMsg;    //!< class variable
    DeviceCmdMsgPayload nodeStatusMsg;   //!< class variable
};

#endif //BASILISK_DATANODEBASE_H
