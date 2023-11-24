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
#include <cstring>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/DataNodeUsageMsgPayload.h"
#include "architecture/msgPayloadDefCpp/DataStorageStatusMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

#ifndef BASILISK_DATASTORAGEUNITBASE_H
#define BASILISK_DATASTORAGEUNITBASE_H

struct dataInstance {
    char dataInstanceName[128];     //!< data instance name
    int64_t dataInstanceSum;         //!< data instance sum value, bits
}; //!< Struct for instances of data stored in a buffer. Includes names and amounts.

/*! @brief on-board data handling base class */
class DataStorageUnitBase: public SysModel {
public:
    DataStorageUnitBase();
    ~DataStorageUnitBase();
    void Reset(uint64_t CurrentSimNanos);
    void addDataNodeToModel(Message<DataNodeUsageMsgPayload> *tmpNodeMsg); //!< Adds dataNode to the storageUnit
    void UpdateState(uint64_t CurrentSimNanos);
    void setDataBuffer(std::string partitionName, int64_t data); //!< Adds/removes the data from the partitionName partition once

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    virtual void integrateDataStatus(double currentTime); //!< Integrates the dataStatus over all of the dataNodes
    virtual void customReset(uint64_t CurrentClock); //!< Custom Reset method, similar to customSelfInit.
    virtual void customWriteMessages(uint64_t CurrentClock); //!< custom Write method, similar to customSelfInit.
    virtual bool customReadMessages(); //!< Custom read method, similar to customSelfInit; returns `true' by default.
    int messageInStoredData(DataNodeUsageMsgPayload *tmpNodeMsg); //!< Returns index of the dataName if it's already in storedData
    int64_t sumAllData(); //!< Sums all of the data in the storedData vector

public:
    std::vector<ReadFunctor<DataNodeUsageMsgPayload>> nodeDataUseInMsgs; //!< Vector of data node input message names
    Message<DataStorageStatusMsgPayload> storageUnitDataOutMsg; //!< Vector of message names to be written out by the storage unit
    int64_t storageCapacity; //!< Storage capacity of the storage unit
    BSKLogger bskLogger;    //!< logging variable

protected:
    DataStorageStatusMsgPayload storageStatusMsg; //!< class variable
    std::vector<DataNodeUsageMsgPayload> nodeBaudMsgs; //!< class variable
    int64_t storedDataSum; //!< [bits] Stored data in bits.
    std::vector<dataInstance> storedData; //!< Vector of data. Represents the makeup of the data buffer.
    double previousTime; //!< Previous time used for integration
    double currentTimestep;//!< [s] Timestep duration in seconds.
    double netBaud; //!< Net baud rate at a given time step
};

#endif //BASILISK_DATASTORAGEUNITBASE_H
