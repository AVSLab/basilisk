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
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/dataStorageStatusSimMsg.h"
#include "simMessages/dataNodeUsageSimMsg.h"

#ifndef BASILISK_DATASTORAGEUNITBASE_H
#define BASILISK_DATASTORAGEUNITBASE_H

//struct dataInstance{
//    std::string dataInstanceName;
//    double dataInstanceSum;
//}; //!< Struct for instances of data stored in a buffer. Includes names and amounts.

class DataStorageUnitBase: public SysModel {
public:
    DataStorageUnitBase();
    ~DataStorageUnitBase();
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    void addDataNodeToModel(std::string tmpNodeMsgName);
    void UpdateState(uint64_t CurrentSimNanos);
    std::vector<dataInstance> getStoredDataAll();
    double getStoredDataSum();
    DataStorageStatusSimMsg getStoredDataMsg();

protected:
    void writeMessages(uint64_t CurrentClock);
    bool readMessages();
    virtual void integrateDataStatus(double currentTime);
    virtual void customSelfInit(){};
    virtual void customCrossInit(){};
    virtual void customReset(uint64_t CurrentClock){};
    virtual void customWriteMessages(uint64_t CurrentClock){};
    virtual bool customReadMessages(){return true;};
    int messageInStoredData(DataNodeUsageSimMsg *tmpNodeMsg);
    double sumAllData();

public:
    std::vector<std::string> nodeDataUseMsgNames; //!< Vector of data node input message names
    std::string storageUnitDataOutMsgName; //!< Vector of message names to be written out by the storage unit
    double storedDataSum_Init;//!< [bits] Initial stored data set by the user. Defaults to 0.
    double storageCapacity; //! Storage capacity of the storage unit

protected:
    std::vector<std::int64_t> nodeDataUseMsgIds;
    int64_t storageUnitDataOutMsgId; //!< Message ID of storage Unit output message
    DataStorageStatusSimMsg storageStatusMsg;
    std::vector<DataNodeUsageSimMsg> nodeBaudMsgs;
    double storedDataSum; //!< [bits] Stored data in bits.
    std::vector<dataInstance> storedData; //! Vector of data. Represents the makeup of the data buffer.
    double previousTime; //! Previous time used for integration
    double currentTimestep;//! [s] Timestep duration in seconds.
    double netBaud; //! Net baud rate at a given time step
    uint64_t outputBufferCount;
};

#endif //BASILISK_DATASTORAGEUNITBASE_H
