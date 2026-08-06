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

#include "dataStorageUnitBase.h"
#include "architecture/utilities/macroDefinitions.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>

namespace {

bool checkedAddInt64(int64_t left, int64_t right, int64_t& result)
{
    if ((right > 0 && left > std::numeric_limits<int64_t>::max() - right)
        || (right < 0 && left < std::numeric_limits<int64_t>::min() - right)) {
        return false;
    }
    result = left + right;
    return true;
}

}

/*! This method initializes some basic parameters for the module.

 */
DataStorageUnitBase::DataStorageUnitBase(){
    this->previousTime = 0; //! - previousTime initialized to 0.
    this->nodeDataUseInMsgs.clear(); //! - Clear the vector of input messages.
    this->storedDataSum = 0; //! - Initialize the dataSum to 0.
    this->netBaud = 0.0; //! - Initialize the netBaudRate to 0.
    //! - Zero out the partitions
    for(uint64_t i = 0; i < this->storedData.size(); i++){
        this->storedData[i].dataInstanceSum = 0;
    }
    return;
}

/*! Destructor.

 */
DataStorageUnitBase::~DataStorageUnitBase(){
    return;
}



/*! This method is used to reset the module.
 @param CurrentSimNanos

 */
void DataStorageUnitBase::Reset(uint64_t CurrentSimNanos)
{
    this->previousTime = 0;

    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}


/*! Adds a simDataNodeMsg name to be iterated over. Called in Python.
 @param tmpNodeMsg

 */
void DataStorageUnitBase::addDataNodeToModel(Message<DataNodeUsageMsgPayload> *tmpNodeMsg){
    this->nodeDataUseInMsgs.push_back(tmpNodeMsg->addSubscriber());

    return;
}

/*! Reads messages, adds new data to the storage unit, and writes out the storage unit status
 @param CurrentSimNanos The current simulation time in nanoseconds

 */
void DataStorageUnitBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - update data information
    if(this->readMessages())
    {
        this->integrateDataStatus(static_cast<double>(CurrentSimNanos) * NANO2SEC);
    } else {
        //! - Zero the output message if no input messages were received.
        this->storageStatusMsg = this->storageUnitDataOutMsg.zeroMsgPayload;
    }

    //! - write out the storage unit's data status
    this->writeMessages(CurrentSimNanos);

    return;
}

/*! This method is used to read the incoming data supply/outgoing data messages and store them for future use.

 */
bool DataStorageUnitBase::readMessages()
{
    DataNodeUsageMsgPayload nodeMsg;

    this->nodeBaudMsgs.clear();

    //! - read in the data node use/supply messages
    bool dataRead = true;
    bool tmpDataRead;
    if(this->nodeDataUseInMsgs.size() > 0)
    {
        for(long unsigned int c=0; c<this->nodeDataUseInMsgs.size(); c++)
        {
            nodeMsg = this->nodeDataUseInMsgs.at(c)();
            tmpDataRead = this->nodeDataUseInMsgs.at(c).isWritten();
            dataRead = dataRead && tmpDataRead;

            this->nodeBaudMsgs.push_back(nodeMsg);
        }
    }
    else {
        bskLogger.bskLog(BSK_INFORMATION, "Data storage has no data node messages to read.");
        dataRead = false;
    }

    //! - call the custom method to perform additional input reading
    bool customRead = customReadMessages();

    return(dataRead && customRead);
}

/*! Loops through the storedData vector and assigns values to output message.
 @param CurrentClock The current time used for time-stamping the message

 */
void DataStorageUnitBase::writeMessages(uint64_t CurrentClock){
    //! zero output message to begin with
    this->storageStatusMsg = this->storageUnitDataOutMsg.zeroMsgPayload;

    //! - Set first three message parameters
    this->storageStatusMsg.currentNetBaud = this->netBaud;
    this->storageStatusMsg.storageCapacity = static_cast<double>(this->storageCapacity);
    this->storageStatusMsg.storageLevel = static_cast<double>(this->storedDataSum);

    //! - Loop through stored data and copy over to the output message
    for(size_t i = 0; i < this->storedData.size(); i++){
        this->storageStatusMsg.storedDataName.push_back(this->storedData[i].dataInstanceName);
        this->storageStatusMsg.storedData.push_back(static_cast<double>(this->storedData[i].dataInstanceSum));
    }

    this->storageUnitDataOutMsg.write(&this->storageStatusMsg, this->moduleID, CurrentClock);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);
    return;
}

/*! Loops through all of the input messages, integrates the baud rates, and adds the new data to the storedData vector
 @param currentTime

 */
void DataStorageUnitBase::integrateDataStatus(double currentTime){
    int index = -1;
    this->currentTimestep = currentTime - this->previousTime;
    this->netBaud = 0;
    dataInstance tmpDataInstance;

    //! - loop over all the data nodes
    std::vector<DataNodeUsageMsgPayload>::iterator it;
    for(it = nodeBaudMsgs.begin(); it != nodeBaudMsgs.end(); it++) {
        if (std::memchr(it->dataName, '\0', sizeof(it->dataName)) == nullptr) {
            bskLogger.bskError("DataStorageUnitBase: dataName is not null-terminated within %zu characters.",
                             sizeof(it->dataName));
        }
        index = messageInStoredData(&(*it));
        const int64_t dataDelta = this->computeDataDelta(it->baudRate);

        //! - If the storage capacity has not been reached or the baudRate is less than 0 and won't take below 0, then add the data
       if (this->dataFitsInCapacity(dataDelta, true) || (it->baudRate < 0)) {
           //! - if a dataNode exists in storedData vector, integrate and add to current amount
           if (index != -1) {
               //! If this operation takes the sum below zero, set it to zero
               this->applyDataDelta(this->storedData[(size_t)index], dataDelta);
               //! - if a dataNode does not exist in storedData, add it to storedData, integrate baud rate, and add amount
           }
           else if (strcmp(it->dataName, "") != 0) {
               std::snprintf(tmpDataInstance.dataInstanceName,
                             sizeof(tmpDataInstance.dataInstanceName),
                             "%s",
                             it->dataName);
               tmpDataInstance.dataInstanceSum = dataDelta > 0 ? dataDelta : 0;
               this->storedData.push_back(tmpDataInstance);
           }
       }
        this->netBaud += it->baudRate;

        //! - Sum all data in storedData vector
        this->storedDataSum = this->sumAllData();
    }

    //! - Update previousTime
    this->previousTime = currentTime;
    return;
}

/*! Checks to see if a data node is in the storedData vector or not, returns the index.
 * @param tmpNodeMsg
 * @return index
 */
int DataStorageUnitBase::messageInStoredData(DataNodeUsageMsgPayload *tmpNodeMsg){
    // Initialize index as -1 (indicates data is not in storedData)
    int index = -1;

    // Loop through storedData. If dataName is found, set index = i
    for (size_t i = 0; i < this->storedData.size(); i++){
        if (strcmp(this->storedData[i].dataInstanceName, tmpNodeMsg->dataName) == 0){
            index = (int) i;
        }
    }
    return index;
}

/*! Sums all of the data in the storedData vector
 @return double
 */
int64_t DataStorageUnitBase::sumAllData(){
    int64_t dataSum = 0;

    std::vector<dataInstance>::iterator it;
    for(it = storedData.begin(); it != storedData.end(); it++) {
        int64_t updatedSum;
        if (!checkedAddInt64(dataSum, it->dataInstanceSum, updatedSum)) {
            bskLogger.bskError("DataStorageUnitBase: total stored data exceeds the supported int64_t range.");
        }
        dataSum = updatedSum;
    }

    return dataSum;
}

/*! Converts the integrated baud rate for the current time step to a bit count.
 @param baudRate Data production rate in bits per second.
 @return Integrated data change in bits.
 */
int64_t DataStorageUnitBase::computeDataDelta(double baudRate)
{
    const double roundedDataDelta = std::round(baudRate * this->currentTimestep);  // [bits]
    const double int64Limit = std::ldexp(1.0, 63);  // [bits]
    if (!std::isfinite(roundedDataDelta)
        || roundedDataDelta < -int64Limit
        || roundedDataDelta >= int64Limit) {
        bskLogger.bskError("DataStorageUnitBase: baud rate %.17g [bits/s] over timestep %.17g [s] "
                           "produces a data change outside the supported int64_t range.",
                           baudRate,
                           this->currentTimestep);
    }
    return static_cast<int64_t>(roundedDataDelta);
}

/*! Checks whether applying a data change keeps the total within storage capacity.
 @param dataDelta Data change in bits.
 @param allowEqual Whether a result equal to the capacity is accepted.
 @return True when the mathematical sum satisfies the capacity limit.
 */
bool DataStorageUnitBase::dataFitsInCapacity(int64_t dataDelta, bool allowEqual) const
{
    int64_t updatedSum;
    if (!checkedAddInt64(this->storedDataSum, dataDelta, updatedSum)) {
        return dataDelta < 0;
    }
    return allowEqual ? updatedSum <= this->storageCapacity : updatedSum < this->storageCapacity;
}

/*! Applies a data change to one partition without signed integer overflow.
 @param partition Partition to update.
 @param dataDelta Data change in bits.
 */
void DataStorageUnitBase::applyDataDelta(dataInstance& partition, int64_t dataDelta)
{
    int64_t updatedData;
    if (!checkedAddInt64(partition.dataInstanceSum, dataDelta, updatedData)) {
        if (dataDelta < 0) {
            partition.dataInstanceSum = 0;
            return;
        }
        bskLogger.bskError("DataStorageUnitBase: partition data exceeds the supported int64_t range.");
    }
    partition.dataInstanceSum = updatedData > 0 ? updatedData : 0;
}

/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method

 */
void DataStorageUnitBase::customReset(uint64_t CurrentClock [[maybe_unused]])
{
    return;
}

/*! custom Write method, similar to customSelfInit.

 */
void DataStorageUnitBase::customWriteMessages(uint64_t CurrentClock [[maybe_unused]])
{
    return;
}

/*! Custom read method, similar to customSelfInit; returns `true` by default.

 */
bool DataStorageUnitBase::customReadMessages()
{
    return true;
}

/*! Adds a specific amount of data to the storedData vector once
 @param partitionName //Name of the partition to add data to
 @param data          //Amount of data to add to the partition

 */
void DataStorageUnitBase::setDataBuffer(std::string partitionName, int64_t data)
{
    dataInstance tmpDataInstance;

    int index = -1;
    for (uint64_t i = 0; i < this->storedData.size(); i++){
        if (strcmp(this->storedData[i].dataInstanceName, partitionName.c_str()) == 0){
            index = (int) i;
        }
    }

    //! - If the new data won't overflow the storage capacity, then add the data
    if (this->dataFitsInCapacity(data, true)) {
        //! - if a dataNode exists in storedData vector, integrate and add to current amount
        if (index != -1) {
            //! Only perform if this operation will not take the sum below zero
            int64_t updatedData;
            if (checkedAddInt64(this->storedData[(size_t) index].dataInstanceSum, data, updatedData)) {
                if (updatedData >= 0) {
                    this->storedData[(size_t) index].dataInstanceSum = updatedData;
                }
            } else if (data > 0) {
                bskLogger.bskError("DataStorageUnitBase: partition data exceeds the supported int64_t range.");
            }

        }
        //! - if a dataNode does not exist in storedData, add it to storedData, and add amount
        else if (strcmp(partitionName.c_str(), "") != 0) {
            if (partitionName.size() >= sizeof(tmpDataInstance.dataInstanceName)) {
                bskLogger.bskError("DataStorageUnitBase: partitionName is %zu characters, but dataInstanceName "
                                 "supports at most %zu characters.",
                                 partitionName.size(), sizeof(tmpDataInstance.dataInstanceName) - 1);
            }
            std::snprintf(tmpDataInstance.dataInstanceName,
                          sizeof(tmpDataInstance.dataInstanceName),
                          "%s",
                          partitionName.c_str());
            //! Only perform this operation if the resulting sum in the partition is not negative. If it is, initialize to zero.
            if (data < 0) {
                data = 0;
            }
            tmpDataInstance.dataInstanceSum = data;
            this->storedData.push_back(tmpDataInstance);
        }
    }

    //! - Sum all data in storedData vector
    this->storedDataSum = this->sumAllData();
}
