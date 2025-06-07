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
#include <iostream>

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
        this->integrateDataStatus(CurrentSimNanos*NANO2SEC);
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
    this->storageStatusMsg.storageCapacity = this->storageCapacity;
    this->storageStatusMsg.storageLevel = this->storedDataSum;

    //! - Loop through stored data and copy over to the output message
    for(uint64_t i = 0; i < this->storedData.size(); i++){
        this->storageStatusMsg.storedDataName.push_back(this->storedData[i].dataInstanceName);
        this->storageStatusMsg.storedData.push_back(this->storedData[i].dataInstanceSum);
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
        index = messageInStoredData(&(*it));

        //! - If the storage capacity has not been reached or the baudRate is less than 0 and won't take below 0, then add the data
       if ((this->storedDataSum + round(it->baudRate * this->currentTimestep) <= this->storageCapacity) || (it->baudRate < 0)) {
           //! - if a dataNode exists in storedData vector, integrate and add to current amount
           if (index != -1) {
               //! If this operation takes the sum below zero, set it to zero
               if ((this->storedData[(size_t)index].dataInstanceSum + it->baudRate * this->currentTimestep) >= 0) {
                   this->storedData[(size_t)index].dataInstanceSum += round(it->baudRate * this->currentTimestep);
               } else {
                   this->storedData[(size_t)index].dataInstanceSum = 0;
               }
               //! - if a dataNode does not exist in storedData, add it to storedData, integrate baud rate, and add amount
           }
           else if (strcmp(it->dataName, "") != 0) {
               strncpy(tmpDataInstance.dataInstanceName, it->dataName, sizeof(tmpDataInstance.dataInstanceName));
               tmpDataInstance.dataInstanceSum = round(it->baudRate * (this->currentTimestep));
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
    for (uint64_t i = 0; i < this->storedData.size(); i++){
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
    double dataSum = 0;

    std::vector<dataInstance>::iterator it;
    for(it = storedData.begin(); it != storedData.end(); it++) {
        dataSum += it->dataInstanceSum;
    }

    return dataSum;
}

/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method

 */
void DataStorageUnitBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! custom Write method, similar to customSelfInit.

 */
void DataStorageUnitBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! Custom read method, similar to customSelfInit; returns `true' by default.

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
    if (this->storedDataSum + data <= this->storageCapacity) {
        //! - if a dataNode exists in storedData vector, integrate and add to current amount
        if (index != -1) {
            //! Only perform if this operation will not take the sum below zero
            if ((this->storedData[(size_t) index].dataInstanceSum + data) >= 0) {
                this->storedData[(size_t) index].dataInstanceSum += data;
            }

        }
        //! - if a dataNode does not exist in storedData, add it to storedData, and add amount
        else if (strcmp(partitionName.c_str(), "") != 0) {
            strncpy(tmpDataInstance.dataInstanceName, partitionName.c_str(), sizeof(tmpDataInstance.dataInstanceName));
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
