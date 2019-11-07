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

#include "architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"
#include "utilities/linearAlgebra.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "simDataStorageUnitBase.h"

/*! This method initializes some basic parameters for the module.
 @return void
 */
DataStorageUnitBase::DataStorageUnitBase(){
    this->outputBufferCount = 2;
    this->previousTime = 0;
    this->nodeDataUseMsgNames.clear();
    this->storedDataSum = 0.0;
    this->storedDataSum_Init = 0.0;
    //this->storedData.clear();
    return;
}

/*! Destructor.
 @return void
 */
DataStorageUnitBase::~DataStorageUnitBase(){
    return;
}


/*! SelfInit for this class...
 @return void
 */
void DataStorageUnitBase::SelfInit(){
    //string MessageName, MaxSize, NumMessageBuffers, string messageStruct="",
    // How do we determine number of message buffers?
    this->storageUnitDataOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->storageUnitDataOutMsgName, sizeof(DataStorageStatusSimMsg),this->outputBufferCount, "DataStorageStatusSimMsg",this->moduleID);

    //! - call the custom SelfInit() method to add additional self initialization steps
    customSelfInit();

    return;
}

/*! CrossInit for this class...
 @return void
 */
void DataStorageUnitBase::CrossInit(){
    //! - subscribe to the spacecraft messages and create associated output message buffer
    std::vector<std::string>::iterator it;
    for(it = this->nodeDataUseMsgNames.begin(); it != this->nodeDataUseMsgNames.end(); it++){
        this->nodeDataUseMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(DataNodeUsageSimMsg), moduleID));
    }

    //for (auto it : this->nodeDataUseMsgNames)
    //{
    //}
    return;
}

/*! This method is used to reset the module.
 @return void
 */
void DataStorageUnitBase::Reset(uint64_t CurrentSimNanos)
{
    this->previousTime = 0;
    this->storedData.clear();

    if (this->storedDataSum_Init >= 0.0) {
        this->storedDataSum = this->storedDataSum_Init;
    } else {
        BSK_PRINT(MSG_ERROR, "The storedDataSum_Init variable must be set to a non-negative value.");
    }

    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}


/*! Adds a simDataNodeMsg name to be iterated over. - huh? tmpNodeMsgName to be iterated over?
 @return void
 @param tmpScMsgName A spacecraft state message name.
 */
void DataStorageUnitBase::addDataNodeToModel(std::string tmpNodeMsgName){
    this->nodeDataUseMsgNames.push_back(tmpNodeMsgName);
    return;
}

/*!
 * @param CurrentSimNanos The current simulation time in nanoseconds
 */
void DataStorageUnitBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - update data information
    if(this->readMessages())
    {
        this->integrateDataStatus(currentSimNanos*NANO2SEC);
    } else {
        /* zero the output message if no input messages were received. */
        memset(&(this->storageStatusMsg),  0x0, sizeof(DataStorageStatusSimMsg));
    }

    //! - write out neutral density message
    this->writeMessages(currentSimNanos);

    return;
}

/*! This method is used to read the incoming data supply/outgoing data messages and store them for future use.
 @return void
 */
bool DataStorageUnitBase::readMessages()
{
    DataNodeUsageSimMsg nodeMsg;
    SingleMessageHeader localHeader;

    this->nodeBaudMsgs.clear();

    //! - read in the data node use/supply messages
    bool dataRead = true;
    bool tmpDataRead;
    if(this->nodeDataUseMsgIds.size() > 0)
    {
        std::vector<int64_t>::iterator it;
        for(it = nodeDataUseMsgIds.begin(); it!= nodeDataUseMsgIds.end(); it++)
        {
            memset(&nodeMsg, 0x0, sizeof(DataNodeUsageSimMsg));
            tmpDataRead = SystemMessaging::GetInstance()->ReadMessage(*it, &localHeader,
                                                                       sizeof(DataNodeUsageSimMsg),
                                                                       reinterpret_cast<uint8_t*>(&nodeMsg),
                                                                       moduleID);
            dataRead = dataRead && tmpDataRead;

            this->nodeBaudMsgs.push_back(nodeMsg);
        }
    }
    else {
        BSK_PRINT(MSG_WARNING, "Data storage has no data node messages to read.");
        dataRead = false;
    }

    //! - call the custom method to perform additional input reading
    bool customRead = customReadMessages();

    return(dataRead && customRead);
}

/*!
 * @param CurrentClock The current time used for time-stamping the message
 */
void DataStorageUnitBase::writeMessages(uint64_t CurrentClock){
    SystemMessaging::GetInstance()->WriteMessage(this->storageUnitDataOutMsgId,
                                                 CurrentClock,
                                                 sizeof(DataStorageStatusSimMsg),
                                                 reinterpret_cast<uint8_t*> (&(this->storageStatusMsg)),
                                                 moduleID);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}


void DataStorageUnitBase::integrateDataStatus(double currentTime){
    int index = -1;
    double currentDataTmp = 0;
    this->currentTimestep = currentTime - this->previousTime;

    //! - loop over all the data nodes
    std::vector<DataNodeUsageSimMsg>::iterator it;
    for(it = nodeBaudMsgs.begin(); it != nodeBaudMsgs.end(); it++) {
        index = messageInStoredData(it);
        //! - if a dataNode exists in storedData vector, integrate and add to current amount
        if(index != -1){
            this->storedData[index] = {this->storedData[index].dataInstanceName, this->storedData[index].dataInstanceSum + it->baudRate * (this->currentTimestep)};
        //! - if a dataNode does not exist in storedData, add it to storedData, integrate baud rate, and add amount
        } else {
            this->storedData.pushback({it->dataName, it->baudRate * (this->currentTimestep)});
        }
    }

    // Sum all data in storedData vector
    this->storedDataSum = this->sumAllData();

    // Update previousTime
    this->previousTime = currentTime;
    return;
}

int messageInStoredData(DataNodeUsageSimMsg *tmpNodeMsg){
    // Initialize index as -1 (indicates data is not in storedData)
    int index = -1;

    // Loop through storedData. If dataName is found, set index = i
    for (i = 0; i < this->storedData.size(); i++){
        if (this->storedData[i].dataInstanceName == tmpNodeMsg->dataName){
            index = i;
        }
    }

    return index;
}

std::vector<dataInstance> getStoredData(){
    return this->storedData;
}

double sumAllData(){
    double dataSum = 0.0;

    std::vector<dataInstance>::iterator it;
    for(it = storedData.begin(); it != storedData.end(); it++) {
        dataSum += it->dataInstanceSum;
    }

    return dataSum;
}



