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

#include "messaging/system_messaging.h"
#include "dataStorageUnitBase.h"
#include "utilities/macroDefinitions.h"

/*! This method initializes some basic parameters for the module.
 @return void
 */
DataStorageUnitBase::DataStorageUnitBase(){
    this->outputBufferCount = 2;
    this->previousTime = 0; //! - previousTime initialized to 0.
    this->nodeDataUseMsgNames.clear(); //! - Clear the MsgNames.
    this->storedDataSum = 0.0; //! - Initialize the dataSum to 0.
    this->netBaud = 0.0; //! - Initialize the netBaudRate to 0.
    return;
}

/*! Destructor.
 @return void
 */
DataStorageUnitBase::~DataStorageUnitBase(){
    return;
}


/*! SelfInit for this class.
 @return void
 */
void DataStorageUnitBase::SelfInit(){
    this->storageUnitDataOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->storageUnitDataOutMsgName, sizeof(DataStorageStatusSimMsg),this->outputBufferCount, "DataStorageStatusSimMsg",this->moduleID);

    //! - call the custom SelfInit() method to add additional self initialization steps
    customSelfInit();

    return;
}

/*! CrossInit for this class. Subscribes to dataNodes that will send data to storage unit.
 @return void
 */
void DataStorageUnitBase::CrossInit(){
    //! - Subscribe to the data node messages and create associated output message buffer by iterating through the vector of messages to subscribe to.
    std::vector<std::string>::iterator it;
    for(it = this->nodeDataUseMsgNames.begin(); it != this->nodeDataUseMsgNames.end(); it++){
        this->nodeDataUseMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(DataNodeUsageSimMsg), moduleID));
    }

    //!- call the custom CrossInit() method to all additional cross initialization steps
    customCrossInit();

    return;
}

/*! This method is used to reset the module.
 @param CurrentSimNanos
 @return void
 */
void DataStorageUnitBase::Reset(uint64_t CurrentSimNanos)
{
    this->previousTime = 0;
    this->storedData.clear();

    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}


/*! Adds a simDataNodeMsg name to be iterated over. Called in Python.
 @param tmpNodeMsgName
 @return void
 */
void DataStorageUnitBase::addDataNodeToModel(std::string tmpNodeMsgName){
    this->nodeDataUseMsgNames.push_back(tmpNodeMsgName);
    return;
}

/*! Reads messages, adds new data to the storage unit, and writes out the storage unit status
 @param CurrentSimNanos The current simulation time in nanoseconds
 @return void
 */
void DataStorageUnitBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - update data information
    if(this->readMessages())
    {
        this->integrateDataStatus(CurrentSimNanos*NANO2SEC);
    } else {
        //! - Zero the output message if no input messages were received.
        memset(&(this->storageStatusMsg),  0x0, sizeof(DataStorageStatusSimMsg));
    }

    //! - write out the storage unit's data status
    this->writeMessages(CurrentSimNanos);

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
        bskLogger.bskLog(BSK_INFORMATION, "Data storage has no data node messages to read.");
        dataRead = false;
    }

    //! - call the custom method to perform additional input reading
    bool customRead = customReadMessages();

    return(dataRead && customRead);
}

/*! Loops through the storedData vector and assigns values to output message.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void DataStorageUnitBase::writeMessages(uint64_t CurrentClock){

    //! - Set first three message parameters
    this->storageStatusMsg.currentNetBaud = this->netBaud;
    this->storageStatusMsg.storageCapacity = this->storageCapacity;
    this->storageStatusMsg.storageLevel = this->storedDataSum;

    //! - Loop through stored data and copy over to the output message
    for(uint64_t i = 0; i < this->storedData.size(); i++){
        strncpy(this->storageStatusMsg.storedDataName[i], this->storedData[i].dataInstanceName, sizeof(this->storageStatusMsg.storedDataName[i]));
        this->storageStatusMsg.storedData[i] = this->storedData[i].dataInstanceSum;
    }

    SystemMessaging::GetInstance()->WriteMessage(this->storageUnitDataOutMsgId,
                                                 CurrentClock,
                                                 sizeof(DataStorageStatusSimMsg),
                                                 reinterpret_cast<uint8_t*> (&(this->storageStatusMsg)),
                                                 moduleID);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}

/*! Loops through all of the input messages, integrates the baud rates, and adds the new data to the storedData vector
 @param currentTime
 @return void
 */
void DataStorageUnitBase::integrateDataStatus(double currentTime){
    int index = -1;
    this->currentTimestep = currentTime - this->previousTime;
    this->netBaud = 0;
    dataInstance tmpDataInstance;

    //! - loop over all the data nodes
    std::vector<DataNodeUsageSimMsg>::iterator it;
    for(it = nodeBaudMsgs.begin(); it != nodeBaudMsgs.end(); it++) {
        index = messageInStoredData(&(*it));

        //! - If the storage capacity has not been reached or the baudRate is less than 0, then add the data
       if ((this->storedDataSum < this->storageCapacity) || (it->baudRate < 0)) {
           //! - if a dataNode exists in storedData vector, integrate and add to current amount
           if (index != -1) {
               this->storedData[index].dataInstanceSum += it->baudRate * (this->currentTimestep);
               //! - if a dataNode does not exist in storedData, add it to storedData, integrate baud rate, and add amount
           }
           else {
               strncpy(tmpDataInstance.dataInstanceName, it->dataName, sizeof(tmpDataInstance.dataInstanceName));
               tmpDataInstance.dataInstanceSum = it->baudRate * (this->currentTimestep);
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
int DataStorageUnitBase::messageInStoredData(DataNodeUsageSimMsg *tmpNodeMsg){
    // Initialize index as -1 (indicates data is not in storedData)
    int index = -1;

    // Loop through storedData. If dataName is found, set index = i
    for (uint64_t i = 0; i < this->storedData.size(); i++){
        if (strcmp(this->storedData[i].dataInstanceName, tmpNodeMsg->dataName) == 0){
            index = i;
        }
    }

    return index;
}

/*! Getter function for all of the stored data
 @return std::vector<dataInstance>
 */
std::vector<dataInstance> DataStorageUnitBase::getStoredDataAll(){
    return this->storedData;
}

/*! Getter function for sum of the stored data
 @return double
 */
double DataStorageUnitBase::getStoredDataSum(){
    return this->storedDataSum;
}

/*! Sums all of the data in the storedData vector
 @return double
 */
double DataStorageUnitBase::sumAllData(){
    double dataSum = 0.0;

    std::vector<dataInstance>::iterator it;
    for(it = storedData.begin(); it != storedData.end(); it++) {
        dataSum += it->dataInstanceSum;
    }

    return dataSum;
}




/*! Custom SelfInit() method.  This allows a child class to add additional functionality to the SelfInit() method
 @return void
 */
void DataStorageUnitBase::customSelfInit()
{
    return;
}

/*! Custom CrossInit() method.  This allows a child class to add additional functionality to the CrossInit() method
 @return void
 */
void DataStorageUnitBase::customCrossInit()
{
    return;
}

/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method
 @return void
 */
void DataStorageUnitBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! custom Write method, similar to customSelfInit.
 @return void
 */
void DataStorageUnitBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! Custom read method, similar to customSelfInit; returns `true' by default.
 @return void
 */
bool DataStorageUnitBase::customReadMessages()
{
    return true;
}
