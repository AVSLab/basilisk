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

/*! This method initializes some basic parameters for the module.
 @return void
 */
DataStorageUnitBase::DataStorageUnitBase(){
    this->previousTime = 0; //! - previousTime initialized to 0.
    this->nodeDataUseInMsgs.clear(); //! - Clear the vector of input messages.
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
    //! - call the custom SelfInit() method to add additional self initialization steps
    customSelfInit();

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
 @param tmpNodeMsg
 @return void
 */
void DataStorageUnitBase::addDataNodeToModel(Message<DataNodeUsageMsgPayload> *tmpNodeMsg){
    this->nodeDataUseInMsgs.push_back(tmpNodeMsg->addSubscriber());

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
        this->storageStatusMsg = this->storageUnitDataOutMsg.zeroMsgPayload();
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
    DataNodeUsageMsgPayload nodeMsg;

    this->nodeBaudMsgs.clear();

    //! - read in the data node use/supply messages
    bool dataRead = true;
    bool tmpDataRead;
    if(this->nodeDataUseInMsgs.size() > 0)
    {
        for(int c=0; c<this->nodeDataUseInMsgs.size(); c++)
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
 @return void
 */
void DataStorageUnitBase::writeMessages(uint64_t CurrentClock){

    //! zero output message to begin with
    this->storageStatusMsg = this->storageUnitDataOutMsg.zeroMsgPayload();

    //! - Set first three message parameters
    this->storageStatusMsg.currentNetBaud = this->netBaud;
    this->storageStatusMsg.storageCapacity = this->storageCapacity;
    this->storageStatusMsg.storageLevel = this->storedDataSum;

    //! - Loop through stored data and copy over to the output message
    for(uint64_t i = 0; i < this->storedData.size(); i++){
        strncpy(this->storageStatusMsg.storedDataName[i], this->storedData[i].dataInstanceName, sizeof(this->storageStatusMsg.storedDataName[i]));
        this->storageStatusMsg.storedData[i] = this->storedData[i].dataInstanceSum;
    }

    this->storageUnitDataOutMsg.write(&this->storageStatusMsg, this->moduleID, CurrentClock);

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
    std::vector<DataNodeUsageMsgPayload>::iterator it;
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
int DataStorageUnitBase::messageInStoredData(DataNodeUsageMsgPayload *tmpNodeMsg){
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
