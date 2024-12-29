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

#include "simpleTransmitter.h"
#include "architecture/utilities/bskLogging.h"
#include <array>

/*! Constructor, which sets the default nodeDataOut to zero.
*/
SimpleTransmitter::SimpleTransmitter(){
    this->packetSize = 0.0;
    this->nodeBaudRate = 0.0;
    this->packetTransmitted = 0.0;
    this->previousTime = 0.0;
    this->numBuffers = 0;
    return;
}

SimpleTransmitter::~SimpleTransmitter(){

    return;
}

/*! Adds a dataStorageStatusMsgPayload to be accessed by transmitter.

 @param tmpStorageUnitMsg A spacecraft state message name.
 */
void SimpleTransmitter::addStorageUnitToTransmitter(Message<DataStorageStatusMsgPayload> *tmpStorageUnitMsg){
    this->storageUnitInMsgs.push_back(tmpStorageUnitMsg->addSubscriber());
    return;
}


bool SimpleTransmitter::customReadMessages(){

    DataStorageStatusMsgPayload nodeMsg;

    this->storageUnitMsgs.clear();

    //! - read in the data node use/supply messages
    bool dataRead = true;
    bool tmpDataRead;

    if(this->storageUnitInMsgs.size() > 0)
    {
        for(long unsigned int c=0; c<storageUnitInMsgs.size(); c++)
        {
            tmpDataRead = this->storageUnitInMsgs.at(c).isWritten();
            nodeMsg = this->storageUnitInMsgs.at(c)();
            dataRead = dataRead && tmpDataRead;

            this->storageUnitMsgs.push_back(nodeMsg);
        }
    }
    else {
        bskLogger.bskLog(BSK_INFORMATION, "Data storage has no data node messages to read.");
        dataRead = false;
    }

    return true;
}

/*! Loads the nodeDataOut attribute into the dataUsageSimMessage instance.
*/
void SimpleTransmitter::evaluateDataModel(DataNodeUsageMsgPayload *dataUsageSimMsg, double currentTime){

    this->currentTimestep = currentTime - this->previousTime;

    dataUsageSimMsg->baudRate = this->nodeBaudRate;

    this->currentIndex = this->getMaxIndex();

    //! - If we have not transmitted any of the packet, we select a new type of data to downlink
    if (this->currentIndex != -1) {
        if (this->packetTransmitted == 0.0) {

            // Set nodeDataName to the maximum data name
            strncpy(this->nodeDataName, this->storageUnitMsgs.back().storedDataName[this->currentIndex].c_str(),
                    sizeof(this->nodeDataName));

            // strncpy nodeDataName to the name of the output message
            strncpy(dataUsageSimMsg->dataName, this->nodeDataName, sizeof(dataUsageSimMsg->dataName));
            this->packetTransmitted += this->nodeBaudRate * (this->currentTimestep);

            // Check to see if maxVal is less than packet size.
            // If so, set the output message baudRate to zero
            // We do not want to start downlinking until we have enough data for one packet
            if (this->storageUnitMsgs.back().storedData[this->currentIndex] < (-1 * (this->packetSize))) {
                dataUsageSimMsg->baudRate = 0;
                this->packetTransmitted = 0;
            }
        } else {
            strncpy(dataUsageSimMsg->dataName, this->nodeDataName, sizeof(dataUsageSimMsg->dataName));
            this->packetTransmitted += this->nodeBaudRate * (this->currentTimestep);
            // If the transmitted packet size has exceeded the packet size, set packetTransmitted to zero
            // Both of these variables are negative so the comparison is non-intuitive
            if (this->packetTransmitted <= this->packetSize) {
                this->packetTransmitted = 0.0;
            }
        }
    } else {
        dataUsageSimMsg->baudRate = 0;
        this->packetTransmitted = 0;
    }
    this->previousTime = currentTime;
    return;

}

int SimpleTransmitter::getMaxIndex() {
    //! - Loop through storedData, select index w/ largest amt
    double maxVal = -1.0;
    int maxIndex = -1;
    for (long unsigned int i = 0; i < this->storageUnitMsgs.back().storedData.size(); i++) {
        if (this->storageUnitMsgs.back().storedData[i] > maxVal) {
            maxVal = this->storageUnitMsgs.back().storedData[i];
            maxIndex = (int) i;
        }
    }

    return maxIndex;
}
