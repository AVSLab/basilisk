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

#include "spaceToGroundTransmitter.h"
#include "architecture/utilities/bskLogging.h"
#include <array>
#include <iostream>

/*! Constructor, which sets the default nodeDataOut to zero.
*/
SpaceToGroundTransmitter::SpaceToGroundTransmitter(){
    this->packetSize = 0.0;
    this->nodeBaudRate = 0.0;
    this->packetTransmitted = 0.0;
    this->previousTime = 0.0;
    this->numBuffers = 0;
    return;
}

SpaceToGroundTransmitter::~SpaceToGroundTransmitter(){

    return;
}

/*! Adds a dataStorageStatusMsgPayload name to be accessed by transmitter.

 @param tmpStorageUnitMsg A spacecraft state message name.
 */
void SpaceToGroundTransmitter::addStorageUnitToTransmitter(Message<DataStorageStatusMsgPayload> *tmpStorageUnitMsg)
{
    this->storageUnitInMsgs.push_back(tmpStorageUnitMsg->addSubscriber());

    return;
}

/*! Adds a msg name to ground location access list

    @param tmpAccessMsg input name.
*/
void SpaceToGroundTransmitter::addAccessMsgToTransmitter(Message<AccessMsgPayload> *tmpAccessMsg)
{
    this->groundLocationAccessInMsgs.push_back(tmpAccessMsg->addSubscriber());
    return;
}


bool SpaceToGroundTransmitter::customReadMessages(){

    DataStorageStatusMsgPayload nodeMsg;
    AccessMsgPayload accessMsg;

    this->storageUnitMsgsBuffer.clear();
    this->groundLocationAccessMsgs.clear();

    //! - read in the data node use/supply messages
    bool dataRead = true;
    bool tmpDataRead;

    if(this->storageUnitInMsgs.size() > 0)
    {
        for(long unsigned int c=0; c<this->storageUnitInMsgs.size(); c++)
        {
            tmpDataRead = this->storageUnitInMsgs.at(c).isWritten();
            nodeMsg = this->storageUnitInMsgs.at(c)();

            dataRead = dataRead && tmpDataRead;

            this->storageUnitMsgsBuffer.push_back(nodeMsg);
        }
    }
    else {
        bskLogger.bskLog(BSK_INFORMATION, "Data storage has no data node messages to read.");
        dataRead = false;
    }
    if(this->groundLocationAccessInMsgs.size() > 0)
    {
        for(long unsigned int c=0; c<this->groundLocationAccessInMsgs.size(); c++)
        {
            tmpDataRead = this->groundLocationAccessInMsgs.at(c).isWritten();
            accessMsg = this->groundLocationAccessInMsgs.at(c)();
            dataRead = dataRead && tmpDataRead;

            this->groundLocationAccessMsgs.push_back(accessMsg);
        }
    }
    else {
        bskLogger.bskLog(BSK_INFORMATION, "SpaceToGroundTransmitter has no ground stations attached.");
        dataRead = false;
    }


    return true;
}

/*! Loads the nodeDataOut attribute into the dataUsageMessagePayload instance.
 @param dataUsageSimMsg
 @param currentTime
*/
void
SpaceToGroundTransmitter::evaluateDataModel(DataNodeUsageMsgPayload* dataUsageSimMsg, double currentTime)
{

    this->currentTimestep = currentTime - this->previousTime;

    dataUsageSimMsg->baudRate = this->nodeBaudRate;

    // Get the buffer with the most data
    double maxVal = -1.0;
    int maxIndex = -1;

    //! - If the transmitted packet size has exceeded the packet size, set packetTransmitted to zero
    // Both of these variables are negative so the comparison is non-intuitive
    if (this->packetTransmitted <= this->packetSize) {
        this->packetTransmitted = 0.0;
    }

    //! - If transmitted packet data is more than zero, continue downlinking from previous partition
    if (this->packetTransmitted != 0.0) {
        // Loop through the storageUnitMsgsBuffer to find the previous partition
        for (uint64_t i = 0; i < this->storageUnitMsgsBuffer.back().storedDataName.size(); i++) {
            if (this->storageUnitMsgsBuffer.back().storedDataName[i] == this->nodeDataName) {
                maxVal = this->storageUnitMsgsBuffer.back().storedData[i];
                maxIndex = (int)i;
            }
        }
        // If there is no data in the partition, reset maxVal, maxIndex, and packetTransmitted
        if (maxVal <= 0.0) {
            maxIndex = -1;
            this->packetTransmitted = 0.0;
            dataUsageSimMsg->baudRate = 0.0;
        }
    }

    //! - If there is no previous partition being downlinked, find the partition with the most data to downlink
    if (this->packetTransmitted == 0.0) {
        for (uint64_t i = 0; i < this->storageUnitMsgsBuffer.back().storedData.size(); i++) {
            if (this->storageUnitMsgsBuffer.back().storedData[i] > maxVal) {
                maxVal = this->storageUnitMsgsBuffer.back().storedData[i];
                maxIndex = (int)i;
            }
        }
    }

    //! - If we have access to any ground location, do the transmission logic
    if (std::any_of(this->groundLocationAccessMsgs.begin(),
                    this->groundLocationAccessMsgs.end(),
                    [](AccessMsgPayload msg) { return msg.hasAccess > 0; })) {
        // If an index was assigned
        if (maxIndex != -1) {
            //! - If we have not transmitted any of the packet, we select a new type of data to downlink
            if (this->packetTransmitted == 0.0) {
                // Set nodeDataName to the maximum data name
                strncpy(this->nodeDataName,
                        this->storageUnitMsgsBuffer.back().storedDataName[maxIndex].c_str(),
                        sizeof(this->nodeDataName));
                // strncpy nodeDataName to the name of the output message
                strncpy(dataUsageSimMsg->dataName, this->nodeDataName, sizeof(dataUsageSimMsg->dataName));

                // Check to see if maxVal is less than packet size. If not set the output message baudRate to zero
                // We do not want to start downlinking until we have enough data for one packet
                if (maxVal < (-1 * (this->packetSize))) {
                    dataUsageSimMsg->baudRate = 0;
                    this->packetTransmitted = 0;
                } else {
                    // If the downlink exceeds the available data, don't downlink
                    if ((maxVal + this->nodeBaudRate * (this->currentTimestep)) < 0) {
                        this->packetTransmitted = -maxVal;
                    } else {
                        // Otherwise, transmit with the nodeBaudRate
                        this->packetTransmitted += this->nodeBaudRate * (this->currentTimestep);
                    }
                }

            } else {
                strncpy(dataUsageSimMsg->dataName, this->nodeDataName, sizeof(dataUsageSimMsg->dataName));

                // If the downlink exceeds the available data, don't downlink
                if ((maxVal + this->nodeBaudRate * (this->currentTimestep)) < 0) {
                    this->packetTransmitted = -maxVal;
                } else {
                    // Otherwise, transmit with the nodeBaudRate
                    this->packetTransmitted += this->nodeBaudRate * (this->currentTimestep);
                }
            }

        } else {
            dataUsageSimMsg->baudRate = 0;
            this->packetTransmitted = 0;
        }
    }
    // If we don't have access, we can't transmit anything
    else {
        dataUsageSimMsg->baudRate = 0;
        this->packetTransmitted = 0;
    }
    this->previousTime = currentTime;
    return;
}
