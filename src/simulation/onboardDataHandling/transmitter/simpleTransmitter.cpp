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
#include <cstdio>

/*! Constructor, which sets the default nodeDataOut to zero.
 */
SimpleTransmitter::SimpleTransmitter()
{
    this->packetSize = 0.0;
    this->nodeBaudRate = 0.0;
    this->packetTransmitted = 0.0;
    this->numBuffers = 0;
    return;
}

SimpleTransmitter::~SimpleTransmitter()
{

    return;
}

/*! Adds a dataStorageStatusMsgPayload to be accessed by transmitter.

 @param tmpStorageUnitMsg A spacecraft state message name.
 */
void SimpleTransmitter::addStorageUnitToTransmitter(Message<DataStorageStatusMsgPayload>* tmpStorageUnitMsg)
{
    this->storageUnitInMsgs.push_back(tmpStorageUnitMsg->addSubscriber());
    this->packetTransmitted = 0.0; // [bit]
    return;
}

void SimpleTransmitter::customReset(uint64_t currentClock [[maybe_unused]])
{
    this->packetTransmitted = 0.0; // [bit]
    this->currentIndex = -1;
    this->nodeDataMsg = this->nodeDataOutMsg.zeroMsgPayload;
}

bool SimpleTransmitter::customReadMessages()
{
    const auto storageAddress = readStorageMessages(this->storageUnitInMsgs, this->storageUnitMsgs);
    if (this->storageUnitInMsgs.empty()) {
        this->bskLogger.bskLog(BSK_INFORMATION, "SimpleTransmitter has no storage unit attached.");
    }
    if (storageAddress != this->lastStorageAddress) {
        this->packetTransmitted = 0.0; // [bit]
        this->lastStorageAddress = storageAddress;
    }
    if (!storageAddress) {
        this->currentIndex = -1;
    }
    return storageAddress != 0;
}

/*! Loads the nodeDataOut attribute into the dataUsageSimMessage instance.
 */
void SimpleTransmitter::evaluateDataModel(DataNodeUsageMsgPayload* dataUsageSimMsg, double currentTime)
{

    this->currentTimestep = currentTime - this->previousUpdateTime;

    *dataUsageSimMsg = this->nodeDataOutMsg.zeroMsgPayload;
    dataUsageSimMsg->baudRate = this->nodeBaudRate;

    this->currentIndex = this->getMaxIndex();

    //! - If we have not transmitted any of the packet, we select a new type of data to downlink
    if (this->currentIndex != -1) {
        const size_t currentPosition = static_cast<size_t>(this->currentIndex);
        if (this->packetTransmitted == 0.0) {

            // Set nodeDataName to the maximum data name
            const std::string& storedDataName = this->storageUnitMsgs.back().storedDataName[currentPosition];
            if (storedDataName.size() >= sizeof(this->nodeDataName)) {
                bskLogger.bskError("SimpleTransmitter: storedDataName is %zu characters, but nodeDataName "
                                   "supports at most %zu characters.",
                                   storedDataName.size(),
                                   sizeof(this->nodeDataName) - 1);
            }
            std::snprintf(this->nodeDataName, sizeof(this->nodeDataName), "%s", storedDataName.c_str());

            // strncpy nodeDataName to the name of the output message
            std::snprintf(dataUsageSimMsg->dataName, sizeof(dataUsageSimMsg->dataName), "%s", this->nodeDataName);
            this->packetTransmitted += this->nodeBaudRate * (this->currentTimestep);

            // Check to see if maxVal is less than packet size.
            // If so, set the output message baudRate to zero
            // We do not want to start downlinking until we have enough data for one packet
            if (this->storageUnitMsgs.back().storedData[currentPosition] < (-1 * (this->packetSize))) {
                dataUsageSimMsg->baudRate = 0;
                this->packetTransmitted = 0;
            }
        } else {
            std::snprintf(dataUsageSimMsg->dataName, sizeof(dataUsageSimMsg->dataName), "%s", this->nodeDataName);
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
    return;
}

int SimpleTransmitter::getMaxIndex()
{
    //! - Loop through storedData, select index w/ largest amt
    double maxVal = -1.0;
    int maxIndex = -1;
    if (this->storageUnitMsgs.empty()) {
        return maxIndex;
    }
    for (long unsigned int i = 0; i < this->storageUnitMsgs.back().storedData.size(); i++) {
        if (this->storageUnitMsgs.back().storedData[i] > maxVal) {
            maxVal = this->storageUnitMsgs.back().storedData[i];
            maxIndex = (int)i;
        }
    }

    return maxIndex;
}
