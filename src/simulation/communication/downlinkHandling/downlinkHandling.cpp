/*
 ISC License

Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "simulation/communication/downlinkHandling/downlinkHandling.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "architecture/utilities/macroDefinitions.h"

/*! Constructor */
DownlinkHandling::DownlinkHandling()
{
    this->bitRateRequest = 0.0;
    this->packetSizeBits = 256.0;
    this->maxRetransmissions = 10;
    this->receiverAntenna = 0;
    this->requireFullPacket = true;

    this->previousTime = 0.0;
    this->currentTimeStep = 0.0;
    this->availableDataBits = 0.0;

    this->cumulativeAttemptedBits = 0.0;
    this->cumulativeRemovedBits = 0.0;
    this->cumulativeDeliveredBits = 0.0;
    this->cumulativeDroppedBits = 0.0;

    this->linkBudgetValid = false;
    this->linkBudgetBuffer = {};
    this->downlinkOutBuffer = this->downlinkOutMsg.zeroMsgPayload;

    std::strncpy(this->nodeDataName, "STORED DATA", sizeof(this->nodeDataName) - 1);
    this->nodeDataName[sizeof(this->nodeDataName) - 1] = '\0';
}

/*! Add a storage-status message to the module input list */
void DownlinkHandling::addStorageUnitToDownlink(Message<DataStorageStatusMsgPayload> *tmpStorageUnitMsg)
{
    this->storageUnitInMsgs.push_back(tmpStorageUnitMsg->addSubscriber());
}

/*! Module reset hook */
void DownlinkHandling::customReset(uint64_t CurrentClock)
{
    this->previousTime = static_cast<double>(CurrentClock) * NANO2SEC;
    this->currentTimeStep = 0.0;

    this->cumulativeAttemptedBits = 0.0;
    this->cumulativeRemovedBits = 0.0;
    this->cumulativeDeliveredBits = 0.0;
    this->cumulativeDroppedBits = 0.0;

    this->downlinkOutBuffer = this->downlinkOutMsg.zeroMsgPayload;

    if (this->bitRateRequest < 0.0) {
        bskLogger.bskLog(BSK_ERROR, "DownlinkHandling bitRateRequest must be >= 0 [bit/s].");
        this->bitRateRequest = 0.0;
    }
    if (this->packetSizeBits <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "DownlinkHandling packetSizeBits must be > 0 [bit].");
    }
    if (this->maxRetransmissions < 1) {
        bskLogger.bskLog(BSK_WARNING, "DownlinkHandling maxRetransmissions must be >= 1. Setting to 1.");
        this->maxRetransmissions = 1;
    }
    if (this->receiverAntenna != 0 && this->receiverAntenna != 1 && this->receiverAntenna != 2) {
        bskLogger.bskLog(BSK_WARNING, "DownlinkHandling receiverAntenna must be 0 (auto), 1, or 2. Setting to auto.");
        this->receiverAntenna = 0;
    }
    if (!this->linkBudgetInMsg.isLinked()) {
        bskLogger.bskLog(BSK_WARNING, "DownlinkHandling linkBudgetInMsg is not linked.");
    }
    if (this->storageUnitInMsgs.empty()) {
        bskLogger.bskLog(BSK_WARNING, "DownlinkHandling has no storageUnitInMsgs linked.");
    }
}

/*! Read custom input messages */
bool DownlinkHandling::customReadMessages()
{
    this->storageUnitMsgsBuffer.clear();
    for (auto &msg : this->storageUnitInMsgs) {
        this->storageUnitMsgsBuffer.push_back(msg());
    }

    this->linkBudgetValid = this->linkBudgetInMsg.isLinked() && this->linkBudgetInMsg.isWritten();
    if (this->linkBudgetValid) {
        this->linkBudgetBuffer = this->linkBudgetInMsg();
    } else {
        this->linkBudgetBuffer = {};
    }

    return true;
}

/*! Write custom output messages */
void DownlinkHandling::customWriteMessages(uint64_t CurrentClock)
{
    this->downlinkOutMsg.write(&this->downlinkOutBuffer, this->moduleID, CurrentClock);
}

/*! Core data-model evaluation */
void DownlinkHandling::evaluateDataModel(DataNodeUsageMsgPayload *dataUsageMsg, double currentTime)
{
    *dataUsageMsg = this->nodeDataOutMsg.zeroMsgPayload;

    this->currentTimeStep = currentTime - this->previousTime;
    if (this->currentTimeStep < 0.0) {
        this->currentTimeStep = 0.0;
    }

    this->downlinkOutBuffer = this->downlinkOutMsg.zeroMsgPayload;
    this->downlinkOutBuffer.timeStep = this->currentTimeStep;
    this->downlinkOutBuffer.maxRetransmissions = std::max<uint64_t>(1, this->maxRetransmissions);
    this->downlinkOutBuffer.bitRateRequest = this->bitRateRequest;
    this->downlinkOutBuffer.packetSizeBits = this->packetSizeBits;
    this->downlinkOutBuffer.bandwidth = std::max(0.0, this->linkBudgetBuffer.bandwidth);

    int64_t storageIndex = this->selectStorageIndex();
    this->availableDataBits = this->getStorageBitsAtIndex(storageIndex);
    this->setDataNameFromStorageIndex(storageIndex, this->nodeDataName, sizeof(this->nodeDataName));
    std::strncpy(dataUsageMsg->dataName, this->nodeDataName, sizeof(dataUsageMsg->dataName) - 1);
    dataUsageMsg->dataName[sizeof(dataUsageMsg->dataName) - 1] = '\0';

    std::strncpy(this->downlinkOutBuffer.dataName, this->nodeDataName, sizeof(this->downlinkOutBuffer.dataName) - 1);
    this->downlinkOutBuffer.dataName[sizeof(this->downlinkOutBuffer.dataName) - 1] = '\0';
    this->downlinkOutBuffer.availableDataBits = this->availableDataBits;

    double selectedCnr = 0.0;
    uint32_t selectedReceiver = 0;
    this->selectReceiver(&selectedCnr, &selectedReceiver);
    this->downlinkOutBuffer.receiverIndex = selectedReceiver;
    this->downlinkOutBuffer.cnr = std::max(0.0, selectedCnr);

    if (selectedReceiver == 1) {
        std::strncpy(this->downlinkOutBuffer.receiverAntennaName, this->linkBudgetBuffer.antennaName1, sizeof(this->downlinkOutBuffer.receiverAntennaName) - 1);
        std::strncpy(this->downlinkOutBuffer.transmitterAntennaName, this->linkBudgetBuffer.antennaName2, sizeof(this->downlinkOutBuffer.transmitterAntennaName) - 1);
    } else if (selectedReceiver == 2) {
        std::strncpy(this->downlinkOutBuffer.receiverAntennaName, this->linkBudgetBuffer.antennaName2, sizeof(this->downlinkOutBuffer.receiverAntennaName) - 1);
        std::strncpy(this->downlinkOutBuffer.transmitterAntennaName, this->linkBudgetBuffer.antennaName1, sizeof(this->downlinkOutBuffer.transmitterAntennaName) - 1);
    }
    this->downlinkOutBuffer.receiverAntennaName[sizeof(this->downlinkOutBuffer.receiverAntennaName) - 1] = '\0';
    this->downlinkOutBuffer.transmitterAntennaName[sizeof(this->downlinkOutBuffer.transmitterAntennaName) - 1] = '\0';

    bool validParams = this->linkBudgetValid
                       && selectedCnr > 0.0
                       && this->linkBudgetBuffer.bandwidth > 0.0
                       && this->bitRateRequest > 0.0
                       && this->packetSizeBits > 0.0;

    this->downlinkOutBuffer.linkActive = validParams ? 1 : 0;

    double ber = 0.0;
    double per = 0.0;
    double packetSuccessProb = 0.0;
    double packetDropProb = 0.0;
    double expectedAttemptsPerPacket = static_cast<double>(std::max<uint64_t>(1, this->maxRetransmissions));

    double attemptedRatePotential = 0.0;
    double storageRemovalRatePotential = 0.0;
    double deliveredRatePotential = 0.0;
    double droppedRatePotential = 0.0;

    if (validParams) {
        // Convert link quality into per-bit energy so BER can reflect user-selected bit rate.
        this->downlinkOutBuffer.cnr_dB = 10.0 * std::log10(selectedCnr);
        this->downlinkOutBuffer.cNo_dBHz = this->downlinkOutBuffer.cnr_dB + 10.0 * std::log10(this->linkBudgetBuffer.bandwidth);
        this->downlinkOutBuffer.ebN0_dB = this->downlinkOutBuffer.cNo_dBHz - 10.0 * std::log10(this->bitRateRequest);

        ber = this->computeBerFromEbN0dB(this->downlinkOutBuffer.ebN0_dB);
        ber = this->clampProbability(ber);

        // Any corrupted bit causes packet failure (checksum detect model).
        if (ber >= 1.0) {
            per = 1.0;
        } else if (ber <= 0.0) {
            per = 0.0;
        } else {
            per = 1.0 - std::exp(this->packetSizeBits * std::log1p(-ber));
            per = this->clampProbability(per);
        }

        const uint64_t maxRetx = std::max<uint64_t>(1, this->maxRetransmissions);
        const double successOneAttempt = this->clampProbability(1.0 - per);
        const double perPowMax = std::pow(per, static_cast<double>(maxRetx));
        packetDropProb = this->clampProbability(perPowMax);
        packetSuccessProb = this->clampProbability(1.0 - packetDropProb);

        if (successOneAttempt <= 0.0) {
            expectedAttemptsPerPacket = static_cast<double>(maxRetx);
        } else {
            // Truncated geometric expectation under retry cap.
            expectedAttemptsPerPacket = packetSuccessProb / successOneAttempt;
            expectedAttemptsPerPacket = std::max(1.0, expectedAttemptsPerPacket);
        }

        // attemptedRatePotential is channel usage; storageRemoval is source-packet outflow.
        attemptedRatePotential = this->bitRateRequest;
        storageRemovalRatePotential = attemptedRatePotential / expectedAttemptsPerPacket;
        deliveredRatePotential = storageRemovalRatePotential * packetSuccessProb;
        droppedRatePotential = storageRemovalRatePotential - deliveredRatePotential;
    }

    this->downlinkOutBuffer.ber = ber;
    this->downlinkOutBuffer.per = per;
    this->downlinkOutBuffer.packetSuccessProb = packetSuccessProb;
    this->downlinkOutBuffer.packetDropProb = packetDropProb;
    this->downlinkOutBuffer.expectedAttemptsPerPacket = expectedAttemptsPerPacket;

    double scale = 0.0;
    // Optional packet gating plus storage saturation: never remove more than available this step.
    bool enoughForPacket = (!this->requireFullPacket) || (this->availableDataBits >= this->packetSizeBits);
    if (storageRemovalRatePotential > 0.0 && this->currentTimeStep > 0.0 && enoughForPacket) {
        double availableRemovalRate = this->availableDataBits / this->currentTimeStep;
        scale = std::clamp(availableRemovalRate / storageRemovalRatePotential, 0.0, 1.0);
    }

    double attemptedDataRate = attemptedRatePotential * scale;
    double storageRemovalRate = storageRemovalRatePotential * scale;
    double deliveredDataRate = deliveredRatePotential * scale;
    double droppedDataRate = droppedRatePotential * scale;

    dataUsageMsg->baudRate = -storageRemovalRate;

    this->downlinkOutBuffer.attemptedDataRate = attemptedDataRate;
    this->downlinkOutBuffer.storageRemovalRate = storageRemovalRate;
    this->downlinkOutBuffer.deliveredDataRate = deliveredDataRate;
    this->downlinkOutBuffer.droppedDataRate = droppedDataRate;
    this->downlinkOutBuffer.estimatedRemainingDataBits = std::max(0.0, this->availableDataBits - storageRemovalRate * this->currentTimeStep);

    this->cumulativeAttemptedBits += attemptedDataRate * this->currentTimeStep;
    this->cumulativeRemovedBits += storageRemovalRate * this->currentTimeStep;
    this->cumulativeDeliveredBits += deliveredDataRate * this->currentTimeStep;
    this->cumulativeDroppedBits += droppedDataRate * this->currentTimeStep;

    this->downlinkOutBuffer.cumulativeAttemptedBits = this->cumulativeAttemptedBits;
    this->downlinkOutBuffer.cumulativeRemovedBits = this->cumulativeRemovedBits;
    this->downlinkOutBuffer.cumulativeDeliveredBits = this->cumulativeDeliveredBits;
    this->downlinkOutBuffer.cumulativeDroppedBits = this->cumulativeDroppedBits;

    this->previousTime = currentTime;
}

/*! Select the storage partition index with largest available data */
int64_t DownlinkHandling::selectStorageIndex() const
{
    if (this->storageUnitMsgsBuffer.empty()) {
        return -1;
    }

    const auto &storage = this->storageUnitMsgsBuffer.back();
    if (storage.storedData.empty()) {
        return -1;
    }

    int64_t maxIndex = -1;
    double maxBits = 0.0;
    for (std::size_t i = 0; i < storage.storedData.size(); i++) {
        if (storage.storedData[i] > maxBits) {
            maxBits = storage.storedData[i];
            maxIndex = static_cast<int64_t>(i);
        }
    }
    return maxIndex;
}

/*! Return available data bits at selected storage index */
double DownlinkHandling::getStorageBitsAtIndex(int64_t index) const
{
    if (this->storageUnitMsgsBuffer.empty()) {
        return 0.0;
    }

    const auto &storage = this->storageUnitMsgsBuffer.back();
    if (index >= 0 && static_cast<std::size_t>(index) < storage.storedData.size()) {
        return std::max(0.0, storage.storedData[static_cast<std::size_t>(index)]);
    }

    return std::max(0.0, storage.storageLevel);
}

/*! Set data name based on storage partition index */
void DownlinkHandling::setDataNameFromStorageIndex(int64_t index, char *buffer, std::size_t bufferSize) const
{
    if (bufferSize == 0) {
        return;
    }

    std::strncpy(buffer, "STORED DATA", bufferSize - 1);
    buffer[bufferSize - 1] = '\0';

    if (this->storageUnitMsgsBuffer.empty()) {
        return;
    }

    const auto &storage = this->storageUnitMsgsBuffer.back();
    if (index >= 0 && static_cast<std::size_t>(index) < storage.storedDataName.size()) {
        std::strncpy(buffer, storage.storedDataName[static_cast<std::size_t>(index)].c_str(), bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
    }
}

/*! Select receiver CNR from link budget payload */
void DownlinkHandling::selectReceiver(double *cnrLinear, uint32_t *receiverIndex) const
{
    *cnrLinear = 0.0;
    *receiverIndex = 0;

    if (!this->linkBudgetValid) {
        return;
    }

    const bool rx1 = this->isReceiverState(this->linkBudgetBuffer.antennaState1);
    const bool rx2 = this->isReceiverState(this->linkBudgetBuffer.antennaState2);
    const double cnr1 = std::max(0.0, this->linkBudgetBuffer.CNR1);
    const double cnr2 = std::max(0.0, this->linkBudgetBuffer.CNR2);

    if (this->receiverAntenna == 1) {
        *receiverIndex = 1;
        *cnrLinear = rx1 ? cnr1 : 0.0;
        return;
    }
    if (this->receiverAntenna == 2) {
        *receiverIndex = 2;
        *cnrLinear = rx2 ? cnr2 : 0.0;
        return;
    }

    if (rx1 && cnr1 > 0.0 && (!rx2 || cnr1 >= cnr2)) {
        *receiverIndex = 1;
        *cnrLinear = cnr1;
        return;
    }
    if (rx2 && cnr2 > 0.0) {
        *receiverIndex = 2;
        *cnrLinear = cnr2;
    }
}

/*! Check if antenna state allows receiving */
bool DownlinkHandling::isReceiverState(uint32_t state)
{
    const auto antennaState = static_cast<AntennaTypes::AntennaStateEnum>(state);
    return antennaState == AntennaTypes::AntennaStateEnum::ANTENNA_RX
           || antennaState == AntennaTypes::AntennaStateEnum::ANTENNA_RXTX;
}

/*! Compute BER for BPSK over AWGN from Eb/N0 [dB] */
double DownlinkHandling::computeBerFromEbN0dB(double ebN0_dB)
{
    const double ebN0Linear = std::pow(10.0, ebN0_dB / 10.0);
    return 0.5 * std::erfc(std::sqrt(ebN0Linear));
}

/*! Clamp probabilities to [0, 1] */
double DownlinkHandling::clampProbability(double value)
{
    return std::clamp(value, 0.0, 1.0);
}
