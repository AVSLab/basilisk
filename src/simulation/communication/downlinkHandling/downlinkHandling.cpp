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
    this->bitRateRequest = 0.0;                            // [bit/s]
    this->packetSizeBits = 256.0;                          // [bit]
    this->maxRetransmissions = 10;                         // [-]
    this->receiverAntenna = 0;                             // [-]
    this->removalPolicy = RemovalPolicy::REMOVE_ATTEMPTED; // [-]
    this->requireFullPacket = true;                        // [-]

    this->previousTime = 0.0;      // [s]
    this->currentTimeStep = 0.0;   // [s]
    this->availableDataBits = 0.0; // [bit]

    this->cumulativeAttemptedBits = 0.0; // [bit]
    this->cumulativeRemovedBits = 0.0;   // [bit]
    this->cumulativeDeliveredBits = 0.0; // [bit]
    this->cumulativeDroppedBits = 0.0;   // [bit]

    this->linkBudgetValid = false;
    this->linkBudgetBuffer = {};
    this->downlinkOutBuffer = this->downlinkOutMsg.zeroMsgPayload;

    std::strncpy(this->nodeDataName, "STORED DATA", sizeof(this->nodeDataName) - 1);
    this->nodeDataName[sizeof(this->nodeDataName) - 1] = '\0';
}

/*! Add a storage-status message to the module input list */
bool
DownlinkHandling::addStorageUnitToDownlink(Message<DataStorageStatusMsgPayload>* tmpStorageUnitMsg)
{
    if (tmpStorageUnitMsg == nullptr) {
        bskLogger.bskLog(BSK_ERROR, "DownlinkHandling.addStorageUnitToDownlink: null message pointer.");
        return false;
    }

    for (auto* msgPtr : this->storageUnitMsgPtrs) {
        if (msgPtr == tmpStorageUnitMsg) {
            bskLogger.bskLog(BSK_WARNING,
                             "DownlinkHandling.addStorageUnitToDownlink: duplicate storage message ignored.");
            return false;
        }
    }

    this->storageUnitMsgPtrs.push_back(tmpStorageUnitMsg);
    this->storageUnitInMsgs.push_back(tmpStorageUnitMsg->addSubscriber());
    return true;
}

/*! Set requested raw bit rate [bit/s] */
bool
DownlinkHandling::setBitRateRequest(double bitRateRequest)
{
    if (!this->isFiniteNonNegative(bitRateRequest)) {
        bskLogger.bskLog(BSK_ERROR, "DownlinkHandling.setBitRateRequest: value must be finite and >= 0 [bit/s].");
        return false;
    }

    this->bitRateRequest = bitRateRequest;
    return true;
}

/*! Set packet size used for BER-to-PER conversion [bit] */
bool
DownlinkHandling::setPacketSizeBits(double packetSizeBits)
{
    if (!this->isFinitePositive(packetSizeBits)) {
        bskLogger.bskLog(BSK_ERROR, "DownlinkHandling.setPacketSizeBits: value must be finite and > 0 [bit].");
        return false;
    }

    this->packetSizeBits = packetSizeBits;
    return true;
}

/*! Set retry-cap used by the ARQ expectation model [-] */
bool
DownlinkHandling::setMaxRetransmissions(int64_t maxRetransmissions)
{
    if (maxRetransmissions < 1) {
        bskLogger.bskLog(BSK_ERROR, "DownlinkHandling.setMaxRetransmissions: value must be >= 1.");
        return false;
    }

    this->maxRetransmissions = static_cast<uint64_t>(maxRetransmissions);
    return true;
}

/*! Set receiver-selection mode: 0=auto, 1=path1, 2=path2 [-] */
bool
DownlinkHandling::setReceiverAntenna(int64_t receiverAntenna)
{
    if (receiverAntenna != 0 && receiverAntenna != 1 && receiverAntenna != 2) {
        bskLogger.bskLog(BSK_ERROR, "DownlinkHandling.setReceiverAntenna: value must be 0 (auto), 1, or 2.");
        return false;
    }

    this->receiverAntenna = receiverAntenna;
    return true;
}

/*! Set storage-removal policy: 0=remove attempted, 1=remove delivered only */
bool
DownlinkHandling::setRemovalPolicy(int64_t removalPolicy)
{
    if (removalPolicy == static_cast<int64_t>(RemovalPolicy::REMOVE_ATTEMPTED)) {
        this->removalPolicy = RemovalPolicy::REMOVE_ATTEMPTED;
        return true;
    }
    if (removalPolicy == static_cast<int64_t>(RemovalPolicy::REMOVE_DELIVERED_ONLY)) {
        this->removalPolicy = RemovalPolicy::REMOVE_DELIVERED_ONLY;
        return true;
    }

    bskLogger.bskLog(
      BSK_ERROR, "DownlinkHandling.setRemovalPolicy: value must be 0 (REMOVE_ATTEMPTED) or 1 (REMOVE_DELIVERED_ONLY).");
    return false;
}

/*! Set packet-gating behavior flag */
void
DownlinkHandling::setRequireFullPacket(bool requireFullPacket)
{
    this->requireFullPacket = requireFullPacket;
}

/*! Module reset hook */
void
DownlinkHandling::customReset(uint64_t CurrentClock)
{
    this->previousTime = static_cast<double>(CurrentClock) * NANO2SEC;
    this->currentTimeStep = 0.0; // [s]

    this->cumulativeAttemptedBits = 0.0; // [bit]
    this->cumulativeRemovedBits = 0.0;   // [bit]
    this->cumulativeDeliveredBits = 0.0; // [bit]
    this->cumulativeDroppedBits = 0.0;   // [bit]
    this->lastAmbiguousRouteName.clear();

    this->downlinkOutBuffer = this->downlinkOutMsg.zeroMsgPayload;

    if (!this->validateConfiguration()) {
        bskLogger.bskLog(BSK_WARNING, "DownlinkHandling: invalid configuration detected at reset; restoring defaults.");
        this->bitRateRequest = 0.0;                            // [bit/s]
        this->packetSizeBits = 256.0;                          // [bit]
        this->maxRetransmissions = 10;                         // [-]
        this->receiverAntenna = 0;                             // [-]
        this->removalPolicy = RemovalPolicy::REMOVE_ATTEMPTED; // [-]
        this->requireFullPacket = true;                        // [-]
    }

    if (!this->linkBudgetInMsg.isLinked()) {
        bskLogger.bskLog(BSK_WARNING, "DownlinkHandling.linkBudgetInMsg is not linked.");
    }
    if (this->storageUnitInMsgs.empty()) {
        bskLogger.bskLog(BSK_WARNING, "DownlinkHandling has no storageUnitInMsgs linked.");
    }
}

/*! Read custom input messages */
bool
DownlinkHandling::customReadMessages()
{
    this->storageUnitMsgsBuffer.clear();
    for (auto& msg : this->storageUnitInMsgs) {
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
void
DownlinkHandling::customWriteMessages(uint64_t CurrentClock)
{
    if (this->dataStatus <= 0) {
        // When DataNodeBase skips evaluateDataModel() (node disabled), avoid publishing stale diagnostics.
        // Also advance timing state so a later re-enable does not integrate disabled downtime in one step.
        this->previousTime = static_cast<double>(CurrentClock) * NANO2SEC; // [s]
        this->currentTimeStep = 0.0;                                       // [s]
        this->downlinkOutBuffer = this->downlinkOutMsg.zeroMsgPayload;
        this->downlinkOutBuffer.maxRetransmissions = this->maxRetransmissions;
        this->downlinkOutBuffer.removalPolicy = this->getRemovalPolicy();
        this->downlinkOutBuffer.bitRateRequest = this->bitRateRequest;
        this->downlinkOutBuffer.packetSizeBits = this->packetSizeBits;
        this->downlinkOutBuffer.cumulativeAttemptedBits = this->sanitizeNonNegative(this->cumulativeAttemptedBits);
        this->downlinkOutBuffer.cumulativeRemovedBits = this->sanitizeNonNegative(this->cumulativeRemovedBits);
        this->downlinkOutBuffer.cumulativeDeliveredBits = this->sanitizeNonNegative(this->cumulativeDeliveredBits);
        this->downlinkOutBuffer.cumulativeDroppedBits = this->sanitizeNonNegative(this->cumulativeDroppedBits);
    }
    this->downlinkOutMsg.write(&this->downlinkOutBuffer, this->moduleID, CurrentClock);
}

/*! Core data-model evaluation */
void
DownlinkHandling::evaluateDataModel(DataNodeUsageMsgPayload* dataUsageMsg, double currentTime)
{
    *dataUsageMsg = this->nodeDataOutMsg.zeroMsgPayload;

    double computedTimeStep = currentTime - this->previousTime; // [s]
    if (!this->isFiniteNonNegative(computedTimeStep)) {
        computedTimeStep = 0.0; // [s]
    }
    this->currentTimeStep = computedTimeStep;

    this->downlinkOutBuffer = this->downlinkOutMsg.zeroMsgPayload;
    this->downlinkOutBuffer.timeStep = this->currentTimeStep;
    this->downlinkOutBuffer.maxRetransmissions = this->maxRetransmissions;
    this->downlinkOutBuffer.bitRateRequest = this->bitRateRequest;
    this->downlinkOutBuffer.packetSizeBits = this->packetSizeBits;
    this->downlinkOutBuffer.removalPolicy = this->getRemovalPolicy();
    this->downlinkOutBuffer.bandwidth = this->sanitizeNonNegative(this->linkBudgetBuffer.bandwidth);

    const StorageSelection selection = this->selectStorageSelection();
    this->availableDataBits = selection.availableBits;
    this->setDataNameFromStorageSelection(selection, this->nodeDataName, sizeof(this->nodeDataName));
    std::strncpy(dataUsageMsg->dataName, this->nodeDataName, sizeof(dataUsageMsg->dataName) - 1);
    dataUsageMsg->dataName[sizeof(dataUsageMsg->dataName) - 1] = '\0';

    std::strncpy(this->downlinkOutBuffer.dataName, this->nodeDataName, sizeof(this->downlinkOutBuffer.dataName) - 1);
    this->downlinkOutBuffer.dataName[sizeof(this->downlinkOutBuffer.dataName) - 1] = '\0';
    this->downlinkOutBuffer.availableDataBits = this->availableDataBits;
    const bool uniqueStorageRoute = this->isStorageRouteUnique(selection, this->nodeDataName);
    if (!uniqueStorageRoute) {
        if (this->lastAmbiguousRouteName != this->nodeDataName) {
            bskLogger.bskLog(BSK_WARNING,
                             "DownlinkHandling: selected dataName '%s' is present in multiple linked storage units; "
                             "storage removal is forced to zero to avoid ambiguous routing.",
                             this->nodeDataName);
            this->lastAmbiguousRouteName = this->nodeDataName;
        }
    } else {
        this->lastAmbiguousRouteName.clear();
    }

    double selectedCnr = 0.0;      // [-]
    uint32_t selectedReceiver = 0; // [-]
    this->selectReceiver(&selectedCnr, &selectedReceiver);
    this->downlinkOutBuffer.receiverIndex = selectedReceiver;
    this->downlinkOutBuffer.cnr = this->sanitizeNonNegative(selectedCnr);

    if (selectedReceiver == 1) {
        std::strncpy(this->downlinkOutBuffer.receiverAntennaName,
                     this->linkBudgetBuffer.antennaName1,
                     sizeof(this->downlinkOutBuffer.receiverAntennaName) - 1);
        std::strncpy(this->downlinkOutBuffer.transmitterAntennaName,
                     this->linkBudgetBuffer.antennaName2,
                     sizeof(this->downlinkOutBuffer.transmitterAntennaName) - 1);
    } else if (selectedReceiver == 2) {
        std::strncpy(this->downlinkOutBuffer.receiverAntennaName,
                     this->linkBudgetBuffer.antennaName2,
                     sizeof(this->downlinkOutBuffer.receiverAntennaName) - 1);
        std::strncpy(this->downlinkOutBuffer.transmitterAntennaName,
                     this->linkBudgetBuffer.antennaName1,
                     sizeof(this->downlinkOutBuffer.transmitterAntennaName) - 1);
    }
    this->downlinkOutBuffer.receiverAntennaName[sizeof(this->downlinkOutBuffer.receiverAntennaName) - 1] = '\0';
    this->downlinkOutBuffer.transmitterAntennaName[sizeof(this->downlinkOutBuffer.transmitterAntennaName) - 1] = '\0';

    const bool validParams = this->linkBudgetValid && this->isFinitePositive(selectedCnr) &&
                             this->isFinitePositive(this->downlinkOutBuffer.bandwidth) &&
                             this->isFinitePositive(this->bitRateRequest) &&
                             this->isFinitePositive(this->packetSizeBits);
    this->downlinkOutBuffer.linkActive = validParams ? 1 : 0;

    double ber = 0.0;                                                                 // [-]
    double per = 0.0;                                                                 // [-]
    double packetSuccessProb = 0.0;                                                   // [-]
    double packetDropProb = 0.0;                                                      // [-]
    double expectedAttemptsPerPacket = static_cast<double>(this->maxRetransmissions); // [-]

    double attemptedRatePotential = 0.0;      // [bit/s]
    double storageRemovalRatePotential = 0.0; // [bit/s]
    double deliveredRatePotential = 0.0;      // [bit/s]
    double droppedRatePotential = 0.0;        // [bit/s]

    if (validParams) {
        // Convert RF quality to per-bit energy so BER reflects user-selected bit rate.
        this->downlinkOutBuffer.cnr_dB = 10.0 * std::log10(selectedCnr);
        this->downlinkOutBuffer.cNo_dBHz =
          this->downlinkOutBuffer.cnr_dB + 10.0 * std::log10(this->downlinkOutBuffer.bandwidth);
        this->downlinkOutBuffer.ebN0_dB = this->downlinkOutBuffer.cNo_dBHz - 10.0 * std::log10(this->bitRateRequest);

        ber = this->clampProbability(this->computeBerFromEbN0dB(this->downlinkOutBuffer.ebN0_dB));

        // Packet fails if any bit is wrong (checksum detect model).
        if (ber >= 1.0) {
            per = 1.0;
        } else if (ber <= 0.0) {
            per = 0.0;
        } else {
            per = 1.0 - std::exp(this->packetSizeBits * std::log1p(-ber));
            per = this->clampProbability(per);
        }

        const double maxRetxAsDouble = static_cast<double>(this->maxRetransmissions);
        const double successOneAttempt = this->clampProbability(1.0 - per);
        packetDropProb = this->clampProbability(std::pow(per, maxRetxAsDouble));
        packetSuccessProb = this->clampProbability(1.0 - packetDropProb);

        if (successOneAttempt <= 0.0) {
            expectedAttemptsPerPacket = maxRetxAsDouble;
        } else {
            expectedAttemptsPerPacket = packetSuccessProb / successOneAttempt;
            expectedAttemptsPerPacket = std::clamp(expectedAttemptsPerPacket, 1.0, maxRetxAsDouble);
        }

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

    double scale = 0.0; // [-]
    // Optional packet gating plus storage saturation: never remove more than available this step.
    const bool enoughForPacket = (!this->requireFullPacket) || (this->availableDataBits >= this->packetSizeBits);
    if (storageRemovalRatePotential > 0.0 && this->currentTimeStep > 0.0 && enoughForPacket && uniqueStorageRoute) {
        const double availableRemovalRate = this->availableDataBits / this->currentTimeStep; // [bit/s]
        if (this->isFiniteNonNegative(availableRemovalRate)) {
            scale = std::clamp(availableRemovalRate / storageRemovalRatePotential, 0.0, 1.0);
        }
    }

    const double attemptedDataRate = this->sanitizeNonNegative(attemptedRatePotential * scale);              // [bit/s]
    const double modeledStorageRemovalRate = this->sanitizeNonNegative(storageRemovalRatePotential * scale); // [bit/s]
    const double deliveredDataRate = this->sanitizeNonNegative(deliveredRatePotential * scale);              // [bit/s]
    const double droppedDataRate = this->sanitizeNonNegative(droppedRatePotential * scale);                  // [bit/s]

    // Removal policy controls what is actually deleted onboard:
    // attempted mode removes delivered+drop-limited bits, delivered-only keeps dropped bits for retry/future attempts.
    const bool removeDeliveredOnly = (this->removalPolicy == RemovalPolicy::REMOVE_DELIVERED_ONLY);
    const double storageRemovalRate = removeDeliveredOnly ? deliveredDataRate : modeledStorageRemovalRate; // [bit/s]
    dataUsageMsg->baudRate = -storageRemovalRate;

    this->downlinkOutBuffer.attemptedDataRate = attemptedDataRate;
    this->downlinkOutBuffer.storageRemovalRate = storageRemovalRate;
    this->downlinkOutBuffer.deliveredDataRate = deliveredDataRate;
    this->downlinkOutBuffer.droppedDataRate = droppedDataRate;

    const double removedBitsThisStep = storageRemovalRate * this->currentTimeStep; // [bit]
    this->downlinkOutBuffer.estimatedRemainingDataBits =
      this->sanitizeNonNegative(this->availableDataBits - removedBitsThisStep);

    if (this->currentTimeStep > 0.0) {
        this->cumulativeAttemptedBits += attemptedDataRate * this->currentTimeStep;
        this->cumulativeRemovedBits += storageRemovalRate * this->currentTimeStep;
        this->cumulativeDeliveredBits += deliveredDataRate * this->currentTimeStep;
        this->cumulativeDroppedBits += droppedDataRate * this->currentTimeStep;
    }

    this->downlinkOutBuffer.cumulativeAttemptedBits = this->sanitizeNonNegative(this->cumulativeAttemptedBits);
    this->downlinkOutBuffer.cumulativeRemovedBits = this->sanitizeNonNegative(this->cumulativeRemovedBits);
    this->downlinkOutBuffer.cumulativeDeliveredBits = this->sanitizeNonNegative(this->cumulativeDeliveredBits);
    this->downlinkOutBuffer.cumulativeDroppedBits = this->sanitizeNonNegative(this->cumulativeDroppedBits);

    if (this->isFinite(currentTime)) {
        this->previousTime = currentTime;
    } else {
        this->previousTime += this->currentTimeStep;
    }
}

/*! Select storage target with the largest finite data across linked units.
 *
 *  The function prioritizes per-partition values when available and only falls back to
 *  aggregate ``storageLevel`` for messages that provide no partition vector.
 */
DownlinkHandling::StorageSelection
DownlinkHandling::selectStorageSelection() const
{
    StorageSelection selection;
    selection.storageUnitIndex = -1;
    selection.partitionIndex = -1;
    selection.availableBits = 0.0; // [bit]

    for (std::size_t unitIdx = 0; unitIdx < this->storageUnitMsgsBuffer.size(); unitIdx++) {
        const auto& storage = this->storageUnitMsgsBuffer[unitIdx];

        bool foundFinitePartition = false; // [-]
        for (std::size_t partitionIdx = 0; partitionIdx < storage.storedData.size(); partitionIdx++) {
            const double partitionBits = this->sanitizeNonNegative(storage.storedData[partitionIdx]); // [bit]
            foundFinitePartition = true;
            if (partitionBits > selection.availableBits) {
                selection.storageUnitIndex = static_cast<int64_t>(unitIdx);    // [-]
                selection.partitionIndex = static_cast<int64_t>(partitionIdx); // [-]
                selection.availableBits = partitionBits;                       // [bit]
            }
        }

        // Fallback to aggregate storage-level only when no partition vector is provided.
        if (!foundFinitePartition) {
            const double storageLevelBits = this->sanitizeNonNegative(storage.storageLevel); // [bit]
            if (storageLevelBits > selection.availableBits) {
                selection.storageUnitIndex = static_cast<int64_t>(unitIdx); // [-]
                selection.partitionIndex = -1;                              // [-]
                selection.availableBits = storageLevelBits;                 // [bit]
            }
        }
    }

    return selection;
}

/*! Return true only when selected dataName identifies exactly one linked storage unit */
bool
DownlinkHandling::isStorageRouteUnique(const StorageSelection& selection, const char* dataName) const
{
    if (selection.storageUnitIndex < 0) {
        return true;
    }
    if (dataName == nullptr || dataName[0] == '\0') {
        return false;
    }

    // Aggregate-only messages (no partition vector) cannot be uniquely routed across multiple units.
    if (selection.partitionIndex < 0) {
        return this->storageUnitMsgsBuffer.size() <= 1;
    }

    std::size_t unitsWithMatch = 0; // [-]
    for (const auto& storage : this->storageUnitMsgsBuffer) {
        bool hasMatch = false; // [-]
        for (const auto& name : storage.storedDataName) {
            if (name == dataName) {
                hasMatch = true;
                break;
            }
        }
        if (hasMatch) {
            unitsWithMatch++;
            if (unitsWithMatch > 1) {
                return false;
            }
        }
    }

    return unitsWithMatch == 1;
}

/*! Set data name based on selected storage unit and partition */
void
DownlinkHandling::setDataNameFromStorageSelection(const StorageSelection& selection,
                                                  char* buffer,
                                                  std::size_t bufferSize) const
{
    if (bufferSize == 0) {
        return;
    }

    std::strncpy(buffer, "STORED DATA", bufferSize - 1);
    buffer[bufferSize - 1] = '\0';

    if (selection.storageUnitIndex < 0) {
        return;
    }
    const auto selectedUnit = static_cast<std::size_t>(selection.storageUnitIndex);
    if (selectedUnit >= this->storageUnitMsgsBuffer.size()) {
        return;
    }

    const auto& storage = this->storageUnitMsgsBuffer[selectedUnit];
    if (selection.partitionIndex < 0) {
        return;
    }

    const auto selectedPartition = static_cast<std::size_t>(selection.partitionIndex);
    if (selectedPartition < storage.storedDataName.size()) {
        const auto& name = storage.storedDataName[selectedPartition];
        if (!name.empty()) {
            std::strncpy(buffer, name.c_str(), bufferSize - 1);
            buffer[bufferSize - 1] = '\0';
        }
    }
}

/*! Select receiver CNR from link budget payload */
void
DownlinkHandling::selectReceiver(double* cnrLinear, uint32_t* receiverIndex) const
{
    *cnrLinear = 0.0;
    *receiverIndex = 0;

    if (!this->linkBudgetValid) {
        return;
    }

    const bool rx1 = this->isReceiverState(this->linkBudgetBuffer.antennaState1);
    const bool rx2 = this->isReceiverState(this->linkBudgetBuffer.antennaState2);
    const double cnr1 =
      (rx1 && this->isFinitePositive(this->linkBudgetBuffer.CNR1)) ? this->linkBudgetBuffer.CNR1 : 0.0;
    const double cnr2 =
      (rx2 && this->isFinitePositive(this->linkBudgetBuffer.CNR2)) ? this->linkBudgetBuffer.CNR2 : 0.0;

    if (this->receiverAntenna == 1) {
        if (cnr1 > 0.0) {
            *receiverIndex = 1;
            *cnrLinear = cnr1;
        }
        return;
    }
    if (this->receiverAntenna == 2) {
        if (cnr2 > 0.0) {
            *receiverIndex = 2;
            *cnrLinear = cnr2;
        }
        return;
    }

    if (cnr1 >= cnr2 && cnr1 > 0.0) {
        *receiverIndex = 1;
        *cnrLinear = cnr1;
    } else if (cnr2 > 0.0) {
        *receiverIndex = 2;
        *cnrLinear = cnr2;
    }
}

/*! Validate module configuration values */
bool
DownlinkHandling::validateConfiguration() const
{
    return this->isFiniteNonNegative(this->bitRateRequest) && this->isFinitePositive(this->packetSizeBits) &&
           this->maxRetransmissions >= 1 &&
           (this->receiverAntenna == 0 || this->receiverAntenna == 1 || this->receiverAntenna == 2) &&
           (this->removalPolicy == RemovalPolicy::REMOVE_ATTEMPTED ||
            this->removalPolicy == RemovalPolicy::REMOVE_DELIVERED_ONLY);
}

/*! Return true if `value` is finite */
bool
DownlinkHandling::isFinite(double value)
{
    return std::isfinite(value);
}

/*! Return true if `value` is finite and >= 0 */
bool
DownlinkHandling::isFiniteNonNegative(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

/*! Return true if `value` is finite and > 0 */
bool
DownlinkHandling::isFinitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

/*! Check if antenna state allows receiving */
bool
DownlinkHandling::isReceiverState(uint32_t state)
{
    const auto antennaState = static_cast<AntennaTypes::AntennaStateEnum>(state);
    return antennaState == AntennaTypes::AntennaStateEnum::ANTENNA_RX ||
           antennaState == AntennaTypes::AntennaStateEnum::ANTENNA_RXTX;
}

/*! Compute BER for BPSK over AWGN from Eb/N0 [dB] */
double
DownlinkHandling::computeBerFromEbN0dB(double ebN0_dB)
{
    if (!std::isfinite(ebN0_dB)) {
        return 0.5;
    }

    const double ebN0Linear = std::pow(10.0, ebN0_dB / 10.0);
    if (!std::isfinite(ebN0Linear)) {
        return 0.0;
    }
    if (ebN0Linear <= 0.0) {
        return 0.5;
    }
    return 0.5 * std::erfc(std::sqrt(ebN0Linear));
}

/*! Clamp probabilities to [0, 1] */
double
DownlinkHandling::clampProbability(double value)
{
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, 0.0, 1.0);
}

/*! Clamp all invalid/non-physical nonnegative outputs to 0 */
double
DownlinkHandling::sanitizeNonNegative(double value)
{
    if (!std::isfinite(value) || value < 0.0) {
        return 0.0;
    }
    return value;
}
