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

#ifndef DOWNLINKHANDLING_H
#define DOWNLINKHANDLING_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "architecture/msgPayloadDefC/DownlinkHandlingMsgPayload.h"
#include "architecture/msgPayloadDefC/LinkBudgetMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "simulation/communication/_GeneralModuleFiles/AntennaDefinitions.h"
#include "simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h"

/*! @brief Downlink data-handling model that maps link quality to reliable throughput */
class DownlinkHandling : public DataNodeBase
{
  public:
    /*! @brief Storage removal policy selector.
     *
     *  ``REMOVE_ATTEMPTED`` removes both delivered and drop-limited data from storage.
     *  ``REMOVE_DELIVERED_ONLY`` removes only successfully delivered data.
     */
    enum class RemovalPolicy : uint32_t
    {
        REMOVE_ATTEMPTED = 0,     //!< [-] Remove delivered + drop-limited bits (current default behavior)
        REMOVE_DELIVERED_ONLY = 1 //!< [-] Remove only successfully delivered bits
    };

    /*! @brief Construct the downlink handling module with default parameters */
    DownlinkHandling();
    /*! @brief Default destructor */
    ~DownlinkHandling() = default;

    /*! @brief Register a storage-status input message.
     *
     *  Duplicate and null pointers are rejected.
     *
     *  @param tmpStorageUnitMsg Pointer to a storage status output message
     *  @return ``true`` if the message was added, ``false`` otherwise
     */
    bool addStorageUnitToDownlink(Message<DataStorageStatusMsgPayload>* tmpStorageUnitMsg);

    /*! @brief Set requested raw channel bit rate.
     *  @param bitRateRequest Requested bit rate [bit/s], must be finite and :math:`\ge 0`
     *  @return ``true`` if accepted, ``false`` otherwise
     */
    bool setBitRateRequest(double bitRateRequest);
    /*! @brief Set packet size used by BER-to-PER conversion.
     *  @param packetSizeBits Packet size [bit], must be finite and :math:`> 0`
     *  @return ``true`` if accepted, ``false`` otherwise
     */
    bool setPacketSizeBits(double packetSizeBits);
    /*! @brief Set retry cap used by the ARQ model.
     *  @param maxRetransmissions Maximum packet transmission attempts, must be :math:`\ge 1`
     *  @return ``true`` if accepted, ``false`` otherwise
     */
    bool setMaxRetransmissions(int64_t maxRetransmissions);
    /*! @brief Select receiver path mode.
     *  @param receiverAntenna ``0`` = auto, ``1`` = force path 1, ``2`` = force path 2
     *  @return ``true`` if accepted, ``false`` otherwise
     */
    bool setReceiverAntenna(int64_t receiverAntenna);
    /*! @brief Set storage-removal policy.
     *  @param removalPolicy ``0`` = remove attempted, ``1`` = remove delivered only
     *  @return ``true`` if accepted, ``false`` otherwise
     */
    bool setRemovalPolicy(int64_t removalPolicy);
    /*! @brief Set packet gating behavior.
     *  @param requireFullPacket If ``true``, require at least one full packet before downlink
     */
    void setRequireFullPacket(bool requireFullPacket);

    /*! @brief Get requested raw channel bit rate.
     *  @return Requested bit rate [bit/s]
     */
    double getBitRateRequest() const { return this->bitRateRequest; }
    /*! @brief Get configured packet size.
     *  @return Packet size [bit]
     */
    double getPacketSizeBits() const { return this->packetSizeBits; }
    /*! @brief Get retry cap used by ARQ model.
     *  @return Maximum transmission attempts per packet [-]
     */
    uint64_t getMaxRetransmissions() const { return this->maxRetransmissions; }
    /*! @brief Get receiver selection mode.
     *  @return ``0`` (auto), ``1`` (path 1), or ``2`` (path 2)
     */
    int64_t getReceiverAntenna() const { return this->receiverAntenna; }
    /*! @brief Get storage-removal policy.
     *  @return ``0`` (remove attempted) or ``1`` (remove delivered only)
     */
    uint32_t getRemovalPolicy() const { return static_cast<uint32_t>(this->removalPolicy); }
    /*! @brief Get packet gating mode.
     *  @return ``true`` if one full packet is required before downlink
     */
    bool getRequireFullPacket() const { return this->requireFullPacket; }

    ReadFunctor<LinkBudgetMsgPayload> linkBudgetInMsg;  //!< Link-budget input message
    Message<DownlinkHandlingMsgPayload> downlinkOutMsg; //!< Downlink performance output message

    BSKLogger bskLogger; //!< BSK logging interface

    /*! @brief Return 1 if link-quality inputs were valid on the last update, else 0 */
    uint32_t getCurrentLinkActive() const { return this->downlinkOutBuffer.linkActive; }
    /*! @brief Return selected receiver index on the last update (0=no receiver, 1/2=receiver path) */
    uint32_t getCurrentReceiverIndex() const { return this->downlinkOutBuffer.receiverIndex; }
    /*! @brief Return last module time step [s] */
    double getCurrentTimeStep() const { return this->downlinkOutBuffer.timeStep; }
    /*! @brief Return last computed bit-error-rate (BER) [-] */
    double getCurrentBer() const { return this->downlinkOutBuffer.ber; }
    /*! @brief Return last computed packet-error-rate (PER) [-] */
    double getCurrentPer() const { return this->downlinkOutBuffer.per; }
    /*! @brief Return last packet drop probability within retry cap [-] */
    double getCurrentPacketDropProb() const { return this->downlinkOutBuffer.packetDropProb; }
    /*! @brief Return last expected attempts per packet under retry cap [-] */
    double getCurrentExpectedAttemptsPerPacket() const { return this->downlinkOutBuffer.expectedAttemptsPerPacket; }
    /*! @brief Return last data-removal rate from storage [bit/s] */
    double getCurrentStorageRemovalRate() const { return this->downlinkOutBuffer.storageRemovalRate; }
    /*! @brief Return last successfully delivered data rate [bit/s] */
    double getCurrentDeliveredDataRate() const { return this->downlinkOutBuffer.deliveredDataRate; }
    /*! @brief Return last dropped data rate after retries [bit/s] */
    double getCurrentDroppedDataRate() const { return this->downlinkOutBuffer.droppedDataRate; }
    /*! @brief Return estimated remaining bits in selected data partition [bit] */
    double getCurrentEstimatedRemainingDataBits() const { return this->downlinkOutBuffer.estimatedRemainingDataBits; }

  private:
    struct StorageSelection
    {
        int64_t storageUnitIndex; //!< [-] storage-unit index in `storageUnitMsgsBuffer`
        int64_t partitionIndex;   //!< [-] partition index in the selected storage unit
        double availableBits;     //!< [bit] available bits in selected unit/partition
    };

    bool customReadMessages() override;
    void customWriteMessages(uint64_t CurrentClock) override;
    void customReset(uint64_t CurrentClock) override;
    void evaluateDataModel(DataNodeUsageMsgPayload* dataUsageMsg, double currentTime) override;

    StorageSelection selectStorageSelection() const;
    bool isStorageRouteUnique(const StorageSelection& selection, const char* dataName) const;
    void setDataNameFromStorageSelection(const StorageSelection& selection, char* buffer, std::size_t bufferSize) const;
    void selectReceiver(double* cnrLinear, uint32_t* receiverIndex) const;
    bool validateConfiguration() const;
    static bool isFinite(double value);
    static bool isFiniteNonNegative(double value);
    static bool isFinitePositive(double value);
    static bool isReceiverState(uint32_t state);
    static double computeBerFromEbN0dB(double ebN0_dB);
    static double clampProbability(double value);
    static double sanitizeNonNegative(double value);

  private:
    double bitRateRequest;       //!< [bit/s] Raw requested channel bit-rate
    double packetSizeBits;       //!< [bit]   Packet size for BER-to-PER conversion
    uint64_t maxRetransmissions; //!< [-]     Maximum ARQ transmission attempts per packet
    int64_t receiverAntenna;     //!< [-]     0=auto, 1=receiver antenna 1, 2=receiver antenna 2
    RemovalPolicy removalPolicy; //!< [-]     Storage-removal mode for downlinked data
    bool requireFullPacket;      //!< [-]     If true, wait for >=1 full packet before downlinking

    std::vector<Message<DataStorageStatusMsgPayload>*>
      storageUnitMsgPtrs; //!< Storage-msg pointer list for duplicate checks
    std::vector<ReadFunctor<DataStorageStatusMsgPayload>> storageUnitInMsgs; //!< Storage status subscribers
    std::vector<DataStorageStatusMsgPayload> storageUnitMsgsBuffer;          //!< Local storage status buffers

    LinkBudgetMsgPayload linkBudgetBuffer; //!< Local copy of link-budget message
    bool linkBudgetValid;                  //!< True when linkBudget message is linked and written

    DownlinkHandlingMsgPayload downlinkOutBuffer; //!< Local copy of downlink output message

    double previousTime;      //!< [s] previous simulation time
    double currentTimeStep;   //!< [s] current integration step
    double availableDataBits; //!< [bit] selected storage data available

    double cumulativeAttemptedBits; //!< [bit] attempted channel bits (includes retransmissions)
    double cumulativeRemovedBits;   //!< [bit] bits removed from storage
    double cumulativeDeliveredBits; //!< [bit] successfully delivered bits
    double cumulativeDroppedBits;   //!< [bit] dropped bits after retransmission limit

    std::string lastAmbiguousRouteName; //!< [-] last dataName warned for ambiguous multi-storage routing
};

#endif
