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
#include <vector>

#include "architecture/msgPayloadDefC/DownlinkHandlingMsgPayload.h"
#include "architecture/msgPayloadDefC/LinkBudgetMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "simulation/communication/_GeneralModuleFiles/AntennaDefinitions.h"
#include "simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h"

/*! @brief Downlink data-handling model that maps link quality to reliable throughput */
class DownlinkHandling : public DataNodeBase {
public:
    DownlinkHandling();
    ~DownlinkHandling() = default;

    void addStorageUnitToDownlink(Message<DataStorageStatusMsgPayload> *tmpStorageUnitMsg);

    ReadFunctor<LinkBudgetMsgPayload> linkBudgetInMsg;             //!< Link-budget input message
    Message<DownlinkHandlingMsgPayload> downlinkOutMsg;            //!< Downlink performance output message

    double bitRateRequest;                                         //!< [bit/s] Raw requested channel bit-rate
    double packetSizeBits;                                         //!< [bit]   Packet size for BER-to-PER conversion
    uint64_t maxRetransmissions;                                   //!< [-]     Maximum ARQ retransmissions per packet
    int64_t receiverAntenna;                                       //!< [-]     0=auto, 1=receiver antenna 1, 2=receiver antenna 2
    bool requireFullPacket;                                        //!< [-]     If true, wait for >=1 full packet before downlinking

    BSKLogger bskLogger;                                           //!< BSK logging interface

    /*! @brief Return 1 if link-quality inputs were valid on the last update, else 0 */
    uint32_t getCurrentLinkActive() const {return this->downlinkOutBuffer.linkActive;}
    /*! @brief Return selected receiver index on the last update (0=no receiver, 1/2=receiver path) */
    uint32_t getCurrentReceiverIndex() const {return this->downlinkOutBuffer.receiverIndex;}
    /*! @brief Return last module time step [s] */
    double getCurrentTimeStep() const {return this->downlinkOutBuffer.timeStep;}
    /*! @brief Return last computed bit-error-rate (BER) [-] */
    double getCurrentBer() const {return this->downlinkOutBuffer.ber;}
    /*! @brief Return last computed packet-error-rate (PER) [-] */
    double getCurrentPer() const {return this->downlinkOutBuffer.per;}
    /*! @brief Return last packet drop probability within retry cap [-] */
    double getCurrentPacketDropProb() const {return this->downlinkOutBuffer.packetDropProb;}
    /*! @brief Return last expected attempts per packet under retry cap [-] */
    double getCurrentExpectedAttemptsPerPacket() const {return this->downlinkOutBuffer.expectedAttemptsPerPacket;}
    /*! @brief Return last data-removal rate from storage [bit/s] */
    double getCurrentStorageRemovalRate() const {return this->downlinkOutBuffer.storageRemovalRate;}
    /*! @brief Return last successfully delivered data rate [bit/s] */
    double getCurrentDeliveredDataRate() const {return this->downlinkOutBuffer.deliveredDataRate;}
    /*! @brief Return last dropped data rate after retries [bit/s] */
    double getCurrentDroppedDataRate() const {return this->downlinkOutBuffer.droppedDataRate;}
    /*! @brief Return estimated remaining bits in selected data partition [bit] */
    double getCurrentEstimatedRemainingDataBits() const {return this->downlinkOutBuffer.estimatedRemainingDataBits;}

private:
    bool customReadMessages() override;
    void customWriteMessages(uint64_t CurrentClock) override;
    void customReset(uint64_t CurrentClock) override;
    void evaluateDataModel(DataNodeUsageMsgPayload *dataUsageMsg, double currentTime) override;

    int64_t selectStorageIndex() const;
    double getStorageBitsAtIndex(int64_t index) const;
    void setDataNameFromStorageIndex(int64_t index, char *buffer, std::size_t bufferSize) const;
    void selectReceiver(double *cnrLinear, uint32_t *receiverIndex) const;
    static bool isReceiverState(uint32_t state);
    static double computeBerFromEbN0dB(double ebN0_dB);
    static double clampProbability(double value);

private:
    std::vector<ReadFunctor<DataStorageStatusMsgPayload>> storageUnitInMsgs; //!< Storage status subscribers
    std::vector<DataStorageStatusMsgPayload> storageUnitMsgsBuffer;           //!< Local storage status buffers

    LinkBudgetMsgPayload linkBudgetBuffer;                        //!< Local copy of link-budget message
    bool linkBudgetValid;                                         //!< True when linkBudget message is linked and written

    DownlinkHandlingMsgPayload downlinkOutBuffer;                 //!< Local copy of downlink output message

    double previousTime;                                          //!< [s] previous simulation time
    double currentTimeStep;                                       //!< [s] current integration step
    double availableDataBits;                                     //!< [bit] selected storage data available

    double cumulativeAttemptedBits;                               //!< [bit] attempted channel bits (includes retransmissions)
    double cumulativeRemovedBits;                                 //!< [bit] bits removed from storage
    double cumulativeDeliveredBits;                               //!< [bit] successfully delivered bits
    double cumulativeDroppedBits;                                 //!< [bit] dropped bits after retransmission limit
};

#endif
