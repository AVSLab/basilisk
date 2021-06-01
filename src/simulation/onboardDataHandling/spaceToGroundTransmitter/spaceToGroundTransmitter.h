
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

#ifndef BASILISK_SPACETOGROUNDTRANSMITTER_H
#define BASILISK_SPACETOGROUNDTRANSMITTER_H

#include "simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h"
#include "architecture/msgPayloadDefC/AccessMsgPayload.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief space to ground data transmitter class */
class SpaceToGroundTransmitter: public DataNodeBase {
public:
    SpaceToGroundTransmitter();
    ~SpaceToGroundTransmitter();
    void addStorageUnitToTransmitter(Message<DataStorageStatusMsgPayload> *tmpStorageUnitMsg);
    void addAccessMsgToTransmitter(Message<AccessMsgPayload> *tmpAccessMsg);

private:
    void evaluateDataModel(DataNodeUsageMsgPayload *dataUsageMsg, double currentTime);
    bool customReadMessages();

public:
    double packetSize; //!< Size of packet to downklink (bytes)
    int numBuffers; //!< Number of buffers the transmitter can access
    std::vector<ReadFunctor<DataStorageStatusMsgPayload>> storageUnitInMsgs; //!< vector of input messages for storage unit messages
    std::vector<ReadFunctor<AccessMsgPayload>> groundLocationAccessInMsgs;   //!< vector of input message for ground location access
    std::vector<DataStorageStatusMsgPayload> storageUnitMsgsBuffer;   //!< local copy of data storage messages
    uint64_t hasAccess;                                     //!< class variable
    BSKLogger bskLogger;                                    //!< class variable

private:
    double packetTransmitted; //!< Amount of packet downlinked (bytes)
    double currentTimestep; //!< Current timestep tracked for data packet integration
    double previousTime; //!< Previous timestep tracked for data packet integration
    std::vector<AccessMsgPayload> groundLocationAccessMsgs; //!< local copy of ground access messages
};
#endif //BASILISK_SPACETOGROUNDTRANSMITTER_H
