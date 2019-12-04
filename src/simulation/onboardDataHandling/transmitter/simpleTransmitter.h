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

#ifndef BASILISK_SIMPLETRANSMITTER_H
#define BASILISK_SIMPLETRANSMITTER_H

#include "onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h"

class SimpleTransmitter: public DataNodeBase {
public:
    SimpleTransmitter();
    ~SimpleTransmitter();
    void addStorageUnitToTransmitter(std::string tmpStorageUnitMsgName);

private:
    void evaluateDataModel(DataNodeUsageSimMsg *dataUsageMsg, double currentTime);
    bool customReadMessages();
    void customCrossInit();

public:
    double packetSize; //!< Size of packet to downklink (bytes)
    int numBuffers; //!< Number of buffers the transmitter can access
    std::vector<std::string> storageUnitMsgNames; //!< Vector of data node input message names
    std::vector<std::int64_t> storageUnitMsgIds;
    std::vector<DataStorageStatusSimMsg> storageUnitMsgs;

private:
    double packetTransmitted; //!< Amount of packet downlinked (bytes)
    double currentTimestep; //!< Current timestep tracked for data packet integration
    double previousTime; //!< Previous timestep tracked for data packet integration
    std::vector<dataInstance> storedData; //! Vector of data. Represents the makeup of the data buffer. Created from input messages.
};


#endif //BASILISK_SIMPLETRANSMITTER_H
