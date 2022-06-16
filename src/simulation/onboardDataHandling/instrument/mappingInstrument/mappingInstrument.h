/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef MAPPINGINSTRUMENT_H
#define MAPPINGINSTRUMENT_H

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/AccessMsgPayload.h"
#include "architecture/msgPayloadDefC/DataNodeUsageMsgPayload.h"
#include "architecture/messaging/messaging.h"

/*! @brief This module receives a vector of accessMsgPayloads and outputs a vector of DataNodeUsageMsgPayloads for each accessible point.
 */
class MappingInstrument: public SysModel {
public:
    MappingInstrument();
    ~MappingInstrument();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void addMappingPoint(Message<AccessMsgPayload> *tmpAccessMsg, std::string dataName); //!< connects accessMsgPayload to instrument

public:
    std::vector<Message<DataNodeUsageMsgPayload>*> dataNodeOutMsgs; //!< vector of data node output messages
    std::vector<ReadFunctor<AccessMsgPayload>> accessInMsgs; //!< vector of ground location access messages
    BSKLogger bskLogger; //!< -- BSK Logging
    double nodeBaudRate = -1; //!< [baud] Data provided (+).

private:
    std::vector<std::string> mappingPoints;
    std::vector<DataNodeUsageMsgPayload> dataNodeOutMsgBuffer;                  //!< buffer of data node output data

};

#endif //BASILISK_MAPPINGINSTRUMENT_H
