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

#include "simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase.h"
#include "architecture/msgPayloadDefC/AccessMsgPayload.h"

/*! @brief This module receives a vector of accessMsgPayloads and outputs a vector of DataNodeUsageMsgPayloads for each accessible point.
 */
class MappingInstrument: public DataNodeBase {
public:
    MappingInstrument();
    ~MappingInstrument();

public:
    std::vector<Message<DataNodeUsageMsgPayload>*> dataNodeOutMsgs;    //!< vector of ground location output message
    std::vector<ReadFunctor<AccessMsgPayload>*> accessInMsgs;           //!< vector of ground location access messages

    BSKLogger bskLogger;              //!< -- BSK Logging

private:
    void evaluateDataModel(DataNodeUsageMsgPayload *dataUsageMsg, double currentTime); //!< Sets the name and baud rate for the data in the output message.

};


#endif
