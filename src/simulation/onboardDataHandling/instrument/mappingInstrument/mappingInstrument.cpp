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

#include "simulation/onboardDataHandling/instrument/mappingInstrument/mappingInstrument.h"
#include "string.h"

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
MappingInstrument::MappingInstrument()
{
}

/*! Module Destructor */
MappingInstrument::~MappingInstrument()
{
    for (long unsigned int c = 0; c < this->dataNodeOutMsgs.size(); c++) {
        delete this->dataNodeOutMsgs.at(c);
    }
}

/*! This method is used to reset the module. The nodeBaudRate is checked for a non-zero value.
 @param CurrentSimNanos

 */
void MappingInstrument::Reset(uint64_t CurrentSimNanos)
{
    // check that the baud rate is set
    if (this->nodeBaudRate < 0.0){
        bskLogger.bskLog(BSK_ERROR, "MappingInstrument.nodeBaudRate is not set to a positive value.");
    }

    return;
}

/*! This method updates the state by reading messages, calling computeDataStatus, and writing messages
 @param CurrentSimNanos

 */
void MappingInstrument::UpdateState(uint64_t CurrentSimNanos)
{
    /* Loop through each access message */
    for (long unsigned int c=0; c< this->accessInMsgs.size(); c++) {
        /* Zero the output message buffer */
        this->dataNodeOutMsgBuffer.at(c) = this->dataNodeOutMsgs.at(c)->zeroMsgPayload;

        /* Read the access message */
        AccessMsgPayload accessMsg;
        accessMsg = this->accessInMsgs.at(c)();

        /* Check for access, set the data rate */
        if (accessMsg.hasAccess) {
            dataNodeOutMsgBuffer.at(c).baudRate = this->nodeBaudRate;
        } else {
            dataNodeOutMsgBuffer.at(c).baudRate = 0;
        }

        strcpy(dataNodeOutMsgBuffer.at(c).dataName, mappingPoints[c].c_str());

        /* Write the output message */
        this->dataNodeOutMsgs.at(c)->write(&this->dataNodeOutMsgBuffer.at(c), this->moduleID, CurrentSimNanos);
    }

    return;
}


/*! Adds a mapping point (access message and name) to the module
 *
*/
void MappingInstrument::addMappingPoint(Message<AccessMsgPayload> *tmpAccessMsg, std::string dataName){
    /* Add the name of the mapping point */
    this->mappingPoints.push_back(dataName);

    /* Add the access message */
    this->accessInMsgs.push_back(tmpAccessMsg->addSubscriber());

    /* Create buffer output messages */
    Message<DataNodeUsageMsgPayload> *msg;
    msg = new Message<DataNodeUsageMsgPayload>;
    this->dataNodeOutMsgs.push_back(msg);

    /* Expand the data node usage buffer vectors */
    DataNodeUsageMsgPayload dataNodeUsageMsg;
    this->dataNodeOutMsgBuffer.push_back(dataNodeUsageMsg);

    return;
}
