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

#include "architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"
#include "utilities/linearAlgebra.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "dataNodeBase.h"
#include <string.h>

/*! This method initializes the messaging parameters to either empty strings for message names or -1 for message IDs.
 @return void
 */
DataNodeBase::DataNodeBase()
{
    this->outputBufferCount = 2;
    this->nodeDataOutMsgName = "dataNodeOutputMessage";
    this->nodeStatusInMsgName = ""; //By default, no node status message name is used.
    this->nodeDataOutMsgId = -1;
    this->nodeStatusInMsgId = -1;
    this->dataStatus = 1; //! Node defaults to on unless overwritten.
    memset(&(this->nodeDataMsg), 0x0, sizeof(DataNodeUsageSimMsg)); //! Data node message is zero by default.

    return;
}

/*! Destructor.
 @return void
 */
DataNodeBase::~DataNodeBase()
{
    return;
}

/*! SelfInit creates a PowerNodeUsageSimMsg using the provided message output name.
 @return void
*/
void DataNodeBase::SelfInit()
{

    this->nodeDataOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->nodeDataOutMsgName, sizeof(DataNodeUsageSimMsg),this->outputBufferCount, "DataNodeUsageSimMsg",this->moduleID);
    //! - call the custom SelfInit() method to add additional self initialization steps
    customSelfInit();

    return;
}

/*! This method subscribes to anything that would tell the power node to turn on/off.
 @return void
 */
void DataNodeBase::CrossInit()
{
    //! - subscribe to the spacecraft messages and create associated output message buffer
    if(this->nodeStatusInMsgName.length() > 0) {
        this->nodeStatusInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->nodeStatusInMsgName,
                                                                                     sizeof(DataNodeStatusIntMsg),
                                                                                     moduleID);
    }
    //!- call the custom CrossInit() method to all additional cross initialization steps
    customCrossInit();
    return;
}

void DataNodeBase::Reset(uint64_t CurrentSimNanos)
{
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}

void DataNodeBase::writeMessages(uint_64t CurrentClock)
{
    std::vector<int64_t>::iterator it;
    //! - write magnetic field output messages for each spacecaft's locations
    SystemMessaging::GetInstance()->WriteMessage(this->nodeDataOutMsgId,
                                                 CurrentClock,
                                                 sizeof(DataNodeUsageSimMsg),
                                                 reinterpret_cast<uint8_t*>(&(this->nodeDataMsg)),
                                                 moduleID);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);
    return;
}

bool DataNodeBase::readMessages()
{
    DataNodeStatusIntMsg statusMsg;
    SingleMessageHeader localHeader;

    //! - read in the power node use/supply messages
    bool dataRead = true;
    bool tmpStatusRead = true;
    if(this->nodeStatusInMsgId >= 0)
    {
        memset(&statusMsg, 0x0, sizeof(DataNodeStatusIntMsg));
        tmpStatusRead = SystemMessaging::GetInstance()->ReadMessage(this->nodeStatusInMsgId, &localHeader,
                                                                    sizeof(DataNodeStatusIntMsg),
                                                                    reinterpret_cast<uint8_t*>(&statusMsg),
                                                                    moduleID);

        this->nodeStatusMsg = statusMsg;
        this->dataStatus = this->nodeStatusMsg.dataStatus;
        dataRead = dataRead && tmpStatusRead;
    }

    //! - call the custom method to perform additional input reading
    bool customRead = this->customReadMessages();

    return(dataRead && customRead);
}

void DataNodeBase::computeDataStatus(double CurrentTime)
{
    if(this->dataStatus > 0)
    {
        this->evaluateDataModel(&this->nodeDataMsg);
    }
    else
    {
        memset(&(this->nodeDataMsg), 0x0, sizeof(DataNodeUsageSimMsg));
    }
    return;
}

void DataNodeBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Only update the power status if we were able to read in messages.
    if(this->readMessages())
    {
        this->computeDataStatus(CurrentSimNanos*NANO2SEC);
    } else {
        /* if the read was not successful then zero the output message */
        memset(&(this->nodeDataMsg), 0x0, sizeof(DataNodeUsageSimMsg));
    }

    this->writeMessages(CurrentSimNanos);
    return;
}