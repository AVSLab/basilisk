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

#include "architecture/utilities/macroDefinitions.h"
#include "dataNodeBase.h"
#include "string.h"

/*! Constructor.

 */
DataNodeBase::DataNodeBase()
{
    this->dataStatus = 1; //!< Node defaults to on unless overwritten.
    this->nodeDataMsg = this->nodeDataOutMsg.zeroMsgPayload;

    return;
}

/*! Destructor.

 */
DataNodeBase::~DataNodeBase()
{
    return;
}


/*! This method is used to reset the module. In general, no functionality is reset.
 @param CurrentSimNanos

 */
void DataNodeBase::Reset(uint64_t CurrentSimNanos)
{
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}

/*! This method writes out the data node messages (dataName, baudRate)
 @param CurrentClock

 */
void DataNodeBase::writeMessages(uint64_t CurrentClock)
{
    //! - write dataNode output messages - baud rate and name
    this->nodeDataOutMsg.write(&this->nodeDataMsg, this->moduleID, CurrentClock);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);
    return;
}

/*! This method reads the device status messages and calls a customReadMessages method
 @return bool
 */
bool DataNodeBase::readMessages()
{
    //! - read in the data node use/supply messages
    bool dataRead = true;
    bool tmpStatusRead = true;
    if(this->nodeStatusInMsg.isLinked())
    {
        this->nodeStatusMsg = this->nodeStatusInMsg();
        this->dataStatus = this->nodeStatusMsg.deviceCmd;
        tmpStatusRead = this->nodeStatusInMsg.isWritten();
        dataRead = dataRead && tmpStatusRead;
    }

    //! - call the custom method to perform additional input reading
    bool customRead = this->customReadMessages();
    return(dataRead && customRead);
}

/*! This method evaluates the implementation-specific data model if the device is set to on.
 @param CurrentTime

 */
void DataNodeBase::computeDataStatus(double CurrentTime)
{
    if(this->dataStatus > 0)
    {
        this->evaluateDataModel(&this->nodeDataMsg, CurrentTime);
    }
    else
    {
        this->nodeDataMsg = this->nodeDataOutMsg.zeroMsgPayload;
    }
    return;
}

/*! This method updates the state by reading messages, calling computeDataStatus, and writing messages
 @param CurrentSimNanos

 */
void DataNodeBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Only update the data status if we were able to read in messages.
    if(this->readMessages())
    {
        this->computeDataStatus(CurrentSimNanos*NANO2SEC);
    } else {
        //! - If the read was not successful then zero the output message
        this->nodeDataMsg = this->nodeDataOutMsg.zeroMsgPayload;
    }

    this->writeMessages(CurrentSimNanos);
    return;
}


/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method

 */
void DataNodeBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! custom Write method, similar to customSelfInit.

 */
void DataNodeBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! Custom read method, similar to customSelfInit; returns `true' by default.

 */
bool DataNodeBase::customReadMessages()
{
    return true;
}
