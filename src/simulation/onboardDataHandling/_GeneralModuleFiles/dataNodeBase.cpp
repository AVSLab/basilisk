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
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "dataNodeBase.h"
#include "string.h"

/*! Constructor.
 @return void
 */
DataNodeBase::DataNodeBase()
{
    this->outputBufferCount = 2;
    this->nodeDataOutMsgName = "dataNodeOutputMessage";
    this->nodeStatusInMsgName = ""; //!<By default, no node status message name is used.
    this->nodeDataOutMsgId = -1;
    this->nodeStatusInMsgId = -1;
    this->dataStatus = 1; //!< Node defaults to on unless overwritten.
    memset(&(this->nodeDataMsg), 0x0, sizeof(DataNodeUsageSimMsg)); //!< Data node message is zero by default.

    return;
}

/*! Destructor.
 @return void
 */
DataNodeBase::~DataNodeBase()
{
    return;
}

/*!
 \verbatim embed:rst
    SelfInit creates a :ref:`DataNodeUsageSimMsg` using the provided message output name.
 \endverbatim
 @return void
 */
void DataNodeBase::SelfInit()
{
    this->nodeDataOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->nodeDataOutMsgName, sizeof(DataNodeUsageSimMsg),this->outputBufferCount, "DataNodeUsageSimMsg",this->moduleID);
    //! - call the custom SelfInit() method to add additional self initialization steps
    customSelfInit();

    return;
}

/*! This method subscribes to anything that would tell the data node to turn on/off.
 @return void
 */
void DataNodeBase::CrossInit()
{
    //! - subscribe to the deviceStatus messages and create associated output message buffer
    if(this->nodeStatusInMsgName.length() > 0) {
        this->nodeStatusInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->nodeStatusInMsgName,
                                                                                     sizeof(DeviceStatusIntMsg),
                                                                                     moduleID);
    }
    //!- call the custom CrossInit() method to all additional cross initialization steps
    customCrossInit();
    return;
}

/*! This method is used to reset the module. In general, no functionality is reset.
 @param CurrentSimNanos
 @return void
 */
void DataNodeBase::Reset(uint64_t CurrentSimNanos)
{
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}

/*! This method writes out the data node messages (dataName, baudRate)
 @param CurrentClock
 @return void
 */
void DataNodeBase::writeMessages(uint64_t CurrentClock)
{
    //! - write dataNode output messages - baud rate and name
    SystemMessaging::GetInstance()->WriteMessage(this->nodeDataOutMsgId,
                                                 CurrentClock,
                                                 sizeof(DataNodeUsageSimMsg),
                                                 reinterpret_cast<uint8_t*>(&(this->nodeDataMsg)),
                                                 moduleID);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}

/*! This method reads the device status messages and calls a customReadMessages method
 @return bool
 */
bool DataNodeBase::readMessages()
{
    DeviceStatusIntMsg statusMsg;
    SingleMessageHeader localHeader;

    //! - read in the data node use/supply messages
    bool dataRead = true;
    bool tmpStatusRead = true;
    if(this->nodeStatusInMsgId >= 0)
    {
        memset(&statusMsg, 0x0, sizeof(DeviceStatusIntMsg));
        tmpStatusRead = SystemMessaging::GetInstance()->ReadMessage(this->nodeStatusInMsgId, &localHeader,
                                                                    sizeof(DeviceStatusIntMsg),
                                                                    reinterpret_cast<uint8_t*>(&statusMsg),
                                                                    moduleID);

        this->nodeStatusMsg = statusMsg;
        this->dataStatus = this->nodeStatusMsg.deviceStatus;
        dataRead = dataRead && tmpStatusRead;
    }

    //! - call the custom method to perform additional input reading
    bool customRead = this->customReadMessages();

    return(dataRead && customRead);
}

/*! This method evaluates the implementation-specific data model if the device is set to on.
 @param CurrentTime
 @return void
 */
void DataNodeBase::computeDataStatus(double CurrentTime)
{
    if(this->dataStatus > 0)
    {
        this->evaluateDataModel(&this->nodeDataMsg, CurrentTime);
    }
    else
    {
        memset(&(this->nodeDataMsg), 0x0, sizeof(DataNodeUsageSimMsg));
    }
    return;
}

/*! This method updates the state by reading messages, calling computeDataStatus, and writing messages
 @param CurrentSimNanos
 @return void
 */
void DataNodeBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Only update the data status if we were able to read in messages.
    if(this->readMessages())
    {
        this->computeDataStatus(CurrentSimNanos*NANO2SEC);
    } else {
        //! - If the read was not successful then zero the output message
        memset(&(this->nodeDataMsg), 0x0, sizeof(DataNodeUsageSimMsg));
    }

    this->writeMessages(CurrentSimNanos);
    return;
}


/*! Custom SelfInit() method.  This allows a child class to add additional functionality to the SelfInit() method
 @return void
 */
void DataNodeBase::customSelfInit()
{
    return;
}

/*! Custom CrossInit() method.  This allows a child class to add additional functionality to the CrossInit() method
 @return void
 */
void DataNodeBase::customCrossInit()
{
    return;
}

/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method
 @return void
 */
void DataNodeBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! custom Write method, similar to customSelfInit.
 @return void
 */
void DataNodeBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! Custom read method, similar to customSelfInit; returns `true' by default.
 @return void
 */
bool DataNodeBase::customReadMessages()
{
    return true;
}


