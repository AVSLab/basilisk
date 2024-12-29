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
#include "powerNodeBase.h"

/*! This method initializes the messaging parameters to either empty strings for message names or -1 for message IDs.

 */
PowerNodeBase::PowerNodeBase()
{
    this->powerStatus = 1; //! Node defaults to on unless overwritten.
    this->nodePowerMsg = this->nodePowerOutMsg.zeroMsgPayload;  //! Power node message is zero by default.
    return;
}

/*! Destructor.

 */
PowerNodeBase::~PowerNodeBase()
{
    return;
}


/*! This method is used to reset the module. In general, no functionality must be reset.

 */
void PowerNodeBase::Reset(uint64_t CurrentSimNanos)
{
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}

/*! This method writes out a message.

 */
void PowerNodeBase::writeMessages(uint64_t CurrentClock)
{
    //! - write power output message
    this->nodePowerOutMsg.write(&this->nodePowerMsg, this->moduleID, CurrentClock);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}


/*! This method is used to read incoming power status messages.

 */
bool PowerNodeBase::readMessages()
{
    DeviceStatusMsgPayload statusMsg;

    //! - read in the power node use/supply messages
    bool powerRead = true;
    bool tmpStatusRead = true;
    if(this->nodeStatusInMsg.isLinked())
    {
        statusMsg = this->nodeStatusInMsg();
        tmpStatusRead = this->nodeStatusInMsg.isWritten();
        this->nodeStatusMsg = statusMsg;
        this->powerStatus = this->nodeStatusMsg.deviceStatus;
        powerRead = powerRead && tmpStatusRead;
    }


    //! - call the custom method to perform additional input reading
    bool customRead = this->customReadMessages();

    return(powerRead && customRead);
}

/*! Core compute operation that implements switching logic and computes module-wise power consumption.
 */

void PowerNodeBase::computePowerStatus(double currentTime)
{
    if(this->powerStatus > 0)
    {
        this->evaluatePowerModel(&this->nodePowerMsg);
    }
    else
    {
        this->nodePowerMsg = this->nodePowerOutMsg.zeroMsgPayload;
    }

    return;
}

/*! Provides logic for running the read / compute / write operation that is the module's function.
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void PowerNodeBase::UpdateState(uint64_t CurrentSimNanos)
{

    //! - Only update the power status if we were able to read in messages.
    if(this->readMessages())
    {
        this->computePowerStatus(CurrentSimNanos*NANO2SEC);
    } else {
        /* if the read was not successful then zero the output message */
        this->nodePowerMsg = this->nodePowerOutMsg.zeroMsgPayload;
    }

    this->writeMessages(CurrentSimNanos);

    return;
}



/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method

 */
void PowerNodeBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! custom Write method, similar to customSelfInit.

 */
void PowerNodeBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! Custom read method, similar to customSelfInit; returns `true' by default.

 */
bool PowerNodeBase::customReadMessages()
{
    return true;
}
