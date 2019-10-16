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
#include "powerNodeBase.h"


/*! This method initializes the messaging parameters to either empty strings for message names or -1 for message IDs.
 @return void
 */
PowerNodeBase::PowerNodeBase()
{
    this->outputBufferCount = 2;

    this->nodePowerOutMsgName = "powerNodeOutputMessage";
    this->nodeStatusInMsgName = ""; //By default, no node status message name is used.
    this->nodePowerOutMsgId = -1;
    this->nodeStatusInMsgId = -1;
    this->powerStatus = 1; //! Node defaults to on unless overwritten.
    memset(&(this->nodePowerMsg), 0x0, sizeof(PowerNodeUsageSimMsg)); //! Power node message is zero by default.
    return;
}

/*! Destructor.
 @return void
 */
PowerNodeBase::~PowerNodeBase()
{
    return;
}

/*! SelfInit creates a PowerNodeUsageSimMsg using the provided message output name.
 @return void
*/
void PowerNodeBase::SelfInit()
{
    this->nodePowerOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->nodePowerOutMsgName, sizeof(PowerNodeUsageSimMsg),this->outputBufferCount, "PowerNodeUsageSimMsg",this->moduleID);
    //! - call the custom SelfInit() method to add addtional self initialization steps
    customSelfInit();

    return;
}

/*! This method subscribes to anything that would tell the power node to turn on/off.
 @return void
 */
void PowerNodeBase::CrossInit()
{
    //! - subscribe to the spacecraft messages and create associated output message buffer
    if(this->nodeStatusInMsgName.length() > 0) {
        this->nodeStatusInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->nodeStatusInMsgName,
                                                                                     sizeof(PowerNodeStatusIntMsg),
                                                                                     moduleID);
    }
    //!- call the custom CrossInit() method to all additional cross initialization steps
    customCrossInit();

    return;
}

/*! This method is used to reset the module. In general, no functionality must be reset.
 @return void
 */
void PowerNodeBase::Reset(uint64_t CurrentSimNanos)
{
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}

/*! This method writes out a message.
 @return void
 */
void PowerNodeBase::writeMessages(uint64_t CurrentClock)
{
    std::vector<int64_t>::iterator it;
    //! - write magnetic field output messages for each spacecaft's locations
    SystemMessaging::GetInstance()->WriteMessage(this->nodePowerOutMsgId,
                                                 CurrentClock,
                                                 sizeof(PowerNodeUsageSimMsg),
                                                 reinterpret_cast<uint8_t*>(&nodePowerMsg),
                                                         moduleID);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}


/*! This method is used to read incoming power status messages.
 @return void
 */
bool PowerNodeBase::readMessages()
{
    PowerNodeStatusIntMsg statusMsg;
    SingleMessageHeader localHeader;

    //! - read in the power node use/supply messages
    bool powerRead = true;
    bool tmpStatusRead = true;
    if(this->nodeStatusInMsgId >= 0)
    {
        memset(&statusMsg, 0x0, sizeof(PowerNodeStatusIntMsg));
        tmpStatusRead = SystemMessaging::GetInstance()->ReadMessage(this->nodeStatusInMsgId, &localHeader,
                                                                       sizeof(PowerNodeStatusIntMsg),
                                                                       reinterpret_cast<uint8_t*>(&statusMsg),
                                                                       moduleID);

        this->nodeStatusMsg = statusMsg;
        this->powerStatus = this->nodeStatusMsg.powerStatus;
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
        this->nodePowerMsg.netPower_W = 0.0;
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
    }
    this->writeMessages(CurrentSimNanos);

    return;
}
