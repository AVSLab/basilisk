//
// Created by andrew on 7/12/19.
//

//
// Created by andrew on 7/9/19.
//

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
#include "simPowerNodeBase.h"


/*! This method initializes some basic parameters for the module.
 @return void
 */
PowerNodeBase::PowerNodeBase()
{
    this->outputBufferCount = 2;

    this->nodePowerOutMsgName = "powerNodeOutputMessage";
    this->nodeStatusInMsgName = ""; //By default, no node status message name is used.
    this->nodePowerOutMsgId = -1;
    this->nodeStatusInMsgId = -1;
    this->powerStatus = 1; //Node defaults to on unless overwritten.
    return;
}

/*! Destructor.
 @return void
 */
PowerNodeBase::~PowerNodeBase()
{
    return;
}

/*! SelfInit for this method creates a seperate magnetic field message for each of the spacecraft
that were added using AddSpacecraftToModel. Additional model outputs are also initialized per-spacecraft.
 @return void
*/
void PowerNodeBase::SelfInit()
{

    this->nodePowerOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->nodePowerOutMsgName, sizeof(PowerNodeUsageSimMsg),this->outputBufferCount, "PowerNodeUsageSimMsg",this->moduleID);

    //! - call the custom SelfInit() method to add addtional self initialization steps
    customSelfInit();

    return;
}

/*! This method is used to connect the input position message from the spacecraft. Additonal model-specific cross inits are also conducted.
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

/*! This method is used to reset the module.
 @return void
 */
void PowerNodeBase::Reset(uint64_t CurrentSimNanos)
{
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}

/*! Custom SelfInit() method.  This allows a child class to add additional functionality to the SelfInit() method
 @return void
 */
void PowerNodeBase::customSelfInit()
{
    return;
}

/*! Custom CrossInit() method.  This allows a child class to add additional functionality to the CrossInit() method
 @return void
 */
void PowerNodeBase::customCrossInit()
{
    return;
}

/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method
 @return void
 */
void PowerNodeBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to write the output magnetic field messages whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message
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

/*! Custom output message writing method.  This allows a child class to add additional functionality.
 @return void
 */
void PowerNodeBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to read the incoming power supply/usage messages and store them for future use.
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


/*! Custom output input reading method.  This allows a child class to add additional functionality.
 @return void
 */
bool PowerNodeBase::customReadMessages()
{
    return true;
}

/*! Computes the current power consumption for the module.
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

/*! Computes the current local magnetic field for each spacecraft and writes their respective messages.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void PowerNodeBase::UpdateState(uint64_t CurrentSimNanos)
{
    memset(&(this->nodePowerMsg), 0x0, sizeof(PowerNodeUsageSimMsg));

    //! - update local neutral density information
    if(this->readMessages())
    {
        this->computePowerStatus(CurrentSimNanos*NANO2SEC);
    }

    //! - write out neutral density message
    this->writeMessages(CurrentSimNanos);

    return;
}
