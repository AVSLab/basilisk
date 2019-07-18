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
#include "simPowerStorageBase.h"


/*! This method initializes some basic parameters for the module.
 @return void
 */
PowerStorageBase::PowerStorageBase()
{
    this->outputBufferCount = 2;
    this->previousTime = 0;
    this->nodePowerUseMsgNames.clear();

    return;
}

/*! Destructor.
 @return void
 */
PowerStorageBase::~PowerStorageBase()
{
    return;
}


/*! Adds the spacecraft message name to a vector of sc message names and automatically creates an output message name.
 @return void
 @param tmpScMsgName A spacecraft state message name.
 */
void PowerStorageBase::addPowerNodeToModel(std::string tmpNodeMsgName){
    this->nodePowerUseMsgNames.push_back(tmpNodeMsgName);
    return;
}

/*! SelfInit for this method creates a seperate magnetic field message for each of the spacecraft
that were added using AddSpacecraftToModel. Additional model outputs are also initialized per-spacecraft.
 @return void
*/
void PowerStorageBase::SelfInit()
{

    this->batPowerOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->batPowerOutMsgName, sizeof(PowerStorageStatusSimMsg),this->outputBufferCount, "PowerStorageStatusSimMsg",this->moduleID);

    //! - call the custom SelfInit() method to add addtional self initialization steps
    customSelfInit();

    return;
}

/*! This method is used to connect the input position message from the spacecraft. Additonal model-specific cross inits are also conducted.
 @return void
 */
void PowerStorageBase::CrossInit()
{
    //! - subscribe to the spacecraft messages and create associated output message buffer
    std::vector<std::string>::iterator it;
    for(it = this->nodePowerUseMsgNames.begin(); it != this->nodePowerUseMsgNames.end(); it++){
        this->nodePowerUseMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(PowerNodeUsageSimMsg), moduleID));
    }

    //!- call the custom CrossInit() method to all additional cross initialization steps
    customCrossInit();

    return;
}

/*! This method is used to reset the module.
 @return void
 */
void PowerStorageBase::Reset(uint64_t CurrentSimNanos)
{
    this->previousTime = 0;
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}



/*! Custom SelfInit() method.  This allows a child class to add additional functionality to the SelfInit() method
 @return void
 */
void PowerStorageBase::customSelfInit()
{
    return;
}

/*! Custom CrossInit() method.  This allows a child class to add additional functionality to the CrossInit() method
 @return void
 */
void PowerStorageBase::customCrossInit()
{
    return;
}

/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method
 @return void
 */
void PowerStorageBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to write the output magnetic field messages whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void PowerStorageBase::writeMessages(uint64_t CurrentClock)
{
    std::vector<int64_t>::iterator it;
    //! - write magnetic field output messages for each spacecaft's locations
    SystemMessaging::GetInstance()->WriteMessage(this->batPowerOutMsgId,
                                                     CurrentClock,
                                                     sizeof(PowerStorageStatusSimMsg),
                                                     reinterpret_cast<uint8_t*> (&storageStatusMsg),
                                                     moduleID);

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}

/*! Custom output message writing method.  This allows a child class to add additional functionality.
 @return void
 */
void PowerStorageBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to read the incoming power supply/usage messages and store them for future use.
 @return void
 */
bool PowerStorageBase::readMessages()
{
    PowerNodeUsageSimMsg nodeMsg;
    SingleMessageHeader localHeader;

    this->nodeWattMsgs.clear();

    //! - read in the power node use/supply messages
    bool powerRead = true;
    bool tmpPowerRead;
    if(this->nodePowerUseMsgIds.size() > 0)
    {
        std::vector<int64_t>::iterator it;
        for(it = nodePowerUseMsgIds.begin(); it!= nodePowerUseMsgIds.end(); it++)
        {
            memset(&nodeMsg, 0x0, sizeof(PowerNodeUsageSimMsg));
            tmpPowerRead = SystemMessaging::GetInstance()->ReadMessage(*it, &localHeader,
                                                                    sizeof(PowerNodeUsageSimMsg),
                                                                    reinterpret_cast<uint8_t*>(&nodeMsg),
                                                                    moduleID);
            powerRead = powerRead && tmpPowerRead;

            this->nodeWattMsgs.push_back(nodeMsg);
        }
    }
    else {
        BSK_PRINT(MSG_WARNING, "Power storage has no power node messages to read.");
        powerRead = false;
    }

    //! - call the custom method to perform additional input reading
    bool customRead = customReadMessages();

    return(powerRead && customRead);
}


/*! Custom output input reading method.  This allows a child class to add additional functionality.
 @return void
 */
bool PowerStorageBase::customReadMessages()
{
    return true;
}

double PowerStorageBase::sumAllInputs(){
    double currentSum = 0.0;

    std::vector<PowerNodeUsageSimMsg>::iterator it;
    for(it = nodeWattMsgs.begin(); it != nodeWattMsgs.end(); it++) {

        currentSum += (*it).netPower_W;
    }

    return currentSum;
}

/*! This method integrates the power use provided by the attached modules.
  @return void
 */
void PowerStorageBase::integratePowerStatus(double currentTime)
{

    this->currentTimestep = currentTime - this->previousTime;

    //! - loop over all the power nodes and sum their contributions
    this->currentPowerSum = this->sumAllInputs();


    this->evaluateBatteryModel(&(storageStatusMsg), currentTime); // Computes the battery charge status, if applicable.
    this->previousTime = currentTime;
    return;
}


/*! Computes the current local magnetic field for each spacecraft and writes their respective messages.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void PowerStorageBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - update local neutral density information
    if(this->readMessages())
    {
        this->integratePowerStatus(CurrentSimNanos*NANO2SEC);
    }

    //! - write out neutral density message
    this->writeMessages(CurrentSimNanos);

    return;
}
