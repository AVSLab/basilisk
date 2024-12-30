/*
 ISC License

 Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "ReactionWheelPower.h"
#include <math.h>

/*! Constructor
*/
ReactionWheelPower::ReactionWheelPower(){

    this->basePowerNeed = 0.0;
    this->elecToMechEfficiency = 1.0;   //!< default efficiency is 100%
    this->mechToElecEfficiency = -1.0;  //!< negative value turns of mechanical to electrical energy conversion
    return;

}

ReactionWheelPower::~ReactionWheelPower(){

    return;
}


/*! This method is used to reset the module. Here variables are checked for correct values.

 */
void ReactionWheelPower::customReset(uint64_t CurrentSimNanos)
{
    if (this->elecToMechEfficiency <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "PowerRW: elecToMechEfficiency is %f, must a strictly positive value.",
                         this->elecToMechEfficiency);
    }
    return;
}

/*! This method is used to read incoming RW state message.

 */
bool ReactionWheelPower::customReadMessages()
{
    RWConfigLogMsgPayload statusMsg;

    //! - read in the power node use/supply messages
    bool rwRead = true;
    bool tmpStatusRead = false;
    if(this->rwStateInMsg.isLinked())
    {
        statusMsg = this->rwStateInMsg();
        tmpStatusRead = this->rwStateInMsg.isWritten();
        this->rwStatus = statusMsg;

        rwRead = rwRead && tmpStatusRead;
    }

    if (!rwRead){
        bskLogger.bskLog(BSK_WARNING, "PowerRW unable to read RW status message.");
    }

    return(rwRead);
}


/*! Computes the RW power load. Determines the netPower attribute in powerUsageSimMessage.
*/
void ReactionWheelPower::evaluatePowerModel(PowerNodeUsageMsgPayload *powerUsageSimMsg){
    double rwPowerNeed;
    double wheelPower;

    /* add base RW power consumption */
    rwPowerNeed = this->basePowerNeed;

    /* evaluate power required to torque RW */
    wheelPower = this->rwStatus.Omega*this->rwStatus.u_current;
    if (wheelPower > 0.0 ||         /* accelerating the wheel to larger Omega values always takes power */
        this->mechToElecEfficiency < 0.0) {  /* if negative model the breaking as taking power as well */
        rwPowerNeed += fabs(wheelPower)/ this->elecToMechEfficiency;
    } else {
        /* breaking the wheeel speed where some mechanics energy is recovered */
        rwPowerNeed += this->mechToElecEfficiency * wheelPower;
    }

    /* flip sign as a positive RW power requirement is a negative draw on the power system */
    powerUsageSimMsg->netPower = rwPowerNeed * (-1);

    return;
}
