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
#include "../../simMessages/powerNodeUsageSimMsg.h"
#include "architecture/messaging/system_messaging.h"
#include <math.h>

/*! Constructor
*/
ReactionWheelPower::ReactionWheelPower(){

    this->basePowerNeed = 0.0;
    this->eta_e2m = 1.0;            //!< default efficiency is 100%
    this->eta_m2e = -1.0;           //!< negative value turns of mechanical to electrical energy conversion
    return;

}

ReactionWheelPower::~ReactionWheelPower(){

    return;
}

/*! This method subscribes to the RW state message.
 @return void
 */
void ReactionWheelPower::customCrossInit()
{
    //! - subscribe to the RW state message
    if(this->rwStateInMsgName.length() > 0) {
        this->rwStateInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->rwStateInMsgName,
                                                                                  sizeof(RWConfigLogSimMsg),
                                                                                  moduleID);
    } else {
        bskLogger.bskLog(BSK_ERROR, "PowerRW failed to have rwStateInMsgName defined.");
    }

    return;
}

/*! This method is used to reset the module. Here variables are checked for correct values.
 @return void
 */
void ReactionWheelPower::customReset(uint64_t CurrentSimNanos)
{
    if (this->eta_e2m <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "PowerRW: eta_e2m is %f, must a strictly positive value.", this->eta_e2m);
    }
    return;
}

/*! This method is used to read incoming RW state message.
 @return void
 */
bool ReactionWheelPower::customReadMessages()
{
    RWConfigLogSimMsg statusMsg;
    SingleMessageHeader localHeader;

    //! - read in the power node use/supply messages
    bool rwRead = true;
    bool tmpStatusRead = false;
    if(this->rwStateInMsgId >= 0)
    {
        memset(&statusMsg, 0x0, sizeof(DeviceStatusIntMsg));
        tmpStatusRead = SystemMessaging::GetInstance()->ReadMessage(this->rwStateInMsgId, &localHeader,
                                                                    sizeof(RWConfigLogSimMsg),
                                                                    reinterpret_cast<uint8_t*>(&statusMsg),
                                                                    moduleID);

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
void ReactionWheelPower::evaluatePowerModel(PowerNodeUsageSimMsg *powerUsageSimMsg){
    double rwPowerNeed;
    double wheelPower;

    /* add base RW power consumption */
    rwPowerNeed = this->basePowerNeed;

    /* evaluate power required to torque RW */
    wheelPower = this->rwStatus.Omega*this->rwStatus.u_current;
    if (wheelPower > 0.0 ||         /* accelerating the wheel to larger Omega values always takes power */
        this->eta_m2e < 0.0) {      /* if m2e is negative model the breaking as taking power as well */
        rwPowerNeed += fabs(wheelPower)/ this->eta_e2m;
    } else {
        /* breaking the wheeel speed where some mechanics energy is recovered */
        rwPowerNeed += this->eta_m2e * wheelPower;
    }

    /* flip sign as a positive RW power requirement is a negative draw on the power system */
    powerUsageSimMsg->netPower = rwPowerNeed * (-1);

    return;
}
