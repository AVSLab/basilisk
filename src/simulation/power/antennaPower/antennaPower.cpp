/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU, Norway

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

#include "antennaPower.h"

/*! Constructor
*/
AntennaPower::AntennaPower(){
    this->basePowerNeed        = 0.0; //!< [W] base electrical power required to operate an antenna
}

AntennaPower::~AntennaPower(){
}

/*! This method is used to reset the module. Here variables are checked for correct values.
 */
void AntennaPower::customReset(uint64_t CurrentSimNanos)
{
    // Check that basePowerNeed is non-negative
    if (this->basePowerNeed < 0.0) {
        bskLogger.bskLog(BSK_ERROR, "AntennaPower: basePowerNeed cannot be negative.");
    }
}

/*! This method is used to read incoming antenna state message.
 */
bool AntennaPower::customReadMessages()
{
    //! - read in the power node use/supply messages
    bool antennaRead    = true;
    bool tempStatusRead = false;
    if(this->antennaSetStateInMsg.isLinked())
    {
        this->antennaStatusMsgBuffer = this->antennaSetStateInMsg();
        tempStatusRead               = this->antennaSetStateInMsg.isWritten();
        antennaRead                  = antennaRead && tempStatusRead;
    }

    if (!antennaRead){
        bskLogger.bskLog(BSK_WARNING, "PowerAntenna unable to read antenna status message.");
    }

    return(antennaRead);
}

/*! Computes the antenna power load. Determines the netPower attribute in powerUsageSimMessage.
*/
void AntennaPower::evaluatePowerModel(PowerNodeUsageMsgPayload *powerUsage){
    // Read the input message to get the current antenna state
    if (!this->customReadMessages()) {
        bskLogger.bskLog(BSK_WARNING, "AntennaPower: Failed to read antenna state message. Setting power usage to base power need.");
        powerUsage->netPower = -this->basePowerNeed;
        return;
    }
    switch (static_cast<AntennaTypes::AntennaStateEnum>(this->antennaStatusMsgBuffer.antennaState)) {
        case AntennaTypes::AntennaStateEnum::ANTENNA_OFF:
            powerUsage->netPower = -this->basePowerNeed;
            break;
        case AntennaTypes::AntennaStateEnum::ANTENNA_RX:
            powerUsage->netPower = -this->antennaStatusMsgBuffer.P_Rx - this->basePowerNeed;
            break;
        case AntennaTypes::AntennaStateEnum::ANTENNA_TX:
            powerUsage->netPower = -this->antennaStatusMsgBuffer.P_Tx - this->basePowerNeed;
            break;
        case AntennaTypes::AntennaStateEnum::ANTENNA_RXTX:
            powerUsage->netPower = -this->antennaStatusMsgBuffer.P_Rx - this->antennaStatusMsgBuffer.P_Tx - this->basePowerNeed;
            break;
        default:
            bskLogger.bskLog(BSK_ERROR, "AntennaPower: Invalid antenna state.");
            powerUsage->netPower = -this->basePowerNeed;
            break;
    }
}
