/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "deviceInterface/rwVoltageInterface/rwVoltageInterface.h"
#include "architecture/messaging/system_messaging.h"
#include <iostream>
#include <cstring>

/*! This is the constructor for the RW voltgage interface.  It sets default variable
    values and initializes the various parts of the model */
RWVoltageInterface::RWVoltageInterface()
{
    this->rwVoltageInMsgName = "rw_voltage_input";
    this->rwMotorTorqueOutMsgName = "reactionwheel_cmds";
    this->outputBufferCount = 2;
    this->rwVoltageInMsgID = -1;
    this->rwMotorTorqueOutMsgID = -1;
    this->prevTime = 0;
    return;
}

/*! Destructor.  Nothing here. */
RWVoltageInterface::~RWVoltageInterface()
{
    return;
}

/*! This is the self-init routine for the RW voltage interface module.  It
    creates the desired RW motor torque output message.
    @return void
*/
void RWVoltageInterface::SelfInit()
{
    this->rwMotorTorqueOutMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage(this->rwMotorTorqueOutMsgName,
                         sizeof(RWArrayTorqueMessage),
                         outputBufferCount,
                         "RWArrayTorqueMessage",
                         moduleID);
    return;
}

/*! This method pulls the input message IDs from the messaging system.  It will
    alert the user if either of them are not found in the messaging database
    @return void
*/
void RWVoltageInterface::CrossInit()
{
    //! Begin method steps
    //! - Obtain the ID associated with the input state name and alert if not found.
    this->rwVoltageInMsgID = SystemMessaging::GetInstance()->
        subscribeToMessage(this->rwVoltageInMsgName,
                           sizeof(RWArrayVoltageMessage),
                           moduleID);
    if(this->rwVoltageInMsgID < 0)
    {
        std::cerr << "WARNING: RWVoltageInterface() did not find a valid message with name: ";
        std::cerr << this->rwVoltageInMsgName << "  :" << std::endl<< __FILE__ << std::endl;
    }
    return;
}

/*! This method reads the RW voltage input messages
 */
void RWVoltageInterface::readInputMessages()
{
    if(this->rwVoltageInMsgID < 0)
    {
        std::cerr << "WARNING: rwVoltageInMsgName message ID not set." << std::endl;
        return;
    }

    //! - Zero the input buffer and read the incoming array of voltages
    SingleMessageHeader LocalHeader;
    memset(&(this->inputVoltageBuffer), 0x0, sizeof(RWArrayVoltageMessage));
    SystemMessaging::GetInstance()->ReadMessage(this->rwVoltageInMsgID, &LocalHeader,
                                                sizeof(RWArrayVoltageMessage),
                                                reinterpret_cast<uint8_t*> (&(this->inputVoltageBuffer)),
                                                moduleID);

    return;
}

/*! This method evaluates the RW Motor torque output states.
 @return void
 */
void RWVoltageInterface::computeRWMotorTorque()
{
    int i;
    memset(&(this->rwTorque), 0x0, sizeof(RWArrayTorqueMessage));
    for (i=0;i<MAX_EFF_CNT;i++) {
        this->rwTorque[i] = this->inputVoltageBuffer.voltage[i] * this->voltage2TorqueGain;
    }

    return;
}


/*! This method writes the RW Motor torque output state message.
 @return void
 @param Clock The clock time associated with the model call
 */
void RWVoltageInterface::writeOutputMessages(uint64_t CurrentClock)
{
    int i;

    for (i=0;i<MAX_EFF_CNT;i++) {
        this->outputRWTorqueBuffer.motorTorque[i] = this->rwTorque[i];
    }
    SystemMessaging::GetInstance()->WriteMessage(this->rwMotorTorqueOutMsgID, CurrentClock,
                                                 sizeof(RWArrayTorqueMessage),
                                                 reinterpret_cast<uint8_t*> (&this->outputRWTorqueBuffer),
                                                 moduleID);
    return;
}


/*! This method calls all of the run-time operations for the RW voltage interface module.
    @return void
    @param CurrentSimNanos The clock time associated with the model call
*/
void RWVoltageInterface::UpdateState(uint64_t CurrentSimNanos)
{
    readInputMessages();
    computeRWMotorTorque();
    writeOutputMessages(CurrentSimNanos);

    this->prevTime = CurrentSimNanos;

    return;
}
