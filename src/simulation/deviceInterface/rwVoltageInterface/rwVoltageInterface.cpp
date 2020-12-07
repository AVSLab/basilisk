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
#include "deviceInterface/rwVoltageInterface/rwVoltageInterface.h"

#include <iostream>
#include <cstring>

/*! This is the constructor for the RW voltgage interface.  It sets default variable
    values and initializes the various parts of the model */
RWVoltageInterface::RWVoltageInterface()
{
    this->prevTime = 0;
    this->bias.resize(MAX_EFF_CNT);
    this->bias.fill(0.0);
    this->scaleFactor.resize(MAX_EFF_CNT);
    this->scaleFactor.fill(1.0);
    this->voltage2TorqueGain.resize(MAX_EFF_CNT);
    this->voltage2TorqueGain.fill(1.0);
    return;
}

/*! Destructor.  Nothing here. */
RWVoltageInterface::~RWVoltageInterface()
{
    return;
}


/*! This method reads the RW voltage input messages
 */
void RWVoltageInterface::readInputMessages()
{
    if(!this->rwVoltageInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_WARNING, "rwVoltageInMsg is not linked.");
        return;
    }

    // read the incoming array of voltages
    this->inputVoltageBuffer = this->rwVoltageInMsg();
    
    return;
}

/*! This method evaluates the RW Motor torque output states.
 @return void
 */
void RWVoltageInterface::computeRWMotorTorque()
{
    this->outputRWTorqueBuffer = this->rwMotorTorqueOutMsg.zeroMsgPayload();
    for (uint64_t i=0; i < MAX_EFF_CNT; i++) {
        this->outputRWTorqueBuffer.motorTorque[i] = this->inputVoltageBuffer.voltage[i] * this->voltage2TorqueGain(i) * this->scaleFactor(i) + this->bias(i);
    }
    return;
}

/*! This method sets (per motor) voltage to torque scale factors (linear proportional error)
 @return void
 */
void RWVoltageInterface::setScaleFactors(Eigen::VectorXd scaleFactors){
    for (int i = 0; i < this->scaleFactor.rows(); i++)
    {
        if (i < scaleFactors.rows()){
            this->scaleFactor(i) = scaleFactors(i);
        } else {
            this->scaleFactor(i) = 1.0;
        }
    }
    return;
}

/*! This method sets the list of motor voltage to torque gains.
 @return void
 */
void RWVoltageInterface::setGains(Eigen::VectorXd gains)
{
    for (int i = 0; i < this->voltage2TorqueGain.rows(); i++)
    {
        if (i < gains.rows()){
            this->voltage2TorqueGain(i) = gains(i);
        } else {
            this->voltage2TorqueGain(i) = 1.0;
        }
    }
    return;
}

/*! This method sets the list of voltage to torque biases (per rw)
 @return void
 */
void RWVoltageInterface::setBiases(Eigen::VectorXd biases)
{
    for (int i = 0; i < this->bias.rows(); i++)
    {
        if (i < biases.rows()){
            this->bias(i) = biases(i);
        } else {
            this->bias(i) = 0.0;
        }
    }
    return;
}

/*! This method writes the RW Motor torque output state message.
 @return void
 @param CurrentClock The clock time associated with the model call
 */
void RWVoltageInterface::writeOutputMessages(uint64_t CurrentClock)
{
    this->rwMotorTorqueOutMsg.write(&this->outputRWTorqueBuffer, this->moduleID, CurrentClock);

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


