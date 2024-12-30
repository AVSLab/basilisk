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
#include "simulation/deviceInterface/motorVoltageInterface/motorVoltageInterface.h"

#include <iostream>

/*! This is the constructor for the motor voltgage interface.  It sets default variable
    values and initializes the various parts of the model */
MotorVoltageInterface::MotorVoltageInterface()
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
MotorVoltageInterface::~MotorVoltageInterface()
{
    return;
}

/*! Reset the module to original configuration values.

 */
void MotorVoltageInterface::Reset(uint64_t CurrenSimNanos)
{
    if(!this->motorVoltageInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_WARNING, "motorVoltageInterface.motorVoltageInMsg is not linked.");
        return;
    }
}

/*! This method reads the motor voltage input messages
 */
void MotorVoltageInterface::readInputMessages()
{

    // read the incoming array of voltages
    this->inputVoltageBuffer = this->motorVoltageInMsg();

    return;
}

/*! This method evaluates the motor torque output states.

 */
void MotorVoltageInterface::computeMotorTorque()
{
    this->outputTorqueBuffer = this->motorTorqueOutMsg.zeroMsgPayload;
    for (uint64_t i=0; i < MAX_EFF_CNT; i++) {
        this->outputTorqueBuffer.motorTorque[i] = this->inputVoltageBuffer.voltage[i] * this->voltage2TorqueGain(i) * this->scaleFactor(i) + this->bias(i);
    }
    return;
}

/*! This method sets (per motor) voltage to torque scale factors (linear proportional error)

 */
void MotorVoltageInterface::setScaleFactors(Eigen::VectorXd scaleFactors){
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

 */
void MotorVoltageInterface::setGains(Eigen::VectorXd gains)
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

 */
void MotorVoltageInterface::setBiases(Eigen::VectorXd biases)
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

/*! This method writes the Motor torque output state message.

 @param CurrentClock The clock time associated with the model call
 */
void MotorVoltageInterface::writeOutputMessages(uint64_t CurrentClock)
{
    this->motorTorqueOutMsg.write(&this->outputTorqueBuffer, this->moduleID, CurrentClock);

    return;
}

/*! This method calls all of the run-time operations for the motor voltage interface module.

    @param CurrentSimNanos The clock time associated with the model call
*/
void MotorVoltageInterface::UpdateState(uint64_t CurrentSimNanos)
{
    readInputMessages();
    computeMotorTorque();
    writeOutputMessages(CurrentSimNanos);

    this->prevTime = CurrentSimNanos;

    return;
}
