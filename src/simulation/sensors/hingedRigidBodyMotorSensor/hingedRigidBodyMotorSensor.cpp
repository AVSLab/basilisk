/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/sensors/hingedRigidBodyMotorSensor/hingedRigidBodyMotorSensor.h"
#include <cmath>
#include <stdint.h>
#include <random>
/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
HingedRigidBodyMotorSensor::HingedRigidBodyMotorSensor()
{


    this->rGen.seed((unsigned int)this->RNGSeed); //! RNGSeed is an attribute of all modules

    this->thetaNoiseStd = 0.0;
    this->thetaDotNoiseStd = 0.0;

    this->thetaBias = 0.0;
    this->thetaDotBias = 0.0;

    this->thetaLSB = -1.0; //! -1 for no discretization by default
    this->thetaDotLSB = -1.0; //! -1 for no discretization by default

}

/*! Module Destructor */
HingedRigidBodyMotorSensor::~HingedRigidBodyMotorSensor()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.

*/
void HingedRigidBodyMotorSensor::Reset(uint64_t CurrentSimNanos)
{
    //!< check that required input messages are connected
    if (!this->hingedRigidBodyMotorSensorInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "HingedRigidBodyMotorSensor.hingedRigidBodyMotorSensorInMsg was not linked.");
    }

}

/*! This allows the RNGSeed to be changed.

*/
void HingedRigidBodyMotorSensor::setRNGSeed(unsigned int newSeed)
{
    this->rGen.seed((unsigned int)newSeed);
}

/*! This is the main method that gets called every time the module is updated.  Adds Gaussian noise and bias and diescretizes output.

*/
void HingedRigidBodyMotorSensor::UpdateState(uint64_t CurrentSimNanos)
{
    //! variables for  calculations
    double trueTheta;               //! [rad] actual planel angle
    double trueThetaDot;            //! [rad/s] actual panel angular rate
    double thetaNoise;              //! [rad] gaussian noise for panel angle
    double thetaDotNoise;           //! [rad/s] gaussian noise for panel angular rate
    double sensedTheta;             //! [rad] the sensed output angle
    double sensedThetaDot;          //! [rad/s] the sended output angle rate
    double numLSB;                  //! [] number of times the discretized theta value fits int to sensed value
    double workingTheta;            //! [rad] discretized panel angle
    double workingThetaDot;         //! [rad/s] discretized panel angle rate
    double remainder;               //! [] remainder between sensed state and discretized state

    HingedRigidBodyMsgPayload hingedRigidBodyMotorSensorInMsgBuffer;  //! local copy of message buffer
    HingedRigidBodyMsgPayload hingedRigidBodyMotorSensorOutMsgBuffer;  //! local copy of message buffer

    //! zero the output message buffers before assigning values
    hingedRigidBodyMotorSensorOutMsgBuffer = this->hingedRigidBodyMotorSensorOutMsg.zeroMsgPayload;

    //! read in the input messages
    hingedRigidBodyMotorSensorInMsgBuffer = this->hingedRigidBodyMotorSensorInMsg();

    trueTheta = hingedRigidBodyMotorSensorInMsgBuffer.theta;
    trueThetaDot = hingedRigidBodyMotorSensorInMsgBuffer.thetaDot;

    //! apply sensor noise and bias
    std::normal_distribution<double>::param_type updateThetaPair(0.0, this->thetaNoiseStd);
    std::normal_distribution<double>::param_type updateThetaDotPair(0.0, this->thetaDotNoiseStd);

    this->rNum.param(updateThetaPair);
    thetaNoise = this->rNum(this->rGen); //! sample using thetaNoiseStd
    this->rNum.param(updateThetaDotPair);
    thetaDotNoise = this->rNum(this->rGen); //! sample using thetaDotNoiseStd

    sensedTheta =  trueTheta + thetaNoise + this->thetaBias;
    sensedThetaDot = trueThetaDot + thetaDotNoise + this->thetaDotBias;

    //!< apply discretization (rounds to nearest multiple of LSB)
    if(this->thetaLSB > 0.0)
    {
        numLSB = floor(abs(sensedTheta) / this->thetaLSB); //! number of times the LSB goes into the absolute value of the sensed continuous theta
        workingTheta = numLSB * this->thetaLSB * copysign(1.0,sensedTheta); //! multiply back the signed value of theta times the number of LSBs
        remainder = sensedTheta-workingTheta;
        if(abs(remainder) > (this->thetaLSB/2.0)) { //! add an extra LSB if needed to round up/down
            workingTheta += this->thetaLSB * copysign(1.0,sensedTheta);
        }
        sensedTheta = workingTheta;

    }
    if(this->thetaDotLSB > 0.0)
    {
        numLSB = floor(abs(sensedThetaDot) / this->thetaDotLSB);
        workingThetaDot = numLSB * this->thetaDotLSB * copysign(1.0,sensedThetaDot);
        remainder = sensedThetaDot-workingThetaDot;
        if(abs(remainder) > (this->thetaDotLSB/2.0)) {
            workingThetaDot += this->thetaDotLSB * copysign(1.0,sensedThetaDot);
        }
        sensedThetaDot = workingThetaDot;

    }

    //! write to the output messages
    hingedRigidBodyMotorSensorOutMsgBuffer.theta = sensedTheta;
    hingedRigidBodyMotorSensorOutMsgBuffer.thetaDot = sensedThetaDot;
    this->hingedRigidBodyMotorSensorOutMsg.write(&hingedRigidBodyMotorSensorOutMsgBuffer, this->moduleID, CurrentSimNanos);
}
