/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "tempMeasurement.h"
#include <iostream>


/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model.
    Don't allow random walk by default.
 */
TempMeasurement::TempMeasurement() : faultState{TEMP_FAULT_NOMINAL},
                                    walkBounds{1E-15},
                                    spikeProbability{0.1},
                                    spikeAmount{2.0}
{
    this->noiseModel = GaussMarkov(1, this->RNGSeed);
}

TempMeasurement::~TempMeasurement() = default;

/*! This method is used to reset the module and checks that required input messages are connected.

*/
void TempMeasurement::Reset(uint64_t CurrentSimNanos)
{
    if (!this->tempInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "TempMeasurement.tempInMsg was not linked.");
    }

    if (this->spikeProbability > 1.0 || this->spikeProbability < 0.0) {
        bskLogger.bskLog(BSK_ERROR, "The probability of temperature spike on fault must be between 0 and 1.");
    }
    // set up gaussMarkov and random number generator parameters
    this->spikeProbabilityGenerator.seed(this->RNGSeed);
    this->noiseModel.setRNGSeed(this->RNGSeed);

    Eigen::VectorXd nMatrix(1,1);
    nMatrix(0,0) = this->senNoiseStd;
    this->noiseModel.setNoiseMatrix(nMatrix);

    Eigen::VectorXd pMatrix(1,1);
    pMatrix(0,0) = 1.;
    this->noiseModel.setPropMatrix(pMatrix);

    Eigen::VectorXd bounds(1,1);
    bounds(0,0) = this->walkBounds;
    this->noiseModel.setUpperBounds(bounds);
}

/*! This method adds noise, bias, and fault behaviors to the read-in temperature message.

*/
void TempMeasurement::applySensorErrors()
{
    // apply noise and bias
    double sensorError;
    if(this->senNoiseStd <= 0.0){
        sensorError = this->senBias;
    } else {
        // get current error from gaussMarkov random number generator
        this->noiseModel.computeNextState();
        Eigen::VectorXd currentErrorEigen = this->noiseModel.getCurrentState();
        double sensorNoise = currentErrorEigen(0,0);
        sensorError = this->senBias + sensorNoise;
    }
    this->sensedTemperature = this->trueTemperature + sensorError;

    // apply fault conditions
    if(this->faultState == TEMP_FAULT_STUCK_VALUE){ // stuck at specified value
        this->sensedTemperature = this->stuckValue;
    } else if (this->faultState == TEMP_FAULT_STUCK_CURRENT){ // stuck at last value before flag turned on
        this->sensedTemperature = this->pastValue;
    } else if (this->faultState == TEMP_FAULT_SPIKING){ // spiking periodically with specified probability
        // have to make a new distribution every time because SWIG can't parse putting this in the H file....?
        std::uniform_real_distribution<double> spikeProbabilityDistribution(0.0,1.0);
        double n = spikeProbabilityDistribution(this->spikeProbabilityGenerator); // draw from uniform distribution
        if (n <= this->spikeProbability) { // if drawn number within probability of spiking
            this->sensedTemperature = this->sensedTemperature*this->spikeAmount;
        }
    }

    this->pastValue = this->sensedTemperature; // update past value
}

/*! This is the main method that gets called every time the module is updated.

*/
void TempMeasurement::UpdateState(uint64_t CurrentSimNanos)
{
    TemperatureMsgPayload tempInMsgBuffer;  //!< local copy of message buffer
    TemperatureMsgPayload tempOutMsgBuffer;  //!< local copy of message buffer

    // always zero the output message buffers before assigning values
    tempOutMsgBuffer = this->tempOutMsg.zeroMsgPayload;

    // read in the input messages
    tempInMsgBuffer = this->tempInMsg();
    this->trueTemperature = tempInMsgBuffer.temperature;

    // apply sensor errors
    this->applySensorErrors();
    tempOutMsgBuffer.temperature = this->sensedTemperature;

    // write to the output messages
    this->tempOutMsg.write(&tempOutMsgBuffer, this->moduleID, CurrentSimNanos);
}
