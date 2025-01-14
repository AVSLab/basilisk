/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "simulation/sensors/simpleVoltEstimator/simpleVoltEstimator.h"
#include <iostream>
#include <cstring>
#include "architecture/utilities/avsEigenSupport.h"

/*! This is the constructor for the simple voltage estimator module.  It sets default variable
    values and initializes the various parts of the model */
SimpleVoltEstimator::SimpleVoltEstimator()
{
    this->estVoltState = this->voltOutMsg.zeroMsgPayload;
    this->trueVoltState = this->voltOutMsg.zeroMsgPayload;

    // Initialize matrices with proper sizes and default values
    this->PMatrix.resize(1,1);
    this->PMatrix.fill(0.0);
    this->walkBounds.resize(1);
    this->walkBounds.fill(0.0);
    this->AMatrix.resize(1,1);
    this->AMatrix.setIdentity(1,1);

    // Initialize noise model and set default parameters
    this->errorModel = GaussMarkov(1, this->RNGSeed);
    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setUpperBounds(this->walkBounds);
    this->errorModel.setPropMatrix(this->AMatrix);
}

/*! Destructor.  Nothing here. */
SimpleVoltEstimator::~SimpleVoltEstimator()
{
    return;
}


/*! This method is used to reset the module. It
 initializes the various containers used in the model as well as creates the
 output message.  The error states are allocated as follows:
 Total states: 1
     - Voltage error [0]

 */
void SimpleVoltEstimator::Reset(uint64_t CurrentSimNanos)
{
    // check if input message has not been included
    if (!this->voltInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SimpleVoltEstimator.voltInMsg was not linked.");
    }

    int64_t numStates = 1;

    //! - Alert the user and stop if the noise matrix is the wrong size.  That'd be bad.
    if (this->PMatrix.size() != numStates*numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your process noise matrix (PMatrix) is not %ld*%ld. Size is %ld.  Quitting", numStates, numStates, this->PMatrix.size());
        return;
    }
    if (this->walkBounds.size() != numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your walkbounds vector  is not %ld elements. Quitting", numStates);
    }

    //! - Update the noise model parameters
    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setRNGSeed(this->RNGSeed);
    this->errorModel.setUpperBounds(this->walkBounds);
}


/*! This method reads the input message associated with the spacecraft voltage
 */
void SimpleVoltEstimator::readInputMessages()
{
    this->trueVoltState = this->voltInMsg();
}

/*! This method writes the voltage information into the output state message.

 @param Clock The clock time associated with the model call
 */
void SimpleVoltEstimator::writeOutputMessages(uint64_t Clock)
{
    this->voltOutMsg.write(&this->estVoltState, this->moduleID, Clock);
}

void SimpleVoltEstimator::applyErrors()
{
    //! - Add errors
    this->estVoltState.voltage = this->trueVoltState.voltage + this->voltErrors.data()[0];
}

/*! This method sets the propagation matrix and requests new random errors from
 its GaussMarkov model.

 */
void SimpleVoltEstimator::computeErrors()
{
    Eigen::MatrixXd localProp = this->AMatrix;

    //! - Set the GaussMarkov propagation matrix and compute errors
    this->errorModel.setPropMatrix(localProp);
    this->errorModel.computeNextState();
    this->voltErrors = this->errorModel.getCurrentState();
}

/*! This method calls all of the run-time operations for the simpleVoltEstimator module.

    @param CurrentSimNanos The clock time associated with the model call
*/
void SimpleVoltEstimator::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->computeErrors();
    this->applyErrors();
    this->writeOutputMessages(CurrentSimNanos);
}

/*!
    Setter for `AMatrix` used for error propagation
    @param propMatrix Matrix to set
*/
void SimpleVoltEstimator::setAMatrix(const Eigen::MatrixXd& propMatrix)
{
    if(propMatrix.rows() != 1 || propMatrix.cols() != 1) {
        bskLogger.bskLog(BSK_ERROR, "SimpleVoltEstimator: Propagation matrix must be 1x1");
        return;
    }
    this->AMatrix = propMatrix;
    this->errorModel.setPropMatrix(propMatrix);
}

/*!
    Getter for `AMatrix` used for error propagation
    @return Current matrix
*/
Eigen::MatrixXd SimpleVoltEstimator::getAMatrix() const
{
    return this->AMatrix;
}
