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
#include "simulation/sensors/starTracker/starTracker.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <iostream>
#include "architecture/utilities/gauss_markov.h"

StarTracker::StarTracker()
{
    this->sensorTimeTag = 0;
    m33SetIdentity(RECAST3X3 this->dcm_CB);

    // Initialize noise model
    this->errorModel = GaussMarkov(3, this->RNGSeed);

    // Initialize matrices
    this->PMatrix.resize(3, 3);
    this->AMatrix.resize(3, 3);
    this->walkBounds.resize(3);

    this->PMatrix.fill(0.0);
    this->AMatrix.setIdentity(3, 3);
    this->walkBounds.fill(0.0);
    return;
}

StarTracker::~StarTracker()
{
    return;
}


/*! This method is used to reset the module.
 @param CurrentSimNanos The current simulation time from the architecture
  */
void StarTracker::Reset(uint64_t CurrentSimNanos)
{
    // check if input message has not been included
    if (!this->scStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "starTracker.scStateInMsg was not linked.");
    }

    //! - Alert the user if the noise matrix was not the right size.  That'd be bad.
    if(this->PMatrix.size() != 9)
    {
        bskLogger.bskLog(BSK_ERROR, "Your process noise matrix (PMatrix) is not 3*3. Quitting.");
        return;
    }
    if(this->walkBounds.size() != 3){
        bskLogger.bskLog(BSK_ERROR, "Your walkbounds is not size 3. Quitting");
        return;
    }

    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setRNGSeed(this->RNGSeed);
    this->errorModel.setUpperBounds(this->walkBounds);
}

/*!
    read input messages
 */
void StarTracker::readInputMessages()
{
    this->scState = this->scStateInMsg();
    this->sensorTimeTag = this->scStateInMsg.timeWritten();
}

/*!
   compute sensor errors
 */
void StarTracker::computeSensorErrors()
{
    this->errorModel.setPropMatrix(this->AMatrix);
    this->errorModel.computeNextState();
    this->navErrors = this->errorModel.getCurrentState();
}

/*!
   apply sensor errors
 */
void StarTracker::applySensorErrors()
{
    double sigmaSensed[3];
    PRV2MRP(&(this->navErrors.data()[0]), this->mrpErrors);
    addMRP(this->scState.sigma_BN, this->mrpErrors, sigmaSensed);
    this->computeQuaternion(sigmaSensed, &this->sensedValues);
    this->sensedValues.timeTag = this->sensorTimeTag;
}

/*!
    compute quaternion from MRPs
    @param sigma
    @param sensorValues
 */
void StarTracker::computeQuaternion(double *sigma, STSensorMsgPayload *sensorValues)
{
    double dcm_BN[3][3];            /* dcm, inertial to body frame */
    double dcm_CN[3][3];            /* dcm, inertial to case frame */
    MRP2C(sigma, dcm_BN);
    m33MultM33(RECAST3X3 this->dcm_CB, dcm_BN, dcm_CN);
    C2EP(dcm_CN, sensorValues->qInrtl2Case);
}

/*!
    compute true output values
 */
void StarTracker::computeTrueOutput()
{
    this->trueValues.timeTag = this->sensorTimeTag;
    this->computeQuaternion(this->scState.sigma_BN, &this->trueValues);
}

/*!
    write output messages
 */
void StarTracker::writeOutputMessages(uint64_t CurrentSimNanos)
{
    this->sensorOutMsg.write(&this->sensedValues, this->moduleID, CurrentSimNanos);
}

/*!
    update module states
 */
void StarTracker::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->computeSensorErrors();
    this->computeTrueOutput();
    this->applySensorErrors();
    this->writeOutputMessages(CurrentSimNanos);
}

/*!
    Setter for `AMatrix` used for error propagation
    @param propMatrix Matrix to set
*/
void StarTracker::setAMatrix(const Eigen::MatrixXd& propMatrix)
{
    if(propMatrix.rows() != 3 || propMatrix.cols() != 3) {
        bskLogger.bskLog(BSK_ERROR, "StarTracker: Propagation matrix must be 3x3");
        return;
    }
    this->AMatrix = propMatrix;
    this->errorModel.setPropMatrix(propMatrix);
}

/*!
    Getter for `AMatrix` used for error propagation
    @return Current matrix
*/
Eigen::MatrixXd StarTracker::getAMatrix() const
{
    return this->AMatrix;
}

void StarTracker::setWalkBounds(const Eigen::Vector3d& bounds)
{
    this->walkBounds = bounds;
    this->errorModel.setUpperBounds(bounds);
}

Eigen::Vector3d StarTracker::getWalkBounds() const
{
    return this->walkBounds;
}
