/*
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "simulation/sensors/magnetometer/magnetometer.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <math.h>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"

/*! This is the constructor, setting variables to default values. */
Magnetometer::Magnetometer()
{
    this->numStates = 3;
    this->senBias.fill(0.0); // Tesla
    this->senNoiseStd.fill(-1.0); // Tesla
    this->walkBounds.fill(0.0);

    // Initialize noise model
    this->noiseModel = GaussMarkov(this->numStates, this->RNGSeed);

    // Initialize noise matrices with defaults
    Eigen::MatrixXd nMatrix;
    nMatrix.resize(3,3);
    nMatrix.setZero();
    this->noiseModel.setNoiseMatrix(nMatrix);

    Eigen::MatrixXd pMatrix;
    pMatrix.setIdentity(3,3);
    this->noiseModel.setPropMatrix(pMatrix);

    this->noiseModel.setUpperBounds(this->walkBounds);

    // Initialize other parameters
    this->scaleFactor = 1.0;
    this->maxOutput = 1e200; // Tesla
    this->minOutput = -1e200; // Tesla
    this->saturateUtility = Saturate(this->numStates);
    this->dcm_SB.setIdentity(3, 3);
    this->AMatrix.setIdentity();
    return;
}

/*! This is the destructor, nothing to report here. */
Magnetometer::~Magnetometer()
{
    return;
}

//! - This method composes the transformation matrix from Body to Sensor frame.
Eigen::Matrix3d Magnetometer::setBodyToSensorDCM(double yaw, double pitch, double roll)
{
    this->dcm_SB = eigenM1(roll) * eigenM2(pitch) * eigenM3(yaw);

    return this->dcm_SB;
}


/*! This method is used to reset the module.
 @param CurrentSimNanos The current simulation time from the architecture
  */
void Magnetometer::Reset(uint64_t CurrentSimNanos)
{
    if (!this->magInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Magnetic field interface message name (magInMsg) is empty.");
    }

    if (!this->stateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Spacecraft state message name (stateInMsg) is empty.");
    }

    // Only apply noise if user has configured it
    if (this->walkBounds.norm() > 0 || this->senNoiseStd.norm() > 0) {
        this->noiseModel.setUpperBounds(this->walkBounds);

        Eigen::MatrixXd nMatrix;
        nMatrix.resize(3,3);
        nMatrix.setZero();
        nMatrix(0,0) = this->senNoiseStd(0);
        nMatrix(1,1) = this->senNoiseStd(1);
        nMatrix(2,2) = this->senNoiseStd(2);
        this->noiseModel.setNoiseMatrix(nMatrix);
    }

    // Set saturation bounds
    Eigen::MatrixXd satBounds;
    satBounds.resize(this->numStates, 2);
    satBounds(0, 0) = this->minOutput;
    satBounds(0, 1) = this->maxOutput;
    satBounds(1, 0) = this->minOutput;
    satBounds(1, 1) = this->maxOutput;
    satBounds(2, 0) = this->minOutput;
    satBounds(2, 1) = this->maxOutput;
    this->saturateUtility.setBounds(satBounds);
    return;
}

/*! This method reads necessary input messages. */
void Magnetometer::readInputMessages()
{
    //! - Read magnetic field model ephemeris message
    this->magData = this->magInMsg();

    //! - Read vehicle state ephemeris message
    this->stateCurrent = this->stateInMsg();
}

/*! This method computes the magnetic field vector information in the sensor frame.*/
void Magnetometer::computeMagData()
{
    Eigen::Vector3d tam_N;
    Eigen::Matrix3d dcm_BN;
    Eigen::MRPd sigma_BN;
    //! - Magnetic field vector in inertial frame using a magnetic field model (WMM, Dipole, etc.)
    tam_N = cArray2EigenVector3d(this->magData.magField_N);
    sigma_BN = cArray2EigenMRPd(this->stateCurrent.sigma_BN);
    //! - Get the inertial to sensor frame transformation information and convert tam_N to tam_S
    dcm_BN = sigma_BN.toRotationMatrix().transpose();
    this->tam_S = this->dcm_SB * dcm_BN * tam_N;
}

/*! This method computes the true sensed values for the sensor. */
void Magnetometer::computeTrueOutput()
{
    this->tamTrue_S = this->tam_S;
}

/*! This method takes the true values (tamTrue_S) and converts
 it over to an errored value.  It applies Gaussian noise, constant bias and scale factor to the truth. */
void Magnetometer::applySensorErrors()
{
    //! - If any of the standard deviation vector elements is not positive, do not use noise error from RNG.
    bool anyNoiseComponentUninitialized = false;
    for (unsigned i = 0; i < this->senNoiseStd.size(); i++) {
        if ((this->senNoiseStd(i) <= 0.0)) {
            anyNoiseComponentUninitialized = true;
        }
    }
    if (anyNoiseComponentUninitialized) {
        this->tamSensed_S = this->tamTrue_S;
    } else {
        //! - Get current error from random number generator
        this->noiseModel.computeNextState();
        Eigen::Vector3d currentError = this->noiseModel.getCurrentState();
        //! - Sensed value with noise
        this->tamSensed_S = this->tamTrue_S + currentError;
    }
    //! - Sensed value with bias
    this->tamSensed_S = this->tamSensed_S + this->senBias;
    //! - Multiplying the sensed value with a scale factor
    this->tamSensed_S *= this->scaleFactor;
}

/*! This method applies saturation using the given bounds. */
void Magnetometer::applySaturation()
{
    this->tamSensed_S = this->saturateUtility.saturate(this->tamSensed_S);
}

/*! This method writes the output messages. */
void Magnetometer::writeOutputMessages(uint64_t Clock)
{
    TAMSensorMsgPayload localMessage;
    //! - Zero the output message
    localMessage = this->tamDataOutMsg.zeroMsgPayload;
    eigenVector3d2CArray(this->tamSensed_S, localMessage.tam_S);
    //! - Write the outgoing message to the architecture
    this->tamDataOutMsg.write(&localMessage, this->moduleID, Clock);
}

/*! This method is called at a specified rate by the architecture.  It makes the
 calls to compute the current magnetic field information and write the output message for
 the rest of the model.
 @param CurrentSimNanos The current simulation time from the architecture */
void Magnetometer::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the inputs
    this->readInputMessages();
    //! - Get magnetic field vector
    this->computeMagData();
    //! - Compute true output
    this->computeTrueOutput();
    //! - Apply any set errors
    this->applySensorErrors();
    //! - Apply saturation
    this->applySaturation();
    //! - Write output data
    this->writeOutputMessages(CurrentSimNanos);
}

void Magnetometer::setAMatrix(const Eigen::Matrix3d& matrix)
{
    this->AMatrix = matrix;
    this->noiseModel.setPropMatrix(matrix);
}

Eigen::Matrix3d Magnetometer::getAMatrix() const
{
    return this->AMatrix;
}
