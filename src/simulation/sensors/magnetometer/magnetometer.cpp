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

#include "sensors/magnetometer/magnetometer.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include <math.h>
#include <iostream>
#include <cstring>
#include <algorithm>
#include "utilities/avsEigenSupport.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/avsEigenMRP.h"

/*! This is the constructor, setting variables to default values. */
Magnetometer::Magnetometer()
{
    this->magIntMsgID = -1;
    this->stateIntMsgID = -1;
    this->stateIntMsgName = "inertial_state_output";
    this->magIntMsgName = "";
    this->tamDataOutMsgName = "";
    this->numStates = 3;
    this->senBias.fill(0.0); // Tesla
    this->senNoiseStd = -1.0; // Tesla
    this->walkBounds.fill(0.0);
    this->noiseModel = GaussMarkov(this->numStates);
    this->noiseModel.setRNGSeed(this->RNGSeed);
    this->scaleFactor = 1.0;
    this->maxOutput = 1e200; // Tesla
    this->minOutput = -1e200; // Tesla
    this->saturateUtility = Saturate(this->numStates);
    this->dcm_SB.setIdentity(3, 3);
    this->outputBufferCount = 2;
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

/*! This method performs all of the internal initialization for the model itself.
 Primarily that involves initializing the random number generator and creates
 the output message. */
void Magnetometer::SelfInit()
{
    //! - Create the output message sized to the output message size
    if (this->tamDataOutMsgName != "") {
        this->tamDataOutMsgID = SystemMessaging::GetInstance()->
            CreateNewMessage(this->tamDataOutMsgName, sizeof(TAMDataSimMsg),
                this->outputBufferCount, "TAMDataSimMsg", this->moduleID);
    }
    else {
        bskPrint.printMessage(MSG_ERROR, "Magnetometer message name (tamDataOutMsgName) is empty.");
    }

    return;
}

/*! This method simply calls the LinkMessages method to ensure that input messages
 are matched correctly.*/
void Magnetometer::CrossInit()
{
    //! - Subscribe to the magnetic field ephemeris message and the vehicle state ephemeris
    if (this->magIntMsgName != "") {
        this->magIntMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->magIntMsgName,
            sizeof(MagneticFieldSimMsg),
            this->moduleID);
    }
    else {
        bskPrint.printMessage(MSG_ERROR, "Magnetic field interface message name (magIntMsgName) is empty.");
    }

    if (this->stateIntMsgName != "") {
        this->stateIntMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->stateIntMsgName,
            sizeof(SCPlusStatesSimMsg),
            this->moduleID);
    } else {
        bskPrint.printMessage(MSG_ERROR, "Spacecraft state message name (stateIntMsgName) is empty.");
    }

    return;
}

/*! This method is used to reset the module.
 @param CurrentSimNanos The current simulation time from the architecture
 @return void */
void Magnetometer::Reset(uint64_t CurrentSimNanos)
{
    this->noiseModel.setUpperBounds(this->walkBounds);
    Eigen::Matrix3d idm3d;
    idm3d.setIdentity(3, 3);
    Eigen::Matrix3d nMatrix;
    nMatrix = this->senNoiseStd * 1.5 * idm3d;
    this->noiseModel.setNoiseMatrix(nMatrix);
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
    SingleMessageHeader localHeader;
    //! - Read magnetic field model ephemeris message
    memset(&this->magData, 0x0, sizeof(MagneticFieldSimMsg));
    SystemMessaging::GetInstance()->ReadMessage(this->magIntMsgID, &localHeader,
        sizeof(MagneticFieldSimMsg),
        reinterpret_cast<uint8_t*> (&this->magData),
        this->moduleID);
    //! - Read vehicle state ephemeris message
    memset(&this->stateCurrent, 0x0, sizeof(SCPlusStatesSimMsg));
    SystemMessaging::GetInstance()->ReadMessage(this->stateIntMsgID, &localHeader,
        sizeof(SCPlusStatesSimMsg),
        reinterpret_cast<uint8_t*> (&this->stateCurrent),
        this->moduleID);
}

/*! This method computes the magnetic field vector information in the sensor frame.*/
void Magnetometer::computeMagData()
{
    Eigen::Vector3d tam_N;
    Eigen::Matrix3d dcm_BN;
    Eigen::MRPd sigma_BN;
    //! - Magnetic field vector in inertial frame using a magnetic field model (WMM, Dipole, etc.)
    tam_N = cArray2EigenVector3d(this->magData.magField_N);
    sigma_BN = cArray2EigenVector3d(this->stateCurrent.sigma_BN);
    //! - Get the inertial to sensor frame transformation information and convert tam_N to tam_S
    dcm_BN = sigma_BN.toRotationMatrix().transpose();
    this->tam_S = this->dcm_SB * dcm_BN * tam_N;
}

/*! This method computes the true sensed values for the sensor. */
void Magnetometer::computeTrueOutput()
{
    this->trueValue = this->tam_S;
}

/*! This method takes the true values (trueValue) and converts
 it over to an errored value.  It applies Gaussian noise, constant bias and scale factor to the truth. */
void Magnetometer::applySensorErrors()
{
    //! - If the standard deviation is not positive, do not use noise error from RNG
    if (this->senNoiseStd <= 0.0) {
        this->sensedValue = this->trueValue;
    }
    else {
        //! - Get current error from random number generator
        this->noiseModel.computeNextState();
        Eigen::Vector3d currentError = this->noiseModel.getCurrentState();
        //! - Sensed value with noise
        this->sensedValue = this->trueValue + currentError;
    }
    //! - Sensed value with bias
    this->sensedValue = this->sensedValue + this->senBias;
    //! - Multiplying the sensed value with a scale factor
    this->sensedValue *= this->scaleFactor;
}

/*! This method applies saturation using the given bounds. */
void Magnetometer::applySaturation()
{
    this->sensedValue = this->saturateUtility.saturate(this->sensedValue);
}

/*! This method writes the output messages. */
void Magnetometer::writeOutputMessages(uint64_t Clock)
{
    TAMDataSimMsg localMessage;
    //! - Zero the output message
    memset(&localMessage, 0x0, sizeof(TAMDataSimMsg));
    eigenVector3d2CArray(this->sensedValue, localMessage.OutputData);
    //! - Write the outgoing message to the architecture
    SystemMessaging::GetInstance()->WriteMessage(tamDataOutMsgID, Clock,
        sizeof(TAMDataSimMsg),
        reinterpret_cast<uint8_t*> (&localMessage),
        this->moduleID);
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
