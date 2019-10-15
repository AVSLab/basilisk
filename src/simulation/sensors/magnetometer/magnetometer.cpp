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

 /*
	 Magnetometer Module

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
#include "utilities/bsk_Print.h"

 /*! This is the constructor, setting variables to default values. */
Magnetometer::Magnetometer()
{
	this->magIntMsgID = -1;
	this->stateIntMsgID = -1;
	this->stateIntMsgName = "inertial_state_output";
	this->magIntMsgName = "Magnetic_Field_data";
	this->tamDataOutMsgName = "";
	this->numStates = 3;
	this->senBias = {0.0, 0.0, 0.0};
	this->senNoiseStd = 0.0;
	this->walkBounds.fill(0.0);
	this->noiseModel = GaussMarkov(this->numStates);
	this->tam_B.fill(0.0);
	this->scaleFactor = 1.0;
	this->maxOutput = 1e200; // Tesla
	this->minOutput = -1e200;
	this->saturateUtility = Saturate(this->numStates);
	this->dcm_SB.setIdentity(3, 3);
	v3SetZero(this->B2S321Angles);
	this->setBodyToSensorDCM(B2S321Angles[0], B2S321Angles[1], B2S321Angles[2]);
	this->outputBufferCount = 2;
	this->idm3d.setIdentity(3, 3);
	return;
}

/*! This is the destructor, nothing to report here. */
Magnetometer::~Magnetometer()
{
	return;
}

//! - This method composes the transformation matrix from Body to Sensor frame.
void Magnetometer::setBodyToSensorDCM(double yaw, double pitch, double roll)
{
	this->dcm_SB = eigenM1(roll) * eigenM2(pitch) * eigenM3(yaw);

	return;
}

/*! This method performs all of the internal initialization for the model itself.
 Primarily that involves initializing the random number generator and creates
 the output message. */
void Magnetometer::SelfInit()
{
	//! - Create the output message sized to the output message size
	this->tamDataOutMsgID = SystemMessaging::GetInstance()->
		CreateNewMessage(this->tamDataOutMsgName, sizeof(TAMDataSimMsg),
			this->outputBufferCount, "TAMDataSimMsg", this->moduleID);

	this->noiseModel.setRNGSeed(this->RNGSeed);
	this->noiseModel.setUpperBounds(this->walkBounds);
	Eigen::Matrix3d nMatrix;
	nMatrix = this->senNoiseStd * 1.5 * this->idm3d;
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
}

/*! This method simply calls the LinkMessages method to ensure that input messages
 are matched correctly.*/
void Magnetometer::CrossInit()
{
	//! - Subscribe to the magnetic field ephemeris message and the vehicle state ephemeris
	this->magIntMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->magIntMsgName,
		sizeof(MagneticFieldSimMsg),
		this->moduleID);
	this->stateIntMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->stateIntMsgName,
		sizeof(SCPlusStatesSimMsg),
		this->moduleID);
	//! - If either messages is not valid, send a warning message
	if (this->magIntMsgID < 0 || this->stateIntMsgID < 0) {
		BSK_PRINT(MSG_WARNING, "Failed to link a magnetometer input message: Magnetometer: %lld", this->magIntMsgID);
	}
	return;
}

/*! This method reads necessary input messages. */
void Magnetometer::readInputMessages()
{
	SingleMessageHeader localHeader;
	//! - Read magnetic field model ephemeris message
	SystemMessaging::GetInstance()->ReadMessage(this->magIntMsgID, &localHeader,
		sizeof(MagneticFieldSimMsg),
		reinterpret_cast<uint8_t*> (&this->magData),
		this->moduleID);
	//! - Read vehicle state ephemeris message
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
	Eigen::MRPd sigma_BN_eigen;
	//! - Magnetic field vector in inertial frame using a magnetic field model (WMM, Dipole, etc.) 
	tam_N = cArray2EigenVector3d(this->magData.magField_N);
	sigma_BN_eigen = cArray2EigenVector3d(this->stateCurrent.sigma_BN);
	//! - Get the inertial to body frame transformation information and convert tam_N to tam_B
	dcm_BN = sigma_BN_eigen.toRotationMatrix().transpose();
	this->tam_B = dcm_BN * tam_N;
	//! - Get the body to sensor frame transformation information and convert tam_B to tam_S
	this->tam_S = this->dcm_SB * this->tam_B;
}

/*! This method computes the true sensed values for the sensor. */
void Magnetometer::computeTrueOutput()
{
	this->trueValue = this->tam_S;
}

/*! This method takes the true values (trueValue) and converts
 it over to an errored value.  It applies noise to the truth. */
void Magnetometer::applySensorErrors()
{
	// If the standard deviation is not positive don't use noise error from RNG
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
}

/*! This method multiplies the sensedValue with a scale factor. */
void Magnetometer::scaleSensorValues()
{
	this->sensedValue *= this->scaleFactor; 
	this->trueValue *= this->scaleFactor;
}

/*! This method applies saturation using the given bounds. */
void Magnetometer::applySaturation()
{

	this->sensedValue = this->saturateUtility.saturate(this->sensedValue);

}

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
 @param CurrentSimNanos The current simulation time from the architecture*/
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
	//! - Scale the data
	this->scaleSensorValues();
	//! - Apply saturation
	this->applySaturation();
	//! - Write output data
	this->writeOutputMessages(CurrentSimNanos);
}

