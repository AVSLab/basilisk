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
#include "sensors/imu/imu_sensor.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/linearAlgebra.h"
#include <math.h>
#include <iostream>
#include <cstring>
#include <random>
#include "utilities/gauss_markov.h"

ImuSensor::ImuSensor()
{
    CallCounts = 0;
    this->InputStateID = -1;
    this->InputStateMsg = "inertial_state_output";
    this->OutputDataMsg = "imu_meas_data";
    this->InputMassMsg = "spacecraft_mass_props";
    this->setStructureToPlatformDCM(0.0, 0.0, 0.0);
    this->OutputBufferCount = 2;
    memset(&this->StatePrevious, 0x0, sizeof(OutputStateData));
    memset(&this->StateCurrent, 0x0, sizeof(OutputStateData));
    this->PreviousTime = 0;
    this->NominalReady = false;
    memset(&this->senRotBias[0], 0x0, 3*sizeof(double));
    memset(&this->senTransBias[0], 0x0, 3*sizeof(double));

    return;
}

void ImuSensor::setStructureToPlatformDCM(double yaw, double pitch, double roll)
{
    double q[3] = {yaw, pitch, roll};
    Euler3212C(q, this->Str2Platform);
}

ImuSensor::~ImuSensor()
{
    return;
}

void ImuSensor::SelfInit()
{

    OutputDataID = SystemMessaging::GetInstance()->
        CreateNewMessage( OutputDataMsg, sizeof(ImuSensorOutput),
        OutputBufferCount, "ImuSensorOutput", moduleID);

	uint64_t numStates = 3;

	AMatrixAccel.clear();
	AMatrixAccel.insert(AMatrixAccel.begin(), numStates*numStates, 0.0);
	mSetIdentity(AMatrixAccel.data(), numStates, numStates);
	for(uint32_t i=0; i<3; i++)
	{
		AMatrixAccel.data()[i * 3 + i] = 1.0;
	}
	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(PMatrixAccel.size() != numStates*numStates)
	{
		std::cerr << __FILE__ <<": Your process noise matrix (PMatrix) is not 3*3.";
		std::cerr << "  You should fix that.  Popping zeros onto end"<<std::endl;
		PMatrixAccel.insert(PMatrixAccel.begin()+PMatrixAccel.size(), numStates*numStates - PMatrixAccel.size(),
					   0.0);
	}
	errorModelAccel.setNoiseMatrix(PMatrixAccel);
	errorModelAccel.setRNGSeed(RNGSeed);
	errorModelAccel.setUpperBounds(walkBoundsAccel);

	AMatrixGyro.clear();
	AMatrixGyro.insert(AMatrixGyro.begin(), numStates*numStates, 0.0);
	mSetIdentity(AMatrixGyro.data(), numStates, numStates);
	for(uint32_t i=0; i<3; i++)
	{
		AMatrixGyro.data()[i * 3 + i] = 1.0;
	}
	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(PMatrixGyro.size() != numStates*numStates)
	{
		std::cerr << __FILE__ <<": Your process noise matrix (PMatrix) is not 3*3.";
		std::cerr << "  You should fix that.  Popping zeros onto end"<<std::endl;
		PMatrixGyro.insert(PMatrixGyro.begin()+PMatrixGyro.size(), numStates*numStates - PMatrixGyro.size(),
							0.0);
	}
	errorModelGyro.setNoiseMatrix(PMatrixGyro);
	errorModelGyro.setRNGSeed(RNGSeed);
	errorModelGyro.setUpperBounds(walkBoundsGyro);

}

void ImuSensor::CrossInit()
{
    InputStateID = SystemMessaging::GetInstance()->subscribeToMessage(InputStateMsg,
        sizeof(OutputStateData), moduleID);
    InputMassID = SystemMessaging::GetInstance()->subscribeToMessage(InputMassMsg,
        sizeof(MassPropsData), moduleID);
    if(InputStateID < 0 || InputMassID < 0)
    {
        std::cerr << "WARNING: Failed to link an imu input message: ";
        std::cerr << std::endl << "State: "<<InputStateID;
        std::cerr << std::endl << "Mass: "<<InputMassID;
    }
    return;
}

void ImuSensor::readInputMessages()
{
    SingleMessageHeader LocalHeader;
    
    memset(&this->StateCurrent, 0x0, sizeof(OutputStateData));
    if(InputStateID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(InputStateID, &LocalHeader,
                                                    sizeof(OutputStateData), reinterpret_cast<uint8_t*> (&this->StateCurrent), moduleID);
    }
    memset(&this->MassCurrent, 0x0, sizeof(MassPropsData));
    if(InputMassID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(InputMassID, &LocalHeader,
                                                    sizeof(MassPropsData), reinterpret_cast<uint8_t*> (&this->MassCurrent), moduleID);
    }
}

void ImuSensor::writeOutputMessages(uint64_t Clock)
{
    ImuSensorOutput LocalOutput;
    memcpy(LocalOutput.DVFramePlatform, this->sensedValues.DVFramePlatform, 3*sizeof(double));
    memcpy(LocalOutput.AccelPlatform, this->sensedValues.AccelPlatform, 3*sizeof(double));
    memcpy(LocalOutput.DRFramePlatform, this->sensedValues.DRFramePlatform, 3*sizeof(double));
    memcpy(LocalOutput.AngVelPlatform, this->sensedValues.AngVelPlatform, 3*sizeof(double));
    SystemMessaging::GetInstance()->WriteMessage(OutputDataID, Clock,
                                                 sizeof(ImuSensorOutput), reinterpret_cast<uint8_t*> (&LocalOutput), moduleID);
}

void ImuSensor::applySensorDiscretization(uint64_t CurrentTime)
{
    double scaledMeas[3];
    double intMeas[3];
    double dt;
    
    dt = (CurrentTime - PreviousTime)*1.0E-9;
    
    if(accelLSB > 0.0)
    {
        v3Scale(1.0/accelLSB, sensedValues.AccelPlatform, scaledMeas);
        for(uint32_t i=0; i<3; i++)
        {
            scaledMeas[i] = fabs(scaledMeas[i]);
            scaledMeas[i] = floor(scaledMeas[i]);
            scaledMeas[i] = scaledMeas[i]*accelLSB;
            scaledMeas[i] = copysign(scaledMeas[i], sensedValues.AccelPlatform[i]);
        }
        v3Subtract(sensedValues.AccelPlatform, scaledMeas, intMeas);
        v3Copy(scaledMeas, sensedValues.AccelPlatform);
        v3Scale(dt, intMeas, intMeas);
        v3Subtract(sensedValues.DVFramePlatform, intMeas, sensedValues.DVFramePlatform);
    }
    if(gyroLSB > 0.0)
    {
        v3Scale(1.0/gyroLSB, sensedValues.AngVelPlatform, scaledMeas);
        for(uint32_t i=0; i<3; i++)
        {
            scaledMeas[i] = fabs(scaledMeas[i]);
            scaledMeas[i] = floor(scaledMeas[i]);
            scaledMeas[i] = scaledMeas[i]*gyroLSB;
            scaledMeas[i] = copysign(scaledMeas[i], sensedValues.AngVelPlatform[i]);
        }
        v3Subtract(sensedValues.AngVelPlatform, scaledMeas, intMeas);
        v3Copy(scaledMeas, sensedValues.AngVelPlatform);
        v3Scale(dt, intMeas, intMeas);
        v3Subtract(sensedValues.DRFramePlatform, intMeas, sensedValues.DRFramePlatform); // why? -john
    }
    
}

void ImuSensor::applySensorErrors(uint64_t CurrentTime)
{
    double OmegaErrors[3] = {0, 0, 0};
    double AccelErrors[3]= {0, 0, 0};
    double dt;
    
    dt = (CurrentTime - PreviousTime)*1.0E-9;
    
    for(uint32_t i=0; i<3; i++)
    {
		OmegaErrors[i] = navErrorsGyro[i] + senRotBias[i];
        this->sensedValues.AngVelPlatform[i] =  this->trueValues.AngVelPlatform[i] + OmegaErrors[i];
        this->sensedValues.DRFramePlatform[i] = this->trueValues.DRFramePlatform[i] +  OmegaErrors[i]*dt;
		AccelErrors[i] = navErrorsAccel[i] + senTransBias[i];
        this->sensedValues.AccelPlatform[i] = this->trueValues.AccelPlatform[i] + AccelErrors[i];
        this->sensedValues.DVFramePlatform[i] = this->trueValues.DVFramePlatform[i] + AccelErrors[i]*dt;
    }
    
}

void ImuSensor::computeSensorErrors(uint64_t CurrentTime)
{
	this->errorModelAccel.setPropMatrix(AMatrixAccel);
	this->errorModelAccel.computeNextState();
	this->navErrorsAccel = this->errorModelAccel.getCurrentState();
	this->errorModelGyro.setPropMatrix(AMatrixGyro);
	this->errorModelGyro.computeNextState();
	this->navErrorsGyro = this->errorModelGyro.getCurrentState();
}


void ImuSensor::applySensorSaturation(uint64_t CurrentTime)
{
	double dt;

	dt = (CurrentTime - PreviousTime)*1.0E-9;

	for(uint32_t i=0; i<3; i++)
	{
		if(this->sensedValues.AngVelPlatform[i] > this->senRotMax) {
			this->sensedValues.AngVelPlatform[i] = this->senRotMax;
			this->sensedValues.DRFramePlatform[i] = this->senRotMax * dt;
		} else if (this->sensedValues.AngVelPlatform[i] < -this->senRotMax) {
			this->sensedValues.AngVelPlatform[i] = -this->senRotMax;
			this->sensedValues.DRFramePlatform[i] = -this->senRotMax * dt;
		}
		if(this->sensedValues.AccelPlatform[i] > this->senTransMax) {
			this->sensedValues.AccelPlatform[i] = this->senTransMax;
			this->sensedValues.DVFramePlatform[i] = this->senTransMax * dt;
		} else if (this->sensedValues.AccelPlatform[i] < -this->senTransMax) {
			this->sensedValues.AccelPlatform[i] = -this->senTransMax;
			this->sensedValues.DVFramePlatform[i] = -this->senTransMax * dt;
		}
	}

}

void ImuSensor::computePlatformDR()
{
    
    double MRP_Bdy2Inrtl_Prev[3];
    double MRP_BdyPrev2BdyNow[3];
    double DRBodyFrame[3];
    double T_Bdy2Platform[3][3];
    
    m33MultM33t(Str2Platform, StateCurrent.dcm_BS, T_Bdy2Platform);
    v3Scale(-1.0, StatePrevious.sigma_BN, MRP_Bdy2Inrtl_Prev);
    if(StateCurrent.MRPSwitchCount != StatePrevious.MRPSwitchCount)
    {
        for(uint32_t i=0; i<(StateCurrent.MRPSwitchCount -
                             StatePrevious.MRPSwitchCount); i++)
        {
            double Smag = v3Norm(MRP_Bdy2Inrtl_Prev);
            v3Scale(-1.0/Smag/Smag, MRP_Bdy2Inrtl_Prev, MRP_Bdy2Inrtl_Prev);
        }
    }
    addMRP(MRP_Bdy2Inrtl_Prev, StateCurrent.sigma_BN, MRP_BdyPrev2BdyNow);
    MRP2PRV(MRP_BdyPrev2BdyNow, DRBodyFrame);
    m33MultV3(T_Bdy2Platform, DRBodyFrame, this->trueValues.DRFramePlatform);
    m33MultV3(T_Bdy2Platform, StateCurrent.omega_BN_B, this->trueValues.AngVelPlatform);
}

void ImuSensor::computePlatformDV(uint64_t CurrentTime)
{
    
    double CmRelPos[3];
    double AlphaBodyRough[3];  /// -- Approximation but it shouldn't be too bad
    double omeg_x_omeg_x_r[3];
    double alpha_x_r[3];
    double RotForces[3];
    double InertialAccel[3];
    double dt;
    double T_Bdy2Platform[3][3];
    m33MultM33t(Str2Platform, StateCurrent.dcm_BS, T_Bdy2Platform);
    v3Subtract(SensorPosStr.data(), MassCurrent.CoM, CmRelPos);
    m33MultV3(StateCurrent.dcm_BS, CmRelPos, CmRelPos);
    dt = (CurrentTime - PreviousTime)*1.0E-9;
    v3Subtract(StateCurrent.omega_BN_B, StatePrevious.omega_BN_B, AlphaBodyRough);
    v3Scale(1.0/dt, AlphaBodyRough, AlphaBodyRough);
    v3Cross(AlphaBodyRough, CmRelPos, alpha_x_r);
    v3Cross(StateCurrent.omega_BN_B, CmRelPos, omeg_x_omeg_x_r);
    v3Cross(StateCurrent.omega_BN_B, omeg_x_omeg_x_r, omeg_x_omeg_x_r);
    v3Add(omeg_x_omeg_x_r, alpha_x_r, RotForces);
    v3Subtract(StateCurrent.TotalAccumDVBdy, StatePrevious.TotalAccumDVBdy,
               InertialAccel);
    v3Copy(InertialAccel, this->trueValues.DVFramePlatform);
    v3Scale(1.0/dt, InertialAccel, InertialAccel);
    v3Add(InertialAccel, RotForces, InertialAccel);
    m33MultV3(T_Bdy2Platform, InertialAccel, this->trueValues.AccelPlatform);
    v3Scale(dt, RotForces, RotForces);
    v3Add(this->trueValues.DVFramePlatform, RotForces, this->trueValues.DVFramePlatform);
    m33MultV3(T_Bdy2Platform, this->trueValues.DVFramePlatform, this->trueValues.DVFramePlatform);
    
}

void ImuSensor::UpdateState(uint64_t CurrentSimNanos)
{
    readInputMessages();

    if(NominalReady)
    {
        /* Compute true data */
        computePlatformDR();
        computePlatformDV(CurrentSimNanos);
        /* Compute sensed data */
		computeSensorErrors(CurrentSimNanos);
		applySensorErrors(CurrentSimNanos);
        applySensorDiscretization(CurrentSimNanos);
		applySensorSaturation(CurrentSimNanos);
        /* Output sensed data */
        writeOutputMessages(CurrentSimNanos);
    }
    memcpy(&StatePrevious, &StateCurrent, sizeof(OutputStateData));
    PreviousTime = CurrentSimNanos;
    NominalReady = true;
}
