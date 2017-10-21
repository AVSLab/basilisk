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
#include "utilities/avsEigenSupport.h"
#include "utilities/avsEigenMRP.h"


ImuSensor::ImuSensor()
{
    this->CallCounts = 0;
    this->InputStateID = -1;
    this->InputStateMsg = "inertial_state_output";
    this->OutputDataMsg = "imu_meas_data";
    this->setBodyToPlatformDCM(0.0, 0.0, 0.0);
    this->OutputBufferCount = 2;
    memset(&this->StatePrevious, 0x0, sizeof(SCPlusStatesSimMsg));
    memset(&this->StateCurrent, 0x0, sizeof(SCPlusStatesSimMsg));
    this->PreviousTime = 0;
    this->NominalReady = false;
    this->senRotBias.fill(0.0);
    this->senTransBias.fill(0.0);
    memset(&this->sensedValues, 0x0, sizeof(IMUSensorIntMsg));
    memset(&this->trueValues, 0x0, sizeof(IMUSensorIntMsg));
    this->accelLSB = 0.;
    this->gyroLSB = 0.;
    this->senRotMax = 1e6;
    this->senTransMax = 1e6;

    return;
}

void ImuSensor::setBodyToPlatformDCM(double yaw, double pitch, double roll)
{
    //Eigen::Vector3d q(yaw, pitch, roll);
    this->dcm_PB = eigenM1(roll)*eigenM2(pitch)*eigenM3(yaw);
    //Euler3212C(q, this->dcm_PB);

    return;
}

ImuSensor::~ImuSensor()
{
    return;
}

void ImuSensor::SelfInit()
{

    OutputDataID = SystemMessaging::GetInstance()->
        CreateNewMessage( OutputDataMsg, sizeof(IMUSensorIntMsg),
        OutputBufferCount, "IMUSensorIntMsg", moduleID);

	uint64_t numStates = 3;

	//this->AMatrixAccel.clear();
    this->AMatrixAccel.setIdentity(numStates,numStates);
	//this->AMatrixAccel.insert(this->AMatrixAccel.begin(), numStates*numStates, 0.0);
	//mSetIdentity(this->AMatrixAccel.data(), numStates, numStates);
	//for(uint32_t i=0; i<3; i++)
	//{
	//	this->AMatrixAccel.data()[i * 3 + i] = 1.0;
	//}
	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(this->PMatrixAccel.cols() != numStates || this->PMatrixAccel.rows() != numStates)
	{
		std::cerr << __FILE__ <<": Your process noise matrix (PMatrix) is not 3*3.";
		//std::cerr << "  You should fix that.  terminating"<<std::endl;
        std::cerr << "  You should fix that.  Expect Problems"<<std::endl;
		//this->PMatrixAccel.insert(this->PMatrixAccel.begin()+this->PMatrixAccel.size(), numStates*numStates - this->PMatrixAccel.size(),
					   //0.0);
	}
	this->errorModelAccel.setNoiseMatrix(this->PMatrixAccel);
	this->errorModelAccel.setRNGSeed(RNGSeed);
	this->errorModelAccel.setUpperBounds(walkBoundsAccel);

    this->AMatrixGyro.setIdentity(numStates, numStates);
	//this->AMatrixGyro.clear();
	//this->AMatrixGyro.insert(this->AMatrixGyro.begin(), numStates*numStates, 0.0);
	//mSetIdentity(this->AMatrixGyro.data(), numStates, numStates);
	//for(uint32_t i=0; i<3; i++)
	//{
	//	this->AMatrixGyro.data()[i * 3 + i] = 1.0;
	//}
	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(this->PMatrixGyro.rows() != numStates || this->PMatrixGyro.cols() != numStates)
	{
		std::cerr << __FILE__ <<": Your process noise matrix (PMatrix) is not 3*3.";
		//std::cerr << "  You should fix that.  Popping zeros onto end"<<std::endl;
        std::cerr << "  You should fix that.  Expect Problems"<<std::endl;
		//this->PMatrixGyro.insert(this->PMatrixGyro.begin()+this->PMatrixGyro.size(), numStates*numStates - this->PMatrixGyro.size(),
		//					0.0);
	}
	this->errorModelGyro.setNoiseMatrix(this->PMatrixGyro);
	this->errorModelGyro.setRNGSeed(RNGSeed);
	this->errorModelGyro.setUpperBounds(walkBoundsGyro);

    return;
}

void ImuSensor::CrossInit()
{
    InputStateID = SystemMessaging::GetInstance()->subscribeToMessage(InputStateMsg,
        sizeof(SCPlusStatesSimMsg), moduleID);
    if(InputStateID < 0 )
    {
        std::cerr << "WARNING: Failed to link an imu input message: ";
        std::cerr << std::endl << "State: "<<InputStateID;
    }

    return;
}

void ImuSensor::readInputMessages()
{
    SingleMessageHeader LocalHeader;
    
    memset(&this->StateCurrent, 0x0, sizeof(SCPlusStatesSimMsg));
    if(InputStateID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(InputStateID, &LocalHeader,
                                                    sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*> (&this->StateCurrent), moduleID);
    }

    return;
}

void ImuSensor::writeOutputMessages(uint64_t Clock)
{
    IMUSensorIntMsg LocalOutput;
    memcpy(LocalOutput.DVFramePlatform, this->sensedValues.DVFramePlatform, 3*sizeof(double));
    memcpy(LocalOutput.AccelPlatform, this->sensedValues.AccelPlatform, 3*sizeof(double));
    memcpy(LocalOutput.DRFramePlatform, this->sensedValues.DRFramePlatform, 3*sizeof(double));
    memcpy(LocalOutput.AngVelPlatform, this->sensedValues.AngVelPlatform, 3*sizeof(double));
    SystemMessaging::GetInstance()->WriteMessage(OutputDataID, Clock,
                                                 sizeof(IMUSensorIntMsg), reinterpret_cast<uint8_t*> (&LocalOutput), moduleID);

    return;
}

void ImuSensor::applySensorDiscretization(uint64_t CurrentTime)
{
    Eigen::Vector3d scaledMeas;
    Eigen::Vector3d intMeas;
    Eigen::Vector3d accelError;
    Eigen::Vector3d accelError_N;
    double dt;
    Eigen::MRPd sigma_BN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Matrix3d dcm_PN;
    Eigen::Vector3d accel_SN_N;
    Eigen::Vector3d DV_SN_N;
    Eigen::Vector3d DV_SN_P;
    
    dt = (CurrentTime - PreviousTime)*1.0E-9;
    
    if(this->accelLSB > 0.0) //If accelLSB has been set.
    {
        scaledMeas =  cArray2EigenVector3d(this->sensedValues.AccelPlatform) / this->accelLSB;
        //v3Scale(1.0/this->accelLSB, this->sensedValues.AccelPlatform, scaledMeas);
        for(uint32_t i=0; i<3; i++) //Discretize each part of the acceleration
        {
            scaledMeas[i] = fabs(scaledMeas[i]);
            scaledMeas[i] = floor(scaledMeas[i]);
            scaledMeas[i] = scaledMeas[i]*this->accelLSB;
            scaledMeas[i] = copysign(scaledMeas[i], this->sensedValues.AccelPlatform[i]);
        }
        accelError = cArray2EigenVector3d(this->sensedValues.AccelPlatform) - scaledMeas;
        eigenVector3d2CArray(scaledMeas, this->sensedValues.AccelPlatform);
        sigma_BN = cArray2EigenVector3d(this->StateCurrent.sigma_BN);
        dcm_BN =  sigma_BN.toRotationMatrix();
        dcm_PN = this->dcm_PB * dcm_BN;
        accel_SN_N = dcm_PN * cArray2EigenVector3d(this->sensedValues.AccelPlatform);
        DV_SN_N = dcm_PN * cArray2EigenVector3d(this->sensedValues.DVFramePlatform);
        accelError_N = dcm_PN * accelError;
        DV_SN_N = accelError_N * dt;
        DV_SN_P = dcm_PN * DV_SN_N;
        eigenVector3d2CArray(DV_SN_P, this->sensedValues.DVFramePlatform);
        
        //v3Subtract(this->sensedValues.AccelPlatform, scaledMeas, accelError);
        //v3Copy(scaledMeas, this->sensedValues.AccelPlatform);
        //v3Copy(this->StateCurrent.sigma_BN, sigma_BN);
        //MRP2C(sigma_BN, dcm_BN);
        //m33MultM33(this->dcm_PB, dcm_BN, dcm_PN);
        //m33tMultV3(dcm_PN, this->sensedValues.AccelPlatform, accel_SN_N);
        //m33tMultV3(dcm_PN, this->sensedValues.DVFramePlatform, DV_SN_N);
        //m33tMultV3(dcm_PN, accelError, accelError_N);
        //for(uint32_t i=0; i<3; i++){
        //    DV_SN_N[i] -= accelError_N[i]*dt;
        //}
        //m33MultV3(dcm_PN, DV_SN_N, this->sensedValues.DVFramePlatform);
    }
    
    if(gyroLSB > 0.0) //If gyroLSB has been set
    {
        v3Scale(1.0/this->gyroLSB, this->sensedValues.AngVelPlatform, scaledMeas);
        for(uint32_t i=0; i<3; i++) //Discretize each part of the angular rate
        {
            scaledMeas[i] = fabs(scaledMeas[i]);
            scaledMeas[i] = floor(scaledMeas[i]);
            scaledMeas[i] = scaledMeas[i]*this->gyroLSB;
            scaledMeas[i] = copysign(scaledMeas[i], this->sensedValues.AngVelPlatform[i]);
        }
        v3Subtract(this->sensedValues.AngVelPlatform, scaledMeas, intMeas);
        v3Copy(scaledMeas, this->sensedValues.AngVelPlatform);
        v3Scale(dt, intMeas, intMeas);
        v3Subtract(this->sensedValues.DRFramePlatform, intMeas, this->sensedValues.DRFramePlatform); //account for angular rate discretization in PRV output.
    }

    return;
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

    return;
}

void ImuSensor::computeSensorErrors()
{
	this->errorModelAccel.setPropMatrix(this->AMatrixAccel);
	this->errorModelAccel.computeNextState();
	this->navErrorsAccel = this->errorModelAccel.getCurrentState();
	this->errorModelGyro.setPropMatrix(this->AMatrixGyro);
	this->errorModelGyro.computeNextState();
	this->navErrorsGyro = this->errorModelGyro.getCurrentState();

    return;
}


void ImuSensor::applySensorSaturation(uint64_t CurrentTime)
{
	double  dt;
    int     aSat;
    double  sigma_BN[3];
    double  dcm_BN[3][3];
    double  dcm_PN[3][3];
    double  accel_SN_N[3];
    double  DV_SN_N[3];
    
	dt = (CurrentTime - PreviousTime)*1.0E-9;

    aSat = 0;
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
            aSat = 1;
		} else if (this->sensedValues.AccelPlatform[i] < -this->senTransMax) {
			this->sensedValues.AccelPlatform[i] = -this->senTransMax;
            aSat = 1;
		}
	}

    if (aSat){
        v3Copy(this->StateCurrent.sigma_BN, sigma_BN);
        MRP2C(sigma_BN, dcm_BN);
        m33MultM33(this->dcm_PB, dcm_BN, dcm_PN);
        m33tMultV3(dcm_PN, this->sensedValues.AccelPlatform, accel_SN_N);
        for(uint32_t i=0; i<3; i++)
        {
            DV_SN_N[i] = accel_SN_N[i]*dt;
        }
        m33MultV3(dcm_PN, DV_SN_N, this->sensedValues.DVFramePlatform);
    }
        

    return;
}

/*This function gathers actual spacecraft attitude from the spacecraftPlus output message.
 It then differences the state attitude between this time and the last time the IMU was called
 to get a DR (delta radians or delta rotation) The angular rate is retrieved directly from the
 spacecraftPlus output message and passed through to theother IMU functions which add noise, etc. */
void ImuSensor::computePlatformDR()
{
    
    double sigma_BN_2[3];    // MRP from body to inertial frame last time the IMU was called
    double sigma_BN_1[3];    // MRP from body to inertial frame now.
    double dcm_BN_1[3][3];  // direction cosine matrix from N to B at time 1
    double dcm_BN_2[3][3];  // direction cosine matrix from N to B at time 2
    double dcm_PN_1[3][3];  // direction cosine matrix from N to P at time 1
    double dcm_PN_2[3][3];  // direction cosine matrix from N to P at time 2
    double dcm_P2P1[3][3];  // direction cosine matrix from P at time 1 to P at time 2

    //Calculated time averaged cumulative rotation
    v3Copy(StatePrevious.sigma_BN, sigma_BN_1);
    v3Copy(StateCurrent.sigma_BN, sigma_BN_2);
    MRP2C(sigma_BN_1, dcm_BN_1);
    MRP2C(sigma_BN_2, dcm_BN_2);
    m33MultM33(this->dcm_PB, dcm_BN_1, dcm_PN_1);
    m33MultM33(this->dcm_PB, dcm_BN_2, dcm_PN_2);
    m33MultM33t(dcm_PN_2, dcm_PN_1, dcm_P2P1);
    C2PRV(dcm_P2P1, this->trueValues.DRFramePlatform);
    
    //calculate "instantaneous" angular rate
    m33MultV3(this->dcm_PB, this->StateCurrent.omega_BN_B, this->trueValues.AngVelPlatform); //returns instantaneous angular rate of imu sensor in imu platform frame coordinates

    return;
}

/*This functions gathers actual spacecraft velocity from the spacecraftPlus output message.
 It then differences the velocity between this time and the last time the IMU was called to get a
 DV (delta velocity). The acceleration of the spacecraft in the body frame is gathered directly from the spacecraftPlus
 output message. Then, it is converted to the platform frame and rotational terms are added to it
 to account for CoM offset of the platform frame. */
void ImuSensor::computePlatformDV(uint64_t CurrentTime)
{
    double omega_BN_N_1[3];     //omega_BN_N before
    double omega_BN_N_2[3];     //omega_BN_N now
    double omega_x_r_B[3];      //omega_BN_B x r_SB_B in body frame components
    double omega_x_r_N[3];      //above in inertial frame for numerical differencing derivative.
    double omega_x_r_prev_N[3]; //above in the inertial from for numerical differencing derivative.
    double omega_x_omega_x_r_B[3];//omega_BN_B x (omega_BN_B x r_SB_B) in body frame components
    double omegaDot_x_r_B[3];   //(time derivative of omega_BN_B) x r_SB_B in body frame components
    double rotationalTerms[3];  //(time derivative of omega_BN_B) x r_SB_B + omega_BN_B x (omega_BN_B x r_SB_B)
    double rotationalDelta_N[3];//delta in rotationl velocity term of sensor motion in N frame
    
    double r_SB_B[3];           //sensor position relative to B frame origin in B frame coordinates
    double r_SB_N_1[3];         //sensor positino relative to B frame origin in N frame coordinates previously
    double r_SB_N_2[3];         //sensor positino relative to B frame origin in N frame coordinates now
    
    double rDotDot_BN_B[3];     //non-conservative acceleration of body frame relative to inertial frame in body frame coordinates
    double rDotDot_SN_B[3];     //sensor non conservative acceleration relative to inertial frame in body frame coordinates
    double drDot_BN_N[3];       //change in velocity of body frame relative to inertial in body frame coordinates between IMU calls. This does not include delta-v from conservative accelerations.
    double dvSensor_B[3];       //sensor delta v between IMU calls in body frame coordinates
    double dvSensor_N[3];       //above but in the inertial frame.
    double accumDV_BN_N_1[3];   // Inertial DV accumulated since t=0 by the body in the inertial frame due to non-conservative forces at time 1
    double accumDV_BN_N_2[3];   // Inertial DV accumulated since t=0 by the body in the inertial frame due to non-conservative forces at time 1
    
    double dcm_BN_1[3][3];      // direction cosine matrix from N to B at time 1
    double dcm_BN_2[3][3];      // direction cosine matrix from N to B at time 2
    
    double dt;                  // timestep [s]

    dt = (CurrentTime - PreviousTime)*1E-9;
    
    //Calculate "instantaneous" linear acceleration
    v3Copy(this->StateCurrent.nonConservativeAccelpntB_B, rDotDot_BN_B);
    v3Copy(this->sensorPos_B, r_SB_B);
    v3Cross(this->StateCurrent.omegaDot_BN_B, r_SB_B, omegaDot_x_r_B);
    v3Cross(this->StateCurrent.omega_BN_B, r_SB_B, omega_x_r_B);
    v3Cross(this->StateCurrent.omega_BN_B, omega_x_r_B, omega_x_omega_x_r_B);
    v3Add(omegaDot_x_r_B, omega_x_omega_x_r_B, rotationalTerms);
    v3Add(rDotDot_BN_B, rotationalTerms, rDotDot_SN_B);
    m33MultV3(this->dcm_PB, rDotDot_SN_B, this->trueValues.AccelPlatform);
    
    //Calculate time-average cumulative delta v
    MRP2C(this->StatePrevious.sigma_BN, dcm_BN_1);
    MRP2C(this->StateCurrent.sigma_BN, dcm_BN_2);
    m33tMultV3(dcm_BN_1, this->StatePrevious.TotalAccumDV_BN_B, accumDV_BN_N_1);
    m33tMultV3(dcm_BN_2, this->StateCurrent.TotalAccumDV_BN_B, accumDV_BN_N_2);
    v3Subtract(accumDV_BN_N_2, accumDV_BN_N_1, drDot_BN_N);
    m33tMultV3(dcm_BN_1, r_SB_B, r_SB_N_1);
    m33tMultV3(dcm_BN_2, r_SB_B, r_SB_N_2);
    m33tMultV3(dcm_BN_1, this->StatePrevious.omega_BN_B, omega_BN_N_1);
    m33tMultV3(dcm_BN_2, this->StateCurrent.omega_BN_B, omega_BN_N_2);
    v3Cross(omega_BN_N_1, r_SB_N_1, omega_x_r_prev_N);
    v3Cross(omega_BN_N_2, r_SB_N_2, omega_x_r_N);
    v3Subtract(omega_x_r_N, omega_x_r_prev_N, rotationalDelta_N);
    v3Add(drDot_BN_N, rotationalDelta_N, dvSensor_N);
    m33MultV3(dcm_BN_2, dvSensor_N, dvSensor_B);
    m33MultV3(this->dcm_PB, dvSensor_B, this->trueValues.DVFramePlatform);
    
    return;
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
		computeSensorErrors();
		applySensorErrors(CurrentSimNanos);
        applySensorDiscretization(CurrentSimNanos);
		applySensorSaturation(CurrentSimNanos);
        /* Output sensed data */
        writeOutputMessages(CurrentSimNanos);
    }
    memcpy(&StatePrevious, &StateCurrent, sizeof(SCPlusStatesSimMsg));
    PreviousTime = CurrentSimNanos;
    NominalReady = true;

    return;
}
