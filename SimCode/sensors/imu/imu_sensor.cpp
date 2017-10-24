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
#include "simFswInterfaceMessages/macroDefinitions.h"


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
    this->dcm_PB = eigenM1(roll)*eigenM2(pitch)*eigenM3(yaw);

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

    this->AMatrixAccel.setIdentity(numStates,numStates);

	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(this->PMatrixAccel.cols() != numStates || this->PMatrixAccel.rows() != numStates)
	{
		std::cerr << __FILE__ <<": Your process noise matrix (PMatrix) is not 3*3.";
        std::cerr << "  You should fix that.  Expect Problems"<<std::endl;
	}
	this->errorModelAccel.setNoiseMatrix(this->PMatrixAccel);
	this->errorModelAccel.setRNGSeed(RNGSeed);
	this->errorModelAccel.setUpperBounds(walkBoundsAccel);

    this->AMatrixGyro.setIdentity(numStates, numStates);

	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(this->PMatrixGyro.rows() != numStates || this->PMatrixGyro.cols() != numStates)
	{
		std::cerr << __FILE__ <<": Your process noise matrix (PMatrix) is not 3*3.";
        std::cerr << "  You should fix that.  Expect Problems"<<std::endl;
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
    Eigen::Vector3d scaledMeas; //measurement that is scaled due to discretization
    Eigen::Vector3d intMeas; //error due to discretization in angular rate
    Eigen::Vector3d accelError; //error in acceleration in the P Frame
    Eigen::Vector3d accelError_N; //error vector converted to N frame
    double dt; //time step
    Eigen::MRPd sigma_BN; //MRP B wrt N
    Eigen::Matrix3d dcm_BN; //dcm B wrt N
    Eigen::Matrix3d dcm_PN; // dcm P wrt N
    Eigen::Vector3d accel_SN_N; //inertial acceration of sensor in inertial frame
    Eigen::Vector3d DV_SN_N; //accumulated DV of sensor wrt inertial in inertial coordinates
    Eigen::Vector3d DV_SN_P; //accumulated DV of sensor wrt inertial in platform(sensor) frame
    Eigen::Vector3d angVel; //eigen version of this->sensedValues.AngVelPlatform
    Eigen::Vector3d DRFrame; //eigen version of this->sensedValues.DRFramePlatform
    
    dt = (CurrentTime - PreviousTime)*1.0E-9;
    
    if(this->accelLSB > 0.0) //If accelLSB has been set.
    {
        scaledMeas =  cArray2EigenVector3d(this->sensedValues.AccelPlatform) / this->accelLSB;
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
    }
    
    if(gyroLSB > 0.0) //If gyroLSB has been set
    {
        angVel = cArray2EigenVector3d(this->sensedValues.AngVelPlatform);
        scaledMeas = angVel / this->gyroLSB;
        for(uint32_t i=0; i<3; i++) //Discretize each part of the angular rate
        {
            scaledMeas[i] = fabs(scaledMeas[i]);
            scaledMeas[i] = floor(scaledMeas[i]);
            scaledMeas[i] = scaledMeas[i]*this->gyroLSB;
            scaledMeas[i] = copysign(scaledMeas[i], this->sensedValues.AngVelPlatform[i]);
        }
        
        intMeas = angVel - scaledMeas;
        eigenVector3d2CArray(scaledMeas, this->sensedValues.AngVelPlatform); //return discretized angular velocity
        intMeas *= dt;
        DRFrame = cArray2EigenVector3d(this->sensedValues.DRFramePlatform) - intMeas;
        eigenVector3d2CArray(DRFrame, this->sensedValues.DRFramePlatform);
    }

    return;
}

void ImuSensor::applySensorErrors(uint64_t CurrentTime)
{
    Eigen::Vector3d OmegaErrors; //angular noise plus bias
    Eigen::Vector3d AccelErrors; //linear noise plus bias
    Eigen::Vector3d angVel; //eigen version of AngVelPlatform. It starts as trueValue.AngVelPlatform and becomes sensedValues.AngVelPlatform when noise is added. The same goes for DR, accel, and DV below.
    Eigen::Vector3d DR; //eigen version of DRFramePlatform
    Eigen::Vector3d accel; //eigen version of AccelPlatform
    Eigen::Vector3d DV; //eigen version of DVFrame Platform
    double dt; //time step
    
    angVel = cArray2EigenVector3d(this->trueValues.AngVelPlatform);
    DR = cArray2EigenVector3d(this->trueValues.DRFramePlatform);
    accel = cArray2EigenVector3d(this->trueValues.AccelPlatform);
    DV = cArray2EigenVector3d(this->trueValues.DVFramePlatform);
    
    dt = (CurrentTime - PreviousTime)*1.0E-9;
    
    OmegaErrors = navErrorsGyro + senRotBias;
    angVel += OmegaErrors;
    DR += OmegaErrors * dt;
    
    AccelErrors = navErrorsAccel + senTransBias;
    accel += AccelErrors;
    DV += AccelErrors * dt;
    
    eigenVector3d2CArray(angVel, this->sensedValues.AngVelPlatform);
    eigenVector3d2CArray(DR, this->sensedValues.AngVelPlatform);
    eigenVector3d2CArray(accel, this->sensedValues.AccelPlatform);
    eigenVector3d2CArray(DV, this->sensedValues.DVFramePlatform);

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
    Eigen::MRPd  sigma_BN;
    Eigen::Matrix3d  dcm_BN;
    Eigen::Matrix3d  dcm_PN;
    Eigen::Vector3d  accel_SN_N;
    Eigen::Vector3d  DV_SN_N;
    Eigen::Vector3d  DV_SN_P;
    
    Eigen::Vector3d angVel; //eigen version of sensedValues.AngVelPlatform
    Eigen::Vector3d DR; //eigen version of sensedValues.DRFramePlatform
    Eigen::Vector3d accel; //eigen version of sensedValues.AccelPlatform
    Eigen::Vector3d DV; //eigen version of sensedValues.DVFramePlatform
    
	dt = (CurrentTime - PreviousTime)*1.0E-9;
    
    angVel = cArray2EigenVector3d(this->sensedValues.AngVelPlatform);
    DR = cArray2EigenVector3d(this->sensedValues.DRFramePlatform);
    accel = cArray2EigenVector3d(this->sensedValues.AccelPlatform);
    DV = cArray2EigenVector3d(this->sensedValues.DVFramePlatform);

    aSat = 0;
	for(uint32_t i=0; i<3; i++)
	{
		if(angVel[i] > this->senRotMax) {
			angVel[i] = this->senRotMax;
			DR[i] = this->senRotMax * dt;
		} else if (angVel[i] < -this->senRotMax) {
			angVel[i] = -this->senRotMax;
			DR[i] = -this->senRotMax * dt;
		}
		if(accel[i] > this->senTransMax) {
			accel[i] = this->senTransMax;
            aSat = 1;
		} else if (accel[i] < -this->senTransMax) {
			accel[i] = -this->senTransMax;
            aSat = 1;
		}
	}

    if (aSat){
        sigma_BN = cArray2EigenVector3d(this->StateCurrent.sigma_BN);
        dcm_BN = sigma_BN.toRotationMatrix();
        dcm_PN = this->dcm_PB * dcm_BN;
        accel_SN_N = dcm_PN * accel;
        DV_SN_N = accel_SN_N * dt;
        DV_SN_P = dcm_PN * DV_SN_N;
        eigenVector3d2CArray(DV_SN_P, this->sensedValues.DVFramePlatform);
    }
        

    return;
}

/*This function gathers actual spacecraft attitude from the spacecraftPlus output message.
 It then differences the state attitude between this time and the last time the IMU was called
 to get a DR (delta radians or delta rotation) The angular rate is retrieved directly from the
 spacecraftPlus output message and passed through to theother IMU functions which add noise, etc. */
void ImuSensor::computePlatformDR()
{
    Eigen::MRPd sigma_BN_2;    // MRP from body to inertial frame last time the IMU was called
    Eigen::MRPd sigma_BN_1;    // MRP from body to inertial frame now.
    Eigen::Matrix3d dcm_BN_1;  // direction cosine matrix from N to B at time 1
    Eigen::Matrix3d dcm_BN_2;  // direction cosine matrix from N to B at time 2
    Eigen::Matrix3d dcm_PN_1;  // direction cosine matrix from N to P at time 1
    Eigen::Matrix3d dcm_PN_2;  // direction cosine matrix from N to P at time 2
    Eigen::Matrix3d dcm_P2P1;  // direction cosine matrix from P at time 1 to P at time 2
    double dcm_P2P1_cArray[9]; //dcm_P2P1 as cArray for C2PRV conversion
    Eigen::Vector3d omega_BN_B_eigen; //eigen version of statecurrent.omega_BN_B
    Eigen::Vector3d angVel; // eigen version of this->trueValues.AngVelPlatform

    
    //Calculated time averaged cumulative rotation
    sigma_BN_1 = cArray2EigenVector3d(this->StatePrevious.sigma_BN);
    sigma_BN_2 = cArray2EigenVector3d(this->StateCurrent.sigma_BN);
    dcm_BN_1 = sigma_BN_1.toRotationMatrix();
    dcm_BN_2 = sigma_BN_2.toRotationMatrix();
    dcm_PN_1 = this->dcm_PB * dcm_BN_1;
    dcm_PN_2 = this->dcm_PB * dcm_BN_2;
    dcm_P2P1 = dcm_PN_2 * dcm_PN_1.transpose();
    eigenMatrix3d2CArray(dcm_P2P1, dcm_P2P1_cArray);
    
    C2PRV(RECAST3X3 dcm_P2P1_cArray, this->trueValues.DRFramePlatform);
    
    //calculate "instantaneous" angular rate
    omega_BN_B_eigen = cArray2EigenVector3d(this->StateCurrent.omega_BN_B);
    angVel = this->dcm_PB * omega_BN_B_eigen;
    eigenVector3d2CArray(angVel, this->trueValues.AngVelPlatform); //returns instantaneous angular rate of imu sensor in imu platform frame coordinates

    return;
}

/*This functions gathers actual spacecraft velocity from the spacecraftPlus output message.
 It then differences the velocity between this time and the last time the IMU was called to get a
 DV (delta velocity). The acceleration of the spacecraft in the body frame is gathered directly from the spacecraftPlus
 output message. Then, it is converted to the platform frame and rotational terms are added to it
 to account for CoM offset of the platform frame. */
void ImuSensor::computePlatformDV(uint64_t CurrentTime)
{
    Eigen::Vector3d omega_BN_N_1;     //omega_BN_N before
    Eigen::Vector3d omega_BN_N_2;     //omega_BN_N now
    Eigen::Vector3d omegaDot_BN_B_2;  //this->StateCurrent.omegaDot_BN_B as eigen
    Eigen::Vector3d omega_BN_B_1;     //this->StatePrevious.omega_BN_B as eigen
    Eigen::Vector3d omega_BN_B_2;     //this->StateCurrent.omega_BN_B as eigen
    Eigen::Vector3d omega_x_r_B;      //omega_BN_B x r_SB_B in body frame components
    Eigen::Vector3d omega_x_r_N;      //above in inertial frame for numerical differencing derivative.
    Eigen::Vector3d omega_x_r_prev_N; //above in the inertial from for numerical differencing derivative.
    Eigen::Vector3d omega_x_omega_x_r_B;//omega_BN_B x (omega_BN_B x r_SB_B) in body frame components
    Eigen::Vector3d omegaDot_x_r_B;   //(time derivative of omega_BN_B) x r_SB_B in body frame components
    Eigen::Vector3d rotationalTerms;  //(time derivative of omega_BN_B) x r_SB_B + omega_BN_B x (omega_BN_B x r_SB_B)
    Eigen::Vector3d rotationalDelta_N;//delta in rotationl velocity term of sensor motion in N frame
    
    Eigen::Vector3d r_SB_B;           //sensor position relative to B frame origin in B frame coordinates
    Eigen::Vector3d r_SB_N_1;         //sensor positino relative to B frame origin in N frame coordinates previously
    Eigen::Vector3d r_SB_N_2;         //sensor positino relative to B frame origin in N frame coordinates now
    
    Eigen::Vector3d rDotDot_BN_B;     //non-conservative acceleration of body frame relative to inertial frame in body frame coordinates
    Eigen::Vector3d rDotDot_SN_B;     //sensor non conservative acceleration relative to inertial frame in body frame coordinates
    Eigen::Vector3d rDotDot_SN_P;     //vector above converted to P-frame coordinates
    Eigen::Vector3d drDot_BN_N;       //change in velocity of body frame relative to inertial in body frame coordinates between IMU calls. This does not include delta-v from conservative accelerations.
    Eigen::Vector3d dvSensor_B;       //sensor delta v between IMU calls in body frame coordinates
    Eigen::Vector3d dvSensor_N;       //above but in the inertial frame.
    Eigen::Vector3d dvSensor_P;       //above but in the platform frame.
    Eigen::Vector3d accumDV_BN_N_1;   // Inertial DV accumulated since t=0 by the body in the inertial frame due to non-conservative forces at time 1
    Eigen::Vector3d accumDV_BN_N_2;   // Inertial DV accumulated since t=0 by the body in the inertial frame due to non-conservative forces at time 1
    
    Eigen::Matrix3d dcm_BN_1;      // direction cosine matrix from N to B at time 1
    Eigen::Matrix3d dcm_BN_2;      // direction cosine matrix from N to B at time 2
    
    Eigen::MRPd sigma_BN_1;
    Eigen::MRPd sigma_BN_2;
    
    Eigen::Vector3d accumDV_BN_B_1;
    Eigen::Vector3d accumDV_BN_B_2;
    
    double dt;                  // timestep [s]

    dt = (CurrentTime - PreviousTime)*1E-9;
    
    //Calculate "instantaneous" linear acceleration
    rDotDot_BN_B = cArray2EigenVector3d(this->StateCurrent.nonConservativeAccelpntB_B);
    r_SB_B = this->sensorPos_B;
    omegaDot_BN_B_2 = cArray2EigenVector3d(this->StateCurrent.omegaDot_BN_B);
    omegaDot_x_r_B = omegaDot_BN_B_2.cross(r_SB_B);
    omega_BN_B_2 = cArray2EigenVector3d(this->StateCurrent.omega_BN_B);
    omega_x_r_B = omega_BN_B_2.cross(r_SB_B);
    omega_x_omega_x_r_B = omega_BN_B_2.cross(omega_x_r_B);
    rotationalTerms = omegaDot_x_r_B + omega_x_omega_x_r_B;
    rDotDot_SN_B = rDotDot_BN_B + rotationalTerms;
    rDotDot_SN_P = this->dcm_PB * rDotDot_SN_B;
    eigenVector3d2CArray(rDotDot_SN_P, this->trueValues.AccelPlatform);
    
    //Calculate time-average cumulative delta v
    sigma_BN_1 = cArray2EigenVector3d(this->StatePrevious.sigma_BN);
    sigma_BN_2 = cArray2EigenVector3d(this->StateCurrent.sigma_BN);
    dcm_BN_1 = sigma_BN_1.toRotationMatrix();
    dcm_BN_2 = sigma_BN_2.toRotationMatrix();
    accumDV_BN_B_1 = cArray2EigenVector3d(this->StatePrevious.TotalAccumDV_BN_B);
    accumDV_BN_B_2 = cArray2EigenVector3d(this->StateCurrent.TotalAccumDV_BN_B);
    accumDV_BN_N_1 = dcm_BN_1 * accumDV_BN_B_1;
    accumDV_BN_N_2 = dcm_BN_2 * accumDV_BN_B_2;
    drDot_BN_N = accumDV_BN_N_2 - accumDV_BN_N_1;
    r_SB_N_1 = dcm_BN_1 * r_SB_B;
    r_SB_N_2 = dcm_BN_2 * r_SB_B;
    omega_BN_B_1 = cArray2EigenVector3d(this->StatePrevious.omega_BN_B);
    omega_BN_N_1 = dcm_BN_1 * omega_BN_B_1;
    omega_BN_N_2 = dcm_BN_2 * omega_BN_B_2;
    omega_x_r_prev_N = omega_BN_N_1.cross(r_SB_N_1);
    omega_x_r_N = omega_BN_N_2.cross(r_SB_N_2);
    rotationalDelta_N = omega_x_r_N - omega_x_r_prev_N;
    dvSensor_N = drDot_BN_N + rotationalDelta_N;
    dvSensor_B = dcm_BN_2 * dvSensor_N;
    dvSensor_P = this->dcm_PB * dvSensor_B;
    eigenVector3d2CArray(dvSensor_P, this->trueValues.DVFramePlatform);
    
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
