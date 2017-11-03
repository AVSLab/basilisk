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
    
    this->errorModelGyro =  new GaussMarkov(3);
    this->errorModelAccel = new GaussMarkov(3);
    
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
    this->PMatrixGyro.fill(0.0);
    this->AMatrixGyro.fill(0.0);
    this->PMatrixAccel.fill(0.0);
    this->AMatrixAccel.fill(0.0);
    this->walkBoundsGyro.fill(0.0);
    this->walkBoundsAccel.fill(0.0);
    this->navErrorsGyro.fill(0.0);
    this->navErrorsAccel.fill(0.0);
    this->previous_omega_BN_B.fill(0.0);
    this->current_omega_BN_B.fill(0.0);
    this->current_nonConservativeAccelpntB_B.fill(0.0);
    this->current_omegaDot_BN_B.fill(0.0);
    this->previous_TotalAccumDV_BN_B.fill(0.0);
    this->current_TotalAccumDV_BN_B.fill(0.0);
    this->accel_SN_P_out.fill(0.0);
    this->DV_SN_P_out.fill(0.0);
    this->omega_PN_P_out.fill(0.0);
    this->prv_PN_out.fill(0.0);
    
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

    this->OutputDataID = SystemMessaging::GetInstance()->
        CreateNewMessage( this->OutputDataMsg, sizeof(IMUSensorIntMsg),
        this->OutputBufferCount, "IMUSensorIntMsg", this->moduleID);

	uint64_t numStates = 3;

    this->AMatrixAccel.setIdentity(numStates,numStates);

	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(this->PMatrixAccel.cols() != numStates || this->PMatrixAccel.rows() != numStates)
	{
		std::cerr << __FILE__ <<": Your process noise matrix (PMatrixAccel) is not 3*3.";
        std::cerr << "  Quitting."<<std::endl;
        return;
	}
	this->errorModelAccel->setNoiseMatrix(this->PMatrixAccel);
	this->errorModelAccel->setRNGSeed(this->RNGSeed);
	this->errorModelAccel->setUpperBounds(this->walkBoundsAccel);

    this->AMatrixGyro.setIdentity(numStates, numStates);

	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(this->PMatrixGyro.rows() != numStates || this->PMatrixGyro.cols() != numStates)
	{
		std::cerr << __FILE__ <<": Your process noise matrix (PMatrixGyro) is not 3*3.";
        std::cerr << "  Quitting."<<std::endl;
        return;
	}
	this->errorModelGyro->setNoiseMatrix(this->PMatrixGyro);
	this->errorModelGyro->setRNGSeed(this->RNGSeed);
	this->errorModelGyro->setUpperBounds(this->walkBoundsGyro);

    return;
}

void ImuSensor::CrossInit()
{
    this->InputStateID = SystemMessaging::GetInstance()->subscribeToMessage(this->InputStateMsg,
        sizeof(SCPlusStatesSimMsg), this->moduleID);
    if(this->InputStateID < 0 )
    {
        std::cerr << "WARNING: Failed to link an imu input message: ";
        std::cerr << std::endl << "State: "<<this->InputStateID;
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
    this->current_sigma_BN = cArray2EigenVector3d(this->StateCurrent.sigma_BN);
    this->current_omega_BN_B = cArray2EigenVector3d(this->StateCurrent.omega_BN_B);
    this->current_nonConservativeAccelpntB_B = cArray2EigenVector3d(this->StateCurrent.nonConservativeAccelpntB_B);
    this->current_omegaDot_BN_B = cArray2EigenVector3d(this->StateCurrent.omegaDot_BN_B);
    this->current_TotalAccumDV_BN_B = cArray2EigenVector3d(this->StateCurrent.TotalAccumDV_BN_B);


    return;
}

void ImuSensor::writeOutputMessages(uint64_t Clock)
{
    IMUSensorIntMsg LocalOutput;
    
    eigenVector3d2CArray(this->accel_SN_P_out, LocalOutput.AccelPlatform);
    eigenVector3d2CArray(this->DV_SN_P_out, LocalOutput.DVFramePlatform);
    eigenVector3d2CArray(this->omega_PN_P_out, LocalOutput.AngVelPlatform);
    eigenVector3d2CArray(this->prv_PN_out, LocalOutput.DRFramePlatform);
    
    SystemMessaging::GetInstance()->WriteMessage(OutputDataID, Clock,
                                                 sizeof(IMUSensorIntMsg), reinterpret_cast<uint8_t*> (&LocalOutput), moduleID);

    return;
}

void ImuSensor::applySensorDiscretization(uint64_t CurrentTime)
{
    double dt; //time step
    
    Eigen::Vector3d accel_SN_P_disc; //discretized of the above
    Eigen::Vector3d accelError; //error in acceleration in the P Frame
    Eigen::Vector3d accelError_N; //error vector converted to N frame
    Eigen::Matrix3d dcm_BN; //dcm B wrt N
    Eigen::Matrix3d dcm_PN; // dcm P wrt N
    Eigen::Vector3d accel_SN_N_disc; //inertial acceration of sensor in inertial frame
    Eigen::Vector3d DV_SN_N; //accumulated DV of sensor wrt inertial in inertial coordinates
    Eigen::Vector3d DV_SN_N_disc; // discretized of the above
    
    Eigen::Vector3d omega_PN_P_disc; //discretized of the above
    Eigen::Vector3d omega_error_P; //error due to discretization in angular rate
    
    dt = (CurrentTime - this->PreviousTime)*1.0E-9;
    
    
    
    if(this->accelLSB > 0.0) //If accelLSB has been set.
    {
        
        for(uint32_t i=0; i<3; i++) //Discretize each part of the acceleration
        {
            accel_SN_P_disc[i] = this->accel_SN_P_out[i] / this->accelLSB;
            accel_SN_P_disc[i] = fabs(accel_SN_P_disc[i]);
            accel_SN_P_disc[i] = floor(accel_SN_P_disc[i]);
            accel_SN_P_disc[i] = accel_SN_P_disc[i]*this->accelLSB;
            accel_SN_P_disc[i] = copysign(accel_SN_P_disc[i], this->accel_SN_P_out[i]);
        }
        
        //integrate the acceleration discretization error into DV
        accelError = this->accel_SN_P_out - accel_SN_P_disc;
        dcm_BN =  this->current_sigma_BN.toRotationMatrix().transpose();
        dcm_PN = this->dcm_PB * dcm_BN;
        accelError_N = dcm_PN.transpose() * accelError;
        DV_SN_N = dcm_PN.transpose() * this->DV_SN_P_out;
        DV_SN_N_disc = DV_SN_N - accelError_N * dt;
        this->DV_SN_P_out = dcm_PN * DV_SN_N_disc;
        this->accel_SN_P_out = accel_SN_P_disc;
    }
    
    if(this->gyroLSB > 0.0) //If gyroLSB has been set
    {
        
        for(uint32_t i=0; i<3; i++) //Discretize each part of the angular rate
        {
            omega_PN_P_disc[i] = this->omega_PN_P_out[i] / this->gyroLSB;
            omega_PN_P_disc[i] = fabs(omega_PN_P_disc[i]);
            omega_PN_P_disc[i] = floor(omega_PN_P_disc[i]);
            omega_PN_P_disc[i] = omega_PN_P_disc[i] * this->gyroLSB;
            omega_PN_P_disc[i] = copysign(omega_PN_P_disc[i], this->omega_PN_P_out[i]);
        }
        //integrate error through omega
        omega_error_P = this->omega_PN_P_out - omega_PN_P_disc;
        this->prv_PN_out -= omega_error_P * dt;
        this->omega_PN_P_out = omega_PN_P_disc;
    }

    return;
}

void ImuSensor::applySensorErrors(uint64_t CurrentTime)
{
    Eigen::Vector3d OmegaErrors; //angular noise plus bias
    Eigen::Vector3d AccelErrors; //linear noise plus bias
    double dt; //time step

    dt = (CurrentTime - this->PreviousTime)*1.0E-9;
    
    OmegaErrors = this->navErrorsGyro + this->senRotBias;
    this->omega_PN_P_out += OmegaErrors;
    this->prv_PN_out += OmegaErrors * dt;
    
    AccelErrors = this->navErrorsAccel + this->senTransBias;
    this->accel_SN_P_out += AccelErrors;
    this->DV_SN_P_out += AccelErrors * dt;

    return;
}

void ImuSensor::computeSensorErrors()
{
	this->errorModelAccel->setPropMatrix(this->AMatrixAccel);
	this->errorModelAccel->computeNextState();
	this->navErrorsAccel = this->errorModelAccel->getCurrentState();
	this->errorModelGyro->setPropMatrix(this->AMatrixGyro);
	this->errorModelGyro->computeNextState();
	this->navErrorsGyro = this->errorModelGyro->getCurrentState();

    return;
}


void ImuSensor::applySensorSaturation(uint64_t CurrentTime)
{
	double  dt;
    int     aSat;
    Eigen::Vector3d  accel_SN_N; //above in the N frame
    Eigen::Matrix3d  dcm_BN;
    Eigen::Matrix3d  dcm_PN;
    Eigen::Vector3d  DV_SN_N;
    
	dt = (CurrentTime - PreviousTime)*1.0E-9;

    aSat = 0;
	for(uint32_t i=0; i<3; i++)
	{
		if(this->omega_PN_P_out[i] > this->senRotMax) {
			this->omega_PN_P_out[i] = this->senRotMax;
			this->prv_PN_out[i] = this->senRotMax * dt;
		} else if (this->omega_PN_P_out[i] < -this->senRotMax) {
			this->omega_PN_P_out[i] = -this->senRotMax;
			this->prv_PN_out[i] = -this->senRotMax * dt;
		}
		if(this->accel_SN_P_out[i] > this->senTransMax) {
			this->accel_SN_P_out[i] = this->senTransMax;
            aSat = 1;
		} else if (this->accel_SN_P_out[i] < -this->senTransMax) {
			this->accel_SN_P_out[i] = -this->senTransMax;
            aSat = 1;
		}
	}

    if (aSat){
        dcm_BN = this->current_sigma_BN.toRotationMatrix().transpose();
        dcm_PN = this->dcm_PB * dcm_BN;
        accel_SN_N = dcm_PN.transpose() * this->accel_SN_P_out;
        DV_SN_N = accel_SN_N * dt;
        this->DV_SN_P_out = dcm_PN * DV_SN_N;
    }

    return;
}

/*This function gathers actual spacecraft attitude from the spacecraftPlus output message.
 It then differences the state attitude between this time and the last time the IMU was called
 to get a DR (delta radians or delta rotation) The angular rate is retrieved directly from the
 spacecraftPlus output message and passed through to theother IMU functions which add noise, etc. */
void ImuSensor::computePlatformDR()
{
    Eigen::Matrix3d dcm_BN_1;  // direction cosine matrix from N to B at time 1
    Eigen::Matrix3d dcm_BN_2;  // direction cosine matrix from N to B at time 2
    Eigen::Matrix3d dcm_PN_1;  // direction cosine matrix from N to P at time 1
    Eigen::Matrix3d dcm_PN_2;  // direction cosine matrix from N to P at time 2
    Eigen::Matrix3d dcm_P2P1;  // direction cosine matrix from P at time 1 to P at time 2
    double dcm_P2P1_cArray[9]; //dcm_P2P1 as cArray for C2PRV conversion
    double prv_PN_cArray[3]; //cArray of PRV

    
    //Calculated time averaged cumulative rotation
    dcm_BN_1 = this->previous_sigma_BN.toRotationMatrix().transpose();
    dcm_BN_2 = this->current_sigma_BN.toRotationMatrix().transpose();
    dcm_PN_1 = this->dcm_PB * dcm_BN_1;
    dcm_PN_2 = this->dcm_PB * dcm_BN_2;
    dcm_P2P1 = dcm_PN_2 * dcm_PN_1.transpose();
    eigenMatrix3d2CArray(dcm_P2P1, dcm_P2P1_cArray); //makes a 9x1
    C2PRV(RECAST3X3 dcm_P2P1_cArray, prv_PN_cArray); //makes it back into a 3x3
    this->prv_PN_out = cArray2EigenVector3d(prv_PN_cArray);//writes it back to the variable to be passed along.
    
    //calculate "instantaneous" angular rate
    this->omega_PN_P_out = this->dcm_PB * this->current_omega_BN_B;

    return;
}

/*This functions gathers actual spacecraft velocity from the spacecraftPlus output message.
 It then differences the velocity between this time and the last time the IMU was called to get a
 DV (delta velocity). The acceleration of the spacecraft in the body frame is gathered directly from the spacecraftPlus
 output message. Then, it is converted to the platform frame and rotational terms are added to it
 to account for CoM offset of the platform frame. */
void ImuSensor::computePlatformDV(uint64_t CurrentTime)
{
    Eigen::Vector3d rDotDot_BN_B;     //non-conservative acceleration of body frame relative to inertial frame in body frame coordinates
    Eigen::Vector3d rDotDot_SN_B;     //sensor non conservative acceleration relative to inertial frame in body frame coordinates
    Eigen::Vector3d omega_x_r_B;      //omega_BN_B x r_SB_B in body frame components
    Eigen::Vector3d omega_x_r_N;      //above in inertial frame for numerical differencing derivative.
    Eigen::Vector3d omega_x_r_prev_N; //above in the inertial from for numerical differencing derivative.
    Eigen::Vector3d omega_x_omega_x_r_B;//omega_BN_B x (omega_BN_B x r_SB_B) in body frame components
    Eigen::Vector3d omegaDot_x_r_B;   //(time derivative of omega_BN_B) x r_SB_B in body frame components
    Eigen::Vector3d rotationalTerms;  //(time derivative of omega_BN_B) x r_SB_B + omega_BN_B x (omega_BN_B x r_SB_B)
    Eigen::Vector3d r_SB_B;           //sensor position relative to B frame origin in B frame coordinates
    
    Eigen::Vector3d omega_BN_N_1;     //omega_BN_N before
    Eigen::Vector3d omega_BN_N_2;     //omega_BN_N now
    Eigen::Vector3d rotationalDelta_N;//delta in rotationl velocity term of sensor motion in N frame
    Eigen::Vector3d r_SB_N_1;         //sensor positino relative to B frame origin in N frame coordinates previously
    Eigen::Vector3d r_SB_N_2;         //sensor positino relative to B frame origin in N frame coordinates now
    Eigen::Vector3d drDot_BN_N;       //change in velocity of body frame relative to inertial in body frame coordinates between IMU calls. This does not include delta-v from conservative accelerations.
    Eigen::Vector3d dvSensor_B;       //sensor delta v between IMU calls in body frame coordinates
    Eigen::Vector3d dvSensor_N;       //above but in the inertial frame.
    Eigen::Vector3d accumDV_BN_N_1;   // Inertial DV accumulated since t=0 by the body in the inertial frame due to non-conservative forces at time 1
    Eigen::Vector3d accumDV_BN_N_2;   // Inertial DV accumulated since t=0 by the body in the inertial frame due to non-conservative forces at time 1
    Eigen::Matrix3d dcm_NB_1;      // direction cosine matrix from N to B at time 1
    Eigen::Matrix3d dcm_NB_2;      // direction cosine matrix from N to B at time 2
    
    double dt;                  // timestep [s]

    dt = (CurrentTime - PreviousTime)*1E-9;
    
    //Calculate "instantaneous" linear acceleration
    rDotDot_BN_B = this->current_nonConservativeAccelpntB_B;
    r_SB_B = this->sensorPos_B;
    omegaDot_x_r_B = this->current_omegaDot_BN_B.cross(r_SB_B);
    omega_x_r_B = this->current_omega_BN_B.cross(r_SB_B);
    omega_x_omega_x_r_B = this->current_omega_BN_B.cross(omega_x_r_B);
    rotationalTerms = omegaDot_x_r_B + omega_x_omega_x_r_B;
    rDotDot_SN_B = rDotDot_BN_B + rotationalTerms;
    this->accel_SN_P_out = this->dcm_PB * rDotDot_SN_B;
    
    //Calculate time-average cumulative delta v
    dcm_NB_1 = this->previous_sigma_BN.toRotationMatrix();
    dcm_NB_2 = this->current_sigma_BN.toRotationMatrix();
    accumDV_BN_N_1 = dcm_NB_1 * this->previous_TotalAccumDV_BN_B;
    accumDV_BN_N_2 = dcm_NB_2 * this->current_TotalAccumDV_BN_B;
    drDot_BN_N = accumDV_BN_N_2 - accumDV_BN_N_1;
    r_SB_N_1 = dcm_NB_1 * r_SB_B;
    r_SB_N_2 = dcm_NB_2 * r_SB_B;
    omega_BN_N_1 = dcm_NB_1 * this->previous_omega_BN_B;
    omega_BN_N_2 = dcm_NB_2 * this->current_omega_BN_B;
    omega_x_r_prev_N = omega_BN_N_1.cross(r_SB_N_1);
    omega_x_r_N = omega_BN_N_2.cross(r_SB_N_2);
    rotationalDelta_N = omega_x_r_N - omega_x_r_prev_N;
    dvSensor_N = drDot_BN_N + rotationalDelta_N;
    dvSensor_B = dcm_NB_2.transpose() * dvSensor_N;
    this->DV_SN_P_out =this->dcm_PB * dvSensor_B;
    
    return;
}

void ImuSensor::UpdateState(uint64_t CurrentSimNanos)
{
    readInputMessages();

    if(this->NominalReady)
    {
        /* Compute true data */
        this->computePlatformDR();
        this->computePlatformDV(CurrentSimNanos);
        /* Compute sensed data */
		this->computeSensorErrors();
		this->applySensorErrors(CurrentSimNanos);
        this->applySensorDiscretization(CurrentSimNanos);
		this->applySensorSaturation(CurrentSimNanos);
        /* Output sensed data */
        this->writeOutputMessages(CurrentSimNanos);
    }
    
    //record data from the current spacecraft message which is needed for the next IMU call
    this->previous_sigma_BN = this->current_sigma_BN;
    this->previous_omega_BN_B = this->current_omega_BN_B;
    this->previous_TotalAccumDV_BN_B = this->current_TotalAccumDV_BN_B;
    this->PreviousTime = CurrentSimNanos;
    
    this->NominalReady = true;

    return;
}
