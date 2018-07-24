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
#include "utilities/bsk_Print.h"
#include "simFswInterfaceMessages/macroDefinitions.h"


ImuSensor::ImuSensor()
{
    this->CallCounts = 0;
    this->InputStateID = -1;
    this->InputStateMsg = "inertial_state_output";
    this->OutputDataMsg = "imu_meas_data";
    this->numStates = 3;
    this->setBodyToPlatformDCM(0.0, 0.0, 0.0);
    this->OutputBufferCount = 2;
    memset(&this->StatePrevious, 0x0, sizeof(SCPlusStatesSimMsg));
    memset(&this->StateCurrent, 0x0, sizeof(SCPlusStatesSimMsg));
    
    this->errorModelGyro =  GaussMarkov(this->numStates);
    this->errorModelAccel = GaussMarkov(this->numStates);
    
    this->aDisc = Discretize(this->numStates);
    this->oDisc = Discretize(this->numStates);
    
    this->aSat = Saturate(this->numStates);
    this->oSat = Saturate(this->numStates);
    
    this->PreviousTime = 0;
    this->NominalReady = false;
    this->senRotBias.fill(0.0);
    this->senTransBias.fill(0.0);
    memset(&this->sensedValues, 0x0, sizeof(IMUSensorIntMsg));
    memset(&this->trueValues, 0x0, sizeof(IMUSensorIntMsg));
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
    this->accelScale.fill(1.);
    this->gyroScale.fill(1.);
    
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

    this->AMatrixAccel.setIdentity(this->numStates,this->numStates);

	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(this->PMatrixAccel.cols() != this->numStates || this->PMatrixAccel.rows() != this->numStates)
	{
        BSK_PRINT(MSG_ERROR, "Your process noise matrix (PMatrixAccel) is not 3*3. Quitting.");
        return;
	}
	this->errorModelAccel.setNoiseMatrix(this->PMatrixAccel);
	this->errorModelAccel.setRNGSeed(this->RNGSeed);
	this->errorModelAccel.setUpperBounds(this->walkBoundsAccel);

    this->AMatrixGyro.setIdentity(this->numStates, this->numStates);

	//! - Alert the user if the noise matrix was not the right size.  That'd be bad.
	if(this->PMatrixGyro.rows() != this->numStates || this->PMatrixGyro.cols() != this->numStates)
	{
        BSK_PRINT(MSG_ERROR, "Your process noise matrix (PMatrixGyro) is not 3*3. Quitting.");
        return;
	}
	this->errorModelGyro.setNoiseMatrix(this->PMatrixGyro);
	this->errorModelGyro.setRNGSeed(this->RNGSeed);
	this->errorModelGyro.setUpperBounds(this->walkBoundsGyro);
    
    Eigen::MatrixXd oSatBounds;
    oSatBounds.resize(this->numStates, 2);
    oSatBounds(0,0) = -this->senRotMax;
    oSatBounds(0,1) = this->senRotMax;
    oSatBounds(1,0) = -this->senRotMax;
    oSatBounds(1,1) = this->senRotMax;
    oSatBounds(2,0) = -this->senRotMax;
    oSatBounds(2,1) = this->senRotMax;
    this->oSat.setBounds(oSatBounds);
    
    Eigen::MatrixXd aSatBounds;
    aSatBounds.resize(this->numStates, 2);
    aSatBounds(0,0) = -this->senTransMax;
    aSatBounds(0,1) = this->senTransMax;
    aSatBounds(1,0) = -this->senTransMax;
    aSatBounds(1,1) = this->senTransMax;
    aSatBounds(2,0) = -this->senTransMax;
    aSatBounds(2,1) = this->senTransMax;
    this->aSat.setBounds(aSatBounds);

    return;
}

void ImuSensor::CrossInit()
{
    this->InputStateID = SystemMessaging::GetInstance()->subscribeToMessage(this->InputStateMsg,
        sizeof(SCPlusStatesSimMsg), this->moduleID);
    if(this->InputStateID < 0 )
    {
        BSK_PRINT(MSG_WARNING, "Failed to link an imu input message. State: %lld", this->InputStateID);
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

void ImuSensor::setLSBs(double LSBa, double LSBo)
{
    this->aDisc.setLSB(Eigen::Vector3d(LSBa, LSBa, LSBa));
    this->oDisc.setLSB(Eigen::Vector3d(LSBo, LSBo, LSBo));
    return;
    
}

void ImuSensor::setCarryError(bool aCarry, bool oCarry)
{
    this->aDisc.setCarryError(aCarry);
    this->oDisc.setCarryError(oCarry);
    return;
}
void ImuSensor::setRoundDirection(roundDirection_t aRound, roundDirection_t oRound){
    
    this->aDisc.setRoundDirection(aRound);
    this->oDisc.setRoundDirection(oRound);
    
    return;
}

void ImuSensor::applySensorDiscretization(uint64_t CurrentTime)
{

    double dt = (CurrentTime - this->PreviousTime)*1.0E-9;
    
    if(this->aDisc.LSB.any() > 0.0) //If aLSB has been set.
    {
        this->accel_SN_P_out = this->aDisc.discretize(this->accel_SN_P_out);
        this->DV_SN_P_out -= this->aDisc.getDiscretizationErrors() * dt;
    }

    if(this->oDisc.LSB.any() > 0.0) // If oLSB has been set.
    {
        this->omega_PN_P_out = this->oDisc.discretize(this->omega_PN_P_out);
        this->prv_PN_out -= this->oDisc.getDiscretizationErrors() * dt;
    }
    
    return;
}

void ImuSensor::set_oSatBounds(Eigen::MatrixXd oSatBounds){
    this->oSat.setBounds(oSatBounds);
}

void ImuSensor::set_aSatBounds(Eigen::MatrixXd aSatBounds){
    this->aSat.setBounds(aSatBounds);
}

void ImuSensor::scaleTruth()
{
    this->omega_PN_P_out = this->omega_PN_P_out.cwiseProduct(this->gyroScale);
    this->prv_PN_out = this->prv_PN_out.cwiseProduct(this->gyroScale);
    this->accel_SN_P_out = this->accel_SN_P_out.cwiseProduct(this->accelScale);
    this->DV_SN_P_out = this->DV_SN_P_out.cwiseProduct(this->accelScale);
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
	double  dt = (CurrentTime - PreviousTime)*1.0E-9;
    
    Eigen::Vector3d omega_PN_P_in = this->omega_PN_P_out;
    this->omega_PN_P_out = this->oSat.saturate(omega_PN_P_in);
    for (uint64_t i = 0; i < this->numStates; i++){
        if (this->omega_PN_P_out(i) != omega_PN_P_in(i)){
            this->prv_PN_out(i) = this->omega_PN_P_out(i) * dt;
        }
    }
    
    Eigen::Vector3d accel_SN_P_in = this->accel_SN_P_out;
    this->accel_SN_P_out = this->aSat.saturate(accel_SN_P_in);
    for (uint64_t i = 0; i < this->numStates; i++){
        if (this->accel_SN_P_out(i) != accel_SN_P_in(i)){
            this->DV_SN_P_out(i) = this->accel_SN_P_out(i) * dt;
        }
    }
    return;
}

/*This function gathers actual spacecraft attitude from the spacecraftPlus output message.
 It then differences the state attitude between this time and the last time the IMU was called
 to get a DR (delta radians or delta rotation) The angular rate is retrieved directly from the
 spacecraftPlus output message and passed through to theother IMU functions which add noise, etc. */
void ImuSensor::computePlatformDR()
{
    double dcm_P2P1_cArray[9]; //dcm_P2P1 as cArray for C2PRV conversion
    double prv_PN_cArray[3]; //cArray of PRV

    //Calculated time averaged cumulative rotation
    Eigen::Matrix3d dcm_P2P1;  // direction cosine matrix from P at time 1 to P at time 2
    dcm_P2P1 = this->dcm_PB * this->current_sigma_BN.toRotationMatrix().transpose() * (this->dcm_PB * this->previous_sigma_BN.toRotationMatrix().transpose()).transpose();
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
    //Calculate "instantaneous" linear acceleration
    Eigen::Vector3d rDotDot_SN_B;     //sensor non conservative acceleration relative to inertial frame in body frame coordinates
    rDotDot_SN_B = this->current_nonConservativeAccelpntB_B + this->current_omegaDot_BN_B.cross(this->sensorPos_B) + this->current_omega_BN_B.cross(this->current_omega_BN_B.cross(this->sensorPos_B));
    this->accel_SN_P_out = this->dcm_PB * rDotDot_SN_B;
    
    //Calculate time-average cumulative delta v
    Eigen::Matrix3d dcm_NB_1;      // direction cosine matrix from N to B at time 1
    dcm_NB_1 = this->previous_sigma_BN.toRotationMatrix();
    Eigen::Matrix3d dcm_NB_2;      // direction cosine matrix from N to B at time 2
    dcm_NB_2 = this->current_sigma_BN.toRotationMatrix();
    
    this->DV_SN_P_out =this->dcm_PB * dcm_NB_2.transpose() * ((dcm_NB_2 * this->current_TotalAccumDV_BN_B - dcm_NB_1 * this->previous_TotalAccumDV_BN_B) + ((dcm_NB_2 * this->current_omega_BN_B).cross(dcm_NB_2 * this->sensorPos_B) - (dcm_NB_1 * this->previous_omega_BN_B).cross(dcm_NB_1 * this->sensorPos_B)));
    
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
        this->scaleTruth();
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
