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
#include "simulation/sensors/imuSensor/imuSensor.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <cstring>
#include "architecture/utilities/gauss_markov.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include <inttypes.h>


ImuSensor::ImuSensor()
{
    this->numStates = 3;
    this->setBodyToPlatformDCM(0.0, 0.0, 0.0);
    this->OutputBufferCount = 2;
    this->StatePrevious = this->scStateInMsg.zeroMsgPayload;
    this->StateCurrent = this->scStateInMsg.zeroMsgPayload;

    this->errorModelGyro =  GaussMarkov(this->numStates, this->RNGSeed);
    this->errorModelAccel = GaussMarkov(this->numStates, this->RNGSeed);

    this->aDisc = Discretize((uint8_t) this->numStates);
    this->oDisc = Discretize((uint8_t) this->numStates);

    this->aSat = Saturate(this->numStates);
    this->oSat = Saturate(this->numStates);

    this->PreviousTime = 0;
    this->NominalReady = false;
    this->senRotBias.fill(0.0);
    this->senTransBias.fill(0.0);
    this->sensedValues = this->sensorOutMsg.zeroMsgPayload;
    this->trueValues = this->sensorOutMsg.zeroMsgPayload;
    this->senRotMax = 1e6;
    this->senTransMax = 1e6;
    this->PMatrixGyro.fill(0.0);
    this->AMatrixGyro.setIdentity();
    this->PMatrixAccel.fill(0.0);
    this->AMatrixAccel.fill(0.0);
    this->walkBoundsGyro.fill(-1.0);  // Default to -1 to detect if user sets it
    this->walkBoundsAccel.fill(-1.0); // Default to -1 to detect if user sets it
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
    this->sensorPos_B.fill(0.0);
    this->dcm_PB.setIdentity();
    this->errorBoundsGyro.fill(0.0);  // Default to no noise
    this->errorBoundsAccel.fill(0.0); // Default to no noise

    return;
}

/*!
    set body orientation DCM relative to platform
 */
void ImuSensor::setBodyToPlatformDCM(double yaw, double pitch, double roll)
{
    this->dcm_PB = eigenM1(roll)*eigenM2(pitch)*eigenM3(yaw);

    return;
}

ImuSensor::~ImuSensor()
{
    return;
}


/*! Reset the module

 @param CurrentSimNanos current time (ns)
 */
void ImuSensor::Reset(uint64_t CurrentSimNanos)
{
    // check if input message has not been included
    if (!this->scStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "imuSensor.scStateInMsg was not linked.");
    }

    //! - Alert the user if the noise matrix was not the right size.  That'd be bad.
    if(this->PMatrixAccel.cols() != this->numStates || this->PMatrixAccel.rows() != this->numStates)
    {
        bskLogger.bskLog(BSK_ERROR, "Your process noise matrix (PMatrixAccel) is not 3*3. Quitting.");
        return;
    }
    this->errorModelAccel.setNoiseMatrix(this->PMatrixAccel);
    this->errorModelAccel.setRNGSeed(this->RNGSeed);
    this->errorModelAccel.setUpperBounds(this->walkBoundsAccel);

    //! - Alert the user if the noise matrix was not the right size.  That'd be bad.
    if(this->PMatrixGyro.rows() != this->numStates || this->PMatrixGyro.cols() != this->numStates)
    {
        bskLogger.bskLog(BSK_ERROR, "Your process noise matrix (PMatrixGyro) is not 3*3. Quitting.");
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

    // Check if user set deprecated walkBounds
    if(this->walkBoundsAccel(0) != -1.0 || this->walkBoundsAccel(1) != -1.0 || this->walkBoundsAccel(2) != -1.0) {
        bskLogger.bskLog(BSK_WARNING, "ImuSensor: walkBoundsAccel is deprecated. Please use setErrorBoundsAccel() instead.");
        this->setErrorBoundsAccel(this->walkBoundsAccel);
    }

    if(this->walkBoundsGyro(0) != -1.0 || this->walkBoundsGyro(1) != -1.0 || this->walkBoundsGyro(2) != -1.0) {
        bskLogger.bskLog(BSK_WARNING, "ImuSensor: walkBoundsGyro is deprecated. Please use setErrorBoundsGyro() instead.");
        this->setErrorBoundsGyro(this->walkBoundsGyro);
    }

    return;
}


/*!
    read input messages
 */
void ImuSensor::readInputMessages()
{
    this->StateCurrent = this->scStateInMsg.zeroMsgPayload;
    if(this->scStateInMsg.isLinked())
    {
        this->StateCurrent = this->scStateInMsg();
    }
    this->current_sigma_BN = cArray2EigenVector3d(this->StateCurrent.sigma_BN);
    this->current_omega_BN_B = cArray2EigenVector3d(this->StateCurrent.omega_BN_B);
    this->current_nonConservativeAccelpntB_B = cArray2EigenVector3d(this->StateCurrent.nonConservativeAccelpntB_B);
    this->current_omegaDot_BN_B = cArray2EigenVector3d(this->StateCurrent.omegaDot_BN_B);
    this->current_TotalAccumDV_BN_B = cArray2EigenVector3d(this->StateCurrent.TotalAccumDV_BN_B);

    return;
}

/*!
    write output messages
 */
void ImuSensor::writeOutputMessages(uint64_t Clock)
{
    IMUSensorMsgPayload localOutput;

    eigenVector3d2CArray(this->accel_SN_P_out, localOutput.AccelPlatform);
    eigenVector3d2CArray(this->DV_SN_P_out, localOutput.DVFramePlatform);
    eigenVector3d2CArray(this->omega_PN_P_out, localOutput.AngVelPlatform);
    eigenVector3d2CArray(this->prv_PN_out, localOutput.DRFramePlatform);

    this->sensorOutMsg.write(&localOutput, this->moduleID, Clock);

    return;
}

/*!
    set LSB values
 */
void ImuSensor::setLSBs(double LSBa, double LSBo)
{
    this->aDisc.setLSB(Eigen::Vector3d(LSBa, LSBa, LSBa));
    this->oDisc.setLSB(Eigen::Vector3d(LSBo, LSBo, LSBo));
    return;

}

/*!
    set Carry error value
 */
void ImuSensor::setCarryError(bool aCarry, bool oCarry)
{
    this->aDisc.setCarryError(aCarry);
    this->oDisc.setCarryError(oCarry);
    return;
}

/*!
    set round direction value
 */
void ImuSensor::setRoundDirection(roundDirection_t aRound, roundDirection_t oRound){

    this->aDisc.setRoundDirection(aRound);
    this->oDisc.setRoundDirection(oRound);

    return;
}

/*!
    apply sensor direction
    @param CurrentTime
 */
void ImuSensor::applySensorDiscretization(uint64_t CurrentTime)
{

    double dt = (CurrentTime - this->PreviousTime)*1.0E-9;

    if(this->aDisc.LSB.any()) //If aLSB has been set.
    {
        this->accel_SN_P_out = this->aDisc.discretize(this->accel_SN_P_out);
        this->DV_SN_P_out -= this->aDisc.getDiscretizationErrors() * dt;
    }

    if(this->oDisc.LSB.any()) // If oLSB has been set.
    {
        this->omega_PN_P_out = this->oDisc.discretize(this->omega_PN_P_out);
        this->prv_PN_out -= this->oDisc.getDiscretizationErrors() * dt;
    }

    return;
}

/*!
    set o saturation bounds
    @param oSatBounds
 */
void ImuSensor::set_oSatBounds(Eigen::MatrixXd oSatBounds){
    this->oSat.setBounds(oSatBounds);
}

/*!
    set a saturation bounds
    @param aSatBounds
 */
void ImuSensor::set_aSatBounds(Eigen::MatrixXd aSatBounds){
    this->aSat.setBounds(aSatBounds);
}

/*!
    scale truth method
 */
void ImuSensor::scaleTruth()
{
    this->omega_PN_P_out = this->omega_PN_P_out.cwiseProduct(this->gyroScale);
    this->prv_PN_out = this->prv_PN_out.cwiseProduct(this->gyroScale);
    this->accel_SN_P_out = this->accel_SN_P_out.cwiseProduct(this->accelScale);
    this->DV_SN_P_out = this->DV_SN_P_out.cwiseProduct(this->accelScale);
    return;
}

/*!
    apply sensor errors
 */
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

/*!
 compute sensor errors
 */
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

/*!
    apply sensor saturation
 */
void ImuSensor::applySensorSaturation(uint64_t CurrentTime)
{
	double  dt = (CurrentTime - PreviousTime)*1.0E-9;

    Eigen::Vector3d omega_PN_P_in = this->omega_PN_P_out;
    this->omega_PN_P_out = this->oSat.saturate(omega_PN_P_in);
    for (int64_t i = 0; i < this->numStates; i++){
        if (this->omega_PN_P_out(i) != omega_PN_P_in(i)){
            this->prv_PN_out(i) = this->omega_PN_P_out(i) * dt;
        }
    }

    Eigen::Vector3d accel_SN_P_in = this->accel_SN_P_out;
    this->accel_SN_P_out = this->aSat.saturate(accel_SN_P_in);
    for (int64_t i = 0; i < this->numStates; i++){
        if (this->accel_SN_P_out(i) != accel_SN_P_in(i)){
            this->DV_SN_P_out(i) = this->accel_SN_P_out(i) * dt;
        }
    }
    return;
}

/*!
    This function gathers actual spacecraft attitude from the spacecraft output message.
    It then differences the state attitude between this time and the last time the IMU was called
    to get a DR (delta radians or delta rotation) The angular rate is retrieved directly from the
    spacecraft output message and passed through to theother IMU functions which add noise, etc.
 */
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

/*!
    This functions gathers actual spacecraft velocity from the spacecraft output message.
    It then differences the velocity between this time and the last time the IMU was called to get a
    DV (delta velocity). The acceleration of the spacecraft in the body frame is gathered directly from the spacecraft
    output message. Then, it is converted to the platform frame and rotational terms are added to it
    to account for CoM offset of the platform frame.
    @param CurrentTime
 */
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

/*!
    update module states
    @param CurrentSimNanos current time (ns)
 */
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

/*!
    Setter for `AMatrixAccel`
    @param propMatrix Matrix to set
*/
void ImuSensor::setAMatrixAccel(const Eigen::MatrixXd& propMatrix)
{
    this->AMatrixAccel = propMatrix;
    this->errorModelAccel.setPropMatrix(propMatrix);
}

/*!
    Setter for `AMatrixGyro`
    @param propMatrix Matrix to set
*/
void ImuSensor::setAMatrixGyro(const Eigen::MatrixXd& propMatrix)
{
    this->AMatrixGyro = propMatrix;
    this->errorModelGyro.setPropMatrix(propMatrix);
}

/*!
    Getter for `AMatrixAccel`
    @return Current matrix
*/
Eigen::MatrixXd ImuSensor::getAMatrixAccel() const
{
    return this->AMatrixAccel;
}

/*!
    Getter for `AMatrixGyro`
    @return Current matrix
*/
Eigen::MatrixXd ImuSensor::getAMatrixGyro() const
{
    return this->AMatrixGyro;
}

/*!
    Setter for `walkBoundsAccel`
    @param bounds Bounds vector to set
*/
void ImuSensor::setWalkBoundsAccel(const Eigen::Vector3d& bounds)
{
    this->walkBoundsAccel = bounds;
    this->errorModelAccel.setUpperBounds(bounds);
}

/*!
    Setter for `walkBoundsGyro`
    @param bounds Bounds vector to set
*/
void ImuSensor::setWalkBoundsGyro(const Eigen::Vector3d& bounds)
{
    this->walkBoundsGyro = bounds;
    this->errorModelGyro.setUpperBounds(bounds);
}

/*!
    Getter for `walkBoundsAccel`
    @return Current bounds
*/
Eigen::Vector3d ImuSensor::getWalkBoundsAccel() const
{
    return this->walkBoundsAccel;
}

/*!
    Getter for `walkBoundsGyro`
    @return Current bounds
*/
Eigen::Vector3d ImuSensor::getWalkBoundsGyro() const
{
    return this->walkBoundsGyro;
}

/*!
    Setter for `errorBoundsAccel`
    @param bounds Bounds vector to set
*/
void ImuSensor::setErrorBoundsAccel(const Eigen::Vector3d& bounds)
{
    this->errorBoundsAccel = bounds;
    this->errorModelAccel.setUpperBounds(bounds);
}

/*!
    Setter for `errorBoundsGyro`
    @param bounds Bounds vector to set
*/
void ImuSensor::setErrorBoundsGyro(const Eigen::Vector3d& bounds)
{
    this->errorBoundsGyro = bounds;
    this->errorModelGyro.setUpperBounds(bounds);
}

/*!
    Getter for `errorBoundsAccel`
    @return Current bounds
*/
Eigen::Vector3d ImuSensor::getErrorBoundsAccel() const
{
    return this->errorBoundsAccel;
}

/*!
    Getter for `errorBoundsGyro`
    @return Current bounds
*/
Eigen::Vector3d ImuSensor::getErrorBoundsGyro() const
{
    return this->errorBoundsGyro;
}
