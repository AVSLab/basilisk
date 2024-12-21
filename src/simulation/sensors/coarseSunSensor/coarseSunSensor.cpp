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

#include "simulation/sensors/coarseSunSensor/coarseSunSensor.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include <math.h>
#include <algorithm>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenMRP.h"
#include <inttypes.h>

//! Initialize a bunch of defaults in the constructor.  Is this the right thing to do?
CoarseSunSensor::CoarseSunSensor()
{
//    this->CallCounts = 0;
    this->senBias = 0.0;
    this->senNoiseStd = 0.0;
    this->faultNoiseStd = 0.5;
    this->walkBounds = -1.0; // don't allow random walk by default
    this->noiseModel = GaussMarkov(1, this->RNGSeed);
    this->faultNoiseModel = GaussMarkov(1, this->RNGSeed+1);
    this->faultState = NOMINAL;
    this->nHat_B.fill(0.0);
    this->albedoValue = 0.0;
    this->scaleFactor = 1.0;
    this->kellyFactor = 0.0;
    this->kPower = 2.0;
    this->sensedValue = 0.0;
    this->pastValue = 0.0;
    this->maxOutput = 1e6;
    this->minOutput = 0.0;
    this->saturateUtility = Saturate(1);
    this->fov = 1.0471975512;
    this->phi = 0.785398163397;
    this->theta = 0.0;
    v3SetZero(this->B2P321Angles);
    this->r_B.fill(0.0);
    this->r_PB_B.fill(0.0);
    this->setBodyToPlatformDCM(B2P321Angles[0], B2P321Angles[1], B2P321Angles[2]);
    this->setUnitDirectionVectorWithPerturbation(0, 0);
    this->sunVisibilityFactor = this->sunEclipseInMsg.zeroMsgPayload;
    this->sunVisibilityFactor.shadowFactor = 1.0;
    this->sunDistanceFactor = 1.0;
    this->dcm_PB.setIdentity(3,3);
    this->propagationMatrix.resize(1);
    this->propagationMatrix(0) = 1.0;
    return;
}

//! There is nothing to do in the default destructor
CoarseSunSensor::~CoarseSunSensor()
{
    return;
}

/*!
 * Set the unit direction vector (in the body frame) with applied azimuth and elevation angle perturbations.
 *
 *   @param[in] cssPhiPerturb   [rad] css elevation angle, measured positive toward the body z axis from the x-y plane
 *   @param[in] cssThetaPerturb [rad] css azimuth angle, measured positive from the body +x axis around the +z axis
 */
void CoarseSunSensor::setUnitDirectionVectorWithPerturbation(double cssThetaPerturb, double cssPhiPerturb)
{
    double tempPhi = this->phi + cssPhiPerturb;
    double tempTheta = this->theta + cssThetaPerturb;

    //! - Rotation from individual photo diode sensor frame (S) to css platform frame (P)
    Eigen::Vector3d sensorV3_P; // sensor diode normal in platform frame
    sensorV3_P.fill(0.0);

    /*! azimuth and elevation rotations of vec transpose(1,0,0) where vec is the unit normal
     of the photo diode*/
    sensorV3_P[0] = cos(tempPhi) * cos(tempTheta);
    sensorV3_P[1] = cos(tempPhi) * sin(tempTheta);
    sensorV3_P[2] = sin(tempPhi);

    //! Rotation from P frame to body frame (B)
    this->nHat_B = this->dcm_PB.transpose() * sensorV3_P;
}

/*!
 * Set the direction cosine matrix body to css platform tranformation with 3-2-1 angle set.
 *
 *   @param yaw   (radians) third axis rotation about body +z
 *   @param pitch (radians) second axis rotation about interim frame +y
 *   @param roll  (radians) first axis rotation about platform frame +x
 */
void CoarseSunSensor::setBodyToPlatformDCM(double yaw, double pitch, double roll)
{
    double q[3] = {yaw, pitch, roll};
    double dcm_PBcArray[9];
    Euler3212C(q, RECAST3X3 dcm_PBcArray);
    this->dcm_PB = cArray2EigenMatrix3d(dcm_PBcArray);
}



/*! This method is used to reset the module.
 @param CurrentSimNanos The current simulation time from the architecture
  */
void CoarseSunSensor::Reset(uint64_t CurrentSimNanos)
{
    //! - If either messages is not valid, send a warning message
    if(!this->sunInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CoarseSunSensor: Failed to link a sun sensor input message");
    }
    if(!this->stateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "CoarseSunSensor: Failed to link a spacecraft state input message");
    }

    Eigen::VectorXd nMatrix;
    Eigen::VectorXd pMatrix;
    Eigen::VectorXd bounds;
    nMatrix.resize(1,1);
    pMatrix.resize(1,1);
    bounds.resize(1,1);

    this->noiseModel.setRNGSeed(this->RNGSeed);

    // Only apply noise if user has configured it
    if (this->walkBounds > 0 || this->senNoiseStd > 0) {
        nMatrix(0,0) = this->senNoiseStd;
        this->noiseModel.setNoiseMatrix(nMatrix);

        bounds(0,0) = this->walkBounds;
        this->noiseModel.setUpperBounds(bounds);

        // Only set propagation matrix once
        this->noiseModel.setPropMatrix(this->propagationMatrix);
    }

    // Fault Noise Model
    Eigen::VectorXd nMatrixFault;
    Eigen::VectorXd pMatrixFault;
    Eigen::VectorXd boundsFault;
    nMatrixFault.resize(1,1);
    pMatrixFault.resize(1,1);
    boundsFault.resize(1,1);

    this->faultNoiseModel.setRNGSeed(this->RNGSeed+1);

    nMatrixFault(0,0) = this->faultNoiseStd; // sensor noise standard dev
    this->faultNoiseModel.setNoiseMatrix(nMatrixFault);

    boundsFault(0,0) = 2.0; // walk bounds
    this->faultNoiseModel.setUpperBounds(boundsFault);

    pMatrixFault(0,0) = 1.0; // propagation matrix
    this->faultNoiseModel.setPropMatrix(pMatrixFault);

    Eigen::MatrixXd satBounds;
    satBounds.resize(1, 2);
    satBounds(0,0) = this->minOutput;
    satBounds(0,1) = this->maxOutput;
    this->saturateUtility.setBounds(satBounds);

    // Set up noise model with stored propagation matrix
    this->noiseModel.setPropMatrix(this->propagationMatrix);
    this->faultNoiseModel.setPropMatrix(this->propagationMatrix);
}

void CoarseSunSensor::readInputMessages()
{
    //! - Zero ephemeris information
    this->sunData = this->sunInMsg.zeroMsgPayload;
    this->stateCurrent = this->stateInMsg.zeroMsgPayload;

    //! - If we have a valid sun ID, read Sun ephemeris message
    if(this->sunInMsg.isLinked())
    {
        this->sunData = this->sunInMsg();
    }
    //! - If we have a valid state ID, read vehicle state ephemeris message
    if(this->stateInMsg.isLinked())
    {
        this->stateCurrent = this->stateInMsg();
    }
    //! - If we have a valid eclipse ID, read eclipse message
    if(this->sunEclipseInMsg.isLinked()) {
        this->sunVisibilityFactor = this->sunEclipseInMsg();
    }
    //! - If we have a valid albedo ID, read albedo message
    if (this->albedoInMsg.isLinked()) {
        AlbedoMsgPayload albMsgData;
        albMsgData = this->albedoInMsg();
        this->albedoValue = albMsgData.albedoAtInstrument;
    }
}

/*! This method computes the sun-vector heading information in the vehicle
 body frame.*/
void CoarseSunSensor::computeSunData()
{
    Eigen::Vector3d Sc2Sun_Inrtl;
    Eigen::Vector3d sHat_N;
    Eigen::Matrix3d dcm_BN;

    Eigen::Vector3d r_BN_N_eigen;
    Eigen::Vector3d sunPos;
    Eigen::MRPd sigma_BN_eigen;

    //! - Get the position from spacecraft to Sun

    //! - Read Message data to eigen
    r_BN_N_eigen = cArray2EigenVector3d(this->stateCurrent.r_BN_N);
    sunPos = cArray2EigenVector3d(this->sunData.PositionVector);
    sigma_BN_eigen = cArray2EigenMRPd(this->stateCurrent.sigma_BN);


    //! - Find sun heading unit vector
    Sc2Sun_Inrtl = sunPos -  r_BN_N_eigen;
    sHat_N = Sc2Sun_Inrtl / Sc2Sun_Inrtl.norm();

    //! - Get the inertial to body frame transformation information and convert sHat to body frame
    dcm_BN = sigma_BN_eigen.toRotationMatrix().transpose();
    this->sHat_B = dcm_BN * sHat_N;

    //! - compute sun distance factor
    double r_Sun_Sc = Sc2Sun_Inrtl.norm();
    this->sunDistanceFactor = pow(AU*1000., 2.)/pow(r_Sun_Sc, 2.);

}

/*! This method computes the true sensed values for the sensor */
void CoarseSunSensor::computeTrueOutput()
{
    // If sun heading is within sensor field of view, compute signal
    double signal = this->nHat_B.dot(this->sHat_B);
    this->trueValue = signal >= cos(this->fov) ? signal : 0.0;

    // Define epsilon that will avoid dividing by a very small kelly factor, i.e 0.0.
    double eps = 1e-10;

    //! - Apply the kelly fit to the truth direct value
    double kellyFit;
    if (this->kellyFactor > eps) {
        // (1 - e^{-x^kPower/kFactor}})
        kellyFit = 1.0 - exp(-pow(this->trueValue, this->kPower) / this->kellyFactor);
    } else {
        kellyFit = 1.0;
    }
    this->trueValue *= kellyFit;

    // apply sun distance factor (adjust based on flux at current distance from sun)
    this->trueValue *= this->sunDistanceFactor;

    // Also apply shadow factor. Basically, correct the intensity of the light.
    this->trueValue *= this->sunVisibilityFactor.shadowFactor;

    // Adding albedo value (if defined by the user)
    if (this->albedoValue > 0.0){
        this->trueValue += this->albedoValue;}
}

/*! This method takes the true observed cosine value and converts
 it over to an errored value.  It applies noise to the truth. */
void CoarseSunSensor::applySensorErrors()
{
    double sensorError;
    if(this->senNoiseStd <= 0.0){ // only include sensor bias
        sensorError = this->senBias;
    } else { // include bias and noise
        //! - Get current error from random number generator
        this->noiseModel.computeNextState();
        Eigen::VectorXd currentErrorEigen = this->noiseModel.getCurrentState();
        double sensorNoise = currentErrorEigen.coeff(0,0);
        sensorError = this->senBias + sensorNoise;
    }

    this->sensedValue = this->trueValue + sensorError;

    //Apply faults values here.
    this->faultNoiseModel.computeNextState();

    if(this->faultState == CSSFAULT_OFF){
        this->sensedValue = 0.0;
    } else if (this->faultState == CSSFAULT_STUCK_MAX){
        this->sensedValue = 1.0;
    } else if (this->faultState == CSSFAULT_STUCK_CURRENT){
        this->sensedValue = this->pastValue;
    } else if (this->faultState == CSSFAULT_STUCK_RAND){
        this->sensedValue = this->faultNoiseModel.getCurrentState().coeff(0,0);
        this->faultState = CSSFAULT_STUCK_CURRENT;
    } else if (this->faultState == CSSFAULT_RAND){
        this->sensedValue = this->faultNoiseModel.getCurrentState().coeff(0,0);
    } else { // Nominal

    }

    this->pastValue = this->sensedValue;

}

void CoarseSunSensor::scaleSensorValues()
{
    this->sensedValue = this->sensedValue * this->scaleFactor;
    this->trueValue = this->trueValue * this->scaleFactor;
}

void CoarseSunSensor::applySaturation()
{
    Eigen::VectorXd sensedEigen;
    sensedEigen = cArray2EigenMatrixXd(&this->sensedValue, 1, 1);
    sensedEigen = this->saturateUtility.saturate(sensedEigen);
    eigenMatrixXd2CArray(sensedEigen, &this->sensedValue);
}

/*! This method writes the output message.  The output message contains the
 current output of the CSS converted over to some discrete "counts" to
 emulate ADC conversion of S/C.
 @param Clock The current simulation time*/
void CoarseSunSensor::writeOutputMessages(uint64_t Clock)
{
    if (this->cssDataOutMsg.isLinked()) {
        CSSRawDataMsgPayload localMessage;
        //! - Zero the output message
        localMessage = this->cssDataOutMsg.zeroMsgPayload;
        //! - Set the outgoing data to the scaled computation
        localMessage.OutputData = this->sensedValue;
        //! - Write the outgoing message to the architecture
        this->cssDataOutMsg.write(&localMessage, this->moduleID, Clock);
    }

    // create CSS configuration log message
    if (this->cssConfigLogOutMsg.isLinked()) {
        CSSConfigLogMsgPayload configMsg;
        configMsg = this->cssConfigLogOutMsg.zeroMsgPayload;
        configMsg.fov = this->fov;
        configMsg.signal = this->sensedValue;
        configMsg.minSignal = this->minOutput;
        configMsg.maxSignal = this->maxOutput;
        if (this->CSSGroupID >=0) {
            configMsg.CSSGroupID = this->CSSGroupID;
        }
        eigenVector3d2CArray(this->r_B, configMsg.r_B);
        eigenVector3d2CArray(this->nHat_B, configMsg.nHat_B);

        this->cssConfigLogOutMsg.write(&configMsg, this->moduleID, Clock);
    }
}

/*! This method is called at a specified rate by the architecture.  It makes the
 calls to compute the current sun information and write the output message for
 the rest of the model.
 @param CurrentSimNanos The current simulation time from the architecture*/
void CoarseSunSensor::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the inputs
    this->readInputMessages();
    //! - Get sun vector
    this->computeSunData();
    //! - compute true cosine
    this->computeTrueOutput();
    //! - Apply any set errors
    this->applySensorErrors();
    //! - Fit kelly curve
    this->scaleSensorValues();
    //! - Apply Saturation (floor and ceiling values)
    this->applySaturation();
    //! - Write output data
    this->writeOutputMessages(CurrentSimNanos);
}

/*! The default constructor for the constellation really just clears the
 sensor list.*/
CSSConstellation::CSSConstellation()
{
    this->sensorList.clear();
}

/*! The default destructor for the constellation just clears the sensor list.*/
CSSConstellation::~CSSConstellation()
{
    this->sensorList.clear();
}


/*! This method is used to reset the module.
 @param CurrentSimNanos The current simulation time from the architecture
  */
void CSSConstellation::Reset(uint64_t CurrentSimNanos)
{
    std::vector<CoarseSunSensor*>::iterator itp;
    CoarseSunSensor *it;

    //! - Loop over the sensor list and initialize all children
    for(itp=this->sensorList.begin(); itp!= this->sensorList.end(); itp++)
    {
        it = *itp;
        it->Reset(CurrentSimNanos);
    }

    this->outputBuffer = this->constellationOutMsg.zeroMsgPayload;

}

void CSSConstellation::UpdateState(uint64_t CurrentSimNanos)
{
    std::vector<CoarseSunSensor*>::iterator itp;
    CoarseSunSensor* it;

    //! - Loop over the sensor list and update all data
    for(itp=this->sensorList.begin(); itp!= this->sensorList.end(); itp++)
    {
        it = *itp;
        it->readInputMessages();
        it->computeSunData();
        it->computeTrueOutput();
        it->applySensorErrors();
        it->scaleSensorValues();
        it->applySaturation();
        it->writeOutputMessages(CurrentSimNanos);

        this->outputBuffer.CosValue[itp - this->sensorList.begin()] = it->sensedValue;

    }
    this->outputBuffer.timeTag = (double) (CurrentSimNanos * NANO2SEC);
    this->constellationOutMsg.write(&this->outputBuffer, this->moduleID, CurrentSimNanos);
}

void CSSConstellation::appendCSS(CoarseSunSensor* newSensor) {
    sensorList.push_back(newSensor);
    return;
}

/*!
    Setter for `AMatrix` used for error propagation
    @param propMatrix Matrix to set
*/
void CoarseSunSensor::setAMatrix(const Eigen::Matrix<double, -1, 1, 0, -1, 1>& propMatrix)
{
    if(propMatrix.rows() != 1 || propMatrix.cols() != 1) {
        bskLogger.bskLog(BSK_ERROR, "CoarseSunSensor: Propagation matrix must be 1x1");
        return;
    }
    this->propagationMatrix = propMatrix;
    // Set the propagation matrix for both noise models
    this->noiseModel.setPropMatrix(propMatrix);
    this->faultNoiseModel.setPropMatrix(propMatrix);
}

/*!
    Getter for `AMatrix` used for error propagation
    @return Current matrix
*/
Eigen::Matrix<double, -1, 1, 0, -1, 1> CoarseSunSensor::getAMatrix() const
{
    return this->propagationMatrix;
}
