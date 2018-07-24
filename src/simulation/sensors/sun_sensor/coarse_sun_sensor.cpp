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

#include "sensors/sun_sensor/coarse_sun_sensor.h"
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

//! Initialize a bunch of defaults in the constructor.  Is this the right thing to do?
CoarseSunSensor::CoarseSunSensor()
{
//    this->CallCounts = 0;
    this->sunInMsgID = -1;
    this->stateInMsgID = -1;
    this->sunEclipseInMsgId = -1;
    this->stateInMsgName = "inertial_state_output";
    this->sunInMsgName = "sun_planet_data";
    this->cssDataOutMsgName = "";
    this->senBias = 0.0;
    this->senNoiseStd = 0.0;
    this->walkBounds = 1E-15; //don't allow random walk by default
    this->noiseModel = GaussMarkov(1);
    this->faultState = MAX_CSSFAULT;
    this->nHat_B.fill(0.0);
    this->directValue = 0.0;
    this->albedoValue = 0.0;
    this->scaleFactor = 1.0;
    this->kellyFactor = 0.0;
    this->sensedValue = 0.0;
    this->maxOutput = 1e6;
    this->minOutput = 0.0;
    this->saturateUtility = Saturate(1);
    this->fov = 1.0471975512;
    this->phi = 0.785398163397;
    this->theta = 0.0;
    v3SetZero(this->B2P321Angles);
    this->r_B.fill(0.0);
    this->setBodyToPlatformDCM(B2P321Angles[0], B2P321Angles[1], B2P321Angles[2]);
    this->setUnitDirectionVectorWithPerturbation(0, 0);
    this->outputBufferCount = 2;
    this->sunVisibilityFactor.shadowFactor = 1.0;
    this->sunDistanceFactor = 1.0;
    this->dcm_PB.setIdentity(3,3);
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
    //! Begin Method Steps
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
    this->nHat_B = this->dcm_PB * sensorV3_P;
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
    eigenMatrix3d2CArray(this->dcm_PB, dcm_PBcArray);
    Euler3212C(q, RECAST3X3 dcm_PBcArray);
    this->dcm_PB = cArray2EigenMatrix3d(dcm_PBcArray);
}

/*! This method performs all of the internal initialization for the model itself. 
 Primarily that involves initializing the random number generator and creates
 the output message*/
void CoarseSunSensor::SelfInit()
{
    //! Begin Method Steps
    Eigen::VectorXd nMatrix;
    nMatrix.resize(1,1);
    nMatrix(0,0) = this->senNoiseStd*1.5;
    this->noiseModel.setNoiseMatrix(nMatrix);
    Eigen::VectorXd bounds;
    bounds.resize(1,1);
    bounds(0,0) = this->walkBounds;
    this->noiseModel.setUpperBounds(bounds);
    this->noiseModel.setRNGSeed(this->RNGSeed);
    Eigen::VectorXd pMatrix;
    pMatrix.resize(1, 1);
    pMatrix(0,0) = 1.;
    this->noiseModel.setPropMatrix(pMatrix);
    
    Eigen::MatrixXd satBounds;
    satBounds.resize(1, 2);
    satBounds(0,0) = this->minOutput;
    satBounds(0,1) = this->maxOutput;
    this->saturateUtility.setBounds(satBounds);
    
    //! - Create the output message sized to the output message size if the name is valid
    if(this->cssDataOutMsgName != "")
    {
        this->cssDataOutMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage(this->cssDataOutMsgName, sizeof(CSSRawDataSimMsg),
                         this->outputBufferCount, "CSSRawDataSimMsg", this->moduleID);
    }
}

/*! This method simply calls the LinkMessages method to ensure that input messages 
 are matched correctly.*/
void CoarseSunSensor::CrossInit()
{
    //! Begin Method Steps
    //! - Subscribe to the Sun ephemeris message and the vehicle state ephemeris
    this->sunInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->sunInMsgName,
                                                                          sizeof(SpicePlanetStateSimMsg),
                                                                          this->moduleID);
    this->stateInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->stateInMsgName,
                                                                            sizeof(SCPlusStatesSimMsg),
                                                                            this->moduleID);
    
    /* reading in the sun eclipse message is optional.  It only gets used if this message is successfully suscribed.  */
    if (this->sunEclipseInMsgName.length() > 0) {
        this->sunEclipseInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->sunEclipseInMsgName,
                                                                                     sizeof(EclipseSimMsg),
                                                                                     moduleID);
    }
    
    //! - If either messages is not valid, send a warning message
    if(this->sunInMsgID < 0 || this->stateInMsgID < 0) {
        BSK_PRINT(MSG_WARNING, "Failed to link a sun sensor input message: Sun: %lld", this->sunInMsgID);
    }
    return;
}

void CoarseSunSensor::readInputMessages()
{
    SingleMessageHeader localHeader;
    
    //! Begin Method Steps
    
    //! - Zero ephemeris information
    memset(&this->sunData, 0x0, sizeof(SpicePlanetStateSimMsg));
    memset(&this->stateCurrent, 0x0, sizeof(SCPlusStatesSimMsg));
    //! - If we have a valid sun ID, read Sun ephemeris message
    if(this->sunInMsgID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(this->sunInMsgID, &localHeader,
                                                    sizeof(SpicePlanetStateSimMsg),
                                                    reinterpret_cast<uint8_t*> (&this->sunData),
                                                    this->moduleID);
    }
    //! - If we have a valid state ID, read vehicle state ephemeris message
    if(this->stateInMsgID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(this->stateInMsgID, &localHeader,
                                                    sizeof(SCPlusStatesSimMsg),
                                                    reinterpret_cast<uint8_t*> (&this->stateCurrent),
                                                    this->moduleID);
    }
    if(this->sunEclipseInMsgId >= 0) {
        SystemMessaging::GetInstance()->ReadMessage(this->sunEclipseInMsgId, &localHeader,
                                                    sizeof(EclipseSimMsg),
                                                    reinterpret_cast<uint8_t*> (&this->sunVisibilityFactor),
                                                    this->moduleID);
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
    
    //! Begin Method Steps for sun heading vector
    //! - Get the position from spacecraft to Sun
    
    //Read Message data to eigen
    r_BN_N_eigen = cArray2EigenVector3d(this->stateCurrent.r_BN_N);
    sunPos = cArray2EigenVector3d(this->sunData.PositionVector);
    sigma_BN_eigen = cArray2EigenVector3d(this->stateCurrent.sigma_BN);
    
    
    //! - Find sun heading unit vector
    Sc2Sun_Inrtl = sunPos -  r_BN_N_eigen;
    sHat_N = Sc2Sun_Inrtl / Sc2Sun_Inrtl.norm();
    
    //! - Get the inertial to body frame transformation information and convert sHat to body frame
    dcm_BN = sigma_BN_eigen.toRotationMatrix().transpose();
    this->sHat_B = dcm_BN * sHat_N;
    
    //! Begin Method Steps for sun distance factor
    double r_Sun_Sc = Sc2Sun_Inrtl.norm();
    this->sunDistanceFactor = pow(AU*1000., 2.)/pow(r_Sun_Sc, 2.);
    
}

/*! This method computes the tru sensed values for the sensor */
void CoarseSunSensor::computeTrueOutput()
{
    //! Begin Method Steps
    
    this->directValue = this->nHat_B.dot(this->sHat_B) >= cos(this->fov) ? this->nHat_B.dot(this->sHat_B) : 0.0;
    // Define epsilon that will avoid dividing by a very small kelly factor, i.e 0.0.
    double eps = 1e-10;
    //! - Apply the kelly fit to the truth direct value
    double kellyFit = 1.0;
    if (this->kellyFactor > eps) {
        kellyFit -= exp(-this->directValue * this->directValue / this->kellyFactor);
    }
    this->directValue = this->directValue*kellyFit;
    
    // apply sun distance factor (adjust based on flux at current distance from sun)
    // Also apply shadow factor. Basically, correct the intensity of the light.
    this->directValue = this->directValue*this->sunDistanceFactor*this->sunVisibilityFactor.shadowFactor;
    this->trueValue = this->directValue;
    
    //! - Albedo is forced to zero for now. Note that "albedo value" must be the cosine response due to albedo intensity and direction. It can then be stacked on top of the
    //! - sun cosine curve
    //this->albedoValue = 0.0;
    //this->directValue = this->directValue + this->albedoValue
    //this->trueValue = this->directValue
}

/*! This method takes the true observed cosine value (directValue) and converts 
 it over to an errored value.  It applies noise to the truth. */
void CoarseSunSensor::applySensorErrors()
{
    if(this->senNoiseStd <= 0.0){
        this->sensedValue = this->directValue + this->senBias;
        return;
    }
    //! Begin Method Steps
    //! - Get current error from random number generator
    this->noiseModel.computeNextState();
    Eigen::VectorXd currentErrorEigen =  this->noiseModel.getCurrentState();
    double currentError = currentErrorEigen.coeff(0,0);
    
    //Apply saturation values here.
    
    //! - Sensed value is total illuminance with a kelly fit + noise
    this->sensedValue = this->directValue + currentError + this->senBias;
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
    //! Begin Method Steps
    CSSRawDataSimMsg localMessage;
    //! - Zero the output message
    memset(&localMessage, 0x0, sizeof(CSSRawDataSimMsg));
    //! - Set the outgoing data to the scaled computation
    localMessage.OutputData = this->sensedValue;
    //! - Write the outgoing message to the architecture
    SystemMessaging::GetInstance()->WriteMessage(this->cssDataOutMsgID, Clock,
                                                 sizeof(CSSRawDataSimMsg),
                                                 reinterpret_cast<uint8_t *> (&localMessage),
                                                 this->moduleID);
}

/*! This method is called at a specified rate by the architecture.  It makes the 
 calls to compute the current sun information and write the output message for
 the rest of the model.
 @param CurrentSimNanos The current simulation time from the architecture*/
void CoarseSunSensor::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin Method Steps
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
    this->outputBufferCount = 2;
}

/*! The default destructor for the constellation just clears the sensor list.*/
CSSConstellation::~CSSConstellation()
{
    this->sensorList.clear();
}

/*! This method loops through the sensor list and calls the self init method for 
 all of them.*/
void CSSConstellation::SelfInit()
{
    std::vector<CoarseSunSensor>::iterator it;
    //! Begin Method Steps
    //! - Loop over the sensor list and initialize all children
    for(it=this->sensorList.begin(); it!= this->sensorList.end(); it++)
    {
        it->SelfInit();
    }
    
    memset(&this->outputBuffer, 0x0, sizeof(CSSArraySensorIntMsg));
    //! - Create the output message sized to the number of sensors
    outputConstID = SystemMessaging::GetInstance()->CreateNewMessage(outputConstellationMessage,
                     sizeof(CSSArraySensorIntMsg), this->outputBufferCount,
                     "CSSArraySensorIntMsg", this->moduleID);
}

/*! This method loops through the sensor list and calls the CrossInit method for 
 all of those sensors.*/
void CSSConstellation::CrossInit()
{
    std::vector<CoarseSunSensor>::iterator it;
    //! Begin Method Steps
    //! - Loop over the sensor list and initialize all children
    for(it=this->sensorList.begin(); it!= this->sensorList.end(); it++)
    {
        it->CrossInit();
    }
}

void CSSConstellation::UpdateState(uint64_t CurrentSimNanos)
{
    std::vector<CoarseSunSensor>::iterator it;
    //! Begin Method Steps
    //! - Loop over the sensor list and update all data
    for(it=this->sensorList.begin(); it!= this->sensorList.end(); it++)
    {
        it->readInputMessages();
        it->computeSunData();
        it->computeTrueOutput();
        it->applySensorErrors();
        it->scaleSensorValues();
        it->applySaturation();
        this->outputBuffer.CosValue[it - this->sensorList.begin()] = it->sensedValue;
    }
    SystemMessaging::GetInstance()->WriteMessage(outputConstID, CurrentSimNanos,
                                                 sizeof(CSSArraySensorIntMsg),
                                                 reinterpret_cast<uint8_t *>(&outputBuffer));
}

void CSSConstellation::appendCSS(CoarseSunSensor newSensor) {
    sensorList.push_back(newSensor);
    return;
}
