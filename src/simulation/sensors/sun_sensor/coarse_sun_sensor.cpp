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

//! Initialize a bunch of defaults in the constructor.  Is this the right thing to do?
CoarseSunSensor::CoarseSunSensor()
{
    this->CallCounts = 0;
    this->InputSunID = -1;
    this->InputStateID = -1;
    this->sunEclipseInMsgId = -1;
    this->InputStateMsg = "inertial_state_output";
    this->InputSunMsg = "sun_planet_data";
    this->OutputDataMsg = "";
    this->SenBias = 0.0;
    this->SenNoiseStd = 0.0;
    this->walkBounds = 1E-15; //don't allow random walk by default
    
    this->noiseModel= new GaussMarkov(1);
    this->noiseModel->setRNGSeed(this->RNGSeed);
    Eigen::VectorXd bounds;
    bounds.resize(1, 1);
    bounds(0,0) = this->walkBounds;
    this->noiseModel->setUpperBounds(bounds);
    Eigen::VectorXd pMatrix;
    pMatrix.resize(1, 1);
    pMatrix(0,0) = 1.;
    this->noiseModel->setPropMatrix(pMatrix);
    
    this->faultState = MAX_CSSFAULT;
    v3SetZero(this->nHat_B);
    this->directValue = 0.0;
    this->albedoValue = 0.0;
    this->scaleFactor = 1.0;
    this->KellyFactor = 0.0;
    this->sensedValue = 0.0;
    this->maxOutput   = 1e6;
    this->minOutput   = 0. ;
    this->fov           = 1.0471975512;
    this->phi           = 0.785398163397;
    this->theta         = 0.0;
    v3SetZero(this->B2P321Angles);
    v3SetZero(this->r_B);
    this->setBodyToPlatformDCM(B2P321Angles[0], B2P321Angles[1], B2P321Angles[2]);
    this->setUnitDirectionVectorWithPerturbation(0, 0);
    this->OutputBufferCount = 2;
    this->sunVisibilityFactor.shadowFactor = 1.0;
    this->sunDistanceFactor = 1.0;
    m33SetIdentity(this->dcm_PB);    
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
    double sensorV3_P[3] = {0,0,0}; // sensor diode normal in platform frame
    
    /*! azimuth and elevation rotations of vec transpose(1,0,0) where vec is the unit normal
        of the photo diode*/
    sensorV3_P[0] = cos(tempPhi) * cos(tempTheta);
    sensorV3_P[1] = cos(tempPhi) * sin(tempTheta);
    sensorV3_P[2] = sin(tempPhi);
    
    //! Rotation from P frame to body frame (B)
    m33tMultV3(this->dcm_PB, sensorV3_P, this->nHat_B);
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
    Euler3212C(q, this->dcm_PB);
}

//! There is nothing to do in the default destructor
CoarseSunSensor::~CoarseSunSensor()
{
    return;
}

/*! This method performs all of the internal initialization for the model itself. 
    Primarily that involves initializing the random number generator and creates 
    the output message*/
void CoarseSunSensor::SelfInit()
{
    //! Begin Method Steps
    if(this->SenNoiseStd > 0.0){
        Eigen::VectorXd nMatrix;
        nMatrix.resize(1,1);
        nMatrix(0,0) = this->SenNoiseStd*1.5;
        this->noiseModel->setNoiseMatrix(nMatrix);
        Eigen::VectorXd bounds;
        bounds.resize(1,1);
        bounds(0,0) = this->walkBounds;
        this->noiseModel->setUpperBounds(bounds);
    }
    //! - Create the output message sized to the output message size if the name is valid
    if(OutputDataMsg != "")
    {
        OutputDataID = SystemMessaging::GetInstance()->
            CreateNewMessage(OutputDataMsg, sizeof(CSSRawDataSimMsg),
            OutputBufferCount, "CSSRawDataSimMsg", moduleID);
    }
}

/*! This method simply calls the LinkMessages method to ensure that input messages 
    are matched correctly.*/
void CoarseSunSensor::CrossInit()
{
    //! Begin Method Steps
    //! - Subscribe to the Sun ephemeris message and the vehicle state ephemeris
    this->InputSunID = SystemMessaging::GetInstance()->subscribeToMessage(this->InputSunMsg,
                                                                          sizeof(SpicePlanetStateSimMsg), moduleID);
    this->InputStateID = SystemMessaging::GetInstance()->subscribeToMessage(this->InputStateMsg,
                                                                            sizeof(SCPlusStatesSimMsg), moduleID);

    /* reading in the sun eclipse message is optional.  It only gets used if this message is successfully suscribed.  */
    if (this->sunEclipseInMsgName.length() > 0) {
        this->sunEclipseInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->sunEclipseInMsgName,
                                                                                     sizeof(EclipseSimMsg), moduleID);
    }

    //! - If either messages is not valid, send a warning message
    if(this->InputSunID < 0 || this->InputStateID < 0) {
        std::cerr << "WARNING: Failed to link a sun sensor input message: ";
        std::cerr << std::endl << "Sun: "<<this->InputSunID;
        std::cerr << std::endl << "Sun: "<<this->InputStateID;
    }
    return;
}


void CoarseSunSensor::readInputMessages()
{
    SingleMessageHeader LocalHeader;
    
    //! Begin Method Steps
    
    //! - Zero ephemeris information
    memset(&SunData, 0x0, sizeof(SpicePlanetStateSimMsg));
    memset(&StateCurrent, 0x0, sizeof(SCPlusStatesSimMsg));
    //! - If we have a valid sun ID, read Sun ephemeris message
    if(InputSunID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(this->InputSunID, &LocalHeader,
                                                    sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*> (&this->SunData), moduleID);
    }
    //! - If we have a valid state ID, read vehicle state ephemeris message
    if(InputStateID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(this->InputStateID, &LocalHeader,
                                                    sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*> (&this->StateCurrent), moduleID);
    }
    if(this->sunEclipseInMsgId >= 0) {
        SystemMessaging::GetInstance()->ReadMessage(this->sunEclipseInMsgId, &LocalHeader,
                                                    sizeof(EclipseSimMsg), reinterpret_cast<uint8_t*> (&this->sunVisibilityFactor), moduleID);
    }
}

/*! This method computes the sun-vector heading information in the vehicle 
    body frame.*/
void CoarseSunSensor::computeSunData()
{
    double Sc2Sun_Inrtl[3];
    double sHat_N[3];
    double dcm_BN[3][3];
    double r_Sun_Sc; //position vector from s/c to sun
    
    //! Begin Method Steps for sun heading vector
    //! - Get the position from spacecraft to Sun
    v3Scale(-1.0, this->StateCurrent.r_BN_N, Sc2Sun_Inrtl);
    v3Add(Sc2Sun_Inrtl, this->SunData.PositionVector, Sc2Sun_Inrtl);
    //! - Normalize the relative position into a unit vector
    v3Normalize(Sc2Sun_Inrtl, sHat_N);
    //! - Get the inertial to body frame transformation information and convert sHat to body frame
    MRP2C(this->StateCurrent.sigma_BN, dcm_BN);
    m33MultV3(dcm_BN, sHat_N, this->sHat_B);
    
    //! Begin Method Steps for sun distance factor
    r_Sun_Sc = v3Norm(Sc2Sun_Inrtl);
    this->sunDistanceFactor = pow(AU*1000., 2.)/pow(r_Sun_Sc, 2.);
    
}

/*! This method computes the tru sensed values for the sensor */
void CoarseSunSensor::computeTrueOutput()
{
    //! Begin Method Steps
    double temp1 = v3Dot(this->nHat_B, this->sHat_B);
    //! - Get dot product of the CSS normal and the sun vector
    this->directValue = 0.0;
    //! - If the dot product is within the simulated field of view, set direct value to it
    if(temp1 >= cos(this->fov))
    {
       this->directValue = temp1;
    }
    
    // Define epsilon that will avoid dividing by a very small kelly factor, i.e 0.0.
    double eps = 1e-10;
    //! - Apply the kelly fit to the truth direct value
    double KellyFit = 1.0;
    if (this->KellyFactor > eps) {
        KellyFit -= exp(-this->directValue * this->directValue/this->KellyFactor);
    }
    this->directValue = this->directValue*KellyFit;
    
    // apply sun distance factor (adjust based on flux at current distance from sun)
    // Also apply shadow factor. Basically, correct the intensity of the light.
    this->directValue = this->directValue*this->sunDistanceFactor*this->sunVisibilityFactor.shadowFactor;
    this->trueValue = this->directValue; //keep for now. not sure what it does, really. -SJKC
    
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
    if(this->SenNoiseStd <= 0.0){
        this->sensedValue = this->directValue + this->SenBias;
        return;
    }
    //! Begin Method Steps
    //! - Get current error from random number generator
    this->noiseModel->computeNextState();
    Eigen::VectorXd CurrentErrorEigen =  this->noiseModel->getCurrentState();
    double CurrentError = CurrentErrorEigen.coeff(0,0);
    
    //Apply saturation values here.

    //! - Sensed value is total illuminance with a kelly fit + noise
    this->sensedValue = this->directValue + CurrentError + this->SenBias;
}

void CoarseSunSensor::scaleSensorValues()
{
    this->sensedValue = this->sensedValue * this->scaleFactor;
    this->trueValue = this->trueValue * this->scaleFactor;
}

void CoarseSunSensor::applySaturation()
{
    this->sensedValue = std::min(this->maxOutput, this->sensedValue);
    this->sensedValue = std::max(this->minOutput, this->sensedValue);
}
/*! This method writes the output message.  The output message contains the 
    current output of the CSS converted over to some discrete "counts" to 
    emulate ADC conversion of S/C.
    @param Clock The current simulation time*/
void CoarseSunSensor::writeOutputMessages(uint64_t Clock)
{
    //! Begin Method Steps
    CSSRawDataSimMsg LocalMessage;
    //! - Zero the output message
    memset(&LocalMessage, 0x0, sizeof(CSSRawDataSimMsg));
    //! - Set the outgoing data to the scaled computation
    LocalMessage.OutputData = this->sensedValue;
    //! - Write the outgoing message to the architecture
    SystemMessaging::GetInstance()->WriteMessage(this->OutputDataID, Clock,
                                                 sizeof(CSSRawDataSimMsg), reinterpret_cast<uint8_t *> (&LocalMessage), moduleID);
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
    outputConstID = SystemMessaging::GetInstance()->
    CreateNewMessage(outputConstellationMessage,
        sizeof(CSSArraySensorIntMsg), this->outputBufferCount,
        "CSSArraySensorIntMsg", moduleID);
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
                                                 sizeof(CSSArraySensorIntMsg), reinterpret_cast<uint8_t *>(&outputBuffer));
}
