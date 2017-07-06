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
#include <math.h>
#include <iostream>
#include <cstring>
#include <random>

//! Initialize a bunch of defaults in the constructor.  Is this the right thing to do?
CoarseSunSensor::CoarseSunSensor()
{
    CallCounts = 0;
    MessagesLinked = false;
    InputSunID = -1;
    InputStateID = -1;
    InputStateMsg = "inertial_state_output";
    InputSunMsg = "sun_planet_data";
    OutputDataMsg = "";
    this->SenBias = 0.0;
    this->SenNoiseStd = 0.0;
    
    this->faultState = MAX_CSSFAULT;
    this->stuckPercent = 0.0;
    v3SetZero(this->nHat_B);
    v3SetZero(this->horizonPlane);
    this->directValue = 0.0;
    this->albedoValue = 0.0;
    this->scaleFactor = 1.0;
    this->KellyFactor = 0.0;
    this->sensedValue = 0.0;
    this->fov           = 1.0471975512;
    this->maxVoltage    = 0.0;
    this->phi           = 0.785398163397;
    this->theta         = 0.0;
    v3SetZero(this->B2P321Angles);
    v3SetZero(this->r_B);
    this->setBodyToPlatformDCM(B2P321Angles[0], B2P321Angles[1], B2P321Angles[2]);
    this->setUnitDirectionVectorWithPerturbation(0, 0);
    this->OutputBufferCount = 2;
    this->sunVisibilityFactor = 1.0;
    
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
    std::normal_distribution<double>::param_type
    UpdatePair(SenBias, SenNoiseStd);
    //! - Configure the random number generator
    rgen.seed(RNGSeed);
    rnum.param(UpdatePair);
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
    LinkMessages();
}

/*! This method determines if the spacecraft is illuminated by the sun*/
bool CoarseSunSensor::spacecraftIlluminated()
{
    return(true); /// Sun is always shining baby.  Fix this...
}

/*! This method links the input messages with the ID matched to the input message 
    and warns the user if any messages can't be found */
bool CoarseSunSensor::LinkMessages()
{
    //! Begin Method Steps
    //! - Subscribe to the Sun ephemeris message and the vehicle state ephemeris
    this->InputSunID = SystemMessaging::GetInstance()->subscribeToMessage(this->InputSunMsg,
        sizeof(SpicePlanetStateSimMsg), moduleID);
    this->InputStateID = SystemMessaging::GetInstance()->subscribeToMessage(this->InputStateMsg,
        sizeof(SCPlusStatesSimMsg), moduleID);
    this->sunEclipseInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->sunEclipseInMsgName,
                                                                      sizeof(EclipseSimMsg), moduleID);

    //! - If both messages are valid, return true, otherwise warnd and return false
    if(InputSunID >= 0 && InputStateID >= 0)
    {
        return(true);
    }
    else
    {
        std::cerr << "WARNING: Failed to link a sun sensor input message: ";
        std::cerr << std::endl << "Sun: "<<InputSunID;
        std::cerr << std::endl << "Sun: "<<InputStateID;
    }
    return(false);
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
    double dcm_BN[3][3];
    
    //! Begin Method Steps
    //! - Get the position from spacecraft to Sun
    v3Scale(-1.0, this->StateCurrent.r_BN_N, Sc2Sun_Inrtl);
    v3Add(Sc2Sun_Inrtl, this->SunData.PositionVector, Sc2Sun_Inrtl);
    //! - Normalize the relative position into a unit vector
    v3Normalize(Sc2Sun_Inrtl, Sc2Sun_Inrtl);
    //! - Get the inertial to body frame transformation information and convert sHat to body frame
    MRP2C(StateCurrent.sigma_BN, dcm_BN);
    m33MultV3(dcm_BN, Sc2Sun_Inrtl, this->sHat_B);
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

    // apply sun visibility (eclipse) information
    this->directValue = this->directValue * this->sunVisibilityFactor;
    
    //! - Albedo is forced to zero for now.
    this->albedoValue = 0.0;
    this->trueValue = this->directValue + this->albedoValue;
}

/*! This method takes the true observed cosine value (directValue) and converts 
    it over to an errored value.  It applies a Kelly curve fit and then noise 
    to the truth. */
void CoarseSunSensor::applySensorErrors()
{
    //! Begin Method Steps
    //! - Get current error from random number generator
    double CurrentError = rnum(this->rgen);
    //! - Apply the kelly fit to the truth direct value
    double KellyFit = 1.0 - exp(-this->directValue * this->directValue/this->KellyFactor);
    //! - Sensed value is total illuminance with a kelly fit + noise
    this->sensedValue = (this->directValue + this->albedoValue)*KellyFit + CurrentError;
    
}

void CoarseSunSensor::scaleSensorValues()
{
    this->sensedValue = this->sensedValue * this->scaleFactor;
    this->trueValue = this->trueValue * this->scaleFactor;
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
    SystemMessaging::GetInstance()->WriteMessage(OutputDataID, Clock, 
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
    //! - Write output data
    this->writeOutputMessages(CurrentSimNanos);
}

/*! The default constructor for the constellation really just clears the 
    sensor list.*/
CSSConstellation::CSSConstellation()
{
    sensorList.clear();
    outputBufferCount = 2;
}

/*! The default destructor for the constellation just clears the sensor list.*/
CSSConstellation::~CSSConstellation()
{
    sensorList.clear();
}

/*! This method loops through the sensor list and calls the self init method for 
    all of them.*/
void CSSConstellation::SelfInit()
{
    std::vector<CoarseSunSensor>::iterator it;
    //! Begin Method Steps
    //! - Loop over the sensor list and initialize all children
    for(it=sensorList.begin(); it!= sensorList.end(); it++)
    {
        it->SelfInit();
    }

    memset(&outputBuffer, 0x0, sizeof(CSSArraySensorIntMsg));
    //! - Create the output message sized to the number of sensors
    outputConstID = SystemMessaging::GetInstance()->
    CreateNewMessage(outputConstellationMessage,
        sizeof(CSSArraySensorIntMsg), outputBufferCount,
        "CSSArraySensorIntMsg", moduleID);
}

/*! This method loops through the sensor list and calls the CrossInit method for 
    all of those sensors.*/
void CSSConstellation::CrossInit()
{
    std::vector<CoarseSunSensor>::iterator it;
    //! Begin Method Steps
    //! - Loop over the sensor list and initialize all children
    for(it=sensorList.begin(); it!= sensorList.end(); it++)
    {
        it->CrossInit();
    }
}

void CSSConstellation::UpdateState(uint64_t CurrentSimNanos)
{
    std::vector<CoarseSunSensor>::iterator it;
    //! Begin Method Steps
    //! - Loop over the sensor list and update all data
    for(it=sensorList.begin(); it!= sensorList.end(); it++)
    {
        it->readInputMessages();
        it->computeSunData();
        it->computeTrueOutput();
        it->applySensorErrors();
        it->scaleSensorValues();
        outputBuffer.CosValue[it - sensorList.begin()] = it->sensedValue;
    }
    SystemMessaging::GetInstance()->WriteMessage(outputConstID, CurrentSimNanos,
                                                 sizeof(CSSArraySensorIntMsg), reinterpret_cast<uint8_t *>(&outputBuffer));
}
