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

#ifndef COARSE_SUN_SENSOR_H
#define COARSE_SUN_SENSOR_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/cssRawDataSimMsg.h"
#include "simMessages/eclipseSimMsg.h"
#include "simFswInterfaceMessages/cssArraySensorIntMsg.h"
#include "utilities/gauss_markov.h"
#include "utilities/saturate.h"
#include <Eigen/Dense>

typedef enum {
    CSSFAULT_OFF,           /*!< CSS measurement is set to 0 for all future time
                             */
    CSSFAULT_STUCK_CURRENT, /*!< CSS measurement is set to current value for all future time */
    CSSFAULT_STUCK_MAX,     /*!< CSS measurement is set to maximum value for all future time */
    CSSFAULT_STUCK_RAND,    /*!< CSS measurement is set to randomly selected value for all future time */
    CSSFAULT_STUCK,         /*!< CSS measurement is set to percent value for all future time */
    CSSFAULT_RAND,          /*!< CSS measurement returns uniformly distributed random values between 0 and max */
    MAX_CSSFAULT
} CSSFaultState_t;

/*! \addtogroup SimModelGroup
 * @{
 */


//!@brief Coarse sun sensor model
/*! This class is designed to model the state of a single coarse sun sensor 
attached to a spacecraft.  It emulates the "counts" that will typically be 
output by the ADC on board of a spacecraft.

 The module
 [PDF Description](Basilisk-CoarseSunSensor-20170803.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 The corruption types are outlined in this
 [PDF document](BasiliskCorruptions.pdf).
 */
class CoarseSunSensor: public SysModel {
public:
    CoarseSunSensor();
    ~CoarseSunSensor();
    
    void CrossInit(); //!< @brief method for initializing cross dependencies
    void SelfInit();  //!< @brief method for initializing own messages
    void UpdateState(uint64_t CurrentSimNanos); //!< @brief method to update state for runtime
    void setUnitDirectionVectorWithPerturbation(double cssThetaPerturb, double cssPhiPerturb); //!< @brief utility method to perturb CSS unit vector
    void setBodyToPlatformDCM(double yaw, double pitch, double roll); //!< @brief utility method to configure the platform DCM
    void readInputMessages(); //!< @brief method to read the input messages
    void computeSunData(); //!< @brief method to get the sun vector information
    void computeTrueOutput(); //!< @brief method to compute the true sun-fraction of CSS
    void applySensorErrors(); //!< @brief method to set the actual output of the sensor with scaling/kelly
    void scaleSensorValues();
    void applySaturation();     //!< apply saturation effects to sensed output (floor and ceiling)
    void writeOutputMessages(uint64_t Clock); //!< @brief method to write the output message to the system
    
public:
    std::string sunInMsgName;                    //!< [-] Message name for sun data
    std::string stateInMsgName;                  //!< [-] Message name for spacecraft state */
    std::string cssDataOutMsgName;                  //!< [-] Message name for CSS output data */
    std::string sunEclipseInMsgName;            //!< [-] Message name for sun eclipse state message
    CSSFaultState_t     faultState;             //!< [-] Specification used if state is set to COMPONENT_FAULT */
    double              theta;                  //!< [rad] css azimuth angle, measured positive from the body +x axis around the +z axis
    double              phi;                    //!< [rad] css elevation angle, measured positive toward the body +z axis from the x-y plane
    double              B2P321Angles[3];        //!< [-] 321 Euler anhles for body to platform
    Eigen::Matrix3d     dcm_PB;           //!< [-] DCM from platform frame P to body frame B
    Eigen::Vector3d     nHat_B;              //!< [-] css unit direction vector in body frame components
    Eigen::Vector3d     sHat_B;              //!< [-] unit vector to sun in B
    double              directValue;            //!< [-] direct solar irradiance measurement
    double              albedoValue;            //!< [-] albedo irradiance measurement
    double              scaleFactor;            //!< [-] scale factor applied to sensor (common + individual multipliers)
    double              sensedValue;            //!< [-] total measurement including perturbations
    double              trueValue;              //!< [-] total measurement without perturbations
    double              kellyFactor;            //!< [-] Kelly curve fit for output cosine curve
    double              fov;                    //!< [-] rad, field of view half angle
    Eigen::Vector3d     r_B;
    double              senBias;                //!< [-] Sensor bias value
    double              senNoiseStd;            //!< [-] Sensor noise value
    uint64_t            outputBufferCount;      //!< [-] number of output msgs stored
    double              maxOutput;              //!< [-] maximum output (ceiling) for saturation application
    double              minOutput;              //!< [-] minimum output (floor) for saturation application
    double              walkBounds;             //!< [-] Gauss Markov walk bounds

private:
    int64_t sunInMsgID;                         //!< [-] Connect to input time message
    int64_t stateInMsgID;                       //!< [-] Connect to input time message
    int64_t cssDataOutMsgID;                       //!< [-] Connect to output CSS data
    int64_t sunEclipseInMsgId;                  //!< [-] Connect to input sun eclipse message
    SpicePlanetStateSimMsg sunData;            //!< [-] Unused for now, but including it for future
    SCPlusStatesSimMsg stateCurrent;           //!< [-] Current SSBI-relative state
    EclipseSimMsg sunVisibilityFactor;          //!< [-] scaling parameter from 0 (fully obscured) to 1 (fully visible)
    double              sunDistanceFactor;      //! [-] Factor to scale cosine curve magnitude based on solar flux at location
    GaussMarkov noiseModel;                    //! [-] Gauss Markov noise generation model
    Saturate saturateUtility;                  //! [-] Saturation utility
};

//!@brief Constellation of coarse sun sensors for aggregating output information
/*! This class is a thin container on top of the above coarse-sun sensor class.  
It is used to aggregate the output messages of the coarse sun-sensors into a 
a single output for use by downstream models.*/
class CSSConstellation: public SysModel {
 public:
    CSSConstellation();                         //!< @brief [-] Default constructor
    ~CSSConstellation();                        //!< @brief [-] Default Destructor
    void CrossInit();                           //!< @brief [-] Method for initializing cross dependencies
    void SelfInit();                            //!< @brief [-] Method for initializing own messages
    void UpdateState(uint64_t CurrentSimNanos); //!< @brief [-] Main update method for CSS constellation
    void appendCSS(CoarseSunSensor newSensor); //!< @brief [-] Method for adding sensor to list
    
 public:
    uint64_t outputBufferCount;                  //!< [-] Number of messages archived in output data
    std::string outputConstellationMessage;      //!< [-] Message name for the outgoing message
    int64_t outputConstID;                       //!< [-] output ID for the outgoing message
    std::vector<CoarseSunSensor> sensorList;     //!< [-] List of coarse sun sensors in constellation
 private:
    CSSArraySensorIntMsg outputBuffer;             //!< [-] buffer used to write output message
};

/*! @} */

#endif
