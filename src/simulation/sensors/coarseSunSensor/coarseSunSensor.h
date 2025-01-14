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
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefCpp/CSSConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSRawDataMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
#include "architecture/msgPayloadDefC/AlbedoMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSArraySensorMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/gauss_markov.h"
#include "architecture/utilities/saturate.h"
#include "architecture/utilities/bskLogging.h"

#include <Eigen/Dense>

typedef enum {
    CSSFAULT_OFF,           /*!< CSS measurement is set to 0 for all future time */
    CSSFAULT_STUCK_CURRENT, /*!< CSS measurement is set to current value for all future time */
    CSSFAULT_STUCK_MAX,     /*!< CSS measurement is set to maximum value for all future time */
    CSSFAULT_STUCK_RAND,    /*!< CSS measurement is set to randomly selected value for all future time */
    CSSFAULT_RAND,          /*!< CSS measurement returns uniformly distributed random values between 0 and max */
    NOMINAL
} CSSFaultState_t;

/*! @class CoarseSunSensor
 * @brief Coarse sun sensor model that simulates sun vector measurements with configurable noise
 *
 * The CSS supports noise configuration through:
 * - Walk bounds: Maximum allowed deviations from truth [-]
 * - Noise std: Standard deviation of measurement noise [-]
 * - AMatrix: Propagation matrix for error model (defaults to identity)
 * - Fault noise: Additional noise when in fault state [-]
 *
 * Example Python usage:
 * @code
 *     cssSensor = CoarseSunSensor()
 *
 *     # Configure noise (dimensionless)
 *     cssSensor.senNoiseStd = 0.001      # Standard deviation
 *     cssSensor.walkBounds = 0.01        # Maximum error bound
 *
 *     # Optional: Configure error propagation (default is identity)
 *     cssSensor.setAMatrix([[1]])        # 1x1 matrix for scalar measurement
 *
 *     # Optional: Configure fault noise
 *     cssSensor.faultNoiseStd = 0.5      # Noise when in fault state
 * @endcode
 */
class CoarseSunSensor: public SysModel {
public:
    CoarseSunSensor();
    ~CoarseSunSensor();

    void Reset(uint64_t CurrentClock);          //!< Method for reseting the module
    void UpdateState(uint64_t CurrentSimNanos); //!< @brief method to update state for runtime
    void setUnitDirectionVectorWithPerturbation(double cssThetaPerturb, double cssPhiPerturb); //!< @brief utility method to perturb CSS unit vector
    void setBodyToPlatformDCM(double yaw, double pitch, double roll); //!< @brief utility method to configure the platform DCM
    void readInputMessages(); //!< @brief method to read the input messages
    void computeSunData(); //!< @brief method to get the sun vector information
    void computeTrueOutput(); //!< @brief method to compute the true sun-fraction of CSS
    void applySensorErrors(); //!< @brief method to set the actual output of the sensor with scaling/kelly
    void scaleSensorValues();  //!< scale the sensor values
    void applySaturation();     //!< apply saturation effects to sensed output (floor and ceiling)
    void writeOutputMessages(uint64_t Clock); //!< @brief method to write the output message to the system

public:
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg; //!< [-] input message for sun data
    ReadFunctor<SCStatesMsgPayload> stateInMsg;   //!< [-] input message for spacecraft state
    Message<CSSRawDataMsgPayload> cssDataOutMsg;      //!< [-] output message for CSS output data
    Message<CSSConfigLogMsgPayload> cssConfigLogOutMsg;  //!< [-] output message for CSS configuration log data
    ReadFunctor<EclipseMsgPayload> sunEclipseInMsg;   //!< [-] (optional) input message for sun eclipse state message
    ReadFunctor<AlbedoMsgPayload> albedoInMsg;        //!< [-] (optional) input message for albedo message

    CSSFaultState_t     faultState;             //!< [-] Specification used if state is set to COMPONENT_FAULT */
    double              theta;                  //!< [rad] css azimuth angle, measured positive from the body +x axis around the +z axis
    double              phi;                    //!< [rad] css elevation angle, measured positive toward the body +z axis from the x-y plane
    double              B2P321Angles[3];        //!< [-] 321 Euler angles for body to platform
    Eigen::Matrix3d     dcm_PB;                 //!< [-] DCM of platform frame P relative to body frame B
    Eigen::Vector3d     nHat_B;                 //!< [-] css unit direction vector in body frame components
    Eigen::Vector3d     sHat_B;                 //!< [-] unit vector to sun in B
    double              albedoValue = -1.0;     //!< [-] albedo irradiance measurement
    double              scaleFactor;            //!< [-] scale factor applied to sensor (common + individual multipliers)
    double              pastValue;              //!< [-] measurement from last update (used only for faults)
    double              trueValue;              //!< [-] solar irradiance measurement without perturbations
    double              sensedValue;            //!< [-] solar irradiance measurement including perturbations
    double              kellyFactor;            //!< [-] Kelly curve fit for output cosine curve
    double              fov;                    //!< [-] rad, field of view half angle
    Eigen::Vector3d     r_B;                    //!< [m] position vector in body frame
    Eigen::Vector3d     r_PB_B;                 //!< [m] misalignment of CSS platform wrt spacecraft body frame
    double              senBias;                //!< [-] Sensor bias value
    double              senNoiseStd;            //!< [-] Sensor noise value
    double              faultNoiseStd;          //!< [-] Sensor noise value if CSSFAULT_RAND is triggered
    double              maxOutput;              //!< [-] maximum output (ceiling) for saturation application
    double              minOutput;              //!< [-] minimum output (floor) for saturation application
    double              walkBounds;             //!< [-] Gauss Markov walk bounds
    double              kPower;                 //!< [-] Power factor for kelly curve
    int                 CSSGroupID=-1;          //!< [-] (optional) CSS group id identifier, -1 means it is not set and default is used
    BSKLogger bskLogger;                        //!< -- BSK Logging

    void setAMatrix(const Eigen::Matrix<double, -1, 1, 0, -1, 1>& propMatrix);
    Eigen::Matrix<double, -1, 1, 0, -1, 1> getAMatrix() const;

private:
    SpicePlanetStateMsgPayload sunData;             //!< [-] Unused for now, but including it for future
    SCStatesMsgPayload stateCurrent;            //!< [-] Current SSBI-relative state
    EclipseMsgPayload sunVisibilityFactor;          //!< [-] scaling parameter from 0 (fully obscured) to 1 (fully visible)
    double              sunDistanceFactor;      //! [-] Factor to scale cosine curve magnitude based on solar flux at location
    GaussMarkov noiseModel;                     //! [-] Gauss Markov noise generation model
    GaussMarkov faultNoiseModel;                //! [-] Gauss Markov noise generation model exclusively for CSS fault
    Saturate saturateUtility;                   //! [-] Saturation utility
    Eigen::Matrix<double, -1, 1, 0, -1, 1> propagationMatrix;  // Store the propagation matrix
};

//!@brief Constellation of coarse sun sensors for aggregating output information
/*! This class is a thin container on top of the above coarse-sun sensor class.
It is used to aggregate the output messages of the coarse sun-sensors into a
a single output for use by downstream models.*/
class CSSConstellation: public SysModel {
 public:
    CSSConstellation();                         //!< @brief [-] Default constructor
    ~CSSConstellation();                        //!< @brief [-] Default Destructor
    void Reset(uint64_t CurrentClock);          //!< Method for reseting the module
    void UpdateState(uint64_t CurrentSimNanos); //!< @brief [-] Main update method for CSS constellation
    void appendCSS(CoarseSunSensor *newSensor); //!< @brief [-] Method for adding sensor to list

 public:
    Message<CSSArraySensorMsgPayload> constellationOutMsg;  //!< [-] CSS constellation output message
    std::vector<CoarseSunSensor *> sensorList;    //!< [-] List of coarse sun sensors in constellation
 private:
    CSSArraySensorMsgPayload outputBuffer;      //!< [-] buffer used to write output message
};


#endif
