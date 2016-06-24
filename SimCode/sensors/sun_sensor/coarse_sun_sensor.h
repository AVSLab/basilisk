/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#include "utilities/sys_model.h"
#include "environment/spice/spice_interface.h"
#include "utilities/dyn_effector.h"
#include <random>

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

//!@brief CSS output structure data
typedef struct {
    double OutputData;       /*!< CSS measurement output */
}CSSRawOutputData;

//!@brief Coarse sun sensor model
/*! This class is designed to model the state of a single coarse sun sensor 
attached to a spacecraft.  It emulates the "counts" that will typically be 
output by the ADC on board of a spacecraft.*/
class CoarseSunSensor: public SysModel {
public:
    CoarseSunSensor();
    ~CoarseSunSensor();
    
    void CrossInit(); //!< @brief method for initializing cross dependencies
    void SelfInit();  //!< @brief method for initializing own messages
    bool LinkMessages(); //!< @brief method to re-init message linkage
    void UpdateState(uint64_t CurrentSimNanos); //!< @brief method to update state for runtime
    void setUnitDirectionVectorWithPerturbation(double cssThetaPerturb, double cssPhiPerturb); //!< @brief utility method to perturb CSS unit vector
    void setStructureToPlatformDCM(double yaw, double pitch, double roll); //!< @brief utility method to configure the platform DCM
    bool SpacecraftIlluminated(); //!< @brief method to determine if the S/C is illuminated
    void ReadInputs();  //!< @brief method to read the input messages
    void ComputeSunData(); //!< @brief method to get the sun vector information
    void ComputeTruthOutput(); //!< @brief method to compute the true sun-fraction of CSS
    void ComputeActualOutput(); //!< @brief method to set the actual output of the sensor with scaling/kelly
    void WriteOutputs(uint64_t Clock); //!< @brief method to write the output message to the system
    
public:
    std::string InputSunMsg;                    /*!< Message name for sun data */
    std::string InputStateMsg;                  /*!< Message name for spacecraft state */
    std::string OutputDataMsg;                  /*!< Message name for CSS output data */
    CSSFaultState_t     faultState;             /*!< Specification used if state is set to COMPONENT_FAULT */
    double              stuckPercent;           /*!< percent of full value the CSS will remain stuck at if a fault is triggered */
    double              theta;                  /*!< rad, css azimuth angle, measured positive from the body +x axis around the +z axis */
    double              phi;                    /*!< rad, css elevation angle, measured positive toward the body +z axis from the x-y plane */
    double              B2P321Angles[3];
    double              PB[3][3];               /*!< DCM from platform frame P to body frame B */
    double              nHatStr[3];             /*!< css unit direction vector in structural components */
    double              sHatStr[3];             /*!< unit vector to sun in str */
    double              horizonPlane[3];        /*!< unit direction vector defining horizon cut off plane of CSS */
    double              directValue;            /*!< direct solar irradiance measurement */
    double              albedoValue;            /*!< albedo irradiance measurement */
    double              scaleFactor;            /*!< scale factor applied to sensor (common + individual multipliers) */
    double              sensedValue;            /*!< total measurement including perturbations */
    double              ScaledValue;            /*!< Scaled value prior to discretization*/
    double              KellyFactor;            /*!< Kelly curve fit for output cosine curve*/
    double              fov;                    /*!< rad, field of view half angle */
    double              maxVoltage;             /*!< max voltage measurable by CSS, used in discretization */
    double              r_B[3];
    bool MessagesLinked;             //!< [-] Indicator for whether inputs bound
    double SenBias;                  //!< [-] Sensor bias value
    double SenNoiseStd;                 //!< [-] Sensor noise value
    uint64_t OutputBufferCount;      //!< [-] number of output msgs stored
private:
    int64_t InputSunID;              //!< [-] Connect to input time message
    int64_t InputStateID;            //!< [-] Connect to input time message
    int64_t OutputDataID;            //!< [-] Connect to output CSS data
    SpicePlanetState SunData;        //!< [-] Unused for now, but including it for future
    OutputStateData StateCurrent;    //!< [-] Current SSBI-relative state
    std::default_random_engine rgen; //!< [-] Random number generator for disp
    std::normal_distribution<double> rnum;  //! [-] Random number distribution
};

//!@brief Constellation of coarse sun sensors for aggregating output information
/*! This class is a thin container on top of the above coarse-sun sensor class.  
It is used to aggregate the output messages of the coarse sun-sensors into a 
a single output for use by downstream models.*/
class CSSConstellation: public SysModel {
 public:
    CSSConstellation();              //!< @brief [-] Default constructor
    ~CSSConstellation();             //!< @brief [-] Default Destructor
    void CrossInit();                //!< @brief [-] Method for initializing cross dependencies
    void SelfInit();                 //!< @brief [-] Method for initializing own messages
    void UpdateState(uint64_t CurrentSimNanos); //!< @brief [-] Main update method for CSS constellation
    void appendCSS(CoarseSunSensor newSensor) {sensorList.push_back(newSensor);}
    //!< @brief [-] Method for adding sensor to list
    
 public:
    uint64_t outputBufferCount;                  //!< [-] Number of messages archived in output data
    std::string outputConstellationMessage;      //!< [-] Message name for the outgoing message
    int64_t outputConstID;                       //!< [-] output ID for the outgoing message
    std::vector<CoarseSunSensor> sensorList;     //!< [-] List of coarse sun sensors in constellation
 private:
    CSSRawOutputData *outputBuffer;              //!< [-] buffer used to write output message
};

/*! @} */

#endif
