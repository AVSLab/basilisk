/*
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H
#include <vector>
#include <random>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/magneticFieldSimMsg.h"
#include "simFswInterfaceMessages/tamSensorIntMsg.h"
#include "simMessages/tamDataSimMsg.h"
#include "utilities/gauss_markov.h"
#include "utilities/saturate.h"
#include <Eigen/Dense>

 /*! \addtogroup SimModelGroup
 * @{
 */

 /*! @brief This is the three-axis magnetometer (TAM) module.

## Module Purpose

### Executive Summary

This document describes how Three-Axis Magnetometer (TAM) devices are modeled in the Basilisk software. The purpose of this module is to implement magnetic field measurements on the sensor frame \f$ S \f$.

### Module Assumptions and Limitations
Assumptions made in TAM module and the corresponding limitations are shown below:

- <b>Magnetic Field Model Inputs</b>: The magnetometer sensor is limited with the used magnetic field model which are individual magnetic field models complex and having their own assumptions. The reader is referred to the cited literature to learn more about the model limitations and challenges.

- <b>Error Inputs</b>: Since the error models rely on user inputs, these inputs are the most likely source of error in TAM output. Instrument bias would have to be measured experimentally or an educated guess would have to be made. The Gauss-Markov noise model has well-known assumptions and is generally accepted to be a good model for this application.
	  
- <b>External Disturbances</b>: Currently, the module does not consider the external magnetic field, so it is limited to the cases where this effect is not significant. This can be overcome by using magnetic field models taking into these effects account or adding it as an additional term.

### Message Connection Descriptions
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.
Msg Variable Name | Msg Type | Description
------------------|----------|-------------
magIntMsgName| MagneticFieldSimMsg | Magnetic field input messager from one of the magnetic field models
stateIntMsgName| SCPlusStatesSimMsg | Spacecraft state input message
tamDataOutMsgName |  TAMDataSimMsg | TAM sensor output message

## Detailed Module Description
There are a multitude of magnetic field models. As all the magnetic field models are children of MagneticFieldBase base class, magnetometer can be created based on any of the magnetic field model.

### Planet Centric Spacecraft Position Vector
For the following developments, the spacecraft location relative to the planet frame is required.
Let \f$\boldsymbol r_{B/P}\f$ be the spacecraft position vector relative to the planet center.
In the simulation the spacecraft location is given relative to an inertial frame origin \f$ \cal{O}\f$.
The planet centric position vector is computed using

\f{eqnarray*}{
  \boldsymbol r_{B/P} = \boldsymbol r_{B/O} - \boldsymbol r_{P/O}
\f}

If no planet ephemeris message is specified, then the planet position vector \f$r_{P/O}\f$ is set to zero.
Let \f$[PN]\f$ be the direction cosine matrix that relates the rotating planet-fixed frame relative to an inertial frame \f$ \cal{N} \f$.
The simulation provides the spacecraft position vector in inertial frame components.
The planet centric position vector is then written in Earth-fixed frame components using

\f{eqnarray*}{
{}^{\cal{P}}{{\boldsymbol r}_{B/P}} = [PN] \ {}^{\cal{N}}{{\boldsymbol r}_{B/P}}
\f}

### Magnetic Field Models
The truth of the magnetometer measurements in sensor frame coordinates with no errors are output as:

\f{eqnarray*}{
{}^{\cal{S}}{\boldsymbol B} = [SN]\ {}^{\cal{N}}{\boldsymbol B}
\f}

where \f$[SN]\f$ is the direction cosine matrix from \f$\cal{N}\f$ to \f$\cal{S}\f$, and \f${{}^\cal{N}}{ \boldsymbol B}\f$ is the magnetic field vector of the magnetic field model in inertial frame components.

### Sensor Error Modeling
The magnetic field vector of the magnetic field models is considered to be "truth". So, the errors are applied on the "truth" values to simulate a real instrumentation:

\f{eqnarray*}{
\boldsymbol B_{measured} = (\boldsymbol B_{truth} + \boldsymbol e_{noise} + \boldsymbol e_{bias}) f_{scale}
\f}

where \f$ \boldsymbol e_{noise} \f$ is the Gaussian noise, \f$ \boldsymbol e_{bias} \f$ is the bias applied on the magnetic field measurements, and \f$ f_{scale} \f$ is the scale factor applied on the measurements for linear scaling.

### Saturation
Sensors might have specific saturation bounds for their measurements. It also prevents the sensor for giving a value less or higher than the possible hardware output. The saturated values are:

\f{eqnarray*}{
\boldsymbol B_{{sat}_{max}} &=& \mbox{min}(\boldsymbol B_{measured},  \mbox{maxOutput}) \\
\boldsymbol B_{{sat}_{min}} &=& \mbox{max}(\boldsymbol B_{measured},  \mbox{minOutput})
\f}

This is the final output of the sensor module.

## User Guide

### General Module Setup
This section outlines the steps needed to add a Magnetometer module to a sim. First, one of the magnetic field models must be imported:

~~~~~~~{.py}
      from Basilisk.simulation import magneticFieldCenteredDipole
      magModule = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
      magModule.ModelTag = "CenteredDipole"
~~~~~~~

and/or

~~~~~~~{.py}
	  from Basilisk.simulation import magneticFieldWMM
      magModule = magneticFieldWMM.MagneticFieldWMM()
      magModule.ModelTag = "WMM"
~~~~~~~

Then, the magnetic field measurements must be imported and initialized:

~~~~~~~{.py}
	  from Basilisk.simulation import magnetometer
      testModule = magnetometer.Magnetometer()
      testModule.ModelTag = "TAM_sensor"
~~~~~~~

The model can  be added to a task like other simModels.

~~~~~~~{.py}
	  unitTestSim.AddModelToTask(unitTaskName, testModule)
~~~~~~~

Each Magnetometer module calculates the magnetic field based on the magnetic field and output state messages of a spacecraft.  To add spacecraft to the magnetic field model the spacecraft state output message name is sent to the \f$\mbox{addSpacecraftToModel}\f$ method:

~~~~~~~{.py}
	  scObject = spacecraftPlus.SpacecraftPlus()
      scObject.ModelTag = "spacecraftBody"
      magModule.addSpacecraftToModel(scObject.scStateOutMsgName)
~~~~~~~

Magnetic field data is transformed from inertial to body, then to the sensor frame. The transformation from \f$\cal B\f$ to \f$\cal S\f$ can be set via \f$\mbox{dcm_SB}\f$ using the helper function:

	  \f{eqnarray*}{
	  \mbox{setBodyToSensorDCM}(\psi, \theta, \phi)
	  \f}

where \f$(\psi, \theta, \phi)\f$ are classical \f$3-2-1\f$ Euler angles that map from the body frame to the sensor frame \f$\cal S\f$.

### Specifying TAM Sensor Corruptions
Three types of TAM sensor corruptions can be simulated.  If not specified, all these corruptions are zeroed. To add a Gaussian noise component to the output, the variable

	  \f{eqnarray*}{
	  \mbox{senNoiseStd}
	  \f}

is set to a non-zero value.  This is the standard deviation of Gaussian noise in Tesla. Next, to simulate a constant bias, the variable

	  \f{eqnarray*}{
	  \mbox{senBias}
	  \f}

is set to a non-zero value. To simulate a linear scaling of the outputs, the variable
	 
	 \f{eqnarray*}{
	 \mbox{scaleFactor}
	 \f}

is used. Finally, to set saturation values, the variables

	  \f{eqnarray*}{
	  \mbox{maxOutput}
	  \f}

	  \f{eqnarray*}{
	  \mbox{minOutput}
	  \f}

are used. Minimum and maximum bounds for saturation are set to large values as \f$(-10^{200} \mbox{nT})\f$ and \f$(10^{200} \mbox{nT})\f$ respectively in order not to saturate the outputs by default.

   */

class Magnetometer : public SysModel {
public:
	Magnetometer();
	~Magnetometer();
	void SelfInit();                            //!< Method for initializing own messages
	void CrossInit();                           //!< Method for initializing cross dependencies
	void Reset(uint64_t CurrentClock);          //!< Method for reseting the module
	void UpdateState(uint64_t CurrentSimNanos); //!< Method to update state for runtime
	void readInputMessages();                   //!< Method to read the input messages
	void computeTrueOutput();                   //!< Method to compute the true magnetic field vector
	void computeMagData();                      //!< Method to get the magnetic field vector information
	void applySensorErrors();                   //!< Method to set the actual output of the sensor with errors
	void applySaturation();                     //!< Apply saturation effects to sensed output (floor and ceiling)
	void writeOutputMessages(uint64_t Clock);   //!< Method to write the output message to the system
	Eigen::Matrix3d setBodyToSensorDCM(double yaw, double pitch, double roll); //!< Utility method to configure the sensor DCM

public:
	std::string         stateIntMsgName;        //!< [-] Message name for spacecraft state
	std::string         magIntMsgName;          //!< [-] Message name for magnetic field data
	std::string         tamDataOutMsgName;      //!< [-] Message name for TAM output data
	Eigen::Matrix3d     dcm_SB;                 //!< [-] DCM from body frame to sensor frame
	Eigen::Vector3d     tam_S;                  //!< [T] Magnetic field vector in sensor frame
	Eigen::Vector3d     sensedValue;            //!< [T] Measurement including perturbations
	Eigen::Vector3d     trueValue;              //!< [T] Measurement without perturbations
	double              scaleFactor;            //!< [-] Scale factor applied to sensor
	Eigen::Vector3d     senBias;                //!< [T] Sensor bias value
	double              senNoiseStd;            //!< [T] Sensor noise standard deviation value
	uint64_t            outputBufferCount;      //!< [-] Number of output msgs stored
	Eigen::Vector3d     walkBounds;             //!< [T] "3-sigma" errors to permit for states
	double              maxOutput;              //!< [T] Maximum output for saturation application
	double              minOutput;              //!< [T] Minimum output for saturation application

private:
	int64_t magIntMsgID;                         //!< [-] Connect to input magnetic field message
	int64_t stateIntMsgID;                       //!< [-] Connect to input state message
	int64_t tamDataOutMsgID;                     //!< [-] Connect to output magnetometer data
	MagneticFieldSimMsg magData;                 //!< [-] Magnetic field model data
	SCPlusStatesSimMsg stateCurrent;             //!< [-] Current spacecraft state
	uint64_t numStates;                          //!< [-] Number of States for Gauss Markov Models
	GaussMarkov noiseModel;                      //!< [-] Gauss Markov noise generation model
	Saturate saturateUtility;                    //!< [-] Saturation utility
};

/*! @} */

#endif
