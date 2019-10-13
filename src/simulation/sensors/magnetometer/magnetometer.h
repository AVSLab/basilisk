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

class Magnetometer : public SysModel {
public:
	Magnetometer();
	~Magnetometer();
	void CrossInit(); //!< @brief method for initializing cross dependencies
	void SelfInit();  //!< @brief method for initializing own messages
	void UpdateState(uint64_t CurrentSimNanos); //!< @brief method to update state for runtime
	void setBodyToSensorDCM(double yaw, double pitch, double roll); //!< @brief utility method to configure the sensor DCM
	void readInputMessages(); //!< @brief method to read the input messages
	void computeTrueOutput(); //!< @brief method to compute the true magnetic field vector
	void computeMagData(); //!< @brief method to get the magnetic field vector information
	void applySensorErrors(); //!< @brief method to set the actual output of the sensor with scaling
	void scaleSensorValues();
	void applySaturation();     //!< apply saturation effects to sensed output (floor and ceiling)
	void writeOutputMessages(uint64_t Clock); //!< @brief method to write the output message to the system

public:
	std::string stateIntMsgName;                  //!< [-] Message name for spacecraft state */
	std::string magIntMsgName;                    //!< [-] Message name for magnetic field data
	std::string tamDataOutMsgName;                  //!< [-] Message name for TAM output data */
	double              B2S321Angles[3];        //!< [-] 321 Euler angles for body to sensor
	Eigen::Matrix3d     dcm_SB;           //!< [-] DCM from sensor frame S to body frame B
	Eigen::Vector3d     tam_B;              //!< [-] magnetic field vector in body frame components
	Eigen::Vector3d     tam_S;              //!< [-] magnetic field vector in sensor frame components
	Eigen::Vector3d              sensedValue;            //!< [-] total measurement including perturbations
	Eigen::Vector3d              trueValue;              //!< [-] total measurement without perturbations
	double              scaleFactor;            //!< [-] scale factor applied to sensor (common + individual multipliers)
	Eigen::Vector3d              senBias;                //!< [-] Sensor bias value
	double              senNoiseStd;            //!< [-] Sensor noise value
	uint64_t            outputBufferCount;      //!< [-] number of output msgs stored
	Eigen::Matrix3d idm3d; // set 3d identity matrix
	Eigen::Vector3d walkBounds;   //!< [-] "3-sigma" errors to permit for states
	double              maxOutput;              //!< [-] maximum output (ceiling) for saturation application
	double              minOutput;              //!< [-] minimum output (floor) for saturation application

private:
	Eigen::Matrix3d AMatrix;      //!< [-] AMatrix that we use for error propagation
	int64_t magIntMsgID;                         //!< [-] Connect to input time message
	int64_t stateIntMsgID;                       //!< [-] Connect to input time message
	int64_t tamDataOutMsgID;                       //!< [-] Connect to output mag data
	MagneticFieldSimMsg magData;            //!< [-] Unused for now, but including it for future
	SCPlusStatesSimMsg stateCurrent;           //!< [-] Current SSBI-relative state
	uint64_t numStates;                 /// -- Number of States for Gauss Markov Models
	GaussMarkov noiseModel;                    //! [-] Gauss Markov noise generation model
	Saturate saturateUtility;                  //! [-] Saturation utility
	TAMSensorIntMsg outputBuffer;             //!< [-] buffer used to write output message

};

/*! @} */

#endif