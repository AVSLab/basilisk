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

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include <random>
#include "utilities/gauss_markov.h"
#include "simMessages/scPlusStatesMessage.h"
#include "simMessages/scPlusMassPropsMessage.h"
#include "../SimFswMessages/imuSensorMessage.h"


class ImuSensor: public SysModel {
public:
    ImuSensor();
    ~ImuSensor();
    
    void CrossInit();
    void SelfInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);
    void setStructureToPlatformDCM(double yaw, double pitch, double roll);
    void computePlatformDR();
    void computePlatformDV(uint64_t CurrentTime);
    void applySensorErrors(uint64_t CurrentTime);
    void applySensorDiscretization(uint64_t CurrentTime);
	void applySensorSaturation(uint64_t CurrentTime);
	void computeSensorErrors(uint64_t CurrentTime);

public:
    std::string InputStateMsg;          /*!< Message name for spacecraft state */
    std::string InputMassMsg;           /*!< Mass properties message name */
    std::string OutputDataMsg;          /*!< Message name for CSS output data */
    std::vector<double> SensorPosStr;   /// [m] IMU sensor location in structure
    double Str2Platform[3][3];          /// -- Transform from body to platform
    double senRotBias[3];               /// [r/s] Rotational Sensor bias value
    double senTransBias[3];             /// [m/s2] Translational acceleration sen bias
	double senRotMax;					/// [r/s] Gyro saturation value
	double senTransMax;					/// [r/s] Accelerometer saturation value
    uint64_t OutputBufferCount;         /// -- number of output msgs stored
    bool NominalReady;                  /// -- Flag indicating that system is in run
	std::vector<double> PMatrixAccel;   //!< [-] Covariance matrix used to perturb state
	std::vector<double> AMatrixAccel;   //!< [-] AMatrix that we use for error propagation
	std::vector<double> walkBoundsAccel;//!< [-] "3-sigma" errors to permit for states
	std::vector<double> navErrorsAccel; //!< [-] Current navigation errors applied to truth
	std::vector<double> PMatrixGyro;    //!< [-] Covariance matrix used to perturb state
	std::vector<double> AMatrixGyro;    //!< [-] AMatrix that we use for error propagation
	std::vector<double> walkBoundsGyro; //!< [-] "3-sigma" errors to permit for states
	std::vector<double> navErrorsGyro;  //!< [-] Current navigation errors applied to truth

    IMUSensorMessage trueValues;        //!< [-] total measurement without perturbations
    IMUSensorMessage sensedValues;      //!< [-] total measurement including perturbations
    
    double accelLSB;                    //! (-) Discretization value (least significant bit) for accel data
    double gyroLSB;                     //! (-) Discretization value for gyro data
private:
    int64_t InputStateID;               /// -- Connect to input time message
    int64_t InputMassID;                /// -- Message ID for the mass properties
    int64_t OutputDataID;               /// -- Connect to output CSS data
    uint64_t PreviousTime;              /// -- Timestamp from previous frame
    SCPlusStatesMessage StatePrevious;  /// -- Previous state to delta in IMU
    SCPlusStatesMessage StateCurrent;   /// -- Current SSBI-relative state
    SCPlusMassPropsMessage MassCurrent; /// -- Current mass props for the vehicle
	GaussMarkov errorModelAccel;        //!< [-] Gauss-markov error states
	GaussMarkov errorModelGyro;         //!< [-] Gauss-markov error states
};

#endif
