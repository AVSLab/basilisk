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

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <vector>
#include "utilities/sys_model.h"
#include "utilities/dyn_effector.h"
#include <random>

typedef struct{
    double DVFramePlatform[3];      //!< m/s Accumulated DVs in platform
    double AccelPlatform[3];        //!< m/s2 Apparent acceleration of the platform
    double DRFramePlatform[3];      //!< r  Accumulated DRs in platform
    double AngVelPlatform[3];       //!< r/s Angular velocity in platform frame
}ImuSensorOutput;

class ImuSensor: public SysModel {
public:
    ImuSensor();
    ~ImuSensor();
    
    void CrossInit();
    void SelfInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void setStructureToPlatformDCM(double yaw, double pitch, double roll);
    void computePlatformDR();
    void computePlatformDV(uint64_t CurrentTime);
    void readInputs();
    void writeOutputs(uint64_t Clock);
    void applySensorErrors(uint64_t CurrentTime);
    void applySensorDiscretization(uint64_t CurrentTime);
    
public:
    std::string InputStateMsg;                  /*!< Message name for spacecraft state */
    std::string InputMassMsg;                   /*!< Mass properties message name */
    std::string OutputDataMsg;                  /*!< Message name for CSS output data */
    std::vector<double> SensorPosStr;   /// [m[ IMU sensor location in structure
    double Str2Platform[3][3];          /// -- Transform from body to platform
    double senRotBias[3];               /// [r/s] Rotational Sensor bias value
    double senRotNoiseStd[3];           /// [r/s] Rotational sensor standard deviation
    double senTransBias[3];             /// [m/s2] Translational acceleration sen bias
    double senTransNoiseStd[3];         /// [m/s2] Accel sensor standard deviation
    uint64_t OutputBufferCount;         /// -- number of output msgs stored
    bool NominalReady;                  /// -- Flag indicating that system is in run
    double DVFramePlatform[3];          /// [m/s] Accumulated DVs in platform
    double AccelPlatform[3];            /// [m/s2] Apparent acceleration of the platform
    double DRFramePlatform[3];          /// [r]  Accumulated DRs in platform
    double AngVelPlatform[3];           /// [r/s] Angular velocity in platform frame
    double accelLSB;                    //! (-) Discretization value (least significant bit) for accel data
    double gyroLSB;                     //! (-) Discretization value for gyro data
    int32_t isOutputTruth;              /// -- Flag indicating whether the output information is the truth or is corrupted with sensor errors
private:
    int64_t InputStateID;               /// -- Connect to input time message
    int64_t InputMassID;                /// -- Message ID for the mass properties
    int64_t OutputDataID;               /// -- Connect to output CSS data
    uint64_t PreviousTime;              /// -- Timestamp from previous frame
    OutputStateData StatePrevious;      /// -- Previous state to delta in IMU
    OutputStateData StateCurrent;       /// -- Current SSBI-relative state
    MassPropsData MassCurrent;          /// -- Current mass props for the vehicle
    std::default_random_engine rot_rgen[3];         /// -- Random number generator for rotation
    std::normal_distribution<double> rot_rnum[3];   /// -- Random number distribution (rotation)
    std::default_random_engine trans_rgen[3];       /// -- Random number generator for translation
    std::normal_distribution<double> trans_rnum[3]; /// -- Random number distribution (translation)
};

#endif
