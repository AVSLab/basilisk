
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
    void ReadInputs();
    void setStructureToPlatformDCM(double yaw, double pitch, double roll);
    void WriteOutputs(uint64_t Clock);
    void ComputePlatformDR();
    void ComputePlatformDV(uint64_t CurrentTime);
    void ApplySensorErrors(uint64_t CurrentTime);
    
public:
    std::string InputStateMsg;                  /*!< Message name for spacecraft state */
    std::string InputMassMsg;                   /*!< Mass properties message name */
    std::string OutputDataMsg;                  /*!< Message name for CSS output data */
    std::vector<double> SensorPosStr; /// m IMU sensor location in structure
    double Str2Platform[3][3];     /// -- Transform from body to platform
    double senRotBias[3];          /// r/s Rotational Sensor bias value
    double senRotNoiseStd[3];      /// r/s Rotational sensor standard deviation
    double senTransBias[3];        /// m/s2 Translational acceleration sen bias
    double senTransNoiseStd[3];    /// m/s2 Accel sensor standard deviation
    uint64_t OutputBufferCount;      /// -- number of output msgs stored
    bool NominalReady;               /// -- Flag indicating that system is in run
    double DVFramePlatform[3];      /// m/s Accumulated DVs in platform
    double AccelPlatform[3];        /// m/s2 Apparent acceleration of the platform
    double DRFramePlatform[3];      /// r  Accumulated DRs in platform
    double AngVelPlatform[3];       /// r/s Angular velocity in platform frame
private:
    int64_t InputStateID;            /// -- Connect to input time message
    int64_t InputMassID;             /// -- Message ID for the mass properties
    int64_t OutputDataID;            /// -- Connect to output CSS data
    uint64_t PreviousTime;           /// -- Timestamp from previous frame
    OutputStateData StatePrevious;   /// -- Previous state to delta in IMU
    OutputStateData StateCurrent;    /// -- Current SSBI-relative state
    MassPropsData MassCurrent;       /// -- Current mass props for the vehicle
    std::default_random_engine rot_rgen[3]; /// -- Random number generator for rotation
    std::normal_distribution<double> rot_rnum[3];  /// -- Random number distribution (rotation)
    std::default_random_engine trans_rgen[3]; /// -- Random number generator for translation
    std::normal_distribution<double> trans_rnum[3];  /// -- Random number distribution (translation)
};

#endif
