
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include "SimCode/utilities/linearAlgebra.h"
#include <string.h>

/*! This method initializes the ConfigData for theIMU sensor interface.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the IMU sensor interface
 */
void SelfInit_imuProcessTelem(IMUConfigData *ConfigData, uint64_t moduleID)
{
    
    /*! - Create output message for module */
    ConfigData->OutputMsgID = CreateNewMessage(ConfigData->OutputDataName,
        sizeof(IMUOutputData), "IMUOutputData", moduleID);
    
}

/*! This method performs the second stage of initialization for the IMU sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the IMU interface
 */
void CrossInit_imuProcessTelem(IMUConfigData *ConfigData, uint64_t moduleID)
{
    uint64_t UnusedClockTime;
    uint32_t ReadSize;
    vehicleConfigData LocalConfigData;
    /*! Begin method steps */
    /*! - Link the message ID for the incoming sensor data message to here */
    ConfigData->SensorMsgID = FindMessageID(ConfigData->InputDataName);
    ConfigData->PropsMsgID = FindMessageID(ConfigData->InputPropsName);
    if(ConfigData->PropsMsgID >= 0)
    {
        ReadMessage(ConfigData->PropsMsgID, &UnusedClockTime, &ReadSize,
                    sizeof(vehicleConfigData), (void*) &LocalConfigData);
        m33MultM33(LocalConfigData.T_str2body, ConfigData->platform2StrDCM,
                   ConfigData->platform2BdyDCM);
    }
    
}

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the IMU nav.
 @return void
 @param ConfigData The configuration data associated with the IMU interface
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_imuProcessTelem(IMUConfigData *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
    uint64_t UnusedClockTime;
    uint32_t ReadSize;
    IMUOutputData LocalInput;
    ReadMessage(ConfigData->SensorMsgID, &UnusedClockTime, &ReadSize,
                sizeof(IMUOutputData), (void*) &LocalInput);
    
    m33MultV3(ConfigData->platform2BdyDCM, LocalInput.DVFrameBody,
              ConfigData->LocalOutput.DVFrameBody);
    m33MultV3(ConfigData->platform2BdyDCM, LocalInput.AccelBody,
              ConfigData->LocalOutput.AccelBody);
    m33MultV3(ConfigData->platform2BdyDCM, LocalInput.DRFrameBody,
              ConfigData->LocalOutput.DRFrameBody);
    m33MultV3(ConfigData->platform2BdyDCM, LocalInput.AngVelBody,
              ConfigData->LocalOutput.AngVelBody);
    
    WriteMessage(ConfigData->OutputMsgID, callTime, sizeof(IMUOutputData),
                 (void*) &(ConfigData->LocalOutput), moduleID);
    
    return;
}
