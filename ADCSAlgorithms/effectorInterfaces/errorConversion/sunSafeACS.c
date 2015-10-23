
#include "effectorInterfaces/errorConversion/sunSafeACS.h"
#include "attControl/vehControlOut.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the sun safe ACS control.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the sun safe control
 */
void SelfInit_sunSafeACS(sunSafeACSConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->thrData.outputMsgID = CreateNewMessage(
        ConfigData->thrData.outputDataName, sizeof(vehEffectorOut),
        "vehEffectorOut", moduleID);
    
}

/*! This method performs the second stage of initialization for the sun safe ACS
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the sun safe ACS control
 */
void CrossInit_sunSafeACS(sunSafeACSConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputMsgID = FindMessageID(ConfigData->inputControlName);
    
}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param ConfigData The configuration data associated with the sun safe ACS control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunSafeACS(sunSafeACSConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    
    uint64_t ClockTime;
    uint32_t ReadSize;
    vehControlOut cntrRequest;
    
    /*! Begin method steps*/
    /*! - Read the input parsed CSS sensor data message*/
    ReadMessage(ConfigData->inputMsgID, &ClockTime, &ReadSize,
                sizeof(vehControlOut), (void*) &(cntrRequest));
    computeSingleThrustBlock(&(ConfigData->thrData), callTime,
                             &cntrRequest, moduleID);
    
    return;
}
