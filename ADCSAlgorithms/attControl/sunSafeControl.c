
#include "attControl/sunSafeControl.h"
#include "attGuidance/attGuidOut.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the sun safe attitude control.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the sun safe control
 */
void SelfInit_sunSafeControl(sunSafeControlConfig *ConfigData)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
        sizeof(vehControlOut), "vehControlOut");
    
}

/*! This method performs the second stage of initialization for the sun safe attitude
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the sun safe attitude control
 */
void CrossInit_sunSafeControl(sunSafeControlConfig *ConfigData)
{
    /*! - Get the control data message ID*/
    ConfigData->inputMsgID = FindMessageID(ConfigData->inputGuidName);
    
}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param ConfigData The configuration data associated with the sun safe attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunSafeControl(sunSafeControlConfig *ConfigData, uint64_t callTime)
{
    attGuidOut guidCmd;
    uint64_t clockTime;
    uint32_t readSize;
    double v3[3];
    double Lr[3];
    /*! Begin method steps*/
    /*! - Read the current computed guidance errors*/
    ReadMessage(ConfigData->inputMsgID, &clockTime, &readSize,
                sizeof(attGuidOut), (void*) &(guidCmd));
    v3Scale(ConfigData->K, guidCmd.sigma_BR, v3);
    v3Scale(ConfigData->P, guidCmd.omega_BR, Lr);
    v3Add(v3, Lr, Lr);
    
    v3Copy(Lr, ConfigData->controlOut.accelRequestBody);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehControlOut),
                 (void*) &(ConfigData->controlOut));
    
    return;
}
