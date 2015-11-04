
#include "dvGuidance/dvGuidance.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the nominal delta-V maneuver guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the delta-V maneuver guidance
 */
void SelfInit_dvGuidance(dvGuidanceConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(
        ConfigData->outputDataName, sizeof(attCmdOut), "attCmdOut", moduleID);
    return;
    
}

/*! This method performs the second stage of initialization for the delta-V maneuver
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the attitude maneuver guidance
 */
void CrossInit_dvGuidance(dvGuidanceConfig *ConfigData, uint64_t moduleID)
{
    /*ConfigData->inputMPID = subscribeToMessage(ConfigData->inputMassPropName, sizeof() <#int64_t moduleID#>)(ConfigData->inputMassPropName);*/
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavDataName,
        sizeof(NavStateOut), moduleID);
    return;
    
}

/*! This method takes its own internal variables and creates an output attitude 
    command to use for burn execution.  It also flags whether the burn should 
    be happening or not.
 @return void
 @param ConfigData The configuration data associated with the delta-V maneuver guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_dvGuidance(dvGuidanceConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    double T_Inrtl2Burn[3][3];
    double T_Inrtl2Bdy[3][3];
    double dvUnit[3];
    double burnY[3];
    double burnAccum[3];
    double dvExecuteMag;
    uint64_t writeTime;
    uint32_t writeSize;
    NavStateOut navData;
    
    ReadMessage(ConfigData->inputNavID, &writeTime, &writeSize,
        sizeof(NavStateOut), &navData);
    
    ConfigData->dvMag = v3Norm(ConfigData->dvInrtlCmd);
    v3Normalize(ConfigData->dvInrtlCmd, dvUnit);
    v3Copy(dvUnit, T_Inrtl2Burn[0]);
    v3Cross(dvUnit, ConfigData->desiredOffAxis, burnY);
    v3Normalize(burnY, T_Inrtl2Burn[1]);
    v3Cross(T_Inrtl2Burn[0], T_Inrtl2Burn[1], T_Inrtl2Burn[2]);
    v3Normalize(T_Inrtl2Burn[2], T_Inrtl2Burn[2]);
    m33MultM33(ConfigData->Tburn2Bdy, T_Inrtl2Burn, T_Inrtl2Bdy);
    C2MRP(&T_Inrtl2Bdy[0][0], ConfigData->attCmd.sigma_BR);
    v3SetZero(ConfigData->attCmd.omega_BR);
    
    v3SetZero(burnAccum);
    if((ConfigData->burnExecuting == 0 && callTime > ConfigData->burnStartTime)
        && ConfigData->burnComplete != 1)
    {
        ConfigData->burnExecuting = 1;
        v3Copy(navData.vehAccumDV, ConfigData->dvInit);
        ConfigData->burnComplete = 0;
    }

    if(ConfigData->burnExecuting)
    {
        v3Subtract(navData.vehAccumDV, ConfigData->dvInit, burnAccum);
    }
    
    dvExecuteMag = v3Norm(burnAccum);
    ConfigData->burnComplete = ConfigData->burnComplete == 1 ||
        dvExecuteMag > ConfigData->dvMag;
    ConfigData->burnExecuting = ConfigData->burnComplete != 1 &&
        ConfigData->burnExecuting == 1;
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attCmdOut),
        &ConfigData->attCmd, moduleID);
    
    return;
}

