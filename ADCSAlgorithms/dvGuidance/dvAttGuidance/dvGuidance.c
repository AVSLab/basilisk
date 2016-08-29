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

#include "dvGuidance/dvAttGuidance/dvGuidance.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
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
        sizeof(NavTransOut), moduleID);
    ConfigData->inputBurnCmdID = subscribeToMessage(ConfigData->inputBurnDataName,
                                                    sizeof(DvBurnCmdData), moduleID);
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
	double burnTime;
	double rotPRV[3];
	double rotDCM[3][3];
	double omega_BR_N[3];
    uint64_t writeTime;
    uint32_t writeSize;
    NavTransOut navData;
    DvBurnCmdData localBurnData;
    
    ReadMessage(ConfigData->inputNavID, &writeTime, &writeSize,
        sizeof(NavTransOut), &navData, moduleID);
    ReadMessage(ConfigData->inputBurnCmdID, &writeTime, &writeSize,
                sizeof(DvBurnCmdData), &localBurnData, moduleID);
    
    ConfigData->dvMag = v3Norm(localBurnData.dvInrtlCmd);
    v3Normalize(localBurnData.dvInrtlCmd, dvUnit);
    v3Copy(dvUnit, T_Inrtl2Burn[0]);
    v3Cross(localBurnData.dvRotVecUnit, dvUnit, burnY);
    v3Normalize(burnY, T_Inrtl2Burn[1]);
    v3Cross(T_Inrtl2Burn[0], T_Inrtl2Burn[1], T_Inrtl2Burn[2]);
    v3Normalize(T_Inrtl2Burn[2], T_Inrtl2Burn[2]);

	burnTime = ((int64_t) callTime - (int64_t) localBurnData.burnStartTime)*1.0E-9;
	v3Scale(burnTime*localBurnData.dvRotVecMag, localBurnData.dvRotVecUnit, rotPRV);
	PRV2C(rotPRV, rotDCM);
	m33MultM33(rotDCM, T_Inrtl2Burn, T_Inrtl2Burn);

	m33MultM33(RECAST3X3 ConfigData->Tburn2Bdy, T_Inrtl2Burn, T_Inrtl2Bdy);
	C2MRP(RECAST3X3 &T_Inrtl2Bdy[0][0], ConfigData->attCmd.sigma_BR);
	v3Scale(localBurnData.dvRotVecMag, T_Inrtl2Burn[2], omega_BR_N);
	m33MultV3(RECAST3X3 T_Inrtl2Bdy, omega_BR_N, ConfigData->attCmd.omega_BR);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attCmdOut),
        &ConfigData->attCmd, moduleID);
    
    return;
}

