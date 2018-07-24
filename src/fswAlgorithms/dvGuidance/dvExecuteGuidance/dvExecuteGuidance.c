/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "dvGuidance/dvExecuteGuidance/dvExecuteGuidance.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the nominal delta-V maneuver guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the delta-V maneuver guidance
 */
void SelfInit_dvExecuteGuidance(dvExecuteGuidanceConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(
        ConfigData->outputDataName, sizeof(dvExecutionData), "dvExecutionData", moduleID);
    ConfigData->outputThrID = CreateNewMessage(ConfigData->outputThrName, sizeof(THRArrayOnTimeCmdIntMsg),
                                               "THRArrayOnTimeCmdIntMsg", moduleID);
    return;
    
}

/*! This method performs the second stage of initialization for the delta-V maneuver
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the attitude maneuver guidance
 */
void CrossInit_dvExecuteGuidance(dvExecuteGuidanceConfig *ConfigData, uint64_t moduleID)
{
    /*ConfigData->inputMPID = subscribeToMessage(ConfigData->inputMassPropName, sizeof() <#int64_t moduleID#>)(ConfigData->inputMassPropName);*/
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavDataName,
        sizeof(NavTransIntMsg), moduleID);
    ConfigData->inputBurnCmdID = subscribeToMessage(ConfigData->inputBurnDataName,
                                                    sizeof(DvBurnCmdFswMsg), moduleID);
    return;
    
}

/*! This method takes its own internal variables and creates an output attitude 
    command to use for burn execution.  It also flags whether the burn should 
    be happening or not.
 @return void
 @param ConfigData The configuration data associated with the delta-V maneuver guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_dvExecuteGuidance(dvExecuteGuidanceConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    double burnAccum[3];
    double dvExecuteMag;
	double burnTime;
    double dvMag;
    uint64_t writeTime;
    uint32_t writeSize;
    NavTransIntMsg navData;
    DvBurnCmdFswMsg localBurnData;
    dvExecutionData localExeData;
    THRArrayOnTimeCmdIntMsg effCmd;
    
    ReadMessage(ConfigData->inputNavID, &writeTime, &writeSize,
        sizeof(NavTransIntMsg), &navData, moduleID);
    ReadMessage(ConfigData->inputBurnCmdID, &writeTime, &writeSize,
                sizeof(DvBurnCmdFswMsg), &localBurnData, moduleID);
    
    burnTime = ((int64_t) callTime - (int64_t) localBurnData.burnStartTime)*1.0E-9;
    v3SetZero(burnAccum);
    if((ConfigData->burnExecuting == 0 && burnTime >= 0.0)
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
    
    dvMag = v3Norm(localBurnData.dvInrtlCmd);
    dvExecuteMag = v3Norm(burnAccum);
    ConfigData->burnComplete = ConfigData->burnComplete == 1 ||
        dvExecuteMag > dvMag;
    ConfigData->burnExecuting = ConfigData->burnComplete != 1 &&
        ConfigData->burnExecuting == 1;
    
    if(ConfigData->burnComplete)
    {
        memset(&effCmd, 0x0, sizeof(THRArrayOnTimeCmdIntMsg));
        WriteMessage(ConfigData->outputThrID, callTime,
            sizeof(THRArrayOnTimeCmdIntMsg), &effCmd, moduleID);
    }
    
    localExeData.burnComplete = ConfigData->burnComplete;
    localExeData.burnExecuting = ConfigData->burnExecuting;
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(dvExecutionData),
        &localExeData, moduleID);
    
    return;
}

