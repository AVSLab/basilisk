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

#include "attGuidance/attMnvrPoint/attMnvrPoint.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the nominal attitude maneuver guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the attitude maneuver guidance
 */
void SelfInit_attMnvrPoint(attMnvrPointConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
        sizeof(attGuidOut), "attGuidOut", moduleID);
    return;
    
}

/*! This method performs the second stage of initialization for the attitude maneuver
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the attitude maneuver guidance
 */
void CrossInit_attMnvrPoint(attMnvrPointConfig *ConfigData, uint64_t moduleID)
{
    /*! - Find the input IDs for each input message*/
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavStateName,
        sizeof(NavStateOut), moduleID);
    ConfigData->inputCmdID = subscribeToMessage(ConfigData->inputAttCmdName,
        sizeof(attCmdOut), moduleID);
    return;
    
}

/*! This method takes the estimated state and commanded attitude and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param ConfigData The configuration data associated with the attitude maneuver guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_attMnvrPoint(attMnvrPointConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    attCmdOut localCmd;
    NavStateOut localState;
    uint64_t clockTime;
    uint32_t readSize;
    double angSegment;
    double timeSegment;
    double totalAngle;
    double phiDot;
    double attMnvrUse[3];

    
    memset(&localCmd, 0x0, sizeof(attCmdOut));
    memset(&localState, 0x0, sizeof(NavStateOut));
    
    ReadMessage(ConfigData->inputNavID, &clockTime, &readSize,
                sizeof(NavStateOut), (void*) &(localState));
    ReadMessage(ConfigData->inputCmdID, &clockTime, &readSize,
                sizeof(attCmdOut), (void*) &(localCmd));
    
    if(ConfigData->mnvrActive != 1)
    {
        computeNewAttMnvr(ConfigData, &localCmd, &localState);
        ConfigData->mnvrActive = 1;
        ConfigData->startClockRead = callTime;
    }
	ConfigData->mnvrComplete = 0;
    
    ConfigData->currMnvrTime = (callTime*1.0E-9 -
        ConfigData->startClockRead*1.0E-9);
    ConfigData->currMnvrTime = ConfigData->currMnvrTime >
        ConfigData->totalMnvrTime ? ConfigData->totalMnvrTime :
        ConfigData->currMnvrTime;
    
    totalAngle = 0.0;
    timeSegment = ConfigData->totalMnvrTime - ConfigData->currMnvrTime;
    timeSegment = timeSegment > ConfigData->mnvrStartTime ?
        ConfigData->mnvrStartTime : timeSegment;
    angSegment = 0.5*ConfigData->maxAngAccel*timeSegment*timeSegment;
    totalAngle = totalAngle+angSegment;
    phiDot = ConfigData->maxAngAccel * timeSegment;
    
    timeSegment = ConfigData->totalMnvrTime - timeSegment -
       ConfigData->currMnvrTime;
    timeSegment = timeSegment > ConfigData->totalMnvrTime -
    2.0*ConfigData->mnvrStartTime ? ConfigData->totalMnvrTime -
    2.0*ConfigData->mnvrStartTime : timeSegment;
    angSegment = ConfigData->mnvrCruiseRate * timeSegment;
    totalAngle = totalAngle + angSegment;
    
    timeSegment = ConfigData->currMnvrTime - ConfigData->mnvrStartTime;
    timeSegment = timeSegment > 0.0 ? 0.0 : timeSegment;
    angSegment = 0.5*ConfigData->maxAngAccel*(-2.0*timeSegment*
        ConfigData->mnvrStartTime  - timeSegment*timeSegment);
    phiDot = phiDot + timeSegment*ConfigData->maxAngAccel;
    totalAngle = totalAngle+angSegment;
    
    v3Scale(-totalAngle, ConfigData->attMnvrVec, attMnvrUse);
    PRV2MRP(attMnvrUse, attMnvrUse);
    addMRP(localCmd.sigma_BR, attMnvrUse, ConfigData->sigmaCmd);
    v3Scale(-1.0, ConfigData->sigmaCmd, ConfigData->sigmaCmd);
    addMRP(ConfigData->sigmaCmd, localState.sigma_BN,
    ConfigData->attOut.sigma_BR);
    MRPswitch(ConfigData->attOut.sigma_BR, 1.0, ConfigData->attOut.sigma_BR);
	if (ConfigData->currMnvrTime == ConfigData->totalMnvrTime)
	{
		ConfigData->mnvrComplete = 1;
	}
    
    v3Scale(phiDot, ConfigData->attMnvrVec, ConfigData->bodyRateCmd);
    v3Subtract(localState.omega_BN_B, ConfigData->bodyRateCmd,
    ConfigData->attOut.omega_BR_B);

	WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attGuidOut),
		(void*)&(ConfigData->attOut), moduleID);
    
    return;
}

/*! This method computes a new attitude maneuver to take the vehicle from its 
    current state to a commanded end state
    @return void
    @param ConfigData The configuration data associated with the attitude maneuver guidance
    @param endState The commanded attitude for the spacecraft to hit at the end of the maneuver
    @param currState The current state of the spacecraft output from nav
*/
void computeNewAttMnvr(attMnvrPointConfig *ConfigData, attCmdOut *endState,
                       NavStateOut *currState)
{
    double sigmaBcurr2BCmd[3];
    double MRPMnvr[3];
    double totalAngle;
    double startStopAng;
    v3Scale(-1.0, currState->sigma_BN, sigmaBcurr2BCmd);
    addMRP(sigmaBcurr2BCmd, endState->sigma_BR, MRPMnvr);
    MRP2PRV(MRPMnvr, ConfigData->attMnvrVec);
    totalAngle = v3Norm(ConfigData->attMnvrVec);
    v3Normalize(ConfigData->attMnvrVec, ConfigData->attMnvrVec);
    ConfigData->mnvrStartTime = ConfigData->mnvrCruiseRate/
        ConfigData->maxAngAccel;
    startStopAng = ConfigData->mnvrCruiseRate*ConfigData->mnvrCruiseRate/
        ConfigData->maxAngAccel;
    if(startStopAng > totalAngle)
    {
        startStopAng = totalAngle;
        ConfigData->mnvrStartTime = sqrt(startStopAng/ConfigData->maxAngAccel);
    }
    totalAngle = totalAngle - startStopAng;
    ConfigData->totalMnvrTime = 2.0*ConfigData->mnvrStartTime + totalAngle/
        ConfigData->mnvrCruiseRate;
    return;
}
