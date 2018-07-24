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

#include "dvGuidance/dvAttGuidance/dvGuidance.h"
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
void SelfInit_dvGuidance(dvGuidanceConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(
        ConfigData->outputDataName, sizeof(AttRefFswMsg), "AttRefFswMsg", moduleID);
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
void Update_dvGuidance(dvGuidanceConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    double dcm_BuN[3][3];           /*!< dcm, inertial to burn frame */
    double dvUnit[3];
    double burnY[3];
	double burnTime;
	double rotPRV[3];
	double rotDCM[3][3];
    uint64_t writeTime;
    uint32_t writeSize;
    DvBurnCmdFswMsg localBurnData;
    
    ReadMessage(ConfigData->inputBurnCmdID, &writeTime, &writeSize,
                sizeof(DvBurnCmdFswMsg), &localBurnData, moduleID);
    
    ConfigData->dvMag = v3Norm(localBurnData.dvInrtlCmd);
    v3Normalize(localBurnData.dvInrtlCmd, dvUnit);
    v3Copy(dvUnit, dcm_BuN[0]);
    v3Cross(localBurnData.dvRotVecUnit, dvUnit, burnY);
    v3Normalize(burnY, dcm_BuN[1]);
    v3Cross(dcm_BuN[0], dcm_BuN[1], dcm_BuN[2]);
    v3Normalize(dcm_BuN[2], dcm_BuN[2]);

	burnTime = ((int64_t) callTime - (int64_t) localBurnData.burnStartTime)*1.0E-9;
    v3SetZero(rotPRV);
    rotPRV[2] = 1.0;
    v3Scale(burnTime*localBurnData.dvRotVecMag, rotPRV, rotPRV);
    PRV2C(rotPRV, rotDCM);
	m33MultM33(rotDCM, dcm_BuN, dcm_BuN);

	C2MRP(RECAST3X3 &dcm_BuN[0][0], ConfigData->attCmd.sigma_RN);
	v3Scale(localBurnData.dvRotVecMag, dcm_BuN[2], ConfigData->attCmd.omega_RN_N);
    v3SetZero(ConfigData->attCmd.domega_RN_N);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttRefFswMsg),
        &ConfigData->attCmd, moduleID);
    
    return;
}

