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

#include "effectorInterfaces/rwNullSpace/rwNullSpace.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the null space reaction wheel rejection.
 It creates the output message.
 @return void
 @param ConfigData The configuration data associated with RW null space model
 */
void SelfInit_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(
        ConfigData->outputControlName, sizeof(RWArrayTorqueIntMsg),
        "RWArrayTorqueIntMsg", moduleID);
	
}

/*! This method performs the second stage of initialization for the RW null space control
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the sun safe ACS control
 */
void CrossInit_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t moduleID)
{

    double GsTranspose[3 * MAX_EFF_CNT];
    double GsInvHalf[3 * 3];
    double identMatrix[MAX_EFF_CNT*MAX_EFF_CNT];
    double GsTemp[MAX_EFF_CNT*MAX_EFF_CNT];
    double GsMatrix[3*MAX_EFF_CNT];
    RWConstellationFswMsg localRWData;
    int i, j;
    uint64_t ClockTime;
    uint32_t ReadSize;

    /*! - Get the control data message ID*/
    ConfigData->inputRWCmdsID = subscribeToMessage(ConfigData->inputRWCommands,
        sizeof(RWArrayTorqueIntMsg), moduleID);
	/*! - Get the RW speeds ID*/
	ConfigData->inputSpeedsID = subscribeToMessage(ConfigData->inputRWSpeeds,
		sizeof(RWSpeedIntMsg), moduleID);
    ConfigData->inputRWConfID = subscribeToMessage(ConfigData->inputRWConfigData,
        sizeof(RWConstellationFswMsg), moduleID);
    
    ReadMessage(ConfigData->inputRWConfID, &ClockTime, &ReadSize,
                sizeof(RWConstellationFswMsg), &localRWData, moduleID);
    
    ConfigData->numWheels = localRWData.numRW;
    for(i=0; i<ConfigData->numWheels; i=i+1)
    {
        for(j=0; j<3; j=j+1)
        {
            GsMatrix[j*ConfigData->numWheels+i] = localRWData.reactionWheels[i].gsHat_B[j];
        }
    }
    
    mTranspose(GsMatrix, 3, ConfigData->numWheels, GsTranspose);
    mMultM(GsMatrix, 3, ConfigData->numWheels, GsTranspose,
           ConfigData->numWheels, 3, GsInvHalf);
    m33Inverse(RECAST3X3 GsInvHalf, RECAST3X3 GsInvHalf);
    mMultM(GsInvHalf, 3, 3, GsMatrix, 3, ConfigData->numWheels,
           ConfigData->GsInverse);
    mMultM(GsTranspose, ConfigData->numWheels, 3, ConfigData->GsInverse, 3,
           ConfigData->numWheels, GsTemp);
    mSetIdentity(identMatrix, ConfigData->numWheels, ConfigData->numWheels);
    mSubtract(identMatrix, ConfigData->numWheels, ConfigData->numWheels,
              GsTemp, ConfigData->GsInverse);
    
}

/*! This method nulls the outputs of the RWA null space data.  It is used primarily 
    when inhibiting RWA control where we want to zero the RWA command prior to switching 
    to another effector set.
    @return void
    @param ConfigData The configuration data associated with the null space control
    @param callTime The clock time at which the function was called (nanoseconds)
    @param moduleID The ID associated with the ConfigData
 */
void Reset_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t callTime,
                        uint64_t moduleID)
{
    RWArrayTorqueIntMsg finalControl;
    
    memset(&finalControl, 0x0, sizeof(RWArrayTorqueIntMsg));
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(RWArrayTorqueIntMsg),
                 &finalControl, moduleID);
}

/*! This method takes the input reaction wheel commands as well as the observed 
    reaction wheel speeds and balances the commands so that the overall vehicle 
	momentum is minimized.
 @return void
 @param ConfigData The configuration data associated with the null space control
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the ConfigData
 */
void Update_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    
    uint64_t ClockTime;
    uint32_t ReadSize;
    RWArrayTorqueIntMsg cntrRequest;
	RWSpeedIntMsg rwSpeeds;
	RWArrayTorqueIntMsg finalControl;
	double dVector[MAX_EFF_CNT];
    
    /*! Begin method steps*/
    /*! - Read the input RW commands to get the raw RW requests*/
    ReadMessage(ConfigData->inputRWCmdsID, &ClockTime, &ReadSize,
                sizeof(RWArrayTorqueIntMsg), (void*) &(cntrRequest), moduleID);
	ReadMessage(ConfigData->inputSpeedsID, &ClockTime, &ReadSize,
		sizeof(RWSpeedIntMsg), (void*)&(rwSpeeds), moduleID);
    
	memset(&finalControl, 0x0, sizeof(RWArrayTorqueIntMsg));
	vScale(-ConfigData->OmegaGain, rwSpeeds.wheelSpeeds,
		ConfigData->numWheels, dVector);
	mMultV(ConfigData->GsInverse, ConfigData->numWheels, ConfigData->numWheels,
		dVector, finalControl.motorTorque);
	vAdd(finalControl.motorTorque, ConfigData->numWheels,
		cntrRequest.motorTorque, finalControl.motorTorque);

	WriteMessage(ConfigData->outputMsgID, callTime, sizeof(RWArrayTorqueIntMsg),
		&finalControl, moduleID);

    return;
}
