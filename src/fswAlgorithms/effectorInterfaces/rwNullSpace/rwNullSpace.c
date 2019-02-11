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
    /* Create output message for module */
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
    /* Get the control data message ID*/
    ConfigData->inputRWCmdsID = subscribeToMessage(ConfigData->inputRWCommands,
        sizeof(RWArrayTorqueIntMsg), moduleID);
	/* Get the RW speeds ID*/
	ConfigData->inputSpeedsID = subscribeToMessage(ConfigData->inputRWSpeeds,
		sizeof(RWSpeedIntMsg), moduleID);
    /* Get the RW configuration ID */
    ConfigData->inputRWConfID = subscribeToMessage(ConfigData->inputRWConfigData,
        sizeof(RWConstellationFswMsg), moduleID);

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
    RWArrayTorqueIntMsg finalControl;               /*!<      output message container */
    double GsMatrix[3*MAX_EFF_CNT];                 /*!< [-]  [Gs] projection matrix where gs_hat_B RW spin axis form each colum */
    double GsTranspose[3 * MAX_EFF_CNT];            /*!< [-]  [Gs]^T */
    double GsInvHalf[3 * 3];                        /*!< [-]  ([Gs][Gs]^T)^-1 */
    double identMatrix[MAX_EFF_CNT*MAX_EFF_CNT];    /*!< [-]  [I_NxN] identity matrix */
    double GsTemp[MAX_EFF_CNT*MAX_EFF_CNT];         /*!< [-]  temp matrix */
    RWConstellationFswMsg localRWData;              /*!<      local copy of RW configuration data */
    int i, j;
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;

    /* read in the RW spin axis headings */
    ReadMessage(ConfigData->inputRWConfID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(RWConstellationFswMsg), &localRWData, moduleID);

    /* create the 3xN [Gs] RW spin axis projection matrix */
    ConfigData->numWheels = localRWData.numRW;
    for(i=0; i<ConfigData->numWheels; i=i+1)
    {
        for(j=0; j<3; j=j+1)
        {
            GsMatrix[j*ConfigData->numWheels+i] = localRWData.reactionWheels[i].gsHat_B[j];
        }
    }


    mTranspose(GsMatrix, 3, ConfigData->numWheels, GsTranspose);            /* find [Gs]^T */
    mMultM(GsMatrix, 3, ConfigData->numWheels, GsTranspose,
           ConfigData->numWheels, 3, GsInvHalf);                            /* find [Gs].[Gs]^T */
    m33Inverse(RECAST3X3 GsInvHalf, RECAST3X3 GsInvHalf);                   /* find ([Gs].[Gs]^T)^-1 */
    mMultM(GsInvHalf, 3, 3, GsMatrix, 3, ConfigData->numWheels,
           ConfigData->tau);                                                /* find ([Gs].[Gs]^T)^-1.[Gs] */
    mMultM(GsTranspose, ConfigData->numWheels, 3, ConfigData->tau, 3,
           ConfigData->numWheels, GsTemp);                                  /* find [Gs]^T.([Gs].[Gs]^T)^-1.[Gs] */
    mSetIdentity(identMatrix, ConfigData->numWheels, ConfigData->numWheels);
    mSubtract(identMatrix, ConfigData->numWheels, ConfigData->numWheels,    /* find ([I] - [Gs]^T.([Gs].[Gs]^T)^-1.[Gs]) */
              GsTemp, ConfigData->tau);

    /* Ensure that after a reset the output message is reset to zero. */
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
    
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    RWArrayTorqueIntMsg cntrRequest;        /*!< [Nm]  array of the RW motor torque solution vector from the control module */
	RWSpeedIntMsg rwSpeeds;                 /*!< [r/s] array of RW speeds */
	RWArrayTorqueIntMsg finalControl;       /*!< [Nm]  array of final RW motor torques containing both
                                                       the control and null motion torques */
	double dVector[MAX_EFF_CNT];            /*!< [Nm]  null motion wheel speed control array */
    
    /* Begin method steps*/
    /* - Read the input RW commands to get the raw RW requests*/
    ReadMessage(ConfigData->inputRWCmdsID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(RWArrayTorqueIntMsg), (void*) &(cntrRequest), moduleID);
	ReadMessage(ConfigData->inputSpeedsID, &timeOfMsgWritten, &sizeOfMsgWritten,
		sizeof(RWSpeedIntMsg), (void*)&(rwSpeeds), moduleID);

    /* zero the output message */
	memset(&finalControl, 0x0, sizeof(RWArrayTorqueIntMsg));
    /* compute the wheel speed control vector d = -K.Omega */
	vScale(-ConfigData->OmegaGain, rwSpeeds.wheelSpeeds,
		ConfigData->numWheels, dVector);
    /* compute the RW null space motor torque solution to reduce the wheel speeds */
	mMultV(ConfigData->tau, ConfigData->numWheels, ConfigData->numWheels,
		dVector, finalControl.motorTorque);
    /* add the null motion RW torque solution to the RW feedback control torque solution */
	vAdd(finalControl.motorTorque, ConfigData->numWheels,
		cntrRequest.motorTorque, finalControl.motorTorque);


	WriteMessage(ConfigData->outputMsgID, callTime, sizeof(RWArrayTorqueIntMsg),
		&finalControl, moduleID);

    return;
}
