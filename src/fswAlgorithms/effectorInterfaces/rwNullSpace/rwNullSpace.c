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

/*!
 \verbatim embed:rst
    This method creates the module output message of type :ref:`RWArrayTorqueIntMsg`.
 \endverbatim
 @return void
 @param configData The configuration data associated with RW null space model
 @param moduleID The ID associated with the configData
 */
void SelfInit_rwNullSpace(rwNullSpaceConfig *configData, int64_t moduleID)
{
    /* Create output message for module */
    configData->outputMsgID = CreateNewMessage(
        configData->outputControlName, sizeof(RWArrayTorqueIntMsg),
        "RWArrayTorqueIntMsg", moduleID);
	
}

/*! This method performs the second stage of initialization for the RW null space control
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param configData The configuration data associated with the sun safe ACS control
 @param moduleID The ID associated with the configData
 */
void CrossInit_rwNullSpace(rwNullSpaceConfig *configData, int64_t moduleID)
{
    configData->inputRWCmdsID = subscribeToMessage(configData->inputRWCommands,
        sizeof(RWArrayTorqueIntMsg), moduleID);
	configData->inputSpeedsID = subscribeToMessage(configData->inputRWSpeeds,
		sizeof(RWSpeedIntMsg), moduleID);
    configData->inputRWConfID = subscribeToMessage(configData->inputRWConfigData,
        sizeof(RWConstellationFswMsg), moduleID);

}

/*! @brief This resets the module to original states by reading in the RW configuration messages and recreating any module specific variables.  The output message is reset to zero.
    @return void
    @param configData The configuration data associated with the null space control
    @param callTime The clock time at which the function was called (nanoseconds)
    @param moduleID The ID associated with the configData
 */
void Reset_rwNullSpace(rwNullSpaceConfig *configData, uint64_t callTime,
                        int64_t moduleID)
{
    double GsMatrix[3*MAX_EFF_CNT];                 /* [-]  [Gs] projection matrix where gs_hat_B RW spin axis form each colum */
    double GsTranspose[3 * MAX_EFF_CNT];            /* [-]  [Gs]^T */
    double GsInvHalf[3 * 3];                        /* [-]  ([Gs][Gs]^T)^-1 */
    double identMatrix[MAX_EFF_CNT*MAX_EFF_CNT];    /* [-]  [I_NxN] identity matrix */
    double GsTemp[MAX_EFF_CNT*MAX_EFF_CNT];         /* [-]  temp matrix */
    RWConstellationFswMsg localRWData;              /*      local copy of RW configuration data */
    int i, j;
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;

    memset(&localRWData, 0x0, sizeof(RWConstellationFswMsg));
    /*! -# read in the RW spin axis headings */
    ReadMessage(configData->inputRWConfID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(RWConstellationFswMsg), (void *) &localRWData, moduleID);

    /*! -# create the 3xN [Gs] RW spin axis projection matrix */
    configData->numWheels = (uint32_t) localRWData.numRW;
    for(i=0; i<configData->numWheels; i=i+1)
    {
        for(j=0; j<3; j=j+1)
        {
            GsMatrix[j*(int) configData->numWheels+i] = localRWData.reactionWheels[i].gsHat_B[j];
        }
    }

    /*! -# find the [tau] null space projection matrix [tau]= ([I] - [Gs]^T.([Gs].[Gs]^T) */
    mTranspose(GsMatrix, 3, configData->numWheels, GsTranspose);            /* find [Gs]^T */
    mMultM(GsMatrix, 3, configData->numWheels, GsTranspose,
           configData->numWheels, 3, GsInvHalf);                            /* find [Gs].[Gs]^T */
    m33Inverse(RECAST3X3 GsInvHalf, RECAST3X3 GsInvHalf);                   /* find ([Gs].[Gs]^T)^-1 */
    mMultM(GsInvHalf, 3, 3, GsMatrix, 3, configData->numWheels,
           configData->tau);                                                /* find ([Gs].[Gs]^T)^-1.[Gs] */
    mMultM(GsTranspose, configData->numWheels, 3, configData->tau, 3,
           configData->numWheels, GsTemp);                                  /* find [Gs]^T.([Gs].[Gs]^T)^-1.[Gs] */
    mSetIdentity(identMatrix, configData->numWheels, configData->numWheels);
    mSubtract(identMatrix, configData->numWheels, configData->numWheels,    /* find ([I] - [Gs]^T.([Gs].[Gs]^T)^-1.[Gs]) */
              GsTemp, configData->tau);

}

/*! This method takes the input reaction wheel commands as well as the observed 
    reaction wheel speeds and balances the commands so that the overall vehicle 
	momentum is minimized.
 @return void
 @param configData The configuration data associated with the null space control
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Update_rwNullSpace(rwNullSpaceConfig *configData, uint64_t callTime,
    int64_t moduleID)
{
    
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    RWArrayTorqueIntMsg cntrRequest;        /* [Nm]  array of the RW motor torque solution vector from the control module */
	RWSpeedIntMsg rwSpeeds;                 /* [r/s] array of RW speeds */
	RWArrayTorqueIntMsg finalControl;       /* [Nm]  array of final RW motor torques containing both
                                                       the control and null motion torques */
	double dVector[MAX_EFF_CNT];            /* [Nm]  null motion wheel speed control array */
    
    /*! - zero all message containers prior to evaluation */
    memset(&finalControl, 0x0, sizeof(RWArrayTorqueIntMsg));
    memset(&cntrRequest, 0x0, sizeof(RWArrayTorqueIntMsg));
    memset(&rwSpeeds, 0x0, sizeof(RWSpeedIntMsg));


    /*! - Read the input RW commands to get the raw RW requests*/
    ReadMessage(configData->inputRWCmdsID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(RWArrayTorqueIntMsg), (void*) &(cntrRequest), moduleID);
    /*! - Read the RW speeds*/
	ReadMessage(configData->inputSpeedsID, &timeOfMsgWritten, &sizeOfMsgWritten,
		sizeof(RWSpeedIntMsg), (void*)&(rwSpeeds), moduleID);

    /*! - compute the wheel speed control vector d = -K.Omega */
	vScale(-configData->OmegaGain, rwSpeeds.wheelSpeeds,
		configData->numWheels, dVector);
    /*! - compute the RW null space motor torque solution to reduce the wheel speeds */
	mMultV(configData->tau, configData->numWheels, configData->numWheels,
		dVector, finalControl.motorTorque);
    /*! - add the null motion RW torque solution to the RW feedback control torque solution */
	vAdd(finalControl.motorTorque, configData->numWheels,
		cntrRequest.motorTorque, finalControl.motorTorque);

    /*! - write the final RW torque solution to the output message */
	WriteMessage(configData->outputMsgID, callTime, sizeof(RWArrayTorqueIntMsg),
		&finalControl, moduleID);

    return;
}
