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
/*
 Mapping required attitude control torque Lr to RW motor torques
 
 */

#include "effectorInterfaces/rwMotorTorque/rwMotorTorque.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/bsk_Print.h"

/*! This method creates the module output message.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
 */
void SelfInit_rwMotorTorque(rwMotorTorqueConfig *configData, uint64_t moduleID)
{
    /*! - Create commanded control torque output message for module */
    configData->outputMsgID = CreateNewMessage(configData->outputDataName,
                                               sizeof(RWArrayTorqueIntMsg),
                                               "RWArrayTorqueIntMsg",
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
 */
void CrossInit_rwMotorTorque(rwMotorTorqueConfig *configData, uint64_t moduleID)
{
    /*! - subscribe to the control vector Lr input message  */
    configData->controlTorqueInMsgID = subscribeToMessage(configData->inputVehControlName,
                                                       sizeof(CmdTorqueBodyIntMsg), moduleID);

    /*! - suscribe to the RW parameter message */
    configData->rwParamsInMsgID = subscribeToMessage(configData->rwParamsInMsgName,
                                                     sizeof(RWArrayConfigFswMsg), moduleID);

    /*! - If the rwAvailInMsgName is set, subscribe to the RW availability message */
    configData->rwAvailInMsgID = -1;
    if (strlen(configData->rwAvailInMsgName) > 0){
        configData->rwAvailInMsgID = subscribeToMessage(configData->rwAvailInMsgName,
                                                        sizeof(RWAvailabilityFswMsg), moduleID);
    }
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param moduleID The ID associated with the configData
 */
void Reset_rwMotorTorque(rwMotorTorqueConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    double *pAxis;                 /* pointer to the current control axis */
    int i;
    
    /*!- configure the number of axes that are controlled.
     This is determined by checking for a zero row to determinate search */
    configData->numControlAxes = 0;
    for (i = 0; i < 3; i++)
    {
        pAxis = configData->controlAxes_B + 3 * configData->numControlAxes;
        if (v3Norm(pAxis) > 0.0) {
            configData->numControlAxes += 1;
        }
    }
    if (configData->numControlAxes == 0) {
        BSK_PRINT(MSG_WARNING,"rwMotorTorque() is not setup to control any axes!");
    }
    
    /*! - Read static RW config data message and store it in module variables */
    memset(&(configData->rwConfigParams), 0x0, sizeof(RWArrayConfigFswMsg));
    ReadMessage(configData->rwParamsInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(RWArrayConfigFswMsg), &(configData->rwConfigParams), moduleID);
    
    /*! - If no info is provided about RW availability we'll assume that all are available
     and create the [Gs] projection matrix once */
    if (configData->rwAvailInMsgID < 0){
        configData->numAvailRW = configData->rwConfigParams.numRW;
        for (i = 0; i < configData->rwConfigParams.numRW; i++){
            v3Copy(&configData->rwConfigParams.GsMatrix_B[i * 3], &configData->GsMatrix_B[i * 3]);
        }
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Update_rwMotorTorque(rwMotorTorqueConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    RWAvailabilityFswMsg wheelsAvailability;    /* Msg containing RW availability */
    CmdTorqueBodyIntMsg LrInputMsg;             /* Msg containing Lr control torque */
    RWArrayTorqueIntMsg rwMotorTorques;         /* Msg struct to store the output message */
    int i,j,k;
    double Lr_B[3];                             /* [Nm]    commanded ADCS control torque in body frame*/
    double Lr_C[3];                             /* [Nm]    commanded ADCS control torque projected onto control axes */
    double us[MAX_EFF_CNT];                     /* [Nm]    commanded ADCS control torque projected onto RWs g_s-Frames */
    double CGs[3][MAX_EFF_CNT];                 /* []      projection matrix from gs_i onto control axes */

    /*! - zero control torque and RW motor torque variables */
    v3SetZero(Lr_C);
    vSetZero(us, MAX_EFF_CNT);
    memset(&wheelsAvailability, 0x0, sizeof(RWAvailabilityFswMsg)); // wheelAvailability set to 0 (AVAILABLE) by default
    
    /*! - Read the input messages */
    memset(&LrInputMsg, 0x0, sizeof(CmdTorqueBodyIntMsg));
    ReadMessage(configData->controlTorqueInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CmdTorqueBodyIntMsg), (void*) &(LrInputMsg), moduleID);
    v3Copy(LrInputMsg.torqueRequestBody, Lr_B);

    /*! - Check if RW availability message is available */
    if (configData->rwAvailInMsgID >= 0)
    {
        int numAvailRW = 0;

        /*! - Read in current RW availabilit Msg */
        ReadMessage(configData->rwAvailInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                    sizeof(RWAvailabilityFswMsg), &wheelsAvailability, moduleID);

        /*! - create the current [Gs] projection matrix with the available RWs */
        for (i = 0; i < configData->rwConfigParams.numRW; i++) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE)
            {
                v3Copy(&configData->rwConfigParams.GsMatrix_B[i * 3], &configData->GsMatrix_B[numAvailRW * 3]);
                numAvailRW += 1;
            }
        }
        /*! - update the number of currently available RWs */
        configData->numAvailRW = numAvailRW;
    }
    
    /*! - Lr is assumed to be a positive torque onto the body, the [Gs]us must generate -Lr */
    v3Scale(-1.0, Lr_B, Lr_B);
    
    /*! - compute [Lr_C] = [C]Lr */
    mMultV(configData->controlAxes_B, configData->numControlAxes, 3, Lr_B, Lr_C);

    /*! - compute [CGs] */
    mSetZero(CGs, 3, MAX_EFF_CNT);
    for (i=0; i<configData->numControlAxes; i++) {
        for (j=0; j<configData->numAvailRW; j++) {
            CGs[i][j] = v3Dot(&configData->GsMatrix_B[j * 3], &configData->controlAxes_B[3 * i]);
        }
    }
    /*! - Compute minimum norm inverse for us = [CGs].T inv([CGs][CGs].T) [Lr_C]
     Having at least the same # of RW as # of control axes is necessary condition to guarantee inverse matrix exists. If matrix to invert it not full rank, the control torque output is zero. */
    if (configData->numAvailRW >= configData->numControlAxes){
        double v3_temp[3]; /* inv([M]) [Lr_C] */
        double M33[3][3]; /* [M] = [CGs][CGs].T */
        double us_avail[MAX_EFF_CNT];   /* matrix of available RW motor torques */
        
        v3SetZero(v3_temp);
        mSetIdentity(M33, 3, 3);
        for (i=0; i<configData->numControlAxes; i++) {
            for (j=0; j<configData->numControlAxes; j++) {
                M33[i][j] = 0.0;
                for (k=0; k < configData->numAvailRW; k++) {
                    M33[i][j] += CGs[i][k]*CGs[j][k];
                }
            }
        }
        m33Inverse(M33, M33);
        m33MultV3(M33, Lr_C, v3_temp);

        /*! - compute the desired RW motor torques */
        /* us = [CGs].T v3_temp */
        vSetZero(us_avail, MAX_EFF_CNT);
        for (i=0; i<configData->numAvailRW; i++) {
            for (j=0; j<configData->numControlAxes; j++) {
                us_avail[i] += CGs[j][i] * v3_temp[j];
            }
        }
        
        /*! - map the desired RW motor torques to the available RWs */
        j = 0;
        for (i = 0; i < configData->rwConfigParams.numRW; i++) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE)
            {
                us[i] = us_avail[j];
                j += 1;
            }
        }
    }
    
    /* store the output message */
    memset(&(rwMotorTorques), 0x0, sizeof(RWArrayTorqueIntMsg));
    vCopy(us, configData->rwConfigParams.numRW, rwMotorTorques.motorTorque);
    WriteMessage(configData->outputMsgID, callTime, sizeof(RWArrayTorqueIntMsg),
                 (void*) &(rwMotorTorques), moduleID);
    
    return;
}
