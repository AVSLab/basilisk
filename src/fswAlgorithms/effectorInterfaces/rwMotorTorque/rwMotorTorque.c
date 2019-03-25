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

/* modify the path to reflect the new module names */
#include "effectorInterfaces/rwMotorTorque/rwMotorTorque.h"

/* update this include to reflect the required module input messages */
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>


/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/bsk_Print.h"

/*! This method initializes the configData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
 */
void SelfInit_rwMotorTorque(rwMotorTorqueConfig *configData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Create output message for module */
    configData->outputMsgID = CreateNewMessage(configData->outputDataName,
                                               sizeof(RWArrayTorqueIntMsg),
                                               "RWArrayTorqueIntMsg",          /* add the output structure name */
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
    /*! - Get the input message ID's */
    configData->inputVehControlID = subscribeToMessage(configData->inputVehControlName,
                                                       sizeof(CmdTorqueBodyIntMsg), moduleID);
    configData->rwParamsInMsgID = subscribeToMessage(configData->rwParamsInMsgName,
                                                     sizeof(RWArrayConfigFswMsg), moduleID);
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
    int i;
    
    /* configure the number of axes that are controlled */
    double *pAxis;                 /*!< pointer to the current control axis */
    configData->numControlAxes = 0;
    for (i = 0; i < 3; i++)
    {
        pAxis = configData->controlAxes_B + 3 * configData->numControlAxes;
        if (v3Norm(pAxis) > 0.0) {
            configData->numControlAxes += 1;
        }
    }
    if (configData->numControlAxes == 0) {
        BSK_PRINT(MSG_WARNING,"rwMotorTorque() is not setup to control any axes!\n");
    }
    
    /*! - Read static RW config data message and store it in module variables */
    ReadMessage(configData->rwParamsInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(RWArrayConfigFswMsg), &(configData->rwConfigParams), moduleID);
    
    if (configData->rwAvailInMsgID < 0){
        /* If no info is provided about RW availability we'll assume that all are available */
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
    /*! Begin method steps*/
    RWAvailabilityFswMsg wheelsAvailability;
    memset(wheelsAvailability.wheelAvailability, 0x0, MAX_EFF_CNT * sizeof(int)); // wheelAvailability set to 0 (AVAILABLE) by default
    memset(&wheelsAvailability, 0x0, sizeof(RWAvailabilityFswMsg)); // Uncertain if this would provide the same functionality.
    
    int i,j,k;
    double Lr_B[3]; /*!< [Nm]    commanded ADCS control torque in body frame*/
    double Lr_C[3]; /*!< [Nm]    commanded ADCS control torque in control frame */
    double us[MAX_EFF_CNT]; /*!< [Nm]    commanded ADCS control torque projected onto RWs g_s-Frames */
    v3SetZero(Lr_B);
    v3SetZero(Lr_C);
    vSetZero(us, MAX_EFF_CNT);
    
    /*! - Read the input messages */
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    
    ReadMessage(configData->inputVehControlID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CmdTorqueBodyIntMsg), (void*) &(Lr_B), moduleID);
    
    /* If availability message is provided, reflect RW availability in Gs Matrix */
    /*TODO: Confirm that this is a sensible and efficent choice (copying in the vectors every update call,
     this feels like it should be a reset decision)*/
    if (configData->rwAvailInMsgID >= 0)
    {
        int numAvailRW = 0;
        ReadMessage(configData->rwAvailInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                    sizeof(RWAvailabilityFswMsg), &wheelsAvailability, moduleID);
        
        for (i = 0; i < configData->rwConfigParams.numRW; i++) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE)
            {
                v3Copy(&configData->rwConfigParams.GsMatrix_B[i * 3], &configData->GsMatrix_B[numAvailRW * 3]);
                numAvailRW += 1;
            }
            configData->numAvailRW = numAvailRW;
        }
    }
    
    /* Lr is assumed to be a positive torque onto the body */
    v3Scale(-1.0, Lr_B, Lr_B);
    
    /* compute [Lr_C] = [C]Lr */
    for (k = 0; k < configData->numControlAxes; k++){
        Lr_C[k] = v3Dot(configData->controlAxes_B + 3 * k, Lr_B);
    }
    
    /* compute [CGs] */
    double CGs[3][MAX_EFF_CNT];
    mSetZero(CGs, 3, MAX_EFF_CNT);
    for (i=0; i<configData->numControlAxes; i++) {
        for (j=0; j<configData->numAvailRW; j++) {
            CGs[i][j] = v3Dot(&configData->GsMatrix_B[j * 3], &configData->controlAxes_B[3 * i]);
        }
    }
    /* Compute minimum norm inverse for us = [CGs].T inv([CGs][CGs].T) [Lr_C] */
    /* Having at least the same # of RW as # of control axes is necessary condition to guarantee inverse matrix exists */
    /* If matrix to invert it not full rank, the control torque output is zero. */
    if (configData->numAvailRW >= configData->numControlAxes){
        double v3_temp[3]; /* inv([M]) [Lr_C] */
        v3SetZero(v3_temp);
        if (configData->numControlAxes == 3){
            double M33[3][3]; /* [M] = [CGs][CGs].T */
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
        } else if (configData->numControlAxes == 2){
            double M22[2][2];
            for (i=0; i<configData->numControlAxes; i++) {
                for (j=0; j<configData->numControlAxes; j++) {
                    M22[i][j] = 0.0;
                    for (k=0; k < configData->numAvailRW; k++) {
                        M22[i][j] += CGs[i][k]*CGs[j][k];
                    }
                }
            }
            m22Inverse(M22, M22);
            m22MultV2(M22, Lr_C, v3_temp);
        } else if (configData->numControlAxes == 1){
            double M11[1][1];
            for (i=0; i<configData->numControlAxes; i++) {
                for (j=0; j<configData->numControlAxes; j++) {
                    M11[i][j] = 0.0;
                    for (k=0; k < configData->numAvailRW; k++) {
                        M11[i][j] += CGs[i][k]*CGs[j][k];
                    }
                }
            }
            v3_temp[0] = 1.0 / M11[0][0] * Lr_C[0];
        }
        /* compute the RW motor torques */
        /* us = [CGs].T v3_temp */
        double us_avail[MAX_EFF_CNT];
        vSetZero(us_avail, MAX_EFF_CNT);
        for (i=0; i<configData->numAvailRW; i++) {
            for (j=0; j<configData->numControlAxes; j++) {
                us_avail[i] += CGs[j][i] * v3_temp[j];
            }
        }
        int i_torque = 0;
        for (i = 0; i < configData->rwConfigParams.numRW; i++) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE)
            {
                us[i] = us_avail[i_torque];
                i_torque += 1;
            }
        }
    }
    
    /* store the output message */
    mCopy(us, configData->rwConfigParams.numRW, 1, configData->rwMotorTorques.motorTorque);
    WriteMessage(configData->outputMsgID, callTime, sizeof(RWArrayTorqueIntMsg),
                 (void*) &(configData->rwMotorTorques), moduleID);
    
    return;
}
