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

/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(RWArrayTorqueIntMsg),
                                               "RWArrayTorqueIntMsg",          /* add the output structure name */
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the input message ID's */
    ConfigData->inputVehControlID = subscribeToMessage(ConfigData->inputVehControlName,
                                                       sizeof(CmdTorqueBodyIntMsg), moduleID);
    ConfigData->rwParamsInMsgID = subscribeToMessage(ConfigData->rwParamsInMsgName,
                                                     sizeof(RWArrayConfigFswMsg), moduleID);
    ConfigData->rwAvailInMsgID = -1;
    if (strlen(ConfigData->rwAvailInMsgName) > 0){
        ConfigData->rwAvailInMsgID = subscribeToMessage(ConfigData->rwAvailInMsgName,
                                                         sizeof(RWAvailabilityFswMsg), moduleID);
    }
    
    /* configure the number of axes that are controlled */
    double *pAxis;                 /*!< pointer to the current control axis */
    int i;
    ConfigData->numControlAxes = 0;
    for (i = 0; i < 3; i++)
    {
        pAxis = ConfigData->controlAxes_B + 3 * ConfigData->numControlAxes;
        if (v3Norm(pAxis) > 0.0) {
            ConfigData->numControlAxes += 1;
        }
    }
    if (ConfigData->numControlAxes == 0) {
        BSK_PRINT(MSG_WARNING,"rwMotorTorque() is not setup to control any axes!\n");
    }

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t clockTime;
    uint32_t readSize;
    int i;
    /*! - Read static RW config data message and store it in module variables */
    ReadMessage(ConfigData->rwParamsInMsgID, &clockTime, &readSize,
                sizeof(RWArrayConfigFswMsg), &(ConfigData->rwConfigParams), moduleID);
    
    if (ConfigData->rwAvailInMsgID < 0){
        /* If no info is provided about RW availability we'll assume that all are available */
        ConfigData->numAvailRW =ConfigData->rwConfigParams.numRW;
        for (i = 0; i < ConfigData->rwConfigParams.numRW; i++){
            v3Copy(&ConfigData->rwConfigParams.GsMatrix_B[i * 3], &ConfigData->GsMatrix_B[i * 3]);
        }
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! Begin method steps*/
    int i,j,k;
    
    double us[MAX_EFF_CNT];
    RWAvailabilityFswMsg wheelsAvailability;
    memset(us, 0x0, MAX_EFF_CNT * sizeof(double));
    memset(wheelsAvailability.wheelAvailability, 0x0, MAX_EFF_CNT * sizeof(int)); // wheelAvailability set to 0 (AVAILABLE) by default
    
    /*! - Read the input messages */
    uint64_t clockTime;
    uint32_t readSize;
    double Lr_B[3]; /*!< [Nm]    commanded ADCS control torque */
    ReadMessage(ConfigData->inputVehControlID, &clockTime, &readSize,
                sizeof(CmdTorqueBodyIntMsg), (void*) &(Lr_B), moduleID);
    if (ConfigData->rwAvailInMsgID >= 0)
    {
        ReadMessage(ConfigData->rwAvailInMsgID, &clockTime, &readSize,
                    sizeof(RWAvailabilityFswMsg), &wheelsAvailability, moduleID);
        int numAvailRW = 0;
        for (i = 0; i < ConfigData->rwConfigParams.numRW; i++) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE)
            {
                v3Copy(&ConfigData->rwConfigParams.GsMatrix_B[i * 3], &ConfigData->GsMatrix_B[numAvailRW * 3]);
                numAvailRW += 1;
            }
            ConfigData->numAvailRW = numAvailRW;
        }
    }
    
    /* Lr is assumed to be a positive torque onto the body */
    v3Scale(-1.0, Lr_B, Lr_B);
    /* compute [CLr] = [C]Lr */
    double CLr[3];
    for (k = 0; k < ConfigData->numControlAxes; k++){
        CLr[k] = v3Dot(ConfigData->controlAxes_B + 3 * k, Lr_B);
    }
    /* compute [CGs] */
    double CGs[3][MAX_EFF_CNT];
    for (i=0; i<ConfigData->numControlAxes; i++) {
        for (j=0; j<ConfigData->numAvailRW; j++) {
            CGs[i][j] = v3Dot(&ConfigData->GsMatrix_B[j * 3], &ConfigData->controlAxes_B[3 * i]);
        }
    }
    /* Compute minimum norm inverse for us = [CGs].T inv([CGs][CGs].T) [CLr] */
    /* Having at least the same # of RW as # of control axes is necessary condition to guarantee inverse matrix exists */
    /* If matrix to invert it not full rank, the control torque output is zero. */
    if (ConfigData->numAvailRW >= ConfigData->numControlAxes){
        /* [M] = [CGs][CGs].T */
        /* v3_temp = inv([M]) [CLr] */
        int i,j,k;
        double v3_temp[3];
        v3SetZero(v3_temp);
        if (ConfigData->numControlAxes == 3){
            double M33[3][3];
            for (i=0; i<ConfigData->numControlAxes; i++) {
                for (j=0; j<ConfigData->numControlAxes; j++) {
                    M33[i][j] = 0.0;
                    for (k=0; k < ConfigData->numAvailRW; k++) {
                        M33[i][j] += CGs[i][k]*CGs[j][k];
                    }
                }
            }
            m33Inverse(M33, M33);
            m33MultV3(M33, CLr, v3_temp);
        } else if (ConfigData->numControlAxes == 2){
            double M22[2][2];
            for (i=0; i<ConfigData->numControlAxes; i++) {
                for (j=0; j<ConfigData->numControlAxes; j++) {
                    M22[i][j] = 0.0;
                    for (k=0; k < ConfigData->numAvailRW; k++) {
                        M22[i][j] += CGs[i][k]*CGs[j][k];
                    }
                }
            }
            m22Inverse(M22, M22);
            m22MultV2(M22, CLr, v3_temp);
        } else if (ConfigData->numControlAxes == 1){
            double M11[1][1];
            for (i=0; i<ConfigData->numControlAxes; i++) {
                for (j=0; j<ConfigData->numControlAxes; j++) {
                    M11[i][j] = 0.0;
                    for (k=0; k < ConfigData->numAvailRW; k++) {
                        M11[i][j] += CGs[i][k]*CGs[j][k];
                    }
                }
            }
            v3_temp[0] = 1.0 / M11[0][0] * CLr[0];
        }
        /* compute the RW motor torques */
        /* us = [CGs].T v3_temp */
        double us_avail[MAX_EFF_CNT];
        memset(us_avail, 0x0, MAX_EFF_CNT * sizeof(double));
        for (i=0; i<ConfigData->numAvailRW; i++) {
            for (j=0; j<ConfigData->numControlAxes; j++) {
                us_avail[i] += CGs[j][i] * v3_temp[j];
            }
        }
        int i_torque = 0;
        for (i = 0; i < ConfigData->rwConfigParams.numRW; i++) {
            if (wheelsAvailability.wheelAvailability[i] == AVAILABLE)
            {
                us[i] = us_avail[i_torque];
                i_torque += 1;
            }
        }
    }

    /* store the output message */
    mCopy(us, ConfigData->rwConfigParams.numRW, 1, ConfigData->rwMotorTorques.motorTorque);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(RWArrayTorqueIntMsg),
                 (void*) &(ConfigData->rwMotorTorques), moduleID);

    return;
}
