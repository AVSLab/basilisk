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
/*
    MRP_FEEDBACK Module
 
 */

#include "attControl/MRP_Feedback/MRP_Feedback.h"
#include "attGuidance/_GeneralModuleFiles/attGuidOut.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include "SimCode/utilities/astroConstants.h"
#include "effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_MRP_Feedback(MRP_FeedbackConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
        sizeof(vehControlOut), "vehControlOut", moduleID);
    
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_MRP_Feedback(MRP_FeedbackConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputGuidID = subscribeToMessage(ConfigData->inputGuidName,
                                                 sizeof(attGuidOut), moduleID);
    ConfigData->inputVehicleConfigDataID = subscribeToMessage(ConfigData->inputVehicleConfigDataName,
                                                              sizeof(vehicleConfigData), moduleID);

    if(strlen(ConfigData->inputRWConfigData) > 0) {
        ConfigData->inputRWConfID = subscribeToMessage(ConfigData->inputRWConfigData,
                                                       sizeof(RWConstellation), moduleID);
        ConfigData->inputRWSpeedsID = subscribeToMessage(ConfigData->inputRWSpeedsName,
                                                         sizeof(RWSpeedData), moduleID);
    } else {
        ConfigData->numRW = 0;
        ConfigData->inputRWConfID = -1;
        ConfigData->inputRWSpeedsID = -1;
    }
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_MRP_Feedback(MRP_FeedbackConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    RWConstellation localRWData;
    int i;
    uint64_t clockTime;
    uint32_t readSize;
    vehicleConfigData   sc;                 /*!< spacecraft configuration message */

    if (ConfigData->inputRWConfID>0) {
        /*! - Read static RW config data message and store it in module variables*/
        ReadMessage(ConfigData->inputRWConfID, &clockTime, &readSize,
                    sizeof(RWConstellation), &localRWData, moduleID);
        ConfigData->numRW = localRWData.numRW;
        ReadMessage(ConfigData->inputVehicleConfigDataID, &clockTime, &readSize,
                    sizeof(vehicleConfigData), (void*) &(sc), moduleID);

        for(i=0; i<ConfigData->numRW; i=i+1)
        {
            ConfigData->JsList[i] = localRWData.reactionWheels[i].Js;
            m33MultV3(RECAST3X3 sc.BS,
                      localRWData.reactionWheels[i].gsHat_S,
                      &ConfigData->GsMatrix[i*3]);
        }
    }

    ConfigData->priorTime = 0;              /* reset the prior time flag state.  If set
                                             to zero, the control time step is not evaluated on the
                                             first function call */
    v3SetZero(ConfigData->z);               /* reset the integral measure of the rate tracking error */
    v3SetZero(ConfigData->int_sigma);
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_MRP_Feedback(MRP_FeedbackConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    attGuidOut          guidCmd;            /*!< Guidance Message */
    vehicleConfigData   sc;                 /*!< spacecraft configuration message */
    RWSpeedData         wheelSpeeds;        /*!< Reaction wheel speed estimates */
    uint64_t            clockTime;
    uint32_t            readSize;
    double              dt;                 /*!< [s] control update period */
    double              Lr[3];              /*!< required control torque vector [Nm] */
    double              L[3];               /*!< known external torque */
    double              omega_BN_B[3];
    double              v3[3];
    double              v3_1[3];
    double              v3_2[3];
    double              temp;
    int                 i;
    double              *wheelGs;           /*!< Reaction wheel spin axis pointer */
    
    /* compute control update time */
    if (ConfigData->priorTime != 0) {       /* don't compute dt if this is the first call after a reset */
        dt = (callTime - ConfigData->priorTime)*NANO2SEC;
        if (dt > 10.0) dt = 10.0;           /* cap the maximum control time step possible */
        if (dt < 0.0) dt = 0.0;             /* ensure no negative numbers are used */
    } else {
        dt = 0.;                            /* set dt to zero to not use integration on first function call */
    }
    ConfigData->priorTime = callTime;


    /*! Begin method steps*/
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputGuidID, &clockTime, &readSize,
                sizeof(attGuidOut), (void*) &(guidCmd), moduleID);
    ReadMessage(ConfigData->inputVehicleConfigDataID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(sc), moduleID);
    if(ConfigData->numRW>0) {
        ReadMessage(ConfigData->inputRWSpeedsID, &clockTime, &readSize,
                    sizeof(RWSpeedData), (void*) &(wheelSpeeds), moduleID);
    }

    /* compute body rate */
    v3Add(guidCmd.omega_BR_B, guidCmd.omega_RN_B, omega_BN_B);
    
    /* compute known external torque */
    v3Set(0.0, 0.0, 0.0, L);

    /* evaluate integral term */
    if (ConfigData->Ki > 0) {   /* check if integral feedback is turned on  */
        v3Scale(ConfigData->K * dt, guidCmd.sigma_BR, v3);
        v3Add(v3, ConfigData->int_sigma, ConfigData->int_sigma);
        if((temp = v3Norm(ConfigData->int_sigma)) > ConfigData->integralLimit) {
            v3Scale(ConfigData->integralLimit / temp, ConfigData->int_sigma, ConfigData->int_sigma);
        }
        v3Subtract(guidCmd.omega_BR_B, ConfigData->domega0, v3);
        m33MultV3(RECAST3X3 sc.ISCPntB_B, v3, v3_1);
        v3Add(ConfigData->int_sigma, v3_1, ConfigData->z);
    } else {
        /* integral feedback is turned off through a negative gain setting */
        v3SetZero(ConfigData->z);
    }

    /* evaluate required attitude control torque Lr */
    v3Scale(ConfigData->K, guidCmd.sigma_BR, v3);           /* +K sigma_BR */
    v3Scale(ConfigData->P, guidCmd.omega_BR_B,
            Lr);                                            /* +P delta_omega */
    v3Add(v3, Lr, Lr);
    v3Scale(ConfigData->Ki, ConfigData->z, v3_2);
    v3Scale(ConfigData->P, v3_2, v3);                       /* +P*Ki*z */
    v3Add(v3, Lr, Lr);

    m33MultV3(RECAST3X3 sc.ISCPntB_B, omega_BN_B, v3);                    /* -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s) */
    for(i = 0; i < ConfigData->numRW; i++)
    {
        wheelGs = &(ConfigData->GsMatrix[i*3]);
        v3Scale(ConfigData->JsList[i] * (v3Dot(omega_BN_B, wheelGs) + wheelSpeeds.wheelSpeeds[i])
                , wheelGs, v3_1);
        v3Add(v3_1, v3, v3);
    }
    
    
    v3Add(guidCmd.omega_RN_B, v3_2, v3_2);
    v3Cross(v3_2, v3, v3_1);
    v3Subtract(Lr, v3_1, Lr);

    v3Cross(omega_BN_B, guidCmd.omega_RN_B, v3);
    v3Subtract(v3, guidCmd.domega_RN_B, v3_1);
    m33MultV3(RECAST3X3 sc.ISCPntB_B, v3_1, v3);                    /* +[I](-d(omega_r)/dt + omega x omega_r) */
    v3Add(v3, Lr, Lr);

    v3Add(L, Lr, Lr);                                       /* +L */


    /*
     store the output message 
     */
    v3Copy(Lr, ConfigData->controlOut.torqueRequestBody);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehControlOut),
                 (void*) &(ConfigData->controlOut), moduleID);
    
    return;
}

