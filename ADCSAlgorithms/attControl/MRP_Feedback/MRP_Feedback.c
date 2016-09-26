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
#include "effectorInterfaces/_GeneralModuleFiles/rwDeviceStates.h"

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
    ConfigData->vehConfigInMsgID = subscribeToMessage(ConfigData->vehConfigInMsgName,
                                                 sizeof(vehicleConfigData), moduleID);

    if(strlen(ConfigData->rwParamsInMsgName) > 0) {
        ConfigData->rwParamsInMsgID = subscribeToMessage(ConfigData->rwParamsInMsgName,
                                                       sizeof(RWConfigParams), moduleID);
        ConfigData->inputRWSpeedsID = subscribeToMessage(ConfigData->inputRWSpeedsName,
                                                         sizeof(RWSpeedData), moduleID);
        ConfigData->inputRWsAvailID = subscribeToMessage(ConfigData->inputRWsAvailDataName,
                                                         sizeof(RWAvailabilityData), moduleID);
    } else {
        ConfigData->rwConfigParams.numRW = 0;
        ConfigData->rwParamsInMsgID = -1;
        ConfigData->inputRWSpeedsID = -1;
        ConfigData->inputRWsAvailID = -1;
    }
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_MRP_Feedback(MRP_FeedbackConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t clockTime;
    uint32_t readSize;
    vehicleConfigData sc;
    ReadMessage(ConfigData->vehConfigInMsgID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(sc), moduleID);
    for (int i=0; i < 9; i++){
        ConfigData->ISCPntB_B[i] = sc.ISCPntB_B[i];
    };
    
    /*! - Read the input messages */
    if (ConfigData->rwParamsInMsgID >= 0) {
        /*! - Read static RW config data message and store it in module variables*/
        ReadMessage(ConfigData->rwParamsInMsgID, &clockTime, &readSize,
                    sizeof(RWConfigParams), &(ConfigData->rwConfigParams), moduleID);
    }
    
    /* Reset the integral measure of the rate tracking error */
    v3SetZero(ConfigData->z);
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
    //vehicleConfigData   sc;                 /*!< spacecraft configuration message */
    RWSpeedData         wheelSpeeds;        /*!< Reaction wheel speed estimates */
    RWAvailabilityData  wheelsAvailability; /*!< Reaction wheel availability */

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



    /*! Begin method steps*/
    /*! - Read the dynamic input messages */
    ReadMessage(ConfigData->inputGuidID, &clockTime, &readSize,
                sizeof(attGuidOut), (void*) &(guidCmd), moduleID);
    if(ConfigData->rwConfigParams.numRW > 0) {
        ReadMessage(ConfigData->inputRWSpeedsID, &clockTime, &readSize,
                    sizeof(RWSpeedData), (void*) &(wheelSpeeds), moduleID);
        /* #TODO: Do something with availability information */
        ReadMessage(ConfigData->inputRWsAvailID, &clockTime, &readSize,
                    sizeof(RWAvailabilityData), &wheelsAvailability, moduleID);
    }
    
    /* compute control update time */
    if (ConfigData->priorTime != 0) {       /* don't compute dt if this is the first call after a reset */
        dt = (callTime - ConfigData->priorTime) * NANO2SEC;
        if (dt > 10.0) dt = 10.0;           /* cap the maximum control time step possible */
        if (dt < 0.0) dt = 0.0;             /* ensure no negative numbers are used */
    } else {
        dt = 0.;                            /* set dt to zero to not use integration on first function call */
    }
    ConfigData->priorTime = callTime;

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
        m33MultV3(RECAST3X3 ConfigData->ISCPntB_B, v3, v3_1);
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

    m33MultV3(RECAST3X3 ConfigData->ISCPntB_B, omega_BN_B, v3);                    /* -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s) */
    for(i = 0; i < ConfigData->rwConfigParams.numRW; i++)
    {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE){ /* check if wheel is available */
            wheelGs = &(ConfigData->rwConfigParams.GsMatrix_B[i*3]);
            v3Scale(ConfigData->rwConfigParams.JsList[i] * (v3Dot(omega_BN_B, wheelGs) + wheelSpeeds.wheelSpeeds[i]),
                    wheelGs, v3_1);
            v3Add(v3_1, v3, v3);
        }
    }
    
    
    v3Add(guidCmd.omega_RN_B, v3_2, v3_2);
    v3Cross(v3_2, v3, v3_1);
    v3Subtract(Lr, v3_1, Lr);

    v3Cross(omega_BN_B, guidCmd.omega_RN_B, v3);
    v3Subtract(v3, guidCmd.domega_RN_B, v3_1);
    m33MultV3(RECAST3X3 ConfigData->ISCPntB_B, v3_1, v3);                    /* +[I](-d(omega_r)/dt + omega x omega_r) */
    v3Add(v3, Lr, Lr);

    v3Add(L, Lr, Lr);                                       /* +L */
    v3Scale(-1.0, Lr, Lr);                                  /* compute the net positive control torque onto the spacecraft */


    /*
     store the output message 
     */
    v3Copy(Lr, ConfigData->controlOut.torqueRequestBody);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehControlOut),
                 (void*) &(ConfigData->controlOut), moduleID);
    
    return;
}

