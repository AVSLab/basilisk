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
    MRP_FEEDBACK Module
 
 */

#include "attControl/MRP_Feedback/MRP_Feedback.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "simulation/utilities/astroConstants.h"
#include "fswMessages/rwAvailabilityFswMsg.h"
#include "simulation/utilities/bsk_Print.h"

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
        sizeof(CmdTorqueBodyIntMsg), "CmdTorqueBodyIntMsg", moduleID);
    
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
                                                 sizeof(AttGuidFswMsg), moduleID);
    ConfigData->vehConfigInMsgID = subscribeToMessage(ConfigData->vehConfigInMsgName,
                                                 sizeof(VehicleConfigFswMsg), moduleID);
    
    ConfigData->rwParamsInMsgID = -1;
    ConfigData->inputRWSpeedsID = -1;
    ConfigData->rwAvailInMsgID = -1;

    if(strlen(ConfigData->rwParamsInMsgName) > 0) {
        ConfigData->rwParamsInMsgID = subscribeToMessage(ConfigData->rwParamsInMsgName,
                                                       sizeof(RWArrayConfigFswMsg), moduleID);
        if (strlen(ConfigData->inputRWSpeedsName) > 0) {
        ConfigData->inputRWSpeedsID = subscribeToMessage(ConfigData->inputRWSpeedsName,
                                                         sizeof(RWSpeedIntMsg), moduleID);
        } else {
            BSK_PRINT(MSG_ERROR, "Error: the inputRWSpeedsName wasn't set while rwParamsInMsgName was set.\n");
        }
        if(strlen(ConfigData->rwAvailInMsgName) > 0) {
            ConfigData->rwAvailInMsgID = subscribeToMessage(ConfigData->rwAvailInMsgName,
                                                             sizeof(RWAvailabilityFswMsg), moduleID);
        }
    }
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_MRP_Feedback(MRP_FeedbackConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read the input messages */
    uint64_t clockTime;
    uint32_t readSize;
    int i;    

    VehicleConfigFswMsg sc;
    ReadMessage(ConfigData->vehConfigInMsgID, &clockTime, &readSize,
                sizeof(VehicleConfigFswMsg), (void*) &(sc), moduleID);
    for (i=0; i < 9; i++){
        ConfigData->ISCPntB_B[i] = sc.ISCPntB_B[i];
    };
    
    ConfigData->rwConfigParams.numRW = 0;
    if (ConfigData->rwParamsInMsgID >= 0) {
        /*! - Read static RW config data message and store it in module variables*/
        ReadMessage(ConfigData->rwParamsInMsgID, &clockTime, &readSize,
                    sizeof(RWArrayConfigFswMsg), &(ConfigData->rwConfigParams), moduleID);
    }
    
    /* Reset the integral measure of the rate tracking error */
    v3SetZero(ConfigData->z);
    v3SetZero(ConfigData->int_sigma);
    /* Reset the prior time flag state. 
     If zero, control time step not evaluated on the first function call */
    ConfigData->priorTime = 0;
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
    AttGuidFswMsg      guidCmd;            /*!< Guidance Message */
    RWSpeedIntMsg      wheelSpeeds;        /*!< Reaction wheel speed estimates */
    RWAvailabilityFswMsg  wheelsAvailability; /*!< Reaction wheel availability */

    uint64_t            clockTime;
    uint32_t            readSize;
    double              dt;                 /*!< [s] control update period */
    double              Lr[3];              /*!< required control torque vector [Nm] */
    double              omega_BN_B[3];
    double              v3[3];
    double              v3_1[3];
    double              v3_2[3];
    double              temp;
    int                 i;
    double              *wheelGs;           /*!< Reaction wheel spin axis pointer */



    /*! Begin method steps*/
    /*! - Read the dynamic input messages */
    memset(&guidCmd, 0x0, sizeof(AttGuidFswMsg));
    ReadMessage(ConfigData->inputGuidID, &clockTime, &readSize,
                sizeof(AttGuidFswMsg), (void*) &(guidCmd), moduleID);
    
    memset(wheelSpeeds.wheelSpeeds, 0x0, MAX_EFF_CNT * sizeof(double));
    memset(wheelsAvailability.wheelAvailability, 0x0, MAX_EFF_CNT * sizeof(int)); // wheelAvailability set to 0 (AVAILABLE) by default
    if(ConfigData->rwConfigParams.numRW > 0) {
        ReadMessage(ConfigData->inputRWSpeedsID, &clockTime, &readSize,
                    sizeof(RWSpeedIntMsg), (void*) &(wheelSpeeds), moduleID);
        if (ConfigData->rwAvailInMsgID >= 0){
            ReadMessage(ConfigData->rwAvailInMsgID, &clockTime, &readSize,
                        sizeof(RWAvailabilityFswMsg), &wheelsAvailability, moduleID);
        }
    }
    
    /* compute control update time */
    if (ConfigData->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = (callTime - ConfigData->priorTime) * NANO2SEC;
    }
    ConfigData->priorTime = callTime;

    /* compute body rate */
    v3Add(guidCmd.omega_BR_B, guidCmd.omega_RN_B, omega_BN_B);
    
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

    v3Add(ConfigData->knownTorquePntB_B, Lr, Lr);                                       /* +L */
    v3Scale(-1.0, Lr, Lr);                                  /* compute the net positive control torque onto the spacecraft */


    /* store the output message */
    v3Copy(Lr, ConfigData->controlOut.torqueRequestBody);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(CmdTorqueBodyIntMsg),
                 (void*) &(ConfigData->controlOut), moduleID);
    
    return;
}

