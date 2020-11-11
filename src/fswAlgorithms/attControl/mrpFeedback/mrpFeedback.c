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

#include "attControl/mrpFeedback/mrpFeedback.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "utilities/macroDefinitions.h"
#include "simulation/utilities/astroConstants.h"
#include "fswMessages/rwAvailabilityFswMsg.h"

#include <string.h>
#include <math.h>

/*! @brief This method sets up the module output message of type `CmdTorqueBodyMsg`
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
*/
void SelfInit_mrpFeedback(mrpFeedbackConfig *configData, int64_t moduleID)
{
    /*! - Create output message for module */
    CmdTorqueBodyMsg_C_addAuthor(&configData->cmdTorqueOutMsg, &configData->cmdTorqueOutMsg);

}

/*! @brief This method performs the second stage of initialization for this module.
 Its primary function is to link the input messages that were created elsewhere.  The required
 input messages are the attitude tracking error message of type ``AttGuidMsg``
 and the vehicle configuration message of type ``VehicleConfigMsg``.
 Optional messages are the RW configuration message of type ``RWArrayConfigFswMsg``,
 the RW speed message of type `RWSpeedIntMsg`
 and the RW availability message of type :`RWAvailabilityFswMsg`.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
*/
void CrossInit_mrpFeedback(mrpFeedbackConfig *configData, int64_t moduleID)
{
    /*! - Get the control data message ID*/
    configData->rwParamsInMsgId = -1;
    configData->rwSpeedsInMsgId = -1;
    configData->rwAvailInMsgId = -1;

    if(strlen(configData->rwParamsInMsgName) > 0) {
        configData->rwParamsInMsgId = subscribeToMessage(configData->rwParamsInMsgName,
                                                       sizeof(RWArrayConfigFswMsg), moduleID);
        if (strlen(configData->inputRWSpeedsName) > 0) {
        configData->rwSpeedsInMsgId = subscribeToMessage(configData->inputRWSpeedsName,
                                                         sizeof(RWSpeedIntMsg), moduleID);
        } else {
            _bskLog(configData->bskLogger, BSK_ERROR, "Error: the inputRWSpeedsName wasn't set while rwParamsInMsgName was set.");
        }
        if(strlen(configData->rwAvailInMsgName) > 0) {
            configData->rwAvailInMsgId = subscribeToMessage(configData->rwAvailInMsgName,
                                                             sizeof(RWAvailabilityFswMsg), moduleID);
        }
    }
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param configData The configuration data associated with the MRP steering control
 @param moduleID The module identifier
*/
void Reset_mrpFeedback(mrpFeedbackConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /* - Read the input messages */
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    int i;

    /*! - zero and read in vehicle configuration message */
    VehicleConfigMsgPayload sc;
    memset(&sc, 0x0, sizeof(VehicleConfigMsgPayload));
    sc = VehicleConfigMsg_C_read(&configData->vehConfigInMsg);
    /*! - copy over spacecraft inertia tensor */
    for (i=0; i < 9; i++){
        configData->ISCPntB_B[i] = sc.ISCPntB_B[i];
    };

    /*! - zero the number of RW by default */
    configData->rwConfigParams.numRW = 0;

    /*! - check if RW configuration message exists */
    if (configData->rwParamsInMsgId >= 0) {
        /*! - Zero and Read static RW config data message and store it in module variables*/
        memset(&(configData->rwConfigParams), 0x0, sizeof(RWArrayConfigFswMsg));
        ReadMessage(configData->rwParamsInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                    sizeof(RWArrayConfigFswMsg), &(configData->rwConfigParams), moduleID);
    }

    /*! - Reset the integral measure of the rate tracking error */
    v3SetZero(configData->int_sigma);

    /*! - Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    configData->priorTime = 0;
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param configData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_mrpFeedback(mrpFeedbackConfig *configData, uint64_t callTime,
    int64_t moduleID)
{
    AttGuidMsgPayload      guidCmd;            /* attitude tracking error message */
    RWSpeedIntMsg      wheelSpeeds;        /* Reaction wheel speed message */
    RWAvailabilityFswMsg wheelsAvailability; /* Reaction wheel availability message */
    CmdTorqueBodyMsgPayload controlOut;        /* output message */

    uint64_t            timeOfMsgWritten;
    uint32_t            sizeOfMsgWritten;
    double              dt;                 /* [s] control update period */
    double              Lr[3];              /* required control torque vector [Nm] */
    double              omega_BN_B[3];      /* [r/s] body angular velocity message */
    double              v3_1[3];
    double              v3_2[3];
    double              v3_3[3];
    double              v3_4[3];
    double              v3_5[3];
    double              v3_6[3];
    double              v3_7[3];
    double              v3_8[3];
    double              v3_9[3];
    double              v3_10[3];
    double              v3_11[3];
    double              v3_12[3];
    double              intCheck;           /* Check magnitude of integrated attitude error */
    int                 i;
    double              *wheelGs;           /* Reaction wheel spin axis pointer */

    /*! - zero the output message */
    memset(&controlOut, 0x0, sizeof(CmdTorqueBodyMsgPayload));

    /*! - Read the attitude tracking error message */
    guidCmd = AttGuidMsg_C_read(&configData->guidInMsg);

    /*! - read in optional RW speed and availability message */
    if(configData->rwConfigParams.numRW > 0) {
        memset(&wheelSpeeds, 0x0, sizeof(RWSpeedIntMsg));
        ReadMessage(configData->rwSpeedsInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                    sizeof(RWSpeedIntMsg), (void*) &(wheelSpeeds), moduleID);
        memset(&wheelsAvailability, 0x0, sizeof(RWAvailabilityFswMsg)); /* wheelAvailability set to 0 (AVAILABLE) by default */
        if (configData->rwAvailInMsgId >= 0){
            ReadMessage(configData->rwAvailInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                        sizeof(RWAvailabilityFswMsg), &wheelsAvailability, moduleID);
        }
    }

    /*! - compute control update time */
    if (configData->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = (callTime - configData->priorTime) * NANO2SEC;
    }
    configData->priorTime = callTime;

    /*! - compute body rate */
    v3Add(guidCmd.omega_BR_B, guidCmd.omega_RN_B, omega_BN_B);

    /*! - evaluate integral term */
    v3SetZero(configData->z);
    if (configData->Ki > 0) {   /* check if integral feedback is turned on  */
        v3Scale(configData->K * dt, guidCmd.sigma_BR, v3_1);
        v3Add(v3_1, configData->int_sigma, configData->int_sigma);

        for (i=0;i<3;i++) {
            intCheck = fabs(configData->int_sigma[i]);
            if (intCheck > configData->integralLimit) {
                configData->int_sigma[i] *= configData->integralLimit/intCheck;
            }
        }/* keep int_sigma less than integralLimit */
        m33MultV3(RECAST3X3 configData->ISCPntB_B, guidCmd.omega_BR_B, v3_2); /* -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s) */
        v3Add(configData->int_sigma, v3_2, configData->z);
    }

    /*! - evaluate required attitude control torque Lr */
    v3Scale(configData->K, guidCmd.sigma_BR, Lr);           /* +K sigma_BR */
    v3Scale(configData->P, guidCmd.omega_BR_B,
            v3_3);                                          /* +P delta_omega */
    v3Add(v3_3, Lr, Lr);
    v3Scale(configData->Ki, configData->z, v3_4);
    v3Scale(configData->P, v3_4, v3_5);                       /* +P*Ki*z */
    v3Add(v3_5, Lr, Lr);

    /* -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s) */
    m33MultV3(RECAST3X3 configData->ISCPntB_B, omega_BN_B, v3_6);
    for(i = 0; i < configData->rwConfigParams.numRW; i++)
    {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE){ /* check if wheel is available */
            wheelGs = &(configData->rwConfigParams.GsMatrix_B[i*3]);
            v3Scale(configData->rwConfigParams.JsList[i] * (v3Dot(omega_BN_B, wheelGs) + wheelSpeeds.wheelSpeeds[i]),
                    wheelGs, v3_7);                                 /* h_s_i */
            v3Add(v3_6, v3_7, v3_6);
        }
    }

    v3Add(guidCmd.omega_RN_B, v3_4, v3_8);
    v3Cross(v3_8, v3_6, v3_9);
    v3Subtract(Lr, v3_9, Lr);

    v3Cross(omega_BN_B, guidCmd.omega_RN_B, v3_10);
    v3Subtract(v3_10, guidCmd.domega_RN_B, v3_11);
    m33MultV3(RECAST3X3 configData->ISCPntB_B, v3_11, v3_12);   /* +[I](-d(omega_r)/dt + omega x omega_r) */
    v3Add(v3_12, Lr, Lr);

    v3Add(configData->knownTorquePntB_B, Lr, Lr);           /* +L */
    v3Scale(-1.0, Lr, Lr);                                  /* compute the net positive control torque onto the spacecraft */


    /*! - set the output message and write it out */
    v3Copy(Lr, controlOut.torqueRequestBody);
    CmdTorqueBodyMsg_C_write(&controlOut, &configData->cmdTorqueOutMsg, callTime);

    return;
}
