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
    MRP_STEERING Module
 
 */

#include "attControl/MRP_Steering/MRP_Steering.h"
#include "attGuidance/_GeneralModuleFiles/attGuidOut.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include "ADCSUtilities/ADCSDefinitions.h"
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
void SelfInit_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t moduleID)
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
void CrossInit_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message IDs*/
    ConfigData->inputGuidID = subscribeToMessage(ConfigData->inputGuidName,
                                                 sizeof(attGuidOut), moduleID);
    ConfigData->vehConfigInMsgID = subscribeToMessage(ConfigData->vehConfigInMsgName,
                                                 sizeof(vehicleConfigData), moduleID);
    ConfigData->rwParamsInMsgID = -1;
    ConfigData->inputRWSpeedsID = -1;
    ConfigData->rwAvailInMsgID = -1;
    
    if(strlen(ConfigData->rwParamsInMsgName) > 0) {
        ConfigData->rwParamsInMsgID = subscribeToMessage(ConfigData->rwParamsInMsgName,
                                                         sizeof(RWConfigParams), moduleID);
        ConfigData->inputRWSpeedsID = subscribeToMessage(ConfigData->inputRWSpeedsName,
                                                         sizeof(RWSpeedData), moduleID);
        if(strlen(ConfigData->rwAvailInMsgName) > 0) {
            ConfigData->rwAvailInMsgID = subscribeToMessage(ConfigData->rwAvailInMsgName,
                                                            sizeof(RWAvailabilityData), moduleID);
        }
    }
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read the input messages */
    uint64_t clockTime;
    uint32_t readSize;
    
    vehicleConfigData sc;
    ReadMessage(ConfigData->vehConfigInMsgID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(sc), moduleID);
    for (int i=0; i < 9; i++){
        ConfigData->ISCPntB_B[i] = sc.ISCPntB_B[i];
    };
    
    ConfigData->rwConfigParams.numRW = 0;
    if (ConfigData->rwParamsInMsgID >= 0) {
        /*! - Read static RW config data message and store it in module variables*/
        ReadMessage(ConfigData->rwParamsInMsgID, &clockTime, &readSize,
                    sizeof(RWConfigParams), &(ConfigData->rwConfigParams), moduleID);
    }
    
    /* Reset the integral measure of the rate tracking error */
    v3SetZero(ConfigData->z);
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
void Update_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    attGuidOut          guidCmd;            /*!< Guidance Message */
    RWSpeedData         wheelSpeeds;        /*!< Reaction wheel speed estimates */
    RWAvailabilityData  wheelsAvailability; /*!< Reaction wheel availability */
    uint64_t            clockTime;
    uint32_t            readSize;
    double              dt;                 /*!< [s] control update period */
    
    double              Lr[3];              /*!< required control torque vector [Nm] */
    double              L[3];               /*!< known external torque */
    double              omega_BastR_B[3];   /*!< angular velocity of desired Bast frame relative to reference frame R */
    double              omegap_BastR_B[3];  /*!< body frame derivative of omega_BastR */
    double              omega_BastN_B[3];   /*!< angular velocity of B^ast relative to inertial N, in body frame components */
    double              omega_BBast_B[3];   /*!< angular velocity tracking error between actual  body frame B and desired B^ast frame */
    double              omega_BN_B[3];      /*!< angular rate of the body B relative to inertial N, in body frame compononents */
    double              *wheelGs;           /*!< Reaction wheel spin axis pointer */
    /*!< Temporary variables */
    double              v3[3];
    double              v3_1[3];
    int                 i;
    double              temp;
    
    /*! Begin method steps*/
    
    /* compute control update time */
    if (ConfigData->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = (callTime - ConfigData->priorTime) * NANO2SEC;
    }
    ConfigData->priorTime = callTime;

    /*! - Read the dynamic input messages */
    ReadMessage(ConfigData->inputGuidID, &clockTime, &readSize,
                sizeof(attGuidOut), (void*) &(guidCmd), moduleID);
    
    memset(wheelSpeeds.wheelSpeeds, 0x0, MAX_EFF_CNT * sizeof(double));
    memset(wheelsAvailability.wheelAvailability, 0x0, MAX_EFF_CNT * sizeof(int)); // wheelAvailability set to 0 (AVAILABLE) by default
    if(ConfigData->rwConfigParams.numRW > 0) {
        ReadMessage(ConfigData->inputRWSpeedsID, &clockTime, &readSize,
                    sizeof(RWSpeedData), (void*) &(wheelSpeeds), moduleID);
        if (ConfigData->rwAvailInMsgID >= 0){
            ReadMessage(ConfigData->rwAvailInMsgID, &clockTime, &readSize,
                        sizeof(RWAvailabilityData), &wheelsAvailability, moduleID);
        }
    }
    
    /* compute body rate */
    v3Add(guidCmd.omega_BR_B, guidCmd.omega_RN_B, omega_BN_B);

    /* compute known external torque */
    v3SetZero(L);

    /* evalute MRP kinematic steering law */
    MRPSteeringLaw(ConfigData, guidCmd.sigma_BR, omega_BastR_B, omegap_BastR_B);

    /* compute the rate tracking error */
    v3Add(omega_BastR_B, guidCmd.omega_RN_B, omega_BastN_B);
    v3Subtract(omega_BN_B, omega_BastN_B, omega_BBast_B);

    /* integrate rate tracking error  */
    if (ConfigData->Ki > 0) {   /* check if integral feedback is turned on  */
        v3Scale(dt, omega_BBast_B, v3);
        v3Add(v3, ConfigData->z, ConfigData->z);             /* z = integral(del_omega) */
        for (i=0;i<3;i++) {
            temp = fabs(ConfigData->z[i]);
            if (temp > ConfigData->integralLimit) {
                ConfigData->z[i] *= ConfigData->integralLimit/temp;
            }
        }
    } else {
        /* integral feedback is turned off through a negative gain setting */
        v3SetZero(ConfigData->z);
    }

    /* evaluate required attitude control torque Lr */
    v3Scale(ConfigData->P, omega_BBast_B, Lr);              /* +P delta_omega */
    v3Scale(ConfigData->Ki, ConfigData->z, v3);
    v3Add(v3, Lr, Lr);                                      /* +Ki*z */

    /* Lr += - omega_BastN x ([I]omega + [Gs]h_s) */
    m33MultV3(RECAST3X3 ConfigData->ISCPntB_B, omega_BN_B, v3);
    for(i = 0; i < ConfigData->rwConfigParams.numRW; i++)
    {
        if (wheelsAvailability.wheelAvailability[i] == AVAILABLE){ /* check if wheel is available */
            wheelGs = &(ConfigData->rwConfigParams.GsMatrix_B[i*3]);
            v3Scale(ConfigData->rwConfigParams.JsList[i] * (v3Dot(omega_BN_B, wheelGs) + wheelSpeeds.wheelSpeeds[i]),
                    wheelGs, v3_1);
            v3Add(v3_1, v3, v3);
        }
    }
    v3Cross(omega_BastN_B, v3, v3_1);
    v3Subtract(Lr, v3_1, Lr);
    
    /* Lr +=  - [I](d(omega_B^ast/R)/dt + d(omega_r)/dt - omega x omega_r) */
    v3Cross(omega_BN_B, guidCmd.omega_RN_B, v3);
    v3Subtract(guidCmd.domega_RN_B, v3, v3_1);
    v3Add(v3_1, omegap_BastR_B, v3_1);
    m33MultV3(RECAST3X3 ConfigData->ISCPntB_B, v3_1, v3);
    v3Subtract(Lr, v3, Lr);
    
    /* Add external torque: Lr += L */
    v3Add(L, Lr, Lr);
    
    /* Change sign to compute the net positive control torque onto the spacecraft */
    v3Scale(-1.0, Lr, Lr);

    /* Store the output message and pass it to the message bus */
    v3Copy(Lr, ConfigData->controlOut.torqueRequestBody);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehControlOut),
                 (void*) &(ConfigData->controlOut), moduleID);
    
    return;
}


/*! This method computes the MRP Steering law.  A commanded body rate is returned given the MRP
 attitude error measure of the body relative to a reference frame.  The function returns the commanded
 body rate, as well as the body frame derivative of this rate command.
 @return void
 @param ConfigData  The configuration data associated with this module
 @param sigma_BR    MRP attitude error of B relative to R
 @param omega_ast   Commanded body rates
 @param omega_ast_p Body frame derivative of the commanded body rates
 */
void MRPSteeringLaw(MRP_SteeringConfig *ConfigData, double sigma_BR[3], double omega_ast[3], double omega_ast_p[3])
{
    double  sigma_i;        /*!< ith component of sigma_B/R */
    double  B[3][3];        /*!< B-matrix of MRP differential kinematic equations */
    double  sigma_p[3];     /*!< the MRP rate equivalent of the steering rates */
    double  value;
    int     i;

    /* Determine the desired steering rates  */
    for (i=0;i<3;i++) {
        sigma_i      = sigma_BR[i];
        value        = atan(M_PI_2/ConfigData->omega_max*(ConfigData->K1*sigma_i
                       + ConfigData->K3*sigma_i*sigma_i*sigma_i))/M_PI_2*ConfigData->omega_max;
        omega_ast[i] = -value;

    }

    if (ConfigData->ignoreOuterLoopFeedforward) {
        v3SetZero(omega_ast_p);
    } else {
        /* Determine the body frame derivative of the steering rates */
        BmatMRP(sigma_BR, B);
        m33MultV3(B, omega_ast, sigma_p);
        v3Scale(0.25, sigma_p, sigma_p);
        for (i=0;i<3;i++) {
            sigma_i        = sigma_BR[i];
            value          = (3*ConfigData->K3*sigma_i*sigma_i + ConfigData->K1)/(pow(M_PI_2/ConfigData->omega_max*(ConfigData->K1*sigma_i + ConfigData->K3*sigma_i*sigma_i*sigma_i),2) + 1);
            omega_ast_p[i] = - value*sigma_p[i];
        }
    }
    return;
}
