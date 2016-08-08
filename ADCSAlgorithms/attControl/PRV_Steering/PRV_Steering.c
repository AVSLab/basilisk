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
    PRV_STEERING Module
 
 */

#include "attControl/PRV_Steering/PRV_Steering.h"
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
void SelfInit_PRV_Steering(PRV_SteeringConfig *ConfigData, uint64_t moduleID)
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
void CrossInit_PRV_Steering(PRV_SteeringConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputGuidID = subscribeToMessage(ConfigData->inputGuidName,
                                                 sizeof(attGuidOut), moduleID);
    ConfigData->inputVehicleConfigDataID = subscribeToMessage(ConfigData->inputVehicleConfigDataName,
                                                              sizeof(vehicleConfigData), moduleID);
    ConfigData->inputRWSpeedsID = subscribeToMessage(ConfigData->inputRWSpeedsName,
                                                     sizeof(RWSpeedData), moduleID);
    
    ConfigData->inputRWConfID = subscribeToMessage(ConfigData->inputRWConfigData,
                                                   sizeof(RWConstellation), moduleID);
    /*! - Read static RW config data message and store it in module variables*/
    RWConstellation localRWData;
    int i, j;
    uint64_t ClockTime;
    uint32_t ReadSize;
    ReadMessage(ConfigData->inputRWConfID, &ClockTime, &ReadSize,
                sizeof(RWConstellation), &localRWData, moduleID);
    ConfigData->numRWAs = localRWData.numRW;
    for(i=0; i<ConfigData->numRWAs; i=i+1)
    {
        ConfigData->JsList[i] = localRWData.reactionWheels[i].Js;
        for(j=0; j<3; j=j+1)
        {
            ConfigData->GsMatrix[i*3+j] = localRWData.reactionWheels[i].Gs_S[j];
        }
    }
    
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_PRV_Steering(PRV_SteeringConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    ConfigData->priorTime = 0;              /* reset the prior time flag state.  If set
                                             to zero, the control time step is not evaluated on the
                                             first function call */
    v3SetZero(ConfigData->z);               /* reset the integral measure of the rate tracking error */
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_PRV_Steering(PRV_SteeringConfig *ConfigData, uint64_t callTime,
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
    double              v3[3];
    double              v3_1[3];
    double              omega_BastR_B[3];   /*!< angular velocity of desired Bast frame relative to reference frame R */
    double              omegap_BastR_B[3];  /*!< body frame derivative of omega_BastR */
    double              omega_BastN_B[3];   /*!< angular velocity of B^ast relative to inertial N, in body frame components */
    double              omega_BBast_B[3];   /*!< angular velocity tracking error between actual 
                                             body frame B and desired B^ast frame */
    double              omega_BN_B[3];
    int                 i;
    double              temp;
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
    ReadMessage(ConfigData->inputRWSpeedsID, &clockTime, &readSize,
                sizeof(RWSpeedData), (void*) &(wheelSpeeds), moduleID);
    
    /* compute body rate */
    v3Add(guidCmd.omega_BR_B, guidCmd.omega_RN_B, omega_BN_B);

    /* compute known external torque */
    v3SetZero(L);

    /* evalute MRP kinematic steering law */
    PRVSteeringLaw(ConfigData, guidCmd.sigma_BR, omega_BastR_B, omegap_BastR_B);

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

    m33MultV3(RECAST3X3 sc.ISCPntB_B, omega_BN_B, v3);          /* - omega_BastN x ([I]omega + [Gs]h_s) */
    
    for(i = 0; i < ConfigData->numRWAs; i++)
    {
        double scalar;
        wheelGs = &(ConfigData->GsMatrix[i*3]);
        scalar = ConfigData->JsList[i] * (v3Dot(omega_BN_B, wheelGs) + wheelSpeeds.wheelSpeeds[i]);
        v3Scale(scalar, wheelGs, v3_1);
        v3Add(v3_1, v3, v3);
    }
    
    v3Cross(omega_BastN_B, v3, v3_1);
    v3Subtract(Lr, v3_1, Lr);

    v3Cross(omega_BN_B, guidCmd.omega_RN_B, v3);
    v3Subtract(guidCmd.domega_RN_B, v3, v3_1);
    v3Add(v3_1, omegap_BastR_B, v3_1);
    m33MultV3(RECAST3X3 sc.ISCPntB_B, v3_1, v3);
    v3Subtract(Lr, v3, Lr);                                 /* -[I](d(omega_B^ast/R)/dt + d(omega_r)/dt - omega x omega_r) */

    v3Add(L, Lr, Lr);                                       /* +L */

    /* store the output message */
    v3Copy(Lr, ConfigData->controlOut.torqueRequestBody);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehControlOut),
                 (void*) &(ConfigData->controlOut), moduleID);
    
    return;
}


/*! This method computes the PRV Steering law.  A commanded body rate is returned given the PRV
 attitude error measure of the body relative to a reference frame.  The function returns the commanded
 body rate, as well as the body frame derivative of this rate command.
 @return void
 @param ConfigData  The configuration data associated with this module
 @param sigma_BR    MRP attitude error of B relative to R
 @param omega_ast   Commanded body rates
 @param omega_ast_p Body frame derivative of the commanded body rates
 */
void PRVSteeringLaw(PRV_SteeringConfig *configData, double sigma_BR[3], double omega_ast[3], double omega_ast_p[3])
{
    double e_hat[3];        /*!< principal rotation axis of MRP */
    double phi;             /*!< principal rotation angle of MRP */
    double sigma_Norm;      /*!< norm of the MRP attitude error */
    double value;

    sigma_Norm = v3Norm(sigma_BR);
    if (sigma_Norm > 0.00000000001) {
        v3Scale(1./sigma_Norm, sigma_BR, e_hat);
    } else {
        e_hat[0] = 1.;
        e_hat[1] = 0.;
        e_hat[2] = 0.;
    }
    phi = 4.*atan(sigma_Norm);

    value = atan(M_PI_2/configData->omega_max*(configData->K1*phi + configData->K3*phi*phi*phi))/M_PI_2*configData->omega_max;

    v3Scale(-value, e_hat, omega_ast);

    value *= (3*configData->K3*phi*phi + configData->K1)/(pow(M_PI_2/configData->omega_max*(configData->K1*phi + configData->K3*phi*phi*phi),2) + 1);

    v3Scale(value, e_hat, omega_ast_p);
    
    return;
}
