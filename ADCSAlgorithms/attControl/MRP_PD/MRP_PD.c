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
    MRP_PD Module
 
 */

#include "attControl/MRP_PD/MRP_PD.h"
#include "attGuidance/_GeneralModuleFiles/attGuidOut.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include "ADCSUtilities/ADCSDefinitions.h"
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
void SelfInit_MRP_PD(MRP_PDConfig *ConfigData, uint64_t moduleID)
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
void CrossInit_MRP_PD(MRP_PDConfig *ConfigData, uint64_t moduleID)
{
    
    RWConstellation localRWData;
    int i, j;
    uint64_t ClockTime;
    uint32_t ReadSize;

    /*! - Get the control data message IDs*/
    ConfigData->inputGuidID = subscribeToMessage(ConfigData->inputGuidName,
                                                 sizeof(attGuidOut), moduleID);
    ConfigData->inputVehicleConfigDataID = subscribeToMessage(ConfigData->inputVehicleConfigDataName,
                                                              sizeof(vehicleConfigData), moduleID);
    ConfigData->inputRWSpeedsID = subscribeToMessage(ConfigData->inputRWSpeedsName,
                                                     sizeof(RWSpeedData), moduleID);
    
    ConfigData->inputRWConfID = subscribeToMessage(ConfigData->inputRWConfigData,
                                                   sizeof(RWConstellation), moduleID);
    
    ReadMessage(ConfigData->inputRWConfID, &ClockTime, &ReadSize,
                sizeof(RWConstellation), &localRWData, moduleID);
    
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
void Reset_MRP_PD(MRP_PDConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_MRP_PD(MRP_PDConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    attGuidOut          guidCmd;            /*!< Guidance Message */
    vehicleConfigData   sc;                 /*!< spacecraft configuration message */
    RWSpeedData         wheelSpeeds;        /*!< Reaction wheel speed estimates */
    uint64_t            clockTime;
    uint32_t            readSize;
    double              Lr[3];              /*!< required control torque vector [Nm] */
    double              L[3];               /*!< known external torque */
    double              omega_BN_B[3];      /*!< Angular body rate with respect to the inertial N frame
                                             expressed in body B-frame components */
    double              h_s;                /*!< Reaction wheel momentum vector */
    double              *wheelGs;           /*!< Reaction wheel spin axis pointer */
    int                 i;                  /*!< Iterator */
    double              v3_temp1[3];        /*!< Temporal vector for insight computations */
    double              v3_temp2[3];        /*!< Temporal vector for insight computations */


    /*! Begin method steps*/
    
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputGuidID, &clockTime, &readSize,
                sizeof(attGuidOut), (void*) &(guidCmd), moduleID);
    ReadMessage(ConfigData->inputVehicleConfigDataID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(sc), moduleID);
    ReadMessage(ConfigData->inputRWSpeedsID, &clockTime, &readSize,
                sizeof(RWSpeedData), (void*) &(wheelSpeeds), moduleID);
    
    /* Compute body rate */
    v3Add(guidCmd.omega_BR_B, guidCmd.omega_RN_B, omega_BN_B);
    
    /* Compute known external torque */
    v3SetZero(L);
    
    /* 
     Evaluate required attitude control torque:
     Lr =  K*sigma_BR + P*delta_omega  - omega x ([I]omega + [Gs]h_s - omega_r) +
            - [I](d(omega_r)/dt - omega x omega_r) + L
     */
    v3Scale(ConfigData->K, guidCmd.sigma_BR, v3_temp1);           /* + K*sigma_BR */
    v3Scale(ConfigData->P, guidCmd.omega_BR_B, v3_temp2);         /* + P*delta_omega */
    v3Add(v3_temp1, v3_temp2, Lr);

    m33MultV3(RECAST3X3 sc.ISCPntB_B, omega_BN_B, v3_temp1);              /* - omega x ([I]omega + [Gs]h_s - omega_r) */
    v3Subtract(v3_temp1, guidCmd.omega_RN_B, v3_temp1);
    for(i = 0; i < ConfigData->numRWAs; i++)
    {
        wheelGs = &(ConfigData->GsMatrix[i*3]);
        h_s = ConfigData->JsList[i] * (v3Dot(omega_BN_B, wheelGs) + wheelSpeeds.wheelSpeeds[i]);
        v3Scale(h_s, wheelGs, v3_temp2);
        v3Add(v3_temp1, v3_temp2, v3_temp1);
    }
    v3Cross(omega_BN_B, v3_temp1, v3_temp1);
    v3Subtract(Lr, v3_temp1, Lr);
    
    v3Cross(omega_BN_B, guidCmd.omega_RN_B, v3_temp1);             /* - [I](d(omega_r)/dt - omega x omega_r) */
    v3Subtract(guidCmd.domega_RN_B, v3_temp1, v3_temp1);
    m33MultV3(RECAST3X3 sc.ISCPntB_B, v3_temp1, v3_temp1);
    v3Subtract(Lr, v3_temp1, Lr);
    
    v3Add(L, Lr, Lr);                                              /* + L */


    /* Store and write the output message */
    v3Copy(Lr, ConfigData->controlOut.torqueRequestBody);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehControlOut),
                 (void*) &(ConfigData->controlOut), moduleID);
    
    return;
}

