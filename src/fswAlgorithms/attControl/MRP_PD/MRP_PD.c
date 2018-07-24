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
    MRP_PD Module
 
 */

#include "attControl/MRP_PD/MRP_PD.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "fswUtilities/fswDefinitions.h"
#include "simulation/utilities/astroConstants.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
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
        sizeof(CmdTorqueBodyIntMsg), "CmdTorqueBodyIntMsg", moduleID);


}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_MRP_PD(MRP_PDConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message IDs*/
    ConfigData->inputGuidID = subscribeToMessage(ConfigData->inputGuidName,
                                                 sizeof(AttGuidFswMsg), moduleID);
    ConfigData->inputVehicleConfigDataID = subscribeToMessage(ConfigData->inputVehicleConfigDataName,
                                                              sizeof(VehicleConfigFswMsg), moduleID);
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
    AttGuidFswMsg      guidCmd;            /*!< Guidance Message */
    VehicleConfigFswMsg   sc;                 /*!< spacecraft configuration message */
    uint64_t            clockTime;
    uint32_t            readSize;
    double              Lr[3];              /*!< required control torque vector [Nm] */
    double              omega_BN_B[3];      /*!< Inertial angular body rate expressed in body B-frame components */
    double              v3_temp1[3];        /*!< Temporal vector for insight computations */
    double              v3_temp2[3];        /*!< Temporal vector for insight computations */


    /*! Begin method steps*/
    
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputGuidID, &clockTime, &readSize,
                sizeof(AttGuidFswMsg), (void*) &(guidCmd), moduleID);
    ReadMessage(ConfigData->inputVehicleConfigDataID, &clockTime, &readSize,
                sizeof(VehicleConfigFswMsg), (void*) &(sc), moduleID);
    
    /*! - Compute body rate */
    v3Add(guidCmd.omega_BR_B, guidCmd.omega_RN_B, omega_BN_B);
        
    /*! - Evaluate required attitude control torque:
     Lr =  K*sigma_BR + P*delta_omega  - omega_r x [I]omega - [I](d(omega_r)/dt - omega x omega_r) + L
     */
    v3Scale(ConfigData->K, guidCmd.sigma_BR, v3_temp1); /* + K * sigma_BR */
    v3Scale(ConfigData->P, guidCmd.omega_BR_B, v3_temp2); /* + P * delta_omega */
    v3Add(v3_temp1, v3_temp2, Lr);
    
    /* - omega x [I]omega */
    m33MultV3(RECAST3X3 sc.ISCPntB_B, omega_BN_B, v3_temp1);
    v3Cross(guidCmd.omega_RN_B, v3_temp1, v3_temp1); /* omega_r x [I]omega */
    v3Subtract(Lr, v3_temp1, Lr);
    
    /* - [I](d(omega_r)/dt - omega x omega_r) */
    v3Cross(omega_BN_B, guidCmd.omega_RN_B, v3_temp1);
    v3Subtract(guidCmd.domega_RN_B, v3_temp1, v3_temp1);
    m33MultV3(RECAST3X3 sc.ISCPntB_B, v3_temp1, v3_temp1);
    v3Subtract(Lr, v3_temp1, Lr);
    
    v3Add(ConfigData->knownTorquePntB_B, Lr, Lr); /* + L */
    v3Scale(-1.0, Lr, Lr);                                  /* compute the net positive control torque onto the spacecraft */


    /*! - Store and write the output message */
    v3Copy(Lr, ConfigData->controlOut.torqueRequestBody);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(CmdTorqueBodyIntMsg),
                 (void*) &(ConfigData->controlOut), moduleID);
    
    return;
}

