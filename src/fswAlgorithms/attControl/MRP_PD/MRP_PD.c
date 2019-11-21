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
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>

/*! This method initializes the configData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with this module
 */
void SelfInit_MRP_PD(MRP_PDConfig *configData, int64_t moduleID)
{
        /*! - Create output message for module */
    configData->bskPrint = _BSKPrint();
    configData->controlOutMsgId = CreateNewMessage(configData->outputDataName,
        sizeof(CmdTorqueBodyIntMsg), "CmdTorqueBodyIntMsg", moduleID);


}

/*! This method performs the second stage of initialization for this module.
 Its primary function is to link the input messages that were created elsewhere.
 @return void
 @param configData The configuration data associated with this module
 */
void CrossInit_MRP_PD(MRP_PDConfig *configData, int64_t moduleID)
{
    /*! - Get the control data message IDs*/
    configData->guidInMsgId = subscribeToMessage(configData->inputGuidName,
                                                 sizeof(AttGuidFswMsg), moduleID);
    configData->vehicleConfigDataInMsgId = subscribeToMessage(configData->inputVehicleConfigDataName,
                                                              sizeof(VehicleConfigFswMsg), moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the MRP steering control
 */
void Reset_MRP_PD(MRP_PDConfig *configData, uint64_t callTime, int64_t moduleID)
{
    uint64_t            timeOfMsgWritten;
    uint32_t            sizeOfMsgWritten;
    VehicleConfigFswMsg   sc;               /* spacecraft configuration message */

    /*! - read in spacecraft configuration message */
    memset(&sc, 0x0, sizeof(VehicleConfigFswMsg));
    ReadMessage(configData->vehicleConfigDataInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(VehicleConfigFswMsg), (void*) &sc, moduleID);
    mCopy(sc.ISCPntB_B, 1, 9, configData->ISCPntB_B);
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param configData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_MRP_PD(MRP_PDConfig *configData, uint64_t callTime,
    int64_t moduleID)
{
    uint64_t            timeOfMsgWritten;
    uint32_t            sizeOfMsgWritten;
    double              Lr[3];              /* required control torque vector [Nm] */
    double              omega_BN_B[3];      /* Inertial angular body vector in body B-frame components */
    CmdTorqueBodyIntMsg controlOutMsg;      /* Control output requests */
    AttGuidFswMsg       guidInMsg;          /* Guidance Message */
    double              v3_temp1[3];
    double              v3_temp2[3];
    double              v3_temp3[3];
    double              v3_temp4[3];

    /*! - zero the output message copy */
    memset(&controlOutMsg, 0x0, sizeof(CmdTorqueBodyIntMsg));

    /*! - Read the guidance input message */
    memset(&guidInMsg, 0x0, sizeof(AttGuidFswMsg));
    ReadMessage(configData->guidInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(AttGuidFswMsg), (void*) &(guidInMsg), moduleID);

    /*! - Compute angular body rate */
    v3Add(guidInMsg.omega_BR_B, guidInMsg.omega_RN_B, omega_BN_B);

    /*! - Evaluate required attitude control torque */
    /* Lr =  K*sigma_BR + P*delta_omega  - omega_r x [I]omega - [I](d(omega_r)/dt - omega x omega_r) + L
     */
    v3Scale(configData->K, guidInMsg.sigma_BR, v3_temp1); /* + K * sigma_BR */
    v3Scale(configData->P, guidInMsg.omega_BR_B, v3_temp2); /* + P * delta_omega */
    v3Add(v3_temp1, v3_temp2, Lr);

    /* omega x [I]omega */
    m33MultV3(RECAST3X3 configData->ISCPntB_B, omega_BN_B, v3_temp3);
    v3Cross(guidInMsg.omega_RN_B, v3_temp3, v3_temp3); /* omega_r x [I]omega */
    v3Subtract(Lr, v3_temp3, Lr);

    /* [I](d(omega_r)/dt - omega x omega_r) */
    v3Cross(omega_BN_B, guidInMsg.omega_RN_B, v3_temp4);
    v3Subtract(guidInMsg.domega_RN_B, v3_temp4, v3_temp4);
    m33MultV3(RECAST3X3 configData->ISCPntB_B, v3_temp4, v3_temp4);
    v3Subtract(Lr, v3_temp4, Lr);

    v3Add(configData->knownTorquePntB_B, Lr, Lr); /* + L */
    v3Scale(-1.0, Lr, Lr);

    /*! - Store and write the output message */
    v3Copy(Lr, controlOutMsg.torqueRequestBody);
    WriteMessage(configData->controlOutMsgId, callTime, sizeof(CmdTorqueBodyIntMsg),
                 (void*) &controlOutMsg, moduleID);

    return;
}
