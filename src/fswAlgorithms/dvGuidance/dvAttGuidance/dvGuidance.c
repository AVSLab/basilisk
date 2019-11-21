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

#include "dvGuidance/dvAttGuidance/dvGuidance.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! This method initializes the configData for the nominal delta-V maneuver guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with the delta-V maneuver guidance
 @param moduleID The unique module identifier
 */
void SelfInit_dvGuidance(dvGuidanceConfig *configData, int64_t moduleID)
{
    configData->bskPrint = _BSKPrint();
    /*! - Create output message for module */
    configData->outputMsgID = CreateNewMessage(configData->outputDataName,
                                               sizeof(AttRefFswMsg), "AttRefFswMsg", moduleID);
    return;

}

/*! This method performs the second stage of initialization for the delta-V maneuver
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param configData The configuration data associated with the attitude maneuver guidance
 @param moduleID The unique module identifier
 */
void CrossInit_dvGuidance(dvGuidanceConfig *configData, int64_t moduleID)
{
    configData->inputBurnCmdID = subscribeToMessage(configData->inputBurnDataName,
                                                    sizeof(DvBurnCmdFswMsg), moduleID);
    return;

}

/*! @brief This resets the module.
 @return void
 @param configData The configuration data associated with this module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The unique module identifier
 */
void Reset_dvGuidance(dvGuidanceConfig *configData, uint64_t callTime,
                       int64_t moduleID)
{
    return;
}

/*! This method takes its own internal variables and creates an output attitude
    command to use for burn execution.  It also flags whether the burn should
    be happening or not.
 @return void
 @param configData The configuration data associated with the delta-V maneuver guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The unique module identifier
 */
void Update_dvGuidance(dvGuidanceConfig *configData, uint64_t callTime,
    int64_t moduleID)
{
    double dcm_BubN[3][3];           /* dcm, inertial to base burn frame */
    double dcm_ButN[3][3];           /* dcm, inertial to current burn frame */
    double dcm_ButBub[3][3];         /* dcm, rotating from base to current burn frame */
    double dvHat_N[3];               /* unit vector, direction of delta velocity in the inertial frame */
    double bu2_N[3];                 /* vector, vector which becomes the BubN DCM's second basis vector */
	double burnTime;                 /* duration for which to thrust */
	double rotPRV[3];                /* principle rotation vector about which to rotate during the burn */
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    DvBurnCmdFswMsg localBurnData;   /* [-] input message container */
    AttRefFswMsg attCmd;             /* [-] Output attitude command data to send */

    /*! - zero the input and output message containers */
    memset(&localBurnData, 0x0, sizeof(DvBurnCmdFswMsg));
    memset(&attCmd, 0x0, sizeof(AttRefFswMsg));

    /*! - read in DV burn command input message */
    ReadMessage(configData->inputBurnCmdID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(DvBurnCmdFswMsg), &localBurnData, moduleID);

    /*! - evaluate DCM from inertial to the base Burn Frame */
    v3Normalize(localBurnData.dvInrtlCmd, dvHat_N);
    v3Copy(dvHat_N, dcm_BubN[0]);
    v3Cross(localBurnData.dvRotVecUnit, dvHat_N, bu2_N);
    v3Normalize(bu2_N, dcm_BubN[1]);
    v3Cross(dcm_BubN[0], dcm_BubN[1], dcm_BubN[2]);
    v3Normalize(dcm_BubN[2], dcm_BubN[2]);

    /*! - evaluate the time since the burn start time */
    burnTime = ((int64_t) callTime - (int64_t) localBurnData.burnStartTime)*NANO2SEC;

    /*! - evaluate the DCM from inertial to the current Burn frame.
     The current frame differs from the base burn frame via a constant 3-axis rotation */
    v3SetZero(rotPRV);
    rotPRV[2] = 1.0;
    v3Scale(burnTime*localBurnData.dvRotVecMag, rotPRV, rotPRV);
    PRV2C(rotPRV, dcm_ButBub);
	m33MultM33(dcm_ButBub, dcm_BubN, dcm_ButN);

    /*! - Compute the reference attitude */
	C2MRP(RECAST3X3 &dcm_ButN, attCmd.sigma_RN);
    /*! - Compute the reference frame angular rate vector */
	v3Scale(localBurnData.dvRotVecMag, dcm_ButN[2], attCmd.omega_RN_N);
    /*! - Zero the reference frame angular acceleration vector */
    v3SetZero(attCmd.domega_RN_N);

    /*! - Write the output message */
    WriteMessage(configData->outputMsgID, callTime, sizeof(AttRefFswMsg),
        &attCmd, moduleID);

    return;
}
