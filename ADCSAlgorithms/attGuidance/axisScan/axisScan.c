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
 Inertial 3D Spin Module
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved
 
 */

#include "attGuidance/axisScan/axisScan.h"
#include <string.h>
#include <math.h>
#include "ADCSUtilities/ADCSDefinitions.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"

/* Required module input messages */
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"
#include "SimCode/environment/spice/spice_planet_state.h"
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"



void SelfInit_axisScan(axisScanConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(attRefOut),
                                               "attRefOut",
                                               moduleID);
    ConfigData->mnvrStartTime = -1;
}

void CrossInit_axisScan(axisScanConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputRefID = subscribeToMessage(ConfigData->inputRefName,
                                                sizeof(attRefOut),
                                                moduleID);
}

void Reset_axisScan(axisScanConfig *ConfigData, uint64_t moduleID)
{
    ConfigData->mnvrStartTime = -1;
}


void Update_axisScan(axisScanConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read input message */
    uint64_t            writeTime;
    uint32_t            writeSize;
    attRefOut           ref;
    ReadMessage(ConfigData->inputRefID, &writeTime, &writeSize,
                sizeof(attRefOut), (void*) &(ref), moduleID);
    
    
    /*! - Compute output variables */
    computeAxisScanReference(ConfigData,
                             ref.sigma_RN,
                             ref.omega_RN_N,
                             ref.domega_RN_N,
                             callTime);
    
    /*! - Write output message */
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attRefOut),
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    return;
}

void initializeScanReference(axisScanConfig *ConfigData, double sigma_R0N[3])
{
    double  R0N[3][3];      /* DCM mapping from inertial N to input reference R0 */
    double  UN[3][3];       /* DCM mapping from inertial N to initial scanning frame U */
    double  UR0[3][3];      /* DCM mapping from input reference R0 to initial scanning frame U */
    
    double  M2[3][3];       /* temporary DCM */
    double  M3[3][3];       /* temporary DCM */
    
    MRP2C(sigma_R0N, R0N);
    Mi(ConfigData->theta0, 2, M2);
    Mi(-ConfigData->psi0, 3, M3);
    m33MultM33(M2, M3, UR0);
    m33MultM33(UR0, R0N, UN);
    C2MRP(UN, ConfigData->sigma_UN);
}


void computeAxisScanReference(axisScanConfig *ConfigData,
                              double sigma_R0N[3],
                              double omega_R0N_N[3],
                              double domega_R0N_N[3],
                              uint64_t callTime)
{
    double RN[3][3];       /* DCM mapping from inertial N to desired reference R */
    double RU[3][3];       /* DCM mapping from initial scanning frame U to desired reference R */
    double UN[3][3];       /* DCM mapping from inertial N to initial scanning frame U */
    double R0N[3][3];      /* DCM mapping from inertial N to input reference R0 */
    
    double omega_RU_N[3];
    double omega_RU_R0[3];

    double C[3][3];        /* temporary DCM */
    double v[3];           /* temporary vector */
    
    double currMnvrTime;   /* time since maneuver started */
    double psi;            /* current scanning angle */
    
    if (ConfigData->mnvrStartTime == -1)
    {
        /* Initialize Scanning Pointing */
        initializeScanReference(ConfigData, sigma_R0N);
        ConfigData->mnvrStartTime = callTime;
    }
    currMnvrTime = (callTime - ConfigData->mnvrStartTime)*1.0E-9;
    
    /* Integrate Attitude */
    MRP2C(ConfigData->sigma_UN, UN);
    psi = ConfigData->psiDot * currMnvrTime;
    Mi(-psi, 3, RU);
    m33MultM33(RU, UN, RN);
    C2MRP(RN, ConfigData->attRefOut.sigma_RN);
    
    /* Compute angular velocity */
    v3SetZero(omega_RU_R0);
    omega_RU_R0[2] = -ConfigData->psiDot;
    MRP2C(sigma_R0N, R0N);
    m33Transpose(R0N, C);
    m33MultV3(C,  omega_RU_R0, omega_RU_N);
    v3Add(omega_RU_N, omega_R0N_N, ConfigData->attRefOut.omega_RN_N); /* Note that omega_UR0 = 0, 
                                                                       since [UR0] is a constant offset */
    
    /* Compute angular acceleration */
    v3Cross(omega_R0N_N, omega_RU_N, v);
    v3Add(v, domega_R0N_N, ConfigData->attRefOut.domega_RN_N);
}
