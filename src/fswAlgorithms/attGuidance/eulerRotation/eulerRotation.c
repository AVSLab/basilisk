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
 Euler Angle Rotation Guidance Module with Constant Euler Rates
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved
 
 */

#include "attGuidance/eulerRotation/eulerRotation.h"
#include <string.h>
#include <math.h>
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"




void SelfInit_eulerRotation(eulerRotationConfig *ConfigData, uint64_t moduleID)
{
    /* - Create output message for module */
    ConfigData->attRefOutMsgID = CreateNewMessage(ConfigData->attRefOutMsgName,
                                               sizeof(AttRefFswMsg),
                                               "AttRefFswMsg",
                                               moduleID);
    ConfigData->attitudeOutMsgID = -1;
    if(strlen(ConfigData->attitudeOutMsgName) > 0)
    {
        ConfigData->attitudeOutMsgID = CreateNewMessage(ConfigData->attitudeOutMsgName,
                                                        sizeof(AttStateFswMsg),
                                                        "AttStateFswMsg",
                                                        moduleID);
    }

    ConfigData->priorTime = 0;
    v3SetZero(ConfigData->priorCmdSet);
    v3SetZero(ConfigData->priorCmdRates);
}

void CrossInit_eulerRotation(eulerRotationConfig *ConfigData, uint64_t moduleID)
{
    /* - Get the control data message ID*/
    ConfigData->attRefInMsgID = subscribeToMessage(ConfigData->attRefInMsgName,
                                                sizeof(AttRefFswMsg),
                                                moduleID);
    
    ConfigData->desiredAttInMsgID = -1;
    if(strlen(ConfigData->desiredAttInMsgName) > 0)
    {
        ConfigData->desiredAttInMsgID = subscribeToMessage(ConfigData->desiredAttInMsgName,
                                                         sizeof(AttStateFswMsg),
                                                         moduleID);
    }
}

void Reset_eulerRotation(eulerRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    ConfigData->priorTime = 0;
    v3SetZero(ConfigData->priorCmdSet);
    v3SetZero(ConfigData->priorCmdRates);
}


void Update_eulerRotation(eulerRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /* - Read input messages */
    AttRefFswMsg inputRef;
    AttStateFswMsg attStates;
    uint64_t writeTime;
    uint32_t writeSize;
    ReadMessage(ConfigData->attRefInMsgID, &writeTime, &writeSize,
                sizeof(AttRefFswMsg), (void*) &(inputRef), moduleID);
    if (ConfigData->desiredAttInMsgID >= 0)
    {
        /* - Read Raster Manager messages */
        ReadMessage(ConfigData->desiredAttInMsgID, &writeTime, &writeSize,
                    sizeof(AttStateFswMsg), (void*) &(attStates), moduleID);
        /* - Save commanded 321 Euler set and rates */
        v3Copy(attStates.state, ConfigData->cmdSet);
        v3Copy(attStates.rate, ConfigData->cmdRates);
        /* - Check the command is new */
        checkRasterCommands(ConfigData);
    }
    
    /* - Compute time step to use in the integration downstream */
    computeTimeStep(ConfigData, callTime);
    /* - Compute output reference frame */
    computeEulerRotationReference(ConfigData,
                                  inputRef.sigma_RN,
                                  inputRef.omega_RN_N,
                                  inputRef.domega_RN_N);
    /* - Write output messages */
    writeOutputMessages(ConfigData, callTime, moduleID);
    /* - Update last time the module was called to current call time */
    ConfigData->priorTime = callTime;
    return;
}

/*!
 Function: writeOutputMessages
 Purpose: This function writes the the generated reference and the computed euler angle set in different messages
 Input
      ConfigData = module configuration data
      moduleID = message ID of this module (eulerRotation)
      callTime = current simulation time when the module is called
 Output: (-)
 */
void writeOutputMessages(eulerRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /* - Guidance reference output */
    WriteMessage(ConfigData->attRefOutMsgID, callTime, sizeof(AttRefFswMsg),
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    /* - Euler angle set and rates outputed for testing purposes */
    if (ConfigData->attitudeOutMsgID >= 0) {
        v3Copy(ConfigData->angleSet, ConfigData->attStateOut.state);
        v3Copy(ConfigData->angleRates, ConfigData->attStateOut.rate);
        WriteMessage(ConfigData->attitudeOutMsgID, callTime, sizeof(AttStateFswMsg),
                     (void*) &(ConfigData->attStateOut), moduleID);
    }
}


/*!
 Function: checkRasterCommands
 Purpose: This function checks if there is a new commanded raster maneuver available
 Input
      ConfigData = module configuration data
 */
void checkRasterCommands(eulerRotationConfig *ConfigData)
{
    int32_t prevCmdActive = v3IsEqual(ConfigData->cmdSet, ConfigData->priorCmdSet , 1E-12) && v3IsEqual(ConfigData->cmdRates, ConfigData->priorCmdRates , 1E-12);
    if (!prevCmdActive)
    {
        v3Copy(ConfigData->cmdSet, ConfigData->angleSet);
        v3Copy(ConfigData->cmdRates, ConfigData->angleRates);
        
        v3Copy(ConfigData->cmdSet, ConfigData->priorCmdSet);
        v3Copy(ConfigData->cmdRates, ConfigData->priorCmdRates);
    }
}

/*!
 Function: computeTimeStep
 Purpose: This function computes control update time
 Input
      ConfigData = module configuration data
 Output:
      ConfigData: dt is updated
 */
void computeTimeStep(eulerRotationConfig *ConfigData, uint64_t callTime)
{
    if (ConfigData->priorTime == 0)
    {
        ConfigData->dt = 0.0;
    } else {
        ConfigData->dt = (callTime - ConfigData->priorTime)*NANO2SEC;
    }
}

/*!
 Function: computeEuler321_Binv_derivative
 Purpose: This function computes the analytical derivative of the B_inv matrix for the 3-2-1 Euler Angle set
 Input
      angleSet: 3-2-1 euler angles
      angleRates: 3-2-1 euler angle rates
 Output:
      B_inv_deriv
 */
void computeEuler321_Binv_derivative(double angleSet[3], double angleRates[3], double B_inv_deriv[3][3])
{
    double s2;
    double c2;
    double s3;
    double c3;
    
    s2 = sin(angleSet[1]);
    c2 = cos(angleSet[1]);
    s3 = sin(angleSet[2]);
    c3 = cos(angleSet[2]);
    
    
    B_inv_deriv[0][0] = -angleRates[1] * c2;
    B_inv_deriv[0][1] = 0;
    B_inv_deriv[0][2] = 0;
    B_inv_deriv[1][0] = angleRates[2] * c3 * c2 - angleRates[1] * s3 * s2;
    B_inv_deriv[1][1] = -angleRates[2] * s3;
    B_inv_deriv[1][2] = 0;
    B_inv_deriv[2][0] = -angleRates[2] * s3 * c2 - angleRates[1] * c3 * c2;
    B_inv_deriv[2][1] = -angleRates[2] * c3;
    B_inv_deriv[2][2] = 0;
}

/*!
 Function: computeEulerRotationReference
 Purpose: This function computes the reference (MRP attitude Set, angular velocity and angular acceleration)
 associated with a rotation defined in terms of an (3-2-1) - Euler Angle set
 Input
      ConfigData = module configuration data
      inputRef = input base reference
 Output:
      ConfigData: AttRefFswMsg is computed
 */
void computeEulerRotationReference(eulerRotationConfig *ConfigData,
                                   double sigma_R0N[3],
                                   double omega_R0N_N[3],
                                   double domega_R0N_N[3])
{
    /* Compute attitude reference*/
    double attIncrement[3];         /*!< [] increment in attitude coordinates  */
    double RR0[3][3];               /*!< [] DCM rotating from R0 to R */
    double R0N[3][3];               /*!< [] DCM rotating from N to R0 */
    double RN[3][3];                /*!< [] DCM rotating from N to R */
    
    MRP2C(sigma_R0N, R0N);
    v3Scale(ConfigData->dt, ConfigData->angleRates, attIncrement);
    v3Add(ConfigData->angleSet, attIncrement, ConfigData->angleSet);
    Euler3212C(ConfigData->angleSet, RR0);
    m33MultM33(RR0, R0N, RN);
    C2MRP(RN, ConfigData->attRefOut.sigma_RN);
    
    /* Compute angular velocity */
    double B_inv[3][3];             /*!< [] matrix related Euler angle rates to angular velocity vector components */
    double omega_RR0_R[3];          /*!< [r/s] angular velocity vector between R and R0 frame in R frame components */
    double omega_RR0_N[3];          /*!< [r/s] angular velocity vector between R and R0 frame in N frame components */
    BinvEuler321(ConfigData->angleSet, B_inv);
    m33MultV3(B_inv, ConfigData->angleRates, omega_RR0_R);
    m33tMultV3(RN, omega_RR0_R, omega_RR0_N);
    v3Add(omega_R0N_N, omega_RR0_N, ConfigData->attRefOut.omega_RN_N);
 
    /* Compute angular acceleration */
    double B_inv_deriv[3][3];       /*!< [] time derivatie of matrix relating EA rates to omegas */
    double domega_RR0_R[3];         /*!< [r/s] inertial derivative of omega_RR0_R in R frame components */
    double domega_RR0_N[3];         /*!< [r/s] inertial derivative of omega_RR0_R in N frame components */
    double temp[3];
    computeEuler321_Binv_derivative(ConfigData->angleSet, ConfigData->angleRates, B_inv_deriv);
    m33MultV3(B_inv_deriv, ConfigData->angleRates, domega_RR0_R);
    m33tMultV3(RN, domega_RR0_R, domega_RR0_N);
    v3Cross(omega_R0N_N, omega_RR0_N, temp);
    v3Add(temp, domega_RR0_N, domega_RR0_N);
    v3Add(domega_RR0_N, domega_R0N_N, ConfigData->attRefOut.domega_RN_N);
}
