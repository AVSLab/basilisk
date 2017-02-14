/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "attGuidance/eulerRotation/eulerRotation.h"
#include <string.h>
#include <math.h>
#include "fswUtilities/fswDefinitions.h"
#include "SimFswInterfaceMessages/macroDefinitions.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"




void SelfInit_eulerRotation(eulerRotationConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(AttRefMessage),
                                               "AttRefMessage",
                                               moduleID);
    ConfigData->outputEulerSetID = CreateNewMessage(ConfigData->outputEulerSetName,
                                               sizeof(EulerAngleMessage),
                                               "EulerAngleMessage",
                                                    moduleID);
    ConfigData->outputEulerRatesID = CreateNewMessage(ConfigData->outputEulerRatesName,
                                                    sizeof(EulerAngleMessage),
                                                    "EulerAngleMessage",
                                                    moduleID);
    ConfigData->priorTime = -1;
    v3SetZero(ConfigData->priorCmdSet);
    v3SetZero(ConfigData->priorCmdRates);
}

void CrossInit_eulerRotation(eulerRotationConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputRefID = subscribeToMessage(ConfigData->inputRefName,
                                                sizeof(AttRefMessage),
                                                moduleID);
    
    ConfigData->inputEulerSetID = ConfigData->inputEulerRatesID = -1;
    if(strlen(ConfigData->inputEulerSetName) > 0 && strlen(ConfigData->inputEulerRatesName) > 0)
    {
        ConfigData->inputEulerSetID = subscribeToMessage(ConfigData->inputEulerSetName,
                                                         sizeof(EulerAngleMessage),
                                                         moduleID);
        ConfigData->inputEulerRatesID = subscribeToMessage(ConfigData->inputEulerRatesName,
                                                           sizeof(EulerAngleMessage),
                                                           moduleID);
    }
}

void Reset_eulerRotation(eulerRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    ConfigData->priorTime = -1;
    v3SetZero(ConfigData->priorCmdSet);
    v3SetZero(ConfigData->priorCmdRates);
}


void Update_eulerRotation(eulerRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read input messages */
    AttRefMessage inputRef;
    EulerAngleMessage angles;
    EulerAngleMessage rates;
    uint64_t writeTime;
    uint32_t writeSize;
    ReadMessage(ConfigData->inputRefID, &writeTime, &writeSize,
                sizeof(AttRefMessage), (void*) &(inputRef), moduleID);
    if (ConfigData->inputEulerSetID > 0 && ConfigData->inputEulerRatesID > 0)
    {
        /*! - Read Raster Manager messages */
        ReadMessage(ConfigData->inputEulerSetID, &writeTime, &writeSize,
                    sizeof(EulerAngleMessage), (void*) &(angles), moduleID);
        ReadMessage(ConfigData->inputEulerRatesID, &writeTime, &writeSize,
                    sizeof(EulerAngleMessage), (void*) &(rates), moduleID);
        /*! - Save commanded 321 Euler set and rates */
        v3Copy(angles.set, ConfigData->cmdSet);
        v3Copy(rates.set, ConfigData->cmdRates);
        /*! - Check the command is new */
        checkRasterCommands(ConfigData);
    }
    
    /*! - Compute time step to use in the integration downstream */
    computeTimeStep(ConfigData, callTime);
    /*! - Compute output reference frame */
    computeEulerRotationReference(ConfigData,
                                  inputRef.sigma_RN,
                                  inputRef.omega_RN_N,
                                  inputRef.domega_RN_N);
    /*! - Write output messages */
    writeOutputMessages(ConfigData, callTime, moduleID);
    /*! - Update last time the module was called to current call time */
    ConfigData->priorTime = callTime;
    return;
}

/*
 * Function: writeOutputMessages
 * Purpose: This function writes the the generated reference and the computed euler angle set in different messages
 * Input
 *      ConfigData = module configuration data
 *      moduleID = message ID of this module (eulerRotation)
 *      callTime = current simulation time when the module is called
 * Output: (-)
 */
void writeOutputMessages(eulerRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Guidance reference output */
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttRefMessage),
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    /*! - Euler angle set and rates outputed for testing purposes */
    v3Copy(ConfigData->angleSet, ConfigData->eulerSetOut.set);
    WriteMessage(ConfigData->outputEulerSetID, callTime, sizeof(EulerAngleMessage),
                 (void*) &(ConfigData->eulerSetOut), moduleID);
    v3Copy(ConfigData->angleRates, ConfigData->eulerRatesOut.set);
    WriteMessage(ConfigData->outputEulerRatesID, callTime, sizeof(EulerAngleMessage),
                 (void*) &(ConfigData->eulerRatesOut), moduleID);
}


/*
 * Function: checkRasterCommands
 * Purpose: This function checks if there is a new commanded raster maneuver available
 * Input
 *      ConfigData = module configuration data
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

/*
 * Function: computeTimeStep
 * Purpose: This function computes control update time
 * Input
 *      ConfigData = module configuration data
 * Output:
 *      ConfigData: dt is updated
 */
void computeTimeStep(eulerRotationConfig *ConfigData, uint64_t callTime)
{
    if (ConfigData->priorTime == -1)
    {
        ConfigData->dt = 0.0;
    } else {
        ConfigData->dt = (callTime - ConfigData->priorTime)*NANO2SEC;
    }
}

/*
 * Function: computeEuler321_Binv_derivative
 * Purpose: This function computes the analytical derivative of the B_inv matrix for the 3-2-1 Euler Angle set
 * Input
 *      angleSet: 3-2-1 euler angles
 *      angleRates: 3-2-1 euler angle rates
 * Output:
 *      B_inv_deriv
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

/*
 * Function: computeEulerRotationReference
 * Purpose: This function computes the reference (MRP attitude Set, angular velocity and angular acceleration)
 associated with a rotation defined in terms of an 123 - Euler Angle set
 * Input
 *      ConfigData = module configuration data
 *      inputRef = input base reference
 * Output:
 *      ConfigData: AttRefMessage is computed
 */
void computeEulerRotationReference(eulerRotationConfig *ConfigData,
                                   double sigma_R0N[3],
                                   double omega_R0N_N[3],
                                   double domega_R0N_N[3])
{
    /* Compute attitude reference*/
    double angleVar[3];
    double RR0[3][3];
    double R0N[3][3];
    double RN[3][3];
    MRP2C(sigma_R0N, R0N);
    v3Scale(ConfigData->dt, ConfigData->angleRates, angleVar);
    v3Add(ConfigData->angleSet, angleVar, ConfigData->angleSet);
    Euler3212C(ConfigData->angleSet, RR0);
    m33MultM33(RR0, R0N, RN);
    C2MRP(RN, ConfigData->attRefOut.sigma_RN);
    
    /* Compute angular velocity */
    double B_inv[3][3];
    double omega_RR0_R[3];
    double omega_RR0_N[3];
    BinvEuler321(ConfigData->angleSet, B_inv);
    m33MultV3(B_inv, ConfigData->angleRates, omega_RR0_R);
    m33tMultV3(RN, omega_RR0_R, omega_RR0_N);
    v3Add(omega_R0N_N, omega_RR0_N, ConfigData->attRefOut.omega_RN_N);
 
    /* Compute angular acceleration */
    double B_inv_deriv[3][3];
    double domega_RR0_R[3];
    double domega_RR0_N[3];
    double temp[3];
    computeEuler321_Binv_derivative(ConfigData->angleSet, ConfigData->angleRates, B_inv_deriv);
    m33MultV3(B_inv_deriv, ConfigData->angleRates, domega_RR0_R);
    m33tMultV3(RN, domega_RR0_R, domega_RR0_N);
    v3Cross(omega_R0N_N, omega_RR0_N, temp);
    v3Add(temp, domega_RR0_N, domega_RR0_N);
    v3Add(domega_RR0_N, domega_R0N_N, ConfigData->attRefOut.domega_RN_N);
}
