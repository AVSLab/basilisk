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
 MRP Rotation Guidance Module with a Constant Body Rate Vector
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved
 
 */

#include "attGuidance/mrpRotation/mrpRotation.h"
#include <string.h>
#include <math.h>
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"




void SelfInit_mrpRotation(mrpRotationConfig *ConfigData, uint64_t moduleID)
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

void CrossInit_mrpRotation(mrpRotationConfig *ConfigData, uint64_t moduleID)
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

void Reset_mrpRotation(mrpRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    ConfigData->priorTime = 0;
    v3SetZero(ConfigData->priorCmdSet);
    v3SetZero(ConfigData->priorCmdRates);
}


void Update_mrpRotation(mrpRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
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
    
    /*! - Compute time step to use in the integration downstream */
    computeTimeStep(ConfigData, callTime);
    /*! - Compute output reference frame */
    computeMRPRotationReference(ConfigData,
                                  inputRef.sigma_RN,
                                  inputRef.omega_RN_N,
                                  inputRef.domega_RN_N);
    /*! - Write output messages */
    writeOutputMessages(ConfigData, callTime, moduleID);
    /*! - Update last time the module was called to current call time */
    ConfigData->priorTime = callTime;
    return;
}

/*!
   Function: writeOutputMessages
   Purpose: This function writes the the generated reference and the computed euler angle set in different messages
   Input
        ConfigData = module configuration data
        moduleID = message ID of this module (mrpRotation)
        callTime = current simulation time when the module is called
   Output: (-)
 */
void writeOutputMessages(mrpRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Guidance reference output */
    WriteMessage(ConfigData->attRefOutMsgID, callTime, sizeof(AttRefFswMsg),
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    /*! - Euler angle set and rates outputed for testing purposes */
    if (ConfigData->attitudeOutMsgID >= 0) {
        v3Copy(ConfigData->mrpSet, ConfigData->attStateOut.state);
        v3Copy(ConfigData->omega_RR0_R, ConfigData->attStateOut.rate);
        WriteMessage(ConfigData->attitudeOutMsgID, callTime, sizeof(AttStateFswMsg),
                     (void*) &(ConfigData->attStateOut), moduleID);
    }
}


/*!
   Function: checkRasterCommands
   Purpose: This function checks if there is a new commanded raster maneuver available
   Input:
        ConfigData = module configuration data
 */
void checkRasterCommands(mrpRotationConfig *ConfigData)
{
    int32_t prevCmdActive = v3IsEqual(ConfigData->cmdSet, ConfigData->priorCmdSet , 1E-12) && v3IsEqual(ConfigData->cmdRates, ConfigData->priorCmdRates , 1E-12);
    if (!prevCmdActive)
    {
        v3Copy(ConfigData->cmdSet, ConfigData->mrpSet);
        v3Copy(ConfigData->cmdRates, ConfigData->omega_RR0_R);
        
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
void computeTimeStep(mrpRotationConfig *ConfigData, uint64_t callTime)
{
    if (ConfigData->priorTime == 0)
    {
        ConfigData->dt = 0.0;
    } else {
        ConfigData->dt = (callTime - ConfigData->priorTime)*NANO2SEC;
    }
}


/*!
 Function: computeMRPRotationReference
 Purpose: This function computes the reference (MRP attitude Set, angular velocity and angular acceleration)
 associated with a rotation defined in terms of an initial MRP set and a constant angular velocity vector
 Input
      ConfigData = module configuration data
      inputRef = input base reference
 Output:
      ConfigData: AttRefFswMsg is computed
 */
void computeMRPRotationReference(mrpRotationConfig *ConfigData,
                                   double sigma_R0N[3],
                                   double omega_R0N_N[3],
                                   double domega_R0N_N[3])
{
    /* Compute attitude reference*/
    double attIncrement[3];         /*!< [] increment in attitude coordinates  */
    double RR0[3][3];               /*!< [] DCM rotating from R0 to R */
    double R0N[3][3];               /*!< [] DCM rotating from N to R0 */
    double RN[3][3];                /*!< [] DCM rotating from N to R */
    double sigmaDot_RR0[3];         /*!< [1/s] MRP rates */
    double B[3][3];                 /*!< [] matrix relating omega to MRP rates */
    double omega_RR0_N[3];          /*!< [r/s] angular velocity of R frame relative to input R0 frame */
    double domega_RR0_N[3];         /*!< [r/s^2] inertial derivative of angular velocity vector between R and R0 frames */

    MRP2C(sigma_R0N, R0N);
    BmatMRP(ConfigData->mrpSet, B);
    m33MultV3(B, ConfigData->omega_RR0_R, sigmaDot_RR0);
    v3Scale(0.25, sigmaDot_RR0, sigmaDot_RR0);
    v3Scale(ConfigData->dt, sigmaDot_RR0, attIncrement);
    v3Add(ConfigData->mrpSet, attIncrement, ConfigData->mrpSet);
    MRP2C(ConfigData->mrpSet, RR0);
    m33MultM33(RR0, R0N, RN);
    C2MRP(RN, ConfigData->attRefOut.sigma_RN);
    
    /* Compute angular velocity */
    m33tMultV3(RN, ConfigData->omega_RR0_R, omega_RR0_N);
    v3Add(omega_R0N_N, omega_RR0_N, ConfigData->attRefOut.omega_RN_N);
 
    /* Compute angular acceleration */
    v3Cross(omega_R0N_N, omega_RR0_N, domega_RR0_N);
    v3Add(domega_RR0_N, domega_R0N_N, ConfigData->attRefOut.domega_RN_N);
}
