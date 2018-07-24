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
    Inertial 3D Spin Module
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */

/* modify the path to reflect the new module names */
#include "attGuidance/inertial3DSpin/inertial3DSpin.h"
#include <string.h>
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"




/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"


/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(AttRefFswMsg),
                                               "AttRefFswMsg",
                                               moduleID);
    ConfigData->priorTime = 0;
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputRefID = subscribeToMessage(ConfigData->inputRefName,
                                                sizeof(AttRefFswMsg),
                                                moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{

    ConfigData->priorTime = 0;              /* reset the prior time flag state.  If set
                                             to zero, the control time step is not evaluated on the
                                             first function call */
}

/*! This method performs all the main computations of the module
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read input message */
    AttRefFswMsg inputRef;
    uint64_t writeTime;
    uint32_t writeSize;
    ReadMessage(ConfigData->inputRefID, &writeTime, &writeSize,
                sizeof(AttRefFswMsg), (void*) &(inputRef), moduleID);
    
    /*! - Get input reference and compute integration time step to use downstream */
    double dt; /* integration time step [s] */
    if (ConfigData->priorTime == 0)
    {
        dt = 0.0;
        v3Copy(inputRef.sigma_RN, ConfigData->sigma_RN);
    } else {
        dt = (callTime - ConfigData->priorTime) * NANO2SEC;
    }
    
    /*! - Generate inertial 3D Spinning Reference */
    computeReference_inertial3DSpin(ConfigData,
                                    inputRef.omega_RN_N,
                                    inputRef.domega_RN_N,
                                    ConfigData->omega_spin,
                                    dt);
    
    /*! - Write output message */
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttRefFswMsg),
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    /*! Update prior time to current for next evaluation */
    ConfigData->priorTime = callTime;
}

void computeReference_inertial3DSpin(inertial3DSpinConfig *ConfigData,
                                     double omega_R0N_N[3],
                                     double domega_R0N_N[3],
                                     double omega_RR0_R[3],
                                     double dt)
{
    double omega_RN_N[3];
    double domega_RN_N[3];
    
    /*! Compute angular rate */
    double dcm_RN[3][3];   /* DCM from inertial frame N to generated ref frame R */
    double omega_RR0_N[3]; /* angular rate of the generated ref R wrt the base ref R0 in inertial N components */
    MRP2C(ConfigData->sigma_RN, dcm_RN);
    m33tMultV3(dcm_RN, omega_RR0_R, omega_RR0_N);
    v3Add(omega_R0N_N, omega_RR0_N, omega_RN_N);
    
    /*! Compute angular acceleration */
    double v3Temp[3]; /* temporary 3x1 array */
    v3Cross(omega_R0N_N, omega_RR0_N, v3Temp);
    v3Add(v3Temp, domega_R0N_N, domega_RN_N);
    
    /*! Integrate Attitude */
    double B[3][3]; /* MRP rate matrix */
    double omega_RN_R[3]; /* inertial angular rate of ref R in R frame components */
    m33MultV3(dcm_RN, omega_RN_N, omega_RN_R);
    BmatMRP(ConfigData->sigma_RN, B);
    m33Scale(0.25 * dt, B, B);
    m33MultV3(B, omega_RN_R, v3Temp);
    v3Add(ConfigData->sigma_RN, v3Temp, ConfigData->sigma_RN);
    MRPswitch(ConfigData->sigma_RN, 1.0, ConfigData->sigma_RN);
    
    /*! Copy output in AttRefFswMsg struct */
    v3Copy(ConfigData->sigma_RN, ConfigData->attRefOut.sigma_RN);
    v3Copy(omega_RN_N, ConfigData->attRefOut.omega_RN_N);
    v3Copy(domega_RN_N, ConfigData->attRefOut.domega_RN_N);
}
