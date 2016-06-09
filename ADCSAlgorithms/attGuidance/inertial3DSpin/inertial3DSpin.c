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

/* modify the path to reflect the new module names */
#include "attGuidance/inertial3DSpin/inertial3DSpin.h"
#include <string.h>
#include "ADCSUtilities/ADCSDefinitions.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"

/* update this include to reflect the required module input messages */
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"



/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"


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
                                               sizeof(attRefOut),
                                               "attRefOut",
                                               moduleID);
    ConfigData->priorTime = -1;
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{

    ConfigData->priorTime = -1;              /* reset the prior time flag state.  If set
                                             to zero, the control time step is not evaluated on the
                                             first function call */
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    double              dt;                 /*!< [s] module update period */

    /* compute control update time */
    if (ConfigData->priorTime == -1)
    {
        dt = 0.0;
    }
    else {
        dt = (callTime - ConfigData->priorTime)*NANO2SEC;
    }
    ConfigData->priorTime = callTime;
    /* compute and store output message */
    computeInertialSpinReference(ConfigData, dt);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attRefOut),   /* update module name */
                 (void*) &(ConfigData->attRefOut), moduleID);

    return;
}


/*
 * Function: computeInertialSpinReference
 * Purpose: compute the reference frame states for the Inertial 3D spin control mode.  This function is
 designed to work both here in FSW to compute estimated pointing errors, as well as in the
 simulation code to compute true pointing errors
 * Inputs:
     ConfigData = module configuration data
 *   integrateFlag = flag to reset the reference orientation
 *                   0 - integrate & evaluate
 *                  -1 - evalute but not integrate)
 *   dt = integration time step (control update period )
 * Outputs:
 *   sigma_RN = MRP attitude error of body relative to reference
 *   omega_RN_N = reference angluar velocity vector in body frame components
 *   domega_RN_N = reference angular acceleration vector in body frame componets
 */
void computeInertialSpinReference(inertial3DSpinConfig *ConfigData, double dt)
{
    double  RN[3][3];               /*!< DCM from inertial to reference frame */
    double  B[3][3];                /*!< MRP rate matrix */
    double  v3Temp[3];              /*!< temporary 3x1 matrix */
    double  omega_RN_R[3];          /*!< reference angular velocity vector in Reference frame R components */


    if (ConfigData->integrateFlag == BOOL_TRUE) {
        /* integrate reference attitude motion */
        MRP2C(ConfigData->sigma_R0N, RN);
        m33MultV3(RN, ConfigData->omega_R0N_N, omega_RN_R);
        BmatMRP(ConfigData->sigma_R0N, B);
        m33Scale(0.25*dt, B, B);
        m33MultV3(B, omega_RN_R, v3Temp);
        v3Add(ConfigData->sigma_R0N, v3Temp, ConfigData->sigma_R0N);
        MRPswitch(ConfigData->sigma_R0N, 1.0, ConfigData->sigma_R0N);
    }

    v3Copy(ConfigData->sigma_R0N, ConfigData->attRefOut.sigma_RN);
    v3Copy(ConfigData->omega_R0N_N, ConfigData->attRefOut.omega_RN_N);
    v3SetZero(ConfigData->attRefOut.domega_RN_N);

}
