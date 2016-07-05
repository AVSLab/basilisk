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

/*! This method performs all the main computations of the module
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    computeTimeStep(ConfigData, callTime);
    integrateInertialSpinRef(ConfigData);
    evaluateInertial3DSpinRef(ConfigData);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attRefOut),
                 (void*) &(ConfigData->attRefOut), moduleID);
    ConfigData->priorTime = callTime;
    return;
}

/*
 * Function: evaluateInertial3DSpinRef
 * Purpose: This function computes control update time
 * Input
 *      ConfigData = module configuration data
 * Output:
 *      ConfigData: dt is updated
 */
void computeTimeStep(inertial3DSpinConfig *ConfigData, uint64_t callTime)
{
    if (ConfigData->priorTime == -1)
    {
        ConfigData->dt = 0.0;
    } else {
        ConfigData->dt = (callTime - ConfigData->priorTime)*NANO2SEC;
    }
}

/*
 * Function: integrateInertialSpinRef
 * Purpose: This function is designed to integrate the reference attitude
 * Input
 *      ConfigData = module configuration data
 *      dt = integration time step
 * Output:
 *      ConfigData: sigma_RN is updated
 */
void integrateInertialSpinRef(inertial3DSpinConfig *ConfigData)
{
    double  RN[3][3];               /*!< DCM from inertial to reference frame */
    double  B[3][3];                /*!< MRP rate matrix */
    double  v3Temp[3];              /*!< temporary 3x1 array */
    double  omega_RN_R[3];          /*!< reference angular velocity vector in Reference frame R components */
    
    MRP2C(ConfigData->sigma_RN, RN);
    m33MultV3(RN, ConfigData->omega_RN_N, omega_RN_R);
    BmatMRP(ConfigData->sigma_RN, B);
    m33Scale(0.25* ConfigData->dt, B, B);
    m33MultV3(B, omega_RN_R, v3Temp);
    v3Add(ConfigData->sigma_RN, v3Temp, ConfigData->sigma_RN);
    MRPswitch(ConfigData->sigma_RN, 1.0, ConfigData->sigma_RN);
}


/*
 * Function: evaluateInertial3DSpinRef
 * Purpose: This function is designed to evaluate the state of the 3D Spinning Reference
 * Input
 *      ConfigData = module configuration data
 * Output:
 *      ConfigData: attRefOut is updated
 */
void evaluateInertial3DSpinRef(inertial3DSpinConfig *ConfigData)
{
    v3Copy(ConfigData->sigma_RN, ConfigData->attRefOut.sigma_RN);
    v3Copy(ConfigData->omega_RN_N, ConfigData->attRefOut.omega_RN_N);
    v3SetZero(ConfigData->attRefOut.domega_RN_N);
}

