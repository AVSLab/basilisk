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
#include "attGuidance/singleAxisSpin/singleAxisSpin.h"
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
void SelfInit_singleAxisSpin(singleAxisSpinConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(attRefOut),
                                               "attRefOut",
                                               moduleID);
    ConfigData->mnvrStartTime = -1;
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_singleAxisSpin(singleAxisSpinConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_singleAxisSpin(singleAxisSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    ConfigData->mnvrStartTime = -1;
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_singleAxisSpin(singleAxisSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Compute output */
    computeSingleAxisSpinReference(ConfigData, callTime);
    
    /*! - Write output message */
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attRefOut),
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    return;
}


/* Function: computeSingleAxisSpinReference */
void computeSingleAxisSpinReference(singleAxisSpinConfig *ConfigData, uint64_t callTime)
{
    double currMnvrTime;
    double PRV_spin[3];
    double C_spin[3][3];
    double R0N[3][3];
    double RN[3][3];
    if (ConfigData->mnvrStartTime == -1)
    {
        ConfigData->mnvrStartTime = callTime;
    }
    currMnvrTime = (callTime - ConfigData->mnvrStartTime)*1.0E-9;
    v3Scale(currMnvrTime, ConfigData->omega_spin, PRV_spin);
    PRV2C(PRV_spin, C_spin);
    MRP2C(ConfigData->sigma_R0N, R0N);
    m33MultM33(C_spin, R0N, RN);
    C2MRP(RN, ConfigData->attRefOut.sigma_RN);
    v3Copy(ConfigData->omega_spin, ConfigData->attRefOut.omega_RN_N);
    v3SetZero(ConfigData->attRefOut.domega_RN_N);
}