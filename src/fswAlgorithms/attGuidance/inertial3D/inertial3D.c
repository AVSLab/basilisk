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
 
 */

/* modify the path to reflect the new module names */
#include "attGuidance/inertial3D/inertial3D.h"
#include <string.h>
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"




/* Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory. */
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"


void SelfInit_inertial3D(inertial3DConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(AttRefFswMsg),
                                               "AttRefFswMsg",
                                               moduleID);
}

void CrossInit_inertial3D(inertial3DConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
}

void Reset_inertial3D(inertial3DConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
}

void Update_inertial3D(inertial3DConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
    /*! - Compute and store output message */
    computeInertialPointingReference(ConfigData);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttRefFswMsg),   /* update module name */
                 (void*) &(ConfigData->attRefOut), moduleID);

    return;
}


/*
 * Function: computeInertialPointingReference
 * Purpose: Generate attitude reference associated with Intertial 3D Pointing
 * Inputs:
     ConfigData = module configuration data
 * Outputs:
 *   sigma_RN = Current attitude error estimate (MRPs)
 *   omega_RN_N = [r/s]  Reference frame rate vector of the of R relative to N in N frame components
 *   domega_RN_N = [r/s2] Reference frame inertial acceleration of  R relative to N in N frame components
 */
void computeInertialPointingReference(inertial3DConfig *ConfigData)
{
    v3Copy(ConfigData->sigma_R0N, ConfigData->attRefOut.sigma_RN);
    v3SetZero(ConfigData->attRefOut.omega_RN_N);
    v3SetZero(ConfigData->attRefOut.domega_RN_N);
}
