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
#include "attGuidance/inertial3D/inertial3D.h"
#include <string.h>
#include "ADCSUtilities/ADCSDefinitions.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"

/* update this include to reflect the required module input messages */
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"



/* Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory. */
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"


void SelfInit_inertial3D(inertial3DConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(attRefOut),
                                               "attRefOut",
                                               moduleID);
    /*!< The inertial pointing is assumed to be fixed, i.e. reference rate is zero */
    v3SetZero(ConfigData->attRefOut.omega_RN_N);
    v3SetZero(ConfigData->attRefOut.domega_RN_N);
}

void CrossInit_inertial3D(inertial3DConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
}

void Reset_inertial3D(inertial3DConfig *ConfigData)
{
    ConfigData->priorTime = 0;               /*!< reset the prior time flag state.  If set
                                              to zero, the control time step is not evaluated on the
                                              first function call */
}

void Update_inertial3D(inertial3DConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
    /*! - Compute control update time */
    double dt;                              /*!< [s] module update period */
    if (ConfigData->priorTime != 0) {       /*!< don't compute dt if this is the first call after a reset */
        dt = (callTime - ConfigData->priorTime)*NANO2SEC;
        if (dt > 10.0) dt = 10.0;           /*!< cap the maximum control time step possible */
        if (dt < 0.0) dt = 0.0;             /*!< ensure no negative numbers are used */
    } else {
        dt = 0.;                            /*!< set dt to zero to not use integration on first function call */
    }
    ConfigData->priorTime = callTime;


    /*! - Compute and store output message */
    computeInertialPointingReference(ConfigData,
                                     ConfigData->attRefOut.sigma_RN,
                                     ConfigData->attRefOut.omega_RN_N,
                                     ConfigData->attRefOut.domega_RN_N);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attRefOut),   /* update module name */
                 (void*) &(ConfigData->attRefOut), moduleID);

    return;
}


/*
 * Function: computeInertialPointingReference
 * Purpose: Generate attitude reference associated with Intertial 3D Pointing
 * Inputs:
     ConfigData = module configuration data
 * Outputs:
 *   sigma_RN = Current attitude error estimate (MRPs) of B relative to R
 *   omega_RN_N = [r/s]  Reference frame rate vector of the of R relative to N in N frame components
 *   domega_RN_N = [r/s2] Reference frame inertial acceleration of  R relative to N in N frame components
 */
void computeInertialPointingReference(inertial3DConfig *ConfigData,
                                      double sigma_RN[3],
                                      double omega_RN_N[3],
                                      double domega_RN_N[3])
{
    v3Copy(ConfigData->sigma_RN, sigma_RN);
    v3SetZero(omega_RN_N);
    v3SetZero(domega_RN_N);
}
