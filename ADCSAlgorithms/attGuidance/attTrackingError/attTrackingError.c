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
    Attitude Tracking Error Module
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */

/* modify the path to reflect the new module names */
#include "attGuidance/attTrackingError/attTrackingError.h"
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
void SelfInit_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(attGuidOut),
                                               "attGuidOut",
                                               moduleID);

    ConfigData->sigma_BcB = ConfigData->sigma_R0R;      /* these two relative orientations labels are the same */

}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputRefID = subscribeToMessage(ConfigData->inputRefName,
                                                sizeof(attRefOut),
                                                moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavName,
                                                sizeof(NavStateOut),
                                                moduleID);

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{

}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the attitude tracking error module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t    clockTime;
    uint32_t    readSize;
    attRefOut   ref;                        /*!< reference guidance message */
    NavStateOut nav;                        /*!< navigation message */

    /*! Begin method steps*/
    /*! - Read the input messages */
    memset(&ref, 0x0, sizeof(attRefOut));
    memset(&nav, 0x0, sizeof(NavStateOut));
    ReadMessage(ConfigData->inputRefID, &clockTime, &readSize,
                sizeof(attRefOut), (void*) &(ref));
    ReadMessage(ConfigData->inputNavID, &clockTime, &readSize,
                sizeof(NavStateOut), (void*) &(nav));


    computeAttitudeError(nav.sigma_BN,
                         nav.omega_BN_B,
                         ref.sigma_RN,
                         ref.omega_RN_N,
                         ref.domega_RN_N,
                         ConfigData,
                         ConfigData->attGuidOut.sigma_BR,
                         ConfigData->attGuidOut.omega_BR_B,
                         ConfigData->attGuidOut.omega_RN_B,
                         ConfigData->attGuidOut.domega_RN_B);


    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attGuidOut),   /* update module name */
                 (void*) &(ConfigData->attGuidOut), moduleID);

    return;
}


void computeAttitudeError(double sigma_BN[3],
                          double omega_BN_B[3],
                          double sigma_R0N[3],
                          double omega_RN_N[3],
                          double domega_RN_N[3],
                          attTrackingErrorConfig *ConfigData,
                          double sigma_BR[3],
                          double omega_BR_B[3],
                          double omega_RN_B[3],
                          double domega_RN_B[3])
{
    double      sigma_RR0[3];               /*!< MRP from the original reference frame R0 to the corrected reference frame R */
    double      sigma_RN[3];                /*!< MRP from inertial to updated reference frame */
    double      BN[3][3];                   /*!< DCM from inertial to body frame */

    /* compute the initial reference frame orientation that takes the corrected body frame into account */
    v3Scale(-1.0, ConfigData->sigma_R0R, sigma_RR0);
    addMRP(sigma_R0N, sigma_RR0, sigma_RN);

    subMRP(sigma_BN, sigma_RN, sigma_BR);               /* compute attitude error */

    MRP2C(sigma_BN, BN);                                /* [BN] */
    m33MultV3(BN, omega_RN_N, omega_RN_B);              /* compute reference omega in body frame components */

    v3Subtract(omega_BN_B, omega_RN_B, omega_BR_B);     /* delta_omega = omega_B - [BR].omega.r */

    m33MultV3(BN, domega_RN_N, domega_RN_B);            /* compute reference d(omega)/dt in body frame components */

}

