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
    Attitude Tracking Error Module
 
 */

/* modify the path to reflect the new module names */
#include "attGuidance/attTrackingError/attTrackingError.h"
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
void SelfInit_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(AttGuidFswMsg),
                                               "AttGuidFswMsg",
                                               moduleID);
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
                                                sizeof(AttRefFswMsg),
                                                moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavName,
                                                sizeof(NavAttIntMsg),
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
    AttRefFswMsg ref;                      /*!< reference guidance message */
    NavAttIntMsg nav;                      /*!< navigation message */

    /*! Begin method steps*/
    /*! - Read the input messages */
    memset(&ref, 0x0, sizeof(AttRefFswMsg));
    memset(&nav, 0x0, sizeof(NavAttIntMsg));
    ReadMessage(ConfigData->inputRefID, &clockTime, &readSize,
                sizeof(AttRefFswMsg), (void*) &(ref), moduleID);
    ReadMessage(ConfigData->inputNavID, &clockTime, &readSize,
                sizeof(NavAttIntMsg), (void*) &(nav), moduleID);


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


    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttGuidFswMsg),   /* update module name */
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
    double      dcm_BN[3][3];               /*!< DCM from inertial to body frame */

    /* compute the initial reference frame orientation that takes the corrected body frame into account */
    v3Scale(-1.0, ConfigData->sigma_R0R, sigma_RR0);
    addMRP(sigma_R0N, sigma_RR0, sigma_RN);

    subMRP(sigma_BN, sigma_RN, sigma_BR);               /* compute attitude error */

    MRP2C(sigma_BN, dcm_BN);                                /* [BN] */
    m33MultV3(dcm_BN, omega_RN_N, omega_RN_B);              /* compute reference omega in body frame components */

    v3Subtract(omega_BN_B, omega_RN_B, omega_BR_B);     /* delta_omega = omega_B - [BR].omega.r */

    m33MultV3(dcm_BN, domega_RN_N, domega_RN_B);            /* compute reference d(omega)/dt in body frame components */

}

