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

#include "attGuidance/attTrackingError/attTrackingError.h"
#include <string.h>
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"


/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the attitude tracking error module
 */
void SelfInit_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t moduleID)
{
        /*! Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(AttGuidFswMsg),
                                               "AttGuidFswMsg",
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the attitude tracking error module
 */
void CrossInit_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the reference and navigation data message ID*/
    ConfigData->inputRefID = subscribeToMessage(ConfigData->inputRefName,
                                                sizeof(AttRefFswMsg),
                                                moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavName,
                                                sizeof(NavAttIntMsg),
                                                moduleID);

}

/*! This method performs a complete reset of the module. Local module variables that retain time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the attitude tracking error module
 */
void Reset_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{

}

/*! The Update method performs reads the Navigation message (containing the spacecraft attitude information), and the Reference message (containing the desired attitude). It computes the attitude error and writes it in the Guidance message.
 @return void
 @param ConfigData The configuration data associated with the attitude tracking error module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The Basilisk module identifier
 */
void Update_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t    timeOfMsgWritten;
    uint32_t    sizeOfMsgWritten;
    AttRefFswMsg ref;                      /* reference guidance message */
    NavAttIntMsg nav;                      /* navigation message */

    /*! - Read the input messages */
    memset(&ref, 0x0, sizeof(AttRefFswMsg));
    memset(&nav, 0x0, sizeof(NavAttIntMsg));
    ReadMessage(ConfigData->inputRefID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(AttRefFswMsg), (void*) &(ref), moduleID);
    ReadMessage(ConfigData->inputNavID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(NavAttIntMsg), (void*) &(nav), moduleID);

    computeAttitudeError(nav, ref, ConfigData);

    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttGuidFswMsg),   /*! - Write guidance message */
                 (void*) &(ConfigData->attGuidOut), moduleID);

    return;
}

/*! This method performs the attitude computations in order to extract the error.
 @return void
 @param nav The spacecraft attitude information
 @param ref The reference attitude
 @param ConfigData The configuration data associated with the attitude tracking error module
 */
void computeAttitudeError(NavAttIntMsg nav, AttRefFswMsg ref, attTrackingErrorConfig *ConfigData){
    double      sigma_RR0[3];               /* MRP from the original reference frame R0 to the corrected reference frame R */
    double      sigma_RN[3];                /* MRP from inertial to updated reference frame */
    double      dcm_BN[3][3];               /* DCM from inertial to body frame */
    
    /*! - compute the initial reference frame orientation that takes the corrected body frame into account */
    v3Scale(-1.0, ConfigData->sigma_R0R, sigma_RR0);
    addMRP(ref.sigma_RN, sigma_RR0, sigma_RN);
    
    subMRP(nav.sigma_BN, sigma_RN, ConfigData->attGuidOut.sigma_BR);               /*! - compute attitude error */
    
    MRP2C(nav.sigma_BN, dcm_BN);                                /* [BN] */
    m33MultV3(dcm_BN, ref.omega_RN_N, ConfigData->attGuidOut.omega_RN_B);              /*! - compute reference omega in body frame components */
    
    v3Subtract(nav.omega_BN_B, ConfigData->attGuidOut.omega_RN_B, ConfigData->attGuidOut.omega_BR_B);     /*! - delta_omega = omega_B - [BR].omega.r */
    
    m33MultV3(dcm_BN, ref.domega_RN_N, ConfigData->attGuidOut.domega_RN_B);            /*! - compute reference d(omega)/dt in body frame components */
    
}

