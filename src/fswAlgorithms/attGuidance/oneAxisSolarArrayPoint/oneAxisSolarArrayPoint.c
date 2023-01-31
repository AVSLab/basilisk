/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include "oneAxisSolarArrayPoint.h"
#include "string.h"
#include <math.h>

#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/astroConstants.h"


/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_oneAxisSolarArrayPoint(OneAxisSolarArrayPointConfig *configData, int64_t moduleID)
{
    AttRefMsg_C_init(&configData->attRefOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_oneAxisSolarArrayPoint(OneAxisSolarArrayPointConfig *configData, uint64_t callTime, int64_t moduleID)
{
    if (!NavAttMsg_C_isLinked(&configData->attNavInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, " oneAxisSolarArrayPoint.attNavInMsg wasn't connected.");
    }

    // check how the input body heading is provided
    if (BodyHeadingMsg_C_isLinked(&configData->bodyHeadingInMsg)) {
        configData->bodyAxisInput = inputBodyHeadingMsg;
    }
    else if (v3Norm(configData->h1Hat_B) > EPS) {
            configData->bodyAxisInput = inputBodyHeadingParameter;
    }
    else {
            _bskLog(configData->bskLogger, BSK_ERROR, " oneAxisSolarArrayPoint.bodyHeadingInMsg wasn't connected and no body heading h1Hat_B was specified.");
    }

    // check how the input inertial heading is provided
    if (InertialHeadingMsg_C_isLinked(&configData->inertialHeadingInMsg)) {
        configData->inertialAxisInput = inputInertialHeadingMsg;
        if (EphemerisMsg_C_isLinked(&configData->ephemerisInMsg)) {
            _bskLog(configData->bskLogger, BSK_WARNING, " both oneAxisSolarArrayPoint.inertialHeadingInMsg and oneAxisSolarArrayPoint.ephemerisInMsg were linked. Inertial heading is computed from oneAxisSolarArrayPoint.inertialHeadingInMsg");
        }
    }
    else if (EphemerisMsg_C_isLinked(&configData->ephemerisInMsg)) {
        if (!NavTransMsg_C_isLinked(&configData->transNavInMsg)) {
            _bskLog(configData->bskLogger, BSK_ERROR, " oneAxisSolarArrayPoint.ephemerisInMsg was specified but oneAxisSolarArrayPoint.transNavInMsg was not.");
        }
        else {
            configData->inertialAxisInput = inputEphemerisMsg;
        }
    }
    else {
        if (v3Norm(configData->hHat_N) > EPS) {
            configData->inertialAxisInput = inputInertialHeadingParameter;
        }
        else {
            _bskLog(configData->bskLogger, BSK_ERROR, " neither oneAxisSolarArrayPoint.inertialHeadingInMsg nor oneAxisSolarArrayPoint.ephemerisInMsg were connected and no inertial heading h_N was specified.");
        }
    }

    // set updateCallCount to zero
    configData->updateCallCount = 0;
}

/*! The Update() function computes the reference MRP attitude, reference angular rate and acceleration
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_oneAxisSolarArrayPoint(OneAxisSolarArrayPointConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! create and zero the output message */
    AttRefMsgPayload attRefOut = AttRefMsg_C_zeroMsgPayload();

    /*! read and allocate the attitude navigation message */
    NavAttMsgPayload attNavIn = NavAttMsg_C_read(&configData->attNavInMsg);

    /*! get requested heading in inertial frame */
    double hReqHat_N[3];
    if (configData->inertialAxisInput == inputInertialHeadingParameter) {
        v3Normalize(configData->hHat_N, hReqHat_N);
    }
    else if (configData->inertialAxisInput == inputInertialHeadingMsg) {
        InertialHeadingMsgPayload inertialHeadingIn = InertialHeadingMsg_C_read(&configData->inertialHeadingInMsg);
        v3Normalize(inertialHeadingIn.rHat_XN_N, hReqHat_N);
    }
    else if (configData->inertialAxisInput == inputEphemerisMsg) {
        EphemerisMsgPayload ephemerisIn = EphemerisMsg_C_read(&configData->ephemerisInMsg);
        NavTransMsgPayload transNavIn = NavTransMsg_C_read(&configData->transNavInMsg);
        v3Subtract(ephemerisIn.r_BdyZero_N, transNavIn.r_BN_N, hReqHat_N);
        v3Normalize(hReqHat_N, hReqHat_N);
    }

    /*! get body frame heading */
    double hRefHat_B[3];
    if (configData->bodyAxisInput == inputBodyHeadingParameter) {
        v3Normalize(configData->h1Hat_B, hRefHat_B);
    }
    else if (configData->bodyAxisInput == inputBodyHeadingMsg) {
        BodyHeadingMsgPayload bodyHeadingIn = BodyHeadingMsg_C_read(&configData->bodyHeadingInMsg);
        v3Normalize(bodyHeadingIn.rHat_XB_B, hRefHat_B);
    }
    
    /*! define the body frame orientation DCM BN */
    double BN[3][3];
    MRP2C(attNavIn.sigma_BN, BN);

    /*! get the solar array drive direction in body frame coordinates */
    double a1Hat_B[3];
    v3Normalize(configData->a1Hat_B, a1Hat_B);

    /*! get the second body frame direction */
    double a2Hat_B[3];
    if (v3Norm(configData->a2Hat_B) > EPS) {
        v3Normalize(configData->a2Hat_B, a2Hat_B);
    }
    else {
        v3SetZero(a2Hat_B);
    }

    /*! read Sun direction in B frame from the attNav message */
    double rHat_SB_B[3];
    v3Copy(attNavIn.vehSunPntBdy, rHat_SB_B);

    /*! map requested heading into B frame */
    double hReqHat_B[3];
    m33MultV3(BN, hReqHat_N, hReqHat_B);
}
