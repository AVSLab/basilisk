/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "fswAlgorithms/attGuidance/locationPointing/locationPointing.h"
#include "string.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/astroConstants.h"
#include <math.h>

/*!
    This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_locationPointing(locationPointingConfig  *configData, int64_t moduleID)
{
    AttGuidMsg_C_init(&configData->attGuidOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_locationPointing(locationPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{

    // check if the required message has not been connected
    if (!SCStatesMsg_C_isLinked(&configData->scInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: locationPointing.SCInMsg was not connected.");
    }
    if (!GroundStateMsg_C_isLinked(&configData->locationInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: locationPointing.LocationInMsg was not connected.");
    }

    configData->counter = 1;

    v3SetZero(configData->sigma_RB_old);
    v3SetZero(configData->omega_RN_B_old);
    configData->time_old = callTime;
}


/*! This method takes the estimated body states and position relative to the ground to compute the current attitude/attitude rate errors and pass them to control.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_locationPointing(locationPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /* Local copies*/
    SCStatesMsgPayload scInMsgBuffer;  //!< local copy of message buffer
    GroundStateMsgPayload locationInMsgBuffer;  //!< local copy of message buffer
    AttGuidMsgPayload attGuidOutMsgBuffer;  //!< local copy of output message buffer

    double r_LS_N[3];                   /*!< Position vector of location w.r.t spacecraft CoM in inertial frame */
    double r_LS_B[3];                   /*!< Position vector of location w.r.t spacecraft CoM in body frame */
    double eHat_B[3];                   /*!< --- Eigen Axis */
    double dcmBN[3][3];                 /*!< inertial spacecraft orientation DCM */
    double phi;                         /*!< principal angle between pHat and heading to location */
    double sigma_RB_Dot[3];             /*!< time derivative of sigma_BR*/
    double sigma_RB[3];                 /*!< MRP of R relative to B */
    double difference[3];
    double time_diff;                   /*!< module update time */
    double Bmat[3][3];                  /*!< BinvMRP for dsigma_RB_R calculations*/

    // zero output buffer
    attGuidOutMsgBuffer = AttGuidMsg_C_zeroMsgPayload();

    // read in the input messages
    scInMsgBuffer = SCStatesMsg_C_read(&configData->scInMsg);
    locationInMsgBuffer = GroundStateMsg_C_read(&configData->locationInMsg);

    /* calculate r_LS_N*/
    v3Subtract(locationInMsgBuffer.r_LN_N, scInMsgBuffer.r_CN_N, r_LS_N);

    /* principle rotation angle to point pHat at location */
    MRP2C(scInMsgBuffer.sigma_BN, dcmBN);
    m33MultV3(dcmBN, r_LS_N, r_LS_B);
    phi = acos(v3Dot(configData->pHat_B, r_LS_N)/v3Norm(r_LS_N));

    /* calculate eHat*/
    v3Cross(configData->pHat_B, r_LS_N, eHat_B);
    v3Normalize(eHat_B, eHat_B);


    /* can calculate sigma now*/
    v3Scale(tan(phi / 4), eHat_B, sigma_RB);
    v3Scale(-1, sigma_RB, attGuidOutMsgBuffer.sigma_BR);

    /* use sigma_BR to compute dsigma_BR if at least two data points*/
        /* counter keeps track of how many update cycles have run, need 2 for dsigma_BR and 4 for domega_RN_B*/
    if (configData->counter >= 2) {
        // update values for accurate finite diff
        v3Copy(configData->sigma_RB_new, configData->sigma_RB_old);
        v3Copy(sigma_RB, configData->sigma_RB_new);

        // get time data
        configData->time_old = configData->time_new;
        configData->time_new = callTime;
            // converted to seconds
        time_diff = (configData->time_new - configData->time_old)*1.0e-9;

        // assume difference is known - calculate dsigma_BR
        v3Subtract(configData->sigma_RB_new, configData->sigma_RB_old, difference);
        v3Scale(1/(time_diff), difference, sigma_RB_Dot);

        // calculate BinvMRP
        BinvMRP(sigma_RB, Bmat);
        
        // compute omega_BR_R
        v3Scale(4, sigma_RB_Dot, sigma_RB_Dot);
        m33MultV3(Bmat, sigma_RB_Dot, attGuidOutMsgBuffer.omega_BR_B);

        /* compute omega_RN_B (subtract as need omega_RB not omega_BR)*/
        v3Subtract(scInMsgBuffer.omega_BN_B, attGuidOutMsgBuffer.omega_BR_B, attGuidOutMsgBuffer.omega_RN_B);
           
        // if performed finite diff twice, then have enough for domega
        if (configData->counter >= 4) {
            // update time dependent data
//            configData->omega_RN_B_old = configData->omega_RN_B_new;
//            configData->omega_RN_B_new = attGuidOutMsgBuffer.omega_RN_B;
//
//            // perform difference and compute finite diff
//            v3Subtract(omega_RN_B_new, omega_RN_B_old, difference);
//            v3Scale(1. / (time_diff), difference, attGuidOutMsgBuffer.domega_RN_B);

        }
    }

    // update counter and data buffers
    configData->counter++;
    v3Copy(configData->sigma_RB_old, attGuidOutMsgBuffer.sigma_BR);
    v3Copy(configData->omega_RN_B_old, attGuidOutMsgBuffer.omega_RN_B);
    configData->time_new = callTime;

    // write to the output messages
    AttGuidMsg_C_write(&attGuidOutMsgBuffer, &configData->attGuidOutMsg, moduleID, callTime);
}

