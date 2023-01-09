/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


/* modify the path to reflect the new module names */
#include "solarArrayReference.h"
#include "string.h"
#include <math.h>

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/macroDefinitions.h"


/*!
    This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_solarArrayReference(solarArrayReferenceConfig *configData, int64_t moduleID)
{
    SpinningBodyMsg_C_init(&configData->spinningBodyRefOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_solarArrayReference(solarArrayReferenceConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required input message is included
    if (!NavAttMsg_C_isLinked(&configData->attNavInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayReference.attNavInMsg wasn't connected.");
    }
    // check if the required input message is included
    if (!AttRefMsg_C_isLinked(&configData->attRefInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayReference.attRefInMsg wasn't connected.");
    }
    // check if the required input message is included
    if (!SpinningBodyMsg_C_isLinked(&configData->spinningBodyInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayReference.spinningBodyInMsg wasn't connected.");
    }
}

/*! This method computes the updated rotation angle reference based on current attitude, reference attitude, and current rotation angle
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_solarArrayReference(solarArrayReferenceConfig *configData, uint64_t callTime, int64_t moduleID)
{
     /*! - Create buffer messages */
    NavAttMsgPayload         attNavIn;
    AttRefMsgPayload         attRefIn;
    SpinningBodyMsgPayload   spinningBodyIn;
    SpinningBodyMsgPayload   spinningBodyRefOut;

    /*! - zero the output message */
    spinningBodyRefOut = SpinningBodyMsg_C_zeroMsgPayload();

    /*! read the attitude navigation message */
    attNavIn = NavAttMsg_C_read(&configData->attNavInMsg);

    /*! read the attitude reference message */
    attRefIn = AttRefMsg_C_read(&configData->attRefInMsg);

    /*! read the solar array angle message */
    spinningBodyIn = SpinningBodyMsg_C_read(&configData->spinningBodyInMsg);

    /*! read Sun direction in B frame from the attNav message and map it to R frame */
    double rS_B[3], rS_R[3], BN[3][3], RN[3][3], RB[3][3];
    v3Normalize(attNavIn.vehSunPntBdy, rS_B);
    switch (configData->bodyFrame) {

        case 0:
        MRP2C(attNavIn.sigma_BN, BN);
        MRP2C(attRefIn.sigma_RN, RN);
        m33MultM33t(RN, BN, RB);
        m33MultV3(RB, rS_B, rS_R);
        break;

        case 1:
        v3Copy(rS_B, rS_R);
        break;

        default:
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayAngle.bodyFrame input can be either 0 or 1.");
    }

    /*! store solar array drive axis and solar array normal axis at zero rotation */
    double a1_R[3], a2_R[3], a3_R[3], a1_R_prime[3], a2_R_prime[3];
    v3Normalize(configData->a1_B, a1_R);
    v3Cross(a1_R, configData->a2_B, a3_R);
    v3Normalize(a3_R, a3_R);
    v3Cross(a3_R, a1_R, a2_R);

    /*! compute a2_R_prime */
    double dotP = v3Dot(a1_R, rS_R);
    for (int n = 0; n < 3; n++) {
        a2_R_prime[n] = rS_R[n] - dotP * a1_R[n];
    }
    v3Normalize(a2_R_prime, a2_R_prime);

    /*! compute a1_R_prime */
    v3Cross(a2_R, a2_R_prime, a1_R_prime);

    /*! compute current rotation angle thetaC from input msg */
    double thetaC, sinThetaC, cosThetaC;
    // clip theta current between 0 and 2*pi
    sinThetaC = sin(spinningBodyIn.theta);
    cosThetaC = cos(spinningBodyIn.theta);
    thetaC = atan2(sinThetaC, cosThetaC);

    /*! compute reference angle thetaR and store in buffer msg */
    double thetaR;
    if (v3Norm(a2_R_prime) < EPS) {
        spinningBodyRefOut.theta = spinningBodyIn.theta;
    }
    else {
        thetaR = acos( fmin(fmax(v3Dot(a2_R, a2_R_prime),-1),1) );
        // if a1_R and a1_R_prime are opposite, take the negative of thetaR
        if (v3Dot(a1_R, a1_R_prime) < 0) {
            thetaR = -thetaR;
        }
        // always make the absolute difference |thetaR-thetaC| smaller that 2*pi
        if (thetaR - thetaC > MPI) {
            spinningBodyRefOut.theta = spinningBodyIn.theta + thetaR - thetaC - 2*MPI;
        }
        else if (thetaR - thetaC < - MPI) {
            spinningBodyRefOut.theta = spinningBodyIn.theta + thetaR - thetaC + 2*MPI;
        }
        else {
            spinningBodyRefOut.theta = spinningBodyIn.theta + thetaR - thetaC;
        }
    }

    /*! implement finite differences to compute thetaDotR */
    double dt;
    if (configData->count == 0) {
        spinningBodyRefOut.thetaDot = 0;
    }
    else {
        dt = (double) (callTime - configData->priorT) * NANO2SEC;
        spinningBodyRefOut.thetaDot = (thetaR - configData->priorThetaR) / dt;
    }
    // update stored variables
    configData->priorThetaR = thetaR;
    configData->priorT = callTime;
    configData->count += 1;

    /* write output message */
    SpinningBodyMsg_C_write(&spinningBodyRefOut, &configData->spinningBodyRefOutMsg, moduleID, callTime);
}
