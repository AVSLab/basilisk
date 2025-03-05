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
#include "architecture/utilities/macroDefinitions.h"
#include <math.h>

/*!
    This method initializes the output messages for this module.

 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_locationPointing(locationPointingConfig  *configData, int64_t moduleID)
{
    AttGuidMsg_C_init(&configData->attGuidOutMsg);
    AttRefMsg_C_init(&configData->attRefOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.

 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_locationPointing(locationPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{

    // check if the required message has not been connected
    if (!NavAttMsg_C_isLinked(&configData->scAttInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: locationPointing.scAttInMsg was not connected.");
    }
    if (!NavTransMsg_C_isLinked(&configData->scTransInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: locationPointing.scTransInMsg was not connected.");
    }
    int numInMsgs = GroundStateMsg_C_isLinked(&configData->locationInMsg)
                    + StripStateMsg_C_isLinked(&configData->locationstripInMsg)
                    + EphemerisMsg_C_isLinked(&configData->celBodyInMsg)
                    + NavTransMsg_C_isLinked(&configData->scTargetInMsg);

    if (numInMsgs == 0) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: In the locationPointing module no target messages were not connected.");
    }
    else if (numInMsgs > 1) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: In the locationPointing module multiple target messages were connected. Defaulting to either ground location, planet location or spacecraft location, in that order.");
    }

    configData->init = 1;

    v3SetZero(configData->sigma_BR_old);
    configData->time_old = callTime;

    /* compute an Eigen axis orthogonal to sHatBdyCmd */
    if (v3Norm(configData->pHat_B)  < 0.1) {
      char info[MAX_LOGGING_LENGTH];
      sprintf(info, "locationPoint: vector pHat_B is not setup as a unit vector [%f, %f %f]",
                configData->pHat_B[0], configData->pHat_B[1], configData->pHat_B[2]);
      _bskLog(configData->bskLogger, BSK_ERROR, info);
    } else {
        double v1[3];
        v3Set(1., 0., 0., v1);
        v3Normalize(configData->pHat_B, configData->pHat_B);    /* ensure that this vector is a unit vector */
        v3Cross(configData->pHat_B, v1, configData->eHat180_B);
        if (v3Norm(configData->eHat180_B) < 0.1) {
            v3Set(0., 1., 0., v1);
            v3Cross(configData->pHat_B, v1, configData->eHat180_B);
        }
        v3Normalize(configData->eHat180_B, configData->eHat180_B);
    }
}


/*! This method takes the estimated body states and position relative to the ground to compute the current attitude/attitude rate errors and pass them to control.

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_locationPointing(locationPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /* Local copies*/
    NavAttMsgPayload scAttInMsgBuffer;  //!< local copy of input message buffer
    NavTransMsgPayload scTransInMsgBuffer;  //!< local copy of input message buffer
    GroundStateMsgPayload locationInMsgBuffer;  //!< local copy of location input message buffer
    StripStateMsgPayload locationstripInMsgBuffer;  //!< local copy of strip state input message buffer
    EphemerisMsgPayload celBodyInMsgBuffer; //!< local copy of celestial body input message buffer
    NavTransMsgPayload scTargetInMsgBuffer;  //!< local copy of input message buffer of target spacecraft
    AttGuidMsgPayload attGuidOutMsgBuffer;  //!< local copy of guidance output message buffer
    AttRefMsgPayload attRefOutMsgBuffer;  //!< local copy of reference output message buffer

    double r_LS_N[3];                   /*!< Position vector of location w.r.t spacecraft CoM in inertial frame */
    double r_LS_B[3];                   /*!< Position vector of location w.r.t spacecraft CoM in body frame */
    double rHat_LS_B[3];                /*!< unit vector of location w.r.t spacecraft CoM in body frame */
    double eHat_B[3];                   /*!< --- Eigen Axis */
    double dcmBN[3][3];                 /*!< inertial spacecraft orientation DCM */
    double dcmBN_update[3][3];          /*!< inertial reference spacecraft orientation DCM after pointing at the target  */
    double phi;                         /*!< principal angle between pHat and heading to location */
    double sigmaDot_BR[3];              /*!< time derivative of sigma_BR*/
    double sigma_BR[3];                 /*!< MRP of B relative to R */
    double sigma_RB[3];                 /*!< MRP of R relative to B */
    double sigma_RN[3];                 /*!< MRP of R relative to N */
    double omega_RN_B[3];               /*!< angular velocity of the R frame w.r.t the B frame in B frame components */
    double difference[3];
    double time_diff;                   /*!< module update time */
    double Binv[3][3];                  /*!< BinvMRP for dsigma_RB_R calculations*/
    double dum1;
    double r_TN_N[3];                   /*!< [m] inertial target location */
    double v_TN_N[3];                   /*!< [m] inertial target velocity */
    double v_TN_B[3];                   /*!< [m/s] v_TN_N in the reference body frame to point at the target */
    double boreRate_B[3];               /*!< [rad/s] rotation rate about target direction */
    bool imagingstrip;                  /*!< bool indicating if we are imaging a strip or not*/
    double dotProd1;                    /*!< dot product between v_TN_B and pHat_B */
    double v_TN_B_proj[3];              /*!< Projection of v_TN_B in the plane perpendicular to pHat_B*/
    double v_perp[3];                   /*!< Perpendicular vector to v_TN_B_proj in the plane perpendicular to pHat_B*/
    double dotProd2;                    /*!< dot product between cHat_B and v_perp  */
    double angle;                       /*!< [rad] angle between cHat_B and v_perp  */
    double sigma_R2R[3];                /*!< MRP of R2 (reference after pointing at the target and being perpendicular to the strip) relative to R (reference after only pointing at the target) */
    double sigma_R2N[3];                /*!< MRP of R2 relative to N */
    double sigma_N2R[3];
    double sigma_BR2[3];                /*!< MRP of B relative to R2 */
    double sigma_R2B[3];                /*!< MRP of R2 relative to B */
    double sigma_NB[3];
    double neg_sigma_R2N[3];
    // zero output buffer
    attGuidOutMsgBuffer = AttGuidMsg_C_zeroMsgPayload();
    attRefOutMsgBuffer = AttRefMsg_C_zeroMsgPayload();

    // read the input messages
    scAttInMsgBuffer = NavAttMsg_C_read(&configData->scAttInMsg);
    scTransInMsgBuffer = NavTransMsg_C_read(&configData->scTransInMsg);
    if (GroundStateMsg_C_isLinked(&configData->locationInMsg)) {
        locationInMsgBuffer = GroundStateMsg_C_read(&configData->locationInMsg);
        v3Copy(locationInMsgBuffer.r_LN_N, r_TN_N);
        imagingstrip=false;
    }
    else if (StripStateMsg_C_isLinked(&configData->locationstripInMsg)) {
        locationstripInMsgBuffer = StripStateMsg_C_read(&configData->locationstripInMsg);
        v3Copy(locationstripInMsgBuffer.r_LN_N, r_TN_N);
        v3Copy(locationstripInMsgBuffer.v_LP_N, v_TN_N);
        imagingstrip=true;
    }
    else if (EphemerisMsg_C_isLinked(&configData->celBodyInMsg)) {
        celBodyInMsgBuffer = EphemerisMsg_C_read(&configData->celBodyInMsg);
        v3Copy(celBodyInMsgBuffer.r_BdyZero_N, r_TN_N);
        imagingstrip=false;
    } else {
        scTargetInMsgBuffer = NavTransMsg_C_read(&configData->scTargetInMsg);
        v3Copy(scTargetInMsgBuffer.r_BN_N, r_TN_N);
        imagingstrip=false;
    }

    /* calculate r_LS_N */
    v3Subtract(r_TN_N, scTransInMsgBuffer.r_BN_N, r_LS_N);

    /* principle rotation angle to point pHat at location */
    MRP2C(scAttInMsgBuffer.sigma_BN, dcmBN);
    m33MultV3(dcmBN, r_LS_N, r_LS_B);
    v3Normalize(r_LS_B, rHat_LS_B);
    dum1 = v3Dot(configData->pHat_B, rHat_LS_B);
    if (fabs(dum1) > 1.0) {
        dum1 = dum1 / fabs(dum1);
    }
    phi = safeAcos(dum1);

    /* calculate sigma_BR */
    if (phi < configData->smallAngle) {
        /* body axis and desired inertial pointing direction are essentially aligned.  Set attitude error to zero. */
         v3SetZero(sigma_BR);
    } else {
        if (M_PI - phi < configData->smallAngle) {
            /* the commanded body vector nearly is opposite the desired inertial heading */
            v3Copy(configData->eHat180_B, eHat_B);
        } else {
            /* normal case where body and inertial heading vectors are not aligned */
            v3Cross(configData->pHat_B, rHat_LS_B, eHat_B);
        }
        v3Normalize(eHat_B, eHat_B);
        v3Scale(-tan(phi / 4.), eHat_B, sigma_BR);
    }
   
    /* Compute the final reference attitude sigma_RN to only point at the target location */
    v3Scale(-1.0, sigma_BR, sigma_RB);
    addMRP(scAttInMsgBuffer.sigma_BN, sigma_RB, sigma_RN);

    /* In case we are not imaging a strip, we have the final reference attitude sigma_RN and the final attitude tracking error sigma_BR */
    if (!imagingstrip || phi < configData->smallAngle){
        v3Copy(sigma_BR, attGuidOutMsgBuffer.sigma_BR);
        v3Copy(sigma_RN, attRefOutMsgBuffer.sigma_RN);
    }
    /* In case we are imaging a strip, we need to perform an extra rotation to image perpendicularly to the strip */
    else{
        //Step 1 : Express v_TN_N in the reference body frame R to point at the target
        MRP2C(sigma_RN, dcmBN_update);
        v3Normalize(v_TN_N, v_TN_N);
        m33MultV3(dcmBN_update, v_TN_N, v_TN_B);
        //Step 2 : Project v_TN_B onto the plane perpendicular to pHat_B
        dotProd1 = v3Dot(v_TN_B, configData->pHat_B);
        for (int i = 0; i < 3; i++) {
                  v_TN_B_proj[i] = v_TN_B[i] - dotProd1 * configData->pHat_B[i];}
        //Step 3 : Compute the perpendicular vector v_perp = pHat_B x v_TN_B_proj
        v3Cross(configData->pHat_B, v_TN_B_proj, v_perp);
        v3Normalize(v_perp, v_perp);
        //Step 4 : Compute the angle between cHat_B and v_perp
        dotProd2 = v3Dot(configData->cHat_B, v_perp);
        if (dotProd2 > 1.0) dotProd2 = 1.0;
        if (dotProd2 < -1.0) dotProd2 = -1.0;
        angle = safeAcos(dotProd2);
        // Step 5: Determine the sign of the rotation using the mixed product
        double crossProd[3];
        v3Cross(configData->cHat_B, v_perp, crossProd);

        if (crossProd[2]< 0) {
            angle = -angle;  // Make the angle negative for the correct rotation direction
        }
        //Step 6 : Compute the MRP for the rotation around pHat_B by the calculated angle
        v3Scale(tan(angle / 4.), configData->pHat_B, sigma_R2R);
        
        //Step 7 : Handling differently small angles
        if (fabs(angle) < configData->smallAngle) {
            /* Set attitude error to zero. */
            v3SetZero(sigma_R2R);
        }
        //Step 8 : Compute the final reference attitude sigma_R2N to point at the target location and image perpendicularly to the strip */
        addMRP(sigma_RN, sigma_R2R, sigma_R2N);
        v3Copy(sigma_R2N, attRefOutMsgBuffer.sigma_RN);

        //Step 9 : Compute the final attitude tracking error sigma_BR2
        subMRP(scAttInMsgBuffer.sigma_BN,sigma_R2N,sigma_BR2);
//        v3Scale(-1.0, scAttInMsgBuffer.sigma_BN,sigma_NB);
//        addMRP(sigma_NB,sigma_R2N, sigma_R2B);
//        v3Scale(-1.0,sigma_R2B,sigma_BR2);
        
        v3Copy(sigma_BR2, attGuidOutMsgBuffer.sigma_BR);

        //Step 10 : sigma_BR takes the value of sigma_BR2 to make it compatible with the computation of d(sigma_BR)/dt
        v3Copy(sigma_BR2, sigma_BR);
    }

    /* use sigma_BR to compute d(sigma_BR)/dt if at least two data points */
    if (configData->init < 1) {
        // module update time
        time_diff = (callTime - configData->time_old)*NANO2SEC;

        // calculate d(sigma_BR)/dt
        v3Subtract(sigma_BR, configData->sigma_BR_old, difference);
        v3Scale(1.0/(time_diff), difference, sigmaDot_BR);

        // calculate BinvMRP
        BinvMRP(sigma_BR, Binv);

        // compute omega_BR_B
        v3Scale(4.0, sigmaDot_BR, sigmaDot_BR);
        m33MultV3(Binv, sigmaDot_BR, attGuidOutMsgBuffer.omega_BR_B);

    } else {
        configData->init -= 1;
    }

    if (configData->useBoresightRateDamping) {
        v3Scale(v3Dot(scAttInMsgBuffer.omega_BN_B, rHat_LS_B), rHat_LS_B, boreRate_B);
        v3Add(attGuidOutMsgBuffer.omega_BR_B, boreRate_B, attGuidOutMsgBuffer.omega_BR_B);
    }

    // compute omega_RN_B
    v3Subtract(scAttInMsgBuffer.omega_BN_B, attGuidOutMsgBuffer.omega_BR_B, omega_RN_B);

    // convert to omega_RN_N
    m33tMultV3(dcmBN, omega_RN_B, attRefOutMsgBuffer.omega_RN_N);

    // copy current attitude states into prior state buffers
    v3Copy(sigma_BR, configData->sigma_BR_old);

    // update former module call time
    configData->time_old = callTime;

    // write to the output messages
    AttGuidMsg_C_write(&attGuidOutMsgBuffer, &configData->attGuidOutMsg, moduleID, callTime);
    AttRefMsg_C_write(&attRefOutMsgBuffer, &configData->attRefOutMsg, moduleID, callTime);
}
