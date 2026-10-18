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
#include <string.h>


#include <math.h>
#include "fswAlgorithms/attGuidance/celestialTwoBodyPoint/celestialTwoBodyPoint.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

/*! This method initializes the configData for the nominal delta-V maneuver guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message

 @param configData The configuration data associated with the celestial body guidance
 @param moduleID The ID associated with the configData
 */
void SelfInit_celestialTwoBodyPoint(celestialTwoBodyPointConfig *configData,
    int64_t moduleID)
{
    (void) moduleID;
    AttRefMsg_C_init(&configData->attRefOutMsg);
    return;

}


void Reset_celestialTwoBodyPoint(celestialTwoBodyPointConfig *configData, uint64_t callTime, int64_t moduleID)
{
    (void) callTime;
    (void) moduleID;
    configData->secCelBodyIsLinked = EphemerisMsg_C_isLinked(&configData->secCelBodyInMsg);

    // check if required input messages have been included
    if (!NavTransMsg_C_isLinked(&configData->transNavInMsg)) {
        _bskError(configData->bskLogger, "Error: celestialTwoBodyPoint.transNavInMsg wasn't connected.");
    }
    if (!EphemerisMsg_C_isLinked(&configData->celBodyInMsg)) {
        _bskError(configData->bskLogger, "Error: celestialTwoBodyPoint.celBodyInMsg wasn't connected.");
    }

    return;
}

/*! This method takes the spacecraft and points a specified axis at a named
 celestial body specified in the configuration data.  It generates the
 commanded attitude and assumes that the control errors are computed
 downstream.

 @param configData The configuration data associated with the celestial body guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Update_celestialTwoBodyPoint(celestialTwoBodyPointConfig *configData,
                                  uint64_t callTime, int64_t moduleID)
{
    /*! - Parse the input messages */
    parseInputMessages(configData, moduleID);
    /*! - Compute the pointing requirements */
    computeCelestialTwoBodyPoint(configData, callTime);
    /*! - Write the output message */
    AttRefMsg_C_write(&configData->attRefOut, &configData->attRefOutMsg, moduleID, callTime);
}

/*! This method takes the navigation translational info as well as the spice data of the
 primary celestial body and, if applicable, the second one, and computes the relative state vectors
 necessary to create the restricted 2-body pointing reference frame.

 @param configData The configuration data associated with the celestial body guidance
 @param moduleID The ID associated with the configData
 */
void parseInputMessages(celestialTwoBodyPointConfig *configData, int64_t moduleID)
{
    (void) moduleID;
    NavTransMsgPayload navData;
    EphemerisMsgPayload primPlanet;
    EphemerisMsgPayload secPlanet;

    double R_P1B_hat_N[3]; /* Unit direction from spacecraft to primary body, in N-frame components */
    double R_P2B_hat_N[3]; /* Unit direction from spacecraft to secondary body, in N-frame components */

    double platAngDiff;             /* Angle between r_P1 and r_P2 */
    double dotProduct;              /* Temporary scalar variable */

    // read input messages
    navData = NavTransMsg_C_read(&configData->transNavInMsg);
    primPlanet = EphemerisMsg_C_read(&configData->celBodyInMsg);


    v3Subtract(primPlanet.r_BdyZero_N, navData.r_BN_N, configData->R_P1B_N);
    v3Subtract(primPlanet.v_BdyZero_N, navData.v_BN_N, configData->v_P1B_N);

    v3SetZero(configData->a_P1B_N); /* accelerations of s/c relative to planets set to zero*/
    v3SetZero(configData->a_P2B_N);

    if(configData->secCelBodyIsLinked)
    {

        secPlanet = EphemerisMsg_C_read(&configData->secCelBodyInMsg);
        /*! - Compute R_P2 and v_P2 */
        v3Subtract(secPlanet.r_BdyZero_N, navData.r_BN_N, configData->R_P2B_N);
        v3Subtract(secPlanet.v_BdyZero_N, navData.v_BN_N, configData->v_P2B_N);
        v3Normalize(configData->R_P1B_N, R_P1B_hat_N);
        v3Normalize(configData->R_P2B_N, R_P2B_hat_N);
        dotProduct = v3Dot(R_P2B_hat_N, R_P1B_hat_N);
        platAngDiff = safeAcos(dotProduct);
    }

    /*! - Cross the P1 states to get R_P2, v_p2 and a_P2 */
    if(!configData->secCelBodyIsLinked ||
       fabs(platAngDiff) < configData->singularityThresh ||
       fabs(platAngDiff) > M_PI - configData->singularityThresh)
    {
        v3Cross(configData->R_P1B_N, configData->v_P1B_N, configData->R_P2B_N);
        v3Cross(configData->R_P1B_N, configData->a_P1B_N, configData->v_P2B_N);
        v3Cross(configData->v_P1B_N, configData->a_P1B_N, configData->a_P2B_N);
    }
}



/*! This method takes the spacecraft and points a specified axis at a named
 celestial body specified in the configuration data.  It generates the
 commanded attitude and assumes that the control errors are computed
 downstream.

 @param configData The configuration data associated with the celestial body guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void computeCelestialTwoBodyPoint(celestialTwoBodyPointConfig *configData, uint64_t callTime)
{
    (void)callTime;
    double temp3_N[3];     /* Temporary vector in N-frame components */
    double temp3_1_N[3];   /* Temporary vector 1 in N-frame components */
    double temp3_2_N[3];   /* Temporary vector 2 in N-frame components */
    double temp3_3_N[3];   /* Temporary vector 3 in N-frame components */
    double temp33[3][3];   /* Temporary matrix acting on N-frame components */
    double temp33_1[3][3]; /* Temporary matrix 1 acting on N-frame components */
    double temp33_2[3][3]; /* Temporary matrix 2 acting on N-frame components */

    double R_N[3]; /* Plane normal from R_P1B_N x R_P2B_N, in N-frame components */
    double v_N[3]; /* First inertial time derivative of R_N, in N-frame components */
    double a_N[3]; /* Second inertial time derivative of R_N, in N-frame components */

    double dcm_RN[3][3]; /* DCM mapping inertial N-frame components to reference R-frame components */
    double r1_hat_N[3];  /* First R-frame basis vector in N-frame components; first row of dcm_RN */
    double r2_hat_N[3];  /* Second R-frame basis vector in N-frame components; second row of dcm_RN */
    double r3_hat_N[3];  /* Third R-frame basis vector in N-frame components; third row of dcm_RN */

    double dr1_hat_N[3]; /* First inertial time derivative of r1_hat_N, in N-frame components */
    double dr2_hat_N[3]; /* First inertial time derivative of r2_hat_N, in N-frame components */
    double dr3_hat_N[3]; /* First inertial time derivative of r3_hat_N, in N-frame components */
    double I_33[3][3];   /* Identity 3x3 matrix */
    double C1[3][3];     /* Projection orthogonal to r1_hat_N, acting on N-frame components */
    double C3[3][3];     /* Projection orthogonal to r3_hat_N, acting on N-frame components */

    double omega_RN_R[3];  /* Angular rate of R relative to N, in R-frame components */
    double domega_RN_R[3]; /* Inertial angular acceleration of R relative to N, in R-frame components */

    double ddr1_hat_N[3]; /* Second inertial time derivative of r1_hat_N, in N-frame components */
    double ddr2_hat_N[3]; /* Second inertial time derivative of r2_hat_N, in N-frame components */
    double ddr3_hat_N[3]; /* Second inertial time derivative of r3_hat_N, in N-frame components */

    configData->attRefOut = AttRefMsg_C_zeroMsgPayload();

    /* - Initial computations: R_n, v_n, a_n */
    v3Cross(configData->R_P1B_N, configData->R_P2B_N, R_N);
    v3Cross(configData->v_P1B_N, configData->R_P2B_N, temp3_1_N);
    v3Cross(configData->R_P1B_N, configData->v_P2B_N, temp3_2_N);
    v3Add(temp3_1_N, temp3_2_N, v_N); /*Eq 4 */
    v3Cross(configData->a_P1B_N, configData->R_P2B_N, temp3_1_N);
    v3Cross(configData->R_P1B_N, configData->a_P2B_N, temp3_2_N);
    v3Add(temp3_1_N, temp3_2_N, temp3_3_N);
    v3Cross(configData->v_P1B_N, configData->v_P2B_N, temp3_N);
    v3Scale(2.0, temp3_N, temp3_N);
    v3Add(temp3_N, temp3_3_N, a_N); /*Eq 5*/

    /* - Reference Frame computation */
    v3Normalize(configData->R_P1B_N, r1_hat_N); /* Eq 9a*/
    v3Normalize(R_N, r3_hat_N);                 /* Eq 9c */
    v3Cross(r3_hat_N, r1_hat_N, r2_hat_N);      /* Eq 9b */
    v3Normalize(r2_hat_N, r2_hat_N);
    v3Copy(r1_hat_N, dcm_RN[0]);
    v3Copy(r2_hat_N, dcm_RN[1]);
    v3Copy(r3_hat_N, dcm_RN[2]);
    C2MRP(dcm_RN, configData->attRefOut.sigma_RN);

    /* - Reference base-vectors first time-derivative */
    m33SetIdentity(I_33);

    v3OuterProduct(r1_hat_N, r1_hat_N, temp33);
    m33Subtract(I_33, temp33, C1);
    m33MultV3(C1, configData->v_P1B_N, temp3_N);
    v3Scale(1.0 / v3Norm(configData->R_P1B_N), temp3_N, dr1_hat_N); /* Eq 11a*/

    v3OuterProduct(r3_hat_N, r3_hat_N, temp33);
    m33Subtract(I_33, temp33, C3);
    m33MultV3(C3, v_N, temp3_N);
    v3Scale(1.0 / v3Norm(R_N), temp3_N, dr3_hat_N); /* Eq 11b*/

    v3Cross(dr3_hat_N, r1_hat_N, temp3_1_N);
    v3Cross(r3_hat_N, dr1_hat_N, temp3_2_N);
    v3Add(temp3_1_N, temp3_2_N, dr2_hat_N); /* Eq 11c*/

    /* - Angular velocity computation */
    omega_RN_R[0] = v3Dot(r3_hat_N, dr2_hat_N);
    omega_RN_R[1] = v3Dot(r1_hat_N, dr3_hat_N);
    omega_RN_R[2] = v3Dot(r2_hat_N, dr1_hat_N);
    m33tMultV3(dcm_RN, omega_RN_R, configData->attRefOut.omega_RN_N);

    /* - Reference base-vectors second time-derivative */
    m33MultV3(C1, configData->a_P1B_N, temp3_1_N);
    v3OuterProduct(dr1_hat_N, r1_hat_N, temp33_1);
    m33Scale(2.0, temp33_1, temp33_1);
    v3OuterProduct(r1_hat_N, dr1_hat_N, temp33_2);
    m33Add(temp33_1, temp33_2, temp33);
    m33MultV3(temp33, configData->v_P1B_N, temp3_2_N);
    v3Subtract(temp3_1_N, temp3_2_N, temp3_N);
    v3Scale(1.0 / v3Norm(configData->R_P1B_N), temp3_N, ddr1_hat_N); /* Eq 13a*/

    m33MultV3(C3, a_N, temp3_1_N);
    v3OuterProduct(dr3_hat_N, r3_hat_N, temp33_1);
    m33Scale(2.0, temp33_1, temp33_1);
    v3OuterProduct(r3_hat_N, dr3_hat_N, temp33_2);
    m33Add(temp33_1, temp33_2, temp33);
    m33MultV3(temp33, v_N, temp3_2_N);
    v3Subtract(temp3_1_N, temp3_2_N, temp3_N);
    v3Scale(1.0 / v3Norm(R_N), temp3_N, ddr3_hat_N); /* Eq 13b*/

    v3Cross(ddr3_hat_N, r1_hat_N, temp3_1_N);
    v3Cross(r3_hat_N, ddr1_hat_N, temp3_2_N);
    v3Add(temp3_1_N, temp3_2_N, temp3_3_N);
    v3Cross(dr3_hat_N, dr1_hat_N, temp3_N);
    v3Scale(2.0, temp3_N, temp3_N);
    v3Add(temp3_N, temp3_3_N, ddr2_hat_N); /* Eq 13c*/

    /* - Angular acceleration computation. For each basis vector, dr_i/dt = omega_RN x r_i,
       so omega_RN dot dr_i/dt is zero when both vectors are expressed in the same frame. */
    domega_RN_R[0] = v3Dot(dr3_hat_N, dr2_hat_N) + v3Dot(r3_hat_N, ddr2_hat_N);
    domega_RN_R[1] = v3Dot(dr1_hat_N, dr3_hat_N) + v3Dot(r1_hat_N, ddr3_hat_N);
    domega_RN_R[2] = v3Dot(dr2_hat_N, dr1_hat_N) + v3Dot(r2_hat_N, ddr1_hat_N);
    m33tMultV3(dcm_RN, domega_RN_R, configData->attRefOut.domega_RN_N);

    return;
}
