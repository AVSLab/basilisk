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

#include "attGuidance/celestialTwoBodyPoint/celestialTwoBodyPoint.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the nominal delta-V maneuver guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the celestial body guidance
 */
void SelfInit_celestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData,
    uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(AttRefFswMsg),
                                               "AttRefFswMsg",
                                               moduleID);
    return;
    
}

/*! This method performs the second stage of initialization for the celestial body
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the attitude maneuver guidance
 */
void CrossInit_celestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData,
    uint64_t moduleID)
{
    ConfigData->inputCelID = subscribeToMessage(ConfigData->inputCelMessName,
                                                sizeof(EphemerisIntMsg), moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavDataName,
                                                sizeof(NavTransIntMsg), moduleID);
    ConfigData->inputSecID = -1;
    if(strlen(ConfigData->inputSecMessName) > 0)
    {
        ConfigData->inputSecID = subscribeToMessage(ConfigData->inputSecMessName,
                                                    sizeof(EphemerisIntMsg), moduleID);
    }
    return;
    
}
void Reset_celestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
}

/*! This method takes the navigation translational info as well as the spice data of the
 primary celestial body and, if applicable, the second one, and computes the relative state vectors
 necessary to create the restricted 2-body pointing reference frame.
 @return void
 */
void parseInputMessages(celestialTwoBodyPointConfig *ConfigData, uint64_t moduleID)
{
    uint64_t writeTime;
    uint32_t writeSize;
    NavTransIntMsg navData;
    EphemerisIntMsg primPlanet;
    EphemerisIntMsg secPlanet;

    /* zero the local planet ephemeris message */
    memset(&primPlanet, 0x0, sizeof(EphemerisIntMsg));


    double R_P1_hat[3];             /* Unit vector in the direction of r_P1 */
    double R_P2_hat[3];             /* Unit vector in the direction of r_P2 */
    
    double platAngDiff;             /* Angle between r_P1 and r_P2 */
    double dotProduct;              /* Temporary scalar variable */
    
    ReadMessage(ConfigData->inputNavID, &writeTime, &writeSize, sizeof(NavTransIntMsg), &navData, moduleID);
    ReadMessage(ConfigData->inputCelID, &writeTime, &writeSize, sizeof(EphemerisIntMsg), &primPlanet, moduleID);
    
    v3Subtract(primPlanet.r_BdyZero_N, navData.r_BN_N, ConfigData->R_P1);
    v3Subtract(primPlanet.v_BdyZero_N, navData.v_BN_N, ConfigData->v_P1);
    
    v3SetZero(ConfigData->a_P1);
    v3SetZero(ConfigData->a_P2);
    
    if(ConfigData->inputSecID >= 0)
    {
        ReadMessage(ConfigData->inputSecID, &writeTime, &writeSize, sizeof(EphemerisIntMsg), &secPlanet, moduleID);
        
        v3Subtract(secPlanet.r_BdyZero_N, navData.r_BN_N, ConfigData->R_P2);
        v3Subtract(secPlanet.v_BdyZero_N, navData.v_BN_N, ConfigData->v_P2);
        v3Normalize(ConfigData->R_P1, R_P1_hat);
        v3Normalize(ConfigData->R_P2, R_P2_hat);
        dotProduct = v3Dot(R_P2_hat, R_P1_hat);
        if (dotProduct >= 1.0)
        {
            platAngDiff = 0.0;
        } else {
            platAngDiff = acos(dotProduct);
        }
    }
    
    if(ConfigData->inputSecID < 0 || fabs(platAngDiff) < ConfigData->singularityThresh)
    {
        v3Cross(ConfigData->R_P1, ConfigData->v_P1, ConfigData->R_P2);
        v3Cross(ConfigData->R_P1, ConfigData->a_P1, ConfigData->v_P2);
        v3Cross(ConfigData->v_P1, ConfigData->a_P1, ConfigData->a_P2);
    }
}

/*! This method takes the spacecraft and points a specified axis at a named
    celestial body specified in the configuration data.  It generates the 
    commanded attitude and assumes that the control errors are computed 
    downstream.
 @return void
 @param ConfigData The configuration data associated with the celestial body guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_celestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData,
    uint64_t callTime, uint64_t moduleID)
{
    parseInputMessages(ConfigData, moduleID);
    computecelestialTwoBodyPoint(ConfigData, callTime);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttRefFswMsg),
                 (void*) &(ConfigData->attRefOut), moduleID);
}

/*! This method takes the spacecraft and points a specified axis at a named
 celestial body specified in the configuration data.  It generates the
 commanded attitude and assumes that the control errors are computed
 downstream.
 @return void
 @param ConfigData The configuration data associated with the celestial body guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void computecelestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData, uint64_t callTime)
{
    double temp3[3];        /* Temporary vector */
    double temp3_1[3];      /* Temporary vector 1 */
    double temp3_2[3];      /* Temporary vector 2 */
    double temp3_3[3];      /* Temporary vector 3 */
    double temp33[3][3];    /* Temporary 3x3 matrix */
    double temp33_1[3][3];  /* Temporary 3x3 matrix 1 */
    double temp33_2[3][3];  /* Temporary 3x3 matrix 2 */
    
    /* Before-hand computations: R_n, v_n, a_n */
    double R_n[3];          /* Normal vector of the plane defined by R_P1 and R_P2 */
    double v_n[3];          /* First time-derivative of R_n */
    double a_n[3];          /* Second time-derivative of R_n */
    
    v3Cross(ConfigData->R_P1, ConfigData->R_P2, R_n);
    v3Cross(ConfigData->v_P1, ConfigData->R_P2, temp3_1);
    v3Cross(ConfigData->R_P1, ConfigData->v_P2, temp3_2);
    v3Add(temp3_1, temp3_2, v_n);
    v3Cross(ConfigData->a_P1, ConfigData->R_P2, temp3_1);
    v3Cross(ConfigData->R_P1, ConfigData->a_P2, temp3_2);
    v3Add(temp3_1, temp3_2, temp3_3);
    v3Cross(ConfigData->v_P1, ConfigData->v_P2, temp3);
    v3Scale(2.0, temp3, temp3);
    v3Add(temp3, temp3_3, a_n);
    
    /* Reference Frame computation */
    double dcm_RN[3][3];    /* DCM that maps from Reference frame to the inertial */
    double r1_hat[3];       /* 1st row vector of RN */
    double r2_hat[3];       /* 2nd row vector of RN */
    double r3_hat[3];       /* 3rd row vector of RN */
    
    v3Normalize(ConfigData->R_P1, r1_hat);
    v3Normalize(R_n, r3_hat);
    v3Cross(r3_hat, r1_hat, r2_hat);
    v3Copy(r1_hat, dcm_RN[0]);
    v3Copy(r2_hat, dcm_RN[1]);
    v3Copy(r3_hat, dcm_RN[2]);
    C2MRP(dcm_RN, ConfigData->attRefOut.sigma_RN);
    
    /* Reference base-vectors first time-derivative */
    double dr1_hat[3];      /* r1_hat first time-derivative */
    double dr2_hat[3];      /* r2_hat first time-derivative */
    double dr3_hat[3];      /* r3_hat first time-derivative */
    double I_33[3][3];      /* Identity 3x3 matrix */
    double C1[3][3];        /* DCM used in the computation of rates and acceleration */
    double C3[3][3];        /* DCM used in the computation of rates and acceleration */
    
    m33SetIdentity(I_33);
    
    v3OuterProduct(r1_hat, r1_hat, temp33);
    m33Subtract(I_33, temp33, C1);
    m33MultV3(C1, ConfigData->v_P1, temp3);
    v3Scale(1.0 / v3Norm(ConfigData->R_P1), temp3, dr1_hat);
    
    v3OuterProduct(r3_hat, r3_hat, temp33);
    m33Subtract(I_33, temp33, C3);
    m33MultV3(C3, v_n, temp3);
    v3Scale(1.0 / v3Norm(R_n), temp3, dr3_hat);
    
    v3Cross(dr3_hat, r1_hat, temp3_1);
    v3Cross(r3_hat, dr1_hat, temp3_2);
    v3Add(temp3_1, temp3_2, dr2_hat);
    
    /* Angular velocity computation */
    double omega_RN_R[3];   /* Angular rate of the reference frame 
                             wrt the inertial in ref R-frame components */
    
    omega_RN_R[0] = v3Dot(r3_hat, dr2_hat);
    omega_RN_R[1]= v3Dot(r1_hat, dr3_hat);
    omega_RN_R[2] = v3Dot(r2_hat, dr1_hat);
    m33tMultV3(dcm_RN, omega_RN_R, ConfigData->attRefOut.omega_RN_N);
    
    /* Reference base-vectors second time-derivative */
    double ddr1_hat[3];     /* r1_hat second time-derivative */
    double ddr2_hat[3];     /* r2_hat second time-derivative */
    double ddr3_hat[3];     /* r3_hat second time-derivative */
    
    m33MultV3(C1, ConfigData->a_P1, temp3_1);
    v3OuterProduct(dr1_hat, r1_hat, temp33_1);
    m33Scale(2.0, temp33_1, temp33_1);
    v3OuterProduct(r1_hat, dr1_hat, temp33_2);
    m33Add(temp33_1, temp33_2, temp33);
    m33MultV3(temp33, ConfigData->v_P1, temp3_2);
    v3Subtract(temp3_1, temp3_2, temp3);
    v3Scale(1.0 / v3Norm(ConfigData->R_P1), temp3, ddr1_hat);
    
    m33MultV3(C3, a_n, temp3_1);
    v3OuterProduct(dr3_hat, r3_hat, temp33_1);
    m33Scale(2.0, temp33_1, temp33_1);
    v3OuterProduct(r3_hat, dr3_hat, temp33_2);
    m33Add(temp33_1, temp33_2, temp33);
    m33MultV3(temp33, v_n, temp3_2);
    v3Subtract(temp3_1, temp3_2, temp3);
    v3Scale(1.0 / v3Norm(R_n), temp3, ddr3_hat);
    
    v3Cross(ddr3_hat, r1_hat, temp3_1);
    v3Cross(r3_hat, ddr1_hat, temp3_2);
    v3Add(temp3_1, temp3_2, temp3_3);
    v3Cross(dr3_hat, dr1_hat, temp3);
    v3Scale(2.0, temp3, temp3);
    v3Add(temp3, temp3_3, ddr2_hat);
    
    /* Angular acceleration computation */
    double domega_RN_R[3];   /* Angular acceleration of the reference frame
                              wrt the inertial in ref R-frame components */
    
    domega_RN_R[0] = v3Dot(dr3_hat, dr2_hat) + v3Dot(r3_hat, ddr2_hat) - v3Dot(omega_RN_R, dr1_hat);
    domega_RN_R[1] = v3Dot(dr1_hat, dr3_hat) + v3Dot(r1_hat, ddr3_hat) - v3Dot(omega_RN_R, dr2_hat);
    domega_RN_R[2] = v3Dot(dr2_hat, dr1_hat) + v3Dot(r2_hat, ddr1_hat) - v3Dot(omega_RN_R, dr3_hat);
    m33tMultV3(dcm_RN, domega_RN_R, ConfigData->attRefOut.domega_RN_N);
    
    return;
}

