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

#include "attitudeMOEFeedback.h"

#include <math.h>

#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/rigidBodyKinematics.h"

static void calc_LyapunovFeedback(attitudeMOEFeedbackConfig *configData, NavTransMsgPayload chiefTransMsg, 
    NavTransMsgPayload deputyTransMsg, NavAttMsgPayload deputyAttNavMsg, 
    CmdForceInertialMsgPayload *forceMsg);
static void calc_B_eq(double mu, equinoctialElements oe_eq, double B[6][3]);
static double adjust_range(double lower, double upper, double angle);
static double wrapPi(double a);
static int nearAngle(double angle, double target, double halfWidth);

// static void calc_hillpointingReference(attitudeMOEFeedbackConfig *configData,
//                                   double r_BN_N[3],
//                                   double v_BN_N[3],
//                                   double celBdyPositonVector[3],
//                                   double celBdyVelocityVector[3],
//                                   AttRefMsgPayload *attRefOut);


void SelfInit_attitudeMOEFeedback(attitudeMOEFeedbackConfig *configData, int64_t moduleID) {
    CmdForceInertialMsg_C_init(&configData->forceOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.  The local copy of the
 message output buffer should be cleared.

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The Basilisk module identifier
 */
void Reset_attitudeMOEFeedback(attitudeMOEFeedbackConfig *configData, uint64_t callTime, int64_t moduleID) {
    // check if the required input messages are included
    if (!NavTransMsg_C_isLinked(&configData->chiefTransInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: meanOEFeedback.chiefTransInMsg wasn't connected.");
    }
    if (!NavTransMsg_C_isLinked(&configData->deputyTransInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: meanOEFeedback.deputyTransInMsg wasn't connected.");
    }

    if (configData->mu <= 0.0) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error in meanOEFeedback: mu must be set to a positive value.");
    }
    if (configData->req <= 0.0) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error in meanOEFeedback: req must be set to a positive value.");
    }
    if (configData->J2 <= 0.0) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error in meanOEFeedback: J2 must be set to a positive value.");
    }

    return;
}

/*! Add a description of what this main Update() routine does for this module

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The Basilisk module identifier
 */
void Update_attitudeMOEFeedback(attitudeMOEFeedbackConfig *configData, uint64_t callTime, int64_t moduleID) {
    // in
    NavTransMsgPayload chiefTransMsg;
    NavTransMsgPayload deputyTransMsg;
    NavAttMsgPayload deputyAttNavMsg;

    // out
    CmdForceInertialMsgPayload forceMsg;

    /*! - Read the input messages */
    chiefTransMsg = NavTransMsg_C_read(&configData->chiefTransInMsg);
    deputyTransMsg = NavTransMsg_C_read(&configData->deputyTransInMsg);
    deputyAttNavMsg = NavAttMsg_C_read(&configData->deputyAttNavInMsg);

    /*! - write the module output message */
    forceMsg = CmdForceInertialMsg_C_zeroMsgPayload();
    calc_LyapunovFeedback(configData, chiefTransMsg, deputyTransMsg, deputyAttNavMsg, &forceMsg);

    CmdForceInertialMsg_C_write(&forceMsg, &configData->forceOutMsg, moduleID, callTime);
    return;
}

/*! This function calculates Lyapunov Feedback Control output based on current orbital element difference
 and target orbital element difference. Mean orbital elements are used.

 @param configData The configuration data associated with the module
 @param chiefTransMsg Chief's position and velocity
 @param deputyTransMsg Deputy's position and velocity
 @param deputyAttNavMsg Deputy's attitude navigation message
 @param forceMsg force output (3-axis)
 */
static void calc_LyapunovFeedback(attitudeMOEFeedbackConfig *configData, NavTransMsgPayload chiefTransMsg,NavTransMsgPayload deputyTransMsg, NavAttMsgPayload deputyAttNavMsg, CmdForceInertialMsgPayload *forceMsg) {
    // position & velocity to osculating classic orbital elements
    ClassicElements oe_cl_osc_c, oe_cl_osc_d;
    rv2elem(configData->mu, chiefTransMsg.r_BN_N, chiefTransMsg.v_BN_N, &oe_cl_osc_c);
    rv2elem(configData->mu, deputyTransMsg.r_BN_N, deputyTransMsg.v_BN_N, &oe_cl_osc_d);

    // osculating classic oe to mean classic oe
    ClassicElements oe_cl_mean_c, oe_cl_mean_d;
    clMeanOscMap(configData->req, configData->J2, &oe_cl_osc_c, &oe_cl_mean_c, -1);
    clMeanOscMap(configData->req, configData->J2, &oe_cl_osc_d, &oe_cl_mean_d, -1);

    // calculate necessary Force in LVLH frame
    double oed[6];
    double B[6][3];
    double force_LVLH[3];

    // mean classic oe to mean equinoctial oe
    equinoctialElements oe_eq_mean_c, oe_eq_mean_d;
    clElem2eqElem(&oe_cl_mean_c, &oe_eq_mean_c);
    clElem2eqElem(&oe_cl_mean_d, &oe_eq_mean_d);

    double windowRad = configData->windowDeg * M_PI / 180.0;  // half-width [rad] 

    /* decide if we are inside ANY allowed window */
    int inWindow = 0;
    
   // calculate relative orbital element difference and check if formation guidance is used
    if (configData->varying_target == 1){
        // compute relative orbital element magnitudes
        double delta_i_mag = configData->ring_height / (2.0 * oe_cl_osc_c.a);   // cross-track height
        double delta_e_mag = configData->ring_diameter / (2.0 * oe_cl_osc_c.a); // in-plane ring radius
        
        // δa, δe_x, δe_y, δi_x, δi_y, δλ target difference 
        double delta_i_x = delta_i_mag * cos(configData->phase_angle);
        double delta_i_y = delta_i_mag * sin(configData->phase_angle);
        double delta_e_x = delta_e_mag * cos(configData->phase_angle);
        double delta_e_y = delta_e_mag * sin(configData->phase_angle);
        
        // Compute gamma for J2-invariant design
        double e2 = oe_cl_osc_c.e * oe_cl_osc_c.e;
        double gamma = -1.5 * configData->J2 * (configData->req / oe_cl_osc_c.a) * sqrt(1.0 / pow(1.0 - e2, 4));
        double delta_a = -7.0 * gamma * sin(2.0 * oe_cl_osc_c.i) * delta_i_x;
        
        // calculate equinoctial oed (da,dP1,dP2,dQ1,dQ2,dl)
        oed[0] = (oe_eq_mean_d.a - oe_eq_mean_c.a) / oe_eq_mean_c.a -delta_a;
        oed[1] = oe_eq_mean_d.P1 - oe_eq_mean_c.P1 - delta_e_x;
        oed[2] = oe_eq_mean_d.P2 - oe_eq_mean_c.P2 - delta_e_y;
        oed[3] = oe_eq_mean_d.Q1 - oe_eq_mean_c.Q1 - delta_i_x;
        oed[4] = oe_eq_mean_d.Q2 - oe_eq_mean_c.Q2 - delta_i_y;
        oed[5] = oe_eq_mean_d.l - oe_eq_mean_c.l;  // set along-track ROE to zero to avoid drift
        }
    else {
        oed[0] = (oe_eq_mean_d.a - oe_eq_mean_c.a) / oe_eq_mean_c.a - configData->targetDiffOeMean[0];
        oed[1] = oe_eq_mean_d.P1 - oe_eq_mean_c.P1 - configData->targetDiffOeMean[1];
        oed[2] = oe_eq_mean_d.P2 - oe_eq_mean_c.P2 - configData->targetDiffOeMean[2];
        oed[3] = oe_eq_mean_d.Q1 - oe_eq_mean_c.Q1 - configData->targetDiffOeMean[3];
        oed[4] = oe_eq_mean_d.Q2 - oe_eq_mean_c.Q2 - configData->targetDiffOeMean[4];
        oed[5] = oe_eq_mean_d.l - oe_eq_mean_c.l - configData->targetDiffOeMean[5];
    }
    oed[5] = adjust_range(-M_PI, M_PI, oed[5]);

    // calculate control matrix B
    calc_B_eq(configData->mu, oe_eq_mean_d, B);

    // calculate Lyapunov Feedback Control
    double K_oed[6];
    m66MultV6(RECAST6X6 configData->K, oed, K_oed);
    mtMultV(B, 6, 3, K_oed, force_LVLH);
    v3Scale(-1, force_LVLH, force_LVLH);

    // convert force to Inertial frame
    double dcm_RN[3][3];
    double h[3];
    v3Cross(deputyTransMsg.r_BN_N, deputyTransMsg.v_BN_N, h);
    v3Normalize(deputyTransMsg.r_BN_N, dcm_RN[0]);
    v3Normalize(h, dcm_RN[2]);
    v3Cross(dcm_RN[2], dcm_RN[0], dcm_RN[1]);
    m33tMultV3(dcm_RN, force_LVLH, forceMsg->forceRequestInertial);

    double force_N[3];
    m33tMultV3(dcm_RN, force_LVLH, force_N);
    
    // compute body force and set force limitations
    double      dcm_BN[3][3];               /* DCM from inertial to body frame */
    MRP2C(deputyAttNavMsg.sigma_BN, dcm_BN);
    double force_B[3];
    m33tMultV3(dcm_BN, force_N, force_B);
        
    // check and scale thrust limit 
    double thrust_mag = sqrt(force_B[0]*force_B[0] +  force_B[1]*force_B[1] + force_B[2]*force_B[2]);

    if (thrust_mag > configData->max_thrust) {
        double scale = configData->max_thrust / thrust_mag;
        v3Scale(scale, force_B, force_B);
    }

    /* thrust at specific locations in the orbit */

    /*
    when true anomaly is 0 deg its at perigee and at 180 deg its at apogee, 
    this is efficient for eccentricity changes. Eccentricity can be changed at other locations 
    in the orbit but this affect is coupled with changes in semi-major axis. In-plane thrusting
    is needed either along-track or radial.
    
    for inclination changes, thrusting at the ascending and descending nodes is most efficient. 
    Out-of-plane thrusting is needed (normal to orbit plane).

    now, looking at the equinoctial elements, 
    when true anomaly is 0 or 180 deg, P1 and P2 are affected (equivalent to eccentricity changes)
    when true anomaly is 90 or 270 deg, Q1 and Q2 are affected (equivalent to inclination changes)
    
    */ 


    // TRUE ANOMALY BASED WINDOWS 
    // when true anomaly is 90 deg its at ascending node and at 270 deg its at descending node
    // using true anomaly of the deputy, thrust near equator and asc/desc nodes within a few degrees 
    // replace hard on/off with a smoother transition to avoid bang-bang behavior
    
    

    /* deputy angles */
    double f_d = wrapPi(oe_cl_osc_d.f);          // true anomaly
    double w_d = wrapPi(oe_cl_osc_d.omega);      // argument of perigee
    double u_d = wrapPi(w_d + f_d);              // argument of latitude


    /* 1) apses windows: thrust near perigee (f≈0) and apogee (f≈π) */
    if (configData->enableApsesWindows) {
        if ( nearAngle(f_d, 0.0,  windowRad) ||    // perigee
            nearAngle(f_d, M_PI, windowRad) ) {   // apogee
            inWindow = 1;
        }
    }

    /* 2) node windows: thrust near ascending/descending nodes (u≈0, π) */
    if (configData->enableNodeWindows) {
        if ( nearAngle(u_d, 0.0,  windowRad) ||    // ascending node
            nearAngle(u_d, M_PI, windowRad) ) {   // descending node
            inWindow = 1;
        }
    }

    /* gate thrust */
    if (!inWindow) {
        v3SetZero(force_B);
    }   
    

    // convert body force back to inertial and set output message 
    double      dcm_NB[3][3];               /* DCM from body to inertial frame */
    m33Transpose(dcm_BN, dcm_NB);
    m33tMultV3(dcm_NB, force_B, forceMsg->forceRequestInertial);
    return;
}


/*! This function calculates Control Matrix (often called B matrix) derived from Gauss' Planetary Equation.
 Especially, this function assumes using nonsingular orbital elements, which help to avoid singularity.
 The B matrix description is provided in
 "Naasz, B. J., Karlgaard, C. D., & Hall, C. D. (2002). Application of several control techniques for
 the ionospheric observation nanosatellite formation."
 Be careful, our definition of equinoctial orbital elements are different from the one used in this paper.

 @param mu
 @param oe_eq nonsingular orbital elements
 @param B
 */
static void calc_B_eq(double mu, equinoctialElements oe_eq, double B[6][3]) {
    // define parameters necessary to calculate Bmatrix
    double a = oe_eq.a;
    double P1 = oe_eq.P1;
    double P2 = oe_eq.P2;
    double Q1 = oe_eq.Q1;
    double Q2 = oe_eq.Q2;
    double L = oe_eq.L;
    double b = a * sqrt(1 - P1 * P1 - P2 * P2);
    double n = sqrt(mu / pow(a, 3));
    double h = n * a * b;
    double p_r = 1 + P1 * sin(L) + P2 * cos(L);
    double r_h = h / (mu * p_r);
    double r = r_h * h;
    double p = p_r * r;

    // substitute into Bmatrix
    B[0][0] = 2.0 * pow(a, 2) / h * (P2 * sin(L) - P1 * cos(L)) / a;  // nomalization
    B[0][1] = 2.0 * pow(a, 2) * p_r / h / a;                          // nomalization
    B[0][2] = 0;

    B[1][0] = -p * cos(L) / h;
    B[1][1] = 1.0 / h * (r * P1 + (r + p) * sin(L));
    B[1][2] = r_h * P2 * (Q2 * sin(L) - Q1 * cos(L));

    B[2][0] = p * sin(L) / h;
    B[2][1] = 1.0 / h * (r * P2 + (r + p) * cos(L));
    B[2][2] = r_h * P1 * (Q1 * cos(L) - Q2 * sin(L));

    B[3][0] = 0;
    B[3][1] = 0;
    B[3][2] = 1.0 / 2.0 * r_h * (1 + Q1 * Q1 + Q2 * Q2) * sin(L);

    B[4][0] = 0;
    B[4][1] = 0;
    B[4][2] = 1.0 / 2.0 * r_h * (1 + Q1 * Q1 + Q2 * Q2) * cos(L);

    B[5][0] = -p * a / (h * (a + b)) * ((P1 * sin(L) + P2 * cos(L)) + 2.0 * b / a);
    B[5][1] = r_h * a * (1.0 + p_r) / (a + b) * (P2 * sin(L) - P1 * cos(L));
    B[5][2] = r_h * (Q2 * sin(L) - Q1 * cos(L));
    return;
}

/*! This function is used to adjust a certain value in a certain range between lower threshold and upper threshold.
 This function is particularily used to adjust angles used in orbital motions such as True Anomaly, Mean Anomaly, and so on.
 @return double
 @param lower lower threshold
 @param upper upper threshold
 @param angle an angle which you want to be between lower and upper
*/
static double adjust_range(double lower, double upper, double angle) {
    if (upper < lower) {
        printf("illegal parameters\n");
        return -1;
    }
    double width = upper - lower;
    double adjusted_angle = angle;
    while (adjusted_angle > upper) {
        adjusted_angle = adjusted_angle - width;
    }
    while (adjusted_angle < lower) {
        adjusted_angle = adjusted_angle + width;
    }
    return adjusted_angle;
}

/*! This function is used to wrap an angle between -pi and pi
 @return double
 @param a an angle which you want to be between -pi and pi
*/
static double wrapPi(double a) {
    while (a <= -M_PI) a += 2.0*M_PI;
    while (a >   M_PI) a -= 2.0*M_PI;
    return a;
}

/*! This function tests if a certain angle is within a certain half-width of a target angle, with wrap-around.
 @return int (boolean)
 @param angle an angle you want to test
 @param target target angle
 @param halfWidth half-width of the acceptance window
*/
static int nearAngle(double angle, double target, double halfWidth) {
    return fabs(wrapPi(angle - target)) <= halfWidth;
}
