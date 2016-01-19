/*
    Inertial 3D Spin Module
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */


#include "attGuidance/hillPoint/hillPoint.h"
#include <string.h>
#include "ADCSUtilities/ADCSDefinitions.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"

/* Required module input messages */
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"



void SelfInit_hillPoint(hillPointConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(attGuidOut),
                                               "attGuidOut",
                                               moduleID);
    
    /* these two relative orientations labels are the same */
    ConfigData->sigma_BcB = ConfigData->sigma_R0R;
    
    /* Obit frame states. #NOTE: should they go here or in Python? */
    ConfigData->OrbitFrameStates.i_r = 0; // x-axis
    ConfigData->OrbitFrameStates.i_theta = 1; // y-axis
    ConfigData->OrbitFrameStates.i_h = 2; // z-axis
    
    ConfigData->OrbitFrameStates.i_hSign = 1;
    ConfigData->OrbitFrameStates.i_rSign = 1;
    ConfigData->OrbitFrameStates.i_thetaSign = ConfigData->OrbitFrameStates.i_rSign * ConfigData->OrbitFrameStates.i_hSign;
    
}

void CrossInit_hillPoint(hillPointConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavName,
                                                sizeof(NavStateOut),
                                                moduleID);
}

void Reset_hillPoint(hillPointConfig *ConfigData)
{
    double sigma_RR0[3];

    /* compute the initial reference frame orientation that takes the corrected body frame into account */
    v3Scale(-1.0, ConfigData->sigma_R0R, sigma_RR0);
    addMRP(ConfigData->sigma_R0N, sigma_RR0, ConfigData->sigma_RN);
    
    /* reset the prior time flag state.  If set to zero, the control time step is not evaluated on thefirst function call */
    ConfigData->priorTime = 0;

}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_hillPoint(hillPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t            clockTime;
    uint32_t            readSize;
    NavStateOut         nav;                /*!< navigation message */
    double              dt;                 /*!< [s] module update period */


    /*! Begin method steps*/
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputNavID, &clockTime, &readSize,
                sizeof(NavStateOut), (void*) &(nav));



    /* compute control update time */
    if (ConfigData->priorTime != 0) {       /* don't compute dt if this is the first call after a reset */
        dt = (callTime - ConfigData->priorTime)*NANO2SEC;
        if (dt > 10.0) dt = 10.0;           /* cap the maximum control time step possible */
        if (dt < 0.0) dt = 0.0;             /* ensure no negative numbers are used */
    } else {
        dt = 0.;                            /* set dt to zero to not use integration on first function call */
    }
    ConfigData->priorTime = callTime;


    /*
     compute and store output message 
     */
    computeHillPointAttitudeError(nav.sigma_BN,
                                     nav.omega_BN_B,
                                     nav.r_BN_N,
                                     nav.v_BN_N,
                                     ConfigData,
                                     ConfigData->attGuidOut.sigma_BR,
                                     ConfigData->attGuidOut.omega_BR_B,
                                     ConfigData->attGuidOut.omega_RN_B,
                                     ConfigData->attGuidOut.domega_RN_B);



    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attGuidOut),   /* update module name */
                 (void*) &(ConfigData->attGuidOut), moduleID);

    return;
}


/*
 * Function: computeInertialSpinAttitudeError
 *  Purpose: compute the attitude and rate errors. This function is designed to work both in:
 *      FSW to compute estimated pointing errors
 *      Simulation code to compute true pointing errors
 * Inputs:
 *   sigma_BN = MRP attitude of body relative to inertial
 *   omega_BN_B = body rate vector
 *   r_BN_B = inertial position vector of the spacecraft in body frame components
 *   v_BN_B = inertial velocity vector of the spacecraft in body frame components
 *   ConfigData = module configuration data
 * Outputs:
 *   sigma_BR = MRP attitude error of body relative to reference
 *   omega_BR_B = angular velocity vector error of body relative to reference
 *   omega_RN_B = reference angluar velocity vector in body frame components
 *   domega_RN_B = reference angular acceleration vector in body frame componets
 */
void computeHillPointAttitudeError(double sigma_BN[3],
                                      double omega_BN_B[3],
                                      double r_BN_N[3],
                                      double v_BN_N[3],
                                      hillPointConfig *ConfigData,
                                      double sigma_BR[3],
                                      double omega_BR_B[3],
                                      double omega_RN_B[3],
                                      double domega_RN_B[3])
{
    
    
    double  BN[3][3];               /*!< DCM from inertial to body frame */
    double  RN[3][3];               /*!< DCM from inertial to reference frame */
    double  BR[3][3];               /*!< DCM from reference to body frame */
    double  temp33[3][3];
    
    double rm;                      /*!< orbit radius */
    double h[3];                    /*!< orbit angular momentum vector */
    double hm;                      /*!< module of the orbit angular momentum vector */
    double dfdt;                    /*!< rotational rate of the orbit frame */
    double ddfdt2;                  /*!< rotational acceleration of the frame */

    
    double  omega_RN_R[3];          /*!< reference angular velocity vector in Reference frame R components */
    double  domega_RN_R[3];          /*!< reference angular acceleration vector in Reference frame R components */


    /* Compute reference attitude states assuming the reference orientation is Nadir pointing*/

    /* Compute BN */
    MRP2C(sigma_BN, BN);
    
    /* Compute RN */
        /* i_r */
    v3Normalize(r_BN_N, RN[ConfigData->OrbitFrameStates.i_r]);
    if(ConfigData->OrbitFrameStates.i_rSign < 0) {
        v3Scale(-1.0, RN[ConfigData->OrbitFrameStates.i_r], RN[ConfigData->OrbitFrameStates.i_r]);
    }
        /* i_h */
    v3Cross(r_BN_N, v_BN_N, h);
    v3Normalize(h, RN[ConfigData->OrbitFrameStates.i_h]);
    if(ConfigData->OrbitFrameStates.i_hSign < 0) {
        v3Scale(-1.0, RN[ConfigData->OrbitFrameStates.i_h], RN[ConfigData->OrbitFrameStates.i_h]);
    }
        /* i_theta = i_h x i_r */
    v3Cross(RN[ConfigData->OrbitFrameStates.i_h], RN[ConfigData->OrbitFrameStates.i_r], RN[ConfigData->OrbitFrameStates.i_theta]);
    
    /* Compute BR */
    m33Transpose(RN, temp33);
    m33MultM33(BN, temp33, BR);
    
    /* compute relative orientation error */
    C2MRP(BR, sigma_BR); // #NEW: subMRP(sigma_BN, ConfigData->sigma_RN, sigma_BR);
    
    /* compute R-frame inertial rate and acceleration */
    rm = v3Norm(r_BN_N);
    hm = v3Norm(h);
    if(rm > 1.) {
        dfdt = hm / (rm * rm);  // n = h / r^2
        ddfdt2 = - 2.0 * v3Dot(v_BN_N, RN[ConfigData->OrbitFrameStates.i_r - 1]) / rm * dfdt;
    } else {
        /* an error has occured, radius shouldn't be less than 1km */
        dfdt   = 0.;
        ddfdt2 = 0.;
    }
    v3SetZero(omega_RN_R);
    v3SetZero(domega_RN_R);
    omega_RN_R[ConfigData->OrbitFrameStates.i_h]  = dfdt * ConfigData->OrbitFrameStates.i_hSign;
    domega_RN_R[ConfigData->OrbitFrameStates.i_h] = ddfdt2 * ConfigData->OrbitFrameStates.i_hSign;
    
    /* compute angular velocity tracking errors */
    m33MultV3(BR, omega_RN_R, omega_RN_B); // #NEW: m33MultV3(BN, ConfigData->omega_RN_N, omega_RN_B);
    m33MultV3(BR, domega_RN_R, domega_RN_B); // #NEW: m33MultV3(BN, ConfigData->domega_RN_N, domega_RN_B);
    v3Subtract(omega_BN_B, omega_RN_B, omega_BR_B);
}
