/*
    Inertial 3D Spin Module
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */

/* modify the path to reflect the new module names */
#include "attGuidance/orbitAxisSpin/orbitAxisSpin.h"
#include <string.h>
#include "math.h"
#include "ADCSUtilities/ADCSDefinitions.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"

/* Required module input messages */
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"
#include "SimCode/environment/spice/spice_planet_state.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"


void SelfInit_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(attRefOut),
                                               "attRefOut",
                                               moduleID);
    /*! - #TEMP: Initialize variable */
    v3SetZero(&ConfigData->sigma_B0N);
    ConfigData->b_spin = 0;
}

void CrossInit_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputCelID = subscribeToMessage(ConfigData->inputCelMessName,
                                                sizeof(SpicePlanetState), moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavDataName,
                                                sizeof(NavStateOut), moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputPointRefName,
                                                sizeof(attRefOut), moduleID);

}

void Reset_orbitAxisSpin(orbitAxisSpinConfig *ConfigData)
{
    ConfigData->priorTime = 0;              /* reset the prior time flag state.  If set
                                             to zero, the control time step is not evaluated on the
                                             first function call */
    ConfigData->phi_spin = 0;
}

void Update_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
     /*! - Compute control update time */
    double dt;                                              /*!< [s] module update period */
    if (ConfigData->priorTime != 0) {                       /* don't compute dt if this is the first call after a reset */
        dt = (callTime - ConfigData->priorTime)*NANO2SEC;
        if (dt > 10.0) dt = 10.0;                           /* cap the maximum control time step possible */
        if (dt < 0.0) dt = 0.0;                             /* ensure no negative numbers are used */
    } else {
        dt = 0.;                                            /* set dt to zero to not use integration on first function call */
    }
    ConfigData->priorTime = callTime;

    /*! - Read input message */
    uint64_t            writeTime;
    uint32_t            writeSize;
    NavStateOut         navData;
    SpicePlanetState    primPlanet;
    attRefOut           pointRefData;
    
    ReadMessage(ConfigData->inputCelID, &writeTime, &writeSize,
                sizeof(SpicePlanetState), &primPlanet);
    ReadMessage(ConfigData->inputNavID, &writeTime, &writeSize,
                sizeof(NavStateOut), &navData);
    ReadMessage(ConfigData->inputRefID, &writeTime, &writeSize,
                sizeof(attRefOut), &pointRefData);
    
    
    /*! - Compute and store output message */
    computeorbitAxisSpinReference(ConfigData,
                             navData.r_BN_N,
                             navData.v_BN_N,
                             primPlanet.PositionVector,
                             primPlanet.VelocityVector,
                             pointRefData.sigma_RN,
                             pointRefData.omega_RN_N,
                             pointRefData.domega_RN_N,
                             BOOL_TRUE,                 /* integrate and update */
                             dt,
                             ConfigData->attRefOut.sigma_RN,
                             ConfigData->attRefOut.omega_RN_N,
                             ConfigData->attRefOut.domega_RN_N);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attRefOut),
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    return;
}

double computeInitialSpinAngle(orbitAxisSpinConfig *ConfigData, double sigma_ON)
{
    double phi_spin_0;
    double spin_align_angle;
    double spin_align_axis[3];
    
    double ON[3][3];
    double BN[3][3];
    double B0N[3][3];
    double OB[3][3];
    
    double BB0[3][3];
    double PRV_align[3];
    
    double temp33[3][3];
    double v3temp[3];
    
    int b1, b2, b3;
    
    b1 = ConfigData->b_spin;
    b2 = b1 + 1;
    if (b2 > 2) {b2 = 0;}
    b3 = b2 + 1;
    if (b3 > 2) {b3 = 0;}
    
    MRP2C(&ConfigData->sigma_B0N, B0N);
    MRP2C(&sigma_ON, temp33);
    v3Copy(temp33[0], ON[b1]);
    v3Copy(temp33[1], ON[b2]);
    v3Cross(ON[b1], ON[b2], ON[b3]);
    
    
    spin_align_angle = acos(v3Dot(B0N[b1],ON[b1])); /* ON[b1] = desired b-spin axis orientation
                                                      B0N[0] = initial b-spin axis orientation */
    v3Cross(B0N[b1], ON[b1], v3temp);
    v3Normalize(v3temp, spin_align_axis);
    v3Scale(spin_align_angle, spin_align_axis, spin_align_axis);
    PRV2C(spin_align_axis, BB0);
    
    /* [OB] = [ON] x transpose([BB0][B0N]) = [ON]xtranspose([BN]) = [ON][NB]*/
    m33tMultM33(BB0, B0N, BN);
    m33MultM33t(ON, BN, OB);
    C2PRV(OB, PRV_align);
    phi_spin_0 = v3Norm(PRV_align);
    if (v3Dot(PRV_align, ON[b1])<0) { phi_spin_0 = - phi_spin_0;}

    return phi_spin_0;
}

/*
 * Function: computeorbitAxisSpinReference
 * Purpose: compute the reference frame states for the Hill Frame spin control mode.  This function is
 designed to work both in: 
        * FSW to compute estimated pointing errors
        * Simulation code to compute true pointing errors
 * Inputs:
 *      ConfigData = module configuration data
 *      r_BN_N = inertial position of the spacecraft in inertial N-frame components
 *      v_BN_N = inertial velocity of the spacecraft in inertial N-frame components
 *      celBdyPositonVector = inertial position of the main celestial body in inertial N-frame components
 *      celBdyVelocityVector = inertial velocity of the main celestial body in inertial N-frame components
 *      integrateFlag = flag to reset the reference orientation
 *                   0 - integrate & evaluate
 *                  -1 - evalute but not integrate)
 *      dt = integration time step (control update period )
 * Outputs:
 *      sigma_RN = MRP attitude error of body relative to reference
 *      omega_RN_N = reference angluar velocity vector in body frame components
 *      domega_RN_N = reference angular acceleration vector in body frame componets
 */
void computeorbitAxisSpinReference(orbitAxisSpinConfig *ConfigData,
                              double r_BN_N[3],
                              double v_BN_N[3],
                              double celBdyPositonVector[3],
                              double celBdyVelocityVector[3],
                              double sigma_R0N[3],
                              double omega_R0N_N[3],
                              double domega_R0N_N[3],
                              int    integrateFlag,
                              double dt,
                              double sigma_RN[3],
                              double omega_RN_N[3],
                              double domega_RN_N[3])
{
    double  relPosVector[3];
    double  relVelVector[3];
    double  R0N[3][3];               /*!< DCM from inertial to pointing reference frame */
    double  ON[3][3];                /*!< DCM from orbit frame to pointing reference frame */
    double  RR0[3][3];               /*!< DCM from pointing reference frame to spin ref-frame */
    

    double  h[3];                    /*!< orbit angular momentum vector */
    
    double  dfdt;                    /*!< rotational rate of the orbit frame */
    double  v3[3];                   /*!< temp 3x1 vector */

    int     o1,o2;
    
    /* Set up orbit frame indicies */
    o1 = ConfigData->o_spin;
    o2 = o1 + 1;
    if ( o2 > 2 ) { o2 = 0; }
    
    /* Compute relative position and velocity of the spacecraft with respect to the main celestial body */
    v3Subtract(r_BN_N, celBdyPositonVector, relPosVector);
    v3Subtract(v_BN_N, celBdyVelocityVector, relVelVector);
    
    /* Compute R0N */
    v3Normalize(relPosVector, R0N[0]);
    v3Cross(relPosVector, relVelVector, h);
    v3Normalize(h, R0N[2]);
    v3Cross(R0N[2], R0N[0], R0N[1]);
    
    /* Compute reference frame rate */
    double omega_RR0_N[3];
    v3Scale(ConfigData->omega_spin, R0N[o1], omega_RR0_N);
    v3Add(omega_R0N_N, omega_RR0_N, omega_RN_N);
    
    
    /* Get true anomaly rate */
    dfdt = omega_R0N_N[2];
    
    /* Compute reference frame acceleration */
    double domega_RR0_N[3];
    v3Cross(R0N[2], R0N[o1], v3);
    v3Scale(dfdt*ConfigData->omega_spin, v3, domega_RR0_N);
    v3Add(domega_R0N_N, domega_RR0_N, domega_RN_N);
    
    /* Compute MRP attitude error */
    double sigma_RR0[3];
    m33SetZero(RR0);
    if (integrateFlag == BOOL_TRUE) { /* integrate the spin angle */
        ConfigData->phi_spin += dt * ConfigData->omega_spin;
    } else {
        /* initialize  spin angle */
        double sigma_ON;
        v3Copy(R0N[o1], ON[0]);
        v3Copy(R0N[o2], ON[1]);
        v3Cross(ON[0],ON[1],ON[2]);
        C2MRP(ON, &sigma_ON);
        ConfigData->phi_spin = computeInitialSpinAngle(ConfigData, sigma_ON);
        
    }
    Mi(ConfigData->phi_spin, o1+1, RR0);
    MRP2C(sigma_RR0, RR0);
    v3Add(sigma_RR0, sigma_R0N, sigma_RN);
}
