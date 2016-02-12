/*
 Inertial 3D Spin Module
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved
 
 */


#include "attGuidance/orbitAxisSpin/orbitAxisSpin.h"
#include <string.h>
#include <math.h>
#include "ADCSUtilities/ADCSDefinitions.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"

/* Required module input messages */
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"
#include "SimCode/environment/spice/spice_planet_state.h"
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"



void SelfInit_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(attRefOut),
                                               "attRefOut",
                                               moduleID);
}

void CrossInit_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputRefID = subscribeToMessage(ConfigData->inputRefName,
                                                sizeof(attRefOut),
                                                moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavName,
                                                sizeof(NavStateOut),
                                                moduleID);
}

void Reset_orbitAxisSpin(orbitAxisSpinConfig *ConfigData)
{
    ConfigData->phi_spin = 0.;
}


void Update_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read input message */
    uint64_t            writeTime;
    uint32_t            writeSize;
    attRefOut           ref;                        /*!< reference guidance message */
    
    
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputRefID, &writeTime, &writeSize,
                sizeof(attRefOut), (void*) &(ref));
    
    double dt;
    if (callTime*NANO2SEC == 0.) { dt = 0.; }
    else (dt = ConfigData->dt);
    
    /*! - Compute and store output message */
    if (ConfigData->integrateFlag == BOOL_FALSE)
    {
        NavStateOut nav;                           /*!< navigation message */
        ReadMessage(ConfigData->inputNavID, &writeTime, &writeSize,
                    sizeof(NavStateOut), (void*) &(nav));
        
        ConfigData->phi_spin = computeInitialSpinAngle(ConfigData,
                                                       ref.sigma_RN,
                                                       nav.sigma_BN);
    }
    computeOrbitAxisSpinReference(ConfigData,
                                  ref.sigma_RN,
                                  ref.omega_RN_N,
                                  ref.domega_RN_N,
                                  dt,
                                  ConfigData->attRefOut.sigma_RN,
                                  ConfigData->attRefOut.omega_RN_N,
                                  ConfigData->attRefOut.domega_RN_N);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attRefOut),
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    return;
}


void computeOrbitAxisSpinReference(orbitAxisSpinConfig *ConfigData,
                                   double sigma_R0N[3],
                                   double omega_R0N_N[3],
                                   double domega_R0N_N[3],
                                   double dt,
                                   double sigma_RN[3],
                                   double omega_RN_N[3],
                                   double domega_RN_N[3])
{
    
    double  R0N[3][3];                                      /*!< DCM from inertial to reference pointing frame */
    int     o1, o2;                                         /*!< orbit axis indices */
    
    double  v3[3], v3temp[3], temp3[3];                     /*!< temporary variables */
    double  m33[3][3];
    
    o1 = ConfigData->o_spin;
    o2 = o1 + 1;
    if (o2 > 2) { o2 = 0; }
    /* Get Orbit Frame DCM */
    MRP2C(sigma_R0N, R0N);
    /* Compute rate */
    v3Scale(ConfigData->omega_spin, R0N[o1], v3);
    v3Add(v3, omega_R0N_N, omega_RN_N);
    /* Compute acceleration */
    v3Cross(omega_R0N_N, v3, v3temp);
    v3Add(v3temp, domega_R0N_N, domega_RN_N);
    /* Compute orientation */
    ConfigData->phi_spin += dt * ConfigData->omega_spin;
    Mi(ConfigData->phi_spin, o1+1, m33);
    C2MRP(m33, temp3);
    v3Add(temp3, sigma_R0N, sigma_RN);
    
}


double computeInitialSpinAngle(orbitAxisSpinConfig *ConfigData,
                               double sigma_R0N[3],
                               double sigma_BN[3])
{
    double R0N[3][3];       /* Input Orbit frame orientation (could be Hill or Velocity) */
    double BN[3][3];        /* Current Body frame orientation */
    double RN[3][3];        /* Desired Reference frame orientation (b_spin axis aligned with o_spin axis) */
    
    double phi_spin_0;
    double spin_align_angle;
    double spin_align_axis[3];
    double PRV_align[3];
    double temp33[3][3], m33[3][3];
    double v3temp[3];
    
    int o1, o2;             /* Orbit axes indices */
    int b1, b2, b3;         /* Body axes indices */
    
    o1 = ConfigData->o_spin;
    o2 = o1 + 1;
    if (o2 > 2) { o2 = 0;}
    
    b1 = ConfigData->b_spin;
    b2 = b1 + 1;
    if (b2 > 2) {b2 = 0;}
    b3 = b2 + 1;
    if (b3 > 2) {b3 = 0;}
    
    MRP2C(sigma_R0N, R0N);
    MRP2C(sigma_BN, BN);
    
    v3Copy(R0N[o1], RN[b1]);
    v3Copy(R0N[o2], RN[b2]);
    v3Cross(RN[b1], RN[b2], RN[b3]);
    
    spin_align_angle = acos(v3Dot(BN[b1], RN[b1])); /* RN[b1] = desired b-spin axis orientation
                                                     BN[b1] = initial b-spin axis orientation */
    v3Cross(BN[b1], RN[b1], v3temp);
    v3Normalize(v3temp, spin_align_axis);
    v3Scale(spin_align_angle, spin_align_axis, spin_align_axis);
    
    PRV2C(spin_align_axis, temp33);
    m33tMultM33(temp33, BN, temp33);
    m33MultM33t(RN, temp33, m33);
    C2PRV(m33, PRV_align);
    phi_spin_0 = v3Norm(PRV_align);
    if (v3Dot(PRV_align, RN[b1])<0)                  /* make sure we initialize the spinning to perform the smallest rotation */
    {
        phi_spin_0 = - phi_spin_0;
    }
    return phi_spin_0;
}



