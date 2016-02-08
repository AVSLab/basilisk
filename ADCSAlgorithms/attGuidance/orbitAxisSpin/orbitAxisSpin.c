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



void SelfInit_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(attRefOut),
                                               "attRefOut",
                                               moduleID);
    /*! - #TEMP: Initialize variable */
    v3SetZero(ConfigData->sigma_BN);
    ConfigData->b_spin = 0;
    
}

void CrossInit_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputRefID = subscribeToMessage(ConfigData->inputRefName,
                                                sizeof(attRefOut),
                                                moduleID);
}

void Reset_orbitAxisSpin(orbitAxisSpinConfig *ConfigData)
{
    ConfigData->phi_spin_0 = 0.;
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
    computeOrbitAxisSpinReference(ConfigData,
                                  ref.sigma_RN,
                                  ref.omega_RN_N,
                                  ref.domega_RN_N,
                                  BOOL_TRUE,
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
                              int    integrateFlag,
                              double dt,
                              double sigma_RN[3],
                              double omega_RN_N[3],
                              double domega_RN_N[3])
{
    
    double  R0N[3][3];               /*!< DCM from inertial to reference pointing frame */
    double  orbitRate;                    /*!< rotational rate of the orbit frame */
    int     o1, o2;
    double  m33[3][3], v3[3], temp3[3], v3temp[3];

    o1 = ConfigData->o_spin;
    o2 = o1 + 1;
    if (o2 > 2) { o2 = 0; }
    
    MRP2C(sigma_R0N, R0N);
    
    v3Scale(ConfigData->omega_spin, R0N[o1], v3);
    v3Add(v3, omega_R0N_N, omega_RN_N);
    
    orbitRate = v3Norm(omega_R0N_N);
    v3Cross(R0N[2], R0N[o1], temp3);
    v3Scale(orbitRate*ConfigData->omega_spin, temp3, v3temp);
    v3Add(v3temp, domega_R0N_N, domega_RN_N);
    
    if (integrateFlag == BOOL_TRUE)
    {
        ConfigData->phi_spin += dt * ConfigData->omega_spin;
    } else {
        double ON[3][3];
        double sigma_ON[3];
        v3Copy(R0N[o1], ON[0]);
        v3Copy(R0N[o2], ON[1]);
        v3Cross(ON[0],ON[1],ON[2]);
        C2MRP(ON, sigma_ON);
        ConfigData->phi_spin = computeInitialSpinAngle(ConfigData, sigma_ON);
    }
    Mi(ConfigData->phi_spin, o1+1, m33);
    C2MRP(m33, v3);
    v3Add(v3, sigma_R0N, sigma_RN);
    
}


double computeInitialSpinAngle(orbitAxisSpinConfig *ConfigData, double sigma_ON[3])
{
    double phi_spin_0;
    double spin_align_angle;
    double spin_align_axis[3];
    
    double ON[3][3];        /* DCM from inertial N to orbit frame O */
    double B0N[3][3];       /* DCM from inertial to initial body B-frame */
    double BN[3][3];        /* DCM from inertial to body B-frame orientation after spin-axis alignment */
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
    
    MRP2C(ConfigData->sigma_BN, B0N);
    MRP2C(sigma_ON, temp33);
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
    if (v3Dot(PRV_align, ON[b1])<0)                  /* make sure we initialize the spinning to perform the smallest rotation */
    {
        phi_spin_0 = - phi_spin_0;
    }
    
    return phi_spin_0;
}



