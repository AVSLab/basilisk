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
/*
    Inertial 3D Spin Module
 
 */


#include "attGuidance/hillPoint/hillPoint.h"
#include <string.h>
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"



void SelfInit_hillPoint(hillPointConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(AttRefFswMsg),
                                               "AttRefFswMsg",
                                               moduleID);
}

void CrossInit_hillPoint(hillPointConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputCelID = subscribeToMessage(ConfigData->inputCelMessName,
                                                sizeof(EphemerisIntMsg), moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavDataName,
                                                sizeof(NavTransIntMsg), moduleID);
}

void Reset_hillPoint(hillPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
}


void Update_hillPoint(hillPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read input message */
    uint64_t            writeTime;
    uint32_t            writeSize;
    NavTransIntMsg         navData;
    EphemerisIntMsg    primPlanet;

    /* zero the local planet ephemeris message */
    memset(&primPlanet, 0x0, sizeof(EphemerisIntMsg));
    
    ReadMessage(ConfigData->inputCelID, &writeTime, &writeSize,
                sizeof(EphemerisIntMsg), &primPlanet, moduleID);
    ReadMessage(ConfigData->inputNavID, &writeTime, &writeSize,
                sizeof(NavTransIntMsg), &navData, moduleID);

    /*! - Compute and store output message */
    computeHillPointingReference(ConfigData,
                                 navData.r_BN_N,
                                 navData.v_BN_N,
                                 primPlanet.r_BdyZero_N,
                                 primPlanet.v_BdyZero_N);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttRefFswMsg),   /* update module name */
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    return;
}


void computeHillPointingReference(hillPointConfig *ConfigData,
                                  double r_BN_N[3],
                                  double v_BN_N[3],
                                  double celBdyPositonVector[3],
                                  double celBdyVelocityVector[3])
{
    
    double  relPosVector[3];
    double  relVelVector[3];
    double  dcm_RN[3][3];            /*!< DCM from inertial to reference frame */
    double  temp33[3][3];
    
    double  rm;                      /*!< orbit radius */
    double  h[3];                    /*!< orbit angular momentum vector */
    double  hm;                      /*!< module of the orbit angular momentum vector */
    
    double  dfdt;                    /*!< rotational rate of the orbit frame */
    double  ddfdt2;                  /*!< rotational acceleration of the frame */
    double  omega_RN_R[3];           /*!< reference angular velocity vector in Reference frame R components */
    double  domega_RN_R[3];          /*!< reference angular acceleration vector in Reference frame R components */
    
    /* Compute relative position and velocity of the spacecraft with respect to the main celestial body */
    v3Subtract(r_BN_N, celBdyPositonVector, relPosVector);
    v3Subtract(v_BN_N, celBdyVelocityVector, relVelVector);
    
    /* Compute RN */
    v3Normalize(relPosVector, dcm_RN[0]);
    v3Cross(relPosVector, relVelVector, h);
    v3Normalize(h, dcm_RN[2]);
    v3Cross(dcm_RN[2], dcm_RN[0], dcm_RN[1]);
    
    /* Compute R-frame orientation */
    C2MRP(dcm_RN, ConfigData->attRefOut.sigma_RN);
    
    /* Compute R-frame inertial rate and acceleration */
    rm = v3Norm(relPosVector);
    hm = v3Norm(h);
    /* Robustness check */
    if(rm > 1.) {
        dfdt = hm / (rm * rm);  /* true anomaly rate */
        ddfdt2 = - 2.0 * v3Dot(relVelVector, dcm_RN[0]) / rm * dfdt; /* derivative of true anomaly rate */
    } else {
        /* an error has occured, radius shouldn't be less than 1km  */
        dfdt   = 0.;
        ddfdt2 = 0.;
    }
    v3SetZero(omega_RN_R);
    v3SetZero(domega_RN_R);
    omega_RN_R[2]  = dfdt;
    domega_RN_R[2] = ddfdt2;

    m33Transpose(dcm_RN, temp33);
    m33MultV3(temp33, omega_RN_R, ConfigData->attRefOut.omega_RN_N);
    m33MultV3(temp33, domega_RN_R, ConfigData->attRefOut.domega_RN_N);
    
}
