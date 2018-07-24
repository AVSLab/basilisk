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
 
 Velocity Pointing Guidance Module
 
 */


#include "attGuidance/velocityPoint/velocityPoint.h"
#include <string.h>
#include <math.h>
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simulation/utilities/orbitalMotion.h"
#include "simulation/utilities/astroConstants.h"


void SelfInit_velocityPoint(velocityPointConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(AttRefFswMsg),
                                               "AttRefFswMsg",
                                               moduleID);
    /*! - Initialize variables for module */
}

void CrossInit_velocityPoint(velocityPointConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputCelID = subscribeToMessage(ConfigData->inputCelMessName,
                                                sizeof(EphemerisIntMsg), moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavDataName,
                                                sizeof(NavTransIntMsg), moduleID);
}

void Reset_velocityPoint(velocityPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
}


void Update_velocityPoint(velocityPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
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
    computeVelocityPointingReference(ConfigData,
                                     navData.r_BN_N,
                                     navData.v_BN_N,
                                     primPlanet.r_BdyZero_N,
                                     primPlanet.v_BdyZero_N);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttRefFswMsg),   /* update module name */
                 (void*) &(ConfigData->attRefOut), moduleID);
    
    return;
}


void computeVelocityPointingReference(velocityPointConfig *ConfigData,
                                      double r_BN_N[3],
                                      double v_BN_N[3],
                                      double celBdyPositonVector[3],
                                      double celBdyVelocityVector[3])
{
    double  dcm_RN[3][3];            /*!< DCM from inertial to reference frame */
    
    double  r[3];                    /*!< relative position vector of the spacecraft with respect to the orbited planet */
    double  v[3];                    /*!< relative velocity vector of the spacecraft with respect to the orbited planet  */
    double  h[3];                    /*!< orbit angular momentum vector */
    double  rm;                      /*!< orbit radius */
    double  hm;                      /*!< module of the orbit angular momentum vector */
    
    double  dfdt;                    /*!< rotational rate of the orbit frame */
    double  ddfdt2;                  /*!< rotational acceleration of the frame */
    double  omega_RN_R[3];           /*!< reference angular velocity vector in Reference frame R components */
    double  domega_RN_R[3];          /*!< reference angular acceleration vector in Reference frame R components */
    
    double  temp33[3][3];
    double  temp;
    double  denom;
    
    /* zero the reference rate and acceleration vectors */
    v3SetZero(omega_RN_R);
    v3SetZero(domega_RN_R);

    /* Compute relative position and velocity of the spacecraft with respect to the main celestial body */
    v3Subtract(r_BN_N, celBdyPositonVector, r);
    v3Subtract(v_BN_N, celBdyVelocityVector, v);
    
    /* Compute RN */
    v3Normalize(v, dcm_RN[1]);
    v3Cross(r, v, h);
    v3Normalize(h, dcm_RN[2]);
    v3Cross(dcm_RN[1], dcm_RN[2], dcm_RN[0]);
    
    /* Compute R-frame orientation */
    C2MRP(dcm_RN, ConfigData->attRefOut.sigma_RN);
    
    /* Compute R-frame inertial rate and acceleration */
    rv2elem(ConfigData->mu, r, v, &ConfigData->oe);
    rm = v3Norm(r);
    hm = v3Norm(h);
    /* Robustness check */
    if(rm > 1.) {
        dfdt = hm / (rm * rm);  /* true anomaly rate */
        ddfdt2    = - 2.0 * (v3Dot(v, r) / (rm * rm)) * dfdt;
        denom = 1 + ConfigData->oe.e * ConfigData->oe.e + 2 * ConfigData->oe.e * cos(ConfigData->oe.f);
        temp = (1 + ConfigData->oe.e * cos(ConfigData->oe.f)) / denom;
        omega_RN_R[2]  = dfdt * temp;
        domega_RN_R[2] = ddfdt2 * temp - dfdt*dfdt* ConfigData->oe.e *(ConfigData->oe.e*ConfigData->oe.e - 1)*sin(ConfigData->oe.f) / (denom*denom);
    } else {
        dfdt   = 0.;
        ddfdt2 = 0.;
    }
    m33Transpose(dcm_RN, temp33);
    m33MultV3(temp33, omega_RN_R, ConfigData->attRefOut.omega_RN_N);
    m33MultV3(temp33, domega_RN_R, ConfigData->attRefOut.domega_RN_N);
}
