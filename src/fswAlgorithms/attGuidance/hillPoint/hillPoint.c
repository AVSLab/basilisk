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



/*! This method creates the module output message of type [AttRefFswMsg](\ref AttRefFswMsg).
 @return void
 @param configData The configuration data associated with RW null space model
 @param moduleID The ID associated with the configData
 */
void SelfInit_hillPoint(hillPointConfig *configData, uint64_t moduleID)
{
    /*! - Create output message for module */
    configData->outputMsgID = CreateNewMessage(configData->outputDataName,
                                               sizeof(AttRefFswMsg),
                                               "AttRefFswMsg",
                                               moduleID);
}

/*! This method performs the second stage of initialization
 interface.  This module has two messages to subscribe to of type [EphemerisIntMsg](\ref EphemerisIntMsg) and [NavTransIntMsg](\ref NavTransIntMsg).
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
 */
void CrossInit_hillPoint(hillPointConfig *configData, uint64_t moduleID)
{
    /*! - subscribe to other message*/
    /*! - inputCelID provides the planet ephemeris message.  Note that if this message does
          not exist, this subscribe function will create an empty planet message.  This behavior
          is by design such that if a planet doesn't have a message, default (0,0,0) position
          and velocity vectors are assumed. */
    configData->inputCelID = subscribeToMessage(configData->inputCelMessName,
                                                sizeof(EphemerisIntMsg), moduleID);
    /*! - inputNavID provides the current spacecraft location and velocity */
    configData->inputNavID = subscribeToMessage(configData->inputNavDataName,
                                                sizeof(NavTransIntMsg), moduleID);
}

/*! This method performs the module reset capability.  This module has no actions.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
 */
void Reset_hillPoint(hillPointConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    
}


/*! This method creates a orbit hill frame reference message.  The desired orientation is
 defined within the module.
 @return void
 @param configData The configuration data associated with the null space control
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Update_hillPoint(hillPointConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read input message */
    uint64_t            timeOfMsgWritten;
    uint32_t            sizeOfMsgWritten;
    NavTransIntMsg      navData;
    EphemerisIntMsg     primPlanet;
    AttRefFswMsg        attRefOut;

    /*! - zero the output message */
    memset(&attRefOut, 0x0, sizeof(AttRefFswMsg));

    /* zero the local planet ephemeris message */
    memset(&primPlanet, 0x0, sizeof(EphemerisIntMsg));
    ReadMessage(configData->inputCelID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(EphemerisIntMsg), &primPlanet, moduleID);
    memset(&navData, 0x0, sizeof(NavTransIntMsg));
    ReadMessage(configData->inputNavID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(NavTransIntMsg), &navData, moduleID);

    printf("r_planet = %g, %g, %g \t", primPlanet.r_BdyZero_N[0], primPlanet.r_BdyZero_N[1], primPlanet.r_BdyZero_N[2]);
    printf("v_planet = %g, %g, %g \n", primPlanet.v_BdyZero_N[0], primPlanet.v_BdyZero_N[1], primPlanet.v_BdyZero_N[2]);
    
    /*! - Compute and store output message */
    computeHillPointingReference(configData,
                                 navData.r_BN_N,
                                 navData.v_BN_N,
                                 primPlanet.r_BdyZero_N,
                                 primPlanet.v_BdyZero_N,
                                 &attRefOut);
    
    WriteMessage(configData->outputMsgID, callTime, sizeof(AttRefFswMsg),   /* update module name */
                 (void*) &(attRefOut), moduleID);
    
    return;
}


void computeHillPointingReference(hillPointConfig *configData,
                                  double r_BN_N[3],
                                  double v_BN_N[3],
                                  double celBdyPositonVector[3],
                                  double celBdyVelocityVector[3],
                                  AttRefFswMsg *attRefOut)
{
    
    double  relPosVector[3];
    double  relVelVector[3];
    double  dcm_RN[3][3];            /* DCM from inertial to reference frame */
    double  dcm_NR[3][3];            /* DCM from reference to inertial frame */
    
    double  rm;                      /* orbit radius */
    double  h[3];                    /* orbit angular momentum vector */
    double  hm;                      /* module of the orbit angular momentum vector */
    
    double  dfdt;                    /* rotational rate of the orbit frame */
    double  ddfdt2;                  /* rotational acceleration of the frame */
    double  omega_RN_R[3];           /* reference angular velocity vector in Reference frame R components */
    double  domega_RN_R[3];          /* reference angular acceleration vector in Reference frame R components */
    
    /*! - Compute relative position and velocity of the spacecraft with respect to the main celestial body */
    v3Subtract(r_BN_N, celBdyPositonVector, relPosVector);
    v3Subtract(v_BN_N, celBdyVelocityVector, relVelVector);
    
    /* Compute RN */
    v3Normalize(relPosVector, dcm_RN[0]);
    v3Cross(relPosVector, relVelVector, h);
    v3Normalize(h, dcm_RN[2]);
    v3Cross(dcm_RN[2], dcm_RN[0], dcm_RN[1]);
    
    /*! - Compute R-frame orientation */
    C2MRP(dcm_RN, attRefOut->sigma_RN);
    
    /*! - Compute R-frame inertial rate and acceleration */
    rm = v3Norm(relPosVector);
    hm = v3Norm(h);

    /*! - determine orbit angular rates and accelerations */
    if(rm > 1.) { /* Robustness check */
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


    m33Transpose(dcm_RN, dcm_NR);
    m33MultV3(dcm_NR, omega_RN_R, attRefOut->omega_RN_N);
    m33MultV3(dcm_NR, domega_RN_R, attRefOut->domega_RN_N);
    
}
