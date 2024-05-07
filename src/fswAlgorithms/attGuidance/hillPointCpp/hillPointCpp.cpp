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
    Hill Point Module

 */


#include "hillPointCpp.h"
#include "architecture/utilities/avsEigenSupport.h"


/*! This method performs the module reset capability.  This module has no actions. */
void HillPointCpp::Reset(uint64_t CurrentSimNanos){
    // check if the required input message is included
    if (!NavTransMsg_C_isLinked(&this->transNavInMsg)) {
        _bskLog(this->bskLogger, BSK_ERROR, "Error: hillPoint.transNavInMsg wasn't connected.");
    }
    this->planetMsgIsLinked = EphemerisMsg_C_isLinked(&this->celBodyInMsg);

    AttRefMsg_C_init(&this->attRefOutMsg);
}


/*! This method creates a orbit hill frame reference message.  The desired orientation is
 defined within the module.
 */
void HillPointCpp::UpdateState(uint64_t CurrentSimNanos){
    /*! - Read input message */
    NavTransMsgPayload      navData;
    EphemerisMsgPayload     primPlanet;
    AttRefMsgPayload        attRefOut;

    /*! - zero the output message */
    attRefOut = AttRefMsg_C_zeroMsgPayload();

    /* zero the local planet ephemeris message */
    primPlanet = EphemerisMsg_C_zeroMsgPayload();       /* zero'd as default, even if not connected */
    if (this->planetMsgIsLinked) {
        primPlanet = EphemerisMsg_C_read(&this->celBodyInMsg);
    }
    navData = NavTransMsg_C_read(&this->transNavInMsg);

    /*! - Compute and store output message */
    computeHillPointingReference((Eigen::Vector3d)navData.r_BN_N,
                                 (Eigen::Vector3d)navData.v_BN_N,
                                 (Eigen::Vector3d)primPlanet.r_BdyZero_N,
                                 (Eigen::Vector3d)primPlanet.v_BdyZero_N,
                                 &attRefOut);

    AttRefMsg_C_write(&attRefOut, &this->attRefOutMsg, moduleID, CurrentSimNanos);
}


void HillPointCpp::computeHillPointingReference(Eigen::Vector3d r_BN_N,
                                                Eigen::Vector3d v_BN_N,
                                                Eigen::Vector3d celBdyPositonVector,
                                                Eigen::Vector3d celBdyVelocityVector,
                                                AttRefMsgPayload *attRefOut) {
    
    Eigen::Vector3d  relPosVector;
    Eigen::Vector3d  relVelVector;
    Eigen::Matrix3d  dcm_RN;            /* DCM from inertial to reference frame */
    Eigen::Vector3d  orbitAngMomentum; /* orbit angular momentum vector */
    Eigen::Vector3d  omega_RN_R;           /* reference angular velocity vector in Reference frame R components */
    Eigen::Vector3d  domega_RN_R;          /* reference angular acceleration vector in Reference frame R components */

    double  orbitRadius;                      /* orbit radius */
    double  dfdt;                    /* rotational rate of the orbit frame */
    double  ddfdt2;                  /* rotational acceleration of the frame */

    /*! - Compute relative position and velocity of the spacecraft with respect to the main celestial body */
    relPosVector = r_BN_N - celBdyPositonVector;
    relVelVector = v_BN_N - celBdyVelocityVector;

    /*! - Compute RN */
    /*! - First row of RN is i_r */
    dcm_RN.row(0) = relPosVector.normalized();

    /*! - Third row of RN is i_h */
    orbitAngMomentum = relPosVector.cross(relVelVector);
    dcm_RN.row(2) = orbitAngMomentum.normalized();

    /*! - Second row of RN is i_theta */
    dcm_RN.row(1) = dcm_RN.row(2).cross(dcm_RN.row(0));

    /*! - Compute R-frame orientation */
    Eigen::Vector3d sigma_RN;
    sigma_RN = eigenMRPd2Vector3d(eigenC2MRP(dcm_RN));
    eigenVector3d2CArray(sigma_RN, attRefOut->sigma_RN);

    /*! - Compute R-frame inertial rate and acceleration */
    orbitRadius = relPosVector.norm();

    /*! - determine orbit angular rates and accelerations */
    if(orbitRadius > 1.) { /* Robustness check */
        dfdt = orbitAngMomentum.norm() / (orbitRadius * orbitRadius);  /* true anomaly rate */
        ddfdt2 = - 2.0 * relVelVector.dot(dcm_RN.row(0)) / orbitRadius * dfdt; /* derivative of true anomaly rate */
    } else {
        /* an error has occured, radius shouldn't be less than 1km  */
        dfdt   = 0.;
        ddfdt2 = 0.;
    }
    omega_RN_R << 0.0, 0.0, dfdt;
    domega_RN_R << 0.0, 0.0, ddfdt2;

    Eigen::Vector3d temp;
    temp = dcm_RN.transpose()*omega_RN_R;
    eigenVector3d2CArray(temp, attRefOut->omega_RN_N);
    temp = dcm_RN.transpose()*domega_RN_R;
    eigenVector3d2CArray(temp, attRefOut->domega_RN_N);
}
