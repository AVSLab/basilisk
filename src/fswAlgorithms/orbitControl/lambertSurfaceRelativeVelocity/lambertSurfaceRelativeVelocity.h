/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef LAMBERTSURFACERELATIVEVELOCITY_H
#define LAMBERTSURFACERELATIVEVELOCITY_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/DesiredVelocityMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"
#include <vector>
#include <array>

/*! @brief This module computes the inertial velocity corresponding to a given position and relative velocity to the
    celestial body surface
 */
class LambertSurfaceRelativeVelocity: public SysModel {
public:
    LambertSurfaceRelativeVelocity();
    ~LambertSurfaceRelativeVelocity();

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;

    ReadFunctor<LambertProblemMsgPayload> lambertProblemInMsg;          //!< lambert problem input message
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg;                    //!< ephemeris input message
    Message<DesiredVelocityMsgPayload> desiredVelocityOutMsg;           //!< desired inertial velocity output message

    BSKLogger bskLogger;                                                //!< BSK Logging

    /** setter for `vRelativeDesired_S` */
    void setVRelativeDesired_S(const Eigen::Vector3d value);
    /** getter for `vRelativeDesired_S` */
    Eigen::Vector3d getVRelativeDesired_S() const {return this->vRelativeDesired_S;}
    /** setter for `time` */
    void setTime(const double value);
    /** getter for `time` */
    double getTime() const {return this->time;}

private:
    void readMessages();
    void writeMessages(uint64_t currentSimNanos);

    Eigen::Vector3d vRelativeDesired_S; //!< [m/s] desired relative velocity, in surface frame S (East-North-Up)
    double time{}; //!< [s] time for the desired velocity of the spacecraft
    Eigen::Vector3d r_BN_N; //!< [m] position of spacecraft, expressed in inertial frame N
    Eigen::Vector3d v_BN_N; //!< [m/s] velocity of spacecraft, expressed in inertial frame N
    Eigen::Matrix3d dcm_PN; //!< DCM of the orbital body fixed frame relative to inertial
    Eigen::Vector3d omega_PN_N; //!< [r/s] angular velocity of the orbital body relative to inertial
};

#endif
