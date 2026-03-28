/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef STRIP_LOCATION_H
#define STRIP_LOCATION_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/AccessMsgPayload.h"
#include "architecture/msgPayloadDefC/StripStateMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/geodeticConversion.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief strip location class */
class StripLocation: public SysModel {
public:
    StripLocation();
    ~StripLocation();
    void UpdateState(uint64_t currentSimNanos);
    void Reset(uint64_t currentSimNanos);
    bool ReadMessages();
    void WriteMessages(uint64_t currentClock);
    void addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg);
    void specifyLocationStart(double lat, double longitude, double alt);
    void specifyLocationEnd(double lat, double longitude, double alt);

private:
    void updateInertialPosition();
    void updateTargetPositionPCPF(uint64_t currentClock);
    void computeAccess();
    void newpstart();
    Eigen::Vector3d PositionCentralLine(double t, double teta, const Eigen::Vector3d& pStart, const Eigen::Vector3d& pEnd) const;
    Eigen::Vector3d VelocityCentralLine(double t, double teta, const Eigen::Vector3d& pStart, const Eigen::Vector3d& pEnd) const;


public:
    double minimumElevation; //!< [rad] minimum elevation above the local horizon ; defaults to 10 degrees equivalent
    double maximumRange; //!< [m] (optional) maximum range ; defaults to -1, which represents no maximum range
    double smallAngle; //!< [rad] numerical tolerance when evaluating near-zero strip central angles
    double planetRadius; //!< [m] planet radius
    Eigen::Vector3d r_LP_P_Start; //!< [m] ground location of the starting point of the strip relative to PCPF
    Eigen::Vector3d r_LP_P_End; //!< [m] ground location of the ending point of the strip relative to PCPF
    Eigen::Vector3d pStart; //!< [m] normalized starting point to make sure it is on the Earth surface for the interpolation
    Eigen::Vector3d pStartUpdated; //!< [m] updated starting point after pre-imaging update
    Eigen::Vector3d pEnd; //!< [m] normalized ending point to make sure it is on the Earth surface for the interpolation
    double acquisitionSpeed; //!< [m/ns] constant acquisition speed of the camera
    double preImagingTime;  //!< [ns] pre-imaging time used to artificially modify pStart
    ReadFunctor<SpicePlanetStateMsgPayload> planetInMsg;            //!< planet state input message
    Message<StripStateMsgPayload> currentStripStateOutMsg;    //!< current target location and velocity output message
    std::vector<Message<AccessMsgPayload>*> accessOutMsgs;           //!< vector of current ground target access message
    std::vector<ReadFunctor<SCStatesMsgPayload>> scStateInMsgs; //!< vector of spacecraft state input message
    uint64_t durationStripImaging; //!< [ns] time already spent to image the strip (need to be re-initialized to 0 when imaging consecutive strips)
    bool isStartPositionUpdated; //!< flag indicating if the updated start position has already been computed
    BSKLogger bskLogger;         //!< BSK Logging

private:
    StripStateMsgPayload currentStripStateBuffer;  //!< buffer of target location and velocity output data
    SpicePlanetStateMsgPayload planetState; //!< buffer of planet data
    std::vector<AccessMsgPayload> accessMsgBuffer;                  //!< buffer of access output data
    std::vector<SCStatesMsgPayload> scStatesBuffer;             //!< buffer of spacecraft states

    Eigen::Vector3d r_North_N; //!< inertial 3rd axis, defined as "North"
    Eigen::Vector3d r_LP_P; //!< [m] ground location of the current target point on the central line of the strip relative to PCPF
    Eigen::Vector3d r_LP_N; //!< [m] current target position vector relative to planet origin vector in inertial coordinates
    Eigen::Vector3d r_LN_N;//!< [m] current target position vector relative to inertial frame origin in inertial coordinates
    Eigen::Vector3d r_PN_N; //!< [m] planet position vector relative to inertial frame origin
    Eigen::Vector3d v_LP_P; //!< [m/s] velocity of the current target point on the central line of the strip relative to PCPF
    Eigen::Vector3d v_LP_N; //!< [m/s] velocity vector of the current target point relative to the planet in the inertial frame
    Eigen::Vector3d v_LN_N; //!< [m/s] velocity vector of the current target point relative to the inertial origin in the inertial frame
    Eigen::Matrix3d dcm_LP; //!< rotation matrix from planet-centered, planet-fixed frame P to site-local topographic (SEZ) frame L coordinates
    Eigen::Matrix3d dcm_PN; //!< rotation matrix from inertial frame N to planet-centered to planet-fixed frame P
    Eigen::Matrix3d dcm_PN_dot; //!< rotation matrix derivative from inertial frame N to planet-centered to planet-fixed frame P
    Eigen::Vector3d w_PN; //!< [rad/s] angular velocity of planet-fixed frame P relative to inertial frame N
    Eigen::Vector3d rhat_LP_N;//!< surface normal vector from the target location in inertial coordinates
    double theta;//!< [rad] angle between pStart and pEnd from the center of Earth (the Earth is assumed perfectly spherical)
    double thetaUpdated; //!< [rad] updated angle between pStartUpdated and pEnd used in target propagation
    double lengthCentralLine; //!< [m] length of the central line
    double lengthCentralLineUpdated; //!< [m] updated length of the central line taking into account the pre-imaging time
    uint64_t OldSimNanos; //!< [ns] previous currentSimNanos
};
#endif /* StripLocation */
