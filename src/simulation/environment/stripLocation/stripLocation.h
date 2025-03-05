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
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);
    bool ReadMessages();
    void WriteMessages(uint64_t CurrentClock);
    void addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg);
    void specifyLocationStart(double lat, double longitude, double alt);
    void specifyLocationEnd(double lat, double longitude, double alt);
    void lenght_line();
    void newpstart();


private:
    void updateInertialPosition();
    void updateTargetPositionPCPF(uint64_t CurrentClock);
    void computeAccess();
    Eigen::Vector3d PositionCentralLine(double t);
    Eigen::Vector3d VelocityCentralLine(double t);


public:
    double minimumElevation; //!< [rad] minimum elevation above the local horizon needed to see a spacecraft; defaults to 10 degrees equivalent.
    double maximumRange; //!< [m] (optional) Maximum slant range to compute access for; defaults to -1, which represents no maximum range.
    double planetRadius; //!< [m] Planet radius in meters.
    Eigen::Vector3d r_LP_P_Start; //!< [m] Ground location of the first point to image on the central line of the strip relative to PCPF
    Eigen::Vector3d r_LP_P_End; //!< [m] Ground location of the last point to image on the central line of the strip relative to PCPF
    Eigen::Vector3d p_start; //!< [m] Making sure the location of the first point is on the Earth surface for the interpolation
    Eigen::Vector3d p_end;//[m] Making sure the location of the last point is on the Earth surface for the interpolation
    double acquisition_speed; //!< [m/s] Constant acquisition speed of the camera
    double pre_imaging_time;  //!< [s] Pre_imaging time used to artificially modify r_LP_P_Start
    ReadFunctor<SpicePlanetStateMsgPayload> planetInMsg;            //!< planet state input message
    Message<StripStateMsgPayload> currentStripStateOutMsg;    //!< strip location output message
    std::vector<Message<AccessMsgPayload>*> accessOutMsgs;           //!< vector of ground location access messages
    std::vector<ReadFunctor<SCStatesMsgPayload>> scStateInMsgs; //!< vector of sc state input messages
    BSKLogger bskLogger;         //!< -- BSK Logging
    double theta;//!< [rad] Angle between r_LP_P_Start and r_LP_P_End from the center of Earth (The Earth is assumed perfectly spherical.)
    double lenght_central_line; //!< [m] Lenght of the central line
    Eigen::Vector3d r_PN_N; //!< [m] Planet position vector relative to inertial frame origin.
    Eigen::Vector3d v_LP_N; //!< [m] Velocity vector of the current target point on the central line of the strip in the inertial frame 


private:
    StripStateMsgPayload currentStripStateBuffer;  //!< buffer of ground station output data
    SpicePlanetStateMsgPayload planetState; //!< buffer of planet data
    Eigen::Vector3d r_North_N; //!<[-] Inertial 3rd axis, defined internally as "North".
    Eigen::Vector3d r_LP_P; //!< [m] Ground location of the current target point on the central line of the strip relative to PCPF
    Eigen::Vector3d v_LP_P; //!< [m/s] Velocity of the current target point on the central line of the strip relative to PCPF
    uint64_t duration_strip_imaging; //!< [s] Time already spent to image the strip
    uint64_t OldSimNanos; //!< [s] Previous CurrentSimNanos
   

    std::vector<AccessMsgPayload> accessMsgBuffer;                  //!< buffer of access output data
    std::vector<SCStatesMsgPayload> scStatesBuffer;             //!< buffer of spacecraft states
    Eigen::Matrix3d dcm_LP; //!< Rotation matrix from planet-centered, planet-fixed frame P to site-local topographic (SEZ) frame L coordinates.
    Eigen::Matrix3d dcm_PN; //!< Rotation matrix from inertial frame N to planet-centered to planet-fixed frame P.
    Eigen::Matrix3d dcm_PN_dot; //!< Rotation matrix derivative from inertial frame N to planet-centered to planet-fixed frame P.
    Eigen::Vector3d w_PN; //!< [rad/s] Angular velocity of planet-fixed frame P relative to inertial frame N.
    Eigen::Vector3d r_LP_N; //!< [m] Gound Location position vector relative to planet origin vector in inertial coordinates.
    Eigen::Vector3d rhat_LP_N;//!< [-] Surface normal vector from the target location in inertial coordinates.
    Eigen::Vector3d r_LN_N;//!< [m] Ground Location position vector relative to inertial frame origin in inertial coordinates.
};
#endif /* StripLocation */







