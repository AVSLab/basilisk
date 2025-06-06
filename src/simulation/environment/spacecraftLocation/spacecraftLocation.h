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

#ifndef GROUND_LOCATION_H
#define GROUND_LOCATION_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AccessMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief ground location class */
class SpacecraftLocation : public SysModel
{
  public:
    SpacecraftLocation();
    ~SpacecraftLocation();
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);
    bool ReadMessages();
    void WriteMessages(uint64_t CurrentClock);
    void addSpacecraftToModel(Message<SCStatesMsgPayload>* tmpScMsg);

  private:
    void computeAccess();

  public:
    double rEquator;     //!< [m] equatorial planet radius
    double rPolar;       //!< [m] polar planet radius
    double maximumRange; //!< [m] Maximum slant range to compute access for; defaults to -1, which represents no maximum
                         //!< range.
    Eigen::Vector3d aHat_B; //!< [] (optional) unit direction vector of the sensor/communication boresight axis
    double theta;           //!< [r] (optional) sensor/communication half-cone angle, must be set if shat_B is specified
    double theta_solar;     //!< [r] (optional) illumination half-cone angle, treating aHat_B as the surface normal
    double min_shadow_factor; //!< [] (optional) minimum amount of illumination due to eclipse necessary to observe

    ReadFunctor<SCStatesMsgPayload> primaryScStateInMsg;        //!< primary spacecraft input message
    ReadFunctor<SpicePlanetStateMsgPayload> planetInMsg;        //!< (optional) planet state input message
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;           //!< (optional) sun data input message
    ReadFunctor<EclipseMsgPayload> eclipseInMsg;                //!< (optional) eclipse input message
    std::vector<Message<AccessMsgPayload>*> accessOutMsgs;      //!< vector of ground location access messages
    std::vector<ReadFunctor<SCStatesMsgPayload>> scStateInMsgs; //!< vector of other sc state input messages
    Eigen::Vector3d
      r_LB_B; //!< [m]  position of the location relative to the spacecraft frame origin B, in B frame components

    BSKLogger bskLogger; //!< -- BSK Logging

  private:
    std::vector<AccessMsgPayload> accessMsgBuffer;  //!< buffer of access output data
    std::vector<SCStatesMsgPayload> scStatesBuffer; //!< buffer of other spacecraft states
    SCStatesMsgPayload primaryScStatesBuffer;       //!< buffer of primary spacecraft states
    SpicePlanetStateMsgPayload planetState;         //!< buffer of planet data
    SpicePlanetStateMsgPayload sunData;             //!< buffer of sun data
    EclipseMsgPayload eclipseInMsgData;             //!< buffer of eclipse data

    Eigen::Matrix3d dcm_PN; //!< Rotation matrix from inertial frame N to planet-centered to planet-fixed frame P
    Eigen::Vector3d r_PN_N; //!< [m] Planet to inertial frame origin vector.
    Eigen::Vector3d r_BP_P; //!< primary spacecraft relative to planet
    Eigen::Vector3d r_BN_N; //!< primary spacecraft relative to inertial
    double zScale;          //!< ration of rEquator over rPolar, used for affine scaling to turn ellipsoid to sphere
    Eigen::Vector3d r_HN_N; //!< [m] sun position in inertial frame
};

#endif /* GroundLocation */
