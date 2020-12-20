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

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../../_GeneralModuleFiles/sys_model.h"

#include "msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "msgPayloadDefC/SCPlusStatesMsgPayload.h"
#include "msgPayloadDefC/AccessMsgPayload.h"
#include "msgPayloadDefC/GroundStateMsgPayload.h"
#include "messaging2/messaging2.h"

#include "../utilities/geodeticConversion.h"
#include "utilities/astroConstants.h"
#include "utilities/bskLogging.h"

/*! @brief ground location class */
class GroundLocation:  public SysModel {
public:
    GroundLocation();
    ~GroundLocation();
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);
    bool ReadMessages();
    void WriteMessages(uint64_t CurrentClock);
    void addSpacecraftToModel(Message<SCPlusStatesMsgPayload> *tmpScMsg);
    void specifyLocation(double lat, double longitude, double alt);
    
private:
    void updateInertialPositions();
    void computeAccess();

public:
    double planetRadius; //!< [m] Planet radius in meters.
    double minimumElevation; //!< [rad] minimum elevation above the local horizon needed to see a spacecraft; defaults to 10 degrees equivalent.
    double maximumRange; //!< [m] Maximum slant range to compute access for; defaults to -1, which represents no maximum range.

    ReadFunctor<SpicePlanetStateMsgPayload> planetInMsg;            //!< planet state input message
    Message<GroundStateMsgPayload> currentGroundStateOutMsg;    //!< ground location output message
    std::vector<Message<AccessMsgPayload>> accessOutMsgs;           //!< vector of ground location access messages
    std::vector<ReadFunctor<SCPlusStatesMsgPayload>> scStateInMsgs; //!< vector of sc state input messages
    Eigen::Vector3d r_LP_P_Init; //!< [m] Initial position of the location in planet-centric coordinates; can also be set using setGroundLocation.
    BSKLogger bskLogger;         //!< -- BSK Logging

private:
    std::vector<AccessMsgPayload> accessMsgBuffer;                  //!< buffer of access output data
    std::vector<SCPlusStatesMsgPayload> scStatesBuffer;             //!< buffer of spacecraft states
    SpicePlanetStateMsgPayload planetState;                         //!< buffer of planet data
    GroundStateMsgPayload currentGroundStateBuffer;                 //!< buffer of ground station output data

    Eigen::Matrix3d dcm_LP; //!< Rotation matrix from planet-centered, planet-fixed frame P to site-local topographic (SEZ) frame L coordinates
    Eigen::Matrix3d dcm_PN; //!< Rotation matrix from inertial frame N to planet-centered to planet-fixed frame P 
    Eigen::Vector3d r_PN_N; //!< [m] Planet to inertial frame origin vector.
    Eigen::Vector3d r_LP_P; //!< [m] Ground Location to planet origin vector in planet frame coordinates.
    Eigen::Vector3d r_LP_N; //!< [m] Gound Location to planet origin vector in inertial coordinates.
    Eigen::Vector3d rhat_LP_N;//!< [-] Surface normal vector from the target location.
    Eigen::Vector3d r_LN_N;
    Eigen::Vector3d r_North_N; //!<[-] Inertial 3rd axis, defined internally as "North".
};


#endif /* GroundLocation */
