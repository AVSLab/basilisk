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
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/accessSimMsg.h"
#include "../utilities/geodeticConversion.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"

/*! \addtogroup SimModelGroup
 * @{
 */

/*! @brief Represents a location on a planetary body and computes access from that location to specified spacecraft.

This class is used to represent a point attached to the surface of a planetary body which computes the access of a set of spacecraft.
It considers both the relative position of a set of spacecraft to the location and a minimum elevation for the location (set to 10\f[\circ\f] by default.

To use this module, instantiate the class and provide it with a body-fixed location (in either lat/long/altitude, via the specifyLocation method, or in
planet-centered planet-fixed coordinates directly via the r_LP_P_Init attribute) and a planet position/attitude message (i.e., an instance of SpicePlanetStatesSimMsg);
to compute access, at least one scPlusStatesSimMsg must be added to the class using the addSpacecraftToModel method, which operates on the message's name. By default,
output messages are named "ModelTag_#_access", where "ModelTag" is the ModelTag associated with that groundLocation instance and # is the index of the spacecraft state
message as they were added to the class; the first spacecraft is "0", the second is "1", and so on.

 */
class GroundLocation:  public SysModel {
public:
    GroundLocation();
    ~GroundLocation();
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset();
    void ReadMessages();
    void WriteMessages(uint64_t CurrentClock);
    void addSpacecraftToModel(std::string tmpScMsgName);
    void specifyLocation(double lat, double longitude, double alt);
    
private:
    void updateInertialPositions();
    void computeAccess();

public:
    double planetRadius; //! [m] Planet radius in meters.
    double minimumElevation; //! [deg] minimum elevation above the local horizon needed to see a spacecraft; defaults to 10 degrees
    double maximumRange; //! [m] Maximum slant range to compute access for; defaults to -1, which represents no maximum range.
    std::string planetInMsgName;
    std::vector<std::string> accessOutMsgNames;
    Eigen::Vector3d r_LP_P_Init; //! [m] Initial position of the location in planet-centric coordinates; can also be set using setGroundLocation.

private:
    uint64_t OutputBufferCount = 2;
    std::vector<AccessSimMsg> accessMsgBuffer;
    std::vector<std::string> scStateInMsgNames;
    std::vector<SCPlusStatesSimMsg> scStates;
    std::vector<int64_t> scStateInMsgIds;
    std::vector<int64_t> accessOutMsgIds;
    int64_t planetInMsgId;
    SpicePlanetStateSimMsg planetState;
    Eigen::Vector3d r_PN_N; //! [m]Planet to inertial frame origin vector.
    Eigen::Vector3d r_LP_P; //! [m] Location to planet origin vector.
    Eigen::Vector3d r_LP_N; //! [m] Location to planet origin vector in inertial coordinates.
    Eigen::Vector3d rhat_LP_N;//! [-] Surface normal vector from the target location.
    Eigen::Vector3d r_LN_N;
};

/*! @} */

#endif /* GroundLocation */
