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

/*! \defgroup groundInteractionSystem
  @brief This module simulates the existence of a locatio[n on a fixed point of a planet's surface, such as a science target,
 or communications array, and computes the access of a set of spacecraft relative to that target.

 ## Module Purpose

 ### Module Assumptions and Limitations
   This module assumes that it is affixed to a spherical body with a constant radius. Elevation constraints are computed assuming
   a conical field of view around the normal vector fom the body's surface at the location.

 ## Message Connection Descriptions
    The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.
    Msg Variable Name | Msg Type | Description
    ------------------|----------|-------------
    scStateInMsgNames | SCPlusStatesSimMsg |Provides inertial position of spacecraft. Set using the addSpacecraftToModel method.
    planetInMsgName   |  SPICEPlanetStateSimMsg | Provides inertial planet location and orientation.
    accessOutMsgNames | AccessSimMsg | List of access message names; defaults to `modelTag_scNumber_access`; created during addSpacecraftToModel calls.

 ## Detailed Module Description

 groundLocation handles the following behvaior:
 1. Body-fixed location representation: a single groundLocation instance represents one body-fixed location on a body, including
 translation and rotation due to the motion of that body as computed by a module that writes a SPICEPlanetStateSimMsg.
 2. Conversion of latitude, longitude, altitude coordinates to planet-centered, planet-fixed coordinates
 3. Computation of spacecraft visibility ("access") considering range and ground location field-of-view constraints
 4. Support for multiple spacecraft given one groundLocation instance

 ## User Guide
 A new instance of groundLocation, alongside necessary user-supplied parameters, can be created by calling:
~~~~~~~{.py}
    groundTarget = groundLocation.GroundLocation()
    groundTarget.ModelTag = "groundTarget"
    groundTarget.planetRadius = orbitalMotion.REQ_EARTH * 1000.
    groundTarget.maximumRange = 100e3 # Sets maximum range for visibility in meters
    groundTarget.minimumElevation = np.radians(10.) #   Sets necessary minimum elevation for visibility to 10 deg in radians
    groundTarget.specifyLocation(np.radians(0.), np.radians(0.), 0.) #  Sets location in latitude, longitude, altitude coordinates
    scSim.AddModelToTask(simTaskName, groundTarget)
~~~~~~~

 A groundLocation can be affixed to a specaific planet by setting its planetInMsgName attribute:
~~~~~~~{.py}
    groundTarget.planetInMsgName = planet_message_name
~~~~~~~

Spacecraft can be added to the model by calling:
 ~~~~~~~{.py}
    groundTarget.addSpacecraftToModel(sc1_message_name)
    groundTarget.addSpacecraftToModel(sc2_message_name)
    groundTarget.addSpacecraftToModel(sc3_message_name)

    #   Sim code
    sc1_access = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.hasAccess',range(1))
    sc1_slant = scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.slantRange',range(1))
    sc1_elevation =scSim.pullMessageLogData(groundTarget.accessOutMsgNames[0] + '.elevation',range(1))
~~~~~~~


 */

class GroundLocation:  public SysModel {
public:
    GroundLocation();
    ~GroundLocation();
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset();
    bool ReadMessages();
    void WriteMessages(uint64_t CurrentClock);
    void addSpacecraftToModel(std::string tmpScMsgName);
    void specifyLocation(double lat, double longitude, double alt);
    
private:
    void updateInertialPositions();
    void computeAccess();

public:
    double planetRadius; //! [m] Planet radius in meters.
    double minimumElevation; //! [rad] minimum elevation above the local horizon needed to see a spacecraft; defaults to 10 degrees
    double maximumRange; //! [m] Maximum slant range to compute access for; defaults to -1, which represents no maximum range.
    std::string planetInMsgName;
    std::vector<std::string> accessOutMsgNames;
    Eigen::Vector3d r_LP_P_Init; //! [m] Initial position of the location in planet-centric coordinates; can also be set using setGroundLocation.

private:
    uint64_t OutputBufferCount = 2;
    std::vector<std::string> scStateInMsgNames;
    std::vector<AccessSimMsg> accessMsgBuffer;
    std::vector<SCPlusStatesSimMsg> scStates;
    std::vector<int64_t> scStateInMsgIds;
    std::vector<int64_t> accessOutMsgIds;
    int64_t planetInMsgId;
    SpicePlanetStateSimMsg planetState;
    Eigen::Matrix3d C_PFPZ; //! Rotation matrix from planet-centered, planet-fixed into site-local topographic (SEZ) coordinates (i.e., the site location is [
    Eigen::Vector3d r_PN_N; //! [m]Planet to inertial frame origin vector.
    Eigen::Vector3d r_LP_P; //! [m] Location to planet origin vector.
    Eigen::Vector3d r_LP_N; //! [m] Location to planet origin vector in inertial coordinates.
    Eigen::Vector3d rhat_LP_N;//! [-] Surface normal vector from the target location.
    Eigen::Vector3d r_LN_N;
    Eigen::Vector3d r_North_N; //![-] Inertial 3rd axis, defined internally as "North".
};

/*! @} */

#endif /* GroundLocation */
