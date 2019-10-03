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

/*! \addtogroup SimModelGroup
 * @{
 */

/*! @brief Represents a location on a planetary body and computes access from that location to specified spacecraft.

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
    void addSpacecraftToModel(std::string tmpScMsgName);
    void setGroundLocation(double lat, double longitude, double alt);
    
private:
    void updateInertialPositions();
    void computeAccess();

public:
    double planetRadius; //! [m] Planet radius in meters.
    double minimumElevation; //! [deg] minimum elevation above the local horizon needed to see a spacecraft; defaults to 10 degrees
    std::vector<std::string> scStateInMsgNames;
    std::string planetInMsgName;
    std::vector<std::string> accessOutMsgNames;
    Eigen::Vector3d r_LP_P_init; //! [m] Initial position of the location in planet-centric coordinates; can also be set using setGroundLocation.

private:
    std::vector<int64_t> scPositionInMsgIds;
    std::vector<int64_t> accessOutMsgIds;
    int64_t planetInMsgId;
    Eigen::Vector3d r_PN_N; //! [m]Planet to inertial frame origin vector.
    Eigen::Vector3d r_LP_P; //! [m] Location to planet origin vector.
    Eigen::Vector3d r_LP_N; //! [m] Location to planet origin vector in inertial coordinates.
};

/*! @} */

#endif /* EXPONENTIAL_ATMOSPHERE_H */
