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

#pragma once

#include "_GeneralModuleFiles/sys_model.h"
#include <Eigen/Dense>
#include "../simulation/utilities/avsEigenMRP.h"


/*! \addtogroup SimModelGroup
 * @{
 */

/*!  @brief Calculate the unit heading vector to a planet in the spacecraft body frame

 ## Module Purpose
 ### Executive Summary
    Takes in a spacecraft position message and spice planet message and provides the heading to the planet in the s/c body frame.

 ### Module Assumptions and Limitations
    This model needs to be provided spacecraft body frame and planet positions in the inertial frame.

    This model captures only a static heading vector, not the rate of said heading vector.

    This model is limited to finding the body heading to a planet (not other spacecraft, etc).

 ### Message Connection Descriptions
    The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.
    Msg Variable Name | Msg Type | Description
    ------------------|----------|-------------
    planetPositionInMsgName | SpicePlanetStateSimMsg | This message is used to get the planet's position
    spacecraftStateInMsgName | SCPlusStatesSimMsg | This message is used to get the spacecraft's position and attitude MRP
    planetHeadingOutMsgName | BodyHeadingSimMsg | This message is used to output the unit heading vector to the planet in the spacecraft's body frame

 ## Detailed Module Description
   Many physical effects and spacecraft controls depend on the heading vector to a planet. Generally, it is useful for this vector to be given in spacecraft body frame components. Examples of such models in Basilisk that can use this data are course sun sensors (sun heading), and various control setups to point to a planet (planet heading). Because this information is so widely useful, it ought to be implemented in a single location to reduce computation and ensure consistency.

 ### Design Philosophy
   Overall, this module was created because many other modules need this information.

 ### Equations
    The heading is calculated by using the spacecraft and planet positions and spacecraft body frame attitude with respect to the inertial frame.

    First the inertial vector to the planet is calculated:

    \f$ ^N\bar{r}_{\mathrm{PS}} = ^N\bar{r}_{\mathrm{PN}} = ^N\bar{r}_{\mathrm{SN}} \f$

    Then, it is converted to the body frame and normalized

    \f$ ^B\bar{r}_{\mathrm{PS}} = \mathrm{DCM(\bar{\sigma}_{BN})} ^N \bar{r}_{\mathrm{PS}} \f$

    \f$ ^B \hat{\bar{r}}_{\mathrm{PS}} = \frac{^B\bar{r}_{\mathrm{PS}}}{||^B\bar{r}_{\mathrm{PS}}||} \f$

 ## User Guide
    The user can only instantiate this module, change the i/o names, and add it to a task.
    The names below are only special in that they are useful defaults.  Only the spacecraftStateInMsgName is actually a default.
    ~~~~~~~{.py}
    from Basilisk.simulation import planetHeading
    from Basilisk.utilities import SimulationBaseClass()

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)

    ph = planetHeading.PlanetHeading()
    ph.planetPositionInMsgName = "earth_planet_data"
    ph.spacecraftStateInMsgName = "inertial_state_output"
    ph.planetHeadingOutMsgName = "planet_heading"
    sim.AddModelToTask(task.Name, ph)
    ~~~~~~~
 */

class PlanetHeading: public SysModel {
public:
    PlanetHeading();
    ~PlanetHeading(){};
    
    void SelfInit() override;
    void CrossInit() override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    void Reset(uint64_t CurrentSimNanos) override;
    void writeMessages(uint64_t CurrentSimNanos);
    void readMessages();

public:
    std::string planetPositionInMsgName;
    std::string spacecraftStateInMsgName;
    std::string planetHeadingOutMsgName;

private:
    Eigen::Vector3d r_PN_N;  //!< [m] planet position
    Eigen::Vector3d r_BN_N;  //!< [m] s/c position
    Eigen::Vector3d rHat_PB_B;  //!< [] planet heading in s/c body frame (unit mag)
    Eigen::MRPd sigma_BN;  //!< [] s/c body att wrt inertial
    int64_t planetPositionInMsgId = -1;
    int64_t spacecraftStateInMsgId = -1;
    int64_t planetHeadingOutMsgId = -1;
};
/*! @} */