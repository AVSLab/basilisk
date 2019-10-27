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

/*! \addtogroup SimModelGroup
 * @{
 */

/*!  @brief Calculate the Solar Flux at a spacecraft position

 ## Module Purpose
 ### Executive Summary
    Take in a spacecraft position message and sun position message and calculates the solar flux [W/m2] at the spacecraft location.

 ### Module Assumptions and Limitations
    This model uses the solar flux value, SOLAR_FLUX_EARTH, and astronomical unit, AU, values from astroConstants.h

    This model assumes constant solar flux and a perfect inverse square fall-off of flux as one moves further from the sun.

    This model can take in one spacecraft position message and one sun position message.

 ### Message Connection Descriptions
    The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.
    Msg Variable Name | Msg Type | Description
    ------------------|----------|-------------
    sunPositionInMsgName | SpicePlanetStateSimMsg | This message is used to get the sun's position
    spacecraftStateInMsgName | SCPlusStatesSimMsg | This message is used to get the spacecraft's position
    solarFluxOutMsgName | SolarFluxSimMsg | This message is used to output the solar flux at the spacecraft's position

 ## Detailed Module Description
   Many physical effects from sun sensor outputs to spacecraft heating depend on the solar flux incident on a spacecraft.
   This module provides that flux given the sun and spacecraft positions.

   From the user perspective, this module should be instantiated when there is a desire to provide the solar flux at a
   spacecraft location.

   From the developer perspective, the use of this module in conjunction with the developer's other modules should be preferred
   over individual modules calculating the solar flux internally and applying it. Especially if eclipsing effects on
   the solar flux are desired, this module should be used in conjunction with EclipseEffect rather than other modules
   incorporating these calculations. This does a few things. First,
   it prevents code repetition. Second, it ensures consistency how solar flux values are calculated, avoiding multiple
   sources and hard-coding numbers. Third, it actually enables the use of an EclipseEffect module, without which
   each other module must apply the effect of eclipsing individually.

 ### Equations
    The flux is calculated by scaling the flux at 1 AU:

    \f$ F_{\mathrm{out}} = F_{\mathrm{Earth}} * \frac{\mathrm{AU}^2}{r_{\mathrm{Sun}}^2} \f$

    where \f$ r_{\mathrm{Sun}} \f$ is the distance between the spacecraft and the sun

 ## User Guide
    The user can only instantiate this module, change the i/o names, and add it to a task.
    The names below are only special in that they are useful defaults.
    ~~~~~~~{.py}
    from Basilisk.simulation import solarFlux
    from Basilisk.utilities import SimulationBaseClass()

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)

    sf = solarFlux.SolarFlux()
    sf.sunPositionInMsgName = "sun_planet_data"
    sf.spacecraftStateInMsgName = "inertial_state_output"
    sf.solarFluxOutMsgName = "solar_flux"
    sim.AddModelToTask(task.Name, sf)
    ~~~~~~~
 */
class SolarFlux: public SysModel {
public:
    SolarFlux(){};
    ~SolarFlux(){};
    
    void SelfInit() override;
    void CrossInit() override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    void writeMessages(uint64_t CurrentSimNanos);
    void readMessages();

public:
    std::string sunPositionInMsgName = "sun_planet_data";
    std::string spacecraftStateInMsgName = "inertial_state_output";
    std::string solarFluxOutMsgName = "solar_flux";
    int64_t sunPositionInMsgId = -1;
    int64_t spacecraftStateInMsgId = -1;
    int64_t solarFluxOutMsgId = -1;
private:
    double fluxAtSpacecraft;  //!< [W/m2]
    Eigen::Vector3d r_SN_N;  //!< [m] sun position
    Eigen::Vector3d r_ScN_N;  //!< [m] s/c position
};
/*! @} */