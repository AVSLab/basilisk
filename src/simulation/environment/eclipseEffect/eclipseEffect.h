<<<<<<< develop
=======
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

/*! \addtogroup SimModelGroup
* @{
*/

/*! @brief Apply the input eclipse shadow factor

 ## Module Purpose
 ### Executive Summary
    This module takes in a SolarFluxSimMsg and an EclipseSimMsg. It modifies the flux message data and outputs the
    modified values to a new flux message

 ### Module Assumptions and Limitations
    This model is limited to a single pair of inputs and single output.
    It is assumed that the specified solar flux and eclipse messages are for the same spacecraft
    at the same time and position.

 ### Message Connection Descriptions
    The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.
    Msg Variable Name | Msg Type | Description
    ------------------|----------|-------------
    solarFluxInMsgName| SolarFluxSimMsg | This message is read to get the non-eclipse-modified solar flux value.
    eclpseInMsgName |  EclipseSimMsg | This message is used to get the eclipse message which contains a shadow factor to modify the solar flux.
    solarFluxOutMsgName | SolarFluxSimMsg | This message is used to write out the eclipse-modified solar flux value. the

 ## Detailed Module Description
   From the user perspective, this module should be instantiated when there is a desire to provide the solar flux at a
   spacecraft location that includes the effects of eclipse. I.e. the flux will go to zero when the spacecraft is in eclipse.
   Notably, some modules in Basilisk incorporate the eclipse effect internally, and this module should not be combined with those.

   From the developer perspective, the use of this module in conjunction with the developer's other modules should be preferred
   over individual modules reading eclipse messages and modifying the solar flux internally. This does a few things. First,
   it prevents code repetition. Second, it prevents branching in modules dependent on if an eclipse message was provided. Finally,
   it makes some intuitive physical sense as most physical effects do not "care" whether or not they are being eclipsed,
   but rather just operate on the incident solar flux, whatever it is.

 ### Equations
    The distance is the norm of the vector from the spacecraft to the sun

    \f$ F_{\mathrm{out}} = F_{\mathrm{in}} * f_{\mathrm{eclipse}} \f$

 ## User Guide
    The user can only instantiate this module, change the i/o names, and add it to a task.
    The names below are only special in that they are useful defaults and are, in fact, the defaults.
    ~~~~~~~{.py}
    from Basilisk.simulation import eclipseEffect
    from Basilisk.utilities import SimulationBaseClass()
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)
    eff = eclipseEffect.EclipseEffect()
    eff.solarFluxInMsgName = "solar_flux"
    eff.eclipseInMsgName = "eclipse_data_0"
    eff.solarFluxOutMsgName = "solar_flux_with_eclipse"
    sim.AddModelToTask(task.Name, eff)
    ~~~~~~~

    It will almost always be useful to use this module in conjunction with an Eclipse module and a SolarFlux module.

 */
class EclipseEffect: public SysModel {
public:
    EclipseEffect();
    ~EclipseEffect();
   
    void SelfInit();
    void CrossInit(); 
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentSimNanos);
    
public:
    std::string solarFluxInMsgName = "solar_flux";
    std::string eclipseInMsgName = "eclipse_data_0";
    std::string solarFluxOutMsgName = "solar_flux_with_eclipse";

private:
    double fluxIn;
    double fluxOut;
    double eclipseFactor;
    int64_t solarFluxInMsgId = -1;
    int64_t solarFluxOutMsgId = -1;
    int64_t eclipseInMsgId = -1;
};
/*! @} */
>>>>>>> [BSK-1514] fixed doxygen issue, the TeX command was missing a \ symbol that made nothing work in the end
