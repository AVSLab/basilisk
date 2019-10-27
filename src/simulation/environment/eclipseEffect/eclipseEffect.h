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

//#pragma once

#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/solarFluxSimMsg.h"

/*! \addtogroup SimModelGroup
* @{
*/

/*! @brief Apply the input eclipse shadow factor

 ## Module Purpose
 ### Executive Summary
    This module takes in a solar flux message and an eclipse message. It modifies the flux message data and outputs the
    modified values to the same flux message.

 ### Module Assumptions and Limitations
    This model is limited to a single pair of inputs and single output.
    It is assumed that the specific solar flux and eclipse messages are for the same spacecraft
    at the same time and position.

 ### Message Connection Descriptions
    The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.
    Msg Variable Name | Msg Type | Description
    ------------------|----------|-------------
    solarFluxInOutMsgName| SolarFluxSimMsg | This message is read to get the non-eclipse-modified solar flux value. the modified flux value is written to the same message
    eclpseInMsgName |  EclipseSimMsg | This message is used to get the eclipse message which contains a shadow factor to modify the solar flux.

 ## Detailed Module Description
   Use of this module should be preferred over other modules individually applying eclipse conditions.

 ### Equations
    The distance is the norm of the vector from the spacecraft to the sun

    \f$ F_{out} = F_{in} * f_{eclipse} \f$

 ## User Guide
    The user can only instantiate this module, change the i/o names, and add it to a task.
    The names below are only special in that they are useful defaults.
    ~~~~~~~{.py}
    from Basilisk.simulation import eclipseEffect
    from Basilisk.utilities import SimulationBaseClass()
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)
    eff = eclipseEffect.EclipseEffect()
    eff.solarFluxInOutMsgName = "solar_flux";
    eff.eclipseInMsgName = "eclipse_data_0";
    sim.AddModelToTask(task.Name, eff)
    ~~~~~~~

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
    std::string solarFluxInOutMsgName = "solar_flux";
    std::string eclipseInMsgName = "eclipse_data_0";

private:
    double fluxIn;
    double fluxOut;
    double eclipseFactor;
    int64_t solarFluxInOutMsgId = -1;
    int64_t eclipseInMsgId = -1;
};
/*! @} */
>>>>>>> [BSK-1514] fixed doxygen issue, the TeX command was missing a \ symbol that made nothing work in the end
