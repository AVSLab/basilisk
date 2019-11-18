#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


import numpy as np
import pytest

from Basilisk.simulation import solarFlux
from Basilisk.simulation.simMessages import SpicePlanetStateSimMsg
from Basilisk.simulation.simMessages import SCPlusStatesSimMsg
from Basilisk.simulation.simMessages import EclipseSimMsg

from Basilisk.utilities import orbitalMotion as om
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import SimulationBaseClass

@pytest.mark.parametrize("positionFactor, shadowFactor, eclipseMsgName, relTol", [(np.sqrt(2), 0.5, "eclipse_data_0", 1e-8), (np.sqrt(2), 0.5, "", 1e-8)])
def test_solarFlux(show_plots, positionFactor, shadowFactor, eclipseMsgName, relTol):
    """
    **Test Description**

    Test that solar flux is appropriately modified depending on spacecraft distance from the sun.
    To test this, the module is asked to write the solar flux at 1 AU. Then it is asked to write
    the flux at ``positionFactor*AU`` and the flux is checked to be ``positionFactor**2`` of that at 1 AU to within
    a relative tolerance of relTol.
    The application of the shadowFactor is also checked as a multiple of the un-shadowed flux.

    Args:
        positionFactor (float): positive,
            a factor by which to multiply the original s/c position to check flux at a new position
        shadowFactor (float): between 0 and 1,
            the eclipse factor by which to multiple the solar flux at a position
        eclipseMsgName (string): name of the eclipse message to read.
            It is an empty string if not to read a message
        relTol (float): positive, the relative tolerance to which the result is checked.
    """
    # Define BSKPrint message level
    msgLevel = SimulationBaseClass.sim_model.MSG_DEBUG

    sim = SimulationBaseClass.SimBaseClass(msgLevel)
    sim.terminateSimulation()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)

    sunPositionMessage = SpicePlanetStateSimMsg()
    sunPositionMessage.PositionVector = [0., 0., 0.]
    unitTestSupport.setMessage(sim.TotalSim, proc.Name, "sun_planet_data", sunPositionMessage, "SpicePlanetStateSimMsg")

    scPositionMessage = SCPlusStatesSimMsg()
    scPositionMessage.r_BN_N = [0., 0., om.AU*1000]
    unitTestSupport.setMessage(sim.TotalSim, proc.Name, "inertial_state_output", scPositionMessage, "SCPlusStatesSimMsg")

    eclipseMessage = EclipseSimMsg()
    eclipseMessage.shadowFactor = shadowFactor
    unitTestSupport.setMessage(sim.TotalSim, proc.Name, "eclipse_data_0", eclipseMessage, "EclipseSimMsg")

    sf = solarFlux.SolarFlux()
    sim.AddModelToTask(task.Name, sf)
    sf.eclipseInMsgName = eclipseMsgName
    sim.TotalSim.logThisMessage("solar_flux")
    sim.InitializeSimulationAndDiscover()
    sim.TotalSim.SingleStepProcesses()
    fluxOutEarth = sim.pullMessageLogData("solar_flux.flux")
    scPositionMessage.r_BN_N = [0., 0., positionFactor * om.AU*1000]
    sim.TotalSim.WriteMessageData("inertial_state_output",
                               scPositionMessage.getStructSize(),
                               0,
                               scPositionMessage)
    sf.Reset(1)
    sim.TotalSim.SingleStepProcesses()
    fluxOutFurther = sim.pullMessageLogData("solar_flux.flux")

    if len(eclipseMsgName) == 0:
        shadowFactor = 1.0

    assert fluxOutFurther[1][1] == pytest.approx(fluxOutEarth[0][1] / shadowFactor / (positionFactor**2) * shadowFactor, rel=relTol)


if __name__ == "__main__":
    test_solarFlux(False, np.sqrt(2.0), 0.5, "eclipse_data_0", 1e-8)
