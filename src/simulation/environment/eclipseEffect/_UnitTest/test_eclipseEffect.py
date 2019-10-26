''' '''
'''
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

'''


from Basilisk.simulation import eclipseEffect
from Basilisk.simulation.simMessages import EclipseSimMsg
from Basilisk.simulation.simMessages import SolarFluxSimMsg
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import SimulationBaseClass


def test_eclipseEffect():
    """
    Test Description
    -----------------
    This test checks whether the eclipse effect appropriately modifies solar flux based on
    eclipse conditions.

    Test Variables
    ---------------
    A SolarFluxSimMsg is provided along with an EclipseSimMessage. The product of the
    solar flux value and eclipse shadowFactor value is checked to be
    accurate. The values were chosen arbitrarily and are hard-coded.

    """
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)

    fluxMsgIn = SolarFluxSimMsg()
    fluxMsgIn.flux = 2.0
    unitTestSupport.setMessage(sim.TotalSim, proc.Name, "solar_flux", fluxMsgIn, "SolarFluxSimMsg")

    eclipseMsgIn = EclipseSimMsg()
    eclipseMsgIn.shadowFactor = 0.5
    unitTestSupport.setMessage(sim.TotalSim, proc.Name, "eclipse_data_0", eclipseMsgIn, "EclipseSimMsg")

    eff = eclipseEffect.EclipseEffect()
    sim.AddModelToTask(task.Name, eff)
    sim.TotalSim.logThisMessage("solar_flux")
    sim.InitializeSimulationAndDiscover()
    sim.TotalSim.SingleStepProcesses()

    fluxOut = sim.pullMessageLogData("solar_flux.flux")
    assert fluxOut[0][1] == (fluxMsgIn.flux * eclipseMsgIn.shadowFactor)
    return


if __name__ == "__main__":
    test_eclipseEffect()
