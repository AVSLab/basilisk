#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
import numpy.testing as npt

from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft
from Basilisk.simulation import stochasticAtmDensityStateEffector
from Basilisk.simulation import svIntegrators
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def test_stochasticAtmDensityStateEffector():
    """
    Verify that the module applies rho_out = rho_in * (1 + x)
    and passes localTemp through unchanged.
    """
    dt = 0.1
    tf = 0.2

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("process")
    task_name = "task"
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    sc_object = spacecraft.Spacecraft()
    sim.AddModelToTask(task_name, sc_object)

    integrator = svIntegrators.svStochasticIntegratorMayurama(sc_object)
    integrator.setRNGSeed(0)
    sc_object.setIntegrator(integrator)

    stoch = stochasticAtmDensityStateEffector.StochasticAtmDensityStateEffector()
    stoch.setStationaryStd(0.0)
    stoch.setTimeConstant(1.0e12)

    atmo_payload = messaging.AtmoPropsMsgPayload()
    atmo_payload.neutralDensity = 2.0
    atmo_payload.localTemp = 900.0
    atmo_msg = messaging.AtmoPropsMsg().write(atmo_payload)
    stoch.atmoDensInMsg.subscribeTo(atmo_msg)

    sc_object.addStateEffector(stoch)

    out_recorder = stoch.atmoDensOutMsg.recorder()
    sim.AddModelToTask(task_name, out_recorder)

    sim.InitializeSimulation()
    stoch.setStateValue(0.25)

    sim.ConfigureStopTime(macros.sec2nano(tf))
    sim.ExecuteSimulation()

    npt.assert_allclose(out_recorder.neutralDensity[-1], 2.0 * 1.25, atol=1e-12)
    npt.assert_allclose(out_recorder.localTemp[-1], 900.0, atol=1e-12)
    assert np.isclose(stoch.getStateValue(), 0.25, atol=1e-10)

if __name__ == "__main__":
    test_stochasticAtmDensityStateEffector()
