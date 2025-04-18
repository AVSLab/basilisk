#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import StatefulSysModel
    couldImportMujoco = True
except:
    couldImportMujoco = False

import pytest
import numpy as np

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_stateful():
    """Tests that ``StatefulSysModel`` works as expected.

    We use a simple ``StatefulSysModel`` with a single state. We check
    that said state is registered with the expected name and that its
    value evolves as we would expect.
    """

    # Declared inside, since StatefulSysModel may be undefined if not running with mujoco
    class ExponentialStateModel(StatefulSysModel.StatefulSysModel):
        """A simple model with one state, whose derivative is dx/dt = x*t."""

        def registerStates(self, registerer: StatefulSysModel.DynParamRegisterer):
            """Called once during InitializeSimulation"""
            self.xState = registerer.registerState(1, 1, "x")

        def UpdateState(self, CurrentSimNanos):
            """Called at every integrator step"""
            t = macros.NANO2SEC * CurrentSimNanos
            x = self.xState.getState()[0][0]
            self.xState.setDerivative( [[t*x]] )


    dt = 0.01 # s
    tf = 1 # s

    # Create sim, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>") # empty scene, no multi-body dynamics
    scSim.AddModelToTask("test", scene)

    expState = ExponentialStateModel()
    expState.ModelTag = "testModel"

    scene.AddModelToDynamicsTask(expState)

    # Run the sim
    scSim.InitializeSimulation()
    expState.xState.setState([[1]]) # initialize state to 1

    # Run for tf seconds
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    # Check that the state name has the model tag and ID prepended
    expected_name = f"{expState.ModelTag}_{expState.moduleID}_x"
    assert expState.xState.getName() == expected_name, f"{expState.xState.getName()} != {expected_name}"

    # The state follows dx/dt=x*t for x(0) = 1
    # So we expect x(tf=1) to be e^(tf^2/2)
    expected = np.exp( tf**2 / 2 )
    assert expState.xState.getState()[0][0] == pytest.approx(expected)

if __name__ == "__main__":
    if True:
        test_stateful()
    else:
        pytest.main([__file__])
