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
    from Basilisk.simulation import StatefulNumbaModel
    couldImport = True
except Exception:
    couldImport = False

import pytest
import numpy as np


@pytest.mark.skipif(not couldImport, reason="Compiled Basilisk without --mujoco or numba not available")
def test_StatefulNumbaOscillator():
    """Tests that StatefulNumbaModel works as expected in MJScene.

    We use a simple harmonic oscillator with two 3×1 states (pos, vel) and
    a scalar memory field (omega).  The UpdateStateImpl is a Numba cfunc.

    posDot = vel
    velDot = -omega^2 * pos

    With pos(0) = [1, 0, 0] and vel(0) = [0, omega, 0] and omega = 2*pi
    the analytic solution at tf=0.5 s is:

        pos[0](tf) = cos(omega * tf)
        pos[1](tf) = sin(omega * tf)
        pos[2](tf) = 0

        vel[0](tf) = -omega * sin(omega * tf)
        vel[1](tf) =  omega * cos(omega * tf)
        vel[2](tf) = 0
    """

    # Declared inside the test so the import guard works correctly.
    class OscillatorModel(StatefulNumbaModel.StatefulNumbaModel):
        """3-D harmonic oscillator integrated by MJScene's RK4 solver."""

        def registerStates(self, registerer):
            self.posState = registerer.registerState(3, 1, "pos")
            self.velState = registerer.registerState(3, 1, "vel")

        def Reset(self, CurrentSimNanos=0):
            super().Reset(CurrentSimNanos)
            # memory fields must be set before Reset, but we can re-set here
            # because Reset() propagates the updated value into the cfunc buffer.
            self.memory.omega = self._omega

        @staticmethod
        def UpdateStateImpl(posState, posStateDeriv, velState, velStateDeriv, memory):
            posStateDeriv[:, 0] = velState[:, 0]
            velStateDeriv[:, 0] = -memory.omega ** 2 * posState[:, 0]

    omega = 2.0 * np.pi   # rad/s
    dt    = 1e-3           # s - small step for accuracy
    tf    = 0.5            # s

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("test", scene)

    osc = OscillatorModel()
    osc.ModelTag = "oscillator"
    osc.memory.omega = omega   # set before Reset is called
    osc._omega = omega         # stash for Reset()

    scene.AddModelToDynamicsTask(osc)

    scSim.InitializeSimulation()

    # Set initial conditions: pos = [1, 0, 0], vel = [0, omega, 0]
    osc.posState.setState([[1.0], [0.0], [0.0]])
    osc.velState.setState([[0.0], [omega], [0.0]])

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    pos = osc.posState.getState()
    vel = osc.velState.getState()

    expectedPos = np.array([[np.cos(omega * tf)],
                              [np.sin(omega * tf)],
                              [0.0]])
    expectedVel = np.array([[-omega * np.sin(omega * tf)],
                              [ omega * np.cos(omega * tf)],
                              [0.0]])

    # RK4 with dt=1ms should give ~1e-8 accuracy over 0.5 s
    np.testing.assert_allclose(pos, expectedPos, atol=1e-6)
    np.testing.assert_allclose(vel, expectedVel, atol=1e-5)


@pytest.mark.skipif(not couldImport, reason="Compiled Basilisk without --mujoco or numba not available")
def test_StatefulNumbaStateNames():
    """Tests that state names are prefixed with model tag and ID."""

    class ScalarModel(StatefulNumbaModel.StatefulNumbaModel):
        def registerStates(self, registerer):
            self.xState = registerer.registerState(1, 1, "x")

        def Reset(self, CurrentSimNanos=0):
            super().Reset(CurrentSimNanos)

        @staticmethod
        def UpdateStateImpl(xState, xStateDeriv):
            xStateDeriv[0, 0] = 1.0   # dx/dt = 1

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(0.1)))

    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("test", scene)

    model = ScalarModel()
    model.ModelTag = "myModel"
    scene.AddModelToDynamicsTask(model)

    scSim.InitializeSimulation()

    expectedName = f"{model.ModelTag}_{model.moduleID}_x"
    assert model.xState.getName() == expectedName, (
        f"State name {model.xState.getName()!r} != {expectedName!r}"
    )


if __name__ == "__main__":
    if True:
        test_StatefulNumbaOscillator()
        test_StatefulNumbaStateNames()
    else:
        pytest.main([__file__])
