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
            """Register position and velocity states for the oscillator."""
            self.posState = registerer.registerState(3, 1, "pos")
            self.velState = registerer.registerState(3, 1, "vel")
            self.posState.setState([[0], [0], [0]])
            self.velState.setState([[0], [0], [0]])

        def Reset(self, CurrentSimNanos=0):
            """Refresh the compiled state and restore the stored frequency."""
            super().Reset(CurrentSimNanos)
            # memory fields must be set before Reset, but we can re-set here
            # because Reset() propagates the updated value into the cfunc buffer.
            self.memory.omega = self._omega

        @staticmethod
        def UpdateStateImpl(posState, posStateDeriv, velState, velStateDeriv, memory):
            """Write the first-order oscillator dynamics into the derivative buffers."""
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
        """Minimal scalar state model used to verify generated state names."""

        def registerStates(self, registerer):
            """Register a single scalar state named ``x``."""
            self.xState = registerer.registerState(1, 1, "x")
            self.xState.setState([[0]])

        def Reset(self, CurrentSimNanos=0):
            """Delegate to the base reset to compile and bind state pointers."""
            super().Reset(CurrentSimNanos)

        @staticmethod
        def UpdateStateImpl(xState, xStateDeriv):
            """Set a constant unit derivative for the scalar state."""
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


@pytest.mark.skipif(not couldImport, reason="Compiled Basilisk without --mujoco or numba not available")
def test_StatefulNumbaDiffusion():
    """Tests that xStateDiffusionN correctly wires to the StateData diffusion buffer.

    We run a scalar Ornstein-Uhlenbeck process:
        dx = theta*(mu - x)*dt + sigma*dW

    once using StatefulNumbaModel (diffusion written via xStateDiffusion0 in
    UpdateStateImpl) and once using StatefulSysModel (diffusion written via
    setDiffusion in Python UpdateState).  With the same integrator seed both
    simulations must produce bit-identical final states.
    """
    from Basilisk.simulation import svIntegrators
    from Basilisk.simulation import StatefulSysModel

    theta = 0.5
    mu    = 0.0
    sigma = 0.1
    x0    = 1.0
    dt    = 0.1
    tf    = 1.0
    seed  = 42

    def _make_sim():
        scSim = SimulationBaseClass.SimBaseClass()
        scSim.CreateNewProcess("test").addTask(
            scSim.CreateNewTask("test", macros.sec2nano(dt))
        )
        scene = mujoco.MJScene("<mujoco/>")
        scSim.AddModelToTask("test", scene)
        intg = svIntegrators.svStochasticIntegratorMayurama(scene)
        intg.setRNGSeed(seed)
        scene.setIntegrator(intg)
        # intg must be returned: scene stores only a raw C++ pointer, so Python
        # would GC the integrator before the simulation runs without this.
        return scSim, scene, intg

    # ---- Numba model: diffusion written via xStateDiffusion0 in cfunc ----
    class OuNumba(StatefulNumbaModel.StatefulNumbaModel):
        """Stateful Numba implementation of the scalar OU process."""

        def registerStates(self, registerer):
            """Register the scalar stochastic state and one noise source."""
            self.xState = registerer.registerState(1, 1, "x")
            self.xState.setNumNoiseSources(1)

        @staticmethod
        def UpdateStateImpl(xState, xStateDeriv, xStateDiffusion0, memory):
            """Write the OU drift and diffusion terms into the state buffers."""
            xStateDeriv[0, 0]      = memory.theta * (memory.mu - xState[0, 0])
            xStateDiffusion0[0, 0] = memory.sigma

    scSimA, sceneA, intgA = _make_sim()
    modelA = OuNumba()
    modelA.ModelTag     = "ou_numba"
    modelA.memory.theta = theta
    modelA.memory.mu    = mu
    modelA.memory.sigma = sigma
    sceneA.AddModelToDynamicsTask(modelA)
    sceneA.AddModelToDiffusionDynamicsTask(modelA)
    scSimA.InitializeSimulation()
    modelA.xState.setState([[x0]])
    scSimA.ConfigureStopTime(macros.sec2nano(tf))
    scSimA.ExecuteSimulation()
    xNumba = np.array(modelA.xState.getState())[0, 0]

    # ---- Reference: plain StatefulSysModel using setDiffusion in Python ----
    class OuStateful(StatefulSysModel.StatefulSysModel):
        """Reference Python StatefulSysModel implementation of the OU process."""

        def registerStates(self, registerer):
            """Register the scalar stochastic state and one noise source."""
            self.xState = registerer.registerState(1, 1, "x")
            self.xState.setNumNoiseSources(1)

        def UpdateState(self, CurrentSimNanos):
            """Update the OU drift and diffusion using the Python API."""
            x = np.array(self.xState.getState())[0, 0]
            self.xState.setDerivative([[theta * (mu - x)]])
            self.xState.setDiffusion([[sigma]], index=0)

    scSimB, sceneB, intgB = _make_sim()
    modelB = OuStateful()
    modelB.ModelTag = "ou_stateful"
    sceneB.AddModelToDynamicsTask(modelB)
    sceneB.AddModelToDiffusionDynamicsTask(modelB)
    scSimB.InitializeSimulation()
    modelB.xState.setState([[x0]])
    scSimB.ConfigureStopTime(macros.sec2nano(tf))
    scSimB.ExecuteSimulation()
    xStateful = np.array(modelB.xState.getState())[0, 0]

    np.testing.assert_allclose(xNumba, xStateful, atol=1e-14, rtol=0)


if __name__ == "__main__":
    test_StatefulNumbaDiffusion()
    if True:
        test_StatefulNumbaOscillator()
        test_StatefulNumbaStateNames()
        test_StatefulNumbaDiffusion()
    else:
        pytest.main([__file__])
