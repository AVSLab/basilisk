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
import pytest
import numpy as np
import numpy.testing as npt
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.architecture import messaging
from Basilisk.utilities import macros

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJLinearTimeInvariantSystem
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("usePythonSubclass", [False, True])
def test_linearTimeInvariantSystemFirstOrder(usePythonSubclass: bool,
                                              showPlots: bool = False):
    """
    Unit test for LinearTimeInvariantSystem:

    - When usePythonSubclass == False:
        Uses the C++ SingleActuatorLTI subclass.

    - When usePythonSubclass == True:
        Uses a Python subclass of LinearTimeInvariantSystem that overrides
        readInput and writeOutput via directors.

    Both implement the same first order system:

        x_dot = -a x + a u,    y = x,   u = 1 (constant step)

    so that in continuous time:

        x(t) = 1 - exp(-a t)
    """
    # Simulation setup
    dt = 0.01  # s
    tf = 2.0   # s

    # First order dynamics parameter
    a = 1.0  # 1/s; time constant tau = 1/a = 1 s

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Empty MuJoCo scene: just a container for system models
    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("test", scene)

    # Constant input message u = 1.0
    cmdMsg = messaging.SingleActuatorMsg()
    cmdPayload = messaging.SingleActuatorMsgPayload()
    cmdPayload.input = 1.0
    cmdMsg.write(cmdPayload)

    if usePythonSubclass:
        # Python subclass of LinearTimeInvariantSystem
        class PyFirstOrderLTI(MJLinearTimeInvariantSystem.LinearTimeInvariantSystem):
            def __init__(self):
                super().__init__()
                # Input/output messaging
                self.inMsg = messaging.SingleActuatorMsgReader()
                self.outMsg = messaging.SingleActuatorMsg()

            def getInputSize(self) -> int:
                return 1

            def getOutputSize(self) -> int:
                return 1

            def readInput(self, CurrentSimNanos):
                # Return 1x1 vector from SingleActuatorMsg
                payload = self.inMsg()
                return np.array([[payload.input]])

            def writeOutput(self, CurrentSimNanos, y):
                # Write y[0] to SingleActuatorMsg
                payload = messaging.SingleActuatorMsgPayload()
                payload.input = y[0][0]
                self.outMsg.write(payload, self.moduleID, CurrentSimNanos)

        system = PyFirstOrderLTI()

    else:
        # C++ subclass: SingleActuatorLTI
        system = MJLinearTimeInvariantSystem.SingleActuatorLTI()

    # First order system: x_dot = -a x + a u, y = x
    A = np.array([[-a]])
    B = np.array([[a]])
    C = np.array([[1.0]])
    D = np.array([[0.0]])
    system.setA(A)
    system.setB(B)
    system.setC(C)
    system.setD(D)
    system.inMsg.subscribeTo(cmdMsg)

    # Add system to the scene dynamics
    scene.AddModelToDynamicsTask(system)

    # Recorder for the system output
    outRecorder = system.outMsg.recorder()
    scSim.AddModelToTask("test", outRecorder)

    # Initialize and run
    scSim.InitializeSimulation()

    # Basic size checks use the LinearTimeInvariantSystem API
    assert system.getInputSize() == 1
    assert system.getOutputSize() == 1
    assert system.getStateSize() == 1

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    # Extract data
    tNanos = outRecorder.times()
    yVals = np.asarray(outRecorder.input)  # SingleActuatorMsgPayload.input

    # Convert times to seconds for plotting / diagnostics
    t = tNanos * 1.0e-9

    if showPlots:
        fig, ax = plt.subplots()
        ax.plot(t, yVals, label="y(t)")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Output y")
        ax.grid(True)
        ax.legend()
        plt.show()

    # Continuous time target: x(t) = 1 - exp(-a t); y = x
    yTargetFinal = 1.0 - np.exp(-a * tf)

    # Use the last sample
    yFinal = float(yVals[-1])

    # Assert that final value is reasonably close to the continuous solution
    # Tolerance is relaxed a bit to allow for integration and discretization error
    npt.assert_allclose(yFinal, yTargetFinal, rtol=0.02, atol=1e-2)

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_linearTimeInvariantSystemSecondOrder(showPlots: bool = False):
    """
    Unit test for an LTI model configured as a second-order system.

    This test sets up a critically damped second-order linear time-invariant (LTI) system
    with natural frequency wn = 10 rad/s and damping ratio zeta = 1.0. The system is driven
    by a constant unit step input (u = 1.0). The test verifies:

    1. The final output value approaches 1.0 (steady-state response).
    2. The output does not overshoot beyond a small numerical margin (no overshoot for critical damping).
    3. The output time history closely matches the analytic step response for a critically damped system:
       y(t) = 1 - exp(-wn * t) * (1 + wn * t)
    """
    # Simulation setup
    dt = 0.01  # s
    tf = 2.0   # s

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Empty MuJoCo scene: just a container for system models
    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("test", scene)

    # Constant input message u = 1.0
    cmdMsg = messaging.SingleActuatorMsg()
    cmdPayload = messaging.SingleActuatorMsgPayload()
    cmdPayload.input = 1.0
    cmdMsg.write(cmdPayload)

    # Second order critically damped LTI: wn = 10 rad/s, zeta = 1, k = 1
    wn = 10.0
    zeta = 1.0

    system = MJLinearTimeInvariantSystem.SingleActuatorLTI()
    system.configureSecondOrder(wn, zeta)  # default k = 1
    system.inMsg.subscribeTo(cmdMsg)

    # Add system to the scene dynamics
    scene.AddModelToDynamicsTask(system)

    # Recorder for the system output
    outRecorder = system.outMsg.recorder()
    scSim.AddModelToTask("test", outRecorder)

    # Initialize and run
    scSim.InitializeSimulation()

    # Basic size checks use the LinearTimeInvariantSystem API
    assert system.getInputSize() == 1
    assert system.getOutputSize() == 1
    assert system.getStateSize() == 2

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    # Extract data
    tNanos = outRecorder.times()
    yVals = np.asarray(outRecorder.input)  # SingleActuatorMsgPayload.input

    # Convert times to seconds for plotting / diagnostics
    t = tNanos * 1.0e-9

    if showPlots:
        fig, ax = plt.subplots()
        ax.plot(t, cmdPayload.input * np.ones_like(t), "--", label="Input")
        ax.plot(t, yVals, label="Output")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("y(t) [-]")
        ax.grid(True)
        ax.legend()
        plt.show()

    # Analytic critically damped step response for:
    # G(s) = wn^2 / (s^2 + 2*wn*s + wn^2), unit step input
    # y(t) = 1 - exp(-wn t) * (1 + wn t)
    yAnalytic = 1.0 - np.exp(-wn * t) * (1.0 + wn * t)

    # 1) Final value near 1.0
    npt.assert_allclose(yVals[-1], 1.0, atol=0.005)

    # 2) No overshoot beyond a small numerical margin
    assert np.max(yVals) <= 1.01

    # 3) Shape close to analytic critically damped response
    npt.assert_allclose(
        yVals,
        yAnalytic,
        rtol=0.1,
        atol=0.05
    )


if __name__ == "__main__":
    assert couldImportMujoco
    test_linearTimeInvariantSystemFirstOrder(usePythonSubclass=False, showPlots=True)
    test_linearTimeInvariantSystemFirstOrder(usePythonSubclass=True, showPlots=True)
    test_linearTimeInvariantSystemSecondOrder(showPlots=True)
