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

import os
import pytest

try:
    from Basilisk.simulation import mujoco

    couldImportMujoco = True
except:
    couldImportMujoco = False

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging

import numpy as np
import matplotlib.pyplot as plt


TEST_FOLDER = os.path.dirname(__file__)
XML_PATH_ORIGINAL = f"{TEST_FOLDER}/test_sat_2.xml"
XML_PATH_CHANGED_MASS = f"{TEST_FOLDER}/test_sat_2_changed_mass.xml"
XML_PATH_BALL = f"{TEST_FOLDER}/test_ball.xml"


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_continuouslyChangingMass(showPlots: bool = False):
    """A ball with mass ``m`` is at rest, then accelerated with constant thrust T=100
    for 25 units of time in the positive x-axis direction. The initial mass ``m_0`` is that
    of a sphere of radius 1 and density 1000. The mass is constantly decreasing, with a slope
    of -100 units of mass per unit of time. The evolution of the velocity is given by the ODE:

        dm/dt = -100
        dv/dt = 100/m

    which has solution family:

        m(t) = c_1 - 100*t
        v(t) = c_2 - log(100*t - c_1)

    to meet the initial conditions, ``c_1`` must be the initial mass ``m_0``, while ``c_2`` is
    ``log(-m_0)``. Solving for v(t):

        v(t) = log(m_0) - log(m_0 - 100t)

    Integrating once more gives the position (with x(0) = 0):

        x(t) = t + (m_0 - 100t)/100 * log((m_0 - 100t)/m_0)

    In this function we run this scenario in MuJoCo to check that simulation with
    variable mass returns the same as the analytical solution of the velocity and
    position.
    """

    dt = 1 # s
    tf = 25 # s

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene.fromFile(XML_PATH_BALL)
    # With extraEoMCall the final message is written inside equationsOfMotion, so
    # the recorded position reflects the mid-step mj_setConst qpos state directly.
    scene.extraEoMCall = True
    scSim.AddModelToTask("test", scene)

    actPayload = messaging.SingleActuatorMsgPayload()
    actPayload.input = 100 # N
    actMsg = messaging.SingleActuatorMsg()
    actMsg.write(actPayload)

    scene.getSingleActuator("ball").actuatorInMsg.subscribeTo(actMsg)

    massDepletionRate = 100 # kg/s

    mDotPayload = messaging.SCMassPropsMsgPayload()
    mDotPayload.massSC = -massDepletionRate # kg/s
    mDotMsg = messaging.SCMassPropsMsg()
    mDotMsg.write(mDotPayload)

    ballBody = scene.getBody("ball")
    ballBody.derivativeMassPropertiesInMsg.subscribeTo(mDotMsg)

    massRec = ballBody.massPropertiesOutMsg.recorder()
    posRec = ballBody.getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", massRec)
    scSim.AddModelToTask("test", posRec)

    scSim.InitializeSimulation()

    # initial position should not matter
    ballBody.setPosition([0, 1, 0])

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    m_0 = massRec.massSC[0] # kg
    t = macros.NANO2SEC * massRec.times() # s
    m = m_0 - massDepletionRate * t # kg
    expected_v = np.log(m_0) - np.log(m) # m/s
    expected_x = t + m / massDepletionRate * np.log(m / m_0) # m

    if showPlots:
        plt.plot(massRec.times() * macros.NANO2SEC, massRec.massSC)
        plt.xlabel("Time [s]")
        plt.ylabel("Mass [kg]")

        plt.figure()
        plt.plot(
            posRec.times() * macros.NANO2SEC, posRec.v_BN_N[:, 0], label="Simulated"
        )
        plt.plot(
            posRec.times() * macros.NANO2SEC, expected_v, "k--", label="Analytical"
        )
        plt.legend()
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity [m/s]")

        plt.figure()
        plt.plot(
            posRec.times() * macros.NANO2SEC, posRec.r_BN_N[:, 0], label="Simulated"
        )
        plt.plot(
            posRec.times() * macros.NANO2SEC, expected_x, "k--", label="Analytical"
        )
        plt.legend()
        plt.xlabel("Time [s]")
        plt.ylabel("Position [m]")
        plt.show()

    assert expected_v[-1] == pytest.approx(posRec.v_BN_N[-1, 0])

    # The position catches mj_setConst leaving qpos at the reference pose
    # (velocity alone integrates correctly even then).
    assert expected_x[-1] == pytest.approx(posRec.r_BN_N[-1, 0], rel=1e-3)


if __name__ == "__main__":
    if True:
        test_continuouslyChangingMass(True)

    else:
        pytest.main([__file__])
