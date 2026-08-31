# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import numpy as np
import pytest

from Basilisk import hasBuildFeature
from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import twoHingeDamper
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


@pytest.mark.parametrize(
    "theta,phiDot,thetaDot,dampingCoeff",
    [
        (0.6, 0.4, -0.3, 2.5),
        (0.0, -0.2, 0.5, 1.0),
        (0.5*np.pi, 0.4, 0.3, 2.0),
        (0.3, 0.0, 0.0, 0.0),
    ],
)
def test_two_hinge_damper_cartesian_law(
        theta, phiDot, thetaDot, dampingCoeff):
    r"""Verify the generalized torques from isotropic Cartesian bob damping.

    For the two-hinge rod parameterization, the generalized damping torques
    are

    .. math::

        Q_\phi=-c\cos^2\theta\,\dot\phi,\qquad
        Q_\theta=-c\dot\theta.
    """
    timeStep = 0.1  # [s]
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    process.addTask(
        simulation.CreateNewTask("testTask", macros.sec2nano(timeStep))
    )

    damper = twoHingeDamper.TwoHingeDamper()
    damper.dampingCoeff = dampingCoeff

    def jointStateMessage(value):
        return messaging.ScalarJointStateMsg().write(
            messaging.ScalarJointStateMsgPayload(state=value)
        )

    thetaMsg = jointStateMessage(theta)
    phiDotMsg = jointStateMessage(phiDot)
    thetaDotMsg = jointStateMessage(thetaDot)
    damper.thetaInMsg.subscribeTo(thetaMsg)
    damper.phiDotInMsg.subscribeTo(phiDotMsg)
    damper.thetaDotInMsg.subscribeTo(thetaDotMsg)

    phiRecorder = damper.phiTorqueOutMsg.recorder()
    thetaRecorder = damper.thetaTorqueOutMsg.recorder()
    simulation.AddModelToTask("testTask", damper)
    simulation.AddModelToTask("testTask", phiRecorder)
    simulation.AddModelToTask("testTask", thetaRecorder)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(timeStep))
    simulation.ExecuteSimulation()

    assert phiRecorder.input[-1] == pytest.approx(
        -dampingCoeff*np.cos(theta)**2*phiDot
    )
    assert thetaRecorder.input[-1] == pytest.approx(
        -dampingCoeff*thetaDot
    )


@pytest.mark.parametrize(
    "missingInput,dampingCoeff,expectedMessage",
    [
        ("thetaInMsg", 1.0, "thetaInMsg is not linked"),
        ("phiDotInMsg", 1.0, "phiDotInMsg is not linked"),
        ("thetaDotInMsg", 1.0, "thetaDotInMsg is not linked"),
        (None, -1.0, "dampingCoeff must be finite and nonnegative"),
        (None, np.nan, "dampingCoeff must be finite and nonnegative"),
    ],
)
def test_two_hinge_damper_reset_validation(
        missingInput, dampingCoeff, expectedMessage):
    """Reject missing joint-state inputs and invalid damping coefficients."""
    damper = twoHingeDamper.TwoHingeDamper()
    damper.dampingCoeff = dampingCoeff
    inputs = {
        "thetaInMsg": messaging.ScalarJointStateMsg(),
        "phiDotInMsg": messaging.ScalarJointStateMsg(),
        "thetaDotInMsg": messaging.ScalarJointStateMsg(),
    }
    for name, message in inputs.items():
        if name != missingInput:
            getattr(damper, name).subscribeTo(message)

    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    process.addTask(
        simulation.CreateNewTask("testTask", macros.sec2nano(0.1))
    )
    simulation.AddModelToTask("testTask", damper)
    with pytest.raises(BasiliskError, match=expectedMessage):
        simulation.InitializeSimulation()


@pytest.mark.skipif(
    not hasBuildFeature("mujoco"),
    reason="Basilisk was built without MuJoCo",
)
def test_two_hinge_damper_apply_to_enforces_joint_contract():
    """Accept same-scene hinges and reject slides, ball joints, and mixed scenes."""
    from Basilisk.simulation import mujoco

    def makeScene(phiType="hinge", thetaType="hinge"):
        return mujoco.MJScene(
            f"""
<mujoco>
  <worldbody>
    <body name="pendulum">
      <joint name="phi" type="{phiType}" axis="1 0 0"/>
      <joint name="theta" type="{thetaType}" axis="0 1 0"/>
      <geom type="sphere" size="0.1" mass="1"/>
      <body name="ballBody">
        <joint name="ball" type="ball"/>
        <geom type="sphere" size="0.1" mass="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
        )

    hingeScene = makeScene()
    phiHinge = hingeScene.getBody("pendulum").getScalarJoint("phi")
    thetaHinge = hingeScene.getBody("pendulum").getScalarJoint("theta")

    with pytest.raises(ValueError, match="distinct hinge joints"):
        twoHingeDamper.TwoHingeDamper().applyTo(phiHinge, phiHinge)

    # The rejected call must not have created an actuator or otherwise prevent
    # the same scene from accepting the valid two-joint attachment.
    actuators = twoHingeDamper.TwoHingeDamper().applyTo(
        phiHinge, thetaHinge)
    assert len(actuators) == 2

    for phiType, thetaType, expectedName in (
            ("slide", "hinge", "phiJoint"),
            ("hinge", "slide", "thetaJoint")):
        slideScene = makeScene(phiType, thetaType)
        phiJoint = slideScene.getBody("pendulum").getScalarJoint("phi")
        thetaJoint = slideScene.getBody("pendulum").getScalarJoint("theta")
        with pytest.raises(ValueError, match=expectedName+" must be a hinge"):
            twoHingeDamper.TwoHingeDamper().applyTo(phiJoint, thetaJoint)

    ballJoint = hingeScene.getBody("ballBody").getBallJoint()
    with pytest.raises(TypeError, match="phiJoint must be an MJScalarJoint"):
        twoHingeDamper.TwoHingeDamper().applyTo(ballJoint, thetaHinge)

    otherScene = makeScene()
    otherTheta = otherScene.getBody("pendulum").getScalarJoint("theta")
    with pytest.raises(ValueError, match="same scene"):
        twoHingeDamper.TwoHingeDamper().applyTo(phiHinge, otherTheta)
