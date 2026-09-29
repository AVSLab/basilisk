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

import os
import pytest

from Basilisk import hasBuildFeature

mujocoEnabled = hasBuildFeature("mujoco")
pytestmark = pytest.mark.skipif(
    not mujocoEnabled,
    reason="Requires Basilisk built with --mujoco True",
)
if mujocoEnabled:
    from Basilisk.simulation import mujoco

from Basilisk.architecture import sysModel
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

import numpy as np
import pytest

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{TEST_FOLDER}/sat_hub_only.xml"

ROTATED_SITE_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
      <site name="rotated" quat="0.7071067811865476 0 0 0.7071067811865476"/>
    </body>
  </worldbody>
</mujoco>
"""

SLIDER_SITE_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="slider">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.1" mass="1"/>
      <site name="tip"/>
    </body>
  </worldbody>
</mujoco>
"""


class PublicationProbe(sysModel.SysModel):
    """Record site-message header state from within integration stages."""

    def __init__(self, reader):
        super().__init__()
        self.reader = reader
        self.samples = []

    def UpdateState(self, CurrentSimNanos):
        """Record whether the site message was written at this stage time."""
        is_written = self.reader.isWritten()
        write_time = self.reader.timeWritten() if is_written else None
        self.samples.append((CurrentSimNanos, is_written, write_time))


class DirectJointStateMutator(sysModel.SysModel):
    """Mutate joint records without using scene-aware joint setters."""

    def __init__(self, joint):
        super().__init__()
        self.joint = joint

    def UpdateState(self, CurrentSimNanos):
        self.joint.getPositionState().setState([[2.5]])
        self.joint.getVelocityState().setState([[-0.75]])


class SiteStateProbe(sysModel.SysModel):
    def __init__(self, site):
        super().__init__()
        self.site = site
        self.samples = []

    def UpdateState(self, CurrentSimNanos):
        state = self.site.stateOutMsg.read()
        self.samples.append(
            (state.r_BN_N[0], state.v_BN_N[0])
        )


def test_siteVelocity():
    """Test the velocity output of sites."""

    dt = 1 # s
    tf = 30 # s

    # Create sim, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create scene
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Set initial conditions
    initialPosition = [1.5,-1,0.75]
    initialVelocity = [5e-3,-5e-3,5e-3]
    initialAttitude = [0.15,-0.1,0.1]
    initialAttitudeRate = [0.01,-0.01,0.005]

    # Save body state through recorder
    bodyStateRecorder = scene.getBody("hub").getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    # Initialize sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))

    # Set initial states
    scene.getBody("hub").setPosition(initialPosition)
    scene.getBody("hub").setVelocity(initialVelocity)
    scene.getBody("hub").setAttitude(initialAttitude)
    scene.getBody("hub").setAttitudeRate(initialAttitudeRate)

    # run sim
    scSim.ExecuteSimulation()

    # validate the simulation output values
    v_BN_N = bodyStateRecorder.v_BN_N[-1]
    omega_BN_B = bodyStateRecorder.omega_BN_B[-1]

    np.testing.assert_allclose(v_BN_N, np.array(initialVelocity), rtol=1e-8,
                               err_msg=f"Linear velocity {v_BN_N} not close to expected {initialVelocity}")
    np.testing.assert_allclose(omega_BN_B, np.array(initialAttitudeRate), rtol=1e-8,
                               err_msg=f"Angular velocity {omega_BN_B} not close to expected {initialAttitudeRate}")


def test_rotated_site_angular_velocity():
    """A rotated site reports global angular velocity in its local frame."""
    dt = 0.1  # [s]
    sc_sim = SimulationBaseClass.SimBaseClass()
    process = sc_sim.CreateNewProcess("test")
    process.addTask(sc_sim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene(ROTATED_SITE_XML)
    sc_sim.AddModelToTask("test", scene)
    sc_sim.InitializeSimulation()
    scene.getBody("hub").setAttitudeRate([0.1, 0.2, 0.3])
    sc_sim.ConfigureStopTime(macros.sec2nano(dt))
    sc_sim.ExecuteSimulation()

    state = scene.getSite("rotated").stateOutMsg.read()
    np.testing.assert_allclose(
        state.omega_BN_B,
        [0.2, -0.1, 0.3],
        atol=1e-14,
    )


def test_explicit_forward_detects_direct_joint_state_mutation():
    scene = mujoco.MJScene(SLIDER_SITE_XML)
    joint = scene.getBody("slider").getScalarJoint("slide")
    site = scene.getSite("tip")
    mutator = DirectJointStateMutator(joint)
    probe = SiteStateProbe(site)

    scene.AddModelToDynamicsTask(mutator, 100)
    scene.AddFwdKinematicsToDynamicsTask(50)
    scene.AddModelToDynamicsTask(probe, 0)
    scene.Reset(0)

    scene.integrateState(1)

    assert probe.samples
    np.testing.assert_allclose(
        probe.samples,
        np.tile([2.5, -0.75], (len(probe.samples), 1)),
        atol=1e-14,
    )


@pytest.mark.parametrize("extra_eom_call", [False, True])
def test_site_messages_publish_stage_and_final_states(extra_eom_call):
    """Joint-state integration preserves stage and final site publication."""
    dt = 0.1  # [s]
    sc_sim = SimulationBaseClass.SimBaseClass()
    process = sc_sim.CreateNewProcess("test")
    process.addTask(sc_sim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene(ROTATED_SITE_XML)
    scene.extraEoMCall = extra_eom_call
    sc_sim.AddModelToTask("test", scene)

    site = scene.getSite("rotated")
    reader = site.stateOutMsg.addSubscriber()
    probe = PublicationProbe(reader)
    scene.AddModelToDynamicsTask(probe, 0)

    sc_sim.InitializeSimulation()
    scene.getBody("hub").setPosition([2.0, 0.0, 0.0])  # [m]
    scene.getBody("hub").setVelocity([1.0, 0.0, 0.0])  # [m/s]
    sc_sim.ConfigureStopTime(macros.sec2nano(dt))
    sc_sim.ExecuteSimulation()

    positive_stage_samples = [
        sample for sample in probe.samples if sample[0] > 0
    ]
    assert positive_stage_samples
    assert all(
        is_written and write_time == stage_time
        for stage_time, is_written, write_time in positive_stage_samples
    )
    assert reader.isWritten()
    final_state = site.stateOutMsg.read()
    assert reader.timeWritten() == macros.sec2nano(dt)
    assert final_state.r_BN_N[0] == pytest.approx(2.0 + dt)


if __name__ == "__main__":
    test_siteVelocity()
