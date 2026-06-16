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

try:
    from Basilisk.architecture import messaging
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import pointMassGravityModel
    from Basilisk.simulation import svIntegrators
    from Basilisk.utilities import SimulationBaseClass
    from Basilisk.utilities import macros

    couldImportMujoco = True
except:
    couldImportMujoco = False

import pytest

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{TEST_FOLDER}/test_sat.xml"
FREE_BODY_FORCE_XML = """
<mujoco>
    <option gravity="0 0 0" integrator="Euler" timestep="1"/>
    <worldbody>
        <body name="ball">
            <freejoint/>
            <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
            <site name="center"/>
        </body>
    </worldbody>
    <custom>
        <text name="basilisk:forceactuator" data="push@center"/>
    </custom>
</mujoco>
"""


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_loading():
    """Tests that MJObject are created from the XML as expected"""

    scene = mujoco.MJScene.fromFile(XML_PATH)

    expected_bodies_and_sites = {
        "cube": ("cube_com", "cube_origin"),
        "panel_1": ("panel_1_com", "panel_1_origin", "test_site"),
        "panel_2": ("panel_2_com", "panel_2_origin"),
    }

    for body, sites in expected_bodies_and_sites.items():
        bodyObj = scene.getBody(body)
        for site in sites:
            bodyObj.getSite(site)

    expected_actuators = {
        "panel_1_elevation": scene.getSingleActuator,
        "panel_2_elevation": scene.getSingleActuator,
        "test_act_1": scene.getForceActuator,
        "test_act_2": scene.getForceActuator,
        "test_act_3": scene.getForceTorqueActuator,
        "test_act_4": scene.getForceTorqueActuator,
    }

    for actuator, getter in expected_actuators.items():
        getter(actuator)

    scene.Reset(0)  # this will compile the existing model

    # this will trigger changes in the model
    scene.addTorqueActuator("test_add_1", "panel_2_origin")
    scene.getBody("cube").addSite("test_add_1", [0, 0, 0])

    # Check that we can retrieve the newly added elements
    scene.getTorqueActuator("test_add_1")
    scene.getBody("cube").getSite("test_add_1")

    # Move time forward, which will trigger a re-compile (since
    # we added actuators), and check that the simulation runs ok
    scene.UpdateState(1)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_adaptive_free_joint_translation_tolerances_are_stage_independent():
    """Tests MJScene configures bulk relTol through the adaptive-integrator interface."""

    scene = mujoco.MJScene.fromFile(XML_PATH)
    cube_body = scene.getBody("cube")

    gravity = pointMassGravityModel.PointMassGravityModel()
    gravity.muBody = 1.0  # [m^3/s^2]
    cube_body.addGravitySource(gravity)

    integrator = svIntegrators.svIntegratorRKF78(scene)
    scene.setIntegrator(integrator)
    scene.Reset(0)

    assert integrator.getRelativeTolerance("mujocoQpos") == pytest.approx(0.0)
    assert integrator.getRelativeTolerance("mujocoQvel") == pytest.approx(0.0)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_add_gravity_source_preserves_force_actuator_acceleration():
    """Tests addGravitySource augments, rather than replaces, MuJoCo free-body qacc."""

    dt = 1.0  # [s]
    mass = 10.0  # [kg]
    mu = 8.0  # [m^3/s^2]
    radius = 2.0  # [m]
    force_accel = 3.0  # [m/s^2]

    sc_sim = SimulationBaseClass.SimBaseClass()
    process = sc_sim.CreateNewProcess("test")
    process.addTask(sc_sim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene(FREE_BODY_FORCE_XML)
    scene.setIntegrator(svIntegrators.svIntegratorEuler(scene))
    sc_sim.AddModelToTask("test", scene)

    ball_body = scene.getBody("ball")
    gravity = pointMassGravityModel.PointMassGravityModel()
    gravity.muBody = mu
    ball_body.addGravitySource(gravity)

    force_msg = messaging.ForceAtSiteMsg()
    force_msg.write(
        messaging.ForceAtSiteMsgPayload(force_S=[mass * force_accel, 0.0, 0.0])
    )
    scene.getForceActuator("push").forceInMsg.subscribeTo(force_msg)

    state_recorder = ball_body.getOrigin().stateOutMsg.recorder()
    sc_sim.AddModelToTask("test", state_recorder)

    sc_sim.InitializeSimulation()
    ball_body.setPosition([radius, 0.0, 0.0])
    ball_body.setVelocity([0.0, 0.0, 0.0])

    sc_sim.ConfigureStopTime(macros.sec2nano(dt))
    sc_sim.ExecuteSimulation()

    gravity_accel = -mu / radius**2  # [m/s^2]
    expected_velocity = (force_accel + gravity_accel) * dt  # [m/s]
    assert state_recorder.v_BN_N[-1, 0] == pytest.approx(expected_velocity)
    assert state_recorder.v_BN_N[-1, 1:] == pytest.approx([0.0, 0.0])


if __name__ == "__main__":
    if True:
        test_loading()
    else:
        pytest.main([__file__])
