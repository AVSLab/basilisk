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

import ctypes
import os
import subprocess
import sys
import textwrap

import numpy as np
import pytest

from Basilisk import hasBuildFeature

mujoco_enabled = hasBuildFeature("mujoco")
pytestmark = pytest.mark.skipif(
    not mujoco_enabled,
    reason="Requires Basilisk built with --mujoco True",
)

if mujoco_enabled:
    from Basilisk.simulation import _mujoco
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import svIntegrators

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk


TEST_FOLDER = os.path.dirname(__file__)
SATELLITE_XML = os.path.join(TEST_FOLDER, "test_sat.xml")

SPHERE_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="sphere" size="1" mass="10"/>
    </body>
  </worldbody>
</mujoco>
"""

HINGE_XML = """
<mujoco>
  <worldbody>
    <body name="link">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

BALL_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="rotor">
      <joint name="ball" type="ball"/>
      <geom type="sphere" size="1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

BALL_THEN_SLIDE_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="mixed">
      <joint name="ball" type="ball"/>
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

FREE_THEN_SLIDE_XML = """
<mujoco>
  <worldbody>
    <body name="hub">
      <freejoint name="free"/>
      <geom type="sphere" size="1" mass="10"/>
      <body name="link" pos="2 0 0">
        <joint name="slide" type="slide" axis="0 1 0"/>
        <geom type="sphere" size="0.5" mass="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

MIXED_JOINT_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="free_body">
      <freejoint name="free"/>
      <geom type="sphere" size="0.2" mass="10"/>
      <body name="ball_body" pos="1 0 0">
        <joint name="ball" type="ball"/>
        <geom type="sphere" size="0.15" mass="2"/>
        <body name="slide_body" pos="1 0 0">
          <joint name="slide" type="slide" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.mark.parametrize("high_order", [False, True])
def test_joint_owned_state_specs_and_repeated_epoch(high_order):
    scene = mujoco.MJScene.fromFile(SATELLITE_XML)
    scene.highOrderAttitudeIntegration = high_order
    scene.Reset(0)

    free_joint = scene.getBody("cube").getFreeJoint()
    translation = free_joint.getTranslationPositionState()
    attitude = free_joint.getAttitudeState()
    linear_velocity = free_joint.getTranslationVelocityState()
    angular_velocity = free_joint.getAttitudeRateState()

    assert translation.stateShape().rows == 3
    assert translation.derivativeShape().rows == 3
    assert attitude.stateShape().rows == 4
    assert attitude.derivativeShape().rows == (4 if high_order else 3)
    assert attitude.diffusionShape().rows == 3
    assert linear_velocity.stateShape().rows == 3
    assert angular_velocity.stateShape().rows == 3
    assert scene.dynManager.statesAreFinalized()

    handles = (
        int(translation.this),
        int(attitude.this),
        int(linear_velocity.this),
        int(angular_velocity.this),
    )
    scene.Reset(0)

    free_joint = scene.getBody("cube").getFreeJoint()
    assert (
        int(free_joint.getTranslationPositionState().this),
        int(free_joint.getAttitudeState().this),
        int(free_joint.getTranslationVelocityState().this),
        int(free_joint.getAttitudeRateState().this),
    ) == handles


def test_high_order_mode_is_finalized_policy_topology():
    scene = mujoco.MJScene(HINGE_XML)
    scene.Reset(0)

    with pytest.raises(
        RuntimeError,
        match="attitude integration mode cannot change",
    ):
        scene.highOrderAttitudeIntegration = True

    assert scene.highOrderAttitudeIntegration is False
    scene.Reset(0)


def test_runtime_equality_activation_survives_compatible_recompile():
    scene = mujoco.MJScene(HINGE_XML)
    scene.Reset(0)
    equality = (
        scene.getBody("link")
        .getScalarJoint("hinge")
        .getConstrainedEquality()
    )
    equality.setActive(True)
    assert equality.isActive()

    scene.getBody("link").addSite("new_site", [0.0, 0.0, 0.0])
    scene.Reset(0)

    assert equality.isActive()


def test_scalar_joint_metadata_uses_joint_id_after_free_joint():
    scene = mujoco.MJScene(FREE_THEN_SLIDE_XML)
    scene.Reset(0)

    joint = scene.getBody("link").getScalarJoint("slide")
    assert not joint.isHinge()
    np.testing.assert_allclose(
        np.asarray(joint.getAxis()).reshape(-1),
        [0.0, 1.0, 0.0],
    )


def test_internal_mujoco_derivative_methods_are_not_wrapped():
    scene = mujoco.MJScene(FREE_THEN_SLIDE_XML)
    scene.Reset(0)

    body = scene.getBody("link")
    joint = body.getScalarJoint("slide")
    assert not hasattr(body, "setJointPositionDerivativesFromMujoco")
    assert not hasattr(joint, "setPositionDerivativeFromMujoco")


def _native_integrate_position(xml_path, qpos, qvel, time_step):
    """Advance positions with the native MuJoCo library used by Basilisk."""
    library_path = _mujoco.__file__
    if sys.platform == "win32":
        # Windows extensions do not re-export symbols from their dependencies.
        # Conan copies the MuJoCo DLL into the Basilisk package directory.
        library_path = os.path.join(os.path.dirname(library_path), "..", "mujoco.dll")
    library = ctypes.CDLL(library_path)
    library.mj_loadXML.restype = ctypes.c_void_p
    library.mj_loadXML.argtypes = [
        ctypes.c_char_p,
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.c_int,
    ]
    library.mj_integratePos.restype = None
    library.mj_integratePos.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_double,
    ]
    library.mj_deleteModel.restype = None
    library.mj_deleteModel.argtypes = [ctypes.c_void_p]

    error = ctypes.create_string_buffer(1024)
    model = library.mj_loadXML(
        os.fsencode(xml_path),
        None,
        error,
        len(error),
    )
    if not model:
        raise RuntimeError(error.value.decode())

    expected = np.ascontiguousarray(qpos, dtype=np.float64)
    velocity = np.ascontiguousarray(qvel, dtype=np.float64)
    try:
        library.mj_integratePos(
            model,
            expected.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            velocity.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            time_step,
        )
    finally:
        library.mj_deleteModel(model)
    return expected


def test_native_joint_policies_match_mj_integrate_pos_bits(tmp_path):
    xml_path = tmp_path / "mixed-joints.xml"
    xml_path.write_text(MIXED_JOINT_XML)
    scene = mujoco.MJScene(MIXED_JOINT_XML)
    scene.setIntegrator(svIntegrators.svIntegratorEuler(scene))
    scene.Reset(0)

    free_joint = scene.getBody("free_body").getFreeJoint()
    ball_joint = scene.getBody("ball_body").getBallJoint()
    slide_joint = scene.getBody("slide_body").getScalarJoint("slide")

    free_joint.getTranslationPositionState().setState(
        [[0.25], [-0.5], [0.75]]
    )
    free_joint.getAttitudeState().setState([[0.9], [0.1], [-0.2], [0.3]])
    ball_joint.getPositionState().setState([[0.8], [-0.3], [0.4], [0.1]])
    slide_joint.getPositionState().setState([[0.6]])
    free_joint.getTranslationVelocityState().setState(
        [[0.4], [-0.2], [0.1]]
    )
    free_joint.getAttitudeRateState().setState([[0.3], [-0.5], [0.7]])
    ball_joint.getVelocityState().setState([[-0.6], [0.2], [0.4]])
    slide_joint.getVelocityState().setState([[-0.35]])

    initial_qpos = np.asarray(scene.assembleFullQpos()).reshape(-1)
    qvel = np.asarray(scene.assembleFullQvel()).reshape(-1)
    time_step = 0.0125  # [s]
    expected = _native_integrate_position(
        xml_path,
        initial_qpos,
        qvel,
        time_step,
    )

    scene.UpdateState(macros.sec2nano(time_step))
    actual = np.asarray(scene.assembleFullQpos()).reshape(-1)
    np.testing.assert_array_equal(actual.view(np.uint64), expected.view(np.uint64))


def test_joint_owned_buffer_follows_mujoco_joint_address_order():
    scene = mujoco.MJScene(BALL_THEN_SLIDE_XML)
    scene.Reset(0)

    body = scene.getBody("mixed")
    ball = body.getBallJoint()
    slide = body.getScalarJoint("slide")
    assert ball.getQposAdr() == 0
    assert slide.getQposAdr() == 4

    ball.getPositionState().setState([[1.0], [0.0], [0.0], [0.0]])
    slide.getPositionState().setState([[0.75]])
    ball.getVelocityState().setState([[0.1], [0.2], [0.3]])
    slide.getVelocityState().setState([[0.4]])

    np.testing.assert_array_equal(
        np.asarray(scene.assembleFullQpos()).reshape(-1),
        [1.0, 0.0, 0.0, 0.0, 0.75],
    )
    np.testing.assert_array_equal(
        np.asarray(scene.assembleFullQvel()).reshape(-1),
        [0.1, 0.2, 0.3, 0.4],
    )


@pytest.mark.parametrize("high_order", [False, True])
def test_ball_joint_owned_state_transfer(high_order):
    scene = mujoco.MJScene(BALL_XML)
    scene.highOrderAttitudeIntegration = high_order
    scene.Reset(0)

    joint = scene.getBody("rotor").getBallJoint()
    attitude = joint.getPositionState()
    angular_velocity = joint.getVelocityState()
    assert attitude.stateShape().rows == 4
    assert attitude.derivativeShape().rows == (4 if high_order else 3)
    assert angular_velocity.stateShape().rows == 3

    attitude.setState([[1.0], [0.0], [0.0], [0.0]])
    angular_velocity.setState([[0.2], [-0.1], [0.3]])
    scene.UpdateState(1)

    assert np.linalg.norm(np.asarray(attitude.getState()).reshape(-1)) == pytest.approx(1.0)
    assert np.asarray(angular_velocity.getStateDeriv()).shape == (3, 1)


def test_pre_reset_joint_exception_is_translated():
    code = textwrap.dedent(
        f"""
        from Basilisk.simulation import mujoco
        scene = mujoco.MJScene({HINGE_XML!r})
        joint = scene.getBody("link").getScalarJoint("hinge")
        try:
            joint.getQposAdr()
        except RuntimeError:
            pass
        else:
            raise AssertionError("getQposAdr unexpectedly succeeded before Reset")
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def _run_free_spin(high_order, step, final_time):
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("qposPolicyProcess")
    process.addTask(
        simulation.CreateNewTask("qposPolicyTask", macros.sec2nano(step))
    )

    scene = mujoco.MJScene(SPHERE_XML)
    scene.highOrderAttitudeIntegration = high_order
    scene.extraEoMCall = True
    simulation.AddModelToTask("qposPolicyTask", scene)

    recorder = scene.getBody("hub").getOrigin().stateOutMsg.recorder(
        macros.sec2nano(step)
    )
    simulation.AddModelToTask("qposPolicyTask", recorder)

    simulation.InitializeSimulation()
    scene.getBody("hub").setAttitudeRate([0.4, -0.3, 0.8])
    simulation.ConfigureStopTime(macros.sec2nano(final_time))
    simulation.ExecuteSimulation()

    quaternion = rbk.MRP2EP(np.asarray(recorder.sigma_BN)[-1])
    return quaternion


@pytest.mark.parametrize("high_order", [False, True])
def test_qpos_policy_normalizes_quaternions(high_order):
    quaternion = _run_free_spin(high_order, 0.1, 5.0)
    assert np.linalg.norm(quaternion) == pytest.approx(1.0, abs=1e-12)


def test_high_order_policy_improves_rk4_convergence():
    reference = _run_free_spin(True, 0.0025, 2.0)

    def error(step):
        quaternion = _run_free_spin(True, step, 2.0)
        dcm = rbk.EP2C(quaternion).dot(rbk.EP2C(reference).T)
        return 4.0 * np.arctan(np.linalg.norm(rbk.C2MRP(dcm)))

    assert error(0.1) / error(0.05) > 8.0
