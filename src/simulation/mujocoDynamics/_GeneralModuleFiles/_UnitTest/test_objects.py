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

import gc
import inspect
import os
import weakref

import numpy as np
import pytest

from Basilisk import hasBuildFeature
from Basilisk.architecture.bskLogging import BasiliskError

mujocoEnabled = hasBuildFeature("mujoco")
pytestmark = pytest.mark.skipif(
    not mujocoEnabled,
    reason="Requires Basilisk built with --mujoco True",
)
if mujocoEnabled:
    from Basilisk.simulation import dynParamManager
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import svIntegrators

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{TEST_FOLDER}/test_sat.xml"

OWNERSHIP_XML = """
<mujoco>
  <worldbody>
    <body name="root">
      <joint name="slide" type="slide"/>
      <geom type="sphere" size="1" mass="1"/>
      <site name="site"/>
    </body>
  </worldbody>
  <equality>
    <joint name="lock" joint1="slide"/>
  </equality>
  <actuator>
    <general
      name="stateful"
      joint="slide"
      dyntype="filter"
      dynprm="2"
      gear="1"
    />
  </actuator>
</mujoco>
"""

PREEXISTING_BODY_SITES_XML = """
<mujoco>
  <worldbody>
    <body name="root">
      <joint name="slide" type="slide"/>
      <geom type="sphere" size="1" mass="1"/>
      <site name="root_com"/>
      <site name="root_origin"/>
    </body>
  </worldbody>
</mujoco>
"""


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


def test_actuator_addition_alone_triggers_recompile():
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scene.Reset(0)

    actuator = scene.addTorqueActuator("late_torque", "panel_2_origin")
    scene.UpdateState(1)

    assert actuator.getName() == "late_torque"
    assert scene.getTorqueActuator("late_torque").getName() == "late_torque"


def test_scalar_joint_equality_recompiles_when_body_sites_already_exist():
    scene = mujoco.MJScene(PREEXISTING_BODY_SITES_XML)

    scene.Reset(0)

    equality = (
        scene.getBody("root")
        .getScalarJoint("slide")
        .getConstrainedEquality()
    )
    assert not equality.isActive()


def test_adaptive_free_joint_translation_tolerances_are_stage_independent():
    """Tests MJScene zeroes relTol only for free-joint translation records.

    The relative-tolerance conditioning fix applies to any free-joint body at
    orbital scale, independent of how its gravity is applied, so the presence of
    a free body (``cube`` here) alone should trigger it.
    """

    scene = mujoco.MJScene.fromFile(XML_PATH)

    integrator = svIntegrators.svIntegratorRKF78(scene)
    scene.setIntegrator(integrator)
    scene.Reset(0)

    joint = scene.getBody("cube").getFreeJoint()
    position_name = joint.getTranslationPositionState().getName()
    velocity_name = joint.getTranslationVelocityState().getName()
    attitude_name = joint.getAttitudeState().getName()

    assert integrator.getRelativeTolerance(position_name) == pytest.approx(0.0)
    assert integrator.getRelativeTolerance(velocity_name) == pytest.approx(0.0)
    assert integrator.getRelativeTolerance(attitude_name) != pytest.approx(0.0)


def test_adaptive_free_joint_tolerances_apply_after_integrator_replacement():
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scene.Reset(0)

    integrator = svIntegrators.svIntegratorRKF78(scene)
    scene.setIntegrator(integrator)
    scene.Reset(1)

    joint = scene.getBody("cube").getFreeJoint()
    position_name = joint.getTranslationPositionState().getName()
    velocity_name = joint.getTranslationVelocityState().getName()

    assert integrator.getRelativeTolerance(position_name) == pytest.approx(0.0)
    assert integrator.getRelativeTolerance(velocity_name) == pytest.approx(0.0)


def test_joint_state_proxy_keeps_native_owner_chain_alive():
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scene.Reset(0)

    body = scene.getBody("cube")
    joint = body.getFreeJoint()
    state = joint.getTranslationPositionState()


    del scene
    del body
    del joint

    assert state.getName().endswith("_qposTranslation")


def test_scene_and_joint_states_use_canonical_state_proxies():
    scene = mujoco.MJScene(OWNERSHIP_XML)
    scene.Reset(0)
    joint_state = (
        scene.getBody("root")
        .getScalarJoint("slide")
        .getPositionState()
    )
    mass_state = scene.getMassState()

    for name in dynParamManager._COMPATIBILITY_EXPORTS:
        assert getattr(mujoco, name) is getattr(dynParamManager, name)

    assert type(scene.dynManager) is dynParamManager.DynParamManager
    assert isinstance(scene.dynManager, dynParamManager.DynParamManager)
    assert type(joint_state) is dynParamManager.StateData
    assert isinstance(joint_state, dynParamManager.StateData)
    assert type(mass_state) is dynParamManager.StateData
    assert isinstance(mass_state, dynParamManager.StateData)


def test_constrained_equality_proxy_keeps_joint_owner_chain_alive():
    scene = mujoco.MJScene(OWNERSHIP_XML)
    scene.Reset(0)
    body = scene.getBody("root")
    joint = body.getScalarJoint("slide")
    equality = joint.getConstrainedEquality()


    scene_ref = weakref.ref(scene)
    del scene
    del body
    del joint
    gc.collect()

    assert scene_ref() is not None
    equality.setActive(True)
    assert equality.isActive()

    del equality
    gc.collect()
    assert scene_ref() is None


def test_borrowed_method_signatures_hide_native_implementation():
    expected_signatures = {
        (mujoco.MJScene, "getBody"): "(self, name)",
        (mujoco.MJScene, "getSite"): "(self, name)",
        (mujoco.MJScene, "getEquality"): "(self, name)",
        (mujoco.MJScene, "getSingleActuator"): "(self, name)",
        (mujoco.MJScene, "getForceActuator"): "(self, name)",
        (mujoco.MJScene, "getTorqueActuator"): "(self, name)",
        (mujoco.MJScene, "getForceTorqueActuator"): "(self, name)",
        (mujoco.MJScene, "addJointSingleActuator"): "(self, *args)",
        (mujoco.MJScene, "addSingleActuator"): "(self, *args)",
        (mujoco.MJScene, "addForceActuator"): "(self, *args)",
        (mujoco.MJScene, "addTorqueActuator"): "(self, *args)",
        (mujoco.MJScene, "addForceTorqueActuator"): "(self, *args)",
        (mujoco.MJScene, "getActState"): "(self)",
        (mujoco.MJScene, "getMassState"): "(self)",
        (mujoco.MJBody, "getSite"): "(self, name)",
        (mujoco.MJBody, "getCenterOfMass"): "(self)",
        (mujoco.MJBody, "getOrigin"): "(self)",
        (mujoco.MJBody, "getScalarJoint"): "(self, name)",
        (mujoco.MJBody, "getBallJoint"): "(self)",
        (mujoco.MJBody, "getFreeJoint"): "(self)",
        (mujoco.MJBody, "getScene"): "(self)",
        (mujoco.MJBody, "getSpec"): "(self)",
        (mujoco.MJJoint, "getBody"): "(self)",
        (mujoco.MJScalarJoint, "getConstrainedEquality"): "(self)",
        (mujoco.MJScalarJoint, "getPositionState"): "(self)",
        (mujoco.MJScalarJoint, "getVelocityState"): "(self)",
        (mujoco.MJBallJoint, "getPositionState"): "(self)",
        (mujoco.MJBallJoint, "getVelocityState"): "(self)",
        (mujoco.MJFreeJoint, "getTranslationPositionState"): "(self)",
        (mujoco.MJFreeJoint, "getTranslationVelocityState"): "(self)",
        (mujoco.MJFreeJoint, "getAttitudeState"): "(self)",
        (mujoco.MJFreeJoint, "getAttitudeRateState"): "(self)",
        (mujoco.MJSite, "getBody"): "(self)",
        (mujoco.MJEquality, "getScene"): "(self)",
        (mujoco.MJEquality, "getSpec"): "(self)",
    }

    for (wrapper_type, method_name), expected in expected_signatures.items():
        method = getattr(wrapper_type, method_name)
        if hasattr(method, "__signature__"):
            assert str(inspect.signature(method)) == expected
        assert "_native" not in method.__code__.co_varnames
        assert not hasattr(method, "__wrapped__")

    if hasattr(mujoco.MJScene.__init__, "__signature__"):
        assert str(inspect.signature(mujoco.MJScene.__init__)) == "(self, *args)"
    assert "_native" not in mujoco.MJScene.__init__.__code__.co_varnames
    assert not hasattr(mujoco.MJScene.__init__, "__wrapped__")


def _make_reverse_owner_proxy(proxy_case):
    scene = mujoco.MJScene(OWNERSHIP_XML)
    scene.Reset(0)

    if proxy_case == "joint_body":
        proxy = (
            scene.getBody("root")
            .getScalarJoint("slide")
            .getBody()
        )
    elif proxy_case == "site_body":
        proxy = scene.getSite("site").getBody()
    elif proxy_case == "body_scene":
        proxy = scene.getBody("root").getScene()
    else:
        proxy = scene.getEquality("lock").getScene()

    return weakref.ref(scene), proxy


@pytest.mark.parametrize(
    "proxy_case",
    [
        "joint_body",
        "site_body",
        "body_scene",
        "equality_scene",
    ],
)
def test_reverse_owner_proxy_keeps_scene_alive(proxy_case):
    scene_ref, proxy = _make_reverse_owner_proxy(proxy_case)
    gc.collect()

    assert scene_ref() is not None
    if proxy_case.endswith("_body"):
        assert proxy.getName() == "root"
    else:
        assert "root" in proxy.getBodyNames()

    del proxy
    gc.collect()
    assert scene_ref() is None


@pytest.mark.parametrize(
    "proxy_case",
    [
        "get_body",
        "get_site",
        "get_equality",
        "get_single_actuator",
        "get_force_actuator",
        "get_torque_actuator",
        "get_force_torque_actuator",
        "add_joint_single_actuator",
        "add_single_actuator",
        "add_force_actuator",
        "add_torque_actuator",
        "add_force_torque_actuator",
        "get_mass_state",
        "get_act_state",
    ],
)
def test_scene_borrowed_proxy_keeps_scene_alive(proxy_case):
    """Each borrowed scene result independently retains its native owner."""
    scene = mujoco.MJScene(OWNERSHIP_XML)
    scene.Reset(0)

    if proxy_case == "get_body":
        proxy = scene.getBody("root")
        expected_name = "root"
    elif proxy_case == "get_site":
        proxy = scene.getSite("site")
        expected_name = "site"
    elif proxy_case == "get_equality":
        proxy = scene.getEquality("lock")
        expected_name = "lock"
    elif proxy_case == "get_single_actuator":
        proxy = scene.getSingleActuator("stateful")
        expected_name = "stateful"
    elif proxy_case == "get_force_actuator":
        scene.addForceActuator("force_getter", "site")
        proxy = scene.getForceActuator("force_getter")
        expected_name = "force_getter"
    elif proxy_case == "get_torque_actuator":
        scene.addTorqueActuator("torque_getter", "site")
        proxy = scene.getTorqueActuator("torque_getter")
        expected_name = "torque_getter"
    elif proxy_case == "get_force_torque_actuator":
        scene.addForceTorqueActuator("force_torque_getter", "site")
        proxy = scene.getForceTorqueActuator("force_torque_getter")
        expected_name = "force_torque_getter"
    elif proxy_case == "add_joint_single_actuator":
        proxy = scene.addJointSingleActuator("joint_single_added", "slide")
        expected_name = "joint_single_added"
    elif proxy_case == "add_single_actuator":
        proxy = scene.addSingleActuator(
            "single_added",
            "site",
            np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        )
        expected_name = "single_added"
    elif proxy_case == "add_force_actuator":
        proxy = scene.addForceActuator("force_added", "site")
        expected_name = "force_added"
    elif proxy_case == "add_torque_actuator":
        proxy = scene.addTorqueActuator("torque_added", "site")
        expected_name = "torque_added"
    elif proxy_case == "add_force_torque_actuator":
        proxy = scene.addForceTorqueActuator("force_torque_added", "site")
        expected_name = "force_torque_added"
    elif proxy_case == "get_mass_state":
        proxy = scene.getMassState()
        expected_name = "mujocoMass"
    else:
        proxy = scene.getActState()
        expected_name = "mujocoAct"

    scene_ref = weakref.ref(scene)
    del scene
    gc.collect()

    assert scene_ref() is not None
    assert proxy.getName() == expected_name

    del proxy
    gc.collect()
    assert scene_ref() is None


@pytest.mark.parametrize(
    "proxy_case, expected_name",
    [
        ("site", "site"),
        ("center_of_mass", "root_com"),
        ("origin", "root_origin"),
    ],
)
def test_body_borrowed_site_proxy_keeps_owner_chain_alive(
    proxy_case,
    expected_name,
):
    """A borrowed site retains its body, which in turn retains the scene."""
    scene = mujoco.MJScene(OWNERSHIP_XML)
    scene.Reset(0)
    body = scene.getBody("root")
    if proxy_case == "site":
        proxy = body.getSite("site")
    elif proxy_case == "center_of_mass":
        proxy = body.getCenterOfMass()
    else:
        proxy = body.getOrigin()


    scene_ref = weakref.ref(scene)
    body_ref = weakref.ref(body)
    del scene
    del body
    gc.collect()

    assert scene_ref() is not None
    assert body_ref() is not None
    assert proxy.getName() == expected_name

    del proxy
    gc.collect()
    assert body_ref() is None
    assert scene_ref() is None


def test_failed_candidate_compile_preserves_wrappers_and_recovers():
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scene.Reset(0)
    free_joint = scene.getBody("cube").getFreeJoint()
    position_state = free_joint.getTranslationPositionState()
    handle = int(position_state.this)

    scene.addTorqueActuator("pending_torque", "pending_site")
    with pytest.raises(Exception, match="pending_torque"):
        scene.Reset(0)

    assert int(
        scene.getBody("cube")
        .getFreeJoint()
        .getTranslationPositionState()
        .this
    ) == handle
    position_state.setState([[1.0], [2.0], [3.0]])

    scene.getBody("cube").addSite("pending_site", [0.0, 0.0, 0.0])
    scene.Reset(0)

    assert int(
        scene.getBody("cube")
        .getFreeJoint()
        .getTranslationPositionState()
        .this
    ) == handle
    assert position_state.getState() == [[1.0], [2.0], [3.0]]


def test_nonzero_reset_synchronizes_integration_clocks():
    scene = mujoco.MJScene("<mujoco/>")
    reset_time_nanos = 2_000_000_000
    step_nanos = 125_000_000

    scene.Reset(reset_time_nanos)
    assert scene.timeBefore == pytest.approx(2.0)
    assert scene.timeBeforeNanos == reset_time_nanos

    scene.integrateState(reset_time_nanos + step_nanos)
    assert scene.timeStep == pytest.approx(0.125)


@pytest.mark.parametrize("root_name", ["", 'name="_nameless_1"'], ids=["unnamed-root", "named-root"])
def test_unnamed_body_introspection(root_name):
    """Body, parent, and geometry names agree before initialization and after reset.

    Cover unnamed roots and children, a named child of an unnamed parent, and
    explicit names that would otherwise collide with generated names. Creating
    another scene must not change the generated names for identical XML.
    """
    # MJCF positions and geometry sizes are in [m].
    xml = f"""
        <mujoco>
          <worldbody>
            <body {root_name}>
              <freejoint/>
              <geom type="box" size="0.5 0.5 0.5"/>
              <body pos="0 0 2">
                <joint type="hinge"/>
                <geom type="sphere" size="0.2"/>
                <body name="named_tip" pos="0 0 1">
                  <joint type="hinge"/>
                  <geom type="sphere" size="0.1"/>
                </body>
              </body>
            </body>
            <body pos="3 0 0">
              <freejoint/>
              <geom type="box" size="0.3 0.3 0.3"/>
            </body>
            <body name="_nameless_0" pos="6 0 0">
              <freejoint/>
              <geom type="box" size="0.4 0.4 0.4"/>
            </body>
          </worldbody>
        </mujoco>
    """
    scene = mujoco.MJScene(xml)
    names = list(scene.getBodyNames())
    assert len(names) == len(set(names)) == 5
    assert all(names)
    assert names[2] == "named_tip"
    assert names[4] == "_nameless_0"
    if root_name:
        assert names[0] == "_nameless_1"
    parents = ["world", names[0], names[1], "world", "world"]
    scene_reader = scene.stateOutMsg.addSubscriber()

    def check_introspection():
        """Check all public lookups without asking MuJoCo to recompile."""
        assert list(scene.getBodyNames()) == names
        assert [scene.getBodyParentName(name) for name in names] == parents
        assert [scene.getBody(name).getName() for name in names] == names
        geoms = scene.getGeomInfos()
        assert [geoms[index].bodyName for index in range(len(geoms))] == names
        with pytest.raises(BasiliskError, match="unknown body 'missing'"):
            scene.getBodyParentName("missing")

    check_introspection()
    assert not scene_reader.isWritten()
    other_scene = mujoco.MJScene(xml)
    assert list(other_scene.getBodyNames()) == names

    scene.Reset(0)  # [ns]
    check_introspection()
    assert scene_reader.isWritten()


# All RGBA values below are dimensionless, in the range [0, 1].
@pytest.mark.parametrize(
    "geom_attributes, material_rgba, expected_rgba",
    [
        pytest.param("", "1 0 0 0.25", [0.5, 0.5, 0.5, 1], id="no-material-default"),
        pytest.param('rgba="0 0 1 0.5"', "1 0 0 0.25", [0, 0, 1, 0.5], id="no-material-explicit"),
        pytest.param('material="paint"', "1 0 0 0.25", [1, 0, 0, 0.25], id="material-translucent"),
        pytest.param('material="paint"', "1 0 0 0", [1, 0, 0, 0], id="material-transparent"),
        pytest.param('material="paint"', "1 0 0 1", [1, 0, 0, 1], id="material-opaque"),
        pytest.param(
            'material="paint" rgba="0.5 0.5 0.5 1"', "1 0 0 0.25", [1, 0, 0, 0.25],
            id="explicit-default-still-uses-material",
        ),
        pytest.param(
            'material="paint" rgba="0 1 0 0.5"', "1 0 0 0.25", [0, 1, 0, 0.5],
            id="geom-overrides-material",
        ),
        pytest.param(
            'material="paint" rgba="0.5 0.5 0.5 0.125"', "1 0 0 0.25", [0.5, 0.5, 0.5, 0.125],
            id="alpha-only-override",
        ),
        pytest.param(
            'material="paint" class="tinted"', "1 0 0 0.25", [0, 0, 1, 0.75],
            id="inherited-rgba-overrides-material",
        ),
        pytest.param('class="painted"', "1 0 0 0.25", [1, 0, 0, 0.25], id="inherited-material"),
    ],
)
def test_geometry_rgba_matches_mujoco_material_precedence(
    geom_attributes, material_rgba, expected_rgba
):
    """Export material color and alpha unless the geom has non-default RGBA.

    MuJoCo treats explicit default gray like omitted RGBA, but a change to
    any channel (including alpha alone) overrides all four material channels.
    A second material ensures the export follows the geom's material index.
    """
    # MJCF geometry sizes are in [m]; RGBA values are dimensionless.
    scene = mujoco.MJScene(f"""
        <mujoco>
          <default>
            <default class="tinted"><geom rgba="0 0 1 0.75"/></default>
            <default class="painted"><geom material="paint"/></default>
          </default>
          <asset>
            <material name="unused" rgba="0 1 0 1"/>
            <material name="paint" rgba="{material_rgba}"/>
          </asset>
          <worldbody>
            <body name="hub">
              <freejoint/>
              <geom type="box" size="0.5 0.5 0.5" {geom_attributes}/>
            </body>
          </worldbody>
        </mujoco>
    """)
    scene_reader = scene.stateOutMsg.addSubscriber()
    geoms = scene.getGeomInfos()
    assert len(geoms) == 1
    assert list(geoms[0].rgba) == pytest.approx(expected_rgba)
    assert not scene_reader.isWritten()

    scene.Reset(0)  # [ns]
    geoms = scene.getGeomInfos()
    assert list(geoms[0].rgba) == pytest.approx(expected_rgba)


if __name__ == "__main__":
    if True:
        test_loading()
    else:
        pytest.main([__file__])
