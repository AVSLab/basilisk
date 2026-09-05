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

from Basilisk import hasBuildFeature
from Basilisk.architecture.bskLogging import BasiliskError

mujocoEnabled = hasBuildFeature("mujoco")
pytestmark = pytest.mark.skipif(
    not mujocoEnabled,
    reason="Requires Basilisk built with --mujoco True",
)
if mujocoEnabled:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import svIntegrators

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{TEST_FOLDER}/test_sat.xml"


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


def test_adaptive_free_joint_translation_tolerances_are_stage_independent():
    """Tests MJScene zeroes bulk relTol for a free body through the adaptive-integrator interface.

    The relative-tolerance conditioning fix applies to any free-joint body at
    orbital scale, independent of how its gravity is applied, so the presence of
    a free body (``cube`` here) alone should trigger it.
    """

    scene = mujoco.MJScene.fromFile(XML_PATH)

    integrator = svIntegrators.svIntegratorRKF78(scene)
    scene.setIntegrator(integrator)
    scene.Reset(0)

    assert integrator.getRelativeTolerance("mujocoQpos") == pytest.approx(0.0)
    assert integrator.getRelativeTolerance("mujocoQvel") == pytest.approx(0.0)


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
