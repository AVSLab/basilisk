#
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
#

"""Exercise MuJoCo visualization setup and ownership with real SWIG interfaces."""

import gc
import weakref

import pytest

from Basilisk import hasBuildFeature

visualization_enabled = hasBuildFeature("mujoco") and hasBuildFeature("vizInterface")
pytestmark = pytest.mark.skipif(
    not visualization_enabled,
    reason="Requires Basilisk built with --mujoco True and --vizInterface True",
)

if visualization_enabled:
    from Basilisk.simulation import mujoco, spacecraft
    from Basilisk.utilities import SimulationBaseClass, macros, vizSupport


def _make_scene(name):
    """Create a root and a child body, each needing an additional Vizard shape."""
    child_name = f"{name}_panel" if name else ""
    # MJCF positions and sizes are in [m], with colors and quaternions [-].
    return mujoco.MJScene(f"""
        <mujoco>
          <worldbody>
            <body name="{name}">
              <freejoint/>
              <geom type="box" size="0.5 0.5 0.5"/>
              <geom type="sphere" size="0.2" pos="0 0 1" rgba="1 0 0 1"/>
              <body name="{child_name}" pos="0 0 2">
                <joint type="hinge"/>
                <geom type="box" size="0.5 0.1 0.5"/>
                <geom type="cylinder" size="0.05 0.5" pos="1 0 0"
                      quat="0.7071067811865476 0.7071067811865476 0 0"
                      rgba="0 1 0 1"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
    """)


def _make_visualization(scene_names, output_file=None):
    """Schedule named MuJoCo scenes, or a regular spacecraft for an empty list."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    step_seconds = 0.1  # [s]
    process.addTask(simulation.CreateNewTask("task", macros.sec2nano(step_seconds)))
    models = [_make_scene(name) for name in scene_names]
    if not models:
        model = spacecraft.Spacecraft()
        model.ModelTag = "regularSpacecraft"
        models.append(model)
    for model in models:
        simulation.AddModelToTask("task", model)
    visualization = vizSupport.enableUnityVisualization(
        simulation, "task", models,
        saveFile=str(output_file) if output_file is not None else None,
    )
    return simulation, visualization


def _geometry_snapshot(visualization):
    """Copy values through the C++ pointer vector without retaining shape proxies."""
    snapshot = []
    # Index SWIG vectors and retain intermediate proxies while reading them;
    # do not create iterators over temporary nested C++ vector views.
    bodies = visualization.scData
    for body_index in range(len(bodies)):
        body = bodies[body_index]
        shape_info = body.msmInfo
        shapes = shape_info.msmList
        for shape_index in range(len(shapes)):
            shape = shapes[shape_index]
            snapshot.append((
                body.spacecraftName,
                int(shape.this),
                shape.shape,
                list(shape.position),
                list(shape.dimensions),
                list(shape.rotation),
                list(shape.positiveColor),
            ))
    return snapshot


def test_unnamed_bodies_visualize_before_initialization(tmp_path):
    """Unnamed roots and children retain their hierarchy and geometry during setup."""
    # Leave the root and child unnamed in the multi-geometry scene.
    scene = _make_scene("")
    names = list(scene.getBodyNames())
    output_readers = [scene.getBody(name).getOrigin().stateOutMsg.addSubscriber() for name in names]
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    step_seconds = 0.1  # [s]
    process.addTask(simulation.CreateNewTask("task", macros.sec2nano(step_seconds)))
    simulation.AddModelToTask("task", scene)

    visualization = vizSupport.enableUnityVisualization(
        simulation, "task", scene, saveFile=str(tmp_path / "unnamed.bin")
    )
    bodies = visualization.scData
    assert len(bodies) == 2
    assert bodies[0].spacecraftName == names[0]
    assert bodies[1].spacecraftName == names[1]
    assert bodies[1].parentSpacecraftName == names[0]
    geometry = _geometry_snapshot(visualization)
    assert [shape[0] for shape in geometry] == names
    assert all(not reader.isWritten() for reader in output_readers)

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(step_seconds))
    simulation.ExecuteSimulation()
    assert _geometry_snapshot(visualization) == geometry
    assert all(reader.isWritten() for reader in output_readers)


def test_material_rgba_reaches_vizard_primitives(tmp_path):
    """Preserve material and overriding geom RGBA for base and additional shapes."""
    # MJCF positions and sizes are in [m]; RGBA channels are dimensionless.
    scene = mujoco.MJScene("""
        <mujoco>
          <asset>
            <material name="redGlass" rgba="1 0 0 0.25"/>
            <material name="invisibleYellow" rgba="1 1 0 0"/>
          </asset>
          <worldbody>
            <body name="hub">
              <freejoint/>
              <geom type="box" size="0.5 0.5 0.5" material="redGlass"/>
              <geom type="sphere" size="0.2" pos="0 0 1"
                    material="redGlass" rgba="0 1 0 0.5"/>
              <body name="panel" pos="0 0 2">
                <joint type="hinge"/>
                <geom type="box" size="0.5 0.1 0.5"
                      material="redGlass" rgba="0 0 1 0.75"/>
                <geom type="sphere" size="0.1" pos="1 0 0" material="invisibleYellow"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
    """)
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    step_seconds = 0.1  # [s]
    process.addTask(simulation.CreateNewTask("task", macros.sec2nano(step_seconds)))
    simulation.AddModelToTask("task", scene)
    visualization = vizSupport.enableUnityVisualization(
        simulation, "task", scene, saveFile=str(tmp_path / "materials.bin")
    )

    # Vizard stores RGBA as integer channels in [0, 255], including alpha.
    settings = visualization.settings
    models = settings.customModelList
    assert len(models) == 2
    model_colors = {}
    for index in range(len(models)):
        model = models[index]
        model_colors[tuple(model.simBodiesToModify)] = list(model.color)
    assert model_colors == {("hub",): [255, 0, 0, 63], ("panel",): [0, 0, 255, 191]}

    bodies = visualization.scData
    expected_extra_colors = {"hub": [0, 255, 0, 127], "panel": [255, 255, 0, 0]}
    for index in range(len(bodies)):
        body = bodies[index]
        shape_info = body.msmInfo
        shapes = shape_info.msmList
        assert len(shapes) == 1
        shape = shapes[0]
        assert list(shape.positiveColor) == expected_extra_colors[body.spacecraftName]
        assert list(shape.negativeColor) == expected_extra_colors[body.spacecraftName]

    # Exercise C++ serialization with translucent and fully transparent colors.
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(step_seconds))
    simulation.ExecuteSimulation()


@pytest.mark.parametrize("second_uses_mujoco", [False, True], ids=["spacecraft", "mujoco"])
def test_generated_geometry_lives_with_its_visualization(
    monkeypatch, tmp_path, second_uses_mujoco
):
    """Another setup must neither release live geometry nor retain a destroyed setup.

    The factory observer records only weak references; all shapes and pointer
    vectors are real SWIG objects. Check liveness before reading the C++ vector
    so a regression fails an assertion rather than dereferencing freed memory.
    """
    shape_refs = []
    original_factory = vizSupport._makeMJSceneMultiShape

    def observe_shape(viz_info):
        """Observe each real Python-owned C++ shape without extending its lifetime."""
        shape = original_factory(viz_info)
        assert shape.thisown
        shape_refs.append(weakref.ref(shape))
        return shape

    monkeypatch.setattr(vizSupport, "_makeMJSceneMultiShape", observe_shape)
    first_simulation, first_visualization = _make_visualization(
        ["primary", "companion"], tmp_path / "first.bin"
    )
    first_shape_refs = list(shape_refs)
    assert len(first_shape_refs) == 4  # One extra geom on each root and child.
    expected_geometry = _geometry_snapshot(first_visualization)
    first_visualization_ref = weakref.ref(first_visualization)
    del first_visualization

    # A caller need not retain the returned visualization: the scheduled
    # module must retain its generated geometry.
    gc.collect()
    assert all(reference() is not None for reference in first_shape_refs)
    first_simulation.InitializeSimulation()
    first_simulation.ConfigureStopTime(0)  # [ns]
    first_simulation.ExecuteSimulation()

    second_scenes = ["second"] if second_uses_mujoco else []
    second_simulation, second_visualization = _make_visualization(second_scenes)
    second_shape_refs = shape_refs[len(first_shape_refs):]
    assert len(second_shape_refs) == (2 if second_uses_mujoco else 0)
    gc.collect()

    assert all(reference() is not None for reference in first_shape_refs), (
        "Creating another visualization released the first visualization's geometry"
    )
    assert _geometry_snapshot(first_visualization_ref()) == expected_geometry

    # Exercise serialization of the first module's C++ geometry after the
    # second setup, not only reads through the Python proxies.
    stop_seconds = 0.2  # [s]
    first_simulation.ConfigureStopTime(macros.sec2nano(stop_seconds))
    first_simulation.ExecuteSimulation()

    del first_simulation
    gc.collect()
    assert first_visualization_ref() is None
    assert all(reference() is None for reference in first_shape_refs)
    assert all(reference() is not None for reference in second_shape_refs)

    del second_visualization, second_simulation
    gc.collect()
    assert all(reference() is None for reference in second_shape_refs)
