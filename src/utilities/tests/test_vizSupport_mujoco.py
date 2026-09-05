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

"""Exercise MuJoCo visualization ownership with the real SWIG interfaces."""

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
    # MJCF positions and sizes are in [m], with colors and quaternions [-].
    return mujoco.MJScene(f"""
        <mujoco>
          <worldbody>
            <body name="{name}">
              <freejoint/>
              <geom type="box" size="0.5 0.5 0.5"/>
              <geom type="sphere" size="0.2" pos="0 0 1" rgba="1 0 0 1"/>
              <body name="{name}_panel" pos="0 0 2">
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
