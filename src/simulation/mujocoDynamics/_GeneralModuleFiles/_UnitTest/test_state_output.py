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

"""Regression coverage for owning MuJoCo scene-state output messages."""

import numpy as np
import pytest

from Basilisk import hasBuildFeature

mujoco_enabled = hasBuildFeature("mujoco")
pytestmark = pytest.mark.skipif(
    not mujoco_enabled,
    reason="Requires Basilisk built with --mujoco True",
)
if mujoco_enabled:
    from Basilisk.simulation import mujoco


@pytest.mark.parametrize("actuator_count", [0, 2])
def test_scene_actuator_output_owns_snapshot(actuator_count):
    """Publish current activation values without aliasing later state updates.

    The output must also represent an empty activation vector when the scene
    has no actuator states. Zero-duration updates isolate message publication
    from changes due to integration.
    """
    # MuJoCo geometry dimensions are in m, mass is in kg, and dynprm is in s.
    actuators = "".join(
        f'<general name="drive{index}" joint="hinge" '
        'dyntype="filter" dynprm="0.2"/>'
        for index in range(actuator_count)
    )
    scene = mujoco.MJScene(f"""
        <mujoco>
          <option gravity="0 0 0"/>
          <worldbody>
            <body name="body">
              <joint name="hinge"/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
          <actuator>{actuators}</actuator>
        </mujoco>
    """)
    scene.Reset(0)  # [ns]
    recorder = scene.stateOutMsg.recorder()
    activation_state = scene.getActState()
    initial = np.array([[0.25], [-0.5]])[:actuator_count]  # [-]
    updated = np.array([[-0.75], [0.125]])[:actuator_count]  # [-]
    if actuator_count:
        activation_state.setState(initial)
    else:
        assert activation_state is None

    scene.UpdateState(0)  # [ns]
    recorder.UpdateState(0)  # [ns]
    np.testing.assert_array_equal(
        np.asarray(scene.stateOutMsg.read().act).reshape(-1), initial.reshape(-1)
    )

    if actuator_count:
        activation_state.setState(updated)
    # A published message must remain a snapshot even before the next write.
    np.testing.assert_array_equal(
        np.asarray(scene.stateOutMsg.read().act).reshape(-1), initial.reshape(-1)
    )

    scene.UpdateState(0)  # [ns]
    recorder.UpdateState(1)  # [ns]
    np.testing.assert_array_equal(
        np.asarray(scene.stateOutMsg.read().act).reshape(-1), updated.reshape(-1)
    )
    np.testing.assert_array_equal(
        np.asarray(recorder.act).reshape(2, actuator_count),
        np.stack([initial.reshape(-1), updated.reshape(-1)]),
    )
