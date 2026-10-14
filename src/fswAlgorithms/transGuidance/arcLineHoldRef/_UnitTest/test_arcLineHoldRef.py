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

"""Unit tests for the fixed arc-line-hold translation reference."""

import numpy as np
import pytest

from Basilisk.architecture import bskLogging
from Basilisk.fswAlgorithms import arcLineHoldRef
from Basilisk.utilities import macros


def _configured_module():
    """Create a short maneuver for direct, fast boundary evaluations."""
    module = arcLineHoldRef.ArcLineHoldRef()
    module.bskLogger = bskLogging.BSKLogger()
    module.setRadius(6.0)  # [m]
    module.setTCircle(20.0)  # [s]
    module.setTTransition(5.0)  # [s]
    module.setTLinear(10.0)  # [s]
    module.Reset(0)
    return module


def _assert_states_close(actual, expected):
    """Compare position, velocity, and acceleration at a phase boundary."""
    tolerances = (1e-9, 1e-9, 1e-9)  # [m], [m/s], [m/s^2]
    for actual_state, expected_state, tolerance in zip(actual, expected, tolerances):
        np.testing.assert_allclose(actual_state, expected_state, rtol=0.0, atol=tolerance)


def test_phase_boundary_continuity():
    """Verify position, velocity, and acceleration join at every phase boundary.

    **Validation Test Description**

    Compare the analytic states on both sides of the arc-to-blend,
    blend-to-deceleration, and deceleration-to-hold boundaries.

    **Description of Variables Being Tested**

    Position, velocity, and acceleration references are continuous.
    """
    module = _configured_module()
    hold = (module.finalPosition, np.zeros(3), np.zeros(3))
    boundary_states = (
        (module.circleStates(module.tCircle), module.blendStates(0.0)),
        (module.blendStates(module.tTransition), module.decelStates(0.0)),
        (module.decelStates(module.tLinear), hold),
    )
    for before, after in boundary_states:
        _assert_states_close(before, after)


def test_output_across_phases():
    """Verify the published reference switches phases and remains at the hold.

    **Validation Test Description**

    Evaluate the output message in each phase, at the phase boundaries, and
    after the maneuver ends.

    **Description of Variables Being Tested**

    The ``TransRefMsg`` position, velocity, and acceleration fields match the
    selected phase and the final stationary reference.
    """
    module = _configured_module()
    arc_end = module.tCircle  # [s]
    blend_end = arc_end + module.tTransition  # [s]
    stop_time = blend_end + module.tLinear  # [s]
    hold = (module.finalPosition, np.zeros(3), np.zeros(3))
    cases = (
        (0.0, module.circleStates(0.0)),
        (arc_end / 2.0, module.circleStates(arc_end / 2.0)),
        (arc_end, module.blendStates(0.0)),
        (arc_end + module.tTransition / 2.0, module.blendStates(module.tTransition / 2.0)),
        (blend_end, module.decelStates(0.0)),
        (blend_end + module.tLinear / 2.0, module.decelStates(module.tLinear / 2.0)),
        (stop_time, hold),
        (stop_time + 1.0, hold),  # [s]
    )
    for time, expected in cases:
        module.UpdateState(macros.sec2nano(time))
        payload = module.transRefOutMsg.read()
        actual = (payload.r_RN_N, payload.v_RN_N, payload.a_RN_N)
        _assert_states_close(actual, expected)


@pytest.mark.parametrize("phase", ["circle", "blend", "deceleration"])
def test_phase_derivatives(phase):
    """Verify the reported velocity and acceleration are time derivatives.

    **Validation Test Description**

    Compare analytic derivatives with centered finite differences at an
    interior point of each moving phase.

    **Description of Variables Being Tested**

    Velocity is the derivative of position, and acceleration is the
    derivative of velocity.
    """
    module = _configured_module()
    phase_states = {
        "circle": (module.circleStates, module.tCircle / 2.0),
        "blend": (module.blendStates, module.tTransition / 2.0),
        "deceleration": (module.decelStates, module.tLinear / 2.0),
    }
    states, time = phase_states[phase]
    step = 1e-4  # [s]
    before = states(time - step)
    position, velocity, acceleration = states(time)
    after = states(time + step)
    velocity_difference = (after[0] - before[0]) / (2.0 * step)  # [m/s]
    acceleration_difference = (after[1] - before[1]) / (2.0 * step)  # [m/s^2]
    np.testing.assert_allclose(velocity, velocity_difference, rtol=0.0, atol=1e-6)
    np.testing.assert_allclose(acceleration, acceleration_difference, rtol=0.0, atol=1e-6)


@pytest.mark.parametrize(
    "setter, endpoint, name",
    [
        ("setBlendEndPosition", [1.0, -16.0, 0.0], "blendEndPosition"),
        ("setBlendEndPosition", [0.0, -16.0, -2.0], "blendEndPosition"),
        ("setFinalPosition", [1.0, -5.0, 0.0], "finalPosition"),
        ("setFinalPosition", [0.0, -5.0, 2.0], "finalPosition"),
        ("setFinalPosition", [0.0, -5.0], "finalPosition"),
        ("setBlendEndPosition", [0.0, np.nan, 0.0], "blendEndPosition"),
    ],
)
def test_invalid_endpoints(setter, endpoint, name):
    """Reject endpoints that would cause a jump or lack a finite position.

    **Validation Test Description**

    Set off-axis, malformed, and non-finite blend or final endpoints before
    reset and verify that configuration fails explicitly.

    **Description of Variables Being Tested**

    ``blendEndPosition`` and ``finalPosition`` must be finite three-component
    vectors with zero inertial x and z components.
    """
    module = arcLineHoldRef.ArcLineHoldRef()
    module.bskLogger = bskLogging.BSKLogger()
    getattr(module, setter)(endpoint)  # [m]
    with pytest.raises(bskLogging.BasiliskError, match=name):
        module.Reset(0)


@pytest.mark.parametrize(
    "setting, value",
    [
        ("setTTransition", 0.0),
        ("setTLinear", 0.0),
        ("setTCircle", -3601.0),
    ],
)
def test_invalid_durations(setting, value):
    """Reject timing configurations that make the reference undefined.

    **Validation Test Description**

    Reset a module with a nonpositive transition or deceleration duration,
    or with a nonpositive circle-plus-transition duration.

    **Description of Variables Being Tested**

    The three maneuver timing parameters must satisfy the module's reset
    constraints.
    """
    module = arcLineHoldRef.ArcLineHoldRef()
    module.bskLogger = bskLogging.BSKLogger()
    getattr(module, setting)(value)  # [s]
    with pytest.raises(bskLogging.BasiliskError, match="duration"):
        module.Reset(0)
