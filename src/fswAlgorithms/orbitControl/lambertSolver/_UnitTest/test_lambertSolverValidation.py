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
"""Validate Lambert numeric inputs and alignment configuration before solving."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.fswAlgorithms import lambertSolver
from test_lambertSolverMultiRev import MU, RADIUS, solve_problem


def invalid_input_and_recovery(method, revolutions, field, value):
    """Reject a malformed update, then solve a corrected request without a reset."""
    payload = messaging.LambertProblemMsgPayload()
    payload.mu = MU
    payload.r1_N = [RADIUS, 0.0, 0.0]  # [m]
    payload.r2_N = [0.0, RADIUS, 0.0]  # [m]
    payload.transferTime = 2000.0 if revolutions == 0 else 10000.0  # [s]
    payload.numRevolutions = revolutions
    payload.solverMethod = method
    message = messaging.LambertProblemMsg().write(payload)
    module = lambertSolver.LambertSolver()
    module.lambertProblemInMsg.subscribeTo(message)
    module.Reset(0)
    module.UpdateState(0)
    assert module.lambertSolutionOutMsg.read().valid == 1
    original = getattr(payload, field)
    setattr(payload, field, value)
    message.write(payload)
    with pytest.raises(BasiliskError, match=field):
        module.UpdateState(0)

    setattr(payload, field, original)
    payload.transferTime *= 1.1
    message.write(payload)
    module.UpdateState(0)
    expected = solve_problem(payload.r1_N, payload.r2_N, payload.transferTime, revolutions, method)
    actual = (module.lambertSolutionOutMsg.read(), module.lambertPerformanceOutMsg.read())
    for fields, result, reference in zip(
        [("v1_N", "v2_N", "valid", "v1Sol2_N", "v2Sol2_N", "validSol2"),
         ("x", "numIter", "errX", "xSol2", "numIterSol2", "errXSol2")], actual, expected
    ):
        for name in fields:
            np.testing.assert_array_equal(getattr(result, name), getattr(reference, name))


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("revolutions", [0, 1])
@pytest.mark.parametrize("field", ["mu", "transferTime"])
@pytest.mark.parametrize("value", [0.0, -1.0, np.nan, np.inf, -np.inf])  # [m^3/s^2] or [s], by field
def test_invalid_scalar_input(method, revolutions, field, value):
    """Require finite, positive gravity and flight time on every update."""
    invalid_input_and_recovery(method, revolutions, field, value)


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("revolutions", [0, 1])
@pytest.mark.parametrize("field", ["r1_N", "r2_N"])
@pytest.mark.parametrize("component", [0, 1, 2])
@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf])  # [m]
def test_nonfinite_position_input(method, revolutions, field, component, value):
    """Reject a nonfinite component in either endpoint before geometry calculations."""
    position = np.array([RADIUS, RADIUS, RADIUS])  # [m]
    position[component] = value
    invalid_input_and_recovery(method, revolutions, field, position)


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("revolutions", [0, 1])
@pytest.mark.parametrize("field", ["r1_N", "r2_N"])
@pytest.mark.parametrize("magnitude", [0.0, 1e-300, 1e300])  # [m]
def test_unusable_position_norm(method, revolutions, field, magnitude):
    """Reject zero, underflowed, or overflowed norms before dividing by them."""
    invalid_input_and_recovery(method, revolutions, field, [magnitude, 0.0, 0.0])


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("value", [-1, int(np.iinfo(np.intc).min)])
def test_negative_revolution_count(method, value):
    """Retain the existing rejection of negative revolution counts."""
    invalid_input_and_recovery(method, 1, "numRevolutions", value)


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf, -1.0, 0.0, 90.0, 91.0, 180.0, 360.0])  # [deg]
def test_invalid_alignment_threshold(method, value):
    """Reject invalid thresholds without changing the previous valid configuration."""
    module = lambertSolver.LambertSolver()
    module.setAlignmentThreshold(2.0)  # [deg]
    with pytest.raises(BasiliskError, match="alignmentThreshold"):
        module.setAlignmentThreshold(value)
    assert module.getlignmentThreshold() == 2.0  # [deg]
    r1 = [RADIUS, 0.0, 0.0]  # [m]
    r2 = [0.0, RADIUS, 0.0]  # [m]
    solution, _ = solve_problem(r1, r2, 2000.0, 0, method, module)  # Flight time [s].
    assert solution.valid == 1


@pytest.mark.parametrize("value", [np.nextafter(0.0, 1.0), 1.0, np.nextafter(90.0, 0.0)])  # [deg]
def test_valid_alignment_threshold_boundaries(value):
    """Accept finite thresholds strictly between zero and ninety degrees."""
    module = lambertSolver.LambertSolver()
    module.setAlignmentThreshold(value)
    assert module.getlignmentThreshold() == value
    solution, _ = solve_problem(
        [RADIUS, 0.0, 0.0], [0.0, RADIUS, 0.0], 2000.0, 0, module=module  # Positions [m], time [s].
    )
    assert solution.valid == 1


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("revolutions", [0, 1])
@pytest.mark.parametrize("threshold", [np.nextafter(0.0, 1.0), 1e-15])  # [deg]
@pytest.mark.parametrize("radial", [[1, 2, 3], [2, -5, 11], [-3, 7, -13]])
@pytest.mark.parametrize("radius_ratio", [-3.0, 3.0])
def test_non_axis_aligned_collinearity(method, direction, revolutions, threshold, radial, radius_ratio):
    """Reject parallel and antiparallel endpoints despite normalization roundoff.

    A rejected update must clear both previously populated solution slots, then
    allow a corrected request to recover on the same module without a reset.
    """
    r1 = 1e6*np.array(radial, dtype=float)  # [m]
    r2 = radius_ratio*r1  # [m]
    np.testing.assert_array_equal(np.cross(r1, r2), np.zeros(3))
    payload = messaging.LambertProblemMsgPayload()
    payload.mu = MU
    payload.solverMethod = method
    payload.transferDirection = direction
    payload.numRevolutions = revolutions
    # Keep even the largest antiparallel case above the one-revolution minimum
    # so an infeasible flight time cannot mask a missing collinearity check.
    payload.transferTime = 10000.0 if revolutions == 0 else 100000.0  # [s]
    message = messaging.LambertProblemMsg()
    module = lambertSolver.LambertSolver()
    module.setAlignmentThreshold(threshold)
    module.lambertProblemInMsg.subscribeTo(message)
    module.Reset(0)
    expected = solve_problem(
        [RADIUS, 0.0, 0.0], [0.0, RADIUS, 0.0], payload.transferTime, revolutions,
        method, transfer_direction=direction,  # Positions [m].
    )
    assert expected[0].valid == 1
    assert expected[0].validSol2 == int(revolutions > 0)
    for aligned in (False, True, False):
        payload.r1_N = r1 if aligned else [RADIUS, 0.0, 0.0]  # [m]
        payload.r2_N = r2 if aligned else [0.0, RADIUS, 0.0]  # [m]
        message.write(payload)
        module.UpdateState(0)
        actual = (module.lambertSolutionOutMsg.read(), module.lambertPerformanceOutMsg.read())
        for fields, result, reference in zip(
            [("v1_N", "v2_N", "valid", "v1Sol2_N", "v2Sol2_N", "validSol2"),
             ("x", "numIter", "errX", "xSol2", "numIterSol2", "errXSol2")], actual, expected
        ):
            for name in fields:
                target = np.zeros_like(getattr(reference, name)) if aligned else getattr(reference, name)
                np.testing.assert_array_equal(getattr(result, name), target)


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("radial", [[1, 0, 0], [1, 2, 3]])
@pytest.mark.parametrize("radius_ratio", [-3.0, 3.0])
def test_resolvable_near_collinear_plane(method, direction, radial, radius_ratio):
    """Preserve small cross products that are resolvable above cancellation roundoff."""
    r1 = 1e6*np.array(radial, dtype=float)  # [m]
    transverse = np.array([-radial[1], radial[0], 0])
    r2 = radius_ratio*r1 + 1e-5*transverse  # [m]
    module = lambertSolver.LambertSolver()
    module.setAlignmentThreshold(1e-15)  # [deg]
    solution, _ = solve_problem(
        r1, r2, 10000.0, 0, method, module, transfer_direction=direction  # Flight time [s].
    )
    assert solution.valid == 1
    assert np.isfinite([solution.v1_N, solution.v2_N]).all()
