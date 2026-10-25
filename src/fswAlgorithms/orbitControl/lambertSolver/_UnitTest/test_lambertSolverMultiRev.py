#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
#
#
"""Regression coverage for Lambert starters, transfer direction, and the minimum-time boundary."""

import copy

import numpy as np
import pytest
from scipy.integrate import solve_ivp
from scipy.optimize import brentq, minimize_scalar

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.fswAlgorithms import lambertSolver
from Basilisk.utilities import orbitalMotion


MU = 3.986004360e14  # [m^3/s^2]
RADIUS = 8.0e6  # [m]


def solve_problem(
    r1, r2, transfer_time, revolutions, method=messaging.GOODING, module=None, *, transfer_direction=None
):
    """Run one module update and return owned solution and performance snapshots."""
    if module is None:
        module = lambertSolver.LambertSolver()
    payload = messaging.LambertProblemMsgPayload()
    payload.solverMethod = method
    payload.r1_N = r1
    payload.r2_N = r2
    payload.transferTime = transfer_time
    payload.mu = MU
    payload.numRevolutions = revolutions
    if transfer_direction is not None:
        payload.transferDirection = transfer_direction
    message = messaging.LambertProblemMsg().write(payload)
    module.lambertProblemInMsg.subscribeTo(message)
    module.Reset(0)
    module.UpdateState(0)
    # Message reads are borrowed views; copy every field while the module is alive.
    solution_view = module.lambertSolutionOutMsg.read()
    solution = messaging.LambertSolutionMsgPayload()
    for field in ("v1_N", "v2_N", "valid", "v1Sol2_N", "v2Sol2_N", "validSol2"):
        setattr(solution, field, getattr(solution_view, field))
    performance_view = module.lambertPerformanceOutMsg.read()
    performance = messaging.LambertPerformanceMsgPayload()
    for field in ("x", "numIter", "errX", "xSol2", "numIterSol2", "errXSol2"):
        setattr(performance, field, getattr(performance_view, field))
    return solution, performance


def lagrange_time(x, lam, revolutions):
    """Evaluate elliptic flight time using Lagrange's eccentric-angle formulation."""
    axis = 1.0/(1.0-x*x)
    alpha = 2.0*np.arccos(x)
    beta = 2.0*np.arcsin(lam/np.sqrt(axis))
    return axis**1.5*(alpha-np.sin(alpha)-beta+np.sin(beta)+2.0*np.pi*revolutions)/2.0


def problem_geometry(lam, revolutions):
    """Construct endpoints and independently minimize their Lagrange time curve."""
    angle = np.pi-4.0*np.arctan(lam)  # [rad]
    r1 = np.array([RADIUS, 0.0, 0.0])  # [m]
    r2 = RADIUS*np.array([np.cos(angle), np.sin(angle), 0.0])  # [m]
    semiperimeter = (np.linalg.norm(r1)+np.linalg.norm(r2)+np.linalg.norm(r2-r1))/2.0
    time_scale = np.sqrt(2.0*MU/semiperimeter**3)  # [1/s]
    minimum = minimize_scalar(
        lagrange_time, args=(lam, revolutions), bounds=(-0.9, 0.9),
        method="bounded", options={"xatol": 1e-14},
    )
    assert minimum.success
    return r1, r2, time_scale, minimum.x, minimum.fun


@pytest.mark.parametrize("lam", [-0.99, -0.9, -0.5, -0.1, 0.1, 0.263584442471, 0.5, 0.9])
@pytest.mark.parametrize("revolutions", [1, 2, 3, 4, 7])
@pytest.mark.parametrize("time_ratio", [1.00000001, 1.0001, 1.01, 1.041779966, 1.1, 1.5, 10.0, 100.0])
def test_gooding_multirevolution_branches(lam, revolutions, time_ratio):
    """Check both roots against independent bracketing over near and distant transfers.

    The geometry parameter covers short and long transfers, including the issue
    1651 geometry. Time ratios cover the neighborhood of the minimum, both
    starter patches, and long transfers. The ``lam = -0.99`` case exercises a
    minimum-time Halley step that leaves the elliptic domain without a safeguard.
    Each root must be on its own side of the minimum and satisfy the independently
    evaluated flight-time equation.
    """
    r1, r2, time_scale, x_min, t_min = problem_geometry(lam, revolutions)
    target = time_ratio*t_min
    solution, performance = solve_problem(r1, r2, target/time_scale, revolutions)
    assert solution.valid == solution.validSol2 == 1
    roots = np.array([performance.x, performance.xSol2])
    assert -1.0 < roots[0] < x_min < roots[1] < 1.0
    reference = [
        brentq(lambda x: lagrange_time(x, lam, revolutions)-target, -1.0+1e-10, x_min),
        brentq(lambda x: lagrange_time(x, lam, revolutions)-target, x_min, 1.0-1e-10),
    ]
    np.testing.assert_allclose(roots, reference, rtol=0.0, atol=2e-8)
    np.testing.assert_allclose(lagrange_time(roots, lam, revolutions), target, rtol=2e-12, atol=0.0)
    assert np.isfinite([solution.v1_N, solution.v2_N, solution.v1Sol2_N, solution.v2Sol2_N]).all()
    assert np.linalg.norm(np.array(solution.v1_N)-solution.v1Sol2_N) > 1e-3  # [m/s]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("lam", [-0.99, -0.9, -0.5, 0.1, 0.5, 0.9])
@pytest.mark.parametrize("revolutions", [1, 3, 7])
def test_minimum_time_boundary(method, direction, lam, revolutions):
    """Check infeasible, coalesced, and distinct roots on successive module updates.

    At the minimum, both valid slots represent the same orbit. Values below the
    minimum are invalid, and values above it must recover two distinct roots.
    Reusing the module also checks recovery after an infeasible request.
    """
    r1, r2, time_scale, x_min, t_min = problem_geometry(lam, revolutions)
    if direction == messaging.RETROGRADE:
        r2[1] = -r2[1]
    module = lambertSolver.LambertSolver()
    for ratio in [0.9999, 1.01, 1.0, 0.9999, 1.01]:
        solution, performance = solve_problem(
            r1, r2, ratio*t_min/time_scale, revolutions, method, module, transfer_direction=direction
        )
        if ratio < 1.0:
            assert solution.valid == solution.validSol2 == 0
            np.testing.assert_array_equal(
                [solution.v1_N, solution.v2_N, solution.v1Sol2_N, solution.v2Sol2_N], np.zeros((4, 3))
            )
        elif ratio == 1.0:
            assert solution.valid == solution.validSol2 == 1
            np.testing.assert_allclose([performance.x, performance.xSol2], x_min, rtol=0.0, atol=2e-8)
            np.testing.assert_array_equal(solution.v1_N, solution.v1Sol2_N)
            np.testing.assert_array_equal(solution.v2_N, solution.v2Sol2_N)
            assert performance.numIter == performance.numIterSol2 == 0
            assert performance.errX == performance.errXSol2 == 0.0
        else:
            assert solution.valid == solution.validSol2 == 1
            assert performance.x < x_min < performance.xSol2
            roots = np.array([performance.x, performance.xSol2])
            np.testing.assert_allclose(
                lagrange_time(roots, lam, revolutions), ratio*t_min, rtol=2e-12, atol=0.0
            )
            assert np.isfinite([solution.v1_N, solution.v2_N, solution.v1Sol2_N, solution.v2Sol2_N]).all()
        if solution.valid:
            expected_sign = -1 if direction == messaging.RETROGRADE else 1
            for velocity in (solution.v1_N, solution.v1Sol2_N):
                assert expected_sign*np.cross(r1, velocity)[2] > 0.0


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
def test_nonfinite_minimum_time_is_invalid(method):
    """Reject nonfinite minimum-time data instead of publishing valid NaN velocities."""
    position = np.array([RADIUS, 0.0, 0.0])  # [m]
    target = np.array([np.nan, RADIUS, 0.0])  # [m]
    transfer_time = 1.0e4  # [s]
    solution, performance = solve_problem(position, target, transfer_time, 1, method)
    assert solution.valid == solution.validSol2 == 0
    np.testing.assert_array_equal(
        [solution.v1_N, solution.v2_N, solution.v1Sol2_N, solution.v2Sol2_N], np.zeros((4, 3))
    )
    assert performance.x == performance.xSol2 == 0.0


def propagate(r1, v1, transfer_time):
    """Integrate the Cartesian two-body equations independently of the Lambert solver."""
    def dynamics(time, state):
        """Return the Cartesian position and velocity derivatives."""
        position = state[:3]
        return np.concatenate((state[3:], -MU*position/np.linalg.norm(position)**3))

    result = solve_ivp(
        dynamics, (0.0, transfer_time), np.concatenate((r1, v1)), method="DOP853",
        rtol=2e-12, atol=1e-9,  # Absolute tolerances in [m] and [m/s], respectively.
    )
    assert result.success
    return result.y[:3, -1], result.y[3:, -1]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
def test_issue_1651_reported_orbit(method):
    """Recover the reported orbit and propagate both distinct solutions to the target."""
    r1 = np.array([4155609.690, 3501920.014, 3501920.014])  # [m]
    r2 = np.array([-8931746.497, 1113628.050, 1113628.050])  # [m]
    transfer_time = 9302.364  # [s]
    known_velocity = np.array([-6239.193, 4156.804, 4156.804])  # [m/s]
    solution, performance = solve_problem(r1, r2, transfer_time, 1, method)
    assert solution.valid == solution.validSol2 == 1
    assert performance.x < performance.xSol2
    np.testing.assert_allclose(solution.v1Sol2_N, known_velocity, rtol=0.0, atol=2e-3)  # [m/s]
    for v1, v2 in [(solution.v1_N, solution.v2_N), (solution.v1Sol2_N, solution.v2Sol2_N)]:
        final_position, final_velocity = propagate(r1, v1, transfer_time)
        np.testing.assert_allclose(final_position, r2, rtol=0.0, atol=1e-2)  # [m]
        np.testing.assert_allclose(final_velocity, v2, rtol=0.0, atol=1e-5)  # [m/s]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("revolutions", [0, 1, 2, 3])
@pytest.mark.parametrize("final_anomaly", [140.0, 260.0])  # [deg]
@pytest.mark.parametrize("inclination", [45.0, 150.0])  # [deg]
def test_known_elliptic_orbit(method, revolutions, final_anomaly, inclination):
    """Recover exact Keplerian velocities for zero and multiple revolutions in both angle regimes."""
    elements = orbitalMotion.ClassicElements()
    elements.a = RADIUS
    elements.e = 0.2  # [-]
    elements.i = np.deg2rad(inclination)  # [rad]
    elements.Omega = 0.0  # [rad]
    elements.omega = np.deg2rad(30.0)  # [rad]
    elements.f = np.deg2rad(20.0)  # [rad]
    final_elements = copy.deepcopy(elements)
    final_elements.f = np.deg2rad(final_anomaly)
    r1, v1 = orbitalMotion.elem2rv(MU, elements)
    r2, v2 = orbitalMotion.elem2rv(MU, final_elements)
    mean1 = orbitalMotion.E2M(orbitalMotion.f2E(elements.f, elements.e), elements.e)
    mean2 = orbitalMotion.E2M(orbitalMotion.f2E(final_elements.f, elements.e), elements.e)
    transfer_time = ((mean2-mean1) % (2.0*np.pi)+2.0*np.pi*revolutions)/np.sqrt(MU/elements.a**3)
    direction = messaging.RETROGRADE if inclination > 90.0 else messaging.PROGRADE  # [deg]
    solution, _ = solve_problem(r1, r2, transfer_time, revolutions, method, transfer_direction=direction)
    assert solution.valid == 1
    if revolutions == 0:
        assert solution.validSol2 == 0
        computed_v1, computed_v2 = solution.v1_N, solution.v2_N
    else:
        assert solution.validSol2 == 1
        if np.linalg.norm(np.array(solution.v1_N)-v1) < np.linalg.norm(np.array(solution.v1Sol2_N)-v1):
            computed_v1, computed_v2 = solution.v1_N, solution.v2_N
        else:
            computed_v1, computed_v2 = solution.v1Sol2_N, solution.v2Sol2_N
    np.testing.assert_allclose(computed_v1, v1, rtol=0.0, atol=1e-5)  # [m/s]
    np.testing.assert_allclose(computed_v2, v2, rtol=0.0, atol=1e-5)  # [m/s]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
def test_issue_1652_reported_retrograde_orbit(method):
    """Recover the reported retrograde orbit while preserving the default prograde solution."""
    r1 = np.array([5660550.077, -2909642.491, 3387572.056])  # [m]
    r2 = np.array([-7203517.962, -4382872.685, -734884.231])  # [m]
    transfer_time = 2228.114  # [s]
    known_velocity = np.array([-4029.260, -6525.015, 1390.544])  # [m/s]
    module = lambertSolver.LambertSolver()
    default, _ = solve_problem(r1, r2, transfer_time, 0, method, module)
    assert messaging.LambertProblemMsgPayload().transferDirection == messaging.PROGRADE
    for direction in (messaging.PROGRADE, messaging.RETROGRADE, messaging.PROGRADE):
        solution, _ = solve_problem(
            r1, r2, transfer_time, 0, method, module, transfer_direction=direction
        )
        assert solution.valid == 1
        assert solution.validSol2 == 0
        if direction == messaging.RETROGRADE:
            np.testing.assert_allclose(solution.v1_N, known_velocity, rtol=0.0, atol=2e-3)  # [m/s]
            assert np.cross(r1, solution.v1_N)[2] < 0.0
            final_position, final_velocity = propagate(r1, solution.v1_N, transfer_time)
            np.testing.assert_allclose(final_position, r2, rtol=0.0, atol=1e-2)  # [m]
            np.testing.assert_allclose(final_velocity, solution.v2_N, rtol=0.0, atol=1e-5)  # [m/s]
        else:
            np.testing.assert_array_equal(solution.v1_N, default.v1_N)
            np.testing.assert_array_equal(solution.v2_N, default.v2_N)
            assert np.cross(r1, solution.v1_N)[2] > 0.0


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("eccentricity", [1.0, 1.2])  # [-]
@pytest.mark.parametrize("inclination", [30.0, 150.0])  # [deg]
def test_directed_unbound_orbit(method, eccentricity, inclination):
    """Recover parabolic and hyperbolic Keplerian velocities in both directions."""
    elements = orbitalMotion.ClassicElements()
    elements.a = -RADIUS
    elements.e = eccentricity
    elements.i = np.deg2rad(inclination)  # [rad]
    elements.Omega = np.deg2rad(35.0)  # [rad]
    elements.omega = np.deg2rad(30.0)  # [rad]
    elements.f = np.deg2rad(10.0)  # [rad]
    r1, v1 = orbitalMotion.elem2rv_parab(MU, elements)
    final_anomaly = np.deg2rad(130.0)  # [rad]
    if eccentricity == 1.0:
        tangent1 = np.tan(elements.f/2.0)
        tangent2 = np.tan(final_anomaly/2.0)
        transfer_time = (tangent2+tangent2**3/3.0-tangent1-tangent1**3/3.0)/np.sqrt(MU/(2.0*RADIUS**3))
    else:
        mean1 = orbitalMotion.H2N(orbitalMotion.f2H(elements.f, eccentricity), eccentricity)
        mean2 = orbitalMotion.H2N(orbitalMotion.f2H(final_anomaly, eccentricity), eccentricity)
        transfer_time = (mean2-mean1)/np.sqrt(MU/RADIUS**3)
    elements.f = final_anomaly
    r2, v2 = orbitalMotion.elem2rv_parab(MU, elements)
    direction = messaging.RETROGRADE if inclination > 90.0 else messaging.PROGRADE  # [deg]
    solution, _ = solve_problem(r1, r2, transfer_time, 0, method, transfer_direction=direction)
    assert solution.valid == 1
    assert solution.validSol2 == 0
    np.testing.assert_allclose(solution.v1_N, v1, rtol=0.0, atol=1e-5)  # [m/s]
    np.testing.assert_allclose(solution.v2_N, v2, rtol=0.0, atol=1e-5)  # [m/s]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
def test_polar_direction_tie(method, direction):
    """Use opposite orbit normals deterministically when their z-component is exactly zero."""
    r1 = np.array([RADIUS, 0.0, 0.0])  # [m]
    r2 = np.array([0.0, 0.0, RADIUS])  # [m]
    period = 2.0*np.pi*np.sqrt(RADIUS**3/MU)
    sign = -1 if direction == messaging.RETROGRADE else 1
    transfer_time = period*(0.75 if direction == messaging.RETROGRADE else 0.25)
    speed = np.sqrt(MU/RADIUS)
    solution, _ = solve_problem(r1, r2, transfer_time, 0, method, transfer_direction=direction)
    assert solution.valid == 1
    assert solution.validSol2 == 0
    np.testing.assert_allclose(solution.v1_N, [0.0, 0.0, sign*speed], rtol=0.0, atol=1e-5)  # [m/s]
    np.testing.assert_allclose(solution.v2_N, [-sign*speed, 0.0, 0.0], rtol=0.0, atol=1e-5)  # [m/s]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [int(np.iinfo(np.intc).min), -1, 2, int(np.iinfo(np.intc).max)])
def test_invalid_transfer_direction(method, direction):
    """Reject unknown direction values instead of silently choosing a prograde transfer."""
    r1 = np.array([RADIUS, 0.0, 0.0])  # [m]
    r2 = np.array([0.0, RADIUS, 0.0])  # [m]
    transfer_time = 2000.0  # [s]
    with pytest.raises(BasiliskError, match="transferDirection"):
        solve_problem(r1, r2, transfer_time, 0, method, transfer_direction=direction)


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("revolutions", [0, 1, 3])
@pytest.mark.parametrize("azimuth", [0.0, 0.4, 1.7, 3.2])  # [rad]
def test_rotated_polar_direction_roundoff(method, direction, revolutions, azimuth):
    """Preserve the polar tie under rotation with unequal endpoint radii.

    The unrotated endpoints have an exactly zero cross-product z-component,
    but normalizing them separately introduces a small negative component.
    Both solution branches must follow the documented tie convention and
    agree with an axis-aligned reference rotated about the input-frame z-axis.
    """
    r1 = np.array([1.0e6, 5.0e6, 6.0e6])  # [m]
    r2 = np.array([2.0e6, 10.0e6, -7.0e6])  # [m]
    reference_r1 = np.array([np.hypot(r1[0], r1[1]), 0.0, r1[2]])
    reference_r2 = np.array([np.hypot(r2[0], r2[1]), 0.0, r2[2]])
    initial_azimuth = np.arctan2(r1[1], r1[0])
    rotation = np.array([
        [np.cos(azimuth), -np.sin(azimuth), 0.0],
        [np.sin(azimuth), np.cos(azimuth), 0.0],
        [0.0, 0.0, 1.0],
    ])
    r1, r2 = rotation@r1, rotation@r2
    reference_angle = initial_azimuth+azimuth
    reference_rotation = np.array([
        [np.cos(reference_angle), -np.sin(reference_angle), 0.0],
        [np.sin(reference_angle), np.cos(reference_angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    transfer_time = 5000.0*(2*revolutions+1)  # [s]
    reference, _ = solve_problem(
        reference_r1, reference_r2, transfer_time, revolutions, method, transfer_direction=direction
    )
    solution, _ = solve_problem(r1, r2, transfer_time, revolutions, method, transfer_direction=direction)
    assert solution.valid == reference.valid == 1
    assert solution.validSol2 == reference.validSol2 == int(revolutions > 0)
    for field in ("v1_N", "v2_N", "v1Sol2_N", "v2Sol2_N"):
        np.testing.assert_allclose(
            getattr(solution, field), reference_rotation@getattr(reference, field), rtol=0.0, atol=1e-7  # [m/s]
        )
    sign = -1 if direction == messaging.RETROGRADE else 1
    for velocity in ([solution.v1_N, solution.v1Sol2_N] if revolutions else [solution.v1_N]):
        assert sign*np.dot(np.cross(r1, velocity), np.cross(r1, r2)) > 0.0


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("tilt", [-1e-10, 1e-10])  # [rad]
def test_resolved_near_polar_direction(method, direction, tilt):
    """Keep a resolvable angular-momentum sign on either side of the polar tie."""
    r1 = np.array([np.sqrt(26.0)*1e6, 0.0, 6e6])  # [m]
    r2 = np.array([2.0*np.sqrt(26.0)*1e6, 0.0, -7e6])  # [m]
    azimuth = 0.7  # [rad]
    rotation_z = np.array([
        [np.cos(azimuth), -np.sin(azimuth), 0.0],
        [np.sin(azimuth), np.cos(azimuth), 0.0],
        [0.0, 0.0, 1.0],
    ])
    rotation_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(tilt), -np.sin(tilt)],
        [0.0, np.sin(tilt), np.cos(tilt)],
    ])
    rotation = rotation_z@rotation_x
    # The unrotated normal points along +y, so its tilted z-component follows tilt.
    short_angle = (tilt > 0.0) == (direction == messaging.PROGRADE)
    reference_direction = messaging.PROGRADE if short_angle else messaging.RETROGRADE
    transfer_time = 5000.0  # [s]
    reference, _ = solve_problem(r1, r2, transfer_time, 0, method, transfer_direction=reference_direction)
    rotated_r1, rotated_r2 = rotation@r1, rotation@r2
    solution, _ = solve_problem(
        rotated_r1, rotated_r2, transfer_time, 0, method, transfer_direction=direction
    )
    assert solution.valid == reference.valid == 1
    np.testing.assert_allclose(solution.v1_N, rotation@reference.v1_N, rtol=0.0, atol=1e-7)  # [m/s]
    np.testing.assert_allclose(solution.v2_N, rotation@reference.v2_N, rtol=0.0, atol=1e-7)  # [m/s]
    sign = -1 if direction == messaging.RETROGRADE else 1
    assert sign*np.cross(rotated_r1, solution.v1_N)[2] > 0.0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
