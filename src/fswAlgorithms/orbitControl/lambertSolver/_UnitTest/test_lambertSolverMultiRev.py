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
"""Regression coverage for the Gooding starters and the minimum-time boundary."""

import copy

import numpy as np
import pytest
from scipy.integrate import solve_ivp
from scipy.optimize import brentq, minimize_scalar

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertSolver
from Basilisk.utilities import orbitalMotion


MU = 3.986004360e14  # [m^3/s^2]
RADIUS = 8.0e6  # [m]


def solve_problem(r1, r2, transfer_time, revolutions, method=messaging.GOODING, module=None):
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
@pytest.mark.parametrize("lam", [-0.99, -0.9, -0.5, 0.1, 0.5, 0.9])
@pytest.mark.parametrize("revolutions", [1, 3, 7])
def test_minimum_time_boundary(method, lam, revolutions):
    """Check infeasible, coalesced, and distinct roots on successive module updates.

    At the minimum, both valid slots represent the same orbit. Values below the
    minimum are invalid, and values above it must recover two distinct roots.
    Reusing the module also checks recovery after an infeasible request.
    """
    r1, r2, time_scale, x_min, t_min = problem_geometry(lam, revolutions)
    module = lambertSolver.LambertSolver()
    for ratio in [0.9999, 1.01, 1.0, 0.9999, 1.01]:
        solution, performance = solve_problem(r1, r2, ratio*t_min/time_scale, revolutions, method, module)
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
def test_known_elliptic_orbit(method, revolutions, final_anomaly):
    """Recover exact Keplerian velocities for zero and multiple revolutions in both angle regimes."""
    elements = orbitalMotion.ClassicElements()
    elements.a = RADIUS
    elements.e = 0.2  # [-]
    elements.i = np.deg2rad(45.0)  # [rad]
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
    solution, _ = solve_problem(r1, r2, transfer_time, revolutions, method)
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


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
