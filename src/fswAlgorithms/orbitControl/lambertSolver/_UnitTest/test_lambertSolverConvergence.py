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
"""Check root convergence and invalid-output handling independently of the starters."""

import numpy as np
import pytest
from scipy.optimize import brentq
from scipy.special import hyp2f1

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertSolver, lambertValidator
from test_lambertSolverMultiRev import (
    MU, RADIUS, lagrange_time, problem_geometry, propagate, solve_problem,
)


def battin_time(x, lam):
    """Evaluate smooth zero-revolution flight time with SciPy's hypergeometric function."""
    y = np.sqrt(1.0-lam*lam*(1.0-x*x))
    eta = y-lam*x
    return (eta**3*4.0/3.0*hyp2f1(3.0, 1.0, 2.5, (1.0-lam-x*eta)/2.0)+4.0*lam*eta)/2.0


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("r2_xy, transfer_time, reference_x", [
    # Endpoint [m], flight time [s], independently bracketed root [-].
    ((10499578.7058981, -1008665.595563449), 66294.046738751, -0.9222028885569338),
    ((-8007153.311464242, 1134336.721226995), 1517.1851310277925, 1.003755300013543),
    ((-1373963.0772790438, 21169783.315275904), 1029.9495735576572, 3.9070784702480257),
    ((-1160389.9429338065, 9261648.1844296623), 1347.9302728188266, 0.9692339680381958),
    ((17175236.41488691, -6712075.4724303577), 1589.8316798218957, 1.9840057290698279),
], ids=["rounded-correction", "hyperbolic-upper-endpoint", "first-step", "bisection-izzo", "bisection-gooding"])
def test_safeguards_preserve_fast_convergence(method, direction, r2_xy, transfer_time, reference_x):
    """Keep starters, fallback recovery, and tiny corrections within the validator's iteration budget."""
    r1 = np.array([RADIUS, 0.0, 0.0])  # [m]
    r2 = np.array([*r2_xy, 0.0])  # [m]
    chord = np.linalg.norm(r2-r1)
    semiperimeter = (RADIUS+np.linalg.norm(r2)+chord)/2.0
    lam = np.copysign(np.sqrt(1.0-chord/semiperimeter), r2[1])
    target = transfer_time*np.sqrt(2.0*MU/semiperimeter**3)
    if direction == messaging.RETROGRADE:
        r2[1] = -r2[1]
    solution, performance = solve_problem(
        r1, r2, transfer_time, 0, method, transfer_direction=direction
    )
    assert solution.valid == 1
    assert solution.validSol2 == 0
    # lambertValidator rejects six or more refinements and step errors >= 1e-8.
    assert performance.numIter < 6
    assert performance.errX < 1e-8
    np.testing.assert_allclose(performance.x, reference_x, rtol=0.0, atol=1e-12)
    reference_time = (
        lagrange_time(performance.x, lam, 0) if reference_x < 1.0 else battin_time(performance.x, lam)
    )
    np.testing.assert_allclose(reference_time, target, rtol=1e-12, atol=0.0)


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("r2_xy, transfer_time", [
    # Endpoint [m] and flight time [s].
    ((-8007153.311464242, 1134336.721226995), 1517.1851310277925),
    ((-1373963.0772790438, 21169783.315275904), 1029.9495735576572),
    ((-1160389.9429338065, 9261648.1844296623), 1347.9302728188266),
    ((7823692.80546992, 1670278.6850218216), 168.1732550120743),
], ids=["hyperbolic-upper-endpoint", "first-step", "bisection", "intermediate-izzo-starter"])
def test_converged_solution_passes_validator(method, direction, r2_xy, transfer_time):
    """Accept converged solutions with the validator's default convergence checks."""
    r1 = np.array([RADIUS, 0.0, 0.0])  # [m]
    r2 = np.array([*r2_xy, 0.0])  # [m]
    if direction == messaging.RETROGRADE:
        r2[1] = -r2[1]
    solution, performance = solve_problem(
        r1, r2, transfer_time, 0, method, transfer_direction=direction
    )
    expected_dv = np.array([1.0, 0.0, 0.0])  # [m/s]
    nav = messaging.NavTransMsgPayload()
    nav.r_BN_N = r1
    nav.v_BN_N = np.array(solution.v1_N)-expected_dv
    problem = messaging.LambertProblemMsgPayload()
    problem.mu = MU
    problem.r1_N = r1
    problem.r2_N = r2
    problem.transferTime = transfer_time
    problem.solverMethod = method
    problem.transferDirection = direction
    # Retain all input messages for both updates; readers do not own their sources.
    nav_message = messaging.NavTransMsg().write(nav)
    problem_message = messaging.LambertProblemMsg().write(problem)
    solution_message = messaging.LambertSolutionMsg().write(solution)
    performance_message = messaging.LambertPerformanceMsg().write(performance)
    validator = lambertValidator.LambertValidator()
    validator.navTransInMsg.subscribeTo(nav_message)
    validator.lambertProblemInMsg.subscribeTo(problem_message)
    validator.lambertSolutionInMsg.subscribeTo(solution_message)
    validator.lambertPerformanceInMsg.subscribeTo(performance_message)
    validator.setFinalTime(transfer_time)
    validator.setUncertaintyStates(np.zeros((6, 6)))
    validator.setUncertaintyDV(0.0)  # [m/s]
    validator.setMaxDistanceTarget(1.0)  # [m]
    validator.Reset(0)
    # Two identical solutions satisfy the default checks on successive commands.
    validator.UpdateState(0)
    validator.UpdateState(0)
    diagnostics = validator.lambertValidatorOutMsg.read()
    assert diagnostics.failedValidLambert == 0
    assert diagnostics.failedNumIterationsLambert == 0
    assert diagnostics.failedXToleranceLambert == 0
    command = validator.dvBurnCmdOutMsg.read()
    np.testing.assert_allclose(command.dvInrtlCmd, expected_dv, rtol=0.0, atol=1e-10)  # [m/s]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("angle_degrees", [1.01, 1.4455193992490614, 90.0, 175.0])  # [deg]
@pytest.mark.parametrize("time_offset", [-5e-10, -5e-13, 0.0, 5e-13, 5e-10])  # [-]
def test_parabolic_roundoff_and_nearby_times(method, direction, angle_degrees, time_offset):
    """Resolve exact and nearby parabolic transfers without a flat flight-time interval."""
    angle = np.deg2rad(angle_degrees)
    sign = -1 if direction == messaging.RETROGRADE else 1
    r1 = np.array([RADIUS, 0.0, 0.0])  # [m]
    r2 = 2.0*RADIUS/(1.0+np.cos(angle))*np.array([np.cos(angle), sign*np.sin(angle), 0.0])
    # Barker's equation gives the parabolic flight time from periapsis.
    tangent = np.tan(angle/2.0)
    parabolic_time = np.sqrt(2.0*RADIUS**3/MU)*(tangent+tangent**3/3.0)
    transfer_time = parabolic_time*(1.0+time_offset)
    chord = np.linalg.norm(r2-r1)
    semiperimeter = (RADIUS+np.linalg.norm(r2)+chord)/2.0
    lam = np.sqrt(1.0-chord/semiperimeter)
    target = transfer_time*np.sqrt(2.0*MU/semiperimeter**3)
    solution, performance = solve_problem(
        r1, r2, transfer_time, 0, method, transfer_direction=direction
    )
    assert solution.valid == 1
    assert solution.validSol2 == 0
    np.testing.assert_allclose(battin_time(performance.x, lam), target, rtol=1e-12, atol=0.0)
    if time_offset == 0.0:
        assert performance.x == 1.0
        assert performance.numIter == 0
        assert performance.errX == 0.0
        expected_velocity = [0.0, sign*np.sqrt(2.0*MU/RADIUS), 0.0]  # [m/s]
        np.testing.assert_allclose(solution.v1_N, expected_velocity, rtol=0.0, atol=1e-8)  # [m/s]
        if angle_degrees < 2.0:  # [deg]
            final_position, _ = propagate(r1, solution.v1_N, transfer_time)
            np.testing.assert_allclose(final_position, r2, rtol=0.0, atol=1e-5)  # [m]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("lam, fraction", [
    (0.9473913043478261, 0.01),
    (0.9615942028985508, 0.04379310344827586),
    (0.99, 0.21275862068965518),
])
def test_short_transfer_convergence(method, direction, lam, fraction):
    """Propagate short transfers that previously exhausted Izzo's eight iterations."""
    r1, r2, scale, _, _ = problem_geometry(lam, 1)
    if direction == messaging.RETROGRADE:
        r2[1] = -r2[1]
    time_at_zero = np.arccos(lam)+lam*np.sqrt(1.0-lam*lam)
    parabolic_time = 2.0/3.0*(1.0-lam**3)
    transfer_time = (parabolic_time+(time_at_zero-parabolic_time)*fraction)/scale
    solution, performance = solve_problem(
        r1, r2, transfer_time, 0, method, transfer_direction=direction
    )
    assert solution.valid == 1
    assert solution.validSol2 == 0
    assert np.isfinite(performance.errX) and performance.errX <= 1e-8
    final_position, final_velocity = propagate(r1, solution.v1_N, transfer_time)
    np.testing.assert_allclose(final_position, r2, rtol=0.0, atol=1e-3)  # [m]
    np.testing.assert_allclose(final_velocity, solution.v2_N, rtol=0.0, atol=1e-5)  # [m/s]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("lam", [-0.99, -0.5, 0.5, 0.99])
@pytest.mark.parametrize("revolutions", [1, 3])
@pytest.mark.parametrize("excess_time", [1e-12, 1e-10])
def test_roots_converge_near_minimum(method, direction, lam, revolutions, excess_time):
    """Resolve both roots just above the minimum using independent branch brackets."""
    r1, r2, scale, x_min, t_min = problem_geometry(lam, revolutions)
    if direction == messaging.RETROGRADE:
        r2[1] = -r2[1]
    target = t_min*(1.0+excess_time)
    solution, performance = solve_problem(
        r1, r2, target/scale, revolutions, method, transfer_direction=direction
    )
    assert solution.valid == solution.validSol2 == 1
    roots = np.array([performance.x, performance.xSol2])
    reference = [
        brentq(lambda x: lagrange_time(x, lam, revolutions)-target, -0.99, x_min, xtol=1e-14),
        brentq(lambda x: lagrange_time(x, lam, revolutions)-target, x_min, 0.99, xtol=1e-14),
    ]
    assert -1.0 < roots[0] < x_min < roots[1] < 1.0
    np.testing.assert_allclose(roots, reference, rtol=0.0, atol=2e-8)
    np.testing.assert_allclose(lagrange_time(roots, lam, revolutions), target, rtol=2e-12)
    errors = np.array([performance.errX, performance.errXSol2])
    assert np.isfinite(errors).all() and np.all(errors <= 1e-8)


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("revolutions", [0, 1])
def test_numerical_failure_and_recovery(method, direction, revolutions):
    """Zero failed solutions, then recover without reusing their roots or validity."""
    r1 = np.array([RADIUS, 0.0, 0.0])  # [m]
    sign = -1 if direction == messaging.RETROGRADE else 1
    r2 = np.array([0.0, sign*RADIUS, 0.0])  # [m]
    normal_time = 2000.0 if revolutions == 0 else 10000.0  # [s]
    unresolved_time = 1e300  # [s] The required root is closer to |x| = 1 than double precision resolves.
    module = lambertSolver.LambertSolver()
    for transfer_time in [normal_time, unresolved_time, 1.1*normal_time]:
        solution, performance = solve_problem(
            r1, r2, transfer_time, revolutions, method, module, transfer_direction=direction
        )
        if transfer_time == unresolved_time:
            assert solution.valid == solution.validSol2 == 0
            np.testing.assert_array_equal(
                [solution.v1_N, solution.v2_N, solution.v1Sol2_N, solution.v2Sol2_N], np.zeros((4, 3))
            )
            assert performance.x == performance.xSol2 == 0.0
            assert performance.numIter == performance.numIterSol2 == 0
            assert performance.errX == performance.errXSol2 == 0.0
        else:
            assert solution.valid == 1
            assert solution.validSol2 == int(revolutions > 0)
            assert np.isfinite([solution.v1_N, solution.v2_N, solution.v1Sol2_N, solution.v2Sol2_N]).all()
            if transfer_time == normal_time:
                previous_velocity = np.array(solution.v1_N)
            else:
                assert np.linalg.norm(np.array(solution.v1_N)-previous_velocity) > 1e-3  # [m/s]


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
def test_independent_branch_validity(method, direction):
    """Preserve a resolved branch when the other exceeds the evaluator's resolution.

    The long-time right root lies close to x = 1, where small root errors produce
    large flight-time errors. If resolved, it must meet the independent residual
    check. A following normal request must resolve both roots.
    """
    lam = 0.5
    r1, r2, scale, _, t_min = problem_geometry(lam, 1)
    if direction == messaging.RETROGRADE:
        r2[1] = -r2[1]
    long_time = lagrange_time(-1.0+1e-8, lam, 1)
    module = lambertSolver.LambertSolver()
    for target in [long_time, 1.1*t_min]:
        solution, performance = solve_problem(
            r1, r2, target/scale, 1, method, module, transfer_direction=direction
        )
        assert solution.valid == 1
        np.testing.assert_allclose(lagrange_time(performance.x, lam, 1), target, rtol=2e-12, atol=0.0)
        if solution.validSol2:
            np.testing.assert_allclose(lagrange_time(performance.xSol2, lam, 1), target, rtol=2e-12, atol=0.0)
        else:
            assert target == long_time
            np.testing.assert_array_equal([solution.v1Sol2_N, solution.v2Sol2_N], np.zeros((2, 3)))
            assert performance.xSol2 == performance.errXSol2 == 0.0
            assert performance.numIterSol2 == 0


@pytest.mark.parametrize("method", [messaging.GOODING, messaging.IZZO])
@pytest.mark.parametrize("lam", [-0.99, -0.5, 0.5, 0.99])
@pytest.mark.parametrize("time_offset", [-5e-10, 5e-10])  # [-]
def test_small_step_alone_is_not_convergence(method, lam, time_offset):
    """Require an independent flight-time residual even when the final step is small.

    The former near-parabolic evaluator had a flat interval. The smooth evaluator
    must resolve this request rather than treating a small step alone as success.
    """
    r1, r2, scale, _, _ = problem_geometry(lam, 1)
    target = 2.0/3.0*(1.0-lam**3)*(1.0+time_offset)
    solution, performance = solve_problem(r1, r2, target/scale, 0, method)
    assert solution.valid == 1
    assert performance.errX <= 1e-8
    np.testing.assert_allclose(battin_time(performance.x, lam), target, rtol=1e-12, atol=0.0)
