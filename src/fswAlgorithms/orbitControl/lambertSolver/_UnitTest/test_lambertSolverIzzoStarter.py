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
"""Check the intermediate Izzo starter and its effect on solver convergence."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import brentq

from Basilisk.architecture import messaging
from test_lambertSolverConvergence import battin_time
from test_lambertSolverMultiRev import MU, RADIUS, lagrange_time, solve_problem


@pytest.fixture(scope="module")
def izzo_reference():
    """Load the local reference without colliding with other ``Support`` packages."""
    path = Path(__file__).parent / "Support" / "IzzoLambert.py"
    spec = importlib.util.spec_from_file_location("lambert_izzo_reference", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.IzzoSolve


def intermediate_problem(lam, fraction):
    """Place a zero-revolution request along the logarithmic time interval.

    ``fraction = 0`` corresponds to the time at ``x = 0`` and
    ``fraction = 1`` to the parabolic time at ``x = 1``. Values outside
    that interval exercise the adjacent starter branches.
    """
    angle = np.pi - 4.0*np.arctan(lam)  # [rad]
    r1 = np.array([RADIUS, 0.0, 0.0])  # [m]
    r2 = RADIUS*np.array([np.cos(angle), np.sin(angle), 0.0])  # [m]
    semiperimeter = (2.0*RADIUS + np.linalg.norm(r2-r1))/2.0  # [m]
    time_scale = np.sqrt(2.0*MU/semiperimeter**3)  # [1/s]
    time_at_zero = np.arccos(lam) + lam*np.sqrt(1.0-lam*lam)  # [-]
    parabolic_time = 2.0/3.0*(1.0-lam**3)  # [-]
    target = time_at_zero*(parabolic_time/time_at_zero)**fraction  # [-]
    return r1, r2, target/time_scale, target


@pytest.mark.parametrize("lam", [-0.99, -0.5, 0.1, 0.5, 0.9, 0.99])
@pytest.mark.parametrize("fraction", [0.0, 1e-6, 0.1, 0.5, 0.9, 1.0-1e-6, 1.0])
def test_reference_starter_logarithmic_interpolation(monkeypatch, izzo_reference, lam, fraction):
    """Inspect the reference starter before refinement can hide a wrong exponent."""
    starters = []

    def capture_starter(self, target, x0, revolutions, tolerance, max_iterations):
        """Record the starter without refining it in the Python reference solver."""
        assert revolutions == 0
        starters.append(x0)
        return 0

    monkeypatch.setattr(izzo_reference, "householder", capture_starter)
    r1, r2, transfer_time, _ = intermediate_problem(lam, fraction)
    izzo_reference(r1, r2, transfer_time, MU, max_multiRev=0)
    # Linear interpolation in log(1+x) gives 0, log(2)/2, log(2) at
    # the endpoints and midpoint, independent of the geometry parameter.
    expected = np.expm1(fraction*np.log(2.0))
    np.testing.assert_allclose(starters, [expected], rtol=0.0, atol=2e-14)


def test_reference_rounded_parabolic_starter(izzo_reference):
    """Run refinement when the intermediate starter rounds to exactly one."""
    r1 = np.array([8000000.0, 0.0, 0.0])  # [m]
    r2 = np.array([2240000.0, -7680000.0, 0.0])  # [m]
    transfer_time = 1216.4446013793424  # [s]
    # This request has lambda = -0.5 and a flight time just above T_1.
    # Previously the rounded starter caused 0/0 in the unmocked refinement.
    with np.errstate(divide="raise", invalid="raise", over="raise"):
        reference = izzo_reference(r1, r2, transfer_time, MU, max_multiRev=0)
    np.testing.assert_allclose(reference.x, [1.0], rtol=0.0, atol=1e-14)
    assert np.isfinite([reference.v1, reference.v2]).all()
    np.testing.assert_allclose(
        battin_time(reference.x[0], reference.lambd), reference.T, rtol=2e-14, atol=0.0
    )


@pytest.mark.parametrize("lam", [-0.99, -0.5, 0.1, 0.5, 0.9, 0.99])
@pytest.mark.parametrize("time_offset", [-1e-7, -1e-12, -2e-16, 0.0, 2e-16, 1e-12, 1e-7])  # [-]
def test_reference_parabolic_boundary(izzo_reference, lam, time_offset):
    """Keep the actual reference solver finite and accurate on both sides of the boundary."""
    r1, r2, parabolic_time, _ = intermediate_problem(lam, 1.0)
    transfer_time = parabolic_time*(1.0+time_offset)  # [s]
    with np.errstate(divide="raise", invalid="raise", over="raise"):
        reference = izzo_reference(r1, r2, transfer_time, MU, max_multiRev=0)
        analytic_time = reference.x2tof(1.0, 0)
    assert np.isfinite([reference.x[0], *reference.v1[0], *reference.v2[0]]).all()
    np.testing.assert_allclose(analytic_time, 2.0/3.0*(1.0-reference.lambd**3), rtol=2e-15)
    np.testing.assert_allclose(
        battin_time(reference.x[0], reference.lambd), reference.T, rtol=2e-12, atol=0.0
    )
    if abs(time_offset) >= 1e-12:
        # Nearby transfers must be refined rather than flattened to x = 1.
        assert (reference.x[0]-1.0)*time_offset < 0.0
    else:
        np.testing.assert_allclose(reference.x, [1.0], rtol=0.0, atol=2e-14)
        semiperimeter = (np.linalg.norm(r1)+np.linalg.norm(r2)+np.linalg.norm(r2-r1))/2.0  # [m]
        speed_scale = np.sqrt(MU*semiperimeter/2.0)/RADIUS  # [m/s]
        radial2 = r2/RADIUS
        tangent2 = np.cross([0.0, 0.0, 1.0], radial2)
        expected_v1 = speed_scale*np.array([lam-1.0, lam+1.0, 0.0])  # [m/s]
        expected_v2 = speed_scale*((1.0-lam)*radial2+(1.0+lam)*tangent2)  # [m/s]
        np.testing.assert_allclose(reference.v1[0], expected_v1, rtol=2e-12, atol=1e-9)  # [m/s]
        np.testing.assert_allclose(reference.v2[0], expected_v2, rtol=2e-12, atol=1e-9)  # [m/s]


@pytest.mark.parametrize("direction", [messaging.PROGRADE, messaging.RETROGRADE])
@pytest.mark.parametrize("lam", [-0.99, -0.5, 0.1, 0.5, 0.9, 0.99])
@pytest.mark.parametrize("fraction", [-1e-6, 1e-6, 0.1, 0.5, 0.9, 1.0-1e-6, 1.0+1e-6])
def test_intermediate_izzo_convergence(direction, lam, fraction):
    """Resolve the intermediate interval and its boundaries within the validator budget.

    An incorrect starter can still converge under the root safeguards, but
    requests just above the parabolic flight time previously needed six or
    more refinements and were rejected by the validator's default policy.
    Root and residual checks use independent Lagrange and Battin evaluators.
    """
    r1, r2, transfer_time, target = intermediate_problem(lam, fraction)
    if direction == messaging.RETROGRADE:
        r2[1] = -r2[1]
    solution, performance = solve_problem(
        r1, r2, transfer_time, 0, messaging.IZZO, transfer_direction=direction
    )
    assert solution.valid == 1
    assert solution.validSol2 == 0
    assert performance.numIter < 6
    assert performance.errX < 1e-8

    def independent_time(x):
        """Avoid cancellation in the elliptic formula near the parabolic root."""
        return lagrange_time(x, lam, 0) if x < 0.5 else battin_time(x, lam)

    reference_x = brentq(lambda x: independent_time(x)-target, -0.1, 1.1, xtol=1e-14)
    np.testing.assert_allclose(performance.x, reference_x, rtol=0.0, atol=2e-12)
    np.testing.assert_allclose(independent_time(performance.x), target, rtol=2e-12, atol=0.0)
    assert np.isfinite([solution.v1_N, solution.v2_N]).all()
    expected_sign = -1 if direction == messaging.RETROGRADE else 1
    assert expected_sign*np.cross(r1, solution.v1_N)[2] > 0.0
