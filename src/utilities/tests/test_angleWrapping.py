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

"""Check angle wrapping across multiple revolutions and interval boundaries."""

import numpy as np
import pytest

from Basilisk.utilities import RigidBodyKinematics as rbk


@pytest.fixture(params=["python", "numba"])
def wrapping_backend(request):
    """Check the ordinary Python function and the compiled Numba implementation."""
    if request.param == "numba":
        pytest.importorskip("numba")
        pytest.importorskip("scipy")
        from Basilisk.utilities import RigidBodyKinematicsNumba
        return RigidBodyKinematicsNumba
    return rbk


@pytest.mark.parametrize("turns", [-1000000, -100, -10, -3, -2, -1, 0, 1, 2, 3, 10, 100, 1000000])
def test_angle_multiple_revolutions(wrapping_backend, turns):
    """Remove whole revolutions while retaining the known fractional angle."""
    for offset in [-2.9, -0.2, 0.0, 0.2, 2.9]:  # [rad]
        angle = turns * (2.0 * np.pi) + offset  # [rad]
        result = wrapping_backend.Picheck(angle)
        assert -np.pi <= result <= np.pi
        # Allow rounding from forming a large input before it reaches Picheck.
        tolerance = 4.0 * np.spacing(max(abs(angle), 1.0))  # [rad]
        assert abs(result - offset) <= tolerance
        assert wrapping_backend.Picheck(result) == result


@pytest.mark.parametrize("angle", [
    0.0, -0.0, 1e-12, -1e-12, 0.2, -0.2, np.pi, -np.pi,
    np.nextafter(np.pi, 0.0), np.nextafter(-np.pi, 0.0),
])  # [rad]
def test_angle_already_wrapped(wrapping_backend, angle):
    """Leave angles within the closed interval unchanged, including signed zero."""
    result = wrapping_backend.Picheck(angle)
    assert result == angle
    assert np.signbit(result) == np.signbit(angle)


@pytest.mark.parametrize("multiple", [1, 3, 5])
def test_angle_half_turn_boundaries(wrapping_backend, multiple):
    """Preserve exact half-turn signs and wrap immediately across each boundary."""
    boundary = multiple * np.pi  # [rad]
    for sign in [-1.0, 1.0]:
        angle = sign * boundary
        assert wrapping_backend.Picheck(angle) == sign * np.pi
        inside = wrapping_backend.Picheck(np.nextafter(angle, 0.0))
        outside = wrapping_backend.Picheck(np.nextafter(angle, sign * np.inf))
        assert 0.0 < sign * inside < np.pi
        assert -np.pi < sign * outside < 0.0


@pytest.mark.parametrize("multiple", [2, 4, 8])
def test_angle_full_turn_signed_zero(wrapping_backend, multiple):
    """Keep the sign of zero when removing exact whole revolutions."""
    for sign in [-1.0, 1.0]:
        result = wrapping_backend.Picheck(sign * multiple * np.pi)
        assert result == 0.0
        assert np.signbit(result) == (sign < 0.0)


@pytest.mark.parametrize("angle", [np.nan, np.inf, -np.inf])
def test_angle_nonfinite(wrapping_backend, angle):
    """Return NaN consistently when an angle has no finite wrapped value."""
    assert np.isnan(wrapping_backend.Picheck(angle))
