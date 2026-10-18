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

"""Domain and scaling regressions for Hill and equinoctial conversions."""

import math

import numpy as np
import pytest

from Basilisk.architecture import keplerianOrbit
from Basilisk.utilities import orbitalMotion


@pytest.mark.parametrize("scale", [1e-200, 1.0, 1e160])  # Common distance-unit scaling.
def test_hill_scaled_round_trip(scale):
    """The frame and rotating velocity stay correct even when raw cross products overflow."""
    rc = scale * np.array([7000.0, 0.0, 0.0])  # [L]
    vc = scale * np.array([1.0, 0.0, 7.0])  # [L/s]
    rho = scale * np.array([2.0, 3.0, 4.0])  # [L]
    rho_prime = scale * np.array([0.01, 0.02, 0.03])  # [L/s]
    frame = orbitalMotion.hillFrameChecked(rc, vc)
    np.testing.assert_allclose(frame, [[1, 0, 0], [0, 0, 1], [0, -1, 0]], atol=1e-15)
    rd, vd = orbitalMotion.hill2rvChecked(rc, vc, rho, rho_prime)
    np.testing.assert_allclose(rd / scale, [7002.0, -4.0, 3.0], rtol=2e-15)
    np.testing.assert_allclose(vd / scale, [1.007, -0.03, 7.022], rtol=2e-15)
    actual = orbitalMotion.rv2hillChecked(rc, vc, rd, vd)
    for found, expected in zip(actual, (rho, rho_prime)):
        np.testing.assert_allclose(found / scale, expected / scale, rtol=0.0, atol=2e-12)


@pytest.mark.parametrize("position,velocity", [
    ([0, 0, 0], [0, 1, 0]), ([1, 0, 0], [0, 0, 0]),
    ([1, 0, 0], [1, 0, 0]), ([1, 0, 0], [-1, 0, 0]),
    ([1, 0, 0], [1, 1e-16, 0]), ([np.nan, 0, 0], [0, 1, 0]),
    ([1, 0, 0], [0, np.inf, 0]), ([1, 0], [0, 1, 0]),
    ([1+1j, 0, 0], [0, 1, 0]),
])  # [L], [L/s]
def test_hill_invalid_chief(position, velocity):
    """All three entry points reject chiefs that cannot define an orbital plane."""
    with pytest.raises(ValueError):
        orbitalMotion.hillFrameChecked(position, velocity)
    for transform in (orbitalMotion.hill2rvChecked, orbitalMotion.rv2hillChecked):
        with pytest.raises(ValueError):
            transform(position, velocity, [1, 2, 3], [0, 0, 0])


@pytest.mark.parametrize("transform", [orbitalMotion.hill2rvChecked, orbitalMotion.rv2hillChecked])
@pytest.mark.parametrize("argument", [2, 3])
@pytest.mark.parametrize("bad", [[np.nan, 0, 0], [0, np.inf, 0], [0, 0], [0, 0, 1j]])
def test_hill_invalid_deputy(transform, argument, bad):
    """Deputy and relative inputs must also satisfy the finite-three-vector contract."""
    arguments = [[7000, 0, 0], [0, 7, 0], [1, 2, 3], [0.01, 0.02, 0.03]]  # [km], [km/s]
    arguments[argument] = bad
    with pytest.raises(ValueError):
        transform(*arguments)


@pytest.mark.parametrize("transform", [orbitalMotion.hill2rvChecked, orbitalMotion.rv2hillChecked])
def test_hill_unrepresentable_rate(transform):
    """Finite vectors whose frame rate overflows fail explicitly."""
    with pytest.raises(ValueError, match="angular rate"):
        transform([1e-200, 0, 0], [0, 1e200, 0], [1, 0, 0], [0, 0, 0])  # [L], [L/s]


@pytest.mark.parametrize("implementation", [orbitalMotion, keplerianOrbit])
@pytest.mark.parametrize("inclination", [0.0, 0.7, math.pi-1e-10])  # [rad]
def test_equinoctial_supported_domain(implementation, inclination):
    """Circular, equatorial and resolved retrograde ellipses retain valid coordinates."""
    elements = implementation.ClassicElements()
    elements.a = 8000.0  # [km]
    elements.e = 0.0
    elements.i = inclination
    elements.Omega = 0.2  # [rad]
    elements.omega = -0.4  # [rad]
    elements.f = 0.3  # [rad]
    output = orbitalMotion.EquinoctialElements() if implementation is orbitalMotion else implementation.equinoctialElements()
    implementation.clElem2eqElemChecked(elements, output)
    assert output.a == elements.a
    assert output.P1 == output.P2 == 0.0
    np.testing.assert_allclose([output.Q1, output.Q2],
                               np.tan(inclination/2) * np.array([np.sin(0.2), np.cos(0.2)]), rtol=2e-15)
    np.testing.assert_allclose([output.l, output.L], [0.1, 0.1], atol=2e-15)


@pytest.mark.parametrize("implementation", [orbitalMotion, keplerianOrbit])
@pytest.mark.parametrize("field,value", [("a", 0.0), ("e", -0.1), ("e", 1.0), ("e", 1.5),
                                         ("i", math.pi), ("i", math.pi+0.1), ("i", -0.1)]
                         + [(field, value) for field in ("a", "e", "i", "Omega", "omega", "f")
                            for value in (math.nan, math.inf)])
def test_equinoctial_invalid_domain(implementation, field, value):
    """Reject invalid elliptic inputs and the retrograde pole without partial outputs."""
    elements = implementation.ClassicElements()
    elements.a = 8000.0  # [km]
    elements.e = 0.1
    elements.i = 0.7  # [rad]
    elements.Omega = elements.omega = elements.f = 0.0
    setattr(elements, field, value)  # Length [km], angles [rad], eccentricity [-].
    output = orbitalMotion.EquinoctialElements() if implementation is orbitalMotion else implementation.equinoctialElements()
    fields = ("a", "P1", "P2", "Q1", "Q2", "l", "L")
    for name in fields:
        setattr(output, name, 0.0)
    if implementation is orbitalMotion:
        with pytest.raises(ValueError):
            implementation.clElem2eqElemChecked(elements, output)
        assert all(getattr(output, name) == 0.0 for name in fields)
    else:
        implementation.clElem2eqElemChecked(elements, output)
        assert all(math.isnan(getattr(output, name)) for name in fields)
