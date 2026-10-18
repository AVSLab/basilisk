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

"""Independent zonal-potential references and tests of all native export paths."""

import math
import warnings

import numpy as np
import pytest
from numpy.polynomial.legendre import Legendre

from Basilisk.architecture import keplerianOrbit
from Basilisk.simulation import orbElemConvert, planetEphemeris
from Basilisk.utilities import orbitalMotion, deprecated


BODIES = ["MERCURY", "VENUS", "EARTH", "MOON", "MARS", "JUPITER", "SATURN", "URANUS", "NEPTUNE", "SUN", "PLUTO"]
BINDINGS = [orbElemConvert, planetEphemeris, keplerianOrbit]
IMPLEMENTATIONS = [None, *BINDINGS]


def _potential_gradient(position, degree, body):
    """Differentiate the unnormalized Legendre zonal potential analytically."""
    result = np.zeros(3)
    if body in ("SUN", "PLUTO"):
        return result
    mu = getattr(orbitalMotion, "MU_" + body)  # [km^3/s^2]
    reference_radius = getattr(orbitalMotion, "REQ_" + body)  # [km]
    radius = np.linalg.norm(position)
    direction = position / radius
    latitude_sine = direction[2]
    for order in range(2, degree + 1):
        coefficient = getattr(orbitalMotion, f"J{order}_{body}", 0.0)
        polynomial = Legendre.basis(order)
        value = polynomial(latitude_sine)
        derivative = polynomial.deriv()(latitude_sine)
        scale = mu / radius**2 * coefficient * (reference_radius / radius)**order
        result[:2] += scale * direction[:2] * ((order + 1) * value + latitude_sine * derivative)
        result[2] += scale * ((order + 1) * latitude_sine * value - (1.0 - latitude_sine**2) * derivative)
    return result


def _native_result(binding, position, degree, planet=None, legacy_call=False):
    """Exercise the generated pointer and enum wrappers, releasing both arrays."""
    input_array = orbElemConvert.new_doubleArray(3)
    output_array = orbElemConvert.new_doubleArray(3)
    try:
        for index in range(3):
            orbElemConvert.doubleArray_setitem(input_array, index, float(position[index]))
            orbElemConvert.doubleArray_setitem(output_array, index, math.nan)
        if legacy_call:
            binding.jPerturb(input_array, degree, output_array, planet)
        elif planet is None:
            binding.jPerturb(input_array, degree, output_array)
        else:
            binding.jPerturbForPlanet(input_array, degree, output_array, planet)
        return np.array([orbElemConvert.doubleArray_getitem(output_array, index) for index in range(3)])
    finally:
        orbElemConvert.delete_doubleArray(input_array)
        orbElemConvert.delete_doubleArray(output_array)


@pytest.mark.parametrize("binding", IMPLEMENTATIONS,
                         ids=["python", "orbElemConvert", "planetEphemeris", "keplerianOrbit"])
@pytest.mark.parametrize("body", BODIES)
@pytest.mark.parametrize("degree", range(2, 7))
def test_zonal_potential_reference(binding, body, degree):
    """Check each supported body and degree, including poles, against the potential gradient."""
    radius = 1.3 * getattr(orbitalMotion, "REQ_" + body)  # [km]; outside the reference surface
    for direction in ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [2.0 / 3.0, -1.0 / 3.0, -2.0 / 3.0]):
        position = radius * np.array(direction)
        expected = _potential_gradient(position, degree, body)
        if binding is None:
            actual = orbitalMotion.jPerturb(position, degree, "CELESTIAL_" + body, strict=True)
        else:
            actual = _native_result(binding, position, degree, getattr(binding, "CELESTIAL_" + body))
        assert isinstance(actual, np.ndarray)
        assert actual.shape == (3,)
        if body in ("SUN", "PLUTO"):
            np.testing.assert_array_equal(actual, np.zeros(3))
        else:
            np.testing.assert_allclose(actual, expected, rtol=2e-14, atol=np.linalg.norm(expected) * 2e-15)


@pytest.mark.parametrize("binding", IMPLEMENTATIONS,
                         ids=["python", "orbElemConvert", "planetEphemeris", "keplerianOrbit"])
def test_default_is_earth(binding):
    """The default call must select Earth without consuming a nonexistent argument."""
    position = np.array([6200.0, 100.0, 2000.0])  # [km]
    expected = _potential_gradient(position, 6, "EARTH")
    actual = orbitalMotion.jPerturb(position, 6, strict=True) if binding is None else _native_result(binding, position, 6)
    np.testing.assert_allclose(actual, expected, rtol=2e-14, atol=1e-20)  # [km/s^2]


@pytest.mark.parametrize("binding", BINDINGS, ids=["orbElemConvert", "planetEphemeris", "keplerianOrbit"])
def test_legacy_native_planet_argument_is_not_discarded(binding):
    """Honor explicit planets without deprecating existing Python calls."""
    position = [7000.0, 0.0, 0.0]  # [km]
    with warnings.catch_warnings():
        warnings.simplefilter("error", deprecated.BSKDeprecationWarning)
        warnings.simplefilter("error", deprecated.BSKUrgentDeprecationWarning)
        actual = _native_result(binding, position, 2, binding.CELESTIAL_MARS, legacy_call=True)
    np.testing.assert_allclose(actual, _potential_gradient(position, 2, "MARS"), rtol=2e-14)


@pytest.mark.parametrize("body", ["EARTH", "SATURN", "SUN", "PLUTO"])
@pytest.mark.parametrize("degree", [-1, 0, 1, 7, 99])
def test_invalid_degree(body, degree):
    """Validate degree even when the selected approximation has zero coefficients."""
    position = [1000000.0, 0.0, 0.0]  # [km]
    with pytest.raises(ValueError, match="jPerturb"):
        orbitalMotion.jPerturb(position, degree, "CELESTIAL_" + body, strict=True)
    for binding in BINDINGS:
        assert np.all(np.isnan(_native_result(binding, position, degree, getattr(binding, "CELESTIAL_" + body))))


@pytest.mark.parametrize("degree", [2.0, 2.5, True, "2", None, math.nan, math.inf])
def test_python_degree_type(degree):
    """Reject noninteger degree values instead of silently treating them as a cutoff."""
    with pytest.raises(ValueError, match="jPerturb"):
        orbitalMotion.jPerturb([7000.0, 0.0, 0.0], degree, strict=True)


@pytest.mark.parametrize("body", ["PHOBOS", "DEIMOS", "TYPO"])
def test_unsupported_body(body):
    """Never substitute Earth's gravity coefficients for an unsupported body."""
    position = [7000.0, 0.0, 0.0]  # [km]
    with pytest.raises(ValueError, match="jPerturb"):
        orbitalMotion.jPerturb(position, 2, "CELESTIAL_" + body, strict=True)
    for binding in BINDINGS:
        planet = getattr(binding, "CELESTIAL_" + body, binding.MAX_CELESTIAL)
        assert np.all(np.isnan(_native_result(binding, position, 2, planet)))


@pytest.mark.parametrize("position", [[0.0, 0.0, 0.0], [math.nan, 0.0, 0.0], [0.0, math.inf, 0.0]])  # [km]
@pytest.mark.parametrize("body", ["EARTH", "SUN", "PLUTO"])
def test_invalid_position(position, body):
    """Reject zero-radius and nonfinite states before evaluating any model."""
    with pytest.raises(ValueError, match="jPerturb"):
        orbitalMotion.jPerturb(position, 2, "CELESTIAL_" + body, strict=True)
    for binding in BINDINGS:
        assert np.all(np.isnan(_native_result(binding, position, 2, getattr(binding, "CELESTIAL_" + body))))


@pytest.mark.parametrize("position", [[7000.0, 0.0], [[7000.0, 0.0, 0.0]], [7000.0, 0.0, 0.0, 0.0]])  # [km]
def test_python_position_shape(position):
    """Require exactly one three-component Cartesian vector."""
    with pytest.raises(ValueError, match="jPerturb"):
        orbitalMotion.jPerturb(position, 2, strict=True)


def test_numpy_inputs_and_degree():
    """Accept a NumPy integer degree and leave caller-owned arrays unchanged."""
    position = np.array([6200.0, 100.0, 2000.0])  # [km]
    original = position.copy()
    actual = orbitalMotion.jPerturb(position, np.int64(6), strict=True)
    np.testing.assert_array_equal(position, original)
    np.testing.assert_allclose(actual, _potential_gradient(position, 6, "EARTH"), rtol=2e-14)


@pytest.mark.parametrize("body", ["EARTH", "SATURN"])
def test_numerical_range(body):
    """Report overflow consistently and allow distant accelerations to underflow to zero."""
    position = [1e-100, 0.0, 0.0]  # [km]; intentionally outside the numerical range
    with pytest.raises(ValueError, match="finite numerical range"):
        orbitalMotion.jPerturb(position, 6, "CELESTIAL_" + body, strict=True)
    for binding in BINDINGS:
        assert np.all(np.isnan(_native_result(binding, position, 6, getattr(binding, "CELESTIAL_" + body))))
    distant_position = [1e200, 0.0, 0.0]  # [km]
    np.testing.assert_array_equal(orbitalMotion.jPerturb(distant_position, 6, "CELESTIAL_" + body, strict=True), np.zeros(3))
    for binding in BINDINGS:
        np.testing.assert_array_equal(
            _native_result(binding, distant_position, 6, getattr(binding, "CELESTIAL_" + body)), np.zeros(3))
