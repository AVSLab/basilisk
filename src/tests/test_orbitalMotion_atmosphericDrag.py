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

"""Drag contracts, SI reference calculations, and native binding regressions."""

import math

import numpy as np
import pytest

from Basilisk.architecture import keplerianOrbit
from Basilisk.simulation import orbElemConvert, planetEphemeris
from Basilisk.utilities import orbitalMotion


POSITION = [orbitalMotion.REQ_EARTH + 400.0, 0.0, 0.0]  # [km]
VELOCITY = [3.0, -6.0, 2.0]  # [km/s]
DENSITY_400 = 2.7985217403313244e-12  # [kg/m^3]; reference from the published polynomial coefficients


@pytest.fixture(params=[None, orbElemConvert, planetEphemeris, keplerianOrbit],
                ids=["python", "orbElemConvert", "planetEphemeris", "keplerianOrbit"])
def implementation(request):
    """Exercise Python and every native export of the orbitalMotion header."""
    return request.param


def _drag(binding, cd, area, mass, position, velocity, alias=None):
    """Call drag with its mixed-unit contract and release native pointer arrays."""
    if binding is None:
        return orbitalMotion.atmosphericDrag(cd, area, mass, position, velocity)
    arrays = [orbElemConvert.new_doubleArray(3) for _ in range(3)]
    try:
        for index in range(3):
            for array, vector in zip(arrays[:2], (position, velocity)):
                orbElemConvert.doubleArray_setitem(array, index, float(vector[index]))
            orbElemConvert.doubleArray_setitem(arrays[2], index, math.nan)
        output = arrays[2 if alias is None else alias]
        binding.atmosphericDrag(cd, area, mass, arrays[0], arrays[1], output)
        return np.array([orbElemConvert.doubleArray_getitem(output, index) for index in range(3)])
    finally:
        for array in arrays:
            orbElemConvert.delete_doubleArray(array)


@pytest.mark.parametrize("cd,area,mass", [(2.2, 3.0, 120.0), (0.5, 1.0, 1000.0)])  # [-, m^2, kg]
@pytest.mark.parametrize("velocity", [VELOCITY, [-3.0, 6.0, -2.0], [0.0, 7.5, 0.0]])  # [km/s]
def test_drag_si_reference(implementation, cd, area, mass, velocity):
    """Match dynamic pressure in SI units and oppose the supplied velocity."""
    velocity_si = np.asarray(velocity) * 1000.0  # [m/s]
    speed_si = np.linalg.norm(velocity_si)
    force = 0.5 * DENSITY_400 * speed_si**2 * cd * area  # [N]
    expected = -force / mass * velocity_si / speed_si / 1000.0  # [km/s^2]
    actual = _drag(implementation, cd, area, mass, POSITION, velocity)
    np.testing.assert_allclose(actual, expected, rtol=3e-14, atol=0.0)
    assert np.dot(actual, velocity) < 0.0


@pytest.mark.parametrize("cd,area,velocity", [(2.2, 3.0, [0.0, 0.0, 0.0]),
                                           (0.0, 3.0, VELOCITY), (2.2, 0.0, VELOCITY),
                                           (0.0, 0.0, [0.0, 0.0, 0.0])])  # [-, m^2, km/s]
def test_drag_zero_limits(implementation, cd, area, velocity):
    """Valid zero speed, area, or drag coefficient produces an exact zero vector."""
    np.testing.assert_array_equal(_drag(implementation, cd, area, 100.0, POSITION, velocity), np.zeros(3))


@pytest.mark.parametrize("index,value", [(0, -1.0), (1, -1.0), (2, -1.0), (2, 0.0),
                                        *[(index, value) for index in range(3)
                                          for value in (math.nan, math.inf, -math.inf)]])
def test_drag_invalid_parameters(implementation, index, value):
    """Reject invalid coefficients and mass even at zero velocity."""
    parameters = [2.2, 3.0, 100.0]  # [-, m^2, kg]
    parameters[index] = value
    for velocity in (VELOCITY, [0.0, 0.0, 0.0]):  # [km/s]
        if implementation is None:
            with pytest.raises(ValueError, match="atmosphericDrag"):
                _drag(implementation, *parameters, POSITION, velocity)
        else:
            assert np.all(np.isnan(_drag(implementation, *parameters, POSITION, velocity)))


@pytest.mark.parametrize("position,velocity", [
    ([0.0, 0.0, 0.0], VELOCITY), ([orbitalMotion.REQ_EARTH, 0.0, 0.0], VELOCITY),
    ([orbitalMotion.REQ_EARTH - 1.0, 0.0, 0.0], VELOCITY),
    ([math.nan, 0.0, 0.0], VELOCITY), ([0.0, math.inf, 0.0], VELOCITY),
    (POSITION, [0.0, 0.0, math.nan]), (POSITION, [-math.inf, 0.0, 0.0]),
    ([1.7e308, 1.7e308, 0.0], VELOCITY), (POSITION, [1.7e308, 1.7e308, 0.0]),
])  # [km, km/s]
def test_drag_invalid_vectors(implementation, position, velocity):
    """Validate vectors and altitude before any zero-coefficient shortcut."""
    for cd in (0.0, 2.2):
        if implementation is None:
            with pytest.raises(ValueError, match="atmosphericDrag"):
                _drag(implementation, cd, 3.0, 100.0, position, velocity)
        else:
            assert np.all(np.isnan(_drag(implementation, cd, 3.0, 100.0, position, velocity)))


@pytest.mark.parametrize("index", [0, 1])
@pytest.mark.parametrize("invalid", [[1.0, 2.0], [[1.0, 2.0, 3.0]], [1.0, 2.0, 3.0, 4.0],
                                    None, [1.0 + 1j, 2.0, 3.0]])
def test_drag_python_vector_contract(index, invalid):
    """Reject malformed or complex vectors without incidental indexing errors."""
    vectors = [POSITION, VELOCITY]
    vectors[index] = invalid
    with pytest.raises(ValueError, match="atmosphericDrag"):
        orbitalMotion.atmosphericDrag(2.2, 3.0, 100.0, *vectors)


@pytest.mark.parametrize("invalid", [None, [1.0, 2.0], 1.0 + 1j, "invalid"])
def test_drag_python_scalar_contract(invalid):
    """Report non-real or nonscalar parameters with the documented exception."""
    with pytest.raises(ValueError, match="atmosphericDrag"):
        orbitalMotion.atmosphericDrag(invalid, 3.0, 100.0, POSITION, VELOCITY)


def test_drag_python_input_storage():
    """Accept sequences and strided arrays without modifying caller-owned storage."""
    position_storage = np.array([POSITION, POSITION]).T.copy()
    velocity_storage = np.array([VELOCITY, VELOCITY]).T.copy()
    position = position_storage[:, 0]
    velocity = velocity_storage[:, 0]
    actual = orbitalMotion.atmosphericDrag(np.float64(2.2), 3.0, 100.0, position, velocity)
    expected = orbitalMotion.atmosphericDrag(2.2, 3.0, 100.0, tuple(POSITION), VELOCITY)
    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(position, POSITION)
    np.testing.assert_array_equal(velocity, VELOCITY)


@pytest.mark.parametrize("implementation", [orbElemConvert, planetEphemeris, keplerianOrbit],
                         ids=["orbElemConvert", "planetEphemeris", "keplerianOrbit"])
@pytest.mark.parametrize("alias", [0, 1])
def test_drag_native_aliasing(implementation, alias):
    """Native output may overwrite either complete input vector."""
    expected = _drag(implementation, 2.2, 3.0, 100.0, POSITION, VELOCITY)
    actual = _drag(implementation, 2.2, 3.0, 100.0, POSITION, VELOCITY, alias=alias)
    np.testing.assert_array_equal(actual, expected)


def test_drag_numerical_range(implementation):
    """Avoid intermediate overflow; reject unrepresentable output and allow underflow."""
    # Each pair of tiny/large factors cancels: Cd*A/m*|v|*v = 1 along x.
    actual = _drag(implementation, 1e-300, 1.0, 1e300, POSITION, [1e300, 0.0, 0.0])
    np.testing.assert_allclose(actual, [-500.0 * DENSITY_400, 0.0, 0.0], rtol=3e-14, atol=0.0)
    # Area/mass overflows as a standalone ratio, but the acceleration is finite.
    actual = _drag(implementation, 1.0, 1e300, 1e-300, POSITION, [1e-300, 0.0, 0.0])
    np.testing.assert_allclose(actual, [-500.0 * DENSITY_400, 0.0, 0.0], rtol=3e-14, atol=0.0)
    if implementation is None:
        with pytest.raises(ValueError, match="finite numerical range"):
            _drag(implementation, 2.2, 3.0, 100.0, POSITION, [1e200, 0.0, 0.0])
    else:
        assert np.all(np.isnan(_drag(implementation, 2.2, 3.0, 100.0, POSITION, [1e200, 0.0, 0.0])))
    np.testing.assert_array_equal(
        _drag(implementation, 2.2, 3.0, 100.0, POSITION, [1e-200, 0.0, 0.0]), np.zeros(3))
    np.testing.assert_array_equal(
        _drag(implementation, 2.2, 3.0, 100.0, [1e200, 0.0, 0.0], VELOCITY), np.zeros(3))


@pytest.mark.parametrize("altitude", [50.0, 1000.0, 1000.1, 2000.0])  # [km]
def test_drag_density_extrapolation(implementation, altitude):
    """Preserve the density helper's extrapolation without a new 1000-km cutoff."""
    position = [orbitalMotion.REQ_EARTH + altitude, 0.0, 0.0]  # [km]
    density = orbitalMotion.atmosphericDensity(altitude)
    expected = -0.5 * density * 2.2 * 3.0 / 100.0 * 7000.0**2 / 1000.0  # [km/s^2]
    actual = _drag(implementation, 2.2, 3.0, 100.0, position, [0.0, 7.0, 0.0])
    assert actual[1] < 0.0
    np.testing.assert_allclose(actual, [0.0, expected, 0.0], rtol=3e-14, atol=0.0)
