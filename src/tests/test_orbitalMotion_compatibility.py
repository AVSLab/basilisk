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

"""Public type and convention compatibility regressions for issue 1629."""

import math
import warnings
from types import SimpleNamespace

import numpy as np
import pytest

from Basilisk.architecture import keplerianOrbit
from Basilisk.simulation import orbElemConvert, planetEphemeris
from Basilisk.utilities import deprecated, orbitalMotion as om


@pytest.fixture(autouse=True)
def reject_deprecation_warnings():
    """Existing orbital APIs remain supported without requiring caller migration."""
    with warnings.catch_warnings():
        warnings.simplefilter("error", deprecated.BSKDeprecationWarning)
        warnings.simplefilter("error", deprecated.BSKUrgentDeprecationWarning)
        yield


def _elements(implementation=om, eccentricity=0.1, inclination=0.7):
    """Build an ordinary orbit with consistent kilometer and radian units."""
    elements = implementation.ClassicElements()
    elements.a = 8000.0 if eccentricity < 1.0 else -8000.0  # [km]
    elements.e = eccentricity
    elements.i = inclination  # [rad]
    elements.Omega = -0.4  # [rad]
    elements.omega = -0.6  # [rad]
    elements.f = -0.3  # [rad]
    return elements


@pytest.mark.parametrize("eccentricity", [0.0, 0.1, 1.5])
@pytest.mark.parametrize("inverse", [om.rv2elem, om.rv2elem_parab])
def test_numpy_element_scalar_contract(eccentricity, inverse):
    """Retain scalar dtype methods and the list subtraction used by existing examples."""
    source = _elements(eccentricity=eccentricity)
    position, velocity = om.elem2rv(om.MU_EARTH, source)
    result = inverse(om.MU_EARTH, position, velocity)
    for name in ("a", "e", "rmag", "alpha", "rPeriap", "rApoap"):
        value = getattr(result, name)
        assert type(value) is np.float64
        assert value.shape == ()
        assert value.dtype == np.dtype("float64")
        assert value.astype(float) == value
    # The original debris-reorbit plotting expression, without an array coercion.
    np.testing.assert_allclose([result.a, result.a + 0.1, result.a + 0.25] - result.a,
                               [0.0, 0.1, 0.25], atol=2e-12)  # [km]


@pytest.mark.parametrize("solver,eccentricity", [(om.M2E, 0.4), (om.N2H, 1.4)])
@pytest.mark.parametrize("mean", [0.0, 0.3])  # [rad]
@pytest.mark.parametrize("scalar", [np.float32, np.float64])
@pytest.mark.parametrize("argument", [0, 1])
@pytest.mark.parametrize("zero_dimensional", [False, True])
def test_kepler_numpy_scalar_contract(solver, eccentricity, mean, scalar, argument, zero_dimensional):
    """Either NumPy input retains NumPy arithmetic, including the zero-anomaly shortcut."""
    arguments = [mean, eccentricity]
    arguments[argument] = scalar(arguments[argument])
    if zero_dimensional:
        arguments[argument] = np.asarray(arguments[argument])
    original = float(arguments[argument])
    result = solver(*arguments)
    assert float(arguments[argument]) == original
    if zero_dimensional and argument == 0:
        assert isinstance(result, np.ndarray)
        assert result.shape == ()
        assert result.dtype == np.dtype(scalar)
    else:
        assert type(result) is scalar
    reference = solver(*(float(value) for value in arguments))
    assert result == scalar(reference)


@pytest.mark.parametrize("anomaly", [-0.6, 0.0, 0.6])  # [rad]
def test_parabolic_encoding_compatibility(anomaly):
    """Default Python sentinels survive; canonical encoding is an explicit opt-in."""
    source = _elements()
    source.a = 0.0  # [km]
    source.e = 1.0
    source.rPeriap = 7500.0  # [km]
    source.f = anomaly
    position, velocity = om.elem2rv(om.MU_EARTH, source)
    old = om.rv2elem(om.MU_EARTH, position, velocity)
    assert old.a == pytest.approx(-source.rPeriap)
    assert old.rApoap == -1.0
    assert old.alpha == 0.0
    canonical = om.rv2elem(om.MU_EARTH, position, velocity, canonical=True)
    assert canonical.a == canonical.rApoap == 0.0
    assert canonical.rPeriap == pytest.approx(source.rPeriap)
    for inverse, forward in ((old, om.elem2rv_parab), (canonical, om.elem2rv)):
        recovered = forward(om.MU_EARTH, inverse)
        for actual, expected in zip(recovered, (position, velocity)):
            np.testing.assert_allclose(actual, expected, atol=1e-11, rtol=5e-15)


def test_radial_adapter_velocity_convention():
    """Legacy radial E remains inward-positive; the new core convention is explicit."""
    source = _elements()
    source.e = 1.0
    source.f = 0.4  # [rad]
    position, velocity = om.elem2rv_parab(om.MU_EARTH, source)
    assert np.dot(position, velocity) < 0.0
    recovered = om.rv2elem_parab(om.MU_EARTH, position, velocity)
    assert recovered.f == pytest.approx(source.f, abs=3e-15)
    canonical = om.rv2elem(om.MU_EARTH, position, velocity, canonical=True)
    assert math.sin(canonical.f) < 0.0
    assert canonical.rPeriap == 0.0  # e = 1 also describes rectilinear motion.


@pytest.mark.parametrize("factory,auxiliary", [
    (factory, auxiliary)
    for factory in (om.ClassicElements, SimpleNamespace, orbElemConvert.ClassicElements,
                    planetEphemeris.ClassicElements, keplerianOrbit.ClassicElements)
    for auxiliary in ("default", 0.0, 1234.0)
] + [(om.ClassicElements, None), (SimpleNamespace, None)])  # Auxiliary sentinels: alpha [1/km], others [km].
@pytest.mark.parametrize("axis", [-7500.0, 7500.0])  # [km]
@pytest.mark.parametrize("anomaly", [-0.4, 0.0, 0.4])  # [rad]
def test_parabolic_adapter_input_storage(factory, auxiliary, axis, anomaly):
    """Retain legacy conics and leave Python/SWIG input fields intact on repeated calls."""
    elements = factory()
    elements.a = axis
    elements.e = 1.0
    elements.i = elements.Omega = elements.omega = 0.0  # [rad]
    elements.f = anomaly
    if auxiliary != "default":
        for field in ("rmag", "alpha", "rPeriap", "rApoap"):
            setattr(elements, field, auxiliary)
    original = {field: getattr(elements, field) for field in om.ClassicElements.__slots__
                if hasattr(elements, field)}
    if axis < 0.0:
        # The original adapter uses -a as periapsis, ignoring auxiliary fields.
        parameter = -2.0 * axis
        radius = parameter / (1.0 + math.cos(anomaly))
        expected_r = radius * np.array([math.cos(anomaly), math.sin(anomaly), 0.0])
        expected_v = math.sqrt(om.MU_EARTH / parameter) * np.array(
            [-math.sin(anomaly), 1.0 + math.cos(anomaly), 0.0])
    elif anomaly != 0.0:
        radius = axis * (1.0 - math.cos(anomaly))
        speed = math.sqrt(om.MU_EARTH * (2.0 / radius - 1.0 / axis))
        expected_r = [radius, 0.0, 0.0]
        expected_v = [-math.copysign(speed, math.sin(anomaly)), 0.0, 0.0]
    for _ in range(2):
        if axis > 0.0 and anomaly == 0.0:
            with pytest.raises(ValueError, match="collision"):
                om.elem2rv_parab(om.MU_EARTH, elements)
        else:
            position, velocity = om.elem2rv_parab(om.MU_EARTH, elements)
            np.testing.assert_allclose(position, expected_r, rtol=3e-15, atol=1e-11)
            np.testing.assert_allclose(velocity, expected_v, rtol=3e-15, atol=1e-13)
        assert {field: getattr(elements, field) for field in original} == original
        for field in set(om.ClassicElements.__slots__) - original.keys():
            assert not hasattr(elements, field)


@pytest.mark.parametrize("eccentricity", [0.0, 0.1, 1.5])
def test_signed_legacy_adapter_angles(eccentricity):
    """The legacy adapter retains signed angles and its circular periapsis convention."""
    source = _elements(eccentricity=eccentricity)
    state = om.elem2rv(om.MU_EARTH, source)
    output = om.rv2elem_parab(om.MU_EARTH, *state)
    assert output.Omega == pytest.approx(source.Omega, abs=3e-15)
    if eccentricity == 0.0:
        assert output.f == 0.0
        assert output.omega == pytest.approx(source.omega + source.f, abs=3e-15)
    else:
        assert output.omega == pytest.approx(source.omega, abs=3e-15)
        assert output.f == pytest.approx(source.f, abs=3e-15)
    for actual, expected in zip(om.elem2rv_parab(om.MU_EARTH, output), state):
        np.testing.assert_allclose(actual, expected, rtol=5e-15, atol=1e-11)


@pytest.mark.parametrize("inclination", [0.0, math.pi])  # [rad]
def test_inclination_endpoints_remain_valid(inclination):
    """Both equators are valid inclinations; coordinate-specific singularities remain separate."""
    elements = _elements(inclination=inclination)
    state = om.elem2rv(om.MU_EARTH, elements)
    assert all(np.isfinite(value).all() for value in state)
    native = _elements(keplerianOrbit, inclination=inclination)
    orbit = keplerianOrbit.KeplerianOrbit(native, om.MU_EARTH)
    assert orbit.i() == inclination
