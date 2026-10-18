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

"""Regression tests for the classical mean/osculating J2 mapping."""

import math

import numpy as np
import pytest

from Basilisk.architecture import keplerianOrbit
from Basilisk.utilities import orbitalMotion


FIELDS = ("a", "e", "i", "Omega", "omega", "f")


@pytest.fixture(params=[orbitalMotion, keplerianOrbit], ids=["python", "native"])
def implementation(request):
    """Exercise Python and the compiled C function through its SWIG binding."""
    return request.param


def _elements(implementation, eccentricity=0.1, inclination=0.7, anomaly=0.3):
    """Construct an ellipse using kilometers and radians."""
    elements = implementation.ClassicElements()
    elements.a = 8000.0  # [km]
    elements.e = eccentricity
    elements.i = inclination
    elements.Omega = 0.2  # [rad]
    elements.omega = 0.4  # [rad]
    elements.f = anomaly
    return elements


def _map(implementation, elements, sign, j2=orbitalMotion.J2_EARTH):
    """Return a separately allocated mapped element set."""
    mapped = implementation.ClassicElements()
    implementation.clMeanOscMap(orbitalMotion.REQ_EARTH, j2, elements, mapped, sign)
    assert all(math.isfinite(getattr(mapped, field)) for field in FIELDS)
    return mapped


def _assert_same_state(first, second, tolerance=3e-12):
    """Compare physical states without imposing unique singular angles."""
    for actual, expected in zip(orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, first),
                                orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, second)):
        np.testing.assert_allclose(actual, expected, rtol=0.0, atol=tolerance * np.linalg.norm(expected))


def _equatorial_reference(elements, sign):
    """Evaluate the planar limit in eccentricity and mean-longitude variables.

    At either equator, the plane stays fixed and the correction to mean
    longitude measured along the orbit is ``3*gamma2p*(f-M+e*sin(f))``.
    This avoids the general inclination and node corrections altogether.
    """
    e = elements.e
    f = math.atan2(math.sin(elements.f), math.cos(elements.f))
    eta = math.sqrt(1.0 - e * e)
    eccentric_anomaly = math.atan2(eta * math.sin(f), e + math.cos(f))
    mean = eccentric_anomaly - e * math.sin(eccentric_anomaly)
    gamma = sign * orbitalMotion.J2_EARTH / 2.0 * (orbitalMotion.REQ_EARTH / elements.a)**2
    gamma_prime = gamma / eta**4
    a_over_r = (1.0 + e * math.cos(f)) / eta**2
    delta_e = gamma_prime * (e * eta + e / (1.0 + eta) + 3.0 * math.cos(f)
                            + 3.0 * e * math.cos(f)**2 + e * e * math.cos(f)**3)
    e_delta_mean = -gamma_prime * eta**3 * ((a_over_r * eta)**2 + a_over_r + 1.0) * math.sin(f)
    mean_prime = mean + math.atan2(e_delta_mean, e + delta_e)
    result = _elements(orbitalMotion)
    result.a = elements.a * (1.0 + 2.0 * gamma * (a_over_r**3 - eta**-3))
    result.e = math.hypot(e + delta_e, e_delta_mean)
    result.i = elements.i
    result.Omega = elements.Omega
    result.omega = elements.omega + mean - mean_prime + 3.0 * gamma_prime * (f - mean + e * math.sin(f))
    result.f = orbitalMotion.E2f(orbitalMotion.M2E(mean_prime, result.e), result.e)
    return result


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("inclination", [0.0, 1e-12, 0.7, math.acos(1.0 / math.sqrt(5.0)),
                                       math.pi - 1e-12, math.pi])  # [rad]
def test_mean_osc_zero_j2(implementation, sign, inclination):
    """Zero perturbation is an exact identity, even at singular inclinations."""
    elements = _elements(implementation, inclination=inclination, anomaly=8.0 * math.pi + 0.3)
    expected = [getattr(elements, field) for field in FIELDS]
    mapped = _map(implementation, elements, sign, j2=0.0)
    assert [getattr(mapped, field) for field in FIELDS] == expected
    implementation.clMeanOscMap(orbitalMotion.REQ_EARTH, 0.0, elements, elements, sign)
    assert [getattr(elements, field) for field in FIELDS] == expected


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("inclination", [0.0, math.pi])  # [rad]
@pytest.mark.parametrize("eccentricity", [0.0, 0.1, 0.6])
@pytest.mark.parametrize("anomaly", [-2.4, 0.3])  # [rad]
def test_mean_osc_equatorial_limit(implementation, sign, inclination, eccentricity, anomaly):
    """Match the independent planar limit for circular and eccentric orbits."""
    elements = _elements(implementation, eccentricity, inclination, anomaly)
    mapped = _map(implementation, elements, sign)
    _assert_same_state(mapped, _equatorial_reference(elements, sign), tolerance=2e-14)
    assert mapped.i == inclination


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("inclination", [0.0, 1e-12, 1e-8, 0.7])  # [rad]
def test_mean_osc_reflection(implementation, sign, inclination):
    """Reflecting across the x-z plane must commute with the axisymmetric map."""
    original = _elements(implementation, inclination=inclination)
    reflected = _elements(implementation, inclination=math.pi - inclination)
    reflected.Omega = -original.Omega
    prograde = _map(implementation, original, sign)
    retrograde = _map(implementation, reflected, sign)
    for first, second in zip(orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, prograde),
                             orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, retrograde)):
        np.testing.assert_allclose(first * [1.0, -1.0, 1.0], second, rtol=0.0,
                                   atol=2e-14 * np.linalg.norm(first))
    if inclination > 0.0:
        # A resolvable inclination must not collapse to exactly pi.
        assert math.pi - retrograde.i == pytest.approx(prograde.i, rel=5e-4, abs=5e-16)


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("inclination", [1e-12, 1e-8])  # [rad]
def test_mean_osc_small_inclination(implementation, sign, inclination):
    """The inclination correction must approach its equatorial limit smoothly.

    For small inclination, the mapped inclination is linear in the input.
    This catches cancellation in ``1-cos(i)**2`` that otherwise drops the
    correction once cosine rounds to one.
    """
    reference_inclination = 1e-4  # [rad]
    reference = _map(implementation, _elements(implementation, inclination=reference_inclination), sign)
    mapped = _map(implementation, _elements(implementation, inclination=inclination), sign)
    assert mapped.i / inclination == pytest.approx(reference.i / reference_inclination, rel=5e-10)


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("inclination", [0.0, 0.7, math.pi])  # [rad]
@pytest.mark.parametrize("anomaly", [0.0, 0.3, -0.3, math.pi - 1e-10, math.pi, math.pi + 1e-10])  # [rad]
def test_mean_osc_revolutions(implementation, sign, inclination, anomaly):
    """Whole turns and crossings of the principal branch preserve mapped states."""
    elements = _elements(implementation, inclination=inclination, anomaly=anomaly)
    reference = _map(implementation, elements, sign)
    for turns in [-100, -2, -1, 1, 2, 100]:
        elements.f = anomaly + turns * 2.0 * math.pi  # [rad]
        mapped = _map(implementation, elements, sign)
        _assert_same_state(mapped, reference)


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("inclination", [0.0, math.pi])  # [rad]
def test_mean_osc_equatorial_node_convention(implementation, sign, inclination):
    """Equivalent node/periapsis choices at an equator yield the same state."""
    elements = _elements(implementation, inclination=inclination)
    reference = _map(implementation, elements, sign)
    delta_node = 1.2  # [rad]
    elements.Omega += delta_node
    elements.omega -= math.cos(inclination) * delta_node
    _assert_same_state(_map(implementation, elements, sign), reference, tolerance=2e-14)


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("inclination", [0.0, 1e-8, 0.7, math.pi - 1e-8, math.pi])  # [rad]
def test_mean_osc_in_place(implementation, sign, inclination):
    """Mapping supports aliased input/output and agrees between languages."""
    elements = _elements(implementation, inclination=inclination, anomaly=8.0 * math.pi + 0.3)
    reference = _map(orbitalMotion, elements, sign)
    implementation.clMeanOscMap(orbitalMotion.REQ_EARTH, orbitalMotion.J2_EARTH, elements, elements, sign)
    _assert_same_state(elements, reference, tolerance=2e-14)


@pytest.mark.parametrize("inclination", [math.acos(1/math.sqrt(5)), math.pi-math.acos(1/math.sqrt(5))])  # [rad]
@pytest.mark.parametrize("offset", [0.0, -1e-10, 1e-10])  # [rad]
@pytest.mark.parametrize("sign", [-1, 1])
def test_mean_osc_critical_domain(implementation, inclination, offset, sign):
    """Reject both critical inclinations and their numerically unresolved neighborhoods."""
    elements = _elements(implementation, inclination=inclination+offset)
    _assert_rejected_map(implementation, elements, sign=sign)


def _assert_rejected_map(implementation, elements, req=orbitalMotion.REQ_EARTH,
                         j2=orbitalMotion.J2_EARTH, sign=1):
    """Check C NaNs versus a Python exception with unchanged output, including aliasing."""
    for alias in (False, True):
        output = elements if alias else _elements(implementation)
        before = [getattr(output, field) for field in FIELDS]
        if implementation is orbitalMotion:
            with pytest.raises(ValueError):
                implementation.clMeanOscMapChecked(req, j2, elements, output, sign)
            np.testing.assert_array_equal([getattr(output, field) for field in FIELDS], before)
        else:
            implementation.clMeanOscMapChecked(req, j2, elements, output, sign)
            assert all(math.isnan(getattr(output, field)) for field in FIELDS)


@pytest.mark.parametrize("field,value", [("a", 0.0), ("a", -1.0), ("e", -0.1), ("e", 1.0),
                                         ("i", -0.1), ("i", math.pi+0.1)]
                         + [(field, value) for field in FIELDS for value in (math.nan, math.inf)])
def test_mean_osc_invalid_elements(implementation, field, value):
    """Both implementations reject nonfinite and nonelliptic inputs."""
    elements = _elements(implementation)
    setattr(elements, field, value)  # Length [km], angles [rad], eccentricity [-].
    _assert_rejected_map(implementation, elements)


@pytest.mark.parametrize("kwargs", [{"req": 0.0}, {"req": math.inf}, {"j2": math.nan}, {"sign": 0},
                                    {"sign": math.nan}, {"j2": 1e4}, {"req": 1e308}])
def test_mean_osc_invalid_parameters_and_outputs(implementation, kwargs):
    """Invalid parameters and perturbations producing nonphysical outputs fail explicitly."""
    _assert_rejected_map(implementation, _elements(implementation), **kwargs)
