#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


import math

import numpy as np
import pytest
from Basilisk.utilities import orbitalMotion

a_tol = 1e-14  # array tolerance
s_tol = 1e-6  # scale tolerance


# Reference roots from 90-digit Decimal bisection of Kepler's equations,
# using exact conversions of the binary64 inputs. Anomalies are in radians.
@pytest.mark.parametrize("mean,eccentricity,expected", [
    (0.3, 0.999, 1.247126572242462),
    (0.1, 0.9999, 0.8535302901646385),
    (0.10008, 0.99, 0.8319005379955369),
    (1e-12, 1.0 - 1e-12, 0.0001817010532025818),
    (1e-24, math.nextafter(1.0, 0.0), 8.18424690685419e-9),
    (2.0 * math.pi, math.nextafter(1.0, 0.0), 6.28317393797836),
])
@pytest.mark.parametrize("sign", [-1, 1])
def test_M2E_convergence(mean, eccentricity, expected, sign):
    """Recover elliptic roots where Newton iteration diverged or lost precision."""
    result = orbitalMotion.M2E(sign * mean, eccentricity)
    assert math.isfinite(result)
    assert result == pytest.approx(sign * expected, rel=5e-14, abs=0.0)


@pytest.mark.parametrize("mean,eccentricity,expected", [
    (250.0, 1.5, 5.832212476255278),
    (1000.0, 1.5, 7.202614705676229),
    (1e6, 1.5, 14.103206733523901),
    (1e-12, 1.0 + 1e-12, 0.0001817010517805506),
    (1e-24, math.nextafter(1.0, 2.0), 4.437990012889989e-9),
    (1e300, 1.5, 691.0632099706655),
    (1e308, 1e308, 0.881373587019543),
])
@pytest.mark.parametrize("sign", [-1, 1])
def test_N2H_convergence(mean, eccentricity, expected, sign):
    """Recover hyperbolic roots without overflow or iteration-limit fall-through."""
    result = orbitalMotion.N2H(sign * mean, eccentricity)
    assert math.isfinite(result)
    assert result == pytest.approx(sign * expected, rel=5e-14, abs=0.0)


@pytest.mark.parametrize("eccentricity", [0.0, 0.5, 0.99, math.nextafter(1.0, 0.0)])
@pytest.mark.parametrize("turns", [-100, -1, 0, 1, 100])
def test_M2E_revolutions(eccentricity, turns):
    """Preserve unwrapped anomalies and satisfy the elliptic Kepler equation."""
    mean = 0.3 + turns * 2.0 * math.pi  # [rad]
    result = orbitalMotion.M2E(mean, eccentricity)
    expected = orbitalMotion.M2E(0.3, eccentricity) + turns * 2.0 * math.pi  # [rad]
    assert result == pytest.approx(expected, rel=0.0, abs=5e-13)
    assert result - eccentricity * math.sin(result) == pytest.approx(mean, rel=0.0, abs=5e-13)


@pytest.mark.parametrize("solver,eccentricity", [(orbitalMotion.M2E, 0.999), (orbitalMotion.N2H, 1.001)])
def test_kepler_zero_anomaly(solver, eccentricity):
    """Return the exact zero root at periapsis."""
    assert solver(0.0, eccentricity) == 0.0


@pytest.mark.parametrize("solver,eccentricity", [(orbitalMotion.M2E, 0.5), (orbitalMotion.N2H, 1.5)])
@pytest.mark.parametrize("mean", [math.nan, math.inf, -math.inf])
def test_kepler_nonfinite_mean(solver, eccentricity, mean):
    """Reject non-finite mean anomalies explicitly."""
    with pytest.raises(ValueError, match=solver.__name__):
        solver(mean, eccentricity)


@pytest.mark.parametrize("solver,eccentricity", [
    (orbitalMotion.M2E, -0.1), (orbitalMotion.M2E, 1.0),
    (orbitalMotion.N2H, 0.5), (orbitalMotion.N2H, 1.0),
    *[(solver, e) for solver in (orbitalMotion.M2E, orbitalMotion.N2H)
      for e in (math.nan, math.inf, -math.inf)],
])
def test_kepler_invalid_eccentricity(solver, eccentricity):
    """Reject eccentricities outside each solver's finite conic domain."""
    with pytest.raises(ValueError, match=solver.__name__):
        solver(0.3, eccentricity)


@pytest.mark.parametrize("solver,eccentricity", [(orbitalMotion.M2E, 0.999), (orbitalMotion.N2H, 1.001)])
def test_kepler_iteration_limit(solver, eccentricity, monkeypatch):
    """Report iteration exhaustion instead of returning an unconverged iterate."""
    monkeypatch.setattr(orbitalMotion, "maxIteration", 1)
    with pytest.raises(RuntimeError, match="failed to converge"):
        solver(0.3, eccentricity)


@pytest.mark.parametrize("eccentricity", [1.0 - 1e-10, 1.0 - 5e-13, 1.0, 1.0 + 5e-13, 1.0 + 1e-10])
@pytest.mark.parametrize("scale", [1e-6, 1.0, 1e3])
@pytest.mark.parametrize("anomaly", [-1.1, 0.0, 1.1])  # [rad]
def test_parabolic_limit(eccentricity, scale, anomaly):
    """Keep resolvable conics distinct, independent of the distance unit."""
    mu = orbitalMotion.MU_EARTH * scale**3  # [L^3/s^2]
    periapsis = 7500.0 * scale  # [L]
    elements = orbitalMotion.ClassicElements()
    elements.e = eccentricity
    elements.a = periapsis / (1.0 - eccentricity) if eccentricity != 1.0 else 0.0  # [L]
    elements.rPeriap = periapsis
    elements.i = elements.Omega = elements.omega = 0.0  # [rad]
    elements.f = anomaly
    # Independent planar conic reference, parameterized by periapsis, not a.
    parameter = periapsis * (1.0 + eccentricity)
    radius = parameter / (1.0 + eccentricity * math.cos(anomaly))
    expected_r = radius * np.array([math.cos(anomaly), math.sin(anomaly), 0.0])
    expected_v = math.sqrt(mu / parameter) * np.array([-math.sin(anomaly), eccentricity + math.cos(anomaly), 0.0])
    r, v = orbitalMotion.elem2rv(mu, elements)
    np.testing.assert_allclose(r, expected_r, rtol=3e-15, atol=1e-11 * scale)
    np.testing.assert_allclose(v, expected_v, rtol=3e-15, atol=1e-14 * scale)
    recovered = orbitalMotion.rv2elem(mu, expected_r, expected_v, canonical=True)
    assert recovered.rPeriap == pytest.approx(periapsis, rel=3e-15)
    assert recovered.e == pytest.approx(eccentricity, abs=2e-15)
    if eccentricity == 1.0:
        assert recovered.a == recovered.alpha == recovered.rApoap == 0.0
        assert recovered.e == 1.0
    else:
        assert math.copysign(1.0, recovered.alpha) == math.copysign(1.0, 1.0 - eccentricity)
        # Relative energy/a accuracy is conditioned by the distance from e=1.
        conditioning = 8.0 * np.finfo(float).eps / abs(1.0 - eccentricity)
        assert recovered.a == pytest.approx(elements.a, rel=conditioning)
    reconstructed_r, reconstructed_v = orbitalMotion.elem2rv(mu, recovered)
    tolerance = 8.0 * np.finfo(float).eps / abs(1.0 - eccentricity) if eccentricity != 1.0 else 5e-15
    np.testing.assert_allclose(reconstructed_r, expected_r, rtol=tolerance, atol=1e-10 * scale)
    np.testing.assert_allclose(reconstructed_v, expected_v, rtol=tolerance, atol=1e-13 * scale)


@pytest.mark.parametrize("speed", [-2.0, -math.sqrt(2.0), -0.6, 0.0, 0.6, math.sqrt(2.0), 2.0])  # [circular speed]
@pytest.mark.parametrize("direction", [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [2.0, -3.0, 6.0]])
@pytest.mark.parametrize("scale", [1e-6, 1e3])
def test_rectilinear_state(speed, direction, scale):
    """Reconstruct bound, escape and unbound radial states, including rest."""
    mu = orbitalMotion.MU_EARTH * scale**3  # [L^3/s^2]
    radius = 7500.0 * scale  # [L]
    unit = np.asarray(direction) / np.linalg.norm(direction)
    r = radius * unit
    v = speed * math.sqrt(mu / radius) * unit
    for inverse, forward in [(orbitalMotion.rv2elem, orbitalMotion.elem2rv),
                             (orbitalMotion.rv2elem_parab, orbitalMotion.elem2rv_parab)]:
        elements = inverse(mu, r, v)
        assert elements.e == 1.0
        assert elements.rPeriap == 0.0
        assert elements.alpha * radius == pytest.approx(2.0 - speed**2, abs=3e-15)
        if elements.alpha == 0.0:
            assert elements.a == 0.0
            assert elements.rmag == pytest.approx(radius)
        elif elements.a < 0.0:
            assert math.copysign(1.0, elements.f) == math.copysign(1.0, speed)
            # Negative a in the legacy forward adapter always denotes a planar parabola.
            forward = orbitalMotion.elem2rv
        reconstructed_r, reconstructed_v = forward(mu, elements)
        np.testing.assert_allclose(reconstructed_r, r, rtol=5e-15, atol=1e-10 * scale)
        np.testing.assert_allclose(reconstructed_v, v, rtol=5e-15, atol=1e-13 * scale)


@pytest.mark.parametrize("axis,anomaly", [(7500.0, 1e-9), (7500.0, -1e-9), (-7500.0, 1e-9),
                                         (-7500.0, -1e-9), (-7500.0, 4.0), (-7500.0, -4.0)])  # [km, rad]
def test_rectilinear_forward(axis, anomaly):
    """Avoid cancellation at collision and periodic signs for hyperbolic H."""
    elements = orbitalMotion.ClassicElements()
    elements.a = axis
    elements.e = 1.0
    elements.i = elements.Omega = elements.omega = 0.0  # [rad]
    elements.f = anomaly
    elements.rPeriap = 0.0  # [km]
    half = anomaly / 2.0
    factor = math.sin(half) if axis > 0.0 else math.sinh(half)
    expected_r = 2.0 * abs(axis) * factor**2
    expected_v = math.copysign(math.sqrt(orbitalMotion.MU_EARTH * (2.0 / expected_r - 1.0 / axis)), anomaly)
    forwards = (orbitalMotion.elem2rv, orbitalMotion.elem2rv_parab) if axis > 0.0 else (orbitalMotion.elem2rv,)
    for forward in forwards:
        r, v = forward(orbitalMotion.MU_EARTH, elements)
        np.testing.assert_allclose(r, [expected_r, 0.0, 0.0], rtol=3e-15)
        velocity_sign = -1.0 if forward is orbitalMotion.elem2rv_parab and axis > 0.0 else 1.0
        np.testing.assert_allclose(v, [velocity_sign * expected_v, 0.0, 0.0], rtol=3e-15)


def test_legacy_parabolic_adapter():
    """Preserve the negative-periapsis encoding without mutating the caller."""
    elements = orbitalMotion.ClassicElements()
    elements.a = -7500.0  # [km]; legacy parabolic encoding
    elements.e = 1.0
    elements.i = 2.4  # [rad]
    elements.Omega = 0.7  # [rad]
    elements.omega = 1.2  # [rad]
    elements.f = -1.1  # [rad]
    r, v = orbitalMotion.elem2rv_parab(orbitalMotion.MU_EARTH, elements)
    assert elements.a == -7500.0
    assert elements.rPeriap is None
    recovered = orbitalMotion.rv2elem_parab(orbitalMotion.MU_EARTH, r, v)
    assert recovered.a == pytest.approx(-7500.0)
    assert recovered.e == 1.0
    assert recovered.alpha == 0.0
    r2, v2 = orbitalMotion.elem2rv_parab(orbitalMotion.MU_EARTH, recovered)
    np.testing.assert_allclose(r2, r, rtol=5e-15)
    np.testing.assert_allclose(v2, v, rtol=5e-15)


@pytest.mark.parametrize("anomaly", [1e6, -1e6, 1e10, -1e10, 1e16, -1e16])  # [rad]
@pytest.mark.parametrize("eccentricity", [0.0, 0.4, 1.0, 1.5])
def test_large_wrapped_true_anomaly(anomaly, eccentricity):
    """Preserve the phase of the supplied angle using independent direct trigonometry."""
    elements = orbitalMotion.ClassicElements()
    elements.e = eccentricity
    elements.rPeriap = 7000.0  # [km]
    elements.a = elements.rPeriap / (1.0 - eccentricity) if eccentricity != 1.0 else 0.0  # [km]
    elements.i = elements.Omega = elements.omega = 0.0  # [rad]
    elements.f = anomaly
    parameter = elements.rPeriap * (1.0 + eccentricity)
    radius = parameter / (1.0 + eccentricity * math.cos(anomaly))
    expected_r = radius * np.array([math.cos(anomaly), math.sin(anomaly), 0.0])
    expected_v = math.sqrt(orbitalMotion.MU_EARTH / parameter) * np.array(
        [-math.sin(anomaly), eccentricity + math.cos(anomaly), 0.0])
    position, velocity = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, elements)
    np.testing.assert_allclose(position, expected_r, rtol=5e-15, atol=1e-11)
    np.testing.assert_allclose(velocity, expected_v, rtol=5e-15, atol=1e-13)


@pytest.mark.parametrize("anomaly", [1e6, -1e6, 1e10, -1e10, 1e16, -1e16])  # [rad]
def test_large_wrapped_radial_anomaly(anomaly):
    """Preserve radial position and velocity sign for large eccentric anomalies."""
    elements = orbitalMotion.ClassicElements()
    elements.a = 7500.0  # [km]
    elements.e = 1.0
    elements.i = elements.Omega = elements.omega = 0.0  # [rad]
    elements.f = anomaly
    radius = elements.a * (1.0 - math.cos(anomaly))
    speed = math.sqrt(orbitalMotion.MU_EARTH / elements.a) * math.sin(anomaly) / (1.0 - math.cos(anomaly))
    for forward, sign in ((orbitalMotion.elem2rv, 1.0), (orbitalMotion.elem2rv_parab, -1.0)):
        position, velocity = forward(orbitalMotion.MU_EARTH, elements)
        np.testing.assert_allclose(position, [radius, 0.0, 0.0], rtol=5e-15)
        np.testing.assert_allclose(velocity, [sign * speed, 0.0, 0.0], rtol=5e-15)


def test_near_parabolic_apoapsis_and_infinity():
    """Retain the small denominator and transverse velocity near f = pi."""
    elements = orbitalMotion.ClassicElements()
    elements.a = 7500.0  # [km]
    elements.e = math.nextafter(1.0, 0.0)
    elements.i = elements.Omega = elements.omega = 0.0  # [rad]
    elements.f = math.pi  # [rad]
    r, v = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, elements)
    assert r[0] == pytest.approx(-elements.a * (1.0 + elements.e), rel=3e-15)
    expected_speed = math.sqrt(orbitalMotion.MU_EARTH / elements.a * (1.0 - elements.e) / (1.0 + elements.e))
    assert v[1] == pytest.approx(-expected_speed, rel=3e-15)
    elements.a = 0.0  # [km]
    elements.e = 1.0
    elements.rPeriap = 7500.0  # [km]
    elements.f = math.pi - 1e-8  # [rad]
    r, v = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, elements)
    barker_variable = math.tan(elements.f / 2.0)
    assert np.linalg.norm(r) == pytest.approx(elements.rPeriap * (1.0 + barker_variable**2), rel=3e-15)
    assert v[1] > 0.0
    assert np.dot(v, v) == pytest.approx(2.0 * orbitalMotion.MU_EARTH / np.linalg.norm(r), rel=3e-15)


@pytest.mark.parametrize("transverse", [1e-3, 1e-10])  # [normalized speed]
def test_nearly_rectilinear_state(transverse):
    """Retain resolvable transverse motion and reject an ambiguous e rounded to one."""
    r = np.array([1.0, 0.0, 0.0])  # [L]
    v = np.array([1.0, transverse, 0.0])  # [L/s]
    mu = 1.0  # [L^3/s^2]
    if transverse == 1e-10:
        with pytest.raises(ValueError, match="eccentricity cannot resolve"):
            orbitalMotion.rv2elem(mu, r, v)
    else:
        elements = orbitalMotion.rv2elem(mu, r, v)
        assert elements.e < 1.0
        assert elements.rPeriap > 0.0
        r2, v2 = orbitalMotion.elem2rv(mu, elements)
        np.testing.assert_allclose(r2, r, atol=1e-9, rtol=1e-9)
        np.testing.assert_allclose(v2, v, atol=1e-9, rtol=1e-9)


def test_nearly_rectilinear_energy_consistency():
    """Reject a rounded eccentricity on the wrong side of the parabolic limit."""
    r = [-0.7920102500247754, 0.3704143912399991, -0.48529675727125304]  # [L]
    v = [-1.584020499625905, 0.7408287833858261, -0.9705935145425061]  # [L/s]
    mu = 1.0  # [L^3/s^2]
    # Depending on floating-point evaluation, e may round to either side of one.
    try:
        elements = orbitalMotion.rv2elem(mu, r, v)
    except ValueError as error:
        assert "eccentricity cannot resolve" in str(error)
    else:
        assert elements.a < 0.0
        assert elements.e > 1.0


@pytest.mark.parametrize("axis,eccentricity,anomaly,periapsis", [
    (7500.0, 1.0, 0.0, 0.0), (-7500.0, 1.0, 0.0, 0.0),  # collisions
    (0.0, 1.0, math.pi, 7500.0), (-7500.0, 2.0, math.pi, 0.0),  # asymptote / beyond
    (-7500.0, 2.0, 1e16, 0.0), (-7500.0, 2.0, -1e16, 0.0),  # wrapped beyond infinity
    (0.0, 1.0, 0.0, -1.0), (7500.0, 2.0, 0.0, 0.0), (-7500.0, 0.5, 0.0, 0.0),
])  # [km, dimensionless, rad, km]
def test_invalid_conic_elements(axis, eccentricity, anomaly, periapsis):
    """Reject nonphysical conic combinations and singular collision states."""
    elements = orbitalMotion.ClassicElements()
    elements.a, elements.e, elements.f, elements.rPeriap = axis, eccentricity, anomaly, periapsis
    elements.i = elements.Omega = elements.omega = 0.0  # [rad]
    with pytest.raises(ValueError, match="elem2rv"):
        orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, elements)


@pytest.mark.parametrize("mu,r,v", [(0.0, [1, 0, 0], [0, 1, 0]), (1.0, [0, 0, 0], [0, 1, 0]),
                                    (1.0, [math.nan, 0, 0], [0, 1, 0]), (1.0, [1, 0, 0], [math.inf, 0, 0])])
def test_invalid_cartesian_state(mu, r, v):
    """Report invalid states explicitly in both Python entry points."""
    for inverse in (orbitalMotion.rv2elem, orbitalMotion.rv2elem_parab):
        with pytest.raises(ValueError, match="rv2elem"):
            inverse(mu, r, v)


@pytest.mark.parametrize("eccentricity,inclination", [
    (0.0, 0.0), (0.0, math.pi), (0.3, 0.0), (0.3, math.pi),
    (0.0, 0.7), (0.0, 2.4), (0.3, 2.4),
    (0.0, 1e-8), (0.3, 1e-8), (0.0, math.pi - 1e-8), (0.3, math.pi - 1e-8),
    (0.3, 5e-13), (0.3, 2e-12), (0.3, math.pi - 5e-13), (0.3, math.pi - 2e-12),
    (5e-13, 0.7), (2e-12, 0.7), (5e-13, math.pi), (2e-12, math.pi),
])  # Eccentricity [-], inclination [rad].
@pytest.mark.parametrize("length_scale", [1e-9, 1.0, 1e3])
@pytest.mark.parametrize("true_anomaly", [0.4, 4.6])  # [rad]
def test_rv2elem_singular_orbits(eccentricity, inclination, length_scale, true_anomaly):
    """Check singular-angle conventions, orbital invariants, and Cartesian reconstruction.

    States are constructed independently from orthonormal perifocal axes. Scaling
    all lengths, velocities, and the gravitational parameter consistently must
    preserve the angular elements. Near singularities, individual periapsis and
    anomaly angles have tolerances proportional to the eccentricity conditioning.
    """
    semimajor = 7500.0 * length_scale  # [L]; L is the selected distance unit
    mu = orbitalMotion.MU_EARTH * length_scale**3  # [L^3/s^2]
    node = 1.1  # [rad]
    periapsis = 0.7  # [rad]
    sin_i = 0.0 if inclination in (0.0, math.pi) else math.sin(inclination)
    cos_i = math.cos(inclination)
    ascending = np.array([math.cos(node), math.sin(node), 0.0])
    transverse = np.array([-math.sin(node) * cos_i, math.cos(node) * cos_i, sin_i])
    periapsis_axis = math.cos(periapsis) * ascending + math.sin(periapsis) * transverse
    quadrature_axis = -math.sin(periapsis) * ascending + math.cos(periapsis) * transverse
    parameter = semimajor * (1.0 - eccentricity**2)
    radius = parameter / (1.0 + eccentricity * math.cos(true_anomaly))
    position = radius * (math.cos(true_anomaly) * periapsis_axis + math.sin(true_anomaly) * quadrature_axis)
    velocity = math.sqrt(mu / parameter) * (
        -math.sin(true_anomaly) * periapsis_axis + (eccentricity + math.cos(true_anomaly)) * quadrature_axis)

    result = orbitalMotion.rv2elem(mu, position, velocity)
    equatorial = abs(sin_i) <= 1e-12
    circular = eccentricity <= 1e-12
    direction = 1.0 if cos_i >= 0.0 else -1.0
    expected_i = (0.0 if direction > 0 else math.pi) if equatorial else inclination
    expected_node = 0.0 if equatorial else node  # [rad]
    expected_periapsis = 0.0 if circular else periapsis + (direction * node if equatorial else 0.0)  # [rad]
    expected_anomaly = true_anomaly + (periapsis + (direction * node if equatorial else 0.0) if circular else 0.0)
    angle_tolerance = 3e-14 if circular else max(3e-14, 5e-15 / eccentricity)  # [rad]
    for field in ("a", "e", "i", "Omega", "omega", "f", "alpha", "rmag", "rPeriap", "rApoap"):
        assert math.isfinite(getattr(result, field)), field
    assert result.a / semimajor == pytest.approx(1.0, abs=2e-14, rel=0.0)
    assert result.alpha * semimajor == pytest.approx(1.0, abs=2e-14, rel=0.0)
    assert result.e == pytest.approx(eccentricity, abs=2e-15, rel=0.0)
    assert result.rmag / radius == pytest.approx(1.0, abs=2e-14, rel=0.0)
    assert result.rPeriap / semimajor == pytest.approx(1.0 - eccentricity, abs=2e-14, rel=0.0)
    assert result.rApoap / semimajor == pytest.approx(1.0 + eccentricity, abs=2e-14, rel=0.0)
    assert result.i == pytest.approx(expected_i, abs=2e-15, rel=0.0)
    for actual, expected, tolerance in ((result.Omega, expected_node, 3e-14),
                                        (result.omega, expected_periapsis, angle_tolerance),
                                        (result.f, expected_anomaly, angle_tolerance)):
        assert 0.0 <= actual < 2.0 * math.pi
        assert abs(math.remainder(actual - expected, 2.0 * math.pi)) <= tolerance
    rebuilt_position, rebuilt_velocity = orbitalMotion.elem2rv(mu, result)
    np.testing.assert_allclose(rebuilt_position / semimajor, position / semimajor, rtol=0.0, atol=5e-12)
    speed_scale = math.sqrt(mu / semimajor)
    np.testing.assert_allclose(rebuilt_velocity / speed_scale, velocity / speed_scale, rtol=0.0, atol=5e-12)


def arrayEqualCheck(v1, v2, methodName):
    if len(v1) == len(v2):
        if np.allclose(v1, v2, atol=a_tol):
            return 0
        print(methodName + ' failed')
        return 1
    else:
        print('Mismatching lengths.')

def scaleEqualCheck(s1, s2, methodName):
    if np.allclose(s1, s2, atol = s_tol):
        return 0
    print(methodName + ' failed')
    return 1

def test_orbitalMotion(show_plots):
    e_count = 0

    # Function hillFrame
    rc_N = np.array([353.38362479494975, 6494.841478640714, 2507.239669788398])
    vc_N = np.array([-7.073840333019544, -0.5666429544308719, 2.6565522055197555])
    HN = orbitalMotion.hillFrame(rc_N, vc_N)
    HNTrue = np.array([[0.0506938, 0.931702, 0.35967], [-0.93404, -0.0832604,
  0.347329], [0.353553, -0.353553, 0.866025]])
    if not np.allclose(HN, HNTrue, atol=0.001):
        e_count += 1
        print("orbitalMotion.py failed hillFrame test")

    # Function rv2hill
    rc_N = np.array([353.38362479494975, 6494.841478640714, 2507.239669788398])
    vc_N = np.array([-7.073840333019544, -0.5666429544308719, 2.6565522055197555])
    rd_N = np.array([353.6672082996106, 6494.264242564805, 2507.898786238764])
    vd_N = np.array([-7.073766857682589, -0.5663665778237081, 2.65770594819381])
    rho_H, rhoPrime_H = orbitalMotion.rv2hill(rc_N, vc_N, rd_N, vd_N)
    rho_HValue = np.array([-0.286371, 0.012113, 0.875157])
    rhoPrime_HValue = np.array([0.000689358, 0.000620362, 0.000927434])
    e_count += arrayEqualCheck(rho_H, rho_HValue, 'rv2hill rho_H')
    e_count += arrayEqualCheck(rhoPrime_H, rhoPrime_HValue, 'rv2hill rhoPrime_H')

    # Function hill2rv
    rc_N = np.array([353.38362479494975, 6494.841478640714, 2507.239669788398])
    vc_N = np.array([-7.073840333019544, -0.5666429544308719, 2.6565522055197555])
    rho_H = np.array([-0.286371, 0.012113, 0.875157])
    rhoPrime_H = np.array([0.000689358, 0.000620362, 0.000927434])
    rd_N, vd_N = orbitalMotion.hill2rv(rc_N, vc_N, rho_H, rhoPrime_H)
    rd_NValue = np.array([353.6672082996106, 6494.264242564805, 2507.898786238764])
    vd_NValue = np.array([-7.073766857682589, -0.5663665778237081, 2.65770594819381])
    e_count += arrayEqualCheck(rd_N, rd_NValue, 'hill2rv rd_N')
    e_count += arrayEqualCheck(vd_N, vd_NValue, 'hill2rv vd_N')

    # Function E2f
    f1 = orbitalMotion.E2f(180, 0.5)
    E2FCValue = 3.70494  #input value (180,0.5)
    e_count += scaleEqualCheck(f1, E2FCValue, 'E2f')

    # Function E2M
    M1 = orbitalMotion.E2M(180, 0.5)
    E2MCValue = 180.401
    e_count += scaleEqualCheck(M1, E2MCValue, 'E2M')

    # Function f2E
    Ecc1 = orbitalMotion.f2E(180, 0.5)
    f2ECValue = 4.57142
    e_count += scaleEqualCheck(Ecc1, f2ECValue, 'f2E')

    # Function f2H
    H1 = orbitalMotion.f2H(180, 1.5)
    f2HCValue = -2.86601
    e_count += scaleEqualCheck(H1, f2HCValue, 'f2H')

    # Function H2f
    H2f1 = orbitalMotion.H2f(180, 1.5)
    H2fCValue = 2.30052
    e_count += scaleEqualCheck(H2f1, H2fCValue, 'H2f')

    # Function H2N
    H2N1 = orbitalMotion.H2N(180, 1.5)
    H2NCValue = 1.11704E78
    e_count += scaleEqualCheck(H2N1, H2NCValue, 'H2N')

    # Function M2E
    M2E1 = orbitalMotion.M2E(180, 0.7)
    M2ECValue = 179.629
    e_count += scaleEqualCheck(M2E1, M2ECValue, 'M2E')

    # Function N2H
    N2H1 = orbitalMotion.N2H(180, 1.5)
    N2HCValue = 5.51081
    e_count += scaleEqualCheck(N2H1, N2HCValue, 'N2H')

    # Function elem2rv_parab
    mu = 1000  # [km^3/s^2]
    elements = orbitalMotion.ClassicElements()
    elements.a = 1000  # [km]
    elements.e = 1.0
    elements.Omega = 0.15  # [rad]
    elements.omega = 0.5  # [rad]
    elements.f = 0.2  # [rad]
    elements.i = 0.2  # [rad]
    rVec, vVec = orbitalMotion.elem2rv_parab(mu, elements)
    rVecCValue = np.array([15.8971, 11.8751, 1.8986])  # [km]
    vVecCValue = np.array([-7.94852, -5.9375, -0.949294])  # [km/s]; legacy E > 0 is inward
    e_count += arrayEqualCheck(rVec, rVecCValue, 'elem2rv_parab')
    e_count += arrayEqualCheck(vVec, vVecCValue, 'elem2rv_parab')

    # Function elem2rv
    mu2 = 1000
    elements2 = orbitalMotion.ClassicElements()
    elements2.a = -1000
    elements2.e = 2
    elements2.Omega = 0.15
    elements2.omega = 0.5
    elements2.f = 0.2
    elements2.i = 0.2
    rVec2, vVec2 = orbitalMotion.elem2rv(mu2, elements2)
    rVecCValue2 = np.array([670.817, 748.53, 129.71])
    vVecCValue2 = np.array([-1.12823, 1.2716, 0.289049])
    e_count += arrayEqualCheck(rVec2, rVecCValue2, 'elem2rv')
    e_count += arrayEqualCheck(vVec2, vVecCValue2, 'elem2rv')


    # Function rv2elem_parab
    elements3 = orbitalMotion.rv2elem_parab(mu, rVec, -vVec)
    # a [km], e [-], eccentric anomaly [rad], periapsis [km]. The radial plane is arbitrary.
    rv2elem_parabCValue = np.array([1000.0, 1.0, 2.0 * math.pi - 0.2, 0.0])
    rv2elem_parab_Value = np.array([elements3.a, elements3.e, elements3.f, elements3.rPeriap])
    e_count += arrayEqualCheck(rv2elem_parab_Value, rv2elem_parabCValue, 'rv2elem_parab')

    # Function rv2elem
    mu4 = orbitalMotion.MU_EARTH  # [km^3/s^2]
    rVec4 = np.array([6538.3506963942027, 186.7227227879431, -4119.3008399778619])  # [km]
    vVec4 = np.array([1.4414106130924005, 5.588901415902356, -4.0828931566657038])  # [km/s]
    elements4 = orbitalMotion.rv2elem(mu4, rVec4, vVec4)
    # Semi-major axis [km], eccentricity [-], followed by i, f, Omega, omega [rad].
    rv2elemCValue = np.array([7500.0, 0.5, math.radians(40), math.radians(123), math.radians(133), math.radians(113)])
    rv2elemValue = np.array([elements4.a, elements4.e, elements4.i, elements4.f, elements4.Omega, elements4.omega])
    e_count += arrayEqualCheck(rv2elemValue, rv2elemCValue, 'rv2elem')

    # Function atmosphericDensity
    density1 = orbitalMotion.atmosphericDensity(500)
    density2 = orbitalMotion.atmosphericDensity(2000)
    density1CValue = 4.36496E-13
    density2CValue = 2.48886E-15
    e_count += scaleEqualCheck(density1, density1CValue, 'atmosphericDensity')
    e_count += scaleEqualCheck(density2, density2CValue, 'atmosphericDensity')


    # Function debyeLength
    debyedist1 = orbitalMotion.debyeLength(500)
    debyedist2 = orbitalMotion.debyeLength(2000)
    debyedist1CValue = 0.00454
    debyedist2CValue = 0.0396
    e_count += scaleEqualCheck(debyedist1, debyedist1CValue, 'debyeLength')


    # Function atmosphericDrag
    Cd = 0.5
    A = 50
    m = 5000
    rVec3 = np.array([10000,20000,30000])
    vVec3 = np.array([5,10,15])
    advec = orbitalMotion.atmosphericDrag(Cd,  A, m, rVec3, vVec3)
    advecCvalue = np.array([-5.39826E-15, -1.07965E-14, -1.61948E-14])
    e_count += arrayEqualCheck(advec, advecCvalue, 'atmosphericDrag')


    # Function jPerturb
    rvec = np.array([10, 20, 30])
    num = 6
    ajtot1 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_MERCURY') # CASE MERCURY
    adjtotCVaule1 = np.array([3.56356, 7.12713, 1.03458])
    e_count += arrayEqualCheck(ajtot1, adjtotCVaule1, 'jPerturb')

    ajtot2 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_VENUS') # CASE VENUS
    adjtotCVaule2 = np.array([24.022, 48.0439, 6.97412])
    e_count += arrayEqualCheck(ajtot2, adjtotCVaule2, 'jPerturb')

    ajtot3 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_EARTH') # CASE EARTH
    adjtotCVaule3 = np.array([-4.09853E9, -8.19707E9, -6.85662E9])
    e_count += arrayEqualCheck(ajtot3, adjtotCVaule3, 'jPerturb')

    ajtot4 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_MOON') # CASE MOON
    adjtotCVaule4 = np.array([1.35863, 2.71727, 0.394442])
    e_count += arrayEqualCheck(ajtot4, adjtotCVaule4, 'jPerturb')

    ajtot5 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_MARS') # CASE MARS
    adjtotCVaule5 = np.array([438.607, 877.214, 127.338])
    e_count += arrayEqualCheck(ajtot5, adjtotCVaule5, 'jPerturb')

    ajtot6 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_JUPITER') # CASE JUPITER
    adjtotCVaule6 = np.array([4.32235E9, 8.6447E9, 1.25488E9])
    e_count += arrayEqualCheck(ajtot6, adjtotCVaule6, 'jPerturb')

    ajtot7 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_URANUS') # CASE URANUS
    adjtotCVaule7 = np.array([5.73199E6, 1.1464E7, 1.66413E6])
    e_count += arrayEqualCheck(ajtot7, adjtotCVaule7, 'jPerturb')

    ajtot8 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_NEPTUNE') # CASE NEPTUNE
    adjtotCVaule8 = np.array([6.46744E6, 1.29349E7, 1.87764E6])
    e_count += arrayEqualCheck(ajtot8, adjtotCVaule8, 'jPerturb')

    # Function solarRad
    A = 3
    m = 500
    sunvec = np.array([1.2, 1.2, 1.2])
    arvec = orbitalMotion.solarRad(A, m, sunvec)
    arvecCValue = np.array([-4.732468E-12, -4.732468E-12, -4.732468E-12])
    e_count += arrayEqualCheck(arvec, arvecCValue, 'solarRad')

    # Function clMeanOscMap
    req = 300
    J2 = 1e-3
    elements = orbitalMotion.ClassicElements()
    elements.a = 1000
    elements.e = 0.2
    elements.i = 0.2
    elements.Omega = 0.15
    elements.omega = 0.5
    elements.f = 0.2
    elements_p = orbitalMotion.ClassicElements()
    orbitalMotion.clMeanOscMap(req, J2, elements, elements_p, 1)
    trueElements_p = np.array([1000.07546442015950560744386166334152,
                               0.20017786852908628358882481279579,
                               0.20000333960738947425284095515963,
                               0.15007256499303692209856819772540,
                               0.50011857315729335571319325026707,
                               0.19982315726261962174348241205735])
    mappedElements_p = np.array([elements_p.a, elements_p.e, elements_p.i,
                                 elements_p.Omega, elements_p.omega, elements_p.f])
    e_count += arrayEqualCheck(trueElements_p, mappedElements_p, 'clMeanOscMap')

    # Function clElem2eqElem
    elements_cl = orbitalMotion.ClassicElements()
    elements_eq = orbitalMotion.EquinoctialElements()
    elements_cl.a = 1000
    elements_cl.e = 0.2
    elements_cl.i = 0.2
    elements_cl.Omega = 0.15
    elements_cl.omega = 0.5
    elements_cl.f = 0.2
    orbitalMotion.clElem2eqElem(elements_cl, elements_eq)
    trueElements = np.array([1000.00000000000000000000000000000000,
                             0.12103728114720790909331071816268,
                             0.15921675970981119530023306651856,
                             0.01499382601880069713906618034116,
                             0.09920802187229026125603326136115,
                             0.78093005232114087732497864635661,
                             0.85000000000000008881784197001252])
    convertedElements = np.array([elements_eq.a, elements_eq.P1, elements_eq.P2,
                                  elements_eq.Q1, elements_eq.Q2, elements_eq.l, elements_eq.L])
    e_count += arrayEqualCheck(trueElements, convertedElements, 'clElem2eqElem')

    assert e_count < 1, str(e_count) + " functions failed in orbitalMotion.py script"


if __name__ == "__main__":
    test_orbitalMotion(False)
