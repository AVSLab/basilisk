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

"""Regressions for orbital elements, calendar dates, ephemerides, and rotations."""

from datetime import date, timedelta
from decimal import Decimal

import numpy as np
import pytest

from Basilisk.architecture import astroConstants
from Basilisk.utilities import astroFunctions, orbitalMotion


# Columns: mu [m^3/s^2], position [m], velocity [m/s], expected (i, Omega, omega, nu) [deg].
@pytest.mark.parametrize("mu,position,velocity,angles", [
    (3.986004418e14, [7000e3, 0.0, 0.0], [0.0, 0.0, 7.5e3], [90.0, 0.0, 180.0, 180.0]),
    (3.986004418e14, [0.0, 7000e3, 0.0], [-5e3, 0.0, 5e3], [45.0, 90.0, 180.0, 180.0]),
    (3.986004418e14, [0.0, -7000e3, 0.0], [5e3, 0.0, 5e3], [45.0, 270.0, 180.0, 180.0]),
    (1.0, [0.0, 0.0, 1.0], [0.0, -1.25, 0.0], [90.0, 90.0, 90.0, 0.0]),
    (1.0, [0.0, 0.0, -1.0], [0.0, 1.25, 0.0], [90.0, 90.0, 270.0, 0.0]),
    (1.0, [1.0, 0.0, 0.0], [0.75, 0.0, 1.0], [90.0, 0.0, 270.0, 90.0]),
    (1.0, [1.0, 0.0, 0.0], [-0.75, 0.0, 1.0], [90.0, 0.0, 90.0, 270.0]),
], ids=["polar", "node-90", "node-270", "periapsis-90", "periapsis-270", "anomaly-90", "anomaly-270"])
def test_rv2oe_exact_angle_states(mu, position, velocity, angles):
    """Recover known angles when components or dot products are exactly zero."""
    position = np.array(position)
    velocity = np.array(velocity)
    with np.errstate(all="raise"):
        elements = astroFunctions.RV2OE(mu, position, velocity)
    np.testing.assert_allclose(elements[2:], np.deg2rad(angles), rtol=0, atol=1e-14)
    reconstructed = astroFunctions.OE2RV(mu, *elements)
    for actual, expected in zip(reconstructed, (position, velocity)):
        np.testing.assert_allclose(actual, expected, rtol=1e-13, atol=np.linalg.norm(expected)*1e-13)


# Columns: position [m], velocity [m/s], expected (i, Omega, omega, nu) [rad].
@pytest.mark.parametrize("position,velocity,angles", [
    ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 0.0]),
    ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [np.pi/2, 0.0, 0.0, 0.0]),
    ([1.0, 0.0, 0.0], [0.0, 0.6, 0.8], [np.arctan2(0.8, 0.6), 0.0, 0.0, 0.0]),
    ([0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [np.pi, 0.0, 0.0, 0.0]),
    ([0.0, 1.0, 0.0], [-1.25, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]),
    ([1.0, 0.0, 0.0], [0.5, 0.0, 0.0], [0.0, 0.0, 0.0, np.pi]),
    ([0.0, 0.0, 1.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2*np.pi, np.pi]),
], ids=["circular-equatorial", "circular-polar", "circular-inclined", "circular-retrograde",
        "eccentric-equatorial", "radial", "radial-quadrant"])
def test_rv2oe_singular_angle_conventions(position, velocity, angles):
    """Keep zero placeholders for undefined angles without dividing by zero."""
    mu = 1.0  # [m^3/s^2]
    with np.errstate(all="raise"):
        elements = astroFunctions.RV2OE(mu, np.array(position), np.array(velocity))
    assert np.all(np.isfinite(elements))
    np.testing.assert_allclose(elements[2:], angles, rtol=0, atol=1e-14)


# Columns: radial velocity [m/s], expected semimajor axis [m] for unit radius and mu.
@pytest.mark.parametrize("speed,expected_a", [
    (0.0, 0.5),
    (0.5, 4/7),
    (-2.0, -0.5),
    (np.sqrt(2.0), 0.0),
    (1e8, 1/(2.0-1e16)),
], ids=["at-rest", "bound", "unbound-inward", "escape", "fast-outward"])
def test_rv2oe_radial_energy(speed, expected_a):
    """Radial states have unit eccentricity at every energy."""
    mu = 1.0  # [m^3/s^2]
    position = np.array([1.0, 0.0, 0.0])  # [m]
    velocity = np.array([speed, 0.0, 0.0])  # [m/s]
    with np.errstate(all="raise"):
        a, e, *_ = astroFunctions.RV2OE(mu, position, velocity)
    assert a == pytest.approx(expected_a, rel=1e-14, abs=0.0)
    assert e == 1.0


# Columns: specific energy offset [m^2/s^2], velocity [m/s].
@pytest.mark.parametrize("energy_offset,velocity", [
    (-2.0**-40, [1.0, 1.0, 0.0]),
    (0.0, [1.0, 1.0, 0.0]),
    (0.0, [0.0, np.sqrt(2.0), 0.0]),
    (2.0**-40, [1.0, 1.0, 0.0]),
], ids=["below-escape", "exact-escape", "rounded-escape", "above-escape"])
def test_rv2oe_escape_boundary(energy_offset, velocity):
    """Distinguish both sides of escape from a nonradial parabola."""
    position = np.array([1.0, 0.0, 0.0])  # [m]
    mu = 1.0-energy_offset  # [m^3/s^2]
    with np.errstate(all="raise"):
        a, e, *_ = astroFunctions.RV2OE(mu, position, np.array(velocity))
    if energy_offset == 0.0:
        assert (a, e) == (0.0, 1.0)
    else:
        assert a == pytest.approx(-0.5*mu/energy_offset, rel=1e-12)


def test_rv2oe_int32_angular_momentum():
    """A tangential state with angular momentum greater than int32 range is nonradial."""
    mu = 3.986004418e14  # [m^3/s^2]
    radius = 2**23  # [m]
    speed = 8192  # [m/s]
    position = np.array([radius, 0, 0], dtype=np.int32)
    velocity = np.array([0, speed, 0], dtype=np.int32)
    _, e, *_ = astroFunctions.RV2OE(mu, position, velocity)
    expected_e = radius*speed**2/mu - 1.0  # [-] Tangential velocity at periapsis.
    assert e == pytest.approx(expected_e, rel=1e-14)


@pytest.mark.parametrize("r_dtype,v_dtype", [(np.float32, np.float64), (np.float64, np.float32)])
@pytest.mark.parametrize("offset", [-2.0**-23, 2.0**-23])  # [m^2/s^2]
def test_rv2oe_mixed_precision_escape(r_dtype, v_dtype, offset):
    """Recover the signed semimajor axis on either side of escape with mixed input dtypes."""
    position = np.array([1.0, 0.0, 0.0], dtype=r_dtype)  # [m]
    velocity = np.array([0.0, 1.0, 0.0], dtype=v_dtype)  # [m/s]
    mu = 0.5-offset  # [m^3/s^2], giving specific orbital energy equal to offset
    a, *_ = astroFunctions.RV2OE(mu, position, velocity)
    assert a == -0.5*mu/offset


def test_rv2oe_longdouble_velocity():
    """Resolve an escape-speed offset at the precision of the velocity input."""
    mu = 0.5  # [m^3/s^2]
    position = np.array([1.0, 0.0, 0.0])  # [m]
    speed = 1.0 + 32*np.finfo(np.longdouble).eps  # [m/s], just above escape
    velocity = np.array([speed, 0.0, 0.0], dtype=np.longdouble)  # [m/s]
    a, *_ = astroFunctions.RV2OE(mu, position, velocity)
    expected_a = -mu/((speed-1.0)*(speed+1.0))  # [m], factored to avoid cancellation
    np.testing.assert_allclose(a, expected_a, rtol=1e-14, atol=0)


# Columns: semimajor axis [m], eccentricity [-], inclination/node/periapsis/anomaly [rad].
@pytest.mark.parametrize("expected", [
    (3.0, 0.2, 0.7, 1.1, 0.9, 0.4),
    (5.0, 0.7, 2.3, 4.5, 5.1, 4.8),
    (-4.0, 1.5, 0.9, 2.0, 0.5, 2*np.pi-0.6),
], ids=["prograde-ellipse", "retrograde-ellipse", "hyperbola"])
def test_rv2oe_nonsingular_elements(expected):
    """Preserve ordinary elliptic and hyperbolic elements and the six-value tuple API."""
    mu = 1.0  # [m^3/s^2]
    position, velocity = astroFunctions.OE2RV(mu, *expected)
    with np.errstate(all="raise"):
        actual = astroFunctions.RV2OE(mu, position, velocity)
    assert isinstance(actual, tuple) and len(actual) == 6
    np.testing.assert_allclose(actual, expected, rtol=1e-13, atol=1e-14)


@pytest.mark.parametrize("anomaly", [0.0, np.pi])  # [rad]
def test_rv2oe_apsis_roundoff(anomaly):
    """Keep inverse cosines finite when aligned vectors round beyond their domain."""
    mu = 1.0  # [m^3/s^2]
    semimajor_axis = 3.0  # [m]
    eccentricity = 0.3  # [-]
    inclination = 0.7  # [rad]
    node = 1.1  # [rad]
    angle_tolerance = 5e-8  # [rad] Inverse-cosine precision near +/-1.
    for periapsis in np.linspace(0.1, 6.0, 30):  # [rad]
        position, velocity = astroFunctions.OE2RV(
            mu, semimajor_axis, eccentricity, inclination, node, periapsis, anomaly)
        with np.errstate(all="raise"):
            actual = astroFunctions.RV2OE(mu, position, velocity)
        assert np.all(np.isfinite(actual))
        np.testing.assert_allclose(actual[:2], [semimajor_axis, eccentricity], rtol=1e-14, atol=0)
        difference = actual[5] - anomaly
        wrapped_error = np.arctan2(np.sin(difference), np.cos(difference))
        assert abs(wrapped_error) < angle_tolerance


@pytest.mark.parametrize("year", [2000, 2020, 2021])
@pytest.mark.parametrize("day_fraction", [0.0, 0.5, 1.0-1.0/86400.0])  # [day]
def test_gregorian_date(year, day_fraction):
    """Check every date in common and leap years at midnight, noon, and the last second."""
    first_day = date(year, 1, 1)
    for offset in range((date(year+1, 1, 1) - first_day).days):
        expected = first_day + timedelta(days=offset)
        julian_date = 2451544.5 + (expected - date(2000, 1, 1)).days + day_fraction  # [day]
        actual = astroFunctions.GregorianDate(julian_date)
        assert isinstance(actual, np.ndarray) and np.issubdtype(actual.dtype, np.integer)
        np.testing.assert_array_equal(
            actual, [expected.year, expected.month, expected.day], err_msg=str(expected))


# Columns give the Gregorian date, Julian date at midnight [day], seconds since midnight [s], and clock time.
@pytest.mark.parametrize("ymd,midnight,seconds,clock", [
    ((1900, 3, 1), 2415079.5, 0, "0:0:0"),
    ((2000, 1, 1), 2451544.5, 43200, "12:0:0"),
    ((2000, 2, 29), 2451603.5, 64800, "18:0:0"),
    ((2100, 2, 28), 2488127.5, 86399, "23:59:59"),
    ((2100, 3, 1), 2488128.5, 1, "0:0:1"),
    ((2400, 2, 29), 2597700.5, 45296.75, "12:34:56"),
])
def test_calendar_conversions(ymd, midnight, seconds, clock):
    """Check all three conversions against fixed epochs and Gregorian century boundaries."""
    year, month, day = ymd
    day_fraction = seconds / 86400  # [day]
    jd = midnight + day_fraction  # [day]
    assert astroFunctions.JulianDate(np.array([year, month, day + day_fraction])) == jd
    np.testing.assert_array_equal(astroFunctions.GregorianDate(jd), ymd)
    assert astroFunctions.exactGregorianDate(jd) == f"[{year}, {month}, {day}, {clock}]"


def test_julian_date_array_days():
    """Convert fractional days in a leap month and preserve the array shape."""
    days = np.array([[1, 2.5], [28, 29.75]])  # [day]
    expected = [[2451575.5, 2451577.0], [2451602.5, 2451604.25]]  # [day]
    actual = astroFunctions.JulianDate([2000, 2, days])
    assert isinstance(actual, np.ndarray)
    np.testing.assert_array_equal(actual, expected)


def test_julian_date_longdouble_days_before_midnight():
    """Preserve the last Julian date before midnight at the day array's precision."""
    midnight = np.longdouble("2451544.5")  # [day] 2000-01-01.
    before = np.nextafter(midnight, np.longdouble("-inf"))  # [day]
    days = np.array([32 + (before - midnight)])  # [day] Fractional December 31.
    actual = astroFunctions.JulianDate([1999, 12, days])
    np.testing.assert_array_equal(actual, [before])


@pytest.mark.parametrize("jd", [
    np.nextafter(2451544.5, -np.inf),
    np.nextafter(np.longdouble("2451544.5"), np.longdouble("-inf")),
    Decimal("2451544.4999999999"),
])  # [day] Just before 2000-01-01 at each input precision.
def test_exact_gregorian_date_before_midnight(jd):
    """An instant just before a new year still belongs to the previous year's final second."""
    np.testing.assert_array_equal(astroFunctions.GregorianDate(jd), [1999, 12, 31])
    assert astroFunctions.exactGregorianDate(jd) == "[1999, 12, 31, 23:59:59]"


@pytest.mark.parametrize("days", [
    np.array([1 + 2j, 2 + 3j]),
    np.array([1, np.complex128(2 + 3j)], dtype=object),
])
def test_julian_date_complex_days(days):
    """Reject complex day values before converting an array to Julian dates."""
    with pytest.raises(TypeError):
        astroFunctions.JulianDate([2000, 1, days])


@pytest.mark.parametrize("ymd", [
    (2100, 2, 29), (2100, 2, np.array([28.5, 29])), (2000, 1, [1, np.nan]),
    (2000.5, 1, 1), (2000, 1.5, 1),
])
def test_julian_date_invalid_calendar_fields(ymd):
    """Reject invalid days, including NaN in an array, and fractional years or months."""
    with pytest.raises(ValueError):
        astroFunctions.JulianDate(ymd)


@pytest.mark.parametrize("eccentricity", [0.0, 0.005, 0.05, 0.25])  # [-]
@pytest.mark.parametrize("mean_anomaly", [-450.0, 30.0, 45.0, 90.0, 225.0, 450.0])  # [deg]
def test_compute_coe_kepler(eccentricity, mean_anomaly):
    """Compare the fifth-order equation of center with an independent Kepler solution."""
    longitude_periapsis = 30.0  # [deg]
    # Inputs: mean longitude [deg], semimajor axis [AU], eccentricity [-], angles [deg].
    actual = astroFunctions.computeCOE(
        mean_anomaly+longitude_periapsis, 2.0, eccentricity, 10.0, 20.0, longitude_periapsis)
    expected = (2.0*astroConstants.AU, eccentricity, *np.deg2rad([10.0, 20.0, 10.0]))
    np.testing.assert_allclose(actual[:5], expected, rtol=1e-14, atol=0)
    eccentric_anomaly = orbitalMotion.M2E(np.deg2rad(mean_anomaly), eccentricity)
    true_anomaly = orbitalMotion.E2f(eccentric_anomaly, eccentricity)
    error = np.arctan2(np.sin(actual[5]-true_anomaly), np.cos(actual[5]-true_anomaly))
    tolerance = 3.0*eccentricity**6 + 1e-14  # [rad] Sixth-order truncation and roundoff.
    assert abs(error) < tolerance


# Mean elements from Meeus, Astronomical Algorithms, 1st ed., Tables 30.A and 30.B (J2000).
# Columns: planet, Julian centuries since J2000, (L [deg], a [AU], e [-], i/Omega/Pi [deg]).
@pytest.mark.parametrize("planet,centuries,mean_elements", [
    ("SATURN", -0.5, [-560.979373637625, 9.55491066545, 0.0556818671775,
                      2.48758999025, 113.793810542875, 92.77364361225]),
    ("SATURN", 0.0, [50.077471, 9.554909596, 0.05550862, 2.488878, 113.665524, 93.056787]),
    ("SATURN", 0.5, [661.134420657625, 9.55490852655, 0.0553350500225,
                     2.49014149475, 113.537145732125, 93.34019443275]),
    ("URANUS", -0.5, [99.82150463425, 19.218446080845, 0.04630958821875,
                      0.7740403205, 73.968975287, 172.960474973375]),
    ("URANUS", 0.0, [314.055005, 19.218446062, 0.04629590, 0.773196, 74.005947, 173.005159]),
    ("URANUS", 0.5, [528.28850293575, 19.218446043645, 0.04628225128125,
                     0.7723534245, 74.043121413, 173.049795676625]),
    ("NEPTUNE", -0.5, [195.10556504775, 30.1103869523225, 0.00898488580625,
                       1.7698392075, 131.78713901225, 48.109129280375]),
    ("NEPTUNE", 0.0, [304.348665, 30.110386869, 0.00898809, 1.769952, 131.784057, 48.123691]),
    ("NEPTUNE", 0.5, [413.59176524725, 30.1103867860225, 0.00899129379375,
                      1.7700649075, 131.78097389275, 48.138287974625]),
])
def test_meeus_ephemerides(planet, centuries, mean_elements):
    """Check outer-planet elements before, at, and after J2000 against independent references."""
    longitude, semimajor_axis, eccentricity, inclination, node, periapsis = mean_elements
    julian_date = 2451545.0 + 36525.0*centuries  # [day]
    actual = astroFunctions.ephemeridesMeeus(julian_date, planet)
    expected = (semimajor_axis*astroConstants.AU, eccentricity,
                *np.deg2rad([inclination, node, periapsis-node]))
    np.testing.assert_allclose(actual[:5], expected, rtol=1e-13, atol=0)
    eccentric_anomaly = orbitalMotion.M2E(np.deg2rad(longitude-periapsis), eccentricity)
    true_anomaly = orbitalMotion.E2f(eccentric_anomaly, eccentricity)
    error = np.arctan2(np.sin(actual[5]-true_anomaly), np.cos(actual[5]-true_anomaly))
    tolerance = 3.0*eccentricity**6 + 1e-14  # [rad] Sixth-order truncation and roundoff.
    assert abs(error) < tolerance


# Columns: rotation angle [rad], expected rotated vector [-].
@pytest.mark.parametrize("angle,expected", [
    (0.0, [2.0, -3.0, 4.0]),
    (np.pi/2, [3.0, 2.0, 4.0]),
    (-np.pi/2, [-3.0, -2.0, 4.0]),
    (np.pi, [-2.0, 3.0, 4.0]),
    (np.pi/4, [5/np.sqrt(2), -1/np.sqrt(2), 4.0]),
])
def test_rotation_matrix(angle, expected):
    """Check right-handed z rotations and the defining properties of a rotation matrix."""
    vector = np.array([2.0, -3.0, 4.0])  # [-]
    matrix = astroFunctions.rotationMatrix(angle)
    np.testing.assert_allclose(matrix @ vector, expected, rtol=0, atol=1e-14)
    np.testing.assert_allclose(matrix.T @ matrix, np.identity(3), rtol=0, atol=1e-14)
    np.testing.assert_allclose(np.linalg.det(matrix), 1.0, rtol=0, atol=1e-14)
