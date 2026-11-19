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

"""Regressions for orbital-element and calendar-date conversions in astroFunctions."""

from datetime import date, timedelta

import numpy as np
import pytest

from Basilisk.utilities import astroFunctions


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
