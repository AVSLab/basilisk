
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
# Keplerian OrbitUnit Test
#
# Purpose:  Tests an object-oriented Keplerian Orbit Object
# Author:   Scott Carnahan
# Creation Date:  Sept 10 2019
#

import inspect
import os
from copy import copy

import numpy as np
import pytest
from Basilisk.architecture import keplerianOrbit
from Basilisk.utilities import orbitalMotion

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def test_unitKeplerianOrbit(show_plots=False):
    """Module Unit Test"""
    [testResults, testMessage] = unitKeplerianOrbit(show_plots)
    assert testResults < 1, testMessage


def unitKeplerianOrbit(show_plots=False):
    """
    Unit Test Keplerian Orbit object

    This test covers:
    1) elem to rv conversion
    2) orbit energy calculation
    3) orbital period calculation
    4) mean motion calculation
    5) changing individual orbital elements
    6) copy constructor
    7) constructor with arguments
    8) constructor without arguments

    Parameters
    ----------
    show_plots : bool
        unused. required for Basilisk test suite

    Returns
    -------
    testFailCount : int
        number of tests that failed
    testMessages : str
        a string of all of the failure messages
    """
    testFailCount = 0
    testMessages = []

    # constructor without arguments
    orb = keplerianOrbit.KeplerianOrbit()
    assert orb.a() == 100000000.
    if not orb.a() == 100000000.:
        testFailCount += 1
        testMessages.append('default constructor failure')

    # constructor with arguments
    oe = orb.oe()
    orb2 = keplerianOrbit.KeplerianOrbit(oe, orbitalMotion.MU_EARTH)
    assert orb2.r_BP_P() == orb.r_BP_P()
    if not orb2.r_BP_P() == orb.r_BP_P():
        testFailCount += 1
        testMessages.append('Argumented constructor failure')

    # copy constructor
    orb3 = keplerianOrbit.KeplerianOrbit(orb2)
    assert orb2.v_BP_P() == orb3.v_BP_P()
    if not orb2.v_BP_P() == orb3.v_BP_P():
        testFailCount += 1
        testMessages.append('Copy Constructor Failure')
    try:
        orb4 = copy(orb3)
    except:
        assert False
        testFailCount += 1
        testMessages.append('python copy not working')

    # changing orbital elements
    orb3.set_f(0.0)
    init_r = orb3.r()
    orb3.set_f(1.)
    assert init_r != orb3.r()
    if init_r == orb3.r():
        testFailCount += 1
        testMessages.append('Failure to change element')
    orb3.set_f(0.0)
    assert init_r == orb3.r()
    if init_r != orb3.r():
        testFailCount += 1
        testMessages.append('Failure to change element')

    # mean motion calc
    expected_n = np.sqrt(orbitalMotion.MU_EARTH / orb3.a()**3)
    assert orb3.n() == expected_n
    if not orb3.n() == expected_n:
        testFailCount += 1
        testMessages.append('Bad mean motion calc')

    # orbital period calc
    assert orb3.P() == 2 * np.pi / expected_n
    if not orb3.P() == 2 * np.pi / expected_n:
        testFailCount += 1
        testMessages.append('Bad period calc')

    # orbital energy calc
    expected_E = -orbitalMotion.MU_EARTH / 2.0 / orb3.a()
    assert orb3.Energy() == expected_E
    if not orb3.Energy() == expected_E:
        testFailCount += 1
        testMessages.append('Bad energy calc')

    # rv calc
    expected_r, expected_v = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, orb3.oe())
    dist = np.linalg.norm(np.array(orb3.r_BP_P()).flatten() - expected_r)
    assert dist == 0.0
    if not dist == 0.0:
        testFailCount += 1
        testMessages.append('RV conversion failure')

    return [testFailCount, ''.join(testMessages)]


MU = 3.986004415e14  # [m^3/s^2]
TRUE_ANOMALIES = (0.0, 0.4, np.pi, 4.0, 2 * np.pi, -0.4, -4.0, 2 * np.pi + 0.4)  # [rad]
ECCENTRICITIES = (0.0, 0.1, 0.9)  # [-]


def make_elements(eccentricity, true_anomaly):
    """Return elements for an inclined ellipse with the requested shape and phase."""
    elements = keplerianOrbit.ClassicElements()
    elements.a = 7.0e6  # [m]
    elements.e = eccentricity
    elements.i = 0.5  # [rad]
    elements.Omega = 0.2  # [rad]
    elements.omega = 0.3  # [rad]
    elements.f = true_anomaly
    return elements


def assert_orbit_outputs(orbit, elements, mu):
    """Check elliptic outputs against element formulas and Cartesian state invariants."""
    relative_tolerance = 1.0e-12
    angle_tolerance = 1.0e-13  # [rad]
    rate_tolerance = 1.0e-15  # [rad/s]
    velocity_tolerance = 1.0e-10  # [m/s]
    position_tolerance = 1.0e-8  # [m]
    expected_position, expected_velocity = orbitalMotion.elem2rv(mu, elements)
    position = np.asarray(orbit.r_BP_P()).ravel()
    velocity = np.asarray(orbit.v_BP_P()).ravel()
    np.testing.assert_allclose(
        position, expected_position, rtol=relative_tolerance, atol=position_tolerance
    )
    np.testing.assert_allclose(
        velocity, expected_velocity, rtol=relative_tolerance, atol=velocity_tolerance
    )

    parameter = elements.a * (1 - elements.e**2)
    radius = parameter / (1 + elements.e * np.cos(elements.f))
    mean_motion = np.sqrt(mu / elements.a**3)
    angular_momentum = np.cross(position, velocity)
    expected_f_dot = np.sqrt(mu * parameter) / radius**2
    expected_r_dot = np.sqrt(mu / parameter) * elements.e * np.sin(elements.f)
    expected_e = orbitalMotion.f2E(elements.f, elements.e)
    expected_m = orbitalMotion.E2M(expected_e, elements.e)
    expected_fpa = np.arcsin(
        np.dot(position, velocity) / (np.linalg.norm(position) * np.linalg.norm(velocity))
    )

    assert orbit.E() == pytest.approx(expected_e, rel=relative_tolerance, abs=angle_tolerance)
    assert orbit.M() == pytest.approx(expected_m, rel=relative_tolerance, abs=angle_tolerance)
    assert orbit.fpa() == pytest.approx(expected_fpa, rel=relative_tolerance, abs=angle_tolerance)
    assert orbit.fDot() == pytest.approx(expected_f_dot, rel=relative_tolerance, abs=rate_tolerance)
    assert orbit.rDot() == pytest.approx(expected_r_dot, rel=relative_tolerance, abs=velocity_tolerance)
    assert orbit.fDot() == pytest.approx(
        np.linalg.norm(angular_momentum) / np.dot(position, position),
        rel=relative_tolerance, abs=rate_tolerance
    )
    assert orbit.rDot() == pytest.approx(
        np.dot(position, velocity) / np.linalg.norm(position),
        rel=relative_tolerance, abs=velocity_tolerance
    )
    np.testing.assert_allclose(np.asarray(orbit.h_BP_P()).ravel(), angular_momentum, rtol=relative_tolerance)
    assert orbit.h() == pytest.approx(np.sqrt(mu * parameter), rel=relative_tolerance)
    assert orbit.n() == pytest.approx(mean_motion, rel=relative_tolerance)
    assert orbit.P() == pytest.approx(2 * np.pi / mean_motion, rel=relative_tolerance)
    assert orbit.Energy() == pytest.approx(-mu / (2 * elements.a), rel=relative_tolerance)
    assert orbit.p() == pytest.approx(parameter, rel=relative_tolerance)
    assert orbit.r() == pytest.approx(radius, rel=relative_tolerance)
    assert orbit.v() == pytest.approx(
        np.sqrt(mu * (2 / radius - 1 / elements.a)), rel=relative_tolerance
    )
    assert orbit.r_a() == pytest.approx(elements.a * (1 + elements.e), rel=relative_tolerance)
    assert orbit.r_p() == pytest.approx(elements.a * (1 - elements.e), rel=relative_tolerance)

    returned_elements = orbit.oe()
    for name, getter in (
        ('a', 'a'), ('e', 'e'), ('i', 'i'), ('Omega', 'RAAN'), ('omega', 'omega'), ('f', 'f')
    ):
        assert getattr(returned_elements, name) == getattr(elements, name)
        assert getattr(orbit, getter)() == getattr(elements, name)
    assert returned_elements.rmag == pytest.approx(radius, rel=relative_tolerance)
    assert returned_elements.rmag == orbit.r()
    assert returned_elements.alpha == pytest.approx(1 / elements.a, rel=relative_tolerance)
    assert returned_elements.rApoap == orbit.r_a()
    assert returned_elements.rPeriap == orbit.r_p()


@pytest.mark.parametrize('eccentricity', ECCENTRICITIES)
@pytest.mark.parametrize('true_anomaly', TRUE_ANOMALIES)
@pytest.mark.parametrize('constructor', ('elements', 'copy', 'python_copy'))
def test_constructor_outputs(eccentricity, true_anomaly, constructor):
    """Check anomalies, signed flight-path angle, rates, and radius before any setter runs."""
    elements = make_elements(eccentricity, true_anomaly)
    orbit = keplerianOrbit.KeplerianOrbit(elements, MU)
    if constructor == 'copy':
        orbit = keplerianOrbit.KeplerianOrbit(orbit)
    elif constructor == 'python_copy':
        orbit = copy(orbit)
    assert_orbit_outputs(orbit, elements, MU)


def test_default_constructor_outputs():
    """Check that the default constructor initializes the complete derived state."""
    elements = keplerianOrbit.ClassicElements()
    elements.a = 1.0e8  # [m]
    elements.e = 1.0e-5  # [-]
    elements.i = elements.Omega = elements.omega = elements.f = 0.0  # [rad]
    mu = orbitalMotion.MU_EARTH * 1.0e9  # [m^3/s^2]
    assert_orbit_outputs(keplerianOrbit.KeplerianOrbit(), elements, mu)


@pytest.mark.parametrize('eccentricity', ECCENTRICITIES)
def test_true_anomaly_updates(eccentricity):
    """Check repeated phase changes across apsides, negative angles, and revolutions."""
    elements = make_elements(eccentricity, TRUE_ANOMALIES[0])
    orbit = keplerianOrbit.KeplerianOrbit(elements, MU)
    for true_anomaly in TRUE_ANOMALIES:
        elements.f = true_anomaly
        orbit.set_f(true_anomaly)
        assert_orbit_outputs(orbit, elements, MU)


@pytest.mark.parametrize('true_anomaly', (0.4, 4.0))  # [rad]
@pytest.mark.parametrize('element, setter, value', (
    ('a', 'set_a', 14.0e6),  # [m]
    ('a', 'set_a', 3.5e6),  # [m]
    ('e', 'set_e', 0.7),  # [-]
    ('i', 'set_i', 1.2),  # [rad]
    ('omega', 'set_omega', 0.8),  # [rad]
    ('Omega', 'set_RAAN', 1.1),  # [rad]
))
def test_element_updates(true_anomaly, element, setter, value):
    """Check immediate consistency after each orbit-shape or orientation setter."""
    elements = make_elements(0.1, true_anomaly)  # [-], [rad]
    orbit = keplerianOrbit.KeplerianOrbit(elements, MU)
    setattr(elements, element, value)
    getattr(orbit, setter)(value)
    assert_orbit_outputs(orbit, elements, MU)


@pytest.mark.parametrize('true_anomaly', (0.4, 4.0))  # [rad]
@pytest.mark.parametrize('mu_scale', (0.25, 4.0))  # [-]
def test_gravitational_parameter_updates(true_anomaly, mu_scale):
    """Check immediate gravity updates, subsequent element changes, and copying."""
    elements = make_elements(0.1, true_anomaly)  # [-], [rad]
    orbit = keplerianOrbit.KeplerianOrbit(elements, MU)
    mu = MU * mu_scale
    orbit.set_mu(mu)
    assert_orbit_outputs(orbit, elements, mu)
    elements.f = -true_anomaly
    orbit.set_f(elements.f)
    assert_orbit_outputs(orbit, elements, mu)
    elements.a = 14.0e6  # [m]
    orbit.set_a(elements.a)
    assert_orbit_outputs(orbit, elements, mu)
    assert_orbit_outputs(copy(orbit), elements, mu)


if __name__ == "__main__":
    unitKeplerianOrbit()
