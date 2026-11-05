#
# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

"""Validate opt-in equinoctial averaging without changing the legacy modes.

The reference recovers classical elements and solves Kepler's equation by
bisection, independently of the module's direct equinoctial reconstruction.
The compact formation construction follows carlo98's example in issue 1656.
"""

import math

import numpy as np
import pytest

from Basilisk.architecture import bskLogging, messaging
from Basilisk.fswAlgorithms import formationBarycenter
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion


MU = 3.986004418e14  # [m^3/s^2]
CLASSICAL = 0
EQUINOCTIAL_PROGRADE = 1
EQUINOCTIAL_RETROGRADE = 2


def classical_state(axis, eccentricity, inclination, node, periapsis, mean_anomaly):
    """Construct an independent Cartesian state from an elliptic orbit.

    Distances are in meters, angles in radians and eccentricity dimensionless.
    The returned position and velocity have units of meters and meters/second.
    """
    lower = mean_anomaly-eccentricity
    upper = mean_anomaly+eccentricity
    for _ in range(100):
        eccentric_anomaly = (lower+upper)/2
        if eccentric_anomaly == lower or eccentric_anomaly == upper:
            break
        if eccentric_anomaly-eccentricity*math.sin(eccentric_anomaly) > mean_anomaly:
            upper = eccentric_anomaly
        else:
            lower = eccentric_anomaly
    cosine = math.cos(eccentric_anomaly)
    sine = math.sin(eccentric_anomaly)
    beta = math.sqrt(1-eccentricity**2)
    radius = axis*(1-eccentricity*cosine)  # [m]
    position = axis*np.array([cosine-eccentricity, beta*sine, 0.0])  # [m]
    velocity = math.sqrt(MU*axis)/radius*np.array([-sine, beta*cosine, 0.0])  # [m/s]
    co, so = math.cos(node), math.sin(node)
    cw, sw = math.cos(periapsis), math.sin(periapsis)
    ci, si = math.cos(inclination), math.sin(inclination)
    rotation = np.array([
        [co*cw-so*sw*ci, -co*sw-so*cw*ci, so*si],
        [so*cw+co*sw*ci, -so*sw+co*cw*ci, -co*si],
        [sw*si, cw*si, ci],
    ])
    return rotation @ position, rotation @ velocity


def make_simulation(states, masses, element_type=None, use_orbital=True, mu=MU):
    """Initialize one module and retain writable inputs and both output recorders."""
    assert len(states) == len(masses)
    simulation = SimulationBaseClass.SimBaseClass()
    simulation.CreateNewProcess("process").addTask(
        simulation.CreateNewTask("task", macros.sec2nano(1.0))  # [s]
    )
    module = formationBarycenter.FormationBarycenter()
    module.useOrbitalElements = use_orbital
    module.mu = mu
    if element_type is not None:
        module.orbitalElementType = element_type
    messages = []
    for (position, velocity), mass in zip(states, masses):
        navigation = messaging.NavTransMsgPayload()
        navigation.r_BN_N = np.asarray(position).tolist()
        navigation.v_BN_N = np.asarray(velocity).tolist()
        vehicle = messaging.VehicleConfigMsgPayload()
        vehicle.massSC = mass
        messages.extend([
            messaging.NavTransMsg().write(navigation),
            messaging.VehicleConfigMsg().write(vehicle),
        ])
        module.addSpacecraftToModel(messages[-2], messages[-1])
    recorder = module.transOutMsg.recorder()
    recorder_c = module.transOutMsgC.recorder()
    for model in (module, recorder, recorder_c):
        simulation.AddModelToTask("task", model)
    simulation.InitializeSimulation()
    return simulation, module, messages, recorder, recorder_c


def recorded_state(recorder, recorder_c):
    """Read the latest finite state and require identical C/C++ output payloads."""
    result = np.concatenate([recorder.r_BN_N[-1], recorder.v_BN_N[-1]])
    result_c = np.concatenate([recorder_c.r_BN_N[-1], recorder_c.v_BN_N[-1]])
    np.testing.assert_array_equal(result, result_c)
    assert np.isfinite(result).all()
    return result


def run_module(states, masses, element_type=None, use_orbital=True, mu=MU, after_reset=None):
    """Execute the compiled module once, optionally changing settings after Reset."""
    simulation, module, messages, recorder, recorder_c = make_simulation(
        states, masses, element_type, use_orbital, mu
    )
    if after_reset is not None:
        for attribute, value in after_reset.items():
            setattr(module, attribute, value)
    simulation.ConfigureStopTime(0)
    simulation.ExecuteSimulation()
    return recorded_state(recorder, recorder_c)


def assert_state(actual, expected, rtol=2e-11):
    """Use position and velocity tolerances with their respective physical units."""
    np.testing.assert_allclose(actual[:3], expected[:3], rtol=rtol, atol=2e-6)  # [m]
    np.testing.assert_allclose(actual[3:], expected[3:], rtol=rtol, atol=2e-8)  # [m/s]


def test_default_and_named_choices():
    """Existing scripts retain Cartesian/default-classical selection semantics."""
    module = formationBarycenter.FormationBarycenter()
    assert module.useOrbitalElements is False
    assert module.orbitalElementType == CLASSICAL
    assert module.CLASSICAL == CLASSICAL
    assert module.EQUINOCTIAL_PROGRADE == EQUINOCTIAL_PROGRADE
    assert module.EQUINOCTIAL_RETROGRADE == EQUINOCTIAL_RETROGRADE


@pytest.mark.parametrize("element_type", [CLASSICAL, EQUINOCTIAL_PROGRADE, EQUINOCTIAL_RETROGRADE, -1])
def test_cartesian_ignores_element_selector(element_type):
    """The extra orbital setting cannot change the original Cartesian branch."""
    states = [
        classical_state(7e6, 0.1, 0.9, 0.3, 0.4, phase)  # [m, -, rad, rad, rad, rad]
        for phase in (0.7, 0.9)  # [rad]
    ]
    masses = np.array([100.0, 300.0])  # [kg]
    expected = np.average([np.concatenate(state) for state in states], weights=masses, axis=0)
    actual = run_module(states, masses, element_type, use_orbital=False, mu=0.0)
    np.testing.assert_array_equal(actual, expected)


@pytest.mark.parametrize("element_type", [EQUINOCTIAL_PROGRADE, EQUINOCTIAL_RETROGRADE])
@pytest.mark.parametrize("eccentricity", [0.0, 1e-8, 0.2, 0.7, 0.95])  # [-]
@pytest.mark.parametrize("tilt_index", range(4))
@pytest.mark.parametrize("mean_anomaly", [0.0, 0.7, 3.4, 6.1])  # [rad]
def test_single_spacecraft_identity(element_type, eccentricity, tilt_index, mean_anomaly):
    """Each representation recovers a single input, including its nonsingular pole."""
    inclinations = (
        [0.0, 1e-12, 0.9, 2.8] if element_type == EQUINOCTIAL_PROGRADE
        else [0.2, 0.9, math.pi-1e-12, math.pi]
    )  # [rad]
    state = classical_state(
        2e7, eccentricity, inclinations[tilt_index], 0.3, 1.2, mean_anomaly
    )  # [m, -, rad, rad, rad, rad]
    assert_state(run_module([state], [100.0], element_type), np.concatenate(state))  # [kg]


@pytest.mark.parametrize("phases", [
    (0, 120, 240), (0, 120.1, 240), (0, 121, 240),
    (0, 125, 240), (0, 140, 240), (0, 90, 180, 270),
])  # [deg]
@pytest.mark.parametrize("element_type", [EQUINOCTIAL_PROGRADE, EQUINOCTIAL_RETROGRADE])
def test_reported_compact_formation(phases, element_type):
    """Require an independent state reference and a 1 cm locality budget.

    Each member has nonzero eccentricity and non-equatorial inclination, so
    its classical elements can be recovered without a singular-angle choice.
    The reference then averages known elements and reconstructs through the
    separate bisection/rotation path, not the module's plane equations.
    The centimetre budget applies only to these reported 100 m formations;
    it is not an equality requirement for arbitrary orbital and Cartesian means.
    """
    radius = 7e6  # [m]
    spread = 100.0  # [m]
    chief = orbitalMotion.ClassicElements()
    chief.a, chief.e, chief.i = radius, 0.0, 0.9  # [m, -, rad]
    chief.Omega, chief.omega, chief.f = 0.3, 0.0, 1.0  # [rad]
    rc, vc = orbitalMotion.elem2rv(MU, chief)
    rate = math.sqrt(MU/radius**3)  # [rad/s]
    states = []
    for phase in np.radians(phases):
        offset = spread*np.array([0.5*math.cos(phase), -math.sin(phase), math.cos(phase)])  # [m]
        offset_rate = spread*rate*np.array([-0.5*math.sin(phase), -math.cos(phase), -math.sin(phase)])  # [m/s]
        states.append(orbitalMotion.hill2rv(rc, vc, offset, offset_rate))
    cartesian = np.mean([state[0] for state in states], axis=0)  # [m]
    masses = [100.0]*len(states)  # [kg]
    orbits = []
    for position, velocity in states:
        elements = orbitalMotion.rv2elem(MU, position, velocity)
        assert elements.e > 1e-7 and 0.1 < elements.i < math.pi-0.1  # [-, rad]
        mean_anomaly = orbitalMotion.E2M(orbitalMotion.f2E(elements.f, elements.e), elements.e)  # [rad]
        orbits.append((elements.a, elements.e, elements.i, elements.Omega, elements.omega, mean_anomaly))
    factor = 1 if element_type == EQUINOCTIAL_PROGRADE else -1
    expected = weighted_reference(orbits, masses, factor)
    actual = run_module(states, masses, element_type)
    assert_state(actual, expected, rtol=0.0)
    locality_budget = 0.01  # [m]; geometry-specific regression budget for these compact geometries
    assert np.linalg.norm(expected[:3]-cartesian) < locality_budget
    assert np.linalg.norm(actual[:3]-cartesian) < locality_budget


@pytest.mark.parametrize("element_type", [EQUINOCTIAL_PROGRADE, EQUINOCTIAL_RETROGRADE])
@pytest.mark.parametrize("elapsed", [0.0, 60.0, 200.0])  # [s]
def test_equal_axis_preserves_orbital_clock(element_type, elapsed):
    """On one ellipse, equal-mass symmetric phases retain their time midpoint."""
    axis = 2e7  # [m]
    inclination = 0.9 if element_type == EQUINOCTIAL_PROGRADE else math.pi  # [rad]
    mean_midpoint = 1.1+math.sqrt(MU/axis**3)*elapsed  # [rad]
    orbits = [
        (axis, 0.6, inclination, 0.3, 1.2, mean_midpoint+delta)  # [m, -, rad, rad, rad, rad]
        for delta in (-0.2, 0.2)  # [rad]
    ]
    expected = classical_state(axis, 0.6, inclination, 0.3, 1.2, mean_midpoint)  # [m, -, rad, rad, rad, rad]
    actual = run_module([classical_state(*orbit) for orbit in orbits], [100.0, 100.0], element_type)  # [kg]
    assert_state(actual, np.concatenate(expected))


def weighted_reference(orbits, masses, factor):
    """Average known elements, then invert through a separate classical rotation."""
    weights = np.asarray(masses)/sum(masses)
    averaged = np.zeros(5)
    sine_sum = cosine_sum = 0.0
    for weight, (axis, eccentricity, inclination, node, periapsis, mean_anomaly) in zip(weights, orbits):
        longitude_periapsis = periapsis+factor*node  # [rad]
        tilt = math.tan(inclination/2) if factor == 1 else 1/math.tan(inclination/2)
        averaged += weight*np.array([
            axis, eccentricity*math.sin(longitude_periapsis), eccentricity*math.cos(longitude_periapsis),
            tilt*math.sin(node), tilt*math.cos(node),
        ])  # [m, -, -, -, -]
        longitude = mean_anomaly+longitude_periapsis  # [rad]
        sine_sum += weight*math.sin(longitude)
        cosine_sum += weight*math.cos(longitude)
    axis, shape_sine, shape_cosine, tilt_sine, tilt_cosine = averaged
    eccentricity = math.hypot(shape_sine, shape_cosine)
    tilt = math.hypot(tilt_sine, tilt_cosine)
    inclination = 2*math.atan(tilt) if factor == 1 else 2*math.atan2(1, tilt)  # [rad]
    node = math.atan2(tilt_sine, tilt_cosine)  # [rad]
    longitude_periapsis = math.atan2(shape_sine, shape_cosine)  # [rad]
    periapsis = longitude_periapsis-factor*node  # [rad]
    mean_anomaly = math.atan2(sine_sum, cosine_sum)-longitude_periapsis  # [rad]
    return np.concatenate(classical_state(axis, eccentricity, inclination, node, periapsis, mean_anomaly))


@pytest.mark.parametrize("element_type,factor", [(EQUINOCTIAL_PROGRADE, 1), (EQUINOCTIAL_RETROGRADE, -1)])
def test_weighting_wrap_and_reordering(element_type, factor):
    """Check unequal shapes/tilts/masses, wraparound and mass/permutation invariance."""
    orbits = [
        (2e7, 0.2, 0.6, 0.1, 0.2, 6.0),
        (2.1e7, 0.4, 0.8, 0.2, 0.1, 0.0),
        (1.9e7, 0.1, 1.0, 0.3, 0.0, 0.2),
    ]  # [m, -, rad, rad, rad, rad]
    if factor == -1:
        orbits = [(axis, ecc, math.pi-inc, node, peri, phase) for axis, ecc, inc, node, peri, phase in orbits]
    masses = [100.0, 200.0, 400.0]  # [kg]
    states = [classical_state(*orbit) for orbit in orbits]
    expected = weighted_reference(orbits, masses, factor)
    actual = run_module(states, masses, element_type)
    assert_state(actual, expected)
    assert_state(run_module(states[::-1], masses[::-1], element_type), actual)
    assert_state(run_module(states, np.asarray(masses)*7, element_type), actual)


@pytest.mark.parametrize("element_type", [EQUINOCTIAL_PROGRADE, EQUINOCTIAL_RETROGRADE])
def test_multiple_updates_refresh_navigation_and_mass(element_type):
    """Check six scheduler updates with changing navigation and mass messages.

    Three unequal ellipses share one semimajor axis and advance analytically
    at the common two-body mean motion. Both output forms and sample times
    must match a fresh independent reference at every step. The masses change
    halfway through to expose stale inputs or uncleared averaging sums.
    This tests scheduled message integration, not a numerical dynamics integrator.
    """
    axis = 2e7  # [m]
    rate = math.sqrt(MU/axis**3)  # [rad/s]
    factor = 1 if element_type == EQUINOCTIAL_PROGRADE else -1
    orbits = [
        (axis, 0.2, 0.6, 0.1, 0.2, 0.8),
        (axis, 0.4, 0.8, 0.2, 0.1, 1.0),
        (axis, 0.6, 1.0, 0.3, 0.0, 1.2),
    ]  # [m, -, rad, rad, rad, rad]
    if factor == -1:
        orbits = [(a, ecc, math.pi-inc, node, peri, phase) for a, ecc, inc, node, peri, phase in orbits]
    masses = [100.0, 200.0, 400.0]  # [kg]
    simulation, module, messages, recorder, recorder_c = make_simulation(
        [classical_state(*orbit) for orbit in orbits], masses, element_type
    )
    assert module.orbitalElementType == element_type
    for step in range(6):
        elapsed = float(step)  # [s]; the task period is one second
        current_orbits = [(*orbit[:-1], orbit[-1]+rate*elapsed) for orbit in orbits]
        if step == 3:
            masses = [400.0, 100.0, 200.0]  # [kg]
        for index, (orbit, mass) in enumerate(zip(current_orbits, masses)):
            position, velocity = classical_state(*orbit)
            navigation = messaging.NavTransMsgPayload()
            navigation.r_BN_N = position.tolist()
            navigation.v_BN_N = velocity.tolist()
            vehicle = messaging.VehicleConfigMsgPayload()
            vehicle.massSC = mass
            messages[2*index].write(navigation)
            messages[2*index+1].write(vehicle)
        simulation.TotalSim.SingleStepProcesses()
        assert_state(recorded_state(recorder, recorder_c), weighted_reference(current_orbits, masses, factor))
    expected_times = np.arange(6, dtype=np.uint64)*macros.sec2nano(1.0)  # [ns]
    np.testing.assert_array_equal(recorder.times(), expected_times)
    np.testing.assert_array_equal(recorder_c.times(), expected_times)


@pytest.mark.parametrize("element_type", [EQUINOCTIAL_PROGRADE, EQUINOCTIAL_RETROGRADE])
@pytest.mark.parametrize("binding_eps,accepted", [(8, False), (64, True)])
def test_binding_energy_cutoff(element_type, binding_eps, accepted):
    """Bracket the resolvable elliptic-energy cutoff without an accuracy claim.

    With unit radius and gravity, binding is ``2-speed**2`` and the cutoff
    is ``16*epsilon``. The separated samples allow for square-root rounding.
    Passing the guard at this extreme ellipse only promises finite output.
    """
    epsilon = np.finfo(float).eps
    speed = math.sqrt(2.0-binding_eps*epsilon)  # [m/s]
    factor = 1 if element_type == EQUINOCTIAL_PROGRADE else -1
    state = (np.array([1.0, 0.0, 0.0]), np.array([0.0, factor*speed, 0.0]))  # [m, m/s]
    binding = 2.0-speed*speed
    cutoff = 8*epsilon*max(2.0, speed*speed)
    assert (binding > cutoff) == accepted
    if accepted:
        run_module([state], [1.0], element_type, mu=1.0)  # [kg, m^3/s^2]
    else:
        with pytest.raises(bskLogging.BasiliskError, match="resolvable nonradial"):
            run_module([state], [1.0], element_type, mu=1.0)  # [kg, m^3/s^2]


@pytest.mark.parametrize("element_type", [EQUINOCTIAL_PROGRADE, EQUINOCTIAL_RETROGRADE])
@pytest.mark.parametrize("transverse_eps,first_guard", [(4, True), (8, True), (16, False)])
def test_near_radial_cutoff(element_type, transverse_eps, first_guard):
    """Above the radial guard, an unrepresentable ellipse must still be rejected.

    These almost-radial unit states have eccentricity rounding to one even
    just above the angular-momentum cutoff. The first guard rejects values
    at/below its cutoff; the later elliptic-elements guard rejects the rest.
    Crossing this guard alone must not turn an unresolved ellipse into a valid one.
    """
    epsilon = np.finfo(float).eps
    factor = 1 if element_type == EQUINOCTIAL_PROGRADE else -1
    transverse_speed = transverse_eps*epsilon  # [m/s]
    state = (np.array([1.0, 0.0, 0.0]), np.array([1.0, factor*transverse_speed, 0.0]))  # [m, m/s]
    assert (transverse_speed <= 8*epsilon*math.sqrt(1+transverse_speed**2)) == first_guard
    message = "resolvable nonradial" if first_guard else "representable elliptic elements"
    with pytest.raises(bskLogging.BasiliskError, match=message):
        run_module([state], [1.0], element_type, mu=1.0)  # [kg, m^3/s^2]


@pytest.mark.parametrize("element_type", [EQUINOCTIAL_PROGRADE, EQUINOCTIAL_RETROGRADE])
@pytest.mark.parametrize("tilt_eps,accepted", [(4, False), (8, False), (16, True)])
def test_excluded_pole_cutoff(element_type, tilt_eps, accepted):
    """Reject at/below the pole cutoff and recover a state just outside it.

    Cartesian construction avoids subtracting a tiny tilt from pi. Its unit
    angular momentum has transverse magnitude ``tilt_eps*epsilon`` to floating
    precision and points toward the selected representation's excluded pole.
    """
    epsilon = np.finfo(float).eps
    factor = 1 if element_type == EQUINOCTIAL_PROGRADE else -1
    transverse_speed = tilt_eps*epsilon  # [m/s]
    state = (np.array([1.0, 0.0, 0.0]), np.array([0.0, -factor, transverse_speed]))  # [m, m/s]
    assert (transverse_speed > 8*epsilon) == accepted
    if accepted:
        actual = run_module([state], [1.0], element_type, mu=1.0)  # [kg, m^3/s^2]
        np.testing.assert_allclose(actual, np.concatenate(state), rtol=0.0, atol=64*epsilon)  # [m, m/s]
    else:
        with pytest.raises(bskLogging.BasiliskError, match="pole"):
            run_module([state], [1.0], element_type, mu=1.0)  # [kg, m^3/s^2]


@pytest.mark.parametrize("element_type", [EQUINOCTIAL_PROGRADE, EQUINOCTIAL_RETROGRADE])
@pytest.mark.parametrize("offset_eps,accepted", [(64, False), (256, True)])
def test_longitude_cancellation_cutoff(element_type, offset_eps, accepted):
    """Bracket the 64-epsilon circular-resultant cutoff with equal near-opposites.

    The resultant is approximately half the phase offset. Samples at 32 and
    128 epsilons avoid depending on libm's exact rounding at the boundary.
    The accepted symmetric configuration has its reference along positive y.
    """
    epsilon = np.finfo(float).eps
    offset = offset_eps*epsilon  # [rad]
    factor = 1 if element_type == EQUINOCTIAL_PROGRADE else -1
    states = [
        (np.array([1.0, 0.0, 0.0]), np.array([0.0, factor, 0.0])),
        (np.array([-1.0, offset, 0.0]), np.array([-factor*offset, -factor, 0.0])),
    ]  # [m, m/s]
    assert (offset/2 > 64*epsilon) == accepted
    if accepted:
        actual = run_module(states, [1.0, 1.0], element_type, mu=1.0)  # [kg, m^3/s^2]
        expected = np.array([0.0, 1.0, 0.0, -factor, 0.0, 0.0])  # [m, m/s]
        np.testing.assert_allclose(actual, expected, rtol=0.0, atol=64*epsilon)  # [m, m/s]
    else:
        with pytest.raises(bskLogging.BasiliskError, match="longitude"):
            run_module(states, [1.0, 1.0], element_type, mu=1.0)  # [kg, m^3/s^2]


@pytest.mark.parametrize("element_type", [-1, 3])
def test_invalid_selector(element_type):
    """Unknown orbital choices fail explicitly instead of selecting another method."""
    state = classical_state(7e6, 0.1, 0.9, 0.3, 0.4, 0.7)  # [m, -, rad, rad, rad, rad]
    with pytest.raises(bskLogging.BasiliskError, match="orbitalElementType"):
        run_module([state], [100.0], element_type)  # [kg]


@pytest.mark.parametrize("mu", [0.0, -1.0, math.nan, math.inf])  # [m^3/s^2]
def test_invalid_gravity(mu):
    """New orbital modes require a finite positive gravitational parameter."""
    state = classical_state(7e6, 0.1, 0.9, 0.3, 0.4, 0.7)  # [m, -, rad, rad, rad, rad]
    with pytest.raises(bskLogging.BasiliskError, match="gravitational parameter"):
        run_module([state], [100.0], EQUINOCTIAL_PROGRADE, mu=mu)  # [kg]


@pytest.mark.parametrize("masses", [[0.0], [-1.0], [math.inf], [math.nan]])  # [kg]
def test_invalid_mass(masses):
    """Undefined weights cannot produce a fabricated formation reference."""
    state = classical_state(7e6, 0.1, 0.9, 0.3, 0.4, 0.7)  # [m, -, rad, rad, rad, rad]
    with pytest.raises(bskLogging.BasiliskError, match="mass"):
        run_module([state], masses, EQUINOCTIAL_PROGRADE)


@pytest.mark.parametrize("velocity", [[0.0, 1.0, 1.0], [0.0, 2.0, 0.0]])  # [m/s]
def test_nonelliptic_input_is_rejected(velocity):
    """Exact unit-scale parabolic and hyperbolic inputs are outside the new modes."""
    with pytest.raises(bskLogging.BasiliskError, match="elliptic"):
        state = (np.array([1.0, 0.0, 0.0]), np.array(velocity))  # [m, m/s]
        run_module([state], [1.0], EQUINOCTIAL_PROGRADE, mu=1.0)  # [kg, m^3/s^2]


@pytest.mark.parametrize("element_type,inclination", [
    (EQUINOCTIAL_PROGRADE, math.pi), (EQUINOCTIAL_RETROGRADE, 0.0),
])  # [rad]
def test_opposite_chart_pole_is_rejected(element_type, inclination):
    """A representation's excluded pole is an explicit error, not silent fallback."""
    state = classical_state(7e6, 0.1, inclination, 0.3, 0.4, 0.7)  # [m, -, rad, rad, rad, rad]
    with pytest.raises(bskLogging.BasiliskError, match="pole"):
        run_module([state], [100.0], element_type)  # [kg]


def test_undefined_longitude_is_rejected():
    """Equal opposite phases have no unique circular average longitude."""
    speed = math.sqrt(MU/7e6)  # [m/s]
    state = (np.array([7e6, 0.0, 0.0]), np.array([0.0, speed, 0.0]))  # [m, m/s]
    opposite = (-state[0], -state[1])
    with pytest.raises(bskLogging.BasiliskError, match="longitude"):
        run_module([state, opposite], [100.0, 100.0], EQUINOCTIAL_PROGRADE)  # [kg]


@pytest.mark.parametrize("bad_state", [
    ([0.0, 0.0, 0.0], [0.0, 7500.0, 0.0]),
    ([7e6, 0.0, 0.0], [0.0, 0.0, 0.0]),
    ([math.nan, 0.0, 0.0], [0.0, 7500.0, 0.0]),
    ([7e6, 0.0, 0.0], [0.0, math.inf, 0.0]),
])  # [m, m/s]
def test_invalid_navigation(bad_state):
    """New modes stop on invalid navigation instead of writing nonfinite output."""
    with pytest.raises(bskLogging.BasiliskError):
        run_module([bad_state], [100.0], EQUINOCTIAL_PROGRADE)  # [kg]


def test_zero_mass_member_does_not_contribute():
    """A zero-weight spacecraft cannot move an otherwise defined reference."""
    first = classical_state(7e6, 0.1, 0.9, 0.3, 0.4, 0.7)  # [m, -, rad, rad, rad, rad]
    second = classical_state(8e6, 0.3, 1.1, 0.5, 0.6, 1.7)  # [m, -, rad, rad, rad, rad]
    actual = run_module([first, second], [100.0, 0.0], EQUINOCTIAL_PROGRADE)  # [kg]
    assert_state(actual, np.concatenate(first))


def test_runtime_selection():
    """The new selector can be configured after initialization, like the old switch."""
    state = classical_state(7e6, 0.1, 0.9, 0.3, 0.4, 0.7)  # [m, -, rad, rad, rad, rad]
    actual = run_module([state], [100.0], CLASSICAL, after_reset={"orbitalElementType": EQUINOCTIAL_PROGRADE})  # [kg]
    assert_state(actual, np.concatenate(state))


@pytest.mark.parametrize("changes", [{"orbitalElementType": 3}, {"mu": math.nan}])
def test_runtime_validation(changes):
    """Invalid changes after Reset are rejected again before new-mode computation."""
    state = classical_state(7e6, 0.1, 0.9, 0.3, 0.4, 0.7)  # [m, -, rad, rad, rad, rad]
    with pytest.raises(bskLogging.BasiliskError):
        run_module([state], [100.0], EQUINOCTIAL_PROGRADE, after_reset=changes)  # [kg]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
