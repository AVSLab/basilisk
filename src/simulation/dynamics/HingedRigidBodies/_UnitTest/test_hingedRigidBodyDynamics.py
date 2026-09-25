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

"""Conservation and analytical response checks for a spacecraft with two hinged panels."""

import numpy as np
import pytest

from Basilisk.simulation import extForceTorque, gravityEffector, hingedRigidBodyStateEffector, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros


def _make_spacecraft(time_step, spring_constant, damping, planar=False):
    """Build the historical two-panel geometry using the regular spacecraft module."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("testProcess")
    process.addTask(sim.CreateNewTask("dynamics", macros.sec2nano(time_step)))

    body = spacecraft.Spacecraft()
    body.ModelTag = "spacecraftBody"
    body.hub.mHub = 750.0  # [kg]
    body.hub.IHubPntBc_B = np.diag([900.0, 800.0, 600.0]).tolist()  # [kg m^2]
    body.hub.r_BcB_B = [0.0, 0.0, 1.0] if not planar else [0.0, 0.0, 0.0]  # [m]
    sim.AddModelToTask("dynamics", body)

    hinge_positions = ([0.5, 0.0, 1.0], [-0.5, 0.0, 1.0])  # [m]
    hinge_frames = (np.diag([-1.0, -1.0, 1.0]), np.eye(3))
    if planar:
        hinge_positions = ([0.5, 1.0, 0.0], [-0.5, 1.0, 0.0])  # [m]
        hinge_frames = (
            [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]],
            [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        )

    panels = []
    hinge_logs = []
    for index in range(2):
        panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
        panel.ModelTag = f"panel{index + 1}"
        panel.mass = 100.0  # [kg]
        panel.IPntS_S = np.diag([100.0, 50.0, 50.0]).tolist()  # [kg m^2]
        panel.d = 1.5  # [m]
        panel.k = spring_constant
        panel.c = damping[index]
        panel.r_HB_B = hinge_positions[index]
        panel.dcm_HB = np.asarray(hinge_frames[index]).tolist()
        body.addStateEffector(panel)
        sim.AddModelToTask("dynamics", panel)
        panels.append(panel)

        hinge_log = panel.hingedRigidBodyOutMsg.recorder()
        sim.AddModelToTask("dynamics", hinge_log)
        hinge_logs.append(hinge_log)

    return sim, body, panels, hinge_logs


def _check_hinge_recording(hinge_log, stop_time, time_step):
    """Reject missing samples and non-finite hinge states before comparing dynamics."""
    sample_count = int(round(stop_time / time_step)) + 1
    expected_times = np.arange(sample_count, dtype=np.int64) * macros.sec2nano(time_step)
    np.testing.assert_array_equal(hinge_log.times(), expected_times)
    for values in (hinge_log.theta, hinge_log.thetaDot):
        assert values.shape == (sample_count,)
        assert np.isfinite(values).all(), "Hinge trajectory contains non-finite values"
    return hinge_log.times() * macros.NANO2SEC, hinge_log.theta, hinge_log.thetaDot


def _assert_conserved(history, name):
    """Check every sample using the initial magnitude to normalize vector or scalar errors."""
    assert len(history) > 1
    assert np.isfinite(history).all(), f"{name} contains non-finite values"
    initial_magnitude = np.linalg.norm(history[0])
    assert initial_magnitude > 0.0, f"{name} must have a nonzero initial reference"
    np.testing.assert_allclose(
        (history - history[0]) / initial_magnitude, 0.0,
        rtol=0.0, atol=1e-10, err_msg=f"{name} is not conserved",
    )


@pytest.mark.parametrize("case", ["gravity", "no_gravity", "damping"])
def test_hinged_body_conservation(case):
    """Check energy and momentum histories with gravity, in free flight, and with damping.

    Both undamped cases conserve orbital and rotational energy and angular momentum.
    The damped case conserves orbital energy and both angular momenta, while the loss
    of rotational energy agrees with the integral of hinge damping power.
    """
    time_step = 0.001  # [s]
    stop_time = 2.5  # [s]
    spring_constant = 100.0  # [N m/rad]
    damping = (6.0, 7.0) if case == "damping" else (0.0, 0.0)  # [N m s/rad]
    sim, body, panels, hinge_logs = _make_spacecraft(time_step, spring_constant, damping)
    panels[0].thetaInit = 5.0 * macros.D2R  # [rad]
    body.hub.r_CN_NInit = [0.1, -0.4, 0.3]  # [m]
    body.hub.v_CN_NInit = [-0.2, 0.5, 0.1]  # [m/s]
    body.hub.omega_BN_BInit = [0.1, -0.1, 0.1]  # [rad/s]

    if case == "gravity":
        body.hub.r_CN_NInit = [-4020338.690396649, 7490566.741852513, 5248299.211589362]  # [m]
        body.hub.v_CN_NInit = [-5199.77710904224, -3436.681645356935, 1041.576797498721]  # [m/s]
        earth = gravityEffector.GravBodyData()
        earth.planetName = "earth"
        earth.mu = 0.3986004415e15  # [m^3/s^2]
        earth.isCentralBody = True
        body.gravField.gravBodies = spacecraft.GravBodyVector([earth])

    quantities = ["totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totRotEnergy"]
    energy_momentum_log = body.logger(quantities)
    sim.AddModelToTask("dynamics", energy_momentum_log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(stop_time))
    sim.ExecuteSimulation()

    for hinge_log in hinge_logs:
        _check_hinge_recording(hinge_log, stop_time, time_step)
        np.testing.assert_array_equal(energy_momentum_log.times(), hinge_log.times())
        assert np.ptp(hinge_log.theta) > 1e-4  # [rad] Both panels must participate in the motion.

    for name in quantities[:3]:
        _assert_conserved(getattr(energy_momentum_log, name), name)

    rotational_energy = energy_momentum_log.totRotEnergy
    if case != "damping":
        _assert_conserved(rotational_energy, "totRotEnergy")
    else:
        assert np.isfinite(rotational_energy).all()
        energy_tolerance = 1e-10  # [J]
        assert np.all(np.diff(rotational_energy) <= energy_tolerance)
        energy_loss = rotational_energy[0] - rotational_energy[-1]
        assert energy_loss > energy_tolerance
        damping_power = sum(panel.c * log.thetaDot**2 for panel, log in zip(panels, hinge_logs))
        dissipated_energy = np.sum(0.5 * (damping_power[1:] + damping_power[:-1]) * time_step)
        np.testing.assert_allclose(energy_loss, dissipated_energy, rtol=1e-6, atol=energy_tolerance)


def _make_forced_spacecraft(spring_constant, damping):
    """Apply a constant force to a symmetric, initially stationary planar spacecraft."""
    time_step = 0.01  # [s]
    force = 1.0  # [N]
    sim, body, panels, hinge_logs = _make_spacecraft(
        time_step, spring_constant, (damping, damping), planar=True,
    )
    external_force = extForceTorque.ExtForceTorque()
    external_force.ModelTag = "externalForce"
    external_force.extForce_B = [0.0, force, 0.0]  # [N]
    body.addDynamicEffector(external_force)
    sim.AddModelToTask("dynamics", external_force)
    return sim, body, panels, hinge_logs, external_force, time_step, force


def test_hinged_body_steady_state_deflection():
    r"""Compare both damped hinge angles with the nonlinear static torque balance.

    Under constant force :math:`F`, the system acceleration is :math:`F/M` and each
    hinge satisfies :math:`k\theta + m d (F/M)\cos\theta = 0`. Check the complete
    final settling window, including hinge rates, rather than a single endpoint.
    """
    spring_constant = 100.0  # [N m/rad]
    damping = 75.0  # [N m s/rad]
    stop_time = 60.0  # [s]
    settling_window = 5.0  # [s]
    sim, body, panels, hinge_logs, external_force, time_step, force = _make_forced_spacecraft(
        spring_constant, damping,
    )
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(stop_time))
    sim.ExecuteSimulation()

    total_mass = body.hub.mHub + sum(panel.mass for panel in panels)
    inertial_torque = panels[0].mass * panels[0].d * force / total_mass
    # The small-angle solution and zero bracket the unique negative root.
    lower = -inertial_torque / spring_constant
    upper = 0.0  # [rad]
    for _ in range(50):
        midpoint = 0.5 * (lower + upper)
        residual = spring_constant * midpoint + inertial_torque * np.cos(midpoint)
        if residual > 0.0:
            upper = midpoint
        else:
            lower = midpoint
    expected_angle = 0.5 * (lower + upper)
    angle_tolerance = 1e-6  # [rad]
    rate_tolerance = 1e-6  # [rad/s]

    for hinge_log in hinge_logs:
        times, angles, rates = _check_hinge_recording(hinge_log, stop_time, time_step)
        settled = times >= stop_time - settling_window
        np.testing.assert_allclose(angles[settled], expected_angle, rtol=0.0, atol=angle_tolerance)
        np.testing.assert_allclose(rates[settled], 0.0, rtol=0.0, atol=rate_tolerance)


def test_hinged_body_frequency_and_amplitude():
    r"""Measure Basilisk hinge periods and amplitudes before and after thrust removal.

    For symmetric panel motion the small-angle equations reduce to
    :math:`J_{\mathrm{eff}}\ddot\theta + k\theta = -m d F/M`, where
    :math:`J_{\mathrm{eff}} = I_{yy} + m d^2 - 2 (m d)^2/M`.
    This gives the frequency, forced peak deflection, and free amplitude after a
    known thrust duration independently of either simulated trajectory. All
    measured peaks and periods come from Basilisk's hinge output messages. Check
    every extremum, require enough cycles for each phase duration, and compare
    the full angle and rate histories so a stalled final partial cycle also fails.
    """
    spring_constant = 300.0  # [N m/rad]
    damping = 0.0  # [N m s/rad]
    force_off_time = 29.0  # [s]
    stop_time = 58.0  # [s]
    sim, body, panels, hinge_logs, external_force, time_step, force = _make_forced_spacecraft(
        spring_constant, damping,
    )
    total_mass = body.hub.mHub + sum(panel.mass for panel in panels)
    panel = panels[0]
    effective_inertia = panel.IPntS_S[1][1] + panel.mass * panel.d**2 - 2 * (panel.mass * panel.d)**2 / total_mass
    natural_rate = np.sqrt(spring_constant / effective_inertia)
    expected_period = 2 * np.pi / natural_rate
    equilibrium_angle = -panel.mass * panel.d * force / (total_mass * spring_constant)
    release_angle = equilibrium_angle * (1 - np.cos(natural_rate * force_off_time))
    release_rate = equilibrium_angle * natural_rate * np.sin(natural_rate * force_off_time)
    expected_free_amplitude = np.hypot(release_angle, release_rate / natural_rate)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(force_off_time))
    sim.ExecuteSimulation()
    external_force.extForce_B = [0.0, 0.0, 0.0]  # [N]
    sim.ConfigureStopTime(macros.sec2nano(stop_time))
    sim.ExecuteSimulation()

    for hinge_log in hinge_logs:
        times, angles, rates = _check_hinge_recording(hinge_log, stop_time, time_step)
        forced = times <= force_off_time
        free = times > force_off_time
        reference_angles = equilibrium_angle * (1 - np.cos(natural_rate * times))
        reference_rates = equilibrium_angle * natural_rate * np.sin(natural_rate * times)
        free_phase = natural_rate * (times[free] - force_off_time)
        reference_angles[free] = (
            release_angle * np.cos(free_phase) + release_rate / natural_rate * np.sin(free_phase)
        )
        reference_rates[free] = (
            -release_angle * natural_rate * np.sin(free_phase) + release_rate * np.cos(free_phase)
        )
        free_equilibrium = 0.0  # [rad]
        for phase, equilibrium, amplitude in (
            (forced, equilibrium_angle, abs(equilibrium_angle)),
            (free, free_equilibrium, expected_free_amplitude),
        ):
            phase_times = times[phase]
            phase_angles = angles[phase]
            angle_tolerance = 5e-3 * amplitude
            rate_tolerance = angle_tolerance * natural_rate
            # Include every sample, including the final partial cycle of each phase.
            np.testing.assert_allclose(
                phase_angles, reference_angles[phase], rtol=0.0, atol=angle_tolerance,
            )
            np.testing.assert_allclose(
                rates[phase], reference_rates[phase], rtol=0.0, atol=rate_tolerance,
            )
            minimum_peaks = int(np.floor((phase_times[-1] - phase_times[0]) / expected_period))
            for direction in (1, -1):
                signed_angles = direction * phase_angles
                peaks = np.flatnonzero(
                    (signed_angles[1:-1] > signed_angles[:-2]) &
                    (signed_angles[1:-1] >= signed_angles[2:])
                ) + 1
                assert len(peaks) >= minimum_peaks, "Oscillations do not cover the full phase duration"
                measured_periods = np.diff(phase_times[peaks])
                np.testing.assert_allclose(measured_periods, expected_period, rtol=5e-3, atol=0.0)
                np.testing.assert_allclose(
                    phase_angles[peaks], equilibrium + direction * amplitude,
                    rtol=0.0, atol=angle_tolerance,
                )


if __name__ == "__main__":
    for conservation_case in ("gravity", "no_gravity", "damping"):
        test_hinged_body_conservation(conservation_case)
    test_hinged_body_steady_state_deflection()
    test_hinged_body_frequency_and_amplitude()
