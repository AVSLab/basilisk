
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


"""Validate pulse timing independently of dynamics evaluation order and integrator stages."""

import numpy as np
import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import ExtPulsedTorque, spacecraft, svIntegrators
from Basilisk.utilities import SimulationBaseClass, macros


def configured_pulse(off_count=3):
    """Create a pulse with exactly representable transition times."""
    effector = ExtPulsedTorque.ExtPulsedTorque()
    effector.pulsedTorqueExternalPntB_B = [-1.0, 2.0, -3.0]  # [N*m]
    effector.countOnPulse = 2
    effector.countOff = off_count
    effector.pulseInterval = 0.125  # [s]
    return effector


def pulse_simulation(effector, priority=None, step=0.03125):
    """Attach a pulse to a spacecraft; step is the task interval in seconds."""
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    sim.CreateNewProcess("process").addTask(sim.CreateNewTask("task", macros.sec2nano(step)))
    parent = spacecraft.Spacecraft()
    parent.hub.mHub = 1.0  # [kg]
    parent.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
    parent.addDynamicEffector(effector)
    sim.AddModelToTask("task", parent)
    if priority is not None:
        sim.AddModelToTask("task", effector, priority)
    return sim, parent


@pytest.mark.parametrize("off_count", [0, 3])
def test_pulse_sequence(off_count):
    """Counts specify fixed-duration positive, negative, and off intervals over repeated cycles."""
    effector = configured_pulse(off_count)
    expected = ([1, 1, -1, -1] + [0]*off_count)*2
    amplitude = np.asarray(effector.pulsedTorqueExternalPntB_B).ravel()
    for index, sign in enumerate(expected):
        for fraction in [0.0, 0.25, 0.75]:
            time = (index + fraction)*effector.pulseInterval  # [s]
            for step in [1.0, 0.01, 0.0]:  # [s]; pulse timing must ignore the integrator step
                effector.computeForceTorque(time, step)
                np.testing.assert_array_equal(np.asarray(effector.torqueExternalPntB_B).ravel(), sign*amplitude)


@pytest.mark.parametrize("time,sign", [
    (0.0, 1), (0.25 - 1e-12, 1), (np.nextafter(0.25, 0.0), -1), (0.25, -1),
    (0.5 - 1e-12, -1), (np.nextafter(0.5, 0.0), 0), (0.5, 0),
    (0.875 - 1e-12, 0), (np.nextafter(0.875, 0.0), 1), (0.875, 1),
])  # time: [s], sign: [-]
def test_pulse_transition_boundaries(time, sign):
    """Each transition selects the new pulse segment without an evaluation counter."""
    effector = configured_pulse()
    effector.computeForceTorque(time, 0.0)
    amplitude = np.asarray(effector.pulsedTorqueExternalPntB_B).ravel()
    np.testing.assert_array_equal(np.asarray(effector.torqueExternalPntB_B).ravel(), sign*amplitude)


@pytest.mark.parametrize("interval", [0.1, 0.03, 0.07])  # [s]
def test_decimal_pulse_intervals_select_the_new_segment_at_boundaries(interval):
    """Decimal interval roundoff must not delay a transition by one dynamics evaluation."""
    effector = configured_pulse(1)
    effector.countOnPulse = 1
    effector.pulseInterval = interval
    amplitude = np.asarray(effector.pulsedTorqueExternalPntB_B).ravel()
    for index in list(range(30)) + [1000000, 1000001, 1000002]:
        effector.computeForceTorque(index*interval, 0.0)  # [s]
        sign = [1, -1, 0][index % 3]
        np.testing.assert_array_equal(np.asarray(effector.torqueExternalPntB_B).ravel(), sign*amplitude)


@pytest.mark.parametrize("off_count", [1, 1000000, 2000000000])
@pytest.mark.parametrize("interval", [0.001, 1.0, 1000000.0])  # [s]
def test_long_off_period_does_not_advance_initial_pulse_transitions(off_count, interval):
    """A future off period must not enlarge the roundoff window around either initial pulse edge."""
    effector = configured_pulse(off_count)
    effector.countOnPulse = 1
    effector.pulseInterval = interval  # [s]
    amplitude = np.asarray(effector.pulsedTorqueExternalPntB_B).ravel()
    for boundary, preceding_sign, following_sign in [(1, 1, -1), (2, -1, 0)]:
        transition = boundary*interval  # [s]
        samples = [
            (transition - 1e-6*interval, preceding_sign),
            (transition - 1e-12*interval, preceding_sign),
            (np.nextafter(transition, 0.0), following_sign),
            (transition, following_sign),
            (transition + 1e-12*interval, following_sign),
        ]  # time: [s], sign: [-]
        for time, sign in samples:
            effector.computeForceTorque(time, 0.0)  # [s]
            np.testing.assert_array_equal(np.asarray(effector.torqueExternalPntB_B).ravel(), sign*amplitude)


def test_pulse_repeated_and_reversed_evaluations_preserve_phase():
    """Repeated evaluations, adaptive retries, and resets do not advance or rewind the pulse phase."""
    effector = configured_pulse()
    for time, sign in [(0.75, 0), (0.25, -1), (0.125, 1), (0.0, 1), (0.875, 1), (0.25, -1)]:  # [s], [-]
        for _ in range(3):
            effector.Reset(macros.sec2nano(time))
            effector.UpdateState(macros.sec2nano(time))
            effector.computeForceTorque(time, 0.01)  # [s]
            amplitude = np.asarray(effector.pulsedTorqueExternalPntB_B).ravel()
            np.testing.assert_array_equal(np.asarray(effector.torqueExternalPntB_B).ravel(), sign*amplitude)


def test_pulse_defaults_and_disabling_clear_loads():
    """Default configuration is inert, and disabling an active pulse clears its contributions."""
    effector = ExtPulsedTorque.ExtPulsedTorque()
    effector.Reset(0)
    effector.computeForceTorque(0.0, 1.0)  # [s]
    np.testing.assert_array_equal(effector.torqueExternalPntB_B, np.zeros((3, 1)))
    effector.pulsedTorqueExternalPntB_B = [1.0, 0.0, 0.0]  # [N*m]
    effector.countOnPulse = 1
    effector.computeForceTorque(0.0, 1.0)  # [s]
    effector.countOnPulse = 0
    effector.computeForceTorque(0.0, 1.0)  # [s]
    for field in ["torqueExternalPntB_B", "forceExternal_N", "forceExternal_B"]:
        np.testing.assert_array_equal(getattr(effector, field), np.zeros((3, 1)))


@pytest.mark.parametrize("field,value", [
    ("countOnPulse", -1), ("countOff", -1),
    ("pulseInterval", 0.0), ("pulseInterval", -1.0),
    ("pulseInterval", np.nan), ("pulseInterval", np.inf), ("pulseInterval", -np.inf),
    ("pulseInterval", np.finfo(float).max),
    ("pulsedTorqueExternalPntB_B", [np.nan, 0.0, 0.0]),
    ("pulsedTorqueExternalPntB_B", [np.inf, 0.0, 0.0]),
    ("pulsedTorqueExternalPntB_B", [-np.inf, 0.0, 0.0]),
])  # pulseInterval: [s], torque: [N*m]; counts are dimensionless
@pytest.mark.parametrize("path", ["reset", "attachment", "evaluation"])
def test_invalid_pulse_configuration(field, value, path):
    """Every dynamics initialization path rejects invalid pulse parameters."""
    effector = configured_pulse()
    setattr(effector, field, value)
    with pytest.raises(BasiliskError):
        if path == "reset":
            effector.Reset(0)
        elif path == "attachment":
            sim, parent = pulse_simulation(effector)
            sim.InitializeSimulation()
        else:
            effector.computeForceTorque(0.0, 1.0)


@pytest.mark.parametrize("time", [np.nan, np.inf, -np.inf, -1.0])  # [s]
def test_invalid_pulse_evaluation_time(time):
    """Evaluation time must be finite and non-negative."""
    with pytest.raises(BasiliskError, match="integTime"):
        configured_pulse().computeForceTorque(time, 1.0)


def test_pulse_large_counts_do_not_overflow_integer_arithmetic():
    """The cycle count can exceed the range of an individual count's integer type."""
    effector = configured_pulse()
    effector.countOnPulse = int(np.iinfo(np.int32).max)
    effector.countOff = int(np.iinfo(np.int32).max)
    effector.pulseInterval = 1.0  # [s]
    effector.Reset(0)
    effector.computeForceTorque(float(effector.countOnPulse), 1.0)  # [s]
    np.testing.assert_array_equal(effector.torqueExternalPntB_B, -np.asarray(effector.pulsedTorqueExternalPntB_B))


@pytest.mark.parametrize("integrator_name", ["svIntegratorRK2", "svIntegratorRK4", "svIntegratorRKF45"])
@pytest.mark.parametrize("priority", [None, 100, -100], ids=["attached", "before", "after"])
@pytest.mark.parametrize("step", [0.015625, 0.03125])  # [s]
def test_integrator_stages_and_task_order_do_not_shorten_pulses(integrator_name, priority, step):
    """A spacecraft receives the full torque throughout a pulse regardless of stage count or scheduling."""
    effector = configured_pulse()
    effector.pulsedTorqueExternalPntB_B = [1.0, 0.0, 0.0]  # [N*m]
    sim, parent = pulse_simulation(effector, priority, step)
    integrator = getattr(svIntegrators, integrator_name)(parent)
    parent.setIntegrator(integrator)
    sim.InitializeSimulation()
    stop_time = 0.1875  # [s]; before the first transition at 0.25 s
    sim.ConfigureStopTime(macros.sec2nano(stop_time))
    sim.ExecuteSimulation()
    np.testing.assert_allclose(parent.scStateOutMsg.read().omega_BN_B, [stop_time, 0.0, 0.0],
                               rtol=1e-13, atol=1e-14)  # [rad/s] for the configured torque and inertia


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
