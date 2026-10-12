#
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
#

"""Integration tests for the MuJoCo thruster-arm tracking scenario."""

import importlib
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk import hasBuildFeature
from Basilisk.architecture import messaging
from Basilisk.utilities import macros

SCENARIO_FOLDER = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "examples", "mujoco")
)
sys.path.append(SCENARIO_FOLDER)

pytestmark = pytest.mark.skipif(
    not hasBuildFeature("mujoco"),
    reason="Requires Basilisk built with --mujoco True",
)


def _load_tracking_scenario():
    """Import the tracking example, skipping if SciPy is unavailable."""
    pytest.importorskip("scipy")
    try:
        return importlib.import_module("scenarioThrArmTracking")
    except ModuleNotFoundError as exc:
        if exc.name == "scipy":
            pytest.skip("Tracking scenario requires optional dependency 'scipy'.")
        raise


def _record_thr_arm_control(scenario, record_joints=False):
    """Record outer-loop pulse requests and inner-loop actuator/joint states."""
    fsw = scenario.get_FswModel()
    on_time = fsw.thrFiringRound.thrOnTimeOutMsg.recorder()
    scenario.AddModelToTask(fsw.outerLoopTaskName, on_time)

    force = []
    for force_msg in fsw.thrForceGateways:
        recorder = force_msg.recorder()
        scenario.AddModelToTask(fsw.recorderTaskName, recorder)
        force.append(recorder)

    angles = []
    rates = []
    if record_joints:
        dynamics = scenario.get_DynModel()
        for body_name, joint_name in fsw.jointRefs:
            joint = dynamics.scene.getBody(body_name).getScalarJoint(joint_name)
            angle_rec = joint.stateOutMsg.recorder()
            rate_rec = joint.stateDotOutMsg.recorder()
            scenario.AddModelToTask(fsw.recorderTaskName, angle_rec)
            scenario.AddModelToTask(fsw.recorderTaskName, rate_rec)
            angles.append(angle_rec)
            rates.append(rate_rec)

    profile = None
    if fsw.jointProfiler is not None:
        profile = fsw.jointProfiler.desJointStatesOutMsg.recorder()
        scenario.AddModelToTask(fsw.jointMotionTaskName, profile)

    return on_time, force, angles, rates, profile


@pytest.mark.parametrize(
    "use_profiler, use_penalty",
    [(False, False), (True, False), (False, True), (True, True)],
)
@pytest.mark.scenarioTest
def test_thr_arm_tracking_options(use_profiler, use_penalty):
    """Check short-run tracking outcomes with each optional feature combination."""
    module = _load_tracking_scenario()
    scenario = module.ScenarioThrArmTracking(use_profiler, use_penalty)

    fsw = scenario.get_FswModel()
    assert (fsw.jointProfiler is not None) == use_profiler
    assert fsw.jointThrAllocation.useThetaPenalty == use_penalty
    if use_penalty:
        assert len(fsw.jointThrAllocation.jointStatesInMsgs) == fsw.numJoints
        assert all(reader.isLinked() for reader in fsw.jointThrAllocation.jointStatesInMsgs)

    on_time_rec, force_recs, _, _, profile_rec = _record_thr_arm_control(scenario)
    period = scenario.config.outerLoopPeriod  # [s]
    dt = scenario.config.fswTimeStep  # [s]
    run_time = 2.0 * period + dt  # [s], include the next outer-loop update
    module.runScenario(scenario, run_time, showProgressBar=False)

    outer_times = on_time_rec.times() * macros.NANO2SEC  # [s]
    on_times = np.asarray(on_time_rec.OnTimeRequest)[:, :fsw.numThrusters]  # [s]
    force_times = force_recs[0].times() * macros.NANO2SEC  # [s]
    forces = np.column_stack([np.asarray(rec.input) for rec in force_recs])  # [N]
    max_forces = np.asarray(scenario.config.thrForces)  # [N]

    assert len(outer_times) >= 3
    np.testing.assert_allclose(outer_times[:3], [0.0, period, 2.0 * period], atol=dt)  # [s]
    assert np.all(np.isfinite(on_times))
    assert np.all((on_times >= 0.0) & (on_times <= period + dt))  # [s]
    assert np.allclose(
        on_times / scenario.config.pulseResolution,
        np.rint(on_times / scenario.config.pulseResolution),
        rtol=0.0,
        atol=1e-8,
    )
    assert np.any(on_times[:2] > 0.0)  # [s], avoid vacuous pulse checks
    assert np.all(np.isfinite(forces))
    assert np.all(np.isclose(forces, 0.0) | np.isclose(forces, max_forces))  # [N]
    assert np.any(forces > 0.0)  # [N], the actuator fires during the run
    assert np.all(np.isfinite(np.asarray(scenario.posRec.r_CN_N)))  # [m]
    assert np.all(np.isfinite(np.asarray(scenario.attErrorRec.sigma_BR)))  # [-]

    for window in range(2):
        samples = (force_times >= window * period) & (force_times < (window + 1) * period)
        delivered_impulse = np.sum(forces[samples], axis=0) * dt  # [N*s]
        requested_impulse = max_forces * on_times[window]  # [N*s]
        assert np.all(delivered_impulse <= requested_impulse + 2.0 * max_forces * dt)  # [N*s]

    if use_profiler:
        assert profile_rec is not None
        assert len(profile_rec.times()) > 0
        assert np.all(np.isfinite(np.asarray(profile_rec.states)))  # [rad]
    if use_penalty:
        assert np.isfinite(fsw.jointThrAllocation.costVal)

    figures = scenario.pull_outputs()
    assert len(figures) == 2
    for figure in figures.values():
        plt.close(figure)


@pytest.mark.scenarioTest
def test_thr_arm_profiled_pulse_truncation():
    """Verify profiler-delayed firing is cancelled at the next control update."""
    module = _load_tracking_scenario()
    scenario = module.ScenarioThrArmTracking(useJointProfiler=True)
    fsw = scenario.get_FswModel()

    # Connect fixed test commands before initialization so this test does not
    # depend on which solution a particular SciPy version selects.
    angle_msg = messaging.JointArrayStateMsg()
    force_msg = messaging.THRArrayCmdForceMsg()
    fsw.desAngleReader.subscribeTo(angle_msg)
    fsw.jointProfiler.desJointStatesInMsg.subscribeTo(angle_msg)
    fsw.thrFiringRound.thrForceInMsg.subscribeTo(force_msg)

    angles_cmd = [0.01] + [0.0] * (fsw.numJoints - 1)  # [rad]
    angle_payload = messaging.JointArrayStateMsgPayload()
    for angle in angles_cmd:
        angle_payload.states.push_back(angle)
        angle_payload.stateDots.push_back(0.0)  # [rad/s]
        angle_payload.stateDDots.push_back(0.0)  # [rad/s^2]
    force_payload = messaging.THRArrayCmdForceMsgPayload()
    force_payload.thrForce = scenario.config.thrForces
    force_msg.write(force_payload, 0)
    on_time_rec, force_recs, angle_recs, rate_recs, profile_rec = _record_thr_arm_control(
        scenario, record_joints=True
    )

    scenario.InitializeSimulation()
    scenario.configure_initial_conditions()
    scenario.SetProgressBar(False)
    scenario.ConfigureStopTime(0)
    scenario.ExecuteSimulation()

    period = scenario.config.outerLoopPeriod  # [s]
    dt = scenario.config.fswTimeStep  # [s]
    # Write the joint command after the outer-loop on-time calculation so the
    # event latches the matching pulse rather than its reset-time zero output.
    angle_msg.write(angle_payload, macros.sec2nano(dt))
    first_stop = period - dt  # [s], just before the next outer-loop update
    scenario.ConfigureStopTime(macros.sec2nano(first_stop))
    scenario.ExecuteSimulation()

    force_payload.thrForce = [0.0] * fsw.numThrusters  # [N]
    force_msg.write(force_payload, macros.sec2nano(period))
    scenario.ConfigureStopTime(macros.sec2nano(period))
    scenario.ExecuteSimulation()
    # Trigger the next event after the outer loop has calculated zero on-time.
    angle_msg.write(angle_payload, macros.sec2nano(period + dt))
    final_stop = 2.0 * period + dt  # [s], through the subsequent control window
    scenario.ConfigureStopTime(macros.sec2nano(final_stop))
    scenario.ExecuteSimulation()

    outer_times = on_time_rec.times() * macros.NANO2SEC  # [s]
    on_times = np.asarray(on_time_rec.OnTimeRequest)[:, :fsw.numThrusters]  # [s]
    force_times = force_recs[0].times() * macros.NANO2SEC  # [s]
    forces = np.column_stack([np.asarray(rec.input) for rec in force_recs])  # [N]
    assert len(outer_times) >= 2
    np.testing.assert_allclose(on_times[0], [period] * fsw.numThrusters, atol=dt)  # [s]
    np.testing.assert_allclose(on_times[1], np.zeros(fsw.numThrusters), atol=dt)  # [s]

    first_window = (force_times >= 0.0) & (force_times < period)
    second_window = force_times >= period + dt
    delivered_impulse = np.sum(forces[first_window], axis=0) * dt  # [N*s]
    max_forces = np.asarray(scenario.config.thrForces)  # [N]
    requested_impulse = max_forces * on_times[0]  # [N*s]
    assert np.all(delivered_impulse > 0.0)  # [N*s], firing phase was reached
    assert np.all(delivered_impulse < requested_impulse - max_forces * dt)  # [N*s]
    assert np.all(np.isclose(forces[second_window], 0.0))  # [N], no carryover

    firing_indices = np.flatnonzero(np.any(forces[first_window] > 0.0, axis=1))
    first_firing_index = firing_indices[0]
    assert force_times[first_firing_index] > 0.5 * scenario.config.jointProfileDuration  # [s]
    actual_angles = np.array([rec.state[first_firing_index] for rec in angle_recs])  # [rad]
    actual_rates = np.array([rec.state[first_firing_index] for rec in rate_recs])  # [rad/s]
    assert np.max(np.abs(actual_angles - angles_cmd)) < (  # [rad]
        2.0 * scenario.config.jointPositionTolerance
    )
    assert np.max(np.abs(actual_rates)) < 2.0 * scenario.config.jointRateTolerance  # [rad/s]
    assert profile_rec is not None
    profile_angles = np.asarray(profile_rec.states)  # [rad]
    assert np.all(np.isfinite(profile_angles))
    assert profile_angles[-1, 0] > profile_angles[0, 0]  # [rad]
