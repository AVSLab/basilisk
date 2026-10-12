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

"""Integration test for the MuJoCo thruster-arm regulation scenario."""

import importlib
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk import hasBuildFeature
from Basilisk.utilities import macros

SCENARIO_FOLDER = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "examples", "mujoco")
)
sys.path.append(SCENARIO_FOLDER)

pytestmark = pytest.mark.skipif(
    not hasBuildFeature("mujoco"),
    reason="Requires Basilisk built with --mujoco True",
)


@pytest.mark.scenarioTest
def test_thr_arm_regulation():
    """Check quantized pulses and the short-run regulation response."""
    pytest.importorskip("scipy")
    module = importlib.import_module("scenarioThrArmControl")
    scenario = module.scenarioThrArmControl()
    fsw = scenario.get_FswModel()
    assert fsw.jointProfiler is None
    assert not fsw.jointThrAllocation.useThetaPenalty

    on_time_rec = fsw.thrFiringRound.thrOnTimeOutMsg.recorder()
    scenario.AddModelToTask(fsw.outerLoopTaskName, on_time_rec)
    force_recs = []
    for force_msg in fsw.thrForceGateways:
        recorder = force_msg.recorder()
        scenario.AddModelToTask(fsw.recorderTaskName, recorder)
        force_recs.append(recorder)

    period = scenario.config.outerLoopPeriod  # [s]
    dt = scenario.config.fswTimeStep  # [s]
    run_time = 2.0 * period + dt  # [s], include the next outer-loop update
    module.runScenario(scenario, run_time, showProgressBar=False)

    outer_times = on_time_rec.times() * macros.NANO2SEC  # [s]
    on_times = np.asarray(on_time_rec.OnTimeRequest)[:, :fsw.numThrusters]  # [s]
    force_times = force_recs[0].times() * macros.NANO2SEC  # [s]
    forces = np.column_stack([np.asarray(rec.input) for rec in force_recs])  # [N]
    max_forces = np.asarray(scenario.config.thrForces)  # [N]
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
    assert np.any(forces > 0.0)  # [N], the thrusters actually fire
    for window in range(2):
        samples = (force_times >= window * period) & (force_times < (window + 1) * period)
        delivered_impulse = np.sum(forces[samples], axis=0) * dt  # [N*s]
        requested_impulse = max_forces * on_times[window]  # [N*s]
        assert np.all(delivered_impulse <= requested_impulse + 2.0 * max_forces * dt)  # [N*s]

    position = np.asarray(scenario.posRec.r_CN_N)  # [m]
    velocity = np.asarray(scenario.posRec.v_CN_N)  # [m/s]
    position_error = np.linalg.norm(position, axis=1)  # [m]
    radial_velocity = np.einsum("ij,ij->i", position, velocity) / position_error  # [m/s]
    attitude_error = np.linalg.norm(np.asarray(scenario.attRec.sigma_BN), axis=1)  # [-]
    angular_rate = np.linalg.norm(np.asarray(scenario.attRec.omega_BN_B), axis=1)  # [rad/s]
    assert np.all(np.isfinite(position_error))
    assert np.all(np.isfinite(radial_velocity))
    assert np.all(np.isfinite(attitude_error))
    assert np.all(np.isfinite(angular_rate))
    # Initial motion is away from the target. A short run may only slow that
    # motion, so check radial velocity rather than requiring a smaller distance.
    assert radial_velocity[0] > 0.0  # [m/s]
    assert radial_velocity[-1] < radial_velocity[0]  # [m/s]
    assert angular_rate[-1] < 0.8 * angular_rate[0]  # [rad/s]
    assert attitude_error[-1] < 1.2 * attitude_error[0]  # [-]

    figures = scenario.pull_outputs()
    assert len(figures) == 2
    for figure in figures.values():
        plt.close(figure)
