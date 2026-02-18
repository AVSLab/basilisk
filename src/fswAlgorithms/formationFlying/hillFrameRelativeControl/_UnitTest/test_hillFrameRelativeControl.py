#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import hillFrameRelativeControl
from Basilisk.fswAlgorithms import hillStateConverter
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def _compute_truth_force(chief_r, chief_v, deputy_r, deputy_v, mass, k_gain, p_gain, r_ref, v_ref, mu):
    """Replicate module equations to form a deterministic truth force."""
    r_chief = np.array(chief_r, dtype=float)
    v_chief = np.array(chief_v, dtype=float)
    r_deputy = np.array(deputy_r, dtype=float)
    v_deputy = np.array(deputy_v, dtype=float)

    r_norm = np.linalg.norm(r_chief)
    h_vec = np.cross(r_chief, v_chief)
    h_norm = np.linalg.norm(h_vec)

    i_r = r_chief / r_norm
    i_h = h_vec / h_norm
    i_t = np.cross(i_h, i_r)
    dcm_hn = np.vstack((i_r, i_t, i_h))

    r_rel_n = r_deputy - r_chief
    v_rel_n = v_deputy - v_chief
    r_rel_h = dcm_hn @ r_rel_n

    omega_hn_h = np.array([0.0, 0.0, h_norm / (r_norm * r_norm)])
    v_rel_h = dcm_hn @ v_rel_n - np.cross(omega_hn_h, r_rel_h)

    r_dot = np.dot(r_chief, v_chief) / r_norm
    theta_dot = h_norm / (r_norm * r_norm)
    theta_ddot = -2.0 * r_dot * theta_dot / r_norm
    mu_over_r3 = mu / (r_norm ** 3)

    ff_a1 = np.array([
        (2.0 * mu_over_r3 + theta_dot ** 2) * r_rel_h[0] + theta_ddot * r_rel_h[1],
        -theta_ddot * r_rel_h[0] + (theta_dot ** 2 - mu_over_r3) * r_rel_h[1],
        -mu_over_r3 * r_rel_h[2]
    ])
    ff_a2 = np.array([
        2.0 * theta_dot * v_rel_h[1],
        -2.0 * theta_dot * v_rel_h[0],
        0.0
    ])

    k_mat = np.array(k_gain, dtype=float).reshape((3, 3))
    p_mat = np.array(p_gain, dtype=float).reshape((3, 3))
    a_cmd_h = -k_mat @ (r_rel_h - np.array(r_ref)) \
              - p_mat @ (v_rel_h - np.array(v_ref)) \
              - (ff_a1 + ff_a2)
    return mass * (dcm_hn.T @ a_cmd_h)


@pytest.mark.parametrize("accuracy", [1e-10])
@pytest.mark.parametrize("use_hill_state", [False, True])
def test_hillFrameRelativeControl(show_plots, accuracy, use_hill_state):
    r"""
    **Validation Test Description**

    This unit test verifies the ``hillFrameRelativeControl`` module in both supported input modes:
    direct deputy navigation input mode and precomputed Hill-state input mode. In both cases, the resulting inertial
    force command is compared to an independently computed truth value from the same control-law equations.

    **Test Parameters**

    The test parameters used are the following:

    Args:
        accuracy (float): absolute accuracy value used in the validation test
        use_hill_state (bool): if true, use hillStateConverter output; if false, use deputy nav input directly

    **Description of Variables Being Tested**

    In this file we are checking the values of the variable

    - ``forceRequestInertial``

    recorded from ``forceOutMsg`` and ``forceOutMsgC``. These outputs are compared against ``expected_force``, which
    is computed independently from chief/deputy inertial states, control gains, and references.
    """
    del show_plots

    sim = SimulationBaseClass.SimBaseClass()
    process_name = "TestProcess"
    task_name = "unitTask"
    process_rate = macros.sec2nano(0.1)

    process = sim.CreateNewProcess(process_name)
    process.addTask(sim.CreateNewTask(task_name, process_rate))

    module = hillFrameRelativeControl.HillFrameRelativeControl()
    module.ModelTag = "hillFrameRelativeControl"
    module.setMu(3.986004418e14)  # [m^3/s^2]
    module.setK([
        2.0e-6, 0.0, 0.0,
        0.0, 3.0e-6, 0.0,
        0.0, 0.0, 4.0e-6
    ])  # [1/s^2]
    module.setP([
        1.5e-3, 0.0, 0.0,
        0.0, 1.0e-3, 0.0,
        0.0, 0.0, 2.5e-3
    ])  # [1/s]
    module.setReferencePosition([100.0, -50.0, 20.0])  # [m]
    module.setReferenceVelocity([0.01, -0.02, 0.005])  # [m/s]

    chief_r = [7000e3, 200e3, -100e3]  # [m]
    chief_v = [50.0, 7500.0, 100.0]  # [m/s]
    deputy_r = [7000e3 + 120.0, 200e3 - 80.0, -100e3 + 30.0]  # [m]
    deputy_v = [50.2, 7499.8, 100.05]  # [m/s]

    chief_msg_data = messaging.NavTransMsgPayload()
    chief_msg_data.r_BN_N = chief_r
    chief_msg_data.v_BN_N = chief_v
    chief_msg = messaging.NavTransMsg().write(chief_msg_data)

    deputy_msg_data = messaging.NavTransMsgPayload()
    deputy_msg_data.r_BN_N = deputy_r
    deputy_msg_data.v_BN_N = deputy_v
    deputy_msg = messaging.NavTransMsg().write(deputy_msg_data)

    config_data = messaging.VehicleConfigMsgPayload()
    config_data.massSC = 420.0  # [kg]
    config_msg = messaging.VehicleConfigMsg().write(config_data)

    module.chiefTransInMsg.subscribeTo(chief_msg)
    if use_hill_state:
        hill_state_converter = hillStateConverter.hillStateConverter()
        hill_state_converter.ModelTag = "hillStateConverter"
        hill_state_converter.chiefStateInMsg.subscribeTo(chief_msg)
        hill_state_converter.depStateInMsg.subscribeTo(deputy_msg)
        sim.AddModelToTask(task_name, hill_state_converter)
        module.hillStateInMsg.subscribeTo(hill_state_converter.hillStateOutMsg)
    else:
        module.deputyTransInMsg.subscribeTo(deputy_msg)
    module.deputyVehicleConfigInMsg.subscribeTo(config_msg)

    recorder = module.forceOutMsg.recorder()
    recorder_c = module.forceOutMsgC.recorder()
    sim.AddModelToTask(task_name, module)
    sim.AddModelToTask(task_name, recorder)
    sim.AddModelToTask(task_name, recorder_c)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(process_rate)
    sim.ExecuteSimulation()

    expected_force = _compute_truth_force(
        chief_r=chief_r,
        chief_v=chief_v,
        deputy_r=deputy_r,
        deputy_v=deputy_v,
        mass=config_data.massSC,  # [kg]
        mu=3.986004418e14,  # [m^3/s^2]
        k_gain=[
            2.0e-6, 0.0, 0.0,
            0.0, 3.0e-6, 0.0,
            0.0, 0.0, 4.0e-6
        ],  # [1/s^2]
        p_gain=[
            1.5e-3, 0.0, 0.0,
            0.0, 1.0e-3, 0.0,
            0.0, 0.0, 2.5e-3
        ],  # [1/s]
        r_ref=[100.0, -50.0, 20.0],  # [m]
        v_ref=[0.01, -0.02, 0.005]  # [m/s]
    )

    for out_val, truth_val in zip(recorder.forceRequestInertial[-1], expected_force):
        assert out_val == pytest.approx(truth_val, abs=accuracy)

    for out_val, truth_val in zip(recorder_c.forceRequestInertial[-1], expected_force):
        assert out_val == pytest.approx(truth_val, abs=accuracy)

    for out_cpp, out_c in zip(recorder.forceRequestInertial[-1], recorder_c.forceRequestInertial[-1]):
        assert out_cpp == pytest.approx(out_c, abs=accuracy)


if __name__ == "__main__":
    test_hillFrameRelativeControl(False, 1e-10, False)
