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

"""Verify that ``mrpPDRust`` matches the established C ``mrpPD`` behavior."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.fswAlgorithms import mrpPD
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

mrpPDRust = pytest.importorskip(
    "Basilisk.fswAlgorithms.mrpPDRust",
    reason="Rust modules were not enabled for this Basilisk build.",
)


@pytest.mark.parametrize("set_external_torque", [False, True])
def test_mrp_pd_rust_matches_c_reference_cases(set_external_torque):
    """Run the existing ``mrpPD`` cases through both implementations.

    :param set_external_torque: Include a nonzero known body torque when true.
    """
    c_times, c_torques = _run_controller(mrpPD.mrpPD, set_external_torque)
    rust_times, rust_torques = _run_controller(
        mrpPDRust.mrpPDRust, set_external_torque
    )

    expected_times = np.array([0, 500000000, 1000000000], dtype=np.uint64)  # [ns]
    expected_torque = (
        np.array([-1.535, 3.665, -1.795])
        if set_external_torque
        else np.array([-1.435, 3.865, -1.495])
    )  # [N*m]
    expected_torques = np.repeat(expected_torque[None, :], 3, axis=0)  # [N*m]
    tolerance = 1.0e-12  # [N*m]

    np.testing.assert_array_equal(c_times, expected_times)
    np.testing.assert_array_equal(rust_times, expected_times)
    np.testing.assert_allclose(
        c_torques, expected_torques, rtol=0.0, atol=tolerance
    )
    np.testing.assert_allclose(
        rust_torques, expected_torques, rtol=0.0, atol=tolerance
    )
    np.testing.assert_allclose(rust_torques, c_torques, rtol=0.0, atol=tolerance)


def test_mrp_pd_rust_rejects_invalid_configuration():
    """Reject invalid scalar and fixed-array assignments immediately."""
    controller = mrpPDRust.mrpPDRust()
    controller.K = 0.15  # [N*m]
    controller.P = 150.0  # [N*m*s]
    controller.knownTorquePntB_B = [0.1, 0.2, 0.3]  # [N*m]

    with pytest.raises(
        BasiliskError,
        match=r"^mrpPDRust\.K must be finite and nonnegative$",
    ):
        controller.K = float("nan")  # [N*m]
    with pytest.raises(
        BasiliskError,
        match=r"^mrpPDRust\.P must be finite and nonnegative$",
    ):
        controller.setP(-1.0)  # [N*m*s]
    with pytest.raises(BasiliskError, match="components must be finite"):
        controller.knownTorquePntB_B = [0.1, float("inf"), 0.3]  # [N*m]
    with pytest.raises(BasiliskError, match="wrong number of values"):
        controller.knownTorquePntB_B = [0.1, 0.2]  # [N*m]

    assert controller.K == 0.15
    assert controller.getP() == 150.0
    assert controller.knownTorquePntB_B == [0.1, 0.2, 0.3]


def test_mrp_pd_rust_rejects_invalid_inertia_message():
    """Validate inertia received through the vehicle-configuration message."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    task_time_step = macros.sec2nano(0.5)  # [ns]
    process.addTask(simulation.CreateNewTask("controlTask", task_time_step))

    controller = mrpPDRust.mrpPDRust()
    guidance_message = messaging.AttGuidMsg().write(messaging.AttGuidMsgPayload())
    controller.guidInMsg.subscribeTo(guidance_message)

    vehicle_payload = messaging.VehicleConfigMsgPayload()
    vehicle_payload.ISCPntB_B = [
        1000.0,
        1.0,
        0.0,
        0.0,
        800.0,
        0.0,
        0.0,
        0.0,
        800.0,
    ]  # [kg*m^2]
    vehicle_message = messaging.VehicleConfigMsg().write(vehicle_payload)
    controller.vehConfigInMsg.subscribeTo(vehicle_message)
    simulation.AddModelToTask("controlTask", controller)

    with pytest.raises(BasiliskError, match="vehicle inertia must be symmetric"):
        simulation.InitializeSimulation()


def _run_controller(module_factory, set_external_torque):
    """Run one controller implementation with the established test inputs."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    task_time_step = macros.sec2nano(0.5)  # [ns]
    process.addTask(simulation.CreateNewTask("controlTask", task_time_step))

    controller = module_factory()
    controller.ModelTag = module_factory.__name__
    controller.K = 0.15  # [N*m]
    controller.P = 150.0  # [N*m*s]
    if set_external_torque:
        controller.knownTorquePntB_B = [0.1, 0.2, 0.3]  # [N*m]

    guidance_payload = messaging.AttGuidMsgPayload()
    guidance_payload.sigma_BR = [0.3, -0.5, 0.7]  # [-]
    guidance_payload.omega_BR_B = [0.010, -0.020, 0.015]  # [rad/s]
    guidance_payload.omega_RN_B = [-0.020, -0.010, 0.005]  # [rad/s]
    guidance_payload.domega_RN_B = [0.0002, 0.0003, 0.0001]  # [rad/s^2]
    guidance_message = messaging.AttGuidMsg().write(guidance_payload)

    vehicle_payload = messaging.VehicleConfigMsgPayload()
    vehicle_payload.ISCPntB_B = [
        1000.0,
        0.0,
        0.0,
        0.0,
        800.0,
        0.0,
        0.0,
        0.0,
        800.0,
    ]  # [kg*m^2]
    vehicle_message = messaging.VehicleConfigMsg().write(vehicle_payload)

    controller.guidInMsg.subscribeTo(guidance_message)
    controller.vehConfigInMsg.subscribeTo(vehicle_message)
    recorder = controller.cmdTorqueOutMsg.recorder()
    simulation.AddModelToTask("controlTask", controller)
    simulation.AddModelToTask("controlTask", recorder)

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(1.0))  # [ns]
    simulation.ExecuteSimulation()

    return recorder.times(), recorder.torqueRequestBody


if __name__ == "__main__":
    test_mrp_pd_rust_matches_c_reference_cases(False)
    test_mrp_pd_rust_matches_c_reference_cases(True)
