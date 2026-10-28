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

"""Regression tests for optical navigation measurement freshness."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import faultDetection, horizonOpNav, opNavPoint, pixelLineConverter
from Basilisk.utilities import macros


def setup_module_inputs(module_name, fault_mode=0):
    """Connect unwritten measurements and a calibrated camera to one module."""
    camera = messaging.CameraConfigMsgPayload()
    camera.fieldOfView = 0.8  # [rad]
    camera.resolution = [512, 512]  # [pixel]
    camera_msg = messaging.CameraConfigMsg().write(camera)
    attitude_msg = messaging.NavAttMsg().write(messaging.NavAttMsgPayload())

    if module_name == "pixelLineConverter":
        module = pixelLineConverter.pixelLineConverter()
        payload = messaging.OpNavCirclesMsgPayload()
        payload.planetIds = [2]
        payload.circlesCenters = [200.0, 250.0]  # [pixel]
        payload.circlesRadii = [75.0]  # [pixel]
        payload.uncertainty = np.eye(3).flatten().tolist()  # [pixel^2]
        inputs = [messaging.OpNavCirclesMsg()]
        module.circlesInMsg.subscribeTo(inputs[0])
        payloads = [payload]
    elif module_name == "horizonOpNav":
        module = horizonOpNav.horizonOpNav()
        module.noiseSF = 1.0  # [-]
        payload = messaging.OpNavLimbMsgPayload()
        payload.planetIds = 2
        angles = np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False)  # [rad]
        points = np.column_stack((200.0 + 75.0 * np.cos(angles),
                                  250.0 + 75.0 * np.sin(angles)))  # [pixel]
        payload.limbPoints = points.flatten().tolist()
        payload.numLimbPoints = len(points)
        inputs = [messaging.OpNavLimbMsg()]
        module.limbInMsg.subscribeTo(inputs[0])
        payloads = [payload]
    else:
        module = faultDetection.faultDetection()
        module.faultMode = fault_mode
        module.sigmaFault = 3.0  # [-]
        inputs = [messaging.OpNavMsg(), messaging.OpNavMsg()]
        payloads = []
        for offset in (0.0, 1.0):  # [m]
            payload = messaging.OpNavMsgPayload()
            payload.planetID = 2
            payload.r_BN_C = [1e6 + offset, 2e6, -8e6]  # [m]
            payload.r_BN_B = payload.r_BN_C
            payload.r_BN_N = payload.r_BN_C
            payload.covar_C = np.eye(3).flatten().tolist()  # [m^2]
            payload.covar_B = payload.covar_C
            payload.covar_N = payload.covar_C
            payloads.append(payload)
        module.navMeasPrimaryInMsg.subscribeTo(inputs[0])
        module.navMeasSecondaryInMsg.subscribeTo(inputs[1])

    for payload in payloads:
        payload.valid = 1
    module.cameraConfigInMsg.subscribeTo(camera_msg)
    module.attInMsg.subscribeTo(attitude_msg)
    module.SelfInit()
    module.Reset(0)
    return module, inputs, payloads, camera_msg, attitude_msg


def setup_guidance(module, camera_msg, attitude_msg, time_out):
    """Connect downstream guidance with a nonzero search rate."""
    point = opNavPoint.opNavPoint()
    point.alignAxis_C = [0.0, 0.0, 1.0]
    point.smallAngle = 1e-6  # [rad]
    point.timeOut = time_out
    point.omega_RN_B = [0.0, 0.0, 0.1]  # [rad/s]
    point.opnavDataInMsg.subscribeTo(module.opNavOutMsg)
    point.cameraConfigInMsg.subscribeTo(camera_msg)
    point.imuInMsg.subscribeTo(attitude_msg)
    point.SelfInit()
    point.Reset(0)
    return point


@pytest.mark.parametrize("module_name, fault_mode", [
    ("pixelLineConverter", 0), ("horizonOpNav", 0),
    ("faultDetection", 0), ("faultDetection", 1), ("faultDetection", 2),
])
def test_retained_measurement_timeout(module_name, fault_mode):
    """Retained inputs must expire downstream and a later write must recover.

    Each module receives images at 0 s and 4 s and runs every second upstream
    of ``opNavPoint``. Check output validity, finite navigation data, and the
    guidance search rate at the 2 s timeout boundary. Updating attitude alone
    must not make an image fresh. Reset must accept a time-zero image again.
    """
    module, inputs, payloads, camera_msg, attitude_msg = setup_module_inputs(module_name, fault_mode)
    point = setup_guidance(module, camera_msg, attitude_msg, 2.0)  # [s]

    module.UpdateState(0)
    assert not messaging.OpNavMsg_C_isWritten(module.opNavOutMsg)
    for step in range(7):
        call_time = macros.sec2nano(float(step))  # [ns]
        attitude = messaging.NavAttMsgPayload()
        attitude.sigma_BN = [0.0, 0.0, 0.0] if step == 0 else [0.1, -0.2, 0.3]
        attitude_msg.write(attitude, call_time)
        if step in (0, 4):
            for source, payload in zip(inputs, payloads):
                source.write(payload, call_time)
        module.UpdateState(call_time)
        output = module.opNavOutMsg.read()
        assert output.valid == 1
        assert np.isfinite(output.r_BN_C).all()
        assert np.isfinite(output.covar_C).all()
        assert np.linalg.norm(output.r_BN_C) > 0.0
        if step in (0, 4):
            saved_position = list(output.r_BN_N)
        np.testing.assert_array_equal(output.r_BN_N, saved_position)
        expected_write_time = macros.sec2nano(0.0 if step < 4 else 4.0)  # [ns]
        assert messaging.OpNavMsg_C_timeWritten(module.opNavOutMsg) == expected_write_time
        point.UpdateState(call_time)
        tracking = step in (0, 1, 4, 5)
        expected_rate = np.zeros(3) if tracking else point.omega_RN_B
        np.testing.assert_allclose(point.attGuidanceOutMsg.read().omega_RN_B,
                                   expected_rate, atol=1e-12)  # [rad/s]

    # Rewriting a duplicate or older timestamp must not revive the observation.
    for write_time in (4.0, 3.0):  # [s]
        for source, payload in zip(inputs, payloads):
            source.write(payload, macros.sec2nano(write_time))
        module.UpdateState(macros.sec2nano(7.0))  # [ns]
        assert messaging.OpNavMsg_C_timeWritten(module.opNavOutMsg) == macros.sec2nano(4.0)  # [ns]
        point.UpdateState(macros.sec2nano(7.0))  # [ns]
        np.testing.assert_allclose(point.attGuidanceOutMsg.read().omega_RN_B,
                                   point.omega_RN_B, atol=1e-12)  # [rad/s]

    # Invalid writes also advance freshness; older valid data cannot supersede them.
    for source, payload in zip(inputs, payloads):
        payload.valid = 0
        source.write(payload, macros.sec2nano(8.0))  # [ns]
    module.UpdateState(macros.sec2nano(8.0))  # [ns]
    assert module.opNavOutMsg.read().valid == 0
    assert messaging.OpNavMsg_C_timeWritten(module.opNavOutMsg) == macros.sec2nano(8.0)  # [ns]
    for write_time in (8.0, 7.0, 9.0):  # [s]
        for source, payload in zip(inputs, payloads):
            payload.valid = 1
            source.write(payload, macros.sec2nano(write_time))
        module.UpdateState(macros.sec2nano(9.0))  # [ns]
        assert module.opNavOutMsg.read().valid == int(write_time == 9.0)
        expected_write_time = macros.sec2nano(9.0 if write_time == 9.0 else 8.0)  # [ns]
        assert messaging.OpNavMsg_C_timeWritten(module.opNavOutMsg) == expected_write_time

    module.Reset(0)
    for source, payload in zip(inputs, payloads):
        source.write(payload, 0)
    module.UpdateState(0)
    assert module.opNavOutMsg.read().valid == 1
    module.UpdateState(macros.sec2nano(1.0))  # [ns]
    assert module.opNavOutMsg.read().valid == 1
    assert messaging.OpNavMsg_C_timeWritten(module.opNavOutMsg) == 0  # [ns]


@pytest.mark.parametrize("module_name", ["pixelLineConverter", "horizonOpNav", "faultDetection"])
@pytest.mark.parametrize("producer_period, consumer_period", [(1, 2), (2, 1)])  # [s]
def test_different_update_rates(module_name, producer_period, consumer_period):
    """Deliver observations across unequal update rates without refreshing their age.

    Observations arrive on odd producer updates. A slower consumer therefore
    reads after an intervening producer update, reproducing the lost-observation
    regression. A faster consumer reads retained outputs repeatedly. In both
    cases guidance must track the latest observation and time out after writes stop.
    """
    module, inputs, payloads, camera_msg, attitude_msg = setup_module_inputs(module_name)
    time_out = 4.0  # [s]
    point = setup_guidance(module, camera_msg, attitude_msg, time_out)
    for source, payload in zip(inputs, payloads):
        payload.valid = 0
        source.write(payload, 0)

    image_times = (producer_period, 3 * producer_period, 5 * producer_period)  # [s]
    last_image_time = None
    stop_time = image_times[-1] + int(time_out) + consumer_period  # [s]
    for step in range(stop_time + 1):
        call_time = macros.sec2nano(float(step))  # [ns]
        if step in image_times:
            for source, payload in zip(inputs, payloads):
                payload.valid = 1
                source.write(payload, call_time)
            last_image_time = step
        if step % producer_period == 0:
            module.UpdateState(call_time)
        if step % consumer_period == 0:
            point.UpdateState(call_time)
            if last_image_time is not None:
                expected_write_time = macros.sec2nano(float(last_image_time))  # [ns]
                assert point.hasReceivedMeasurement
                assert point.lastTime == expected_write_time
                assert messaging.OpNavMsg_C_timeWritten(module.opNavOutMsg) == expected_write_time
            tracking = last_image_time is not None and step - last_image_time < time_out
            expected_rate = np.zeros(3) if tracking else point.omega_RN_B
            np.testing.assert_allclose(point.attGuidanceOutMsg.read().omega_RN_B,
                                       expected_rate, atol=1e-12)  # [rad/s]


@pytest.mark.parametrize("fault_mode", [0, 1, 2])
@pytest.mark.parametrize("fresh_input", [0, 1])
def test_fault_detection_independent_freshness(fault_mode, fresh_input):
    """Apply each fault mode to fresh inputs without reusing the other source.

    A fresh primary is usable in modes 0 and 1; a fresh secondary alone is
    usable only in mode 0. Both fresh inputs are required for mode 2. When
    both are fresh, retain the existing merge and dissimilar-check behavior.
    """
    module, inputs, payloads, camera_msg, attitude_msg = setup_module_inputs("faultDetection", fault_mode)
    for source, payload in zip(inputs, payloads):
        source.write(payload, 0)
    module.UpdateState(0)
    assert module.opNavOutMsg.read().valid == 1

    for step in (1, 2):
        call_time = macros.sec2nano(float(step))  # [ns]
        inputs[fresh_input].write(payloads[fresh_input], call_time)
        module.UpdateState(call_time)
        output = module.opNavOutMsg.read()
        expected_valid = fault_mode == 0 or (fault_mode == 1 and fresh_input == 0)
        assert output.valid == int(expected_valid)
        assert output.faultDetected == 0
        if expected_valid:
            np.testing.assert_allclose(output.r_BN_C, payloads[fresh_input].r_BN_C)
            np.testing.assert_allclose(output.covar_C, payloads[fresh_input].covar_C)

    for step, secondary_offset in ((3, 1.0), (4, 100.0)):  # [m]
        call_time = macros.sec2nano(float(step))  # [ns]
        payloads[1].r_BN_C = [1e6 + secondary_offset, 2e6, -8e6]  # [m]
        for source, payload in zip(inputs, payloads):
            source.write(payload, call_time)
        module.UpdateState(call_time)
        output = module.opNavOutMsg.read()
        assert output.valid == 1
        assert output.faultDetected == int(step == 4)
        if step == 4:
            expected_position = payloads[1].r_BN_C
            expected_covariance = payloads[1].covar_C
        elif fault_mode == 0:
            expected_position = (np.array(payloads[0].r_BN_C) + payloads[1].r_BN_C) / 2
            expected_covariance = np.eye(3).flatten() / 2  # [m^2]
        else:
            expected_position = payloads[0].r_BN_C
            expected_covariance = payloads[0].covar_C
        np.testing.assert_allclose(output.r_BN_C, expected_position)
        np.testing.assert_allclose(output.covar_C, expected_covariance)
