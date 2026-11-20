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

"""Sampled hold, illumination, freshness, and readiness controller tests."""

import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import simpleInstrumentController
from Basilisk.utilities import macros


def test_controller_import_preserves_the_illumination_message_alias():
    """Importing the controller must preserve the public eclipse payload binding."""
    payload = messaging.EclipseMsgPayload(illuminationFactor=0.75)  # [-]
    message = messaging.EclipseMsg().write(payload)
    assert message.read().illuminationFactor == pytest.approx(0.75)


def controller(
    cumulative=False,
    hold_s=3.0,  # [s]
    fresh=False,
):
    """Create a controller with independently writable physical constraints."""
    module = simpleInstrumentController.simpleInstrumentController()
    module.attErrTolerance = 0.01  # [MRP norm]
    module.useRateTolerance = 1
    module.rateErrTolerance = 0.01  # [rad/s]
    module.useDurationImaging = 1
    module.useCumulativeHold = int(cumulative)
    module.acquisitionTime = macros.sec2nano(hold_s)  # [ns]
    module.allowedTime = macros.sec2nano(30.0)  # [ns]
    module.requireFreshMessages = int(fresh)
    module.maximumGuidanceAge = macros.sec2nano(1.0)  # [ns]
    module.maximumAccessAge = macros.sec2nano(1.0)  # [ns]
    module.useIlluminationConstraint = 1
    module.illuminationThreshold = 0.5  # [-]
    messages = (
        messaging.AttGuidMsg(), messaging.AccessMsg(), messaging.EclipseMsg(),
        messaging.DeviceStatusMsg(),
    )
    for reader, message in zip(
        (module.attGuidInMsg, module.locationAccessInMsg, module.illuminationInMsg,
         module.captureReadyInMsg), messages
    ):
        reader.subscribeTo(message)
    module.SelfInit()
    module.Reset(0)
    return module, messages


def sample(module, messages, time_s, invalid=None, illumination=0.8, ready=True):
    """Write current samples, then execute exactly one native controller tick."""
    time_ns = macros.sec2nano(time_s)  # [ns]
    guidance = messaging.AttGuidMsgPayload()
    if invalid == "pointing":
        guidance.sigma_BR = [0.02, 0, 0]  # [MRP]
    if invalid == "rate":
        guidance.omega_BR_B = [0.02, 0, 0]  # [rad/s]
    access = messaging.AccessMsgPayload()
    access.hasAccess = int(invalid != "access")
    eclipse = messaging.EclipseMsgPayload()
    eclipse.illuminationFactor = 0.1 if invalid == "illumination" else illumination  # [-]
    readiness = messaging.DeviceStatusMsgPayload()
    readiness.deviceStatus = int(ready)
    for message, payload in zip(messages, (guidance, access, eclipse, readiness)):
        message.write(payload, time_ns)
    module.UpdateState(time_ns)
    return module.deviceCmdOutMsg.read().deviceCmd


@pytest.mark.parametrize("cumulative, capture_s", [(False, 6), (True, 5)])
@pytest.mark.parametrize("constraint", ["access", "pointing", "rate", "illumination"])
def test_hold_counts_only_intervals_with_two_valid_endpoints(cumulative, capture_s, constraint):
    """Each constraint interrupts a continuous hold and pauses a cumulative hold."""
    module, messages = controller(cumulative=cumulative)
    commands = [sample(module, messages, time, invalid=constraint if time == 2 else None)
                for time in range(8)]
    assert commands == [int(time == capture_s) for time in range(8)]
    assert module.captureTime == macros.sec2nano(capture_s)
    assert module.elapsedTime == macros.sec2nano(3.0)


def test_mean_illumination_uses_only_valid_hold_intervals():
    """Invalid intervals contribute neither time nor illumination integral."""
    module, messages = controller(cumulative=True, hold_s=2.0)
    commands = []
    for time, illumination in enumerate([0.6, 0.8, 0.1, 0.9, 0.5]):
        commands.append(sample(module, messages, time, illumination=illumination))
    assert commands == [0, 0, 0, 0, 1]
    assert module.captureIllumination == pytest.approx(0.7)
    assert module.elapsedTime == macros.sec2nano(2.0)


@pytest.mark.parametrize("cumulative, capture_s", [(False, 7), (True, 6)])
def test_disabled_controller_interrupts_the_hold(cumulative, capture_s):
    """Disabled samples cannot become endpoints of valid hold intervals."""
    module, messages = controller(cumulative=cumulative)
    commands = []
    for time in range(9):
        module.controllerStatus = int(time not in (2, 3))
        commands.append(sample(module, messages, time))
    assert commands == [int(time == capture_s) for time in range(9)]


def test_readiness_waits_for_storage_without_losing_the_hold():
    """Storage readiness blocks the command while pointing hold continues."""
    module, messages = controller()
    for time in range(4):
        assert sample(module, messages, time, ready=False) == 0
    assert not module.imaged
    assert sample(module, messages, 4, ready=True) == 1
    assert module.elapsedTime == macros.sec2nano(4.0)


def test_unwritten_old_and_future_messages_cannot_acquire():
    """Zero payloads and invalid message timestamps cannot satisfy a zero hold."""
    module, messages = controller(hold_s=0.0, fresh=True)
    module.UpdateState(macros.sec2nano(1.0))
    assert not module.imaged
    assert sample(module, messages, 0) == 0  # Guidance must follow reset.
    module.UpdateState(macros.sec2nano(2.0))
    assert not module.imaged
    # Write a valid future sample, reset the controller earlier, then read it early.
    sample(module, messages, 5, ready=False)
    module.Reset(macros.sec2nano(2.0))
    module.UpdateState(macros.sec2nano(3.0))
    assert not module.imaged
    assert sample(module, messages, 4) == 1


def test_reset_clears_hold_and_command_before_a_new_target():
    """A reset discards an incomplete hold and clears any previous image pulse."""
    module, messages = controller(hold_s=2.0)
    sample(module, messages, 0)
    sample(module, messages, 1)
    module.Reset(macros.sec2nano(1.0))
    assert module.elapsedTime == 0
    assert sample(module, messages, 2) == 0
    assert sample(module, messages, 3) == 0
    assert sample(module, messages, 4) == 1
    module.Reset(macros.sec2nano(4.0))
    assert module.deviceCmdOutMsg.read().deviceCmd == 0
    assert not module.imaged


def test_irregular_sampling_accumulates_elapsed_time():
    """Hold duration follows actual sample intervals rather than update counts."""
    module, messages = controller(hold_s=3.0)
    assert [sample(module, messages, time) for time in (0, 0.25, 1.5, 3)] == [0, 0, 0, 1]
    assert module.elapsedTime == macros.sec2nano(3.0)
