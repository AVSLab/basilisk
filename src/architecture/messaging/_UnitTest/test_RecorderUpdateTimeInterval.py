#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import pytest
import numpy as np

from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def test_update_time_interval():
    """Test changing a recorder's minimum update time during a paused simulation."""

    sc_sim = SimulationBaseClass.SimBaseClass()
    sim_process = sc_sim.CreateNewProcess("testProcess")

    task_time = macros.sec2nano(1.0)  # [ns]
    sim_process.addTask(sc_sim.CreateNewTask("testTask", task_time))

    msg_payload = messaging.CModuleTemplateMsgPayload()
    msg = messaging.CModuleTemplateMsg().write(msg_payload)

    initial_record_time = macros.sec2nano(10.0)  # [ns]
    msg_recorder = msg.recorder(initial_record_time)
    sc_sim.AddModelToTask("testTask", msg_recorder)

    sc_sim.InitializeSimulation()

    first_stop_time = macros.sec2nano(5.0)  # [ns]
    sc_sim.ConfigureStopTime(first_stop_time)
    sc_sim.ExecuteSimulation()

    short_record_time = macros.sec2nano(3.0)  # [ns]
    second_stop_time = macros.sec2nano(12.0)  # [ns]
    msg_recorder.updateTimeInterval(short_record_time)
    sc_sim.ConfigureStopTime(second_stop_time)
    sc_sim.ExecuteSimulation()

    long_record_time = macros.sec2nano(8.0)  # [ns]
    final_stop_time = macros.sec2nano(20.0)  # [ns]
    msg_recorder.updateTimeInterval(long_record_time)
    sc_sim.ConfigureStopTime(final_stop_time)
    sc_sim.ExecuteSimulation()

    first_record_time = macros.sec2nano(0.0)  # [ns]
    resumed_record_time = macros.sec2nano(6.0)  # [ns]
    second_resumed_record_time = macros.sec2nano(9.0)  # [ns]
    expected_times = np.array([
        first_record_time,
        resumed_record_time,
        second_resumed_record_time,
        second_stop_time,
        final_stop_time,
    ])  # [ns]

    np.testing.assert_array_equal(msg_recorder.times(), expected_times)


def test_record_on_change():
    """Test recording changed message payloads after the minimum update time."""

    sc_sim = SimulationBaseClass.SimBaseClass()
    sim_process = sc_sim.CreateNewProcess("testProcess")

    task_time = macros.sec2nano(1.0)  # [ns]
    sim_process.addTask(sc_sim.CreateNewTask("testTask", task_time))

    test_module = ChangingPayloadModule()
    sc_sim.AddModelToTask("testTask", test_module)

    minimum_record_time = macros.sec2nano(4.0)  # [ns]
    msg_recorder = test_module.dataOutMsg.recorder(minimum_record_time)
    msg_recorder.recordOnChange()
    sc_sim.AddModelToTask("testTask", msg_recorder)

    sc_sim.InitializeSimulation()

    final_stop_time = macros.sec2nano(13.0)  # [ns]
    sc_sim.ConfigureStopTime(final_stop_time)
    sc_sim.ExecuteSimulation()

    first_record_time = macros.sec2nano(0.0)  # [ns]
    first_eligible_change_time = macros.sec2nano(8.0)  # [ns]
    second_eligible_change_time = macros.sec2nano(12.0)  # [ns]
    expected_times = np.array([
        first_record_time,
        first_eligible_change_time,
        second_eligible_change_time,
    ])  # [ns]
    expected_values = np.array([
        [0, 0, 0],
        [1, 0, 0],
        [2, 0, 0],
    ])

    np.testing.assert_array_equal(msg_recorder.times(), expected_times)
    np.testing.assert_array_equal(msg_recorder.dataVector, expected_values)


def test_record_on_change_update_time_interval_after_skipped_record():
    """Test interval updates after a change-only recorder skips an unchanged payload."""

    sc_sim = SimulationBaseClass.SimBaseClass()
    sim_process = sc_sim.CreateNewProcess("testProcess")

    task_time = macros.sec2nano(1.0)  # [ns]
    sim_process.addTask(sc_sim.CreateNewTask("testTask", task_time))

    test_module = ChangingPayloadModule()
    sc_sim.AddModelToTask("testTask", test_module)

    initial_record_time = macros.sec2nano(4.0)  # [ns]
    msg_recorder = test_module.dataOutMsg.recorder(initial_record_time)
    msg_recorder.recordOnChange()
    sc_sim.AddModelToTask("testTask", msg_recorder)

    sc_sim.InitializeSimulation()

    first_stop_time = macros.sec2nano(6.0)  # [ns]
    sc_sim.ConfigureStopTime(first_stop_time)
    sc_sim.ExecuteSimulation()

    updated_record_time = macros.sec2nano(10.0)  # [ns]
    second_stop_time = macros.sec2nano(13.0)  # [ns]
    msg_recorder.updateTimeInterval(updated_record_time)
    sc_sim.ConfigureStopTime(second_stop_time)
    sc_sim.ExecuteSimulation()

    first_record_time = macros.sec2nano(0.0)  # [ns]
    np.testing.assert_array_equal(msg_recorder.times(), np.array([first_record_time]))

    final_stop_time = macros.sec2nano(14.0)  # [ns]
    sc_sim.ConfigureStopTime(final_stop_time)
    sc_sim.ExecuteSimulation()

    expected_times = np.array([
        first_record_time,
        final_stop_time,
    ])  # [ns]
    expected_values = np.array([
        [0, 0, 0],
        [2, 0, 0],
    ])

    np.testing.assert_array_equal(msg_recorder.times(), expected_times)
    np.testing.assert_array_equal(msg_recorder.dataVector, expected_values)


def test_record_on_change_unsupported_payload():
    """recordOnChange() on a payload with no equality support raises BasiliskError.

    VizUserInputMsgPayload contains std::vector<VizEventReply>, which the equality
    generator rejects and no manual specialization exists. Enabling change-only
    recording should fail explicitly instead of falling back to interval recording.
    """
    msg_recorder = messaging.VizUserInputMsg().recorder()

    with pytest.raises(Exception, match="does not support equality comparison.*PayloadEqualityTraits"):
        msg_recorder.recordOnChange()

    msg_recorder.recordOnChange(False)


def test_record_on_change_camera_image_payload():
    """CameraImageMsgPayload has a hand-written PayloadEqualityTraits specialization.

    recordOnChange() should only store a frame when the scalar fields differ from the
    last recorded payload. imagePointer is left null here, so equality reduces to
    comparing timeTag, valid, cameraID, imageType, and imageBufferLength.
    """

    sc_sim = SimulationBaseClass.SimBaseClass()
    sim_process = sc_sim.CreateNewProcess("testProcess")
    sim_process.addTask(sc_sim.CreateNewTask("testTask", macros.sec2nano(1.0)))

    test_module = ChangingCameraModule()
    sc_sim.AddModelToTask("testTask", test_module)

    minimum_record_time = macros.sec2nano(4.0)
    msg_recorder = test_module.cameraOutMsg.recorder(minimum_record_time)
    msg_recorder.recordOnChange()
    sc_sim.AddModelToTask("testTask", msg_recorder)

    sc_sim.InitializeSimulation()
    sc_sim.ConfigureStopTime(macros.sec2nano(10.0))
    sc_sim.ExecuteSimulation()

    # cameraID changes at t = 5 s. With a 4 s minimum interval:
    #   t = 0: recorded (cameraID = 0), next eligible = t = 4 s
    #   t = 4: payload unchanged → skipped,  next eligible = t = 8 s
    #   t = 8: payload changed (cameraID = 1) → recorded
    expected_times = np.array([
        macros.sec2nano(0.0),
        macros.sec2nano(8.0),
    ])
    expected_camera_ids = np.array([0, 1])

    np.testing.assert_array_equal(msg_recorder.times(), expected_times)
    np.testing.assert_array_equal(msg_recorder.cameraID, expected_camera_ids)


class ChangingCameraModule(sysModel.SysModel):
    """Write a CameraImageMsgPayload whose cameraID steps up at a fixed simulation time."""

    def __init__(self):
        super().__init__()
        self.cameraOutMsg = messaging.CameraImageMsg()
        self.change_time = macros.sec2nano(5.0)

    def Reset(self, CurrentSimNanos):
        self._write(CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos):
        self._write(CurrentSimNanos)

    def _write(self, CurrentSimNanos):
        payload = self.cameraOutMsg.zeroMsgPayload
        payload.cameraID = 1 if CurrentSimNanos >= self.change_time else 0
        payload.valid = 1
        self.cameraOutMsg.write(payload, CurrentSimNanos, self.moduleID)


class ChangingPayloadModule(sysModel.SysModel):
    """Write a payload that changes at fixed simulation times."""

    def __init__(self):
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.first_change_time = macros.sec2nano(5.0)  # [ns]
        self.second_change_time = macros.sec2nano(9.0)  # [ns]

    def Reset(self, CurrentSimNanos):
        self.write_payload(CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos):
        self.write_payload(CurrentSimNanos)

    def write_payload(self, CurrentSimNanos):
        payload_value = 0
        if CurrentSimNanos >= self.first_change_time:
            payload_value = 1
        if CurrentSimNanos >= self.second_change_time:
            payload_value = 2

        payload = self.dataOutMsg.zeroMsgPayload
        payload.dataVector = np.array([
            payload_value,
            0,
            0,
        ])
        self.dataOutMsg.write(payload, CurrentSimNanos, self.moduleID)


if __name__ == "__main__":
    test_update_time_interval()
    test_record_on_change()
    test_record_on_change_update_time_interval_after_skipped_record()
    test_record_on_change_unsupported_payload()
    test_record_on_change_camera_image_payload()
