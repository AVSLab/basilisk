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

import numpy as np

from Basilisk.architecture import messaging
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


if __name__ == "__main__":
    test_update_time_interval()
