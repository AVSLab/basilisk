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

"""Check the shared update and reset behavior of the C and C++ templates."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cModuleTemplate, cppModuleTemplate
from Basilisk.utilities import SimulationBaseClass


@pytest.mark.parametrize("module_type", ["C", "C++"])
@pytest.mark.parametrize("connect_input", [False, True])
def test_template_updates_and_resets(module_type, connect_input):
    """Preserve outputs, counters, input data, and timestamps across lifecycle calls.

    Exercise both templates with and without the optional input connected.
    Change the input between updates, reset after two updates, and disconnect
    before the last update to check that prior payloads are never reused.
    """
    if module_type == "C":
        module = cModuleTemplate.cModuleTemplate()
        module.dummy = 9.0  # [-]

        def get_counter():
            """Read the C template's public sample counter."""
            return module.dummy
    else:
        module = cppModuleTemplate.CppModuleTemplate()
        module.setDummy(9.0)  # [-]
        get_counter = module.getDummy

    # Attach the framework logger before exercising lifecycle methods directly.
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    process.addTask(simulation.CreateNewTask("testTask", 10))  # [ns]
    simulation.AddModelToTask("testTask", module)

    input_payload = messaging.CModuleTemplateMsgPayload()
    input_payload.dataVector = [1.0, -0.5, 0.7]  # [-]
    input_message = messaging.CModuleTemplateMsg().write(input_payload)
    if connect_input:
        module.dataInMsg.subscribeTo(input_message)

    recorder = module.dataOutMsg.recorder()
    module.SelfInit()
    counters = []

    module.Reset(10)  # [ns]
    recorder.UpdateState(10)  # [ns]
    counters.append(get_counter())

    module.UpdateState(20)  # [ns]
    recorder.UpdateState(20)  # [ns]
    counters.append(get_counter())
    np.testing.assert_array_equal(input_message.read().dataVector, [1.0, -0.5, 0.7])
    if module_type == "C":
        expected_input = [1.0, -0.5, 0.7] if connect_input else [0.0, 0.0, 0.0]  # [-]
        np.testing.assert_array_equal(module.inputVector, expected_input)

    input_payload.dataVector = [-3.0, 4.0, -5.0]  # [-]
    input_message.write(input_payload)
    module.UpdateState(30)  # [ns]
    recorder.UpdateState(30)  # [ns]
    counters.append(get_counter())
    if module_type == "C":
        expected_input = [-3.0, 4.0, -5.0] if connect_input else [0.0, 0.0, 0.0]  # [-]
        np.testing.assert_array_equal(module.inputVector, expected_input)

    module.Reset(40)  # [ns]
    recorder.UpdateState(40)  # [ns]
    counters.append(get_counter())
    if module_type == "C":
        # Reset preserves the stored input vector, as in the original template.
        np.testing.assert_array_equal(module.inputVector, expected_input)

    module.UpdateState(50)  # [ns]
    recorder.UpdateState(50)  # [ns]
    counters.append(get_counter())
    np.testing.assert_array_equal(input_message.read().dataVector, [-3.0, 4.0, -5.0])

    module.dataInMsg.unsubscribe()
    module.UpdateState(60)  # [ns]
    recorder.UpdateState(60)  # [ns]
    counters.append(get_counter())
    if module_type == "C":
        np.testing.assert_array_equal(module.inputVector, [0.0, 0.0, 0.0])

    if connect_input:
        expected_output = [
            [0.0, 0.0, 0.0],
            [2.0, -0.5, 0.7],
            [-1.0, 4.0, -5.0],
            [0.0, 0.0, 0.0],
            [-2.0, 4.0, -5.0],
            [2.0, 0.0, 0.0],
        ]  # [-]
    else:
        expected_output = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
        ]  # [-]

    np.testing.assert_array_equal(recorder.dataVector, expected_output)
    np.testing.assert_array_equal(counters, [0.0, 1.0, 2.0, 0.0, 1.0, 2.0])
    expected_times = [10, 20, 30, 40, 50, 60]  # [ns]
    np.testing.assert_array_equal(recorder.times(), expected_times)
    np.testing.assert_array_equal(recorder.timesWritten(), expected_times)
