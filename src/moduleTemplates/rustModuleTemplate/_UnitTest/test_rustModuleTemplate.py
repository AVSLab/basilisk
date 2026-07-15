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

"""Validate the Rust module template message behavior."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

rustModuleTemplate = pytest.importorskip(
    "Basilisk.moduleTemplates.rustModuleTemplate",
    reason="Rust modules were not enabled for this Basilisk build.",
)


def test_rust_module_template():
    """Verify that the template reads an optional message and increments output data."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    task_time_step = macros.sec2nano(0.5)  # [ns]
    process.addTask(simulation.CreateNewTask("testTask", task_time_step))

    module = rustModuleTemplate.rustModuleTemplate()
    simulation.AddModelToTask("testTask", module)

    input_payload = messaging.CModuleTemplateMsgPayload()
    input_payload.dataVector = [1.0, -0.5, 0.7]  # [-]
    input_message = messaging.CModuleTemplateMsg().write(input_payload)
    module.dataInMsg.subscribeTo(input_message)

    output_log = module.dataOutMsg.recorder()
    simulation.AddModelToTask("testTask", output_log)

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(1.0))  # [ns]
    simulation.ExecuteSimulation()

    assert len(output_log.dataVector) > 0
    np.testing.assert_allclose(output_log.dataVector[:, 1], input_payload.dataVector[1])
    np.testing.assert_allclose(output_log.dataVector[:, 2], input_payload.dataVector[2])
    np.testing.assert_allclose(np.diff(output_log.dataVector[:, 0]), 1.0)  # [-]
