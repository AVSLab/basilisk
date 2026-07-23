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

"""Validate the Rust module template's public API, ABI, and message behavior."""

import ctypes

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

rustModuleTemplate = pytest.importorskip(
    "Basilisk.moduleTemplates.rustModuleTemplate",
    reason="Rust modules were not enabled for this Basilisk build.",
)


class _ExpectedBskModuleRuntime(ctypes.Structure):
    """Mirror the generated C ``BskRustModuleRuntime`` ABI."""

    _fields_ = [
        ("module_id", ctypes.c_int64),
        ("model_tag", ctypes.c_void_p),
        ("call_counts", ctypes.c_uint64),
        ("rng_seed", ctypes.c_uint32),
    ]


class _ExpectedMsgHeader(ctypes.Structure):
    """Mirror the C message header embedded in every message port."""

    _fields_ = [
        ("is_linked", ctypes.c_int64),
        ("time_written", ctypes.c_uint64),
        ("module_id", ctypes.c_int64),
        ("write_counter", ctypes.c_uint64),
    ]


class _ExpectedCModuleTemplatePayload(ctypes.Structure):
    """Mirror ``CModuleTemplateMsgPayload`` for the compatibility ABI check."""

    _fields_ = [("data_vector", ctypes.c_double * 3)]


class _ExpectedCModuleTemplatePort(ctypes.Structure):
    """Mirror the generated ``CModuleTemplateMsg_C`` port type."""

    _fields_ = [
        ("header", _ExpectedMsgHeader),
        ("payload", _ExpectedCModuleTemplatePayload),
        ("payload_pointer", ctypes.c_void_p),
        ("header_pointer", ctypes.c_void_p),
    ]


class _ExpectedRustModuleTemplateConfig(ctypes.Structure):
    """Mirror the intended 64-bit Rust/C++ config layout."""

    _fields_ = [
        ("runtime", _ExpectedBskModuleRuntime),
        ("dummy", ctypes.c_double),
        ("data_in_msg", _ExpectedCModuleTemplatePort),
        ("data_out_msg", _ExpectedCModuleTemplatePort),
        ("bsk_logger", ctypes.c_void_p),
    ]


def test_rust_module_template_python_api():
    """Freeze the Python-visible config fields and wrapper ownership contract."""
    expected_fields = {"runtime", "dummy", "dataInMsg", "dataOutMsg", "bskLogger"}
    assert expected_fields.issubset(rustModuleTemplate.RustModuleTemplateConfig.__dict__)
    assert expected_fields.issubset(rustModuleTemplate.rustModuleTemplate.__dict__)

    config = rustModuleTemplate.RustModuleTemplateConfig()
    assert config.dummy == 0.0
    config.dummy = 12.5  # [-]
    assert config.dummy == 12.5
    assert hasattr(config.dataInMsg, "subscribeTo")
    assert hasattr(config.dataOutMsg, "recorder")

    config_address = int(config.this)
    wrapper = config.createWrapper()
    wrapped_config = wrapper.getConfig()

    assert config.thisown is False
    assert wrapper.thisown is True
    assert wrapped_config.thisown is False
    assert int(wrapped_config.this) == config_address
    assert wrapper.dummy == 12.5


def test_rust_module_template_abi_layout():
    """Check that SWIG exposes the established 64-bit Rust/C++ ABI layout."""
    if ctypes.sizeof(ctypes.c_void_p) != 8:
        pytest.skip("The current Rust module ABI baseline covers 64-bit platforms.")

    assert ctypes.sizeof(_ExpectedBskModuleRuntime) == 32
    assert ctypes.alignment(_ExpectedBskModuleRuntime) == 8
    assert _ExpectedBskModuleRuntime.module_id.offset == 0
    assert _ExpectedBskModuleRuntime.model_tag.offset == 8
    assert _ExpectedBskModuleRuntime.call_counts.offset == 16
    assert _ExpectedBskModuleRuntime.rng_seed.offset == 24

    assert ctypes.sizeof(_ExpectedCModuleTemplatePort) == 72
    assert ctypes.alignment(_ExpectedCModuleTemplatePort) == 8
    assert _ExpectedCModuleTemplatePort.header.offset == 0
    assert _ExpectedCModuleTemplatePort.payload.offset == 32
    assert _ExpectedCModuleTemplatePort.payload_pointer.offset == 56
    assert _ExpectedCModuleTemplatePort.header_pointer.offset == 64

    assert ctypes.sizeof(_ExpectedRustModuleTemplateConfig) == 192
    assert ctypes.alignment(_ExpectedRustModuleTemplateConfig) == 8
    assert _ExpectedRustModuleTemplateConfig.runtime.offset == 0
    assert _ExpectedRustModuleTemplateConfig.dummy.offset == 32
    assert _ExpectedRustModuleTemplateConfig.data_in_msg.offset == 40
    assert _ExpectedRustModuleTemplateConfig.data_out_msg.offset == 112
    assert _ExpectedRustModuleTemplateConfig.bsk_logger.offset == 184

    config = rustModuleTemplate.RustModuleTemplateConfig()
    data_in_msg = config.dataInMsg
    data_out_msg = config.dataOutMsg
    config_address = int(config.this)

    assert int(data_in_msg.this) - config_address == _ExpectedRustModuleTemplateConfig.data_in_msg.offset
    assert int(data_out_msg.this) - config_address == _ExpectedRustModuleTemplateConfig.data_out_msg.offset

    raw_config = _ExpectedRustModuleTemplateConfig.from_address(config_address)
    config.dummy = 7.25  # [-]
    assert raw_config.dummy == 7.25
    raw_config.dummy = -3.5  # [-]
    assert config.dummy == -3.5
    assert raw_config.bsk_logger is None


@pytest.mark.parametrize("connect_input", [False, True])
def test_rust_module_template(connect_input):
    """Verify reset and update behavior with the optional input unlinked or linked."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    task_time_step = macros.sec2nano(0.5)  # [ns]
    process.addTask(simulation.CreateNewTask("testTask", task_time_step))

    module = rustModuleTemplate.rustModuleTemplate()
    module.dummy = 99.0  # [-]
    simulation.AddModelToTask("testTask", module)

    input_payload = messaging.CModuleTemplateMsgPayload()
    input_payload.dataVector = [1.0, -0.5, 0.7]  # [-]
    if connect_input:
        input_message = messaging.CModuleTemplateMsg().write(input_payload)
        module.dataInMsg.subscribeTo(input_message)

    output_log = module.dataOutMsg.recorder()
    simulation.AddModelToTask("testTask", output_log)

    simulation.InitializeSimulation()
    assert module.dummy == 0.0
    simulation.ConfigureStopTime(macros.sec2nano(1.0))  # [ns]
    simulation.ExecuteSimulation()

    expected_times = np.array([0, 500000000, 1000000000], dtype=np.uint64)  # [ns]
    if connect_input:
        expected_output = np.array(
            [[2.0, -0.5, 0.7], [3.0, -0.5, 0.7], [4.0, -0.5, 0.7]]
        )  # [-]
    else:
        expected_output = np.array(
            [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
        )  # [-]

    np.testing.assert_array_equal(output_log.times(), expected_times)
    np.testing.assert_allclose(output_log.dataVector, expected_output)
    assert module.dummy == 3.0
