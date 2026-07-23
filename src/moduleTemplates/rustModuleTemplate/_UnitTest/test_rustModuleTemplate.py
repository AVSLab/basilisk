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
from Basilisk.architecture.bskLogging import BasiliskError
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


class _ExpectedBskModuleContext(ctypes.Structure):
    """Mirror the generated C ``BskRustModuleContext`` ABI."""

    _fields_ = [
        ("runtime", _ExpectedBskModuleRuntime),
        ("bsk_logger", ctypes.c_void_p),
    ]


class _ExpectedMsgHeader(ctypes.Structure):
    """Mirror the C message header embedded in every message port."""

    _fields_ = [
        ("is_linked", ctypes.c_int64),
        ("is_written", ctypes.c_int64),
        ("time_written", ctypes.c_uint64),
        ("module_id", ctypes.c_int64),
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
        ("dummy", ctypes.c_double),
        ("increment", ctypes.c_double),
        ("data_in_msg", _ExpectedCModuleTemplatePort),
        ("data_out_msg", _ExpectedCModuleTemplatePort),
        ("panic_on_update", ctypes.c_bool),
    ]


_BSK_RUST_ERROR_EXPECTED = 1
_BSK_RUST_ERROR_PANIC = 2
_BSK_RUST_ERROR_INVALID_ARGUMENT = 3


def _consume_abi_error(extension, error):
    """Return and release the classification and message for an ABI error."""
    error_kind = extension.BskRustError_kind
    error_kind.argtypes = [ctypes.c_void_p]
    error_kind.restype = ctypes.c_uint32
    error_message = extension.BskRustError_message
    error_message.argtypes = [ctypes.c_void_p]
    error_message.restype = ctypes.c_char_p
    destroy_error = extension.Destroy_BskRustError
    destroy_error.argtypes = [ctypes.c_void_p]
    destroy_error.restype = None

    kind = error_kind(error)
    message = error_message(error).decode("utf-8")
    destroy_error(error)
    return kind, message


def _assert_no_abi_error(extension, error):
    """Fail with the owned Rust diagnostic when an ABI call returns an error."""
    if error is None:
        return
    kind, message = _consume_abi_error(extension, error)
    pytest.fail(f"Rust ABI error kind {kind}: {message}")


def _create_rust_instance(extension):
    """Create an opaque Rust instance through the guarded ABI."""
    create_instance = extension.Create_rustModuleTemplate
    create_instance.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
    create_instance.restype = ctypes.c_void_p
    handle = ctypes.c_void_p()
    _assert_no_abi_error(extension, create_instance(ctypes.byref(handle)))
    assert handle.value is not None
    return handle


def _destroy_rust_instance(extension, handle):
    """Destroy an opaque Rust instance and verify its guarded ABI result."""
    destroy_instance = extension.Destroy_rustModuleTemplate
    destroy_instance.argtypes = [ctypes.c_void_p]
    destroy_instance.restype = ctypes.c_void_p
    _assert_no_abi_error(extension, destroy_instance(handle))


def test_rust_module_template_python_api():
    """Freeze the module-first Python API retained by the opaque-handle design."""
    expected_fields = {
        "dummy",
        "increment",
        "dataInMsg",
        "dataOutMsg",
        "panicOnUpdate",
    }
    expected_wrapper_methods = {"SelfInit", "Reset", "UpdateState"}
    expected_framework_fields = {
        "ModelTag",
        "moduleID",
        "CallCounts",
        "RNGSeed",
        "bskLogger",
    }

    assert expected_fields.issubset(rustModuleTemplate.RustModuleTemplateConfig.__dict__)
    assert expected_fields.issubset(rustModuleTemplate.rustModuleTemplate.__dict__)
    assert expected_wrapper_methods.issubset(rustModuleTemplate.rustModuleTemplate.__dict__)

    module = rustModuleTemplate.rustModuleTemplate()
    assert module.thisown is True
    assert not hasattr(module, "getConfig")
    for field in expected_framework_fields:
        assert hasattr(module, field)
    assert not hasattr(module, "runtime")
    for framework_field in ("runtime", "bskLogger"):
        assert not hasattr(rustModuleTemplate.RustModuleTemplateConfig, framework_field)

    assert module.dummy == 0.0
    assert module.increment == 1.0
    module.dummy = 12.5  # [-]
    assert module.dummy == 12.5
    assert hasattr(module.dataInMsg, "subscribeTo")
    assert hasattr(module.dataOutMsg, "recorder")
    for internal_name in ("state", "update_history", "last_event", "mode"):
        assert not hasattr(module, internal_name)
        assert not hasattr(rustModuleTemplate.RustModuleTemplateConfig, internal_name)
    assert not hasattr(rustModuleTemplate, "RustModuleTemplateState")
    assert not hasattr(rustModuleTemplate, "RustModuleTemplateConfigHandle")

    with pytest.raises(AttributeError, match="No constructor defined"):
        rustModuleTemplate.RustModuleTemplateConfig()
    assert not hasattr(rustModuleTemplate.RustModuleTemplateConfig, "createWrapper")

    for symbol in (
        "Create_rustModuleTemplate",
        "Config_rustModuleTemplate",
        "Destroy_rustModuleTemplate",
        "SelfInit_rustModuleTemplate",
        "Reset_rustModuleTemplate",
        "Update_rustModuleTemplate",
    ):
        assert not hasattr(rustModuleTemplate, symbol)


def test_rust_module_template_abi_layout():
    """Check the context ABI and framework-free 64-bit config layout."""
    if ctypes.sizeof(ctypes.c_void_p) != 8:
        pytest.skip("The current Rust module ABI baseline covers 64-bit platforms.")

    assert ctypes.sizeof(_ExpectedBskModuleRuntime) == 32
    assert ctypes.alignment(_ExpectedBskModuleRuntime) == 8
    assert _ExpectedBskModuleRuntime.module_id.offset == 0
    assert _ExpectedBskModuleRuntime.model_tag.offset == 8
    assert _ExpectedBskModuleRuntime.call_counts.offset == 16
    assert _ExpectedBskModuleRuntime.rng_seed.offset == 24

    assert ctypes.sizeof(_ExpectedBskModuleContext) == 40
    assert ctypes.alignment(_ExpectedBskModuleContext) == 8
    assert _ExpectedBskModuleContext.runtime.offset == 0
    assert _ExpectedBskModuleContext.bsk_logger.offset == 32

    assert ctypes.sizeof(_ExpectedCModuleTemplatePort) == 72
    assert ctypes.alignment(_ExpectedCModuleTemplatePort) == 8
    assert _ExpectedCModuleTemplatePort.header.offset == 0
    assert _ExpectedCModuleTemplatePort.payload.offset == 32
    assert _ExpectedCModuleTemplatePort.payload_pointer.offset == 56
    assert _ExpectedCModuleTemplatePort.header_pointer.offset == 64

    assert ctypes.sizeof(_ExpectedRustModuleTemplateConfig) == 168
    assert ctypes.alignment(_ExpectedRustModuleTemplateConfig) == 8
    assert _ExpectedRustModuleTemplateConfig.dummy.offset == 0
    assert _ExpectedRustModuleTemplateConfig.increment.offset == 8
    assert _ExpectedRustModuleTemplateConfig.data_in_msg.offset == 16
    assert _ExpectedRustModuleTemplateConfig.data_out_msg.offset == 88
    assert _ExpectedRustModuleTemplateConfig.panic_on_update.offset == 160

    extension = ctypes.CDLL(rustModuleTemplate._rustModuleTemplate.__file__)
    get_config = extension.Config_rustModuleTemplate
    get_config.argtypes = [ctypes.c_void_p]
    get_config.restype = ctypes.c_void_p

    handle = _create_rust_instance(extension)
    try:
        config_address = get_config(handle)
        assert config_address is not None
        raw_config = _ExpectedRustModuleTemplateConfig.from_address(config_address)
        assert raw_config.dummy == 0.0
        assert raw_config.increment == 1.0
        assert raw_config.panic_on_update is False
        raw_config.dummy = -3.5  # [-]
        assert raw_config.dummy == -3.5
    finally:
        _destroy_rust_instance(extension, handle)


def test_rust_module_template_rust_owned_instance_lifecycle():
    """Exercise opaque instance ownership and handle-based lifecycle calls."""
    extension = ctypes.CDLL(rustModuleTemplate._rustModuleTemplate.__file__)
    expected_symbols = (
        "Create_rustModuleTemplate",
        "Config_rustModuleTemplate",
        "Destroy_rustModuleTemplate",
        "SelfInit_rustModuleTemplate",
        "Reset_rustModuleTemplate",
        "Update_rustModuleTemplate",
    )
    for symbol in expected_symbols:
        assert getattr(extension, symbol) is not None
    for obsolete_symbol in (
        "Init_rustModuleTemplate",
        "Drop_rustModuleTemplate",
        "New_rustModuleTemplate",
        "Delete_rustModuleTemplate",
    ):
        with pytest.raises(AttributeError):
            getattr(extension, obsolete_symbol)

    get_config = extension.Config_rustModuleTemplate
    get_config.argtypes = [ctypes.c_void_p]
    get_config.restype = ctypes.c_void_p
    self_init = extension.SelfInit_rustModuleTemplate
    self_init.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(_ExpectedBskModuleContext),
    ]
    self_init.restype = ctypes.c_void_p
    reset = extension.Reset_rustModuleTemplate
    reset.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint64,
        ctypes.POINTER(_ExpectedBskModuleContext),
    ]
    reset.restype = ctypes.c_void_p
    update = extension.Update_rustModuleTemplate
    update.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint64,
        ctypes.POINTER(_ExpectedBskModuleContext),
    ]
    update.restype = ctypes.c_void_p

    handle = _create_rust_instance(extension)
    try:
        config_address = get_config(handle)
        assert config_address is not None
        raw_config = _ExpectedRustModuleTemplateConfig.from_address(config_address)
        assert raw_config.dummy == 0.0
        assert raw_config.increment == 1.0
        assert raw_config.panic_on_update is False
        raw_config.dummy = 42.0  # [-]
        context = _ExpectedBskModuleContext(
            runtime=_ExpectedBskModuleRuntime(
                module_id=7,
                model_tag=None,
                call_counts=0,
                rng_seed=1234,
            ),
            bsk_logger=None,
        )
        invalid_error = reset(None, 10, ctypes.byref(context))  # [ns]
        kind, message = _consume_abi_error(extension, invalid_error)
        assert kind == _BSK_RUST_ERROR_INVALID_ARGUMENT
        assert "module handle must not be null" in message

        _assert_no_abi_error(extension, self_init(handle, ctypes.byref(context)))
        raw_config.increment = 0.0  # [-]
        expected_error = reset(handle, 9, ctypes.byref(context))  # [ns]
        kind, message = _consume_abi_error(extension, expected_error)
        assert kind == _BSK_RUST_ERROR_EXPECTED
        assert (
            message
            == "rustModuleTemplate.increment must be finite and strictly positive"
        )
        assert raw_config.data_out_msg.header.is_written == 0

        raw_config.increment = 1.0  # [-]
        _assert_no_abi_error(
            extension,
            reset(handle, 10, ctypes.byref(context)),  # [ns]
        )
        assert raw_config.dummy == 0.0
        assert raw_config.data_out_msg.header.module_id == 7
        assert raw_config.data_out_msg.header.time_written == 10

        raw_config.panic_on_update = True
        panic_error = update(handle, 11, ctypes.byref(context))  # [ns]
        kind, message = _consume_abi_error(extension, panic_error)
        assert kind == _BSK_RUST_ERROR_PANIC
        assert (
            message
            == "Rust panic in Update_rustModuleTemplate: "
            "deliberate rustModuleTemplate update panic"
        )

        raw_config.panic_on_update = False
        poisoned_error = reset(handle, 12, ctypes.byref(context))  # [ns]
        kind, message = _consume_abi_error(extension, poisoned_error)
        assert kind == _BSK_RUST_ERROR_PANIC
        assert (
            message
            == "Rust module instance cannot execute Reset_rustModuleTemplate "
            "after a previous panic in Update_rustModuleTemplate"
        )
        assert raw_config.dummy == 0.0
    finally:
        _destroy_rust_instance(extension, handle)


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
    assert module.bskLogger is not None

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


def test_rust_module_template_expected_error():
    """Translate a Rust ``BskError`` into a Python ``BasiliskError``."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    task_time_step = macros.sec2nano(0.5)  # [ns]
    process.addTask(simulation.CreateNewTask("testTask", task_time_step))

    module = rustModuleTemplate.rustModuleTemplate()
    module.increment = 0.0  # [-]
    simulation.AddModelToTask("testTask", module)

    with pytest.raises(
        BasiliskError,
        match="rustModuleTemplate.increment must be finite and strictly positive",
    ):
        simulation.InitializeSimulation()


def test_rust_module_template_panic_is_contained(capfd):
    """Translate a Rust panic once without emitting the default hook report."""
    module = rustModuleTemplate.rustModuleTemplate()
    module.panicOnUpdate = True

    with pytest.raises(
        BasiliskError,
        match=(
            "Rust panic in Update_rustModuleTemplate: "
            "deliberate rustModuleTemplate update panic"
        ),
    ):
        module.UpdateState(0)  # [ns]

    module.panicOnUpdate = False
    with pytest.raises(
        BasiliskError,
        match=(
            "Rust module instance cannot execute Reset_rustModuleTemplate "
            "after a previous panic in Update_rustModuleTemplate"
        ),
    ):
        module.Reset(1)  # [ns]

    captured = capfd.readouterr()
    assert "panicked at" not in captured.err
