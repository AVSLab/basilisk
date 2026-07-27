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

"""Verify Python lifetime ownership for C-message ports embedded in Rust modules."""

import gc
import warnings
import weakref

import pytest

from Basilisk import hasBuildFeature
from Basilisk.architecture import messaging

rustModulesEnabled = hasBuildFeature("rustModules")
pytestmark = pytest.mark.skipif(
    not rustModulesEnabled,
    reason="Requires Basilisk built with --rustModules True",
)
if rustModulesEnabled:
    from Basilisk.moduleTemplates import rustModuleTemplate


def test_rust_port_registration_does_not_read_configuration_properties():
    """Register only generated message ports without invoking deprecated fields."""
    with warnings.catch_warnings(record=True) as caught_warnings:
        warnings.simplefilter("always")
        rustModuleTemplate.rustModuleTemplate()

    assert not any(
        "legacyDummy" in str(warning.message) for warning in caught_warnings
    )


def _input_port(module, array_index):
    """Return the template's scalar input or one fixed-array input port."""
    if array_index is None:
        return module.dataInMsg
    return module.dataInMsgs[array_index]


def _output_port(module, array_index):
    """Return the template's scalar output or one fixed-array output port."""
    if array_index is None:
        return module.dataOutMsg
    return module.dataOutMsgs[array_index]


def _subscribe_to_temporary_source(module, array_index, values):
    """Subscribe an embedded reader to a source whose local reference expires."""
    source = messaging.CModuleTemplateMsg().write(
        messaging.CModuleTemplateMsgPayload(dataVector=values)
    )
    source_reference = weakref.ref(source)
    _input_port(module, array_index).subscribeTo(source)
    return source_reference


@pytest.mark.parametrize("array_index", [None, 0], ids=["scalar", "fixed-array"])
def test_rust_reader_keeps_temporary_source_alive(array_index):
    """Retain a temporary source while a scalar or array reader is subscribed."""
    values = [1.0, 2.0, 3.0]  # [-]
    module = rustModuleTemplate.rustModuleTemplate()
    source_reference = _subscribe_to_temporary_source(module, array_index, values)

    gc.collect()

    assert source_reference() is not None
    assert list(_input_port(module, array_index).read().dataVector) == values

    _input_port(module, array_index).unsubscribe()
    gc.collect()

    assert source_reference() is None


@pytest.mark.parametrize("array_index", [None, 1], ids=["scalar", "fixed-array"])
def test_rust_reader_keeps_embedded_source_owner_alive(array_index):
    """Retain a Rust producer rather than its transient output-port proxy."""
    values = [4.0, 5.0, 6.0]  # [-]
    consumer = rustModuleTemplate.rustModuleTemplate()
    producer = rustModuleTemplate.rustModuleTemplate()
    source = _output_port(producer, array_index)
    source.write(messaging.CModuleTemplateMsgPayload(dataVector=values))
    producer_reference = weakref.ref(producer)
    source_reference = weakref.ref(source)

    _input_port(consumer, array_index).subscribeTo(source)
    del producer, source
    gc.collect()

    assert producer_reference() is not None
    assert source_reference() is None
    assert list(_input_port(consumer, array_index).read().dataVector) == values

    _input_port(consumer, array_index).unsubscribe()
    gc.collect()

    assert producer_reference() is None


@pytest.mark.parametrize("array_index", [None, 0], ids=["scalar", "fixed-array"])
def test_rust_output_recorder_keeps_module_owner_alive(array_index):
    """Retain a Rust output's owning module until its recorder is collected."""
    values = [7.0, 8.0, 9.0]  # [-]
    module = rustModuleTemplate.rustModuleTemplate()
    source = _output_port(module, array_index)
    source.write(messaging.CModuleTemplateMsgPayload(dataVector=values))
    module_reference = weakref.ref(module)
    recorder = source.recorder()

    del module, source
    gc.collect()

    assert module_reference() is not None
    recorder.UpdateState(0)  # [ns]
    assert list(recorder.dataVector[0]) == values

    del recorder
    gc.collect()

    assert module_reference() is None
