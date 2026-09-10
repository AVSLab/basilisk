#
#  ISC License
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

"""Payload views must retain their storage independently of the original reader."""

import gc
import weakref

import pytest

from Basilisk.architecture import bskLogging, messaging
from Basilisk.moduleTemplates import cppModuleTemplate


def test_unconnected_reader_preserves_error():
    """An unconnected read still reports a Basilisk error."""
    reader = messaging.EpochMsgReader()
    with pytest.raises(bskLogging.BasiliskError, match="un-connected message"):
        reader()


def test_raw_address_reader_with_explicit_source_owner():
    """An address-based reader still works while the caller owns its source."""
    sourceMsg = messaging.EpochMsg().write(messaging.EpochMsgPayload(year=2023))  # [year]
    reader = messaging.EpochMsgReader()
    reader.subscribeToAddr(int(sourceMsg.this))
    payload = reader()
    reader.unsubscribe()
    assert payload.year == 2023  # [year]
    sourceMsg.write(messaging.EpochMsgPayload(year=2026))  # [year]
    assert payload.year == 2026  # [year]


@pytest.mark.parametrize(
    "message_type",
    [messaging.EpochMsg, messaging.CModuleTemplateMsg, messaging.DataStorageStatusMsg],
)
def test_cached_message_payload_cycle_is_collected(message_type):
    """A message caching its own payload must not prevent cycle collection."""
    sourceMsg = message_type()
    sourceMsg.cached_payload = sourceMsg.read()
    source_reference = weakref.ref(sourceMsg)
    payload_reference = weakref.ref(sourceMsg.cached_payload)
    del sourceMsg

    gc.collect()
    try:
        assert source_reference() is None
        assert payload_reference() is None
    finally:
        # Break the cycle if the regression returns, so a failed test does not leak.
        if source_reference() is not None:
            del source_reference().cached_payload


@pytest.mark.parametrize(
    "source_type", [messaging.CModuleTemplateMsg, messaging.CModuleTemplateMsg_C],
    ids=["cpp_source", "c_source"],
)
@pytest.mark.parametrize("release_action", ["unsubscribe", "resubscribe", "destroy"])
def test_cached_reader_payload_cycle_is_collected(source_type, release_action):
    """A cached reader payload becomes collectible after the subscription ends."""
    sourceMsg = source_type()
    module = cppModuleTemplate.CppModuleTemplate()
    module.dataInMsg.subscribeTo(sourceMsg)
    sourceMsg.cached_payload = module.dataInMsg()
    source_reference = weakref.ref(sourceMsg)
    payload_reference = weakref.ref(sourceMsg.cached_payload)
    del sourceMsg

    if release_action == "unsubscribe":
        module.dataInMsg.unsubscribe()
    elif release_action == "resubscribe":
        module.dataInMsg.subscribeTo(source_type())
    else:
        del module

    gc.collect()
    try:
        assert source_reference() is None
        assert payload_reference() is None
    finally:
        if source_reference() is not None:
            del source_reference().cached_payload


@pytest.mark.parametrize(
    "message_type, payload_type, field, value_type",
    [
        (messaging.CModuleTemplateMsg, messaging.CModuleTemplateMsgPayload,
         "dataVector", list),
        (messaging.DataStorageStatusMsg, messaging.DataStorageStatusMsgPayload,
         "storedData", messaging.DoubleVector),
    ],
    ids=["fixed_array", "cpp_vector"],
)
def test_message_read_retains_live_payload(
    message_type, payload_type, field, value_type
):
    """Reads preserve a live view and release the source with the last payload."""
    initial_values = [1.0, 2.0, 3.0]  # [-] template data; [bit] stored data
    updated_values = [4.0, 5.0, 6.0]  # [-] template data; [bit] stored data
    sourceMsg = message_type().write(
        payload_type(**{field: value_type(initial_values)})
    )
    reference = weakref.ref(sourceMsg)
    first_payload = sourceMsg.read()
    second_payload = sourceMsg.read()
    del sourceMsg

    gc.collect()
    assert reference() is not None
    assert list(getattr(first_payload, field)) == initial_values
    reference().write(payload_type(**{field: value_type(updated_values)}))
    assert list(getattr(first_payload, field)) == updated_values

    del first_payload
    gc.collect()
    assert reference() is not None
    assert list(getattr(second_payload, field)) == updated_values

    del second_payload
    gc.collect()
    assert reference() is None


@pytest.mark.parametrize(
    "source_type", [messaging.CModuleTemplateMsg, messaging.CModuleTemplateMsg_C],
    ids=["cpp_source", "c_source"],
)
@pytest.mark.parametrize(
    "release_action", ["unsubscribe", "resubscribe", "destroy", "payload"]
)
def test_reader_and_payload_retain_source_independently(source_type, release_action):
    """The source survives until both the reader and its payload release it."""
    module = cppModuleTemplate.CppModuleTemplate()
    initial_values = [1.0, 2.0, 3.0]  # [-] template data
    sourceMsg = source_type().write(
        messaging.CModuleTemplateMsgPayload(dataVector=initial_values)
    )
    reference = weakref.ref(sourceMsg)
    module.dataInMsg.subscribeTo(sourceMsg)
    payload = module.dataInMsg()
    del sourceMsg

    if release_action == "payload":
        del payload
        gc.collect()
        assert reference() is not None
        assert list(module.dataInMsg().dataVector) == initial_values
        module.dataInMsg.unsubscribe()
        gc.collect()
        assert reference() is None
        return

    if release_action == "unsubscribe":
        module.dataInMsg.unsubscribe()
    elif release_action == "resubscribe":
        module.dataInMsg.subscribeTo(messaging.CModuleTemplateMsg().write(
            messaging.CModuleTemplateMsgPayload(dataVector=[4.0, 5.0, 6.0])  # [-]
        ))
    else:
        del module

    gc.collect()
    # Check ownership before accessing the payload, avoiding reads of freed memory.
    assert reference() is not None
    assert list(payload.dataVector) == initial_values

    del payload
    gc.collect()
    assert reference() is None
