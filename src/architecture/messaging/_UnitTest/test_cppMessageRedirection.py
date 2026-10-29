# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Exercise the documented Python assignment used for C++ output gateways."""

import gc
import weakref

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cppModuleTemplate


@pytest.mark.parametrize("subscribe_to_output", [False, True])
def test_cpp_output_redirection(subscribe_to_output):
    """Two native producers publish through one gateway to C/C++ readers and recorders."""
    gateway = messaging.CModuleTemplateMsg()
    gateway.write(messaging.CModuleTemplateMsgPayload())
    gateway_recorder = gateway.recorder()
    producers = [cppModuleTemplate.CppModuleTemplate() for _ in range(2)]
    for producer, counter in zip(producers, [10.0, 20.0]):  # [-]
        producer.setDummy(counter)
        producer.dataOutMsg = gateway
    # Assigning an output to itself must preserve its existing redirection.
    producers[0].dataOutMsg = producers[0].dataOutMsg
    output = producers[0].dataOutMsg
    output_recorder = output.recorder()
    source = output if subscribe_to_output else gateway
    cpp_reader = messaging.CModuleTemplateMsgReader()
    c_reader = messaging.CModuleTemplateMsg_C()
    cpp_reader.subscribeTo(source)
    c_reader.subscribeTo(source)

    timestamps = [0, 10, 20]  # [ns]
    expected_values = [11.0, 21.0, 12.0]  # [-]
    for index, timestamp, expected in zip([0, 1, 0], timestamps, expected_values):
        producer = producers[index]
        producer.UpdateState(timestamp)
        for payload in (gateway.read(), output.read(), cpp_reader(), c_reader.read()):
            np.testing.assert_array_equal(payload.dataVector, [expected, 0.0, 0.0])
        assert cpp_reader.timeWritten() == timestamp
        assert cpp_reader.moduleID() == producer.moduleID
        assert messaging.CModuleTemplateMsg_C_timeWritten(c_reader) == timestamp
        gateway_recorder.UpdateState(timestamp)
        output_recorder.UpdateState(timestamp)

    for recorder in (gateway_recorder, output_recorder):
        np.testing.assert_array_equal(recorder.dataVector[:, 0], expected_values)
        np.testing.assert_array_equal(recorder.timesWritten(), timestamps)
    assert output.getHeaderAddress() == gateway.getHeaderAddress()
    assert output.getPayloadAddress() == gateway.getPayloadAddress()
    assert int(output.GetPointers().payload) == gateway.getPayloadAddress()


def test_cpp_output_reredirection():
    """Replacing a gateway redirects future writes and updates borrowed pointer views."""
    producer = cppModuleTemplate.CppModuleTemplate()
    gateways = [messaging.CModuleTemplateMsg() for _ in range(2)]
    output = producer.dataOutMsg
    pointers = output.GetPointers()
    for index, gateway in enumerate(gateways):
        producer.dataOutMsg = gateway
        timestamp = index * 10  # [ns]
        producer.UpdateState(timestamp)
        np.testing.assert_array_equal(gateway.read().dataVector, [index + 1.0, 0.0, 0.0])
        assert int(pointers.header) == gateway.getHeaderAddress()
        assert int(pointers.payload) == gateway.getPayloadAddress()
    np.testing.assert_array_equal(gateways[0].read().dataVector, [1.0, 0.0, 0.0])


def test_cpp_output_retains_gateway():
    """A producer retains a temporary gateway and releases it on reassignment."""
    producer = cppModuleTemplate.CppModuleTemplate()
    gateway = messaging.CModuleTemplateMsg()
    reference = weakref.ref(gateway)
    producer.dataOutMsg = gateway
    producer.dataOutMsg = producer.dataOutMsg
    del gateway
    gc.collect()
    assert reference() is not None
    timestamp = 10  # [ns]
    producer.UpdateState(timestamp)
    np.testing.assert_array_equal(reference().read().dataVector, [1.0, 0.0, 0.0])

    producer.dataOutMsg = messaging.CModuleTemplateMsg()
    gc.collect()
    assert reference() is None


@pytest.mark.parametrize("consumer_kind", ["subscriber", "cpp_reader", "c_reader",
                                          "recorder", "direct_recorder", "payload"])
@pytest.mark.parametrize("replace_gateway", [False, True])
def test_cpp_output_consumers_retain_original_gateway(consumer_kind, replace_gateway):
    """Consumers retain their original storage after producer deletion or redirection."""
    producer = cppModuleTemplate.CppModuleTemplate()
    gateway = messaging.CModuleTemplateMsg()
    reference = weakref.ref(gateway)
    producer.dataOutMsg = gateway
    timestamp = 10  # [ns]
    producer.UpdateState(timestamp)
    if consumer_kind == "subscriber":
        consumer = producer.dataOutMsg.addSubscriber()
    elif consumer_kind == "cpp_reader":
        consumer = messaging.CModuleTemplateMsgReader()
        consumer.subscribeTo(producer.dataOutMsg)
    elif consumer_kind == "c_reader":
        consumer = messaging.CModuleTemplateMsg_C()
        consumer.subscribeTo(producer.dataOutMsg)
    elif consumer_kind == "recorder":
        consumer = producer.dataOutMsg.recorder()
    elif consumer_kind == "direct_recorder":
        consumer = messaging.CModuleTemplateMsgRecorder(producer.dataOutMsg)
    else:
        consumer = producer.dataOutMsg.read()
    del gateway
    if replace_gateway:
        producer.dataOutMsg = messaging.CModuleTemplateMsg()
        producer.UpdateState(timestamp)
    else:
        del producer
    gc.collect()
    # Assert ownership before accessing the payload, so failures cannot read freed memory.
    assert reference() is not None

    if consumer_kind in ("recorder", "direct_recorder"):
        consumer.UpdateState(timestamp)
        np.testing.assert_array_equal(consumer.dataVector, [[1.0, 0.0, 0.0]])
        np.testing.assert_array_equal(consumer.timesWritten(), [timestamp])
    elif consumer_kind == "c_reader":
        np.testing.assert_array_equal(consumer.read().dataVector, [1.0, 0.0, 0.0])
        assert messaging.CModuleTemplateMsg_C_timeWritten(consumer) == timestamp
    elif consumer_kind == "payload":
        np.testing.assert_array_equal(consumer.dataVector, [1.0, 0.0, 0.0])
    else:
        np.testing.assert_array_equal(consumer().dataVector, [1.0, 0.0, 0.0])
        assert consumer.timeWritten() == timestamp
    del consumer
    gc.collect()
    assert reference() is None


def test_cpp_output_redirection_chain_retains_gateway():
    """Assigning a redirected output retains the terminal gateway independently."""
    first = cppModuleTemplate.CppModuleTemplate()
    second = cppModuleTemplate.CppModuleTemplate()
    gateway = messaging.CModuleTemplateMsg()
    reference = weakref.ref(gateway)
    first.dataOutMsg = gateway
    second.dataOutMsg = first.dataOutMsg
    del first, gateway
    gc.collect()
    assert reference() is not None
    timestamp = 10  # [ns]
    second.UpdateState(timestamp)
    np.testing.assert_array_equal(reference().read().dataVector, [1.0, 0.0, 0.0])
    del second
    gc.collect()
    assert reference() is None
