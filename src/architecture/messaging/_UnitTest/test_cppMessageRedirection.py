# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Exercise the documented Python assignment used for C++ output gateways."""

import gc
import weakref

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.simulation import dualHingedRigidBodyStateEffector, spacecraft, twoHingeDamper


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
@pytest.mark.parametrize("embedded_gateway", [False, True])
def test_cpp_output_consumers_retain_original_gateway(consumer_kind, replace_gateway, embedded_gateway):
    """Consumers retain their original storage after producer deletion or redirection."""
    producer = cppModuleTemplate.CppModuleTemplate()
    if embedded_gateway:
        source = cppModuleTemplate.CppModuleTemplate()
        gateway = source.dataOutMsg
        reference = weakref.ref(source)
    else:
        source = gateway = messaging.CModuleTemplateMsg()
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
    del gateway, source
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


@pytest.mark.parametrize("replace_gateway", [False, True])
def test_cpp_output_redirection_retains_source_module(replace_gateway):
    """An embedded gateway keeps its module alive until the destination releases it."""
    source = cppModuleTemplate.CppModuleTemplate()
    destination = cppModuleTemplate.CppModuleTemplate()
    reference = weakref.ref(source)
    destination.dataOutMsg = source.dataOutMsg
    del source
    gc.collect()
    # Check lifetime before writing, so a regression cannot access freed storage.
    assert reference() is not None

    unrelated = cppModuleTemplate.CppModuleTemplate()
    unrelated.setDummy(100.0)  # [-]
    timestamp = 10  # [ns]
    unrelated.UpdateState(timestamp)
    destination.UpdateState(timestamp)
    np.testing.assert_array_equal(reference().dataOutMsg.read().dataVector, [1.0, 0.0, 0.0])
    np.testing.assert_array_equal(unrelated.dataOutMsg.read().dataVector, [101.0, 0.0, 0.0])

    if replace_gateway:
        destination.dataOutMsg = messaging.CModuleTemplateMsg()
    else:
        del destination
    gc.collect()
    assert reference() is None


def test_cpp_output_redirection_keeps_original_embedded_storage():
    """Rerouting the source output does not change the destination's storage owner."""
    source = cppModuleTemplate.CppModuleTemplate()
    destination = cppModuleTemplate.CppModuleTemplate()
    reference = weakref.ref(source)
    destination.dataOutMsg = source.dataOutMsg
    gateway = messaging.CModuleTemplateMsg()
    source.dataOutMsg = gateway
    reader = destination.dataOutMsg.addSubscriber()
    del source
    gc.collect()
    assert reference() is not None

    timestamp = 10  # [ns]
    destination.UpdateState(timestamp)
    np.testing.assert_array_equal(reader().dataVector, [1.0, 0.0, 0.0])
    np.testing.assert_array_equal(gateway.read().dataVector, [0.0, 0.0, 0.0])
    del destination
    gc.collect()
    assert reference() is not None
    del reader
    gc.collect()
    assert reference() is None


def test_cpp_output_redirection_rejects_deleted_source_module():
    """A saved non-owning output proxy cannot redirect into a deleted module."""
    source = cppModuleTemplate.CppModuleTemplate()
    output = source.dataOutMsg
    reference = weakref.ref(source)
    del source
    gc.collect()
    assert reference() is None
    destination = cppModuleTemplate.CppModuleTemplate()
    with pytest.raises(ReferenceError, match="module owning this message has been deleted"):
        destination.dataOutMsg = output


@pytest.mark.parametrize("through_another_module", [False, True])
def test_cpp_output_redirection_does_not_retain_its_own_module(through_another_module):
    """Redirecting co-owned outputs must not make their module retain itself."""
    owner = twoHingeDamper.TwoHingeDamper()
    reference = weakref.ref(owner)
    if through_another_module:
        intermediate = twoHingeDamper.TwoHingeDamper()
        intermediate.phiTorqueOutMsg = owner.thetaTorqueOutMsg
        owner.phiTorqueOutMsg = intermediate.phiTorqueOutMsg
        del intermediate
    else:
        owner.phiTorqueOutMsg = owner.thetaTorqueOutMsg
    payload = messaging.SingleActuatorMsgPayload()
    payload.input = 3.0  # [N*m]
    owner.phiTorqueOutMsg.write(payload)
    assert owner.thetaTorqueOutMsg.read().input == payload.input
    del owner
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


@pytest.mark.parametrize("accessor", ["index", "front", "back", "list_slice", "iter", "list",
                                      "iterator_value", "iterator_copy", "iterator_advance",
                                      "reverse_iterator", "iterator_previous", "slice_iter"])
def test_cpp_output_vector_redirection(accessor):
    """Temporary vector views resolve to the module that owns their messages."""
    source = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    destination = spacecraft.Spacecraft()
    reference = weakref.ref(source)
    accessors = {
        "index": lambda vector: vector[0],
        "front": lambda vector: vector.front(),
        "back": lambda vector: vector.back(),
        "list_slice": lambda vector: list(vector)[:][0],
        "iter": lambda vector: next(iter(vector)),
        "list": lambda vector: list(vector)[0],
        "iterator_value": lambda vector: vector.iterator().value(),
        "iterator_copy": lambda vector: next(vector.iterator().copy()),
        "iterator_advance": lambda vector: (vector.begin() + 1).value(),
        "reverse_iterator": lambda vector: vector.rbegin().value(),
        "iterator_previous": lambda vector: vector.end().previous(),
        "slice_iter": lambda vector: next(iter(list(vector)[:])),
    }
    destination.scStateOutMsg = accessors[accessor](source.dualHingedRigidBodyConfigLogOutMsgs)
    del source
    gc.collect()
    assert reference() is not None
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = [1.0, 2.0, 3.0]  # [m]
    destination.scStateOutMsg.write(payload)
    index = 1 if accessor in ("back", "iterator_advance", "reverse_iterator", "iterator_previous") else 0
    np.testing.assert_array_equal(
        reference().dualHingedRigidBodyConfigLogOutMsgs[index].read().r_BN_N,
        payload.r_BN_N,
    )
    del destination
    gc.collect()
    assert reference() is None


@pytest.mark.parametrize("iterate", [False, True])
def test_cpp_output_vector_rejects_deleted_module(iterate):
    """Keeping a vector view does not hide deletion of the owning module."""
    source = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    vector = source.dualHingedRigidBodyConfigLogOutMsgs
    output = next(iter(vector)) if iterate else vector[0]
    assert getattr(output, "_bskMessageOwner", lambda: None)() is source
    reference = weakref.ref(source)
    del source
    gc.collect()
    assert reference() is None
    destination = spacecraft.Spacecraft()
    with pytest.raises(ReferenceError, match="module owning this message has been deleted"):
        destination.scStateOutMsg = output


def test_cpp_output_iterator_rejects_deleted_module():
    """An iterator checks its weak module owner before accessing native storage."""
    source = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    iterator = iter(source.dualHingedRigidBodyConfigLogOutMsgs)
    # Check tagging before deleting storage so regressions cannot dereference it.
    assert getattr(iterator, "_bskMessageOwner", lambda: None)() is source
    reference = weakref.ref(source)
    del source
    gc.collect()
    assert reference() is None
    with pytest.raises(ReferenceError, match="module owning this message has been deleted"):
        next(iterator)


def test_cpp_pointer_iterator_preserves_other_vector_semantics():
    """The shared iterator still returns numeric values and owning reader copies."""
    source = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    next(iter(source.dualHingedRigidBodyConfigLogOutMsgs))
    numbers = messaging.DoubleVector([1.0, 2.0])  # [-]
    assert list(numbers) == [1.0, 2.0]
    assert (numbers.begin() + 1).value() == 2.0
    message = messaging.NavTransMsg()
    readers = messaging.NavTransMsgInMsgsVector([message.addSubscriber()])
    reader = next(iter(readers))
    assert reader.thisown
    assert getattr(reader, "_bskMessageOwner", None) is None
    reader.unsubscribe()
    assert readers[0].isSubscribedTo(message)


def test_cpp_feedback_using_standalone_gateway_is_supported():
    """A standalone gateway permits feedback without retaining the source module."""
    producer = cppModuleTemplate.CppModuleTemplate()
    reference = weakref.ref(producer)
    gateway = messaging.CModuleTemplateMsg()
    producer.dataOutMsg = gateway
    producer.dataInMsg.subscribeTo(gateway)
    timestamp = 10  # [ns]
    producer.UpdateState(timestamp)
    np.testing.assert_array_equal(producer.dataInMsg().dataVector, [1.0, 0.0, 0.0])
    del producer
    gc.collect()
    assert reference() is None


@pytest.mark.parametrize("iterate", [False, True])
@pytest.mark.parametrize("delete_source", [False, True])
@pytest.mark.parametrize("copy_kind", ["constructor", "slice"])
def test_cpp_unowned_pointer_vector_redirection_is_rejected(iterate, delete_source, copy_kind):
    """A raw pointer-vector copy cannot establish ownership of its pointees."""
    source = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    vector = source.dualHingedRigidBodyConfigLogOutMsgs
    copied = messaging.SCStatesMsgOutMsgsPtrVector(vector) if copy_kind == "constructor" else vector[:]
    output = next(iter(copied)) if iterate else copied[0]
    destination = spacecraft.Spacecraft()
    original_address = destination.scStateOutMsg.getPayloadAddress()
    with pytest.raises(ReferenceError, match="storage owner is unknown"):
        destination.scStateOutMsg = output
    if delete_source:
        del source
        gc.collect()
        with pytest.raises(ReferenceError, match="storage owner is unknown"):
            destination.scStateOutMsg = output
    assert destination.scStateOutMsg.getPayloadAddress() == original_address


@pytest.mark.parametrize("iterate", [False, True])
def test_cpp_mutated_pointer_vector_slice_has_no_owner(iterate):
    """Replacing a raw slice element cannot attribute its storage to the original module."""
    source = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    copied = source.dualHingedRigidBodyConfigLogOutMsgs[:]
    foreign = messaging.SCStatesMsg()
    reference = weakref.ref(foreign)
    copied[0] = foreign
    output = next(iter(copied)) if iterate else copied[0]
    destination = spacecraft.Spacecraft()
    original_address = destination.scStateOutMsg.getPayloadAddress()
    # Check rejection while storage is live so older builds fail safely.
    with pytest.raises(ReferenceError, match="storage owner is unknown"):
        destination.scStateOutMsg = output
    del foreign
    gc.collect()
    assert reference() is None
    with pytest.raises(ReferenceError, match="storage owner is unknown"):
        destination.scStateOutMsg = output
    assert destination.scStateOutMsg.getPayloadAddress() == original_address
