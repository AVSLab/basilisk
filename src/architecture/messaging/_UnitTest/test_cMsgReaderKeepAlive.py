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

#
#   Unit Test Script
#   Module Name:        messaging (Msg_C reader keep-alive)
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      June 25, 2026
#

"""
Regression test for issue #1433 (follow-up to #676 / #1432).

A ``Msg_C`` reader (e.g. a C module's ``dataInMsg``) that subscribes to a Python
source stores raw pointers into that source's memory. If the only Python reference
to the source then goes out of scope, the source must *not* be garbage-collected
while the subscription is live, otherwise the reader points into freed memory. The
keep-alive added for #1433 retains the source owner and releases it again on
``unsubscribe()``, on re-subscribe, when a stand-alone subscriber proxy is
collected, and when the C module that embeds the subscriber is collected.

These tests assert the lifetime contract directly with ``weakref`` (deterministic,
no reliance on memory being clobbered) and additionally check that a module reads
the correct, live data end-to-end after a garbage-collection pass.
"""

import gc
import weakref

import numpy as np
from Basilisk.architecture import bskLogging, messaging
from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.utilities import SimulationBaseClass, macros


def _cppSourceInScope(reader, vec):
    """Subscribe `reader` (a Msg_C) to a C++ Message source created in this scope,
    then let the only Python reference to the source fall out of scope on return.
    Returns a weakref to the source so the caller can observe its lifetime."""
    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = vec
    src = messaging.CModuleTemplateMsg().write(payload)
    reader.subscribeTo(src)
    return weakref.ref(src)


def _cMsgSourceInScope(reader, vec):
    """Same as `_cppSourceInScope` but the source is a stand-alone Msg_C."""
    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = vec
    src = messaging.CModuleTemplateMsg_C()
    src.write(payload)
    reader.subscribeTo(src)
    return weakref.ref(src)


def test_moduleEmbeddedCppSourceSurvivesGC():
    """A C module's Msg_C reader keeps a C++ Message source alive across GC, and
    reads its live data end-to-end."""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(1.0)))

    mod = cModuleTemplate.cModuleTemplate()
    mod.ModelTag = "cMod"
    scSim.AddModelToTask("t", mod)

    vec = [10.0, 20.0, 30.0]
    wr = _cppSourceInScope(mod.dataInMsg, vec)
    gc.collect()
    assert wr() is not None, "source was garbage-collected despite an active subscription (#1433)"

    rec = mod.dataInMsg.recorder()
    scSim.AddModelToTask("t", rec)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(3.0))
    scSim.ExecuteSimulation()

    recorded = np.array(rec.dataVector)
    np.testing.assert_allclose(
        recorded, [vec] * len(recorded), atol=1e-9,
        err_msg="C module read garbage from a GC'd subscribed source (#1433)",
    )


def test_moduleEmbeddedCMsgSourceSurvivesGC():
    """A C module's Msg_C reader keeps a stand-alone Msg_C source alive across GC."""
    mod = cModuleTemplate.cModuleTemplate()
    wr = _cMsgSourceInScope(mod.dataInMsg, [40.0, 50.0, 60.0])
    gc.collect()
    assert wr() is not None, "Msg_C source was garbage-collected despite a subscription (#1433)"


def test_moduleEmbeddedCMsgSourceOwnerSurvivesGC():
    """A C module's Msg_C reader keeps the owning module alive when subscribing
    to another module's embedded Msg_C source."""
    consumer = cModuleTemplate.cModuleTemplate()
    producer = cModuleTemplate.cModuleTemplate()

    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = [14.0, 15.0, 16.0]
    producer.dataOutMsg.write(payload)

    source = producer.dataOutMsg
    producer_ref = weakref.ref(producer)
    source_ref = weakref.ref(source)

    consumer.dataInMsg.subscribeTo(source)
    del producer, source
    gc.collect()

    assert producer_ref() is not None, "embedded Msg_C source owner was not retained (#1433)"
    assert source_ref() is None, "embedded Msg_C proxy was retained instead of its owner (#1433)"
    assert list(consumer.dataInMsg.read().dataVector) == [14.0, 15.0, 16.0]

    consumer.dataInMsg.unsubscribe()
    gc.collect()
    assert producer_ref() is None, "unsubscribe() did not release embedded Msg_C source owner (#1433)"


def test_unsubscribeReleasesKeepAlive():
    """unsubscribe() drops the keep-alive so the source can be collected."""
    mod = cModuleTemplate.cModuleTemplate()
    wr = _cppSourceInScope(mod.dataInMsg, [4.0, 5.0, 6.0])
    gc.collect()
    assert wr() is not None

    mod.dataInMsg.unsubscribe()
    gc.collect()
    assert wr() is None, "unsubscribe() did not release the keep-alive (#1433)"


def test_unsubscribeWeakrefCallbackCanResubscribe():
    """A source weakref callback can safely reconnect an unsubscribing reader."""
    reader = messaging.CModuleTemplateMsg_C()
    old_source = messaging.CModuleTemplateMsg().write(
        messaging.CModuleTemplateMsgPayload(dataVector=[1.0, 2.0, 3.0])
    )
    replacement = messaging.CModuleTemplateMsg().write(
        messaging.CModuleTemplateMsgPayload(dataVector=[4.0, 5.0, 6.0])
    )
    callback_states = []

    def reconnect(_):
        callback_states.append(reader.header.isLinked)
        reader.subscribeTo(replacement)
        callback_states.append(reader.header.isLinked)

    old_source_ref = weakref.ref(old_source, reconnect)
    replacement_ref = weakref.ref(replacement)
    reader.subscribeTo(old_source)
    del old_source
    gc.collect()

    reader.unsubscribe()

    assert old_source_ref() is None
    assert callback_states == [0, 1]
    assert reader.isSubscribedTo(replacement)
    assert list(reader.read().dataVector) == [4.0, 5.0, 6.0]

    del replacement
    gc.collect()
    assert replacement_ref() is not None

    reader.unsubscribe()
    gc.collect()
    assert replacement_ref() is None


def test_resubscribeReplacesKeepAlive():
    """Re-subscribing releases the previous source and retains the new one."""
    mod = cModuleTemplate.cModuleTemplate()
    wrA = _cppSourceInScope(mod.dataInMsg, [1.0, 1.0, 1.0])
    gc.collect()
    assert wrA() is not None

    wrB = _cppSourceInScope(mod.dataInMsg, [2.0, 2.0, 2.0])
    gc.collect()
    assert wrA() is None, "re-subscribe did not release the previous source (#1433)"
    assert wrB() is not None, "re-subscribe did not retain the new source (#1433)"


def test_resubscribeWeakrefCallbackCanUnsubscribe():
    """A source weakref callback can safely unsubscribe during re-subscription."""
    module = cModuleTemplate.cModuleTemplate()
    reader = module.dataInMsg
    old_source = messaging.CModuleTemplateMsg().write(
        messaging.CModuleTemplateMsgPayload(dataVector=[1.0, 1.0, 1.0])
    )
    new_source = messaging.CModuleTemplateMsg().write(
        messaging.CModuleTemplateMsgPayload(dataVector=[2.0, 2.0, 2.0])
    )
    callback_states = []

    def disconnect(_):
        callback_states.append(reader.header.isLinked)
        reader.unsubscribe()
        callback_states.append(reader.header.isLinked)

    old_source_ref = weakref.ref(old_source, disconnect)
    new_source_ref = weakref.ref(new_source)
    reader.subscribeTo(old_source)
    del old_source
    gc.collect()

    reader.subscribeTo(new_source)

    assert old_source_ref() is None
    assert callback_states == [1, 0]
    assert reader.header.isLinked == 0

    del new_source
    gc.collect()
    assert new_source_ref() is None


def test_resubscribeToEmbeddedSourcePreservesOwner():
    """Re-subscribing through a non-owning ``Msg_C`` proxy preserves its owner."""
    consumer = cModuleTemplate.cModuleTemplate()
    producer = cModuleTemplate.cModuleTemplate()

    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = [21.0, 22.0, 23.0]
    producer.dataOutMsg.write(payload)

    source = producer.dataOutMsg
    producer_ref = weakref.ref(producer)
    consumer.dataInMsg.subscribeTo(source)

    del producer
    gc.collect()
    assert producer_ref() is not None

    consumer.dataInMsg.subscribeTo(source)
    gc.collect()

    assert producer_ref() is not None, "re-subscribe released the embedded source owner (#1433)"
    assert list(consumer.dataInMsg.read().dataVector) == [21.0, 22.0, 23.0]

    consumer.dataInMsg.unsubscribe()
    gc.collect()
    assert producer_ref() is None


def test_configReaderKeepAliveTransfersToWrapper():
    """A config reader transfers its active source keep-alive to its wrapper."""
    config = cModuleTemplate.cModuleTemplateConfig()
    source_ref = _cppSourceInScope(config.dataInMsg, [31.0, 32.0, 33.0])
    gc.collect()
    assert source_ref() is not None

    config_ref = weakref.ref(config)
    module = config.createWrapper()
    del config
    gc.collect()

    assert config_ref() is None
    assert source_ref() is not None, "config-to-wrapper transfer released the source (#1433)"
    assert list(module.dataInMsg.read().dataVector) == [31.0, 32.0, 33.0]

    del module
    gc.collect()
    assert source_ref() is None


def test_configEmbeddedSourceOwnerSurvivesGC():
    """A wrapper reader retains the config that owns an embedded source."""
    consumer = cModuleTemplate.cModuleTemplate()
    producer_config = cModuleTemplate.cModuleTemplateConfig()

    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = [41.0, 42.0, 43.0]
    producer_config.dataOutMsg.write(payload)

    source = producer_config.dataOutMsg
    config_ref = weakref.ref(producer_config)
    source_ref = weakref.ref(source)
    consumer.dataInMsg.subscribeTo(source)

    del producer_config, source
    gc.collect()

    assert config_ref() is not None, "embedded source config owner was not retained (#1433)"
    assert source_ref() is None, "embedded config message proxy was retained instead of its owner (#1433)"
    assert list(consumer.dataInMsg.read().dataVector) == [41.0, 42.0, 43.0]

    consumer.dataInMsg.unsubscribe()
    gc.collect()
    assert config_ref() is None


def test_configSourceOwnerLeaseTransfersToWrapper():
    """An existing source lease follows config storage into its new wrapper."""
    consumer = cModuleTemplate.cModuleTemplate()
    producer_config = cModuleTemplate.cModuleTemplateConfig()

    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = [51.0, 52.0, 53.0]
    producer_config.dataOutMsg.write(payload)

    source = producer_config.dataOutMsg
    config_ref = weakref.ref(producer_config)
    consumer.dataInMsg.subscribeTo(source)

    producer = producer_config.createWrapper()
    producer_ref = weakref.ref(producer)
    del producer_config, producer, source
    gc.collect()

    assert config_ref() is None
    assert producer_ref() is not None, "source lease did not follow config ownership transfer (#1433)"
    assert list(consumer.dataInMsg.read().dataVector) == [51.0, 52.0, 53.0]

    consumer.dataInMsg.unsubscribe()
    gc.collect()
    assert producer_ref() is None


def test_moduleDeathReleasesSources():
    """When the owning C module is collected, its subscribed sources are released
    (no leak); this exercises the module-owned keep-alive dictionary."""
    mod = cModuleTemplate.cModuleTemplate()
    wr = _cppSourceInScope(mod.dataInMsg, [1.0, 2.0, 3.0])
    gc.collect()
    assert wr() is not None, "source not retained while the module is alive (#1433)"

    del mod
    gc.collect()
    assert wr() is None, "module garbage-collection did not release the subscribed source (#1433)"


def test_standaloneSubscriberSurvivesGCAndReleasesOnDeath():
    """A stand-alone Msg_C subscriber keeps its source alive, reads it correctly,
    and releases it when the subscriber itself is collected."""
    sub = messaging.CModuleTemplateMsg_C()
    wr = _cppSourceInScope(sub, [7.0, 8.0, 9.0])
    gc.collect()
    assert wr() is not None, "source GC'd despite a stand-alone subscription (#1433)"
    assert list(sub.read().dataVector) == [7.0, 8.0, 9.0]

    del sub
    gc.collect()
    assert wr() is None, "stand-alone subscriber death did not release its source (#1433)"


def test_rawAddressSubscriptionStillWorks():
    """Subscribing by raw integer address must keep working: that path has no Python
    source object to retain (the caller owns the source's lifetime), so the keep-alive
    layer must simply stay out of its way."""
    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = [3.0, 6.0, 9.0]
    persistent = messaging.CModuleTemplateMsg().write(payload)  # kept alive by the caller

    mod = cModuleTemplate.cModuleTemplate()
    mod.dataInMsg.subscribeTo(int(persistent.this))  # raw-address branch
    assert list(mod.dataInMsg.read().dataVector) == [3.0, 6.0, 9.0]


def test_rawAddressResubscribeReleasesPreviousKeepAlive():
    """Re-subscribing by raw integer address releases any previous Python source."""
    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = [9.0, 8.0, 7.0]
    persistent = messaging.CModuleTemplateMsg().write(payload)

    mod = cModuleTemplate.cModuleTemplate()
    wr = _cppSourceInScope(mod.dataInMsg, [1.0, 2.0, 3.0])
    gc.collect()
    assert wr() is not None

    mod.dataInMsg.subscribeTo(int(persistent.this))
    gc.collect()
    assert wr() is None, "raw-address re-subscribe did not release the previous source (#1433)"
    assert list(mod.dataInMsg.read().dataVector) == [9.0, 8.0, 7.0]


if __name__ == "__main__":
    test_moduleEmbeddedCppSourceSurvivesGC()
    test_moduleEmbeddedCMsgSourceSurvivesGC()
    test_moduleEmbeddedCMsgSourceOwnerSurvivesGC()
    test_unsubscribeReleasesKeepAlive()
    test_unsubscribeWeakrefCallbackCanResubscribe()
    test_resubscribeReplacesKeepAlive()
    test_resubscribeWeakrefCallbackCanUnsubscribe()
    test_resubscribeToEmbeddedSourcePreservesOwner()
    test_configReaderKeepAliveTransfersToWrapper()
    test_configEmbeddedSourceOwnerSurvivesGC()
    test_configSourceOwnerLeaseTransfersToWrapper()
    test_moduleDeathReleasesSources()
    test_standaloneSubscriberSurvivesGCAndReleasesOnDeath()
    test_rawAddressSubscriptionStillWorks()
    test_rawAddressResubscribeReleasesPreviousKeepAlive()
    print("All #1433 keep-alive tests passed.")
