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
# Purpose:  Regression test for issue #676 -- a stand-alone message that is subscribed to must
#           not be garbage-collected while a C++ reader still points into its memory. Before the
#           keep-alive fix this test read freed/garbage memory and failed (or crashed).
#

import gc
import weakref

from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport as uts


def _subscribeInLocalScope(module, dataVector):
    """Create a stand-alone message in a scope that returns, subscribe the module to it, and
    let the only local Python reference go out of scope on return. After this returns, the
    message is kept alive *solely* by the subscription keep-alive (issue #676); without the fix
    it is garbage-collected and the module reads freed memory."""
    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = dataVector
    standaloneMsg = messaging.CModuleTemplateMsg().write(payload)
    module.dataInMsg.subscribeTo(standaloneMsg)
    # standaloneMsg goes out of scope here; no other Python reference to it remains


def _recordInLocalScope(dataVector):
    """Create a recorder whose source has no other strong Python reference."""
    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = dataVector
    standaloneMsg = messaging.CModuleTemplateMsg().write(payload)
    sourceReference = weakref.ref(standaloneMsg)
    recorder = standaloneMsg.recorder()
    return recorder, sourceReference


def _addSubscriberInLocalScope(dataVector):
    """Create a reader whose source has no other strong Python reference."""
    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = dataVector
    standaloneMsg = messaging.CModuleTemplateMsg().write(payload)
    sourceReference = weakref.ref(standaloneMsg)
    reader = standaloneMsg.addSubscriber()
    return reader, sourceReference


def test_standaloneMsgSurvivesGarbageCollection():
    """A C++ reader subscribed to a stand-alone C++ message keeps that message alive across GC."""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.)))

    module = cppModuleTemplate.CppModuleTemplate()
    module.ModelTag = "cppModule1"
    scSim.AddModelToTask("dynamicsTask", module)

    inputVector = [10., 20., 30.]
    _subscribeInLocalScope(module, inputVector)

    # Force the failure mode of #676: without the keep-alive, the C++ Message backing the
    # subscription is freed here and the recorded input becomes garbage.
    gc.collect()

    dataInRec = module.dataInMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataInRec)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(3.0))
    scSim.ExecuteSimulation()

    testFailCount, testMessages = uts.compareArray(
        [inputVector] * len(dataInRec.dataVector),
        dataInRec.dataVector,
        0.01,
        "subscribed stand-alone message data was corrupted after GC (issue #676).",
        0,
        [],
    )

    assert testFailCount < 1, testMessages


def test_unsubscribeReleasesKeepAlive():
    """unsubscribe() must drop the keep-alive reference without crashing."""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    module = cppModuleTemplate.CppModuleTemplate()
    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = [1., 2., 3.]
    standaloneMsg = messaging.CModuleTemplateMsg().write(payload)

    module.dataInMsg.subscribeTo(standaloneMsg)
    assert module.dataInMsg.isSubscribedTo(standaloneMsg)

    module.dataInMsg.unsubscribe()
    assert not module.dataInMsg.isLinked()

    # The reader no longer holds the message; dropping our handle and collecting must be safe.
    del standaloneMsg
    gc.collect()


def test_messageRecorderKeepsSourceAlive():
    """A recorder keeps its stand-alone C++ source alive until the recorder is destroyed."""
    inputVector = [4., 5., 6.]
    recorder, sourceReference = _recordInLocalScope(inputVector)

    gc.collect()
    assert sourceReference() is not None

    recorder.UpdateState(0)
    assert recorder.size() == 1
    assert list(recorder.dataVector[0]) == inputVector

    del recorder
    gc.collect()
    assert sourceReference() is None


def test_addSubscriberKeepsSourceAlive():
    """A reader returned by addSubscriber() keeps its C++ source alive."""
    inputVector = [7., 8., 9.]
    reader, sourceReference = _addSubscriberInLocalScope(inputVector)

    gc.collect()
    assert sourceReference() is not None
    assert list(reader().dataVector) == inputVector

    del reader
    gc.collect()
    assert sourceReference() is None


def test_releaseKeepAliveIsReentrantSafe():
    """A source weakref callback may safely re-enter unsubscribe()."""
    module = cppModuleTemplate.CppModuleTemplate()
    standaloneMsg = messaging.CModuleTemplateMsg()
    module.dataInMsg.subscribeTo(standaloneMsg)

    callbackCount = 0

    def unsubscribeAgain(_):
        nonlocal callbackCount
        callbackCount += 1
        module.dataInMsg.unsubscribe()

    sourceReference = weakref.ref(standaloneMsg, unsubscribeAgain)
    del standaloneMsg
    module.dataInMsg.unsubscribe()

    assert sourceReference() is None
    assert callbackCount == 1
    assert not module.dataInMsg.isLinked()


def test_unsubscribeRejectsReentrantKeepAlive():
    """A reentrant subscribe during unsubscribe does not retain its source."""
    module = cppModuleTemplate.CppModuleTemplate()
    replacementMsg = messaging.CModuleTemplateMsg()
    replacementReference = weakref.ref(replacementMsg)
    initialMsg = messaging.CModuleTemplateMsg()
    callbackLinkedStates = []

    def subscribeToReplacement(_):
        callbackLinkedStates.append(module.dataInMsg.isLinked())
        source = replacementReference()
        if source is not None:
            module.dataInMsg.subscribeTo(source)

    initialReference = weakref.ref(initialMsg, subscribeToReplacement)
    module.dataInMsg.subscribeTo(initialMsg)
    del initialMsg
    gc.collect()

    module.dataInMsg.unsubscribe()

    assert initialReference() is None
    assert callbackLinkedStates == [False]
    assert not module.dataInMsg.isLinked()

    del replacementMsg
    gc.collect()
    assert replacementReference() is None


def test_moveAssignmentRejectsReentrantKeepAlive():
    """A reentrant subscribe during move assignment does not leak its source."""
    module = cppModuleTemplate.CppModuleTemplate()
    finalMsg = messaging.CModuleTemplateMsg()
    intermediateMsg = messaging.CModuleTemplateMsg()
    intermediateReference = weakref.ref(intermediateMsg)
    initialMsg = messaging.CModuleTemplateMsg()
    callbackCount = 0

    def subscribeToIntermediate(_):
        nonlocal callbackCount
        callbackCount += 1
        source = intermediateReference()
        assert source is not None
        module.dataInMsg.subscribeTo(source)

    initialReference = weakref.ref(initialMsg, subscribeToIntermediate)
    module.dataInMsg.subscribeTo(initialMsg)
    del initialMsg
    gc.collect()

    module.dataInMsg.subscribeTo(finalMsg)

    assert initialReference() is None
    assert callbackCount == 1
    assert module.dataInMsg.isSubscribedTo(finalMsg)

    del intermediateMsg
    gc.collect()
    assert intermediateReference() is None


def test_copyAssignmentSurvivesReentrantUnsubscribe():
    """A reader-vector copy assignment commits after a reentrant unsubscribe."""
    readers = messaging.CModuleTemplateMsgInMsgsVector()
    initialMsg = messaging.CModuleTemplateMsg()
    finalMsg = messaging.CModuleTemplateMsg()
    finalReference = weakref.ref(finalMsg)

    readers.append(initialMsg.addSubscriber())
    finalReader = finalMsg.addSubscriber()
    callbackCount = 0

    def unsubscribeAgain(_):
        nonlocal callbackCount
        callbackCount += 1
        readers[0].unsubscribe()

    initialReference = weakref.ref(initialMsg, unsubscribeAgain)
    del initialMsg
    gc.collect()

    readers[0] = finalReader

    assert initialReference() is None
    assert callbackCount == 1
    assert readers[0].isLinked()
    assert readers[0].isSubscribedTo(finalMsg)

    del finalReader
    del finalMsg
    gc.collect()
    assert finalReference() is not None

    del readers
    gc.collect()
    assert finalReference() is None


if __name__ == "__main__":
    test_standaloneMsgSurvivesGarbageCollection()
    test_unsubscribeReleasesKeepAlive()
    test_messageRecorderKeepsSourceAlive()
    test_addSubscriberKeepsSourceAlive()
    test_releaseKeepAliveIsReentrantSafe()
    test_unsubscribeRejectsReentrantKeepAlive()
    test_moveAssignmentRejectsReentrantKeepAlive()
    test_copyAssignmentSurvivesReentrantUnsubscribe()
