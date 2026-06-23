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


if __name__ == "__main__":
    test_standaloneMsgSurvivesGarbageCollection()
    test_unsubscribeReleasesKeepAlive()
