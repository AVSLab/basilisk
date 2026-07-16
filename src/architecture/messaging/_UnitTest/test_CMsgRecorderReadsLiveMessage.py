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
#   Module Name:        messaging (C-message recorder)
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      June 24, 2026
#

"""
Regression tests for issues #338 and #1433.

A recorder attached to a C module's C output message must read the module's
live message data, not a stale snapshot. The C-message read path reaches the
payload by pointer arithmetic from the start of the ``{type}_C`` struct, so this
test verifies end-to-end that the recorded payload tracks the values the module
actually writes over time (and is not constant, which would indicate the
recorder is bound to the wrong / a copied address).

The recorder also stores raw pointers into its ``Msg_C`` source. It must retain
the object that owns that storage until the recorder is collected, including
when config-owned storage transfers into a C-module wrapper.
"""

import gc
import weakref

import numpy as np
from Basilisk.architecture import bskLogging, messaging
from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.utilities import SimulationBaseClass, macros


def test_cMsgRecorderReadsLiveMessage():
    """The C-message recorder must capture the module's live, time-varying output."""
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("testProcess")
    process.addTask(scSim.CreateNewTask("testTask", macros.sec2nano(1.0)))

    module = cModuleTemplate.cModuleTemplate()
    module.ModelTag = "cModule"
    scSim.AddModelToTask("testTask", module)

    # Record the module's C output message.
    rec = module.dataOutMsg.recorder()
    scSim.AddModelToTask("testTask", rec)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(5.0))
    scSim.ExecuteSimulation()

    recorded = np.array(rec.dataVector)

    # With no input message linked, cModuleTemplate writes dataVector = [dummy, 0, 0]
    # where dummy increments by 1 each step (reset to 0). So the first component must
    # be the strictly increasing series 1, 2, 3, ... matching what the module wrote.
    firstComponent = recorded[:, 0]
    expected = np.arange(1, len(firstComponent) + 1, dtype=float)
    np.testing.assert_allclose(firstComponent, expected, atol=1e-12)

    # Guard the #338 failure mode directly: a recorder bound to a stale copy would
    # report a constant value rather than the live, time-varying message.
    assert len(np.unique(firstComponent)) > 1, \
        "recorded C message is constant; recorder is not reading the live message"


def test_standaloneCMsgRecorderKeepsSourceAlive():
    """A recorder retains its stand-alone ``Msg_C`` source until collection."""
    inputVector = [1.0, 2.0, 3.0]  # [-]
    payload = messaging.CModuleTemplateMsgPayload(dataVector=inputVector)
    source = messaging.CModuleTemplateMsg_C().write(payload)
    sourceReference = weakref.ref(source)
    recorder = source.recorder()

    del source
    gc.collect()

    assert sourceReference() is not None
    recorder.UpdateState(0)
    assert list(recorder.dataVector[0]) == inputVector

    del recorder
    gc.collect()

    assert sourceReference() is None


def test_copiedCMsgRecorderKeepsSourceAlive():
    """A copied recorder inherits the original recorder's source retention."""
    emptyRecorder = messaging.CModuleTemplateMsgRecorder()
    assert emptyRecorder.size() == 0

    inputVector = [10.0, 11.0, 12.0]  # [-]
    source = messaging.CModuleTemplateMsg_C().write(
        messaging.CModuleTemplateMsgPayload(dataVector=inputVector)
    )
    sourceReference = weakref.ref(source)
    originalRecorder = source.recorder()
    copiedRecorder = type(originalRecorder)(originalRecorder)

    del source, originalRecorder
    gc.collect()

    assert sourceReference() is not None
    copiedRecorder.UpdateState(0)
    assert list(copiedRecorder.dataVector[0]) == inputVector

    del copiedRecorder
    gc.collect()

    assert sourceReference() is None


def test_directCMsgRecorderConstructorKeepsSourceAlive():
    """The public recorder constructor retains a direct ``Msg_C`` argument."""
    inputVector = [13.0, 14.0, 15.0]  # [-]
    source = messaging.CModuleTemplateMsg_C().write(
        messaging.CModuleTemplateMsgPayload(dataVector=inputVector)
    )
    sourceReference = weakref.ref(source)
    recorder = messaging.CModuleTemplateMsgRecorder(source, 0)

    del source
    gc.collect()

    assert sourceReference() is not None
    recorder.UpdateState(0)
    assert list(recorder.dataVector[0]) == inputVector

    del recorder
    gc.collect()

    assert sourceReference() is None


def test_embeddedCMsgRecorderKeepsModuleOwnerAlive():
    """A recorder retains the module that owns an embedded ``Msg_C`` source."""
    inputVector = [4.0, 5.0, 6.0]  # [-]
    module = cModuleTemplate.cModuleTemplate()
    module.dataOutMsg.write(
        messaging.CModuleTemplateMsgPayload(dataVector=inputVector)
    )

    source = module.dataOutMsg
    moduleReference = weakref.ref(module)
    recorder = source.recorder()

    del module, source
    gc.collect()

    assert moduleReference() is not None
    recorder.UpdateState(0)
    assert list(recorder.dataVector[0]) == inputVector

    del recorder
    gc.collect()

    assert moduleReference() is None


def test_configCMsgRecorderLeaseTransfersToWrapper():
    """A recorder lease follows config-owned storage into its C-module wrapper."""
    inputVector = [7.0, 8.0, 9.0]  # [-]
    config = cModuleTemplate.cModuleTemplateConfig()
    config.dataOutMsg.write(
        messaging.CModuleTemplateMsgPayload(dataVector=inputVector)
    )

    source = config.dataOutMsg
    configReference = weakref.ref(config)
    recorder = source.recorder()
    module = config.createWrapper()
    moduleReference = weakref.ref(module)

    del config, module, source
    gc.collect()

    assert configReference() is None
    assert moduleReference() is not None
    recorder.UpdateState(0)
    assert list(recorder.dataVector[0]) == inputVector

    del recorder
    gc.collect()

    assert moduleReference() is None


if __name__ == "__main__":
    test_cMsgRecorderReadsLiveMessage()
    test_standaloneCMsgRecorderKeepsSourceAlive()
    test_copiedCMsgRecorderKeepsSourceAlive()
    test_directCMsgRecorderConstructorKeepsSourceAlive()
    test_embeddedCMsgRecorderKeepsModuleOwnerAlive()
    test_configCMsgRecorderLeaseTransfersToWrapper()
