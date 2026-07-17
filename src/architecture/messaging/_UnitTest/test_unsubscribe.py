#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
import time
from typing import Union

from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging

def assertConnectionWithValues(
    inMsg: Union[messaging.CModuleTemplateMsg_C, messaging.CModuleTemplateMsgReader],
    outMsg: Union[messaging.CModuleTemplateMsg_C, messaging.CModuleTemplateMsg]
):
    val = time.time() # unique value

    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = [val, 0, 0]
    outMsg.write(payload)

    if isinstance(inMsg, messaging.CModuleTemplateMsg_C):
        readPayload: messaging.CModuleTemplateMsgPayload = inMsg.read()
    else:
        readPayload: messaging.CModuleTemplateMsgPayload = inMsg()

    assert readPayload.dataVector[0] == val


def assertRawPointersCleared(
    outMsg: Union[messaging.CModuleTemplateMsg_C, messaging.CModuleTemplateMsg]
):
    """Verify that unsubscribing clears a C++ reader's type-erased pointers."""
    reader = messaging.CModuleTemplateMsgReader()
    reader.subscribeTo(outMsg)

    pointers = reader.GetPointers()
    assert pointers.header is not None
    assert pointers.payload is not None

    reader.unsubscribe()

    assert not reader.isLinked()
    assert pointers.header is None
    assert pointers.payload is None

    currentPointers = reader.GetPointers()
    assert currentPointers.header is None
    assert currentPointers.payload is None


def test_unsubscribe():
    """
    testing the unsubscribe functions of C and C++ messages
    """
    # create modules
    cMod = cModuleTemplate.cModuleTemplate()
    cppMod = cppModuleTemplate.CppModuleTemplate()

    # check that unsubscribe is a noop for unlinked OnMsgs
    cMod.dataInMsg.unsubscribe()
    cppMod.dataInMsg.unsubscribe()

    # check that unsubscribe is noop for (C) OutMsg
    cMod.dataOutMsg.unsubscribe()

    # connect the input messages:
    #   C -> Cpp
    #   Cpp -> C
    cMod.dataInMsg.subscribeTo(cppMod.dataOutMsg)
    cppMod.dataInMsg.subscribeTo(cMod.dataOutMsg)

    # check that unsubscribe is noop for (C) OutMsg
    # even when connected to an InMsg
    cMod.dataOutMsg.unsubscribe()

    assert messaging.CModuleTemplateMsg_C_isLinked(cMod.dataInMsg)
    assert cppMod.dataInMsg.isLinked()

    assert messaging.CModuleTemplateMsg_C_isLinked(cMod.dataOutMsg)

    assertConnectionWithValues(cMod.dataInMsg, cppMod.dataOutMsg)
    assertConnectionWithValues(cppMod.dataInMsg, cMod.dataOutMsg)

    # check that unsubscribe works
    cMod.dataInMsg.unsubscribe()
    cppMod.dataInMsg.unsubscribe()

    assert not messaging.CModuleTemplateMsg_C_isLinked(cMod.dataInMsg)
    assert not cppMod.dataInMsg.isLinked()

    # try connecting differently:
    #  C -> C
    #  Cpp -> Cpp
    cMod.dataInMsg.subscribeTo(cMod.dataOutMsg)
    cppMod.dataInMsg.subscribeTo(cppMod.dataOutMsg)

    # check that unsubscribe is noop for (C) OutMsg
    # even when connected to an InMsg
    cMod.dataOutMsg.unsubscribe()

    assert messaging.CModuleTemplateMsg_C_isLinked(cMod.dataInMsg)
    assert cppMod.dataInMsg.isLinked()

    assertConnectionWithValues(cMod.dataInMsg, cMod.dataOutMsg)
    assertConnectionWithValues(cppMod.dataInMsg, cppMod.dataOutMsg)

    # check that unsubscribe works
    cMod.dataInMsg.unsubscribe()
    cppMod.dataInMsg.unsubscribe()

    assert not messaging.CModuleTemplateMsg_C_isLinked(cMod.dataInMsg)
    assert not cppMod.dataInMsg.isLinked()


def test_unsubscribe_clears_raw_pointers():
    """Verify that raw pointers are cleared for both C and C++ message sources."""
    cMod = cModuleTemplate.cModuleTemplate()
    cppOutMsg = messaging.CModuleTemplateMsg()

    assertRawPointersCleared(cMod.dataOutMsg)
    assertRawPointersCleared(cppOutMsg)


def test_raw_pointer_bookkeeping_is_not_public():
    """Verify that generated bindings hide raw-pointer bookkeeping members."""
    message = messaging.CModuleTemplateMsg()
    reader = messaging.CModuleTemplateMsgReader()

    assert not hasattr(message, "pointers")
    assert not hasattr(message, "reference")
    assert not hasattr(reader, "headerVoidPtr")
    assert not hasattr(reader, "payloadVoidPtr")
    assert not hasattr(reader, "reference")

    assert callable(message.GetPointers)
    assert callable(reader.GetPointers)


if __name__ == "__main__":
    test_unsubscribe()
    test_unsubscribe_clears_raw_pointers()
    test_raw_pointer_bookkeeping_is_not_public()
