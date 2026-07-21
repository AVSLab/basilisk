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


def pointerAddresses(pointers: messaging.messagePointerData):
    """Return the numeric header and payload addresses from a raw pointer view."""
    return int(pointers.header), int(pointers.payload)


def assertRawPointersFollowResubscription(
    firstOutMsg: Union[messaging.CModuleTemplateMsg_C, messaging.CModuleTemplateMsg],
    secondOutMsg: Union[messaging.CModuleTemplateMsg_C, messaging.CModuleTemplateMsg],
):
    """Verify that a borrowed pointer view follows reader re-subscription."""
    reader = messaging.CModuleTemplateMsgReader()
    reader.subscribeTo(firstOutMsg)
    pointers = reader.GetPointers()
    firstAddresses = pointerAddresses(pointers)

    reader.subscribeTo(secondOutMsg)

    heldViewAddresses = pointerAddresses(pointers)
    assert heldViewAddresses != firstAddresses
    assert heldViewAddresses == pointerAddresses(reader.GetPointers())


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
    assert not hasattr(message, "setPointerData")
    assert not hasattr(reader, "setPointerData")
    assert not hasattr(reader, "clearPointerData")

    assert callable(message.GetPointers)
    assert callable(reader.GetPointers)


def test_message_base_types_are_shared_across_payload_modules():
    """Verify that generated messages use the central Python base proxy types."""
    cModuleMessage = messaging.CModuleTemplateMsg()
    navMessage = messaging.NavAttMsg()
    cModuleReader = messaging.CModuleTemplateMsgReader()
    navReader = messaging.NavAttMsgReader()

    assert isinstance(cModuleMessage, messaging.MessageBase)
    assert isinstance(navMessage, messaging.MessageBase)
    assert isinstance(cModuleReader, messaging.ReadFunctorBase)
    assert isinstance(navReader, messaging.ReadFunctorBase)
    assert isinstance(cModuleMessage.GetPointers(), messaging.messagePointerData)
    assert isinstance(navReader.GetPointers(), messaging.messagePointerData)


def test_raw_pointer_views_follow_resubscription():
    """Verify that held views track new C and C++ message sources."""
    firstCModule = cModuleTemplate.cModuleTemplate()
    secondCModule = cModuleTemplate.cModuleTemplate()
    firstCppMessage = messaging.CModuleTemplateMsg()
    secondCppMessage = messaging.CModuleTemplateMsg()

    assertRawPointersFollowResubscription(
        firstCModule.dataOutMsg, secondCModule.dataOutMsg
    )
    assertRawPointersFollowResubscription(firstCppMessage, secondCppMessage)


def test_copied_readers_preserve_raw_pointers():
    """Verify that copied readers retain the subscribed source pointer view."""
    source = messaging.CModuleTemplateMsg()
    sourceAddresses = (
        source.getHeaderAddress(),
        source.getPayloadAddress(),
    )
    reader = source.addSubscriber()

    assert pointerAddresses(reader.GetPointers()) == sourceAddresses

    readers = messaging.CModuleTemplateMsgInMsgsVector()
    readers.append(reader)

    assert pointerAddresses(readers[0].GetPointers()) == sourceAddresses


def test_copied_messages_rebind_raw_pointers():
    """Verify that copied and relocated messages expose their own storage."""
    source = messaging.CModuleTemplateMsg()
    messages = messaging.CModuleTemplateMsgOutMsgsVector()
    messages.append(source)

    copiedMessage = messages[0]
    copiedAddresses = (
        copiedMessage.getHeaderAddress(),
        copiedMessage.getPayloadAddress(),
    )
    assert copiedAddresses != (
        source.getHeaderAddress(),
        source.getPayloadAddress(),
    )
    assert pointerAddresses(copiedMessage.GetPointers()) == copiedAddresses

    previousHeaderAddress = copiedMessage.getHeaderAddress()
    del copiedMessage
    messages.resize(messages.capacity() + 1)

    relocatedMessage = messages[0]
    relocatedAddresses = (
        relocatedMessage.getHeaderAddress(),
        relocatedMessage.getPayloadAddress(),
    )
    assert relocatedMessage.getHeaderAddress() != previousHeaderAddress
    assert pointerAddresses(relocatedMessage.GetPointers()) == relocatedAddresses


def test_raw_pointer_base_types_default_to_null():
    """Verify that standalone pointer and base objects default to null addresses."""
    pointerData = messaging.messagePointerData()
    messageBase = messaging.MessageBase()
    readerBase = messaging.ReadFunctorBase()

    assert pointerData.header is None
    assert pointerData.payload is None
    assert messageBase.GetPointers().header is None
    assert messageBase.GetPointers().payload is None
    assert readerBase.GetPointers().header is None
    assert readerBase.GetPointers().payload is None


if __name__ == "__main__":
    test_unsubscribe()
    test_unsubscribe_clears_raw_pointers()
    test_raw_pointer_bookkeeping_is_not_public()
    test_message_base_types_are_shared_across_payload_modules()
    test_raw_pointer_views_follow_resubscription()
    test_copied_readers_preserve_raw_pointers()
    test_copied_messages_rebind_raw_pointers()
    test_raw_pointer_base_types_default_to_null()
