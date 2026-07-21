/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include <gtest/gtest.h>

#include <type_traits>
#include <utility>
#include <vector>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/CModuleTemplateMsgPayload.h"

class DerivedSysModel : public SysModel {};

namespace {

void
expectMessagePointersReferenceOwnStorage(Message<CModuleTemplateMsgPayload>& message)
{
    messagePointerData* pointers = message.GetPointers();
    EXPECT_EQ(reinterpret_cast<uintptr_t>(pointers->header), message.getHeaderAddress());
    EXPECT_EQ(reinterpret_cast<uintptr_t>(pointers->payload), message.getPayloadAddress());
}

} // namespace

TEST(MessagingPointerContract, PointerDataDefaultsToNull)
{
    messagePointerData pointers;
    MessageBase messageBase;
    ReadFunctorBase readerBase;

    EXPECT_EQ(pointers.header, nullptr);
    EXPECT_EQ(pointers.payload, nullptr);
    EXPECT_EQ(messageBase.GetPointers()->header, nullptr);
    EXPECT_EQ(messageBase.GetPointers()->payload, nullptr);
    EXPECT_EQ(readerBase.GetPointers()->header, nullptr);
    EXPECT_EQ(readerBase.GetPointers()->payload, nullptr);
}

TEST(MessagingPointerContract, ReadFunctorCopiesAndMovesPointerState)
{
    Message<CModuleTemplateMsgPayload> message;
    messagePointerData* messagePointers = message.GetPointers();
    ReadFunctor<CModuleTemplateMsgPayload> original = message.addSubscriber();

    ReadFunctor<CModuleTemplateMsgPayload> copied(original);
    EXPECT_EQ(copied.GetPointers()->header, messagePointers->header);
    EXPECT_EQ(copied.GetPointers()->payload, messagePointers->payload);

    ReadFunctor<CModuleTemplateMsgPayload> moved(std::move(copied));
    EXPECT_EQ(moved.GetPointers()->header, messagePointers->header);
    EXPECT_EQ(moved.GetPointers()->payload, messagePointers->payload);
    EXPECT_EQ(copied.GetPointers()->header, nullptr);
    EXPECT_EQ(copied.GetPointers()->payload, nullptr);
    EXPECT_FALSE(copied.isLinked());

    ReadFunctor<CModuleTemplateMsgPayload> copyAssigned;
    copyAssigned = original;
    EXPECT_EQ(copyAssigned.GetPointers()->header, messagePointers->header);
    EXPECT_EQ(copyAssigned.GetPointers()->payload, messagePointers->payload);

    ReadFunctor<CModuleTemplateMsgPayload> moveAssigned;
    moveAssigned = std::move(copyAssigned);
    EXPECT_EQ(moveAssigned.GetPointers()->header, messagePointers->header);
    EXPECT_EQ(moveAssigned.GetPointers()->payload, messagePointers->payload);
    EXPECT_EQ(copyAssigned.GetPointers()->header, nullptr);
    EXPECT_EQ(copyAssigned.GetPointers()->payload, nullptr);
    EXPECT_FALSE(copyAssigned.isLinked());
}

TEST(MessagingPointerContract, MessageCopiesAndMovesRebindPointerState)
{
    Message<CModuleTemplateMsgPayload> original;
    Message<CModuleTemplateMsgPayload> copied(original);
    expectMessagePointersReferenceOwnStorage(copied);

    Message<CModuleTemplateMsgPayload> moved(std::move(copied));
    expectMessagePointersReferenceOwnStorage(moved);
    expectMessagePointersReferenceOwnStorage(copied);

    Message<CModuleTemplateMsgPayload> copyAssigned;
    copyAssigned = original;
    expectMessagePointersReferenceOwnStorage(copyAssigned);

    Message<CModuleTemplateMsgPayload> moveAssigned;
    moveAssigned = std::move(copyAssigned);
    expectMessagePointersReferenceOwnStorage(moveAssigned);
    expectMessagePointersReferenceOwnStorage(copyAssigned);

    std::vector<Message<CModuleTemplateMsgPayload>> messages;
    messages.reserve(1);
    messages.emplace_back();
    uintptr_t initialHeaderAddress = messages.front().getHeaderAddress();
    messages.emplace_back();

    EXPECT_NE(messages.front().getHeaderAddress(), initialHeaderAddress);
    expectMessagePointersReferenceOwnStorage(messages.front());
}

TEST(RecorderCopyContract, SysModelDerivedTypesAreNotCopyable)
{
    static_assert(!std::is_copy_constructible<SysModel>::value, "SysModel must not be copy constructible.");
    static_assert(!std::is_copy_assignable<SysModel>::value, "SysModel must not be copy assignable.");
    static_assert(!std::is_copy_constructible<DerivedSysModel>::value,
                  "SysModel-derived module types must not be copy constructible by default.");
    static_assert(!std::is_copy_assignable<DerivedSysModel>::value,
                  "SysModel-derived module types must not be copy assignable by default.");
}

TEST(RecorderCopyContract, RecorderCopiesStateWithUniqueModuleId)
{
    static_assert(std::is_copy_constructible<Recorder<CModuleTemplateMsgPayload>>::value,
                  "Recorder must remain copy constructible for value-returning message APIs.");
    static_assert(std::is_copy_assignable<Recorder<CModuleTemplateMsgPayload>>::value,
                  "Recorder must remain copy assignable.");

    Message<CModuleTemplateMsgPayload> msg;
    CModuleTemplateMsgPayload payload = {};
    payload.dataVector[0] = 1.0;  // [units]

    uint64_t firstWriteTime = 0;  // [ns]
    msg.write(&payload, 0, firstWriteTime);

    uint64_t recordInterval = 10;  // [ns]
    Recorder<CModuleTemplateMsgPayload> original(&msg, recordInterval);
    original.ModelTag = "sourceRecorder";
    original.RNGSeed = 7;
    original.UpdateState(firstWriteTime);

    Recorder<CModuleTemplateMsgPayload> copied(original);

    EXPECT_EQ(copied.ModelTag, original.ModelTag);
    EXPECT_EQ(copied.RNGSeed, original.RNGSeed);
    EXPECT_NE(copied.moduleID, original.moduleID);
    ASSERT_EQ(copied.record().size(), 1U);
    EXPECT_DOUBLE_EQ(copied.record()[0].dataVector[0], 1.0);

    payload.dataVector[0] = 2.0;  // [units]
    uint64_t secondWriteTime = 10;  // [ns]
    msg.write(&payload, 0, secondWriteTime);

    original.UpdateState(secondWriteTime);
    copied.UpdateState(secondWriteTime);

    ASSERT_EQ(original.record().size(), 2U);
    ASSERT_EQ(copied.record().size(), 2U);
    EXPECT_DOUBLE_EQ(original.record()[1].dataVector[0], 2.0);
    EXPECT_DOUBLE_EQ(copied.record()[1].dataVector[0], 2.0);
}

TEST(RecorderCopyContract, RecorderAssignmentKeepsTargetModuleId)
{
    Message<CModuleTemplateMsgPayload> msg;
    CModuleTemplateMsgPayload payload = {};
    payload.dataVector[0] = 3.0;  // [units]

    uint64_t firstWriteTime = 0;  // [ns]
    msg.write(&payload, 0, firstWriteTime);

    uint64_t recordInterval = 10;  // [ns]
    Recorder<CModuleTemplateMsgPayload> original(&msg, recordInterval);
    original.UpdateState(firstWriteTime);

    Recorder<CModuleTemplateMsgPayload> assigned;
    int64_t assignedModuleId = assigned.moduleID;
    assigned = original;

    EXPECT_EQ(assigned.moduleID, assignedModuleId);
    EXPECT_NE(assigned.moduleID, original.moduleID);
    ASSERT_EQ(assigned.record().size(), 1U);
    EXPECT_DOUBLE_EQ(assigned.record()[0].dataVector[0], 3.0);
}
