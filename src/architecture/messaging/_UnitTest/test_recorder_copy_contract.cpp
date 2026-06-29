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

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/CModuleTemplateMsgPayload.h"

class DerivedSysModel : public SysModel {};

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
