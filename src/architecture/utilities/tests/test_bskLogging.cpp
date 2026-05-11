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

#include "architecture/utilities/bskLogging.h"
#include <gtest/gtest.h>

namespace {
int callBskErrorFromNonVoid(BSKLogger& bskLogger)
{
    bskLogger.bskError("fatal value %d", 7);
}

#ifdef _MSC_VER
// MSVC /EHc assumes extern "C" calls do not throw, so catch through a C++ helper.
__declspec(noinline)
#endif
void callCBskError(BSKLogger& bskLogger, const char* info)
{
    _bskError(&bskLogger, info);
}
}

TEST(BSKLogging, bskErrorThrowsFormattedMessage)
{
    BSKLogger bskLogger;

    try {
        (void)callBskErrorFromNonVoid(bskLogger);
    } catch (const BasiliskError& error) {
        EXPECT_STREQ("fatal value 7", error.what());
        return;
    }

    FAIL() << "Expected bskError to throw BasiliskError.";
}

TEST(BSKLogging, bskLogErrorMaintainsExistingBehavior)
{
    BSKLogger bskLogger;

    try {
        bskLogger.bskLog(BSK_ERROR, "legacy value %d", 9);
    } catch (const BasiliskError& error) {
        EXPECT_STREQ("legacy value 9", error.what());
        return;
    }

    FAIL() << "Expected bskLog with BSK_ERROR to throw BasiliskError.";
}

TEST(BSKLogging, cBskErrorTreatsMessageAsText)
{
    BSKLogger bskLogger;

    try {
#ifdef _MSC_VER
        // Keep MSVC from applying /EHc's extern "C" non-throwing assumption here.
        using CBskErrorCaller = void (*)(BSKLogger&, const char*);
        CBskErrorCaller volatile cBskErrorCaller = callCBskError;
        cBskErrorCaller(bskLogger, "preformatted %s message");
#else
        callCBskError(bskLogger, "preformatted %s message");
#endif
    } catch (const BasiliskError& error) {
        EXPECT_STREQ("preformatted %s message", error.what());
        return;
    }

    FAIL() << "Expected _bskError to throw BasiliskError.";
}
