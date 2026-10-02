/*
 ISC License

 Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder

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

#include <cstdlib>

#include "architecture/utilities/utcTime.h"

namespace {
struct tm
makeDate(int year, int month, int day, int hour, int minute, int second)
{
    struct tm date{};
    date.tm_year = year - 1900;
    date.tm_mon = month - 1;
    date.tm_mday = day;
    date.tm_hour = hour;
    date.tm_min = minute;
    date.tm_sec = second;
    return date;
}
} // namespace

TEST(UtcTime, returnsSecondsSinceTheUnixEpoch)
{
    struct tm date = makeDate(2026, 1, 1, 0, 0, 0);

    EXPECT_EQ(normalizeUtcTime(&date), 1767225600); // [s] 2026-01-01 00:00:00 UTC
}

TEST(UtcTime, rollsOverTheCalendarFields)
{
    struct tm date = makeDate(2026, 12, 31, 23, 59, 60);
    normalizeUtcTime(&date);

    EXPECT_EQ(date.tm_year, 2027 - 1900);
    EXPECT_EQ(date.tm_mon, 0);
    EXPECT_EQ(date.tm_mday, 1);
    EXPECT_EQ(date.tm_hour, 0);
    EXPECT_EQ(date.tm_min, 0);
    EXPECT_EQ(date.tm_sec, 0);
    EXPECT_EQ(date.tm_yday, 0);
}

#ifndef _WIN32
TEST(UtcTime, isIndependentOfTheTimeZoneAcrossADaylightSavingTransition)
{
    // Europe/Rome changes to summer time on 2026-03-29. The date is advanced like the environment modules do: the
    // epoch is normalized once, then the elapsed seconds are added to a copy that is normalized again.
    setenv("TZ", "Europe/Rome", 1);
    tzset();

    struct tm epoch = makeDate(2026, 3, 20, 0, 0, 0);
    normalizeUtcTime(&epoch);
    struct tm later = epoch;
    later.tm_sec += 10 * 86400; // [s] 10 days, past the transition
    normalizeUtcTime(&later);

    EXPECT_EQ(later.tm_mon, 2);
    EXPECT_EQ(later.tm_mday, 30);
    EXPECT_EQ(later.tm_hour, 0);
    EXPECT_EQ(later.tm_min, 0);
    EXPECT_EQ(later.tm_yday, 31 + 28 + 30 - 1);
}
#endif
