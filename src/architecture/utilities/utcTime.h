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

#ifndef UTC_TIME_H
#define UTC_TIME_H

#include <ctime>

/*! @brief Normalize a broken-down UTC date and time without using the time zone of the computer.
 *
 * The fields of the structure are adjusted to a valid calendar date and time, so that for example adding seconds to
 * `tm_sec` rolls over the minutes, hours, days, and years, and `tm_wday` and `tm_yday` are set. Unlike `mktime`, the
 * structure is interpreted as UTC and not as local time. `mktime` shifts the hour by one when the daylight saving flag
 * left in the structure by an earlier call does not match the date, which happens when a date is advanced across a
 * daylight saving transition on a computer whose time zone has one.
 *
 * @param dateTime [in,out] broken-down UTC date and time, normalized in place; `tm_isdst` is set to 0
 * @return the time in seconds since 1970-01-01 00:00:00 UTC, or -1 if it cannot be represented
 */
static inline time_t
normalizeUtcTime(struct tm* dateTime)
{
    dateTime->tm_isdst = 0;
#ifdef _WIN32
    return _mkgmtime(dateTime);
#else
    return timegm(dateTime);
#endif
}

#endif
