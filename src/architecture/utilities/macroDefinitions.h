/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef SIM_FSW_MACROS_H
#define SIM_FSW_MACROS_H

#define MAX_CIRCLE_NUM 10
#define MAX_LIMB_PNTS 2000
#define MAX_EFF_CNT 36
#define MAX_NUM_CSS_SENSORS 32
#define MAX_ST_VEH_COUNT 4

#define NANO2SEC        1e-9
#define SEC2NANO        1e9
#define RECAST6X6       (double (*)[6])
#define RECAST3X3       (double (*)[3])
#define RECAST2x2       (double (*)[2])
#define SEC2HOUR        1./3600.

#include <stdint.h>  // For uint64_t
#include <float.h>   // For DBL_MANT_DIG
#include <math.h>    // For NAN, fabs()
#include <stdio.h>
#include <inttypes.h>

/**
 * Converts nanoseconds to seconds (double), with basic precision check.
 * Returns NAN if conversion would lose precision.
 */
static inline  double nanoToSec(uint64_t nanos) {
    // Double precision loses exact integer values past 2^53
    const uint64_t MAX_SAFE_UINT64_FOR_DOUBLE = (1ULL << DBL_MANT_DIG);  // Usually 2^53

    if (nanos > MAX_SAFE_UINT64_FOR_DOUBLE) {
        fprintf(stderr,
            "[nano_to_sec] ERROR: Input %" PRIu64
            " exceeds safe conversion limit (~%" PRIu64 "). Precision may be lost. Returning NAN.\n",
            nanos, MAX_SAFE_UINT64_FOR_DOUBLE);
        return NAN;  // Indicate precision loss
    }

    return (double)nanos * NANO2SEC;  // Convert to seconds
}

/**
 * Takes two times in nanoseconds, takes their difference and converts to seconds (double),
 * with basic precision check.
 * Returns NAN if conversion would lose precision.
 */
static inline  double diffNanoToSec(uint64_t time1Nano, uint64_t time2Nano) {
    double signedTimeDifference;

    if (time1Nano >= time2Nano) {
        signedTimeDifference = nanoToSec(time1Nano - time2Nano);
    } else {
        signedTimeDifference = -nanoToSec(time2Nano - time1Nano);
    }

    return signedTimeDifference;
}

/**
 * Converts seconds (double) to nanoseconds (uint64_t).
 * Returns NAN on error (e.g. negative input or overflow)
 */
static inline uint64_t secToNano(double seconds) {
    if (seconds < 0.0) {
        fprintf(stderr,
            "[secToNano] ERROR: Negative time value passed (%.15f seconds). Returning 0.\n",
            seconds);
        return 0;
    }

    double result = seconds * SEC2NANO;

    if (result > (double)UINT64_MAX) {
        fprintf(stderr,
            "[secToNano] ERROR: Input time %.15f seconds exceeds uint64_t capacity. Returning 0.\n",
            seconds);
        return 0;
    }

    return (uint64_t)(result + 0.5);  // Round to nearest integer
}

#endif
