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

#include "architecture/utilities/stateExtrapolation.h"

namespace {
SCStatesMsgPayload
makeState()
{
    SCStatesMsgPayload state{};
    state.r_BN_N[0] = 7.0e6; // [m]
    state.v_BN_N[1] = 7.5e3; // [m/s]
    state.r_CN_N[0] = 7.0e6; // [m]
    state.v_CN_N[1] = 7.5e3; // [m/s]
    return state;
}
} // namespace

TEST(StateExtrapolation, advancesPositionByHalfTheMessageAge)
{
    const uint64_t stepNanos = 10000000000ULL; // [ns] 10 s
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(makeState(), 2 * stepNanos, stepNanos, stepNanos);

    EXPECT_DOUBLE_EQ(out.r_BN_N[0], 7.0e6);
    EXPECT_DOUBLE_EQ(out.r_BN_N[1], 7.5e3 * 5.0); // v * age / 2
    EXPECT_DOUBLE_EQ(out.r_CN_N[1], 7.5e3 * 5.0);
    EXPECT_DOUBLE_EQ(out.v_BN_N[1], 7.5e3);
}

TEST(StateExtrapolation, isNoOpWhenMessageWrittenAtCurrentTime)
{
    const SCStatesMsgPayload in = makeState();
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(in, 5000000000ULL, 5000000000ULL, 0);

    EXPECT_DOUBLE_EQ(out.r_BN_N[1], in.r_BN_N[1]);
}

TEST(StateExtrapolation, isNoOpForMessageOlderThanThePreviousUpdate)
{
    const SCStatesMsgPayload in = makeState();
    // written at t = 0 and read at t = 30 s by a module that last updated at t = 20 s: stale message
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(in, 30000000000ULL, 0, 20000000000ULL);

    EXPECT_DOUBLE_EQ(out.r_BN_N[1], in.r_BN_N[1]);
}
