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

TEST(StateExtrapolation, scalesWithTheMessageAge)
{
    const uint64_t secondNanos = 1000000000ULL; // [ns]
    for (uint64_t ageSeconds : { 1ULL, 5ULL, 60ULL, 3600ULL }) {
        const uint64_t written = 100 * secondNanos;
        const SCStatesMsgPayload out =
          extrapolateScStateToStepMidpoint(makeState(), written + ageSeconds * secondNanos, written, written);

        EXPECT_DOUBLE_EQ(out.r_BN_N[1], 7.5e3 * 0.5 * static_cast<double>(ageSeconds)) << "age [s] " << ageSeconds;
        EXPECT_DOUBLE_EQ(out.r_CN_N[1], 7.5e3 * 0.5 * static_cast<double>(ageSeconds)) << "age [s] " << ageSeconds;
    }
}

TEST(StateExtrapolation, extrapolatesEveryAxisWithItsOwnVelocity)
{
    SCStatesMsgPayload in{};
    for (int i = 0; i < 3; i++) {
        in.r_BN_N[i] = 1.0e6 * (i + 1); // [m]
        in.v_BN_N[i] = 1.0e3 * (i + 1); // [m/s]
    }
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(in, 20000000000ULL, 10000000000ULL, 0);

    for (int i = 0; i < 3; i++) {
        EXPECT_DOUBLE_EQ(out.r_BN_N[i], in.r_BN_N[i] + in.v_BN_N[i] * 5.0);
    }
}

TEST(StateExtrapolation, leavesEverythingExceptThePositionsUnchanged)
{
    SCStatesMsgPayload in = makeState();
    in.sigma_BN[2] = 0.25;         // [-]
    in.omega_BN_B[0] = 1.0e-3;     // [rad/s]
    in.TotalAccumDV_BN_B[1] = 2.5; // [m/s]
    in.MRPSwitchCount = 3;         // [-]
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(in, 10000000000ULL, 0, 0);

    EXPECT_DOUBLE_EQ(out.v_BN_N[1], in.v_BN_N[1]);
    EXPECT_DOUBLE_EQ(out.v_CN_N[1], in.v_CN_N[1]);
    EXPECT_DOUBLE_EQ(out.sigma_BN[2], in.sigma_BN[2]);
    EXPECT_DOUBLE_EQ(out.omega_BN_B[0], in.omega_BN_B[0]);
    EXPECT_DOUBLE_EQ(out.TotalAccumDV_BN_B[1], in.TotalAccumDV_BN_B[1]);
    EXPECT_EQ(out.MRPSwitchCount, in.MRPSwitchCount);
    EXPECT_DOUBLE_EQ(out.r_BN_N[0], in.r_BN_N[0]); // zero x velocity
}

TEST(StateExtrapolation, extrapolatesAMessageWrittenExactlyAtThePreviousUpdate)
{
    // the usual case: the spacecraft wrote its state during the previous step, at the time of the previous module
    // update
    const SCStatesMsgPayload out =
      extrapolateScStateToStepMidpoint(makeState(), 20000000000ULL, 10000000000ULL, 10000000000ULL);

    EXPECT_DOUBLE_EQ(out.r_BN_N[1], 7.5e3 * 5.0);
}

TEST(StateExtrapolation, isNoOpForAMessageOneNanosecondBeforeThePreviousUpdate)
{
    const SCStatesMsgPayload in = makeState();
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(in, 20000000000ULL, 9999999999ULL, 10000000000ULL);

    EXPECT_DOUBLE_EQ(out.r_BN_N[1], in.r_BN_N[1]);
    EXPECT_DOUBLE_EQ(out.r_CN_N[1], in.r_CN_N[1]);
}

TEST(StateExtrapolation, extrapolatesTheMessageOfTheInitialStateAfterReset)
{
    // after Reset the previous update is the start time, and the first state was written at the start time
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(makeState(), 10000000000ULL, 0, 0);

    EXPECT_DOUBLE_EQ(out.r_BN_N[1], 7.5e3 * 5.0);
}

TEST(StateExtrapolation, isNoOpForAMessageWrittenAfterTheCurrentTime)
{
    const SCStatesMsgPayload in = makeState();
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(in, 10000000000ULL, 20000000000ULL, 0);

    EXPECT_DOUBLE_EQ(out.r_BN_N[1], in.r_BN_N[1]);
}

TEST(StateExtrapolation, isNoOpWithZeroVelocityAtAnyAge)
{
    SCStatesMsgPayload in = makeState();
    in.v_BN_N[1] = 0.0;
    in.v_CN_N[1] = 0.0;
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(in, 20000000000ULL, 10000000000ULL, 0);

    EXPECT_DOUBLE_EQ(out.r_BN_N[0], in.r_BN_N[0]);
    EXPECT_DOUBLE_EQ(out.r_BN_N[1], in.r_BN_N[1]);
}

TEST(StateExtrapolation, doesNotModifyItsInput)
{
    const SCStatesMsgPayload in = makeState();
    extrapolateScStateToStepMidpoint(in, 20000000000ULL, 10000000000ULL, 0);

    EXPECT_DOUBLE_EQ(in.r_BN_N[1], 0.0);
    EXPECT_DOUBLE_EQ(in.r_CN_N[1], 0.0);
}
