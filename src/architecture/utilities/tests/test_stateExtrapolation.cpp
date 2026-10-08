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

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "architecture/utilities/astroConstants.h"
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
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(in, 20000000000ULL, 10000000000ULL, 10000000000ULL);

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
    const SCStatesMsgPayload out =
      extrapolateScStateToStepMidpoint(in, 20000000000ULL, 10000000000ULL, 10000000000ULL);

    EXPECT_DOUBLE_EQ(out.r_BN_N[0], in.r_BN_N[0]);
    EXPECT_DOUBLE_EQ(out.r_BN_N[1], in.r_BN_N[1]);
}

TEST(StateExtrapolation, doesNotModifyItsInput)
{
    const SCStatesMsgPayload in = makeState();
    extrapolateScStateToStepMidpoint(in, 20000000000ULL, 10000000000ULL, 10000000000ULL);

    EXPECT_DOUBLE_EQ(in.r_BN_N[1], 0.0);
    EXPECT_DOUBLE_EQ(in.r_CN_N[1], 0.0);
}

TEST(StateExtrapolation, isNoOpForAMessageNewerThanThePreviousUpdate)
{
    // a spacecraft that runs faster than the module: written at 9 s, the module last updated at 0 s and runs at 10 s
    const SCStatesMsgPayload in = makeState();
    const SCStatesMsgPayload out = extrapolateScStateToStepMidpoint(in, 10000000000ULL, 9000000000ULL, 0);

    EXPECT_DOUBLE_EQ(out.r_BN_N[1], in.r_BN_N[1]);
    EXPECT_DOUBLE_EQ(out.r_CN_N[1], in.r_CN_N[1]);
}

namespace {
constexpr uint64_t SECOND_NANOS = 1000000000ULL; // [ns]

/*! Prepare the extrapolation for one spacecraft message and apply it. */
SCStatesMsgPayload
applySingle(ScStateExtrapolation& extrapolation,
            const SCStatesMsgPayload& in,
            uint64_t now,
            uint64_t written,
            uint64_t previousUpdate,
            BSKLogger& logger)
{
    extrapolation.prepare(now, previousUpdate, { written }, logger);
    return extrapolation.apply(in, now, written, previousUpdate);
}

/*! Feed the spacecraft write times seen by a module running at the given period and record which updates were extrapolated. */
void
runModule(ScStateExtrapolation& extrapolation,
          BSKLogger& logger,
          uint64_t modulePeriodNanos,
          uint64_t scPeriodNanos,
          int numUpdates,
          std::vector<bool>& extrapolated,
          bool scRunsFirst = false)
{
    const SCStatesMsgPayload in = makeState();
    uint64_t previousUpdate = 0; // [ns]
    for (int k = 0; k < numUpdates; k++) {
        const uint64_t now = static_cast<uint64_t>(k) * modulePeriodNanos; // [ns]
        // by default the spacecraft runs after the module in each task: its newest message is from the last spacecraft
        // update before the current time. If it runs first, a spacecraft update at the current time is already visible.
        const uint64_t written = scRunsFirst ? (now / scPeriodNanos) * scPeriodNanos
                                             : ((now == 0 ? 0 : now - 1) / scPeriodNanos) * scPeriodNanos; // [ns]
        const SCStatesMsgPayload out = applySingle(extrapolation, in, now, written, previousUpdate, logger);
        extrapolated.push_back(out.r_BN_N[1] != in.r_BN_N[1]);
        previousUpdate = now;
    }
}
} // namespace

TEST(ScStateExtrapolation, matchedTaskPeriodsDoNotWarnAndAreExtrapolated)
{
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    std::vector<bool> extrapolated;
    runModule(extrapolation, logger, 10 * SECOND_NANOS, 10 * SECOND_NANOS, 5, extrapolated);

    EXPECT_FALSE(extrapolation.mismatchDetected());
    // the spacecraft period is only known after two write times: the first two updates are not extrapolated
    EXPECT_FALSE(extrapolated[0]);
    EXPECT_FALSE(extrapolated[1]);
    for (std::size_t k = 2; k < extrapolated.size(); k++) {
        EXPECT_TRUE(extrapolated[k]) << "update " << k;
    }
}

TEST(ScStateExtrapolation, fasterSpacecraftWarnsAndIsNotExtrapolated)
{
    // spacecraft every 1 s, module every 10 s: messages written at 9, 19 and 29 s are read at 10, 20 and 30 s
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    std::vector<bool> extrapolated;
    runModule(extrapolation, logger, 10 * SECOND_NANOS, SECOND_NANOS, 4, extrapolated);

    EXPECT_TRUE(extrapolation.mismatchDetected());
    for (std::size_t k = 1; k < extrapolated.size(); k++) {
        EXPECT_FALSE(extrapolated[k]) << "update " << k;
    }
}

TEST(ScStateExtrapolation, slowerSpacecraftWarnsAndIsNotExtrapolated)
{
    // spacecraft every 10 s, module every 1 s
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    std::vector<bool> extrapolated;
    runModule(extrapolation, logger, SECOND_NANOS, 10 * SECOND_NANOS, 25, extrapolated);

    EXPECT_TRUE(extrapolation.mismatchDetected());
    // Nothing is extrapolated before the spacecraft period is known (k = 1), nor after two successive write times show
    // a spacecraft interval of 10 s that differs from the 1 s module interval (including the update right after each
    // spacecraft write at 21 s, 31 s, ...).
    for (std::size_t k = 0; k < extrapolated.size(); k++) {
        EXPECT_FALSE(extrapolated[k]) << "update " << k;
    }
}

TEST(ScStateExtrapolation, fasterSpacecraftWrittenAtTheCurrentTimeIsNeitherExtrapolatedNorWarned)
{
    // spacecraft every 1 s running before the module, module every 10 s: the message read at 10, 20, ... s was written
    // at the same time, so it needs no extrapolation, and the sampled write times (10 s apart) look like a matched
    // task period, so no warning can be given
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    std::vector<bool> extrapolated;
    runModule(extrapolation, logger, 10 * SECOND_NANOS, SECOND_NANOS, 6, extrapolated, true);

    EXPECT_FALSE(extrapolation.mismatchDetected());
    for (std::size_t k = 0; k < extrapolated.size(); k++) {
        EXPECT_FALSE(extrapolated[k]) << "update " << k;
    }
}

TEST(ScStateExtrapolation, slowerSpacecraftWrittenAtTheCurrentTimeIsNeverExtrapolated)
{
    // spacecraft every 20 s running before the module, module every 10 s: messages written at 0, 0, 20, 20, 40, 40 s
    // are read at 0, 10, 20, 30, 40, 50 s
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    std::vector<bool> extrapolated;
    runModule(extrapolation, logger, 10 * SECOND_NANOS, 20 * SECOND_NANOS, 3, extrapolated, true);

    // up to the update at 20 s the message is read at the time it was written, so there is nothing to warn about
    EXPECT_FALSE(extrapolation.mismatchDetected());

    extrapolated.clear();
    ScStateExtrapolation extrapolationLonger;
    extrapolationLonger.setEnabled(true);
    runModule(extrapolationLonger, logger, 10 * SECOND_NANOS, 20 * SECOND_NANOS, 8, extrapolated, true);

    // the update at 30 s reads the message written at 20 s, and the observed write interval of 20 s differs from the
    // 10 s module interval, so the mismatch is detected from there on
    EXPECT_TRUE(extrapolationLonger.mismatchDetected());
    for (std::size_t k = 0; k < extrapolated.size(); k++) {
        EXPECT_FALSE(extrapolated[k]) << "update " << k;
    }
}

TEST(ScStateExtrapolation, isNotExtrapolatedBeforeTwoWriteTimesAreKnown)
{
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    const SCStatesMsgPayload in = makeState();
    // first observation of a message written at the previous update: the period is unknown
    const SCStatesMsgPayload first = applySingle(extrapolation, in, 20 * SECOND_NANOS, 10 * SECOND_NANOS, 10 * SECOND_NANOS, logger);
    EXPECT_DOUBLE_EQ(first.r_BN_N[1], in.r_BN_N[1]);
    // second write time observed with an interval equal to the module interval
    const SCStatesMsgPayload second = applySingle(extrapolation, in, 30 * SECOND_NANOS, 20 * SECOND_NANOS, 20 * SECOND_NANOS, logger);
    EXPECT_NE(second.r_BN_N[1], in.r_BN_N[1]);
}

TEST(ScStateExtrapolation, staleMessageDoesNotWarn)
{
    // a state written once at the start of the simulation and never rewritten
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    const SCStatesMsgPayload in = makeState();
    for (uint64_t k = 0; k < 5; k++) {
        applySingle(extrapolation, in, k * SECOND_NANOS, 0, k == 0 ? 0 : (k - 1) * SECOND_NANOS, logger);
    }

    EXPECT_FALSE(extrapolation.mismatchDetected());
}

TEST(ScStateExtrapolation, disabledExtrapolationNeverWarnsOrModifiesTheState)
{
    ScStateExtrapolation extrapolation;
    BSKLogger logger;
    std::vector<bool> extrapolated;
    runModule(extrapolation, logger, 10 * SECOND_NANOS, SECOND_NANOS, 4, extrapolated);

    EXPECT_FALSE(extrapolation.mismatchDetected());
    for (bool e : extrapolated) {
        EXPECT_FALSE(e);
    }
}

TEST(PlanetStateExtrapolation, translatingPlanetKeepsTheSeparationAtEveryUpdateIncludingTheStartup)
{
    // The spacecraft and the planet translate together at 30 km/s, 400 km apart radially. The module runs before the
    // spacecraft: at update k it reads the spacecraft written at the previous update and the planet written now.
    const double speed = 3.0e4;       // [m/s]
    const double separation0 = 4.0e5; // [m]
    const uint64_t step = 10 * SECOND_NANOS; // [ns]
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    for (uint64_t k = 0; k < 6; k++) {
        const uint64_t now = k * step;                      // [ns]
        const uint64_t previous = k == 0 ? 0 : (k - 1) * step; // [ns]
        const uint64_t scWritten = previous;                // [ns]
        SCStatesMsgPayload sc = makeState();
        sc.r_BN_N[0] = separation0 + speed * diffNanoToSec(scWritten, 0); // [m]
        sc.v_BN_N[0] = speed;                                             // [m/s]
        SpicePlanetStateMsgPayload planet{};
        planet.PositionVector[0] = speed * diffNanoToSec(now, 0); // [m]
        planet.VelocityVector[0] = speed;                         // [m/s]
        for (int i = 0; i < 3; i++) {
            planet.J20002Pfix[i][i] = 1.0; // [-]
        }
        extrapolation.prepare(now, previous, { scWritten }, logger);
        const SCStatesMsgPayload scOut = extrapolation.apply(sc, now, scWritten, previous);
        const SpicePlanetStateMsgPayload planetOut = extrapolation.applyPlanet(planet, now, now, previous);
        EXPECT_NEAR(scOut.r_BN_N[0] - planetOut.PositionVector[0], separation0, 1e-6) << "update " << k;
    }
}

TEST(PlanetStateExtrapolation, planetIsNotMovedIfTheSpacecraftStateIsNotExtrapolated)
{
    SpicePlanetStateMsgPayload planet{};
    planet.PositionVector[0] = 1.0e7; // [m]
    planet.VelocityVector[0] = 3.0e4; // [m/s]

    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    // message written at 0 s but previous update at 10 s: stale, not extrapolated
    applySingle(extrapolation, makeState(), 20 * SECOND_NANOS, 0, 10 * SECOND_NANOS, logger);
    const SpicePlanetStateMsgPayload out =
      extrapolation.applyPlanet(planet, 20 * SECOND_NANOS, 20 * SECOND_NANOS, 10 * SECOND_NANOS);

    EXPECT_DOUBLE_EQ(out.PositionVector[0], planet.PositionVector[0]);
}

TEST(PlanetStateExtrapolation, spinningPlanetOrientationStaysOrthonormal)
{
    const double spinRate = OMEGA_EARTH; // [rad/s] Earth rotation rate
    SpicePlanetStateMsgPayload planet{};
    planet.J20002Pfix[0][0] = 1.0;       // [-]
    planet.J20002Pfix[1][1] = 1.0;       // [-]
    planet.J20002Pfix[2][2] = 1.0;       // [-]
    // d[PN]/dt = -[omega_P x][PN] with omega along +z: only the 0-1 block is non-zero
    planet.J20002Pfix_dot[0][1] = spinRate;  // [1/s]
    planet.J20002Pfix_dot[1][0] = -spinRate; // [1/s]

    const SpicePlanetStateMsgPayload out = extrapolatePlanetStateToEpoch(planet, 3600 * SECOND_NANOS, 0);

    Eigen::Matrix3d dcm;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            dcm(i, j) = out.J20002Pfix[i][j];
        }
    }
    EXPECT_NEAR((dcm * dcm.transpose() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
    EXPECT_NEAR(dcm(0, 1), std::sin(spinRate * 3600.0), 1e-12);
}

TEST(PlanetStateExtrapolation, spinningPlanetKeepsTheAngularVelocityConsistent)
{
    const double spinRate = OMEGA_EARTH; // [rad/s] Earth rotation rate
    SpicePlanetStateMsgPayload planet{};
    planet.J20002Pfix[0][0] = 1.0;           // [-]
    planet.J20002Pfix[1][1] = 1.0;           // [-]
    planet.J20002Pfix[2][2] = 1.0;           // [-]
    planet.J20002Pfix_dot[0][1] = spinRate;  // [1/s]
    planet.J20002Pfix_dot[1][0] = -spinRate; // [1/s]

    // angular velocity reconstructed as in WindBase::updatePlanetOmegaFromSpice()
    const auto omegaFrom = [](const SpicePlanetStateMsgPayload& p) {
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> C_dot(p.J20002Pfix_dot[0]);
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> C(p.J20002Pfix[0]);
        const Eigen::Matrix3d skewP = -C_dot * C.transpose();
        const Eigen::Vector3d omega_P(skewP(2, 1), skewP(0, 2), skewP(1, 0));
        return Eigen::Vector3d(C.transpose() * omega_P); // [rad/s]
    };

    const SpicePlanetStateMsgPayload out = extrapolatePlanetStateToEpoch(planet, 3600 * SECOND_NANOS, 0);

    EXPECT_NEAR((omegaFrom(out) - omegaFrom(planet)).norm(), 0.0, 1e-15);
}

TEST(PlanetStateExtrapolation, planetIsNotMovedIfTheExtrapolationIsDisabled)
{
    SpicePlanetStateMsgPayload planet{};
    planet.PositionVector[0] = 1.0e7; // [m]
    planet.VelocityVector[0] = 3.0e4; // [m/s]

    ScStateExtrapolation extrapolation;
    BSKLogger logger;
    applySingle(extrapolation, makeState(), 20 * SECOND_NANOS, 10 * SECOND_NANOS, 10 * SECOND_NANOS, logger);
    const SpicePlanetStateMsgPayload out =
      extrapolation.applyPlanet(planet, 20 * SECOND_NANOS, 20 * SECOND_NANOS, 10 * SECOND_NANOS);

    EXPECT_DOUBLE_EQ(out.PositionVector[0], planet.PositionVector[0]);
}

TEST(PlanetStateExtrapolation, resetClearsTheExtrapolatedStateOfThePlanet)
{
    SpicePlanetStateMsgPayload planet{};
    planet.PositionVector[0] = 1.0e7; // [m]
    planet.VelocityVector[0] = 3.0e4; // [m/s]

    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    const uint64_t step = 10 * SECOND_NANOS; // [ns]
    extrapolation.prepare(step, 0, { 0 }, logger); // first write time, the period is known at the next update
    applySingle(extrapolation, makeState(), 2 * step, step, step, logger);
    const SpicePlanetStateMsgPayload moved = extrapolation.applyPlanet(planet, 2 * step, 2 * step, step);
    EXPECT_NE(moved.PositionVector[0], planet.PositionVector[0]);

    extrapolation.reset();
    const SpicePlanetStateMsgPayload out = extrapolation.applyPlanet(planet, 2 * step, 2 * step, step);

    EXPECT_DOUBLE_EQ(out.PositionVector[0], planet.PositionVector[0]);
}

namespace {
/*! Two spacecraft read by one module at 20 s with the previous update at 10 s. */
struct TwoSpacecraftResult
{
    SCStatesMsgPayload first;
    SCStatesMsgPayload second;
    SpicePlanetStateMsgPayload planet;
    bool mismatch;
};

TwoSpacecraftResult runTwoSpacecraft(uint64_t firstWritten, uint64_t secondWritten, const SpicePlanetStateMsgPayload& planet)
{
    const uint64_t now = 20 * SECOND_NANOS;      // [ns]
    const uint64_t previous = 10 * SECOND_NANOS; // [ns]
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    // two earlier updates so that both messages are known to be rewritten by their spacecraft
    extrapolation.prepare(previous, 0, { 0, 0 }, logger);
    extrapolation.prepare(now, previous, { firstWritten, secondWritten }, logger);
    const SCStatesMsgPayload in = makeState();
    return { extrapolation.apply(in, now, firstWritten, previous),
             extrapolation.apply(in, now, secondWritten, previous),
             extrapolation.applyPlanet(planet, now, now, previous),
             extrapolation.mismatchDetected() };
}

SpicePlanetStateMsgPayload
makeMovingPlanet()
{
    SpicePlanetStateMsgPayload planet{};
    planet.PositionVector[0] = 1.0e7; // [m]
    planet.VelocityVector[0] = 3.0e4; // [m/s]
    return planet;
}
} // namespace

TEST(ScStateExtrapolation, allSpacecraftLaggedExtrapolatesBothAndThePlanet)
{
    const SpicePlanetStateMsgPayload planet = makeMovingPlanet();
    const SCStatesMsgPayload in = makeState();
    const TwoSpacecraftResult r = runTwoSpacecraft(10 * SECOND_NANOS, 10 * SECOND_NANOS, planet);

    EXPECT_FALSE(r.mismatch);
    EXPECT_NE(r.first.r_BN_N[1], in.r_BN_N[1]);
    EXPECT_NE(r.second.r_BN_N[1], in.r_BN_N[1]);
    EXPECT_NE(r.planet.PositionVector[0], planet.PositionVector[0]);
}

TEST(ScStateExtrapolation, oneStaleSpacecraftDisablesTheExtrapolationOfBothAndThePlanet)
{
    // the first spacecraft is lagged, the second one was last written at 5 s (not at the previous update, 10 s)
    const SpicePlanetStateMsgPayload planet = makeMovingPlanet();
    const SCStatesMsgPayload in = makeState();
    const TwoSpacecraftResult r = runTwoSpacecraft(10 * SECOND_NANOS, 5 * SECOND_NANOS, planet);

    EXPECT_TRUE(r.mismatch);
    EXPECT_DOUBLE_EQ(r.first.r_BN_N[1], in.r_BN_N[1]);
    EXPECT_DOUBLE_EQ(r.second.r_BN_N[1], in.r_BN_N[1]);
    EXPECT_DOUBLE_EQ(r.planet.PositionVector[0], planet.PositionVector[0]);
}

TEST(ScStateExtrapolation, theOrderOfTheSpacecraftDoesNotChangeTheDecision)
{
    const SpicePlanetStateMsgPayload planet = makeMovingPlanet();
    const SCStatesMsgPayload in = makeState();
    const TwoSpacecraftResult r = runTwoSpacecraft(5 * SECOND_NANOS, 10 * SECOND_NANOS, planet);

    EXPECT_TRUE(r.mismatch);
    EXPECT_DOUBLE_EQ(r.first.r_BN_N[1], in.r_BN_N[1]);
    EXPECT_DOUBLE_EQ(r.second.r_BN_N[1], in.r_BN_N[1]);
    EXPECT_DOUBLE_EQ(r.planet.PositionVector[0], planet.PositionVector[0]);
}

TEST(ScStateExtrapolation, noSpacecraftIsNeverExtrapolated)
{
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    SpicePlanetStateMsgPayload planet{};
    planet.PositionVector[0] = 1.0e9; // [m]
    planet.VelocityVector[0] = 1.0e3; // [m/s]

    extrapolation.prepare(20 * SECOND_NANOS, 10 * SECOND_NANOS, {}, logger);
    const SpicePlanetStateMsgPayload out = extrapolation.applyPlanet(planet, 20 * SECOND_NANOS, 0, 10 * SECOND_NANOS);

    EXPECT_DOUBLE_EQ(out.PositionVector[0], planet.PositionVector[0]);
}

TEST(DcmExtrapolation, advancedRateIsTheRotatedRate)
{
    const double spinRate = OMEGA_EARTH; // [rad/s] Earth rotation rate
    const double dt = 3600.0;            // [s]
    // [NP] of a planet spinning about +z, from the identity orientation
    Eigen::Matrix3d dcm_NPfix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d dcm_NPfix_dot = Eigen::Matrix3d::Zero();
    dcm_NPfix_dot(0, 1) = -spinRate; // [1/s]
    dcm_NPfix_dot(1, 0) = spinRate;  // [1/s]

    const PlanetSpin spin = planetSpin(dcm_NPfix, dcm_NPfix_dot);
    const Eigen::Matrix3d advanced = advanceDcm(dcm_NPfix, spin, dt);
    const Eigen::Matrix3d advancedDot = advanceDcmDot(spin, advanced, dcm_NPfix_dot);

    // the rate of a rotation about +z at the advanced epoch
    EXPECT_NEAR(advancedDot(0, 0), -spinRate * std::sin(spinRate * dt), 1e-15);
    EXPECT_NEAR(advancedDot(0, 1), -spinRate * std::cos(spinRate * dt), 1e-15);
    EXPECT_NEAR(advancedDot(1, 0), spinRate * std::cos(spinRate * dt), 1e-15);
    EXPECT_NEAR(advancedDot(1, 1), -spinRate * std::sin(spinRate * dt), 1e-15);
    // a zero rate stays zero
    const PlanetSpin zeroSpin = planetSpin(dcm_NPfix, Eigen::Matrix3d::Zero());
    EXPECT_TRUE(advanceDcmDot(zeroSpin, advanced, Eigen::Matrix3d::Zero()).isZero());
}

TEST(PlanetStateExtrapolation, positionOnlyPlanetMessageKeepsAValidOrientation)
{
    SpicePlanetStateMsgPayload planet{}; // orientation fields left at their zero value
    planet.PositionVector[0] = 1.0e9;    // [m]
    planet.VelocityVector[0] = 1.0e3;    // [m/s]

    const SpicePlanetStateMsgPayload out = extrapolatePlanetStateToEpoch(planet, 10 * SECOND_NANOS, 0);

    EXPECT_DOUBLE_EQ(out.PositionVector[0], 1.0e9 + 1.0e3 * 10.0);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_DOUBLE_EQ(out.J20002Pfix[i][j], i == j ? 1.0 : 0.0); // [-]
        }
    }
}

TEST(ScStateExtrapolation, applyPlanetTreatsPositionOnlyOrientationAsIdentityOnEveryPath)
{
    SpicePlanetStateMsgPayload planet{}; // orientation fields left at their zero value
    planet.PositionVector[0] = 1.0e9;    // [m]
    planet.VelocityVector[0] = 1.0e3;    // [m/s]
    BSKLogger logger;

    // disabled: the planet is returned as written, with a valid orientation
    ScStateExtrapolation disabled;
    disabled.prepare(20 * SECOND_NANOS, 10 * SECOND_NANOS, { 10 * SECOND_NANOS }, logger);
    const SpicePlanetStateMsgPayload outDisabled =
      disabled.applyPlanet(planet, 20 * SECOND_NANOS, 20 * SECOND_NANOS, 10 * SECOND_NANOS);
    EXPECT_DOUBLE_EQ(outDisabled.PositionVector[0], planet.PositionVector[0]);

    // enabled, dt == 0: the planet is written at the epoch the spacecraft states stay at
    ScStateExtrapolation startup;
    startup.setEnabled(true);
    startup.prepare(20 * SECOND_NANOS, 10 * SECOND_NANOS, { 10 * SECOND_NANOS }, logger);
    const SpicePlanetStateMsgPayload outStartup =
      startup.applyPlanet(planet, 20 * SECOND_NANOS, 10 * SECOND_NANOS, 10 * SECOND_NANOS);
    EXPECT_DOUBLE_EQ(outStartup.PositionVector[0], planet.PositionVector[0]);

    for (const SpicePlanetStateMsgPayload& out : { outDisabled, outStartup }) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                EXPECT_DOUBLE_EQ(out.J20002Pfix[i][j], i == j ? 1.0 : 0.0); // [-]
            }
        }
    }
}

TEST(ScStateExtrapolation, evaluationEpochFollowsTheGeometry)
{
    const uint64_t step = 10 * SECOND_NANOS; // [ns]
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;

    extrapolation.prepare(step, 0, { 0 }, logger); // period not yet known: geometry at the previous update
    EXPECT_EQ(extrapolation.evaluationEpochNanos(step, 0), 0u);

    extrapolation.prepare(2 * step, step, { step }, logger); // period known: geometry at the midpoint
    EXPECT_EQ(extrapolation.evaluationEpochNanos(2 * step, step), step + step / 2);

    ScStateExtrapolation disabled;
    disabled.prepare(2 * step, step, { step }, logger);
    EXPECT_EQ(disabled.evaluationEpochNanos(2 * step, step), 2 * step);
}

namespace {
/*! Stand-in for a ReadFunctor of a spacecraft state message. */
struct FakeScStateMsg
{
    SCStatesMsgPayload payload{};
    uint64_t written = 0; // [ns] time the message was written

    SCStatesMsgPayload operator()() { return this->payload; }
    uint64_t timeWritten() { return this->written; }
};

/*! Stand-in for a ReadFunctor of a planet state message. */
struct FakePlanetMsg
{
    SpicePlanetStateMsgPayload payload{};
    uint64_t written = 0; // [ns] time the message was written

    SpicePlanetStateMsgPayload operator()() { return this->payload; }
    uint64_t timeWritten() { return this->written; }
};
} // namespace

TEST(ScStateExtrapolationMessages, prepareFromMessagesExtrapolatesAllSpacecraftWrittenAtThePreviousUpdate)
{
    const uint64_t step = 10 * SECOND_NANOS; // [ns]
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    std::vector<FakeScStateMsg> msgs(2);
    for (auto& msg : msgs) {
        msg.payload = makeState();
    }

    extrapolation.prepareFromMessages(step, 0, msgs, logger); // first write time observed
    msgs[0].written = msgs[1].written = step;
    extrapolation.prepareFromMessages(2 * step, step, msgs, logger); // period known

    for (auto& msg : msgs) {
        const SCStatesMsgPayload out = extrapolation.applyMessage(msg, 2 * step, step);
        EXPECT_DOUBLE_EQ(out.r_BN_N[1], 7.5e3 * 5.0); // v * age / 2
    }
    EXPECT_FALSE(extrapolation.mismatchDetected());
}

TEST(ScStateExtrapolationMessages, prepareFromMessagesMatchesPrepareWithTheWriteTimes)
{
    const uint64_t step = 10 * SECOND_NANOS; // [ns]
    std::vector<FakeScStateMsg> msgs(2);
    msgs[0].written = step;
    msgs[1].written = 0; // stale: disables the extrapolation of both

    ScStateExtrapolation fromMessages;
    fromMessages.setEnabled(true);
    ScStateExtrapolation fromTimes;
    fromTimes.setEnabled(true);
    BSKLogger loggerMessages;
    BSKLogger loggerTimes;
    for (uint64_t now : { step, 2 * step }) {
        fromMessages.prepareFromMessages(now, now - step, msgs, loggerMessages);
        fromTimes.prepare(now, now - step, { msgs[0].written, msgs[1].written }, loggerTimes);
    }

    EXPECT_EQ(fromMessages.mismatchDetected(), fromTimes.mismatchDetected());
    for (auto& msg : msgs) {
        msg.payload = makeState();
        const SCStatesMsgPayload expected = fromTimes.apply(msg.payload, 2 * step, msg.written, step);
        const SCStatesMsgPayload out = fromMessages.applyMessage(msg, 2 * step, step);
        EXPECT_DOUBLE_EQ(out.r_BN_N[1], expected.r_BN_N[1]);
        EXPECT_DOUBLE_EQ(out.r_BN_N[1], msg.payload.r_BN_N[1]); // not extrapolated
    }
}

TEST(ScStateExtrapolationMessages, prepareFromMessageHandlesASingleSpacecraft)
{
    const uint64_t step = 10 * SECOND_NANOS; // [ns]
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    FakeScStateMsg msg;
    msg.payload = makeState();

    extrapolation.prepareFromMessage(step, 0, msg, logger);
    msg.written = step;
    extrapolation.prepareFromMessage(2 * step, step, msg, logger);

    const SCStatesMsgPayload out = extrapolation.applyMessage(msg, 2 * step, step);
    EXPECT_DOUBLE_EQ(out.r_BN_N[1], 7.5e3 * 5.0);
    EXPECT_EQ(extrapolation.evaluationEpochNanos(2 * step, step), step + step / 2);
}

TEST(ScStateExtrapolationMessages, applyMessageDoesNotExtrapolateWhenDisabled)
{
    const uint64_t step = 10 * SECOND_NANOS; // [ns]
    ScStateExtrapolation extrapolation;
    BSKLogger logger;
    FakeScStateMsg msg;
    msg.payload = makeState();
    msg.written = step;

    extrapolation.prepareFromMessage(2 * step, step, msg, logger);

    const SCStatesMsgPayload out = extrapolation.applyMessage(msg, 2 * step, step);
    EXPECT_DOUBLE_EQ(out.r_BN_N[1], msg.payload.r_BN_N[1]);
}

TEST(ScStateExtrapolationMessages, applyPlanetMessageMovesThePlanetLikeApplyPlanet)
{
    const uint64_t step = 10 * SECOND_NANOS; // [ns]
    ScStateExtrapolation extrapolation;
    extrapolation.setEnabled(true);
    BSKLogger logger;
    FakeScStateMsg scMsg;
    scMsg.payload = makeState();
    FakePlanetMsg planetMsg;
    planetMsg.payload = makeMovingPlanet();
    planetMsg.written = 2 * step;

    extrapolation.prepareFromMessage(step, 0, scMsg, logger);
    scMsg.written = step;
    extrapolation.prepareFromMessage(2 * step, step, scMsg, logger);

    const SpicePlanetStateMsgPayload out = extrapolation.applyPlanetMessage(planetMsg, 2 * step, step);
    const SpicePlanetStateMsgPayload expected =
      extrapolation.applyPlanet(planetMsg.payload, 2 * step, planetMsg.written, step);
    EXPECT_DOUBLE_EQ(out.PositionVector[0], expected.PositionVector[0]);
    EXPECT_DOUBLE_EQ(out.PositionVector[0], 1.0e7 - 3.0e4 * 5.0); // moved back to the step midpoint
}
