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

#include "simulation/communication/linkBudget/linkBudget.h"
#include "simulation/communication/simpleAntenna/simpleAntenna.h"
#include "simulation/environment/groundLocation/groundLocation.h"
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <tuple>

namespace {
const double earthRadius = REQ_EARTH * 1.0e3; // [m]
const double frequency = 20.0e9;              // [Hz]
const double bandwidth = 8.0e6;               // [Hz]
const double directivity = 30.0;              // [dBi]
const double efficiency = 0.65;               // [-]
const double transmitPower = 5.0;             // [W]
const double receiverTemperature = 120.0;     // [K]
const double ambientTemperature = 150.0;      // [K]
const double distance = 2000.0e3;             // [m]

/*! @brief Exercise native producers in ground, antenna, then link execution order. */
class AntennaProducerChain : public ::testing::TestWithParam<std::tuple<double, bool, double>>
{
  protected:
    GroundLocation ground;
    SimpleAntenna transmitter;
    SimpleAntenna receiver;
    LinkBudget budget;
    Message<SCStatesMsgPayload> spacecraftState;
    Message<SpicePlanetStateMsgPayload> planetState;
    Eigen::Vector3d expectedNormal_N;
    Eigen::Vector3d expectedGroundPosition_N;
    double pointingOffset = 0.0; // [rad]

    /*! @brief Configure a tilted planet and an independently constructed antenna boresight. */
    void SetUp() override
    {
        const auto [altitude, groundFirst, offset] = GetParam(); // [m, flag, rad]
        pointingOffset = offset;
        ground.planetRadius = earthRadius;
        ground.addSpacecraftToModel(&spacecraftState);
        ground.planetInMsg.subscribeTo(&planetState);
        transmitter.scStateInMsg.subscribeTo(&spacecraftState);
        receiver.groundStateInMsg.subscribeTo(&ground.currentGroundStateOutMsg);
        configureAntenna(transmitter, "space");
        configureAntenna(receiver, "ground");
        transmitter.setAntennaState(AntennaTypes::ANTENNA_TX);
        receiver.setAntennaState(AntennaTypes::ANTENNA_RX);
        budget.antennaInPayload_1.subscribeTo(groundFirst ? &receiver.antennaOutMsg : &transmitter.antennaOutMsg);
        budget.antennaInPayload_2.subscribeTo(groundFirst ? &transmitter.antennaOutMsg : &receiver.antennaOutMsg);
        budget.atmosAtt = true;
        setGeometry(altitude);
    }

    /*! @brief Set identical RF properties without sky-map or ephemeris-file dependencies.
     *  @param antenna Antenna producer to configure.
     *  @param name Antenna name.
     */
    static void configureAntenna(SimpleAntenna& antenna, const char* name)
    {
        antenna.setAntennaName(name);
        antenna.setAntennaFrequency(frequency);
        antenna.setAntennaBandwidth(bandwidth);
        antenna.setAntennaDirectivity_dB(directivity);
        antenna.setAntennaHpbwRatio(1.0); // [-]
        antenna.setAntennaP_Tx(transmitPower);
        antenna.setAntennaP_Rx(5.0); // [W]
        antenna.setAntennaEquivalentNoiseTemp(receiverTemperature);
        antenna.setAntennaRadEfficiency(efficiency);
        antenna.setAntennaEnvironmentTemperature(ambientTemperature);
    }

    /*! @brief Change the ground altitude and publish only the external source states.
     *  @param altitude Ground station altitude in meters.
     */
    void setGeometry(double altitude)
    {
        const double latitude = 0.35;  // [rad]
        const double longitude = -0.6; // [rad]
        const double tilt = 0.4;       // [rad]
        const double elevation = 0.7;  // [rad]
        ground.specifyLocation(latitude, longitude, altitude);
        const Eigen::Matrix3d dcm_PN = Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()).toRotationMatrix();
        SpicePlanetStateMsgPayload planet = {};
        eigenMatrix3d2CArray(dcm_PN, *planet.J20002Pfix);
        planetState.write(&planet, 0, 0);
        const Eigen::Vector3d normal_P(std::cos(latitude) * std::cos(longitude),
                                       std::cos(latitude) * std::sin(longitude),
                                       std::sin(latitude)); // [-]
        expectedNormal_N = dcm_PN.transpose() * normal_P;
        expectedGroundPosition_N = (earthRadius + altitude) * expectedNormal_N;
        const Eigen::Vector3d east_P(-std::sin(longitude), std::cos(longitude), 0.0); // [-]
        const Eigen::Vector3d ray_N =
            std::sin(elevation) * expectedNormal_N + std::cos(elevation) * dcm_PN.transpose() * east_P; // [-]
        const Eigen::Matrix3d aligned_NB =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), -ray_N).toRotationMatrix();
        const Eigen::Matrix3d dcm_NB =
            aligned_NB * Eigen::AngleAxisd(pointingOffset, Eigen::Vector3d::UnitY()).toRotationMatrix();
        const Eigen::MRPd sigma_BN = eigenC2MRP(dcm_NB.transpose());
        SCStatesMsgPayload spacecraft = {};
        eigenVector3d2CArray(expectedGroundPosition_N + distance * ray_N, spacecraft.r_BN_N);
        eigenMRPd2CArray(sigma_BN, spacecraft.sigma_BN);
        spacecraftState.write(&spacecraft, 0, 0);
    }

    /*! @brief Reset in producer order while the ground output is initially unwritten. */
    void resetChain()
    {
        ground.Reset(0);
        transmitter.Reset(0);
        receiver.Reset(0);
        budget.Reset(0);
    }

    /*! @brief Execute one producer-before-consumer tick without a warmup or extra reset. */
    void updateChain()
    {
        ground.UpdateState(0);
        transmitter.UpdateState(0);
        receiver.UpdateState(0);
        budget.UpdateState(0);
    }

    /*! @brief Compare physical outputs with independent geometry and a written-payload link reference. */
    void checkLink()
    {
        const auto [altitude, groundFirst, offset] = GetParam(); // [m, flag, rad]
        auto groundOutput = ground.currentGroundStateOutMsg.addSubscriber()();
        auto receiveOutput = receiver.antennaOutMsg.addSubscriber()();
        auto transmitOutput = transmitter.antennaOutMsg.addSubscriber()();
        EXPECT_NEAR(cArray2EigenVector3d(groundOutput.rHat_LP_N).dot(expectedNormal_N), 1.0, 1.0e-14);
        EXPECT_NEAR((cArray2EigenVector3d(receiveOutput.nHat_LP_N) - expectedNormal_N).norm(), 0.0, 1.0e-14);
        EXPECT_NEAR((cArray2EigenVector3d(receiveOutput.r_AP_N) - expectedGroundPosition_N).norm(), 0.0, 1.0e-8); // [m]
        EXPECT_EQ(receiveOutput.environment, static_cast<uint32_t>(AntennaTypes::ENVIRONMENT_EARTH));
        EXPECT_EQ(transmitOutput.environment, static_cast<uint32_t>(AntennaTypes::ENVIRONMENT_SPACE));
        EXPECT_TRUE(budget.atmosAtt);

        // Supply the reference normal and altitude independently of antenna output fields.
        auto referenceGround = receiveOutput;
        eigenVector3d2CArray(expectedNormal_N, referenceGround.nHat_LP_N);
        eigenVector3d2CArray(expectedGroundPosition_N, referenceGround.r_AP_N);
        Message<AntennaLogMsgPayload> groundMessage;
        Message<AntennaLogMsgPayload> spaceMessage;
        groundMessage.write(&referenceGround, 0, 0);
        spaceMessage.write(&transmitOutput, 0, 0);
        LinkBudget reference;
        reference.atmosAtt = true;
        reference.antennaInPayload_1.subscribeTo(groundFirst ? &groundMessage : &spaceMessage);
        reference.antennaInPayload_2.subscribeTo(groundFirst ? &spaceMessage : &groundMessage);
        reference.Reset(0);
        reference.UpdateState(0);
        ASSERT_GT(reference.getL_atm(), 0.0);                          // [dB]
        EXPECT_NEAR(budget.getL_atm(), reference.getL_atm(), 1.0e-12); // [dB]

        const double beamwidth = std::sqrt(16.0 * std::log(2.0) / std::pow(10.0, directivity / 10.0));       // [rad]
        const double pointingLoss = 40.0 * std::log(2.0) / std::log(10.0) * std::pow(offset / beamwidth, 2); // [dB]
        const double fspl = 20.0 * std::log10(4.0 * MPI * distance * frequency / SPEED_LIGHT);               // [dB]
        const double noise = K_BOLTZMANN *
                             (receiverTemperature + (1.0 - efficiency) * ambientTemperature + efficiency * 200.0) *
                             bandwidth; // [W] Native ground sky is 200 K.
        const double eirp = 10.0 * std::log10(transmitPower * efficiency) + directivity; // [dBW]
        const double received =
            eirp + directivity + 10.0 * std::log10(efficiency) - fspl - pointingLoss - reference.getL_atm(); // [dBW]
        const double cnr = std::pow(10.0, received / 10.0) / noise;                                          // [-]
        EXPECT_NEAR(budget.getL_FSPL(), fspl, 1.0e-10);                                                      // [dB]
        EXPECT_NEAR(budget.getL_point(), pointingLoss, 1.0e-10);                                             // [dB]
        const auto output = budget.linkBudgetOutPayload.addSubscriber()();
        EXPECT_NEAR(groundFirst ? output.CNR1 : output.CNR2, cnr, cnr * 1.0e-10);
        EXPECT_EQ(groundFirst ? output.CNR2 : output.CNR1, 0.0);
    }
};

/*! @brief A real producer chain must work on its first tick, including elevated stations and tilted normals. */
TEST_P(AntennaProducerChain, InitializesFromFirstProducerTick)
{
    resetChain();
    updateChain();
    checkLink();
}

/*! @brief Missing antenna publications must produce zero service until both producers have run. */
TEST_P(AntennaProducerChain, WaitsForBothAntennaMessages)
{
    resetChain();
    EXPECT_NO_THROW(budget.UpdateState(0));
    EXPECT_EQ(budget.linkBudgetOutPayload.addSubscriber()().CNR1, 0.0);
    EXPECT_EQ(budget.linkBudgetOutPayload.addSubscriber()().CNR2, 0.0);
    EXPECT_EQ(budget.linkBudgetOutPayload.addSubscriber()().bandwidth, 0.0); // [Hz]
    ground.UpdateState(0);
    transmitter.UpdateState(0);
    EXPECT_NO_THROW(budget.UpdateState(0));
    EXPECT_TRUE(budget.atmosAtt);
    EXPECT_EQ(budget.getCNR1(), 0.0);
    EXPECT_EQ(budget.getCNR2(), 0.0);
    EXPECT_EQ(budget.linkBudgetOutPayload.addSubscriber()().bandwidth, 0.0); // [Hz]
    receiver.UpdateState(0);
    budget.UpdateState(0);
    checkLink();
}

/*! @brief An antenna that runs before the ground model must not cache sea-level atmosphere. */
TEST_P(AntennaProducerChain, WaitsForGroundPositionWhenAntennaRunsFirst)
{
    resetChain();
    transmitter.UpdateState(0);
    receiver.UpdateState(0);
    ground.UpdateState(0);
    budget.UpdateState(0);
    EXPECT_TRUE(budget.atmosAtt);
    EXPECT_EQ(budget.getL_atm(), 0.0); // [dB]
    EXPECT_EQ(budget.linkBudgetOutPayload.addSubscriber()().CNR1, 0.0);
    EXPECT_EQ(budget.linkBudgetOutPayload.addSubscriber()().CNR2, 0.0);
    EXPECT_EQ(budget.linkBudgetOutPayload.addSubscriber()().bandwidth, 0.0); // [Hz]
    transmitter.UpdateState(0);
    receiver.UpdateState(0);
    ground.UpdateState(0);
    budget.UpdateState(0);
    checkLink();
}

/*! @brief Reset must discard old results and rebuild atmosphere after the station altitude changes. */
TEST_P(AntennaProducerChain, RebuildsAfterStationAltitudeChanges)
{
    resetChain();
    updateChain();
    checkLink();
    const double firstLoss = budget.getL_atm();              // [dB]
    const auto [altitude, groundFirst, offset] = GetParam(); // [m, flag, rad]
    const double nextAltitude = altitude + 2000.0;           // [m]
    setGeometry(nextAltitude);
    resetChain();
    EXPECT_EQ(budget.linkBudgetOutPayload.addSubscriber()().CNR1, 0.0);
    EXPECT_EQ(budget.linkBudgetOutPayload.addSubscriber()().CNR2, 0.0);
    EXPECT_EQ(budget.getL_atm(), 0.0); // [dB]
    updateChain();
    checkLink();
    EXPECT_LT(budget.getL_atm(), firstLoss);
}

INSTANTIATE_TEST_SUITE_P(GroundGeometry,
                         AntennaProducerChain,
                         ::testing::Combine(::testing::Values(0.0, 1624.0), // [m]
                                            ::testing::Bool(),
                                            ::testing::Values(0.0, 0.02))); // [rad]

/*! @brief Written space-to-space inputs must still disable atmosphere and produce a valid link. */
TEST(LinkBudgetProducerInputs, InitializesSpaceToSpaceLink)
{
    AntennaLogMsgPayload first = {};
    first.environment = static_cast<uint32_t>(AntennaTypes::ENVIRONMENT_SPACE);
    first.antennaState = static_cast<uint32_t>(AntennaTypes::ANTENNA_RXTX);
    first.frequency = frequency;
    first.B = bandwidth;
    first.eta_r = efficiency;
    first.P_N = 1.0e-14;    // [W]
    first.P_eirp_dB = 20.0; // [dBW]
    first.DdB = directivity;
    first.r_AN_N[0] = earthRadius + 800.0e3; // [m]
    auto second = first;
    second.r_AN_N[0] += distance;
    Message<AntennaLogMsgPayload> firstMessage;
    Message<AntennaLogMsgPayload> secondMessage;
    firstMessage.write(&first, 0, 0);
    secondMessage.write(&second, 0, 0);
    LinkBudget model;
    model.atmosAtt = true;
    model.pointingLoss = false;
    model.antennaInPayload_1.subscribeTo(&firstMessage);
    model.antennaInPayload_2.subscribeTo(&secondMessage);
    model.Reset(0);
    model.UpdateState(0);
    EXPECT_FALSE(model.atmosAtt);
    EXPECT_EQ(model.getL_atm(), 0.0); // [dB]
    EXPECT_GT(model.getCNR1(), 0.0);
    EXPECT_GT(model.getCNR2(), 0.0);
}

/*! @brief Links without atmosphere do not cache altitude, so they must not wait for a ground position. */
TEST(LinkBudgetProducerInputs, GroundPositionOnlyGatesAtmosphere)
{
    AntennaLogMsgPayload space = {};
    space.environment = static_cast<uint32_t>(AntennaTypes::ENVIRONMENT_SPACE);
    space.antennaState = static_cast<uint32_t>(AntennaTypes::ANTENNA_RXTX);
    space.frequency = frequency;
    space.B = bandwidth;
    space.eta_r = efficiency;
    space.P_N = 1.0e-14;    // [W]
    space.P_eirp_dB = 20.0; // [dBW]
    space.DdB = directivity;
    space.r_AN_N[0] = earthRadius + distance; // [m]
    auto groundAntenna = space;
    groundAntenna.environment = static_cast<uint32_t>(AntennaTypes::ENVIRONMENT_EARTH);
    groundAntenna.r_AN_N[0] = earthRadius; // [m]
    Message<AntennaLogMsgPayload> spaceMessage;
    Message<AntennaLogMsgPayload> groundMessage;
    spaceMessage.write(&space, 0, 0);
    groundMessage.write(&groundAntenna, 0, 0);
    LinkBudget model;
    model.pointingLoss = false;
    model.antennaInPayload_1.subscribeTo(&spaceMessage);
    model.antennaInPayload_2.subscribeTo(&groundMessage);
    model.Reset(0);
    model.UpdateState(0);
    EXPECT_GT(model.getCNR1(), 0.0);
    EXPECT_GT(model.getCNR2(), 0.0);
}
} // namespace
