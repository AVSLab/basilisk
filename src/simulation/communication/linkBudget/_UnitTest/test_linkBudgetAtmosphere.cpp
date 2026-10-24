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
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <tuple>

/*! @brief Exercise private atmospheric calculations without adding a public module API. */
class LinkBudgetTestAccess
{
  public:
    /*! @brief Build the actual altitude grid using the dry-air continuum without spectral lines.
     *  @param model Module under test.
     *  @param altitude Ground altitude in meters.
     *  @return Precomputed layer data owned by the module.
     */
    static const AttenuationLookupTable& buildLayers(LinkBudget& model, double altitude)
    {
        model.h_0 = altitude;
        const double frequency = 10.0e9; // [Hz]
        model.precomputeAtmosphericAttenuationAtLayers(&model.attenuationLookup, frequency);
        return model.attenuationLookup;
    }

    /*! @brief Integrate a synthetic atmosphere for comparison with an analytic path length.
     *  @param model Module under test.
     *  @param altitude Ground altitude in meters.
     *  @param elevation Ground elevation in radians.
     *  @param constantElevation Whether to vary refractive index to keep elevation constant.
     *  @return Atmospheric attenuation in decibels.
     */
    static double syntheticLoss(LinkBudget& model, double altitude, double elevation, bool constantElevation)
    {
        model.h_0 = altitude;
        model.n_0 = 1.02; // [-] Synthetic refractive index, remaining above unity throughout the column
        model.antennaIn_1 = {};
        model.antennaIn_2 = {};
        model.gndAntPnt = &model.antennaIn_1;
        model.scAntPnt = &model.antennaIn_2;
        const double radius = REQ_EARTH * 1e3 + altitude; // [m]
        const double distance = 2000.0e3;                 // [m]
        model.antennaIn_1.r_AN_N[0] = radius;
        model.antennaIn_1.nHat_LP_N[0] = 1.0; // [-]
        model.antennaIn_2.r_AN_N[0] = radius + distance * std::sin(elevation);
        model.antennaIn_2.r_AN_N[1] = distance * std::cos(elevation);

        const std::size_t layerCount = 4000;
        const double thickness = (model.h_Toa - altitude) / layerCount; // [m]
        model.attenuationLookup = {};
        for (std::size_t i = 0; i < layerCount; ++i) {
            const double height = altitude + (static_cast<double>(i) + 0.5) * thickness; // [m]
            model.attenuationLookup.layer_i.push_back(height);
            model.attenuationLookup.delta_h_layer_i.push_back(thickness);
            const double attenuation = 0.01; // [dB/km] Uniform specific attenuation
            model.attenuationLookup.gamma_i.push_back(attenuation);
            // With n proportional to 1/r, Snell's invariant keeps the elevation constant.
            model.attenuationLookup.n_i.push_back(constantElevation ? model.n_0 * radius / (REQ_EARTH * 1e3 + height)
                                                                    : model.n_0);
        }
        model.calculateAtmosphericLoss();
        return model.getL_atm();
    }
};

namespace {
constexpr double topAltitude = 100.0e3;      // [m]
constexpr double altitudeTolerance = 1.0e-8; // [m]

class LinkBudgetLayerGrid : public ::testing::TestWithParam<double>
{};

/*! @brief Check finite layers, midpoint sampling, exponential growth and complete altitude coverage. */
TEST_P(LinkBudgetLayerGrid, CoversEntireAtmosphere)
{
    LinkBudget model;
    const double groundAltitude = GetParam(); // [m]
    const auto& layers = LinkBudgetTestAccess::buildLayers(model, groundAltitude);
    ASSERT_FALSE(layers.layer_i.empty());
    ASSERT_LE(layers.layer_i.size(), 922U);
    ASSERT_EQ(layers.layer_i.size(), layers.delta_h_layer_i.size());
    ASSERT_EQ(layers.layer_i.size(), layers.gamma_i.size());
    ASSERT_EQ(layers.layer_i.size(), layers.n_i.size());
    if (groundAltitude == 0.0) {
        EXPECT_EQ(layers.layer_i.size(), 922U);
    }

    double previousTop = groundAltitude; // [m]
    double totalThickness = 0.0;         // [m]
    for (std::size_t i = 0; i < layers.layer_i.size(); ++i) {
        SCOPED_TRACE(i);
        const double height = layers.layer_i[i];            // [m]
        const double thickness = layers.delta_h_layer_i[i]; // [m]
        ASSERT_TRUE(std::isfinite(height));
        ASSERT_TRUE(std::isfinite(thickness));
        ASSERT_TRUE(std::isfinite(layers.gamma_i[i]));
        ASSERT_TRUE(std::isfinite(layers.n_i[i]));
        EXPECT_GT(thickness, 0.0);
        EXPECT_GE(layers.gamma_i[i], 0.0);
        EXPECT_GE(layers.n_i[i], 1.0);
        EXPECT_NEAR(height - 0.5 * thickness, previousTop, altitudeTolerance);
        previousTop = height + 0.5 * thickness;
        totalThickness += thickness;
        if (i > 0) {
            EXPECT_GT(height, layers.layer_i[i - 1]);
            EXPECT_NEAR(thickness / layers.delta_h_layer_i[i - 1], std::exp(0.01), 1.0e-8);
        }
    }
    EXPECT_NEAR(previousTop, topAltitude, altitudeTolerance);
    EXPECT_NEAR(totalThickness, topAltitude - groundAltitude, altitudeTolerance);
}

// [m] Sea level, elevated ground, the old overflow region and the last atmospheric meter.
INSTANTIATE_TEST_SUITE_P(Altitudes, LinkBudgetLayerGrid, ::testing::Values(0.0, 1500.0, 25000.0, 80000.0, 99999.0));

/*! @brief Rebuilding replaces cached layers and clears them at or above the atmosphere. */
TEST(LinkBudgetAtmosphere, RebuildingReplacesLayers)
{
    LinkBudget model;
    const auto original = LinkBudgetTestAccess::buildLayers(model, 0.0);  // [m]
    const auto& repeated = LinkBudgetTestAccess::buildLayers(model, 0.0); // [m]
    EXPECT_EQ(original.layer_i, repeated.layer_i);
    EXPECT_EQ(original.delta_h_layer_i, repeated.delta_h_layer_i);
    EXPECT_EQ(original.gamma_i, repeated.gamma_i);
    EXPECT_EQ(original.n_i, repeated.n_i);
    for (double altitude : { 100000.0, 110000.0 }) {   // [m]
        LinkBudgetTestAccess::buildLayers(model, 0.0); // [m]
        const auto& empty = LinkBudgetTestAccess::buildLayers(model, altitude);
        EXPECT_TRUE(empty.layer_i.empty());
        EXPECT_TRUE(empty.delta_h_layer_i.empty());
        EXPECT_TRUE(empty.gamma_i.empty());
        EXPECT_TRUE(empty.n_i.empty());
    }
}

class LinkBudgetRayPath : public ::testing::TestWithParam<std::tuple<double, double>>
{};

/*! @brief Uniform refractive index gives the analytic straight-line chord through a spherical shell. */
TEST_P(LinkBudgetRayPath, MatchesSphericalShellChord)
{
    const auto [altitude, elevationDegrees] = GetParam(); // [m, deg]
    const double elevation = elevationDegrees * D2R;      // [rad]
    LinkBudget model;
    const double actual = LinkBudgetTestAccess::syntheticLoss(model, altitude, elevation, false); // [dB]
    const double radius = REQ_EARTH * 1e3 + altitude;                                             // [m]
    const double height = topAltitude - altitude;                                                 // [m]
    const double verticalProjection = radius * std::sin(elevation);                               // [m]
    const double radiusDifferenceSquared = height * (2.0 * radius + height);                      // [m^2]
    const double pathLength =
        radiusDifferenceSquared /
        (std::sqrt(verticalProjection * verticalProjection + radiusDifferenceSquared) + verticalProjection); // [m]
    const double specificAttenuation = 0.01;                                                                 // [dB/km]
    const double expected = specificAttenuation * pathLength * 1e-3;                                         // [dB]
    EXPECT_NEAR(actual, expected, expected * 1.0e-7);
}

/*! @brief An inverse-radius refractive index yields a constant elevation and an exact cosecant path. */
TEST_P(LinkBudgetRayPath, AccountsForRefractiveIndex)
{
    const auto [altitude, elevationDegrees] = GetParam(); // [m, deg]
    const double elevation = elevationDegrees * D2R;      // [rad]
    LinkBudget model;
    const double actual = LinkBudgetTestAccess::syntheticLoss(model, altitude, elevation, true);         // [dB]
    const double specificAttenuation = 0.01;                                                             // [dB/km]
    const double expected = specificAttenuation * (topAltitude - altitude) * 1e-3 / std::sin(elevation); // [dB]
    EXPECT_NEAR(actual, expected, expected * 1.0e-11);
}

INSTANTIATE_TEST_SUITE_P(Geometry,
                         LinkBudgetRayPath,
                         ::testing::Combine(::testing::Values(0.0, 1500.0),              // [m]
                                            ::testing::Values(10.0, 30.0, 60.0, 90.0))); // [deg]
} // namespace
