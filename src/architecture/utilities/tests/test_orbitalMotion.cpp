/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "unitTestComparators.h"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <tuple>

const double orbitalEnvironmentAccuracy = 1e-10;
const double orbitalElementsAccuracy = 1e-11;

/** @brief Match direct trigonometric references without reducing by a rounded 2*pi divisor. */
TEST(OrbitalMotion, largeWrappedAnomalies)
{
    const double anomalies[] = { 1e6, -1e6, 1e10, -1e10, 1e16, -1e16 }; // [rad]
    for (double anomaly : anomalies) {
        for (double eccentricity : { 0.0, 0.4, 1.0, 1.5 }) {
            ClassicElements elements = {};
            elements.e = eccentricity;
            elements.rPeriap = 7000.0;                                                        // [km]
            elements.a = eccentricity == 1.0 ? 0.0 : elements.rPeriap / (1.0 - eccentricity); // [km]
            elements.f = anomaly;
            const double parameter = elements.rPeriap * (1.0 + eccentricity);
            const double radius = parameter / (1.0 + eccentricity * std::cos(anomaly));
            const double speed = std::sqrt(MU_EARTH / parameter);
            const double expectedR[3] = { radius * std::cos(anomaly), radius * std::sin(anomaly), 0.0 }; // [km]
            const double expectedV[3] = { -speed * std::sin(anomaly),
                                          speed * (eccentricity + std::cos(anomaly)),
                                          0.0 }; // [km/s]
            double position[3], velocity[3];
            elem2rv(MU_EARTH, &elements, position, velocity);
            for (int index = 0; index < 3; ++index) {
                EXPECT_NEAR(position[index], expectedR[index], 5e-15 * radius); // [km]
                EXPECT_NEAR(velocity[index], expectedV[index], 5e-15 * speed);  // [km/s]
            }
        }
        ClassicElements radial = {};
        radial.a = 7500.0; // [km]
        radial.e = 1.0;
        radial.f = anomaly;
        const double radius = radial.a * (1.0 - std::cos(anomaly));
        const double speed = std::sqrt(MU_EARTH / radial.a) * std::sin(anomaly) / (1.0 - std::cos(anomaly));
        double position[3], velocity[3];
        elem2rv(MU_EARTH, &radial, position, velocity);
        EXPECT_NEAR(position[0], radius, 5e-15 * radius);         // [km]
        EXPECT_NEAR(velocity[0], speed, 5e-15 * std::abs(speed)); // [km/s]
        EXPECT_EQ(position[1], 0.0);
        EXPECT_EQ(position[2], 0.0);
        EXPECT_EQ(velocity[1], 0.0);
        EXPECT_EQ(velocity[2], 0.0);
    }
}

class EllipticKeplerTests : public ::testing::TestWithParam<std::tuple<double, double, double>>
{};

/** @brief Recover elliptic roots where Newton iteration diverged or lost precision. */
TEST_P(EllipticKeplerTests, convergence)
{
    const auto [mean, eccentricity, expected] = GetParam();
    for (double sign : { -1.0, 1.0 }) {
        const double result = M2E(sign * mean, eccentricity);
        ASSERT_TRUE(std::isfinite(result));
        EXPECT_NEAR(result, sign * expected, 5e-14 * expected);
    }
}

// Reference roots from 90-digit Decimal bisection using exact binary64 inputs.
// Each tuple contains mean anomaly [rad], eccentricity [-], and root [rad].
INSTANTIATE_TEST_SUITE_P(OrbitalMotion,
                         EllipticKeplerTests,
                         ::testing::Values(std::make_tuple(0.3, 0.999, 1.247126572242462),
                                           std::make_tuple(0.1, 0.9999, 0.8535302901646385),
                                           std::make_tuple(0.10008, 0.99, 0.8319005379955369),
                                           std::make_tuple(1e-12, 1.0 - 1e-12, 0.0001817010532025818),
                                           std::make_tuple(1e-24, std::nextafter(1.0, 0.0), 8.18424690685419e-9),
                                           std::make_tuple(2.0 * M_PI, std::nextafter(1.0, 0.0), 6.28317393797836)));

class HyperbolicKeplerTests : public ::testing::TestWithParam<std::tuple<double, double, double>>
{};

/** @brief Recover hyperbolic roots without overflow or iteration-limit fall-through. */
TEST_P(HyperbolicKeplerTests, convergence)
{
    const auto [mean, eccentricity, expected] = GetParam();
    for (double sign : { -1.0, 1.0 }) {
        const double result = N2H(sign * mean, eccentricity);
        ASSERT_TRUE(std::isfinite(result));
        EXPECT_NEAR(result, sign * expected, 5e-14 * expected);
    }
}

// Each tuple contains mean anomaly [rad], eccentricity [-], and reference root [rad].
INSTANTIATE_TEST_SUITE_P(OrbitalMotion,
                         HyperbolicKeplerTests,
                         ::testing::Values(std::make_tuple(250.0, 1.5, 5.832212476255278),
                                           std::make_tuple(1000.0, 1.5, 7.202614705676229),
                                           std::make_tuple(1e6, 1.5, 14.103206733523901),
                                           std::make_tuple(1e-12, 1.0 + 1e-12, 0.0001817010517805506),
                                           std::make_tuple(1e-24, std::nextafter(1.0, 2.0), 4.437990012889989e-9),
                                           std::make_tuple(1e300, 1.5, 691.0632099706655),
                                           std::make_tuple(1e308, 1e308, 0.881373587019543)));

/** @brief Preserve unwrapped anomalies and satisfy the elliptic Kepler equation. */
TEST(OrbitalMotion, keplerRevolutions)
{
    const double eccentricities[] = { 0.0, 0.5, 0.99, std::nextafter(1.0, 0.0) }; // [-]
    for (double eccentricity : eccentricities) {
        for (int turns : { -100, -1, 0, 1, 100 }) {
            const double mean = 0.3 + turns * 2.0 * M_PI; // [rad]
            const double result = M2E(mean, eccentricity);
            const double expected = M2E(0.3, eccentricity) + turns * 2.0 * M_PI; // [rad]
            EXPECT_NEAR(result, expected, 5e-13);                                // [rad]
            EXPECT_NEAR(result - eccentricity * std::sin(result), mean, 5e-13);  // [rad]
        }
    }
    EXPECT_DOUBLE_EQ(M2E(0.0, 0.999), 0.0);
    EXPECT_DOUBLE_EQ(N2H(0.0, 1.001), 0.0);
}

/** @brief Return NaN for invalid conic domains and non-finite input. */
TEST(OrbitalMotion, keplerInvalidInputs)
{
    const double nonfinite[] = { std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity() };
    for (double value : nonfinite) {
        EXPECT_TRUE(std::isnan(M2E(value, 0.5)));
        EXPECT_TRUE(std::isnan(N2H(value, 1.5)));
        EXPECT_TRUE(std::isnan(M2E(0.3, value)));
        EXPECT_TRUE(std::isnan(N2H(0.3, value)));
    }
    EXPECT_TRUE(std::isnan(M2E(0.3, -0.1)));
    EXPECT_TRUE(std::isnan(M2E(0.3, 1.0)));
    EXPECT_TRUE(std::isnan(N2H(0.3, 0.5)));
    EXPECT_TRUE(std::isnan(N2H(0.3, 1.0)));
}

class SingularOrbitTests : public ::testing::TestWithParam<std::tuple<std::tuple<double, double>, double, double>>
{};

/** @brief Check canonical angles, invariants, and state reconstruction across consistent distance units. */
TEST_P(SingularOrbitTests, rv2elem)
{
    const auto [orbit, lengthScale, trueAnomaly] = GetParam();
    const auto [eccentricity, inclination] = orbit;
    const double semimajor = 7500.0 * lengthScale;                        // [L]; L is the selected distance unit
    const double mu = MU_EARTH * lengthScale * lengthScale * lengthScale; // [L^3/s^2]
    const double node = 1.1;                                              // [rad]
    const double periapsis = 0.7;                                         // [rad]
    const double sinI = inclination == 0.0 || inclination == M_PI ? 0.0 : std::sin(inclination);
    const double cosI = std::cos(inclination);
    const double ascending[3] = { std::cos(node), std::sin(node), 0.0 };                  // [-]
    const double transverse[3] = { -std::sin(node) * cosI, std::cos(node) * cosI, sinI }; // [-]
    const double parameter = semimajor * (1.0 - eccentricity * eccentricity);
    const double radius = parameter / (1.0 + eccentricity * std::cos(trueAnomaly));
    double position[3]; // [L]
    double velocity[3]; // [L/s]
    // Construct the state independently from orthonormal perifocal axes.
    for (int index = 0; index < 3; ++index) {
        const double periapsisAxis = std::cos(periapsis) * ascending[index] + std::sin(periapsis) * transverse[index];
        const double quadratureAxis = -std::sin(periapsis) * ascending[index] + std::cos(periapsis) * transverse[index];
        position[index] = radius * (std::cos(trueAnomaly) * periapsisAxis + std::sin(trueAnomaly) * quadratureAxis);
        velocity[index] = std::sqrt(mu / parameter) * (-std::sin(trueAnomaly) * periapsisAxis +
                                                       (eccentricity + std::cos(trueAnomaly)) * quadratureAxis);
    }
    ClassicElements result;
    rv2elem(mu, position, velocity, &result);
    const bool equatorial = std::fabs(sinI) <= 1e-12;
    const bool circular = eccentricity <= 1e-12;
    const double direction = cosI >= 0.0 ? 1.0 : -1.0;
    const double expectedI = equatorial ? (direction > 0.0 ? 0.0 : M_PI) : inclination;                  // [rad]
    const double expectedNode = equatorial ? 0.0 : node;                                                 // [rad]
    const double expectedPeriapsis = circular ? 0.0 : periapsis + (equatorial ? direction * node : 0.0); // [rad]
    const double expectedAnomaly = trueAnomaly + (circular ? periapsis + (equatorial ? direction * node : 0.0) : 0.0);
    // Individual periapsis and anomaly angles become ill-conditioned as eccentricity tends to zero.
    const double angleTolerance = circular ? 3e-14 : std::fmax(3e-14, 5e-15 / eccentricity); // [rad]
    for (double value : { result.a,
                          result.e,
                          result.i,
                          result.Omega,
                          result.omega,
                          result.f,
                          result.alpha,
                          result.rmag,
                          result.rPeriap,
                          result.rApoap }) {
        ASSERT_TRUE(std::isfinite(value));
    }
    EXPECT_NEAR(result.a / semimajor, 1.0, 2e-14);
    EXPECT_NEAR(result.alpha * semimajor, 1.0, 2e-14);
    EXPECT_NEAR(result.e, eccentricity, 2e-15);
    EXPECT_NEAR(result.rmag / radius, 1.0, 2e-14);
    EXPECT_NEAR(result.rPeriap / semimajor, 1.0 - eccentricity, 2e-14);
    EXPECT_NEAR(result.rApoap / semimajor, 1.0 + eccentricity, 2e-14);
    EXPECT_NEAR(result.i, expectedI, 2e-15);                                          // [rad]
    EXPECT_NEAR(std::remainder(result.Omega - expectedNode, 2.0 * M_PI), 0.0, 3e-14); // [rad]
    EXPECT_NEAR(std::remainder(result.omega - expectedPeriapsis, 2.0 * M_PI), 0.0, angleTolerance);
    EXPECT_NEAR(std::remainder(result.f - expectedAnomaly, 2.0 * M_PI), 0.0, angleTolerance);
    for (double angle : { result.Omega, result.omega, result.f }) {
        EXPECT_GE(angle, 0.0);
        EXPECT_LT(angle, 2.0 * M_PI);
    }
    double rebuiltPosition[3]; // [L]
    double rebuiltVelocity[3]; // [L/s]
    elem2rv(mu, &result, rebuiltPosition, rebuiltVelocity);
    const double speedScale = std::sqrt(mu / semimajor);
    for (int index = 0; index < 3; ++index) {
        EXPECT_NEAR(rebuiltPosition[index] / semimajor, position[index] / semimajor, 5e-12);
        EXPECT_NEAR(rebuiltVelocity[index] / speedScale, velocity[index] / speedScale, 5e-12);
    }
}

// Orbit tuples: eccentricity [-], inclination [rad]; scales [-]; true anomalies [rad].
INSTANTIATE_TEST_SUITE_P(OrbitalMotion,
                         SingularOrbitTests,
                         ::testing::Combine(::testing::Values(std::make_tuple(0.0, 0.0),
                                                              std::make_tuple(0.0, M_PI),
                                                              std::make_tuple(0.3, 0.0),
                                                              std::make_tuple(0.3, M_PI),
                                                              std::make_tuple(0.0, 0.7),
                                                              std::make_tuple(0.0, 2.4),
                                                              std::make_tuple(0.3, 2.4),
                                                              std::make_tuple(0.0, 1e-8),
                                                              std::make_tuple(0.3, 1e-8),
                                                              std::make_tuple(0.0, M_PI - 1e-8),
                                                              std::make_tuple(0.3, M_PI - 1e-8),
                                                              std::make_tuple(0.3, 5e-13),
                                                              std::make_tuple(0.3, 2e-12),
                                                              std::make_tuple(0.3, M_PI - 5e-13),
                                                              std::make_tuple(0.3, M_PI - 2e-12),
                                                              std::make_tuple(5e-13, 0.7),
                                                              std::make_tuple(2e-12, 0.7),
                                                              std::make_tuple(5e-13, M_PI),
                                                              std::make_tuple(2e-12, M_PI)),
                                            ::testing::Values(1e-9, 1.0, 1e3),
                                            ::testing::Values(0.4, 4.6)));

TEST(OrbitalMotion, atmosphericDensity_200km)
{
    double alt = 200.0;
    double result = atmosphericDensity(alt);
    EXPECT_NEAR(result, 1.64100656241e-10, orbitalEnvironmentAccuracy);
}

TEST(OrbitalMotion, atmosphericDensity_2000km)
{
    double alt = 2000.0;
    double result = atmosphericDensity(alt);
    EXPECT_NEAR(result, 2.48885731828e-15, orbitalEnvironmentAccuracy);
}

class DebyeLengthTests : public ::testing::TestWithParam<std::tuple<double, double>>
{};

/** @brief Check table interpolation and both boundaries of the constant Debye-length region. */
TEST_P(DebyeLengthTests, checksDebyeLengthAtAltitude)
{
    auto [altitude, expected] = GetParam();
    EXPECT_NEAR(expected, debyeLength(altitude), orbitalEnvironmentAccuracy);
}

// Each tuple contains altitude [km] and expected Debye length [m].
INSTANTIATE_TEST_SUITE_P(OrbitalMotion,
                         DebyeLengthTests,
                         ::testing::Values(std::make_tuple(200.0, 0.00564),
                                           std::make_tuple(225.0, 0.00478),
                                           std::make_tuple(400.0, 0.00404),
                                           std::make_tuple(1000.0, 0.0159),
                                           std::make_tuple(1775.0, 0.03585),
                                           std::make_tuple(1950.0, 0.0396),
                                           std::make_tuple(1999.0, 0.0396),
                                           std::make_tuple(2000.0, 0.0396),
                                           std::make_tuple(2001.0, 0.0396),
                                           std::make_tuple(10000.0, 0.0396),
                                           std::make_tuple(29999.0, 0.0396),
                                           std::make_tuple(30000.0, 0.0396),
                                           std::make_tuple(30001.0, 0.4),
                                           std::make_tuple(34000.0, 400.3),
                                           std::make_tuple(35000.0, 500.3)));

/** @brief Check that non-finite and out-of-range altitudes return NaN. */
TEST(OrbitalMotion, debyeLengthInvalidAltitudes)
{
    const double invalidAltitudes[] = { // [km]
                                        199.0,
                                        35001.0,
                                        std::numeric_limits<double>::infinity(),
                                        -std::numeric_limits<double>::infinity(),
                                        std::numeric_limits<double>::quiet_NaN()
    };
    for (double altitude : invalidAltitudes) {
        SCOPED_TRACE(altitude);
        EXPECT_TRUE(std::isnan(debyeLength(altitude)));
    }
}

TEST(OrbitalMotion, atmosphericDrag)
{
    double check[3];
    double ans[3];
    double r[3] = { 6200.0, 100.0, 2000.0 };
    double v[3] = { 1.0, 9.0, 1.0 };
    double A = 2.0;
    double Cd = 0.2;
    double m = 50.0;

    v3Set(-2.8245395411253663e-007, -2.5420855870128297e-006, -2.8245395411253663e-007, check);
    atmosphericDrag(Cd, A, m, r, v, ans);
    EXPECT_VECTOR3_NEAR(ans, check, 11); // [km/s^2]
}

TEST(OrbitalMotion, jPerturb_order_6)
{
    double check[3];
    double ans[3];
    double r[3] = { 6200.0, 100.0, 2000.0 };
    int order = 6;

    v3Set(-7.3080959003487213e-006, -1.1787251452175358e-007, -1.1381118473672282e-005, check);
    jPerturb(r, order, ans);
    EXPECT_VECTOR3_NEAR(ans, check, 11); // [km/s^2]
}

TEST(OrbitalMotion, solarRadiationPressure)
{
    double check[3];
    double ans[3];
    double A = 2.0;
    double m = 50.0;
    double r[3] = { 1.0, 0.3, -0.2 };

    v3Set(-1.9825487816e-10, -5.94764634479e-11, 3.96509756319e-11, check);
    solarRad(A, m, r, ans);
    EXPECT_VECTOR3_NEAR(ans, check, 11); // [km/s^2]
}

TEST(OrbitalMotion, elem2rv1DEccentric)
{
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;
    elements.a = 7500.0;
    elements.e = 1.0;
    elements.i = 40.0 * D2R;
    elements.Omega = 133.0 * D2R;
    elements.omega = 113.0 * D2R;
    elements.f = 23.0 * D2R;
    elements.alpha = 1.0 / elements.a;
    elements.rPeriap = elements.a * (1. - elements.e);
    elements.rApoap = elements.a * (1. + elements.e);
    elem2rv(MU_EARTH, &elements, r, v);
    v3Set(-148.596902253492, -457.100381534593, 352.773096481799, r2);
    v3Set(-8.93065944520745, -27.4716886950712, 21.2016289595043, v3_2);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST(OrbitalMotion, parabolicLimit)
{
    for (double scale : { 1e-6, 1.0, 1e3 }) {
        const double mu = MU_EARTH * scale * scale * scale; // [L^3/s^2]
        const double periapsis = 7500.0 * scale;            // [L]
        for (double eccentricity : { 1.0 - 1e-10, 1.0 - 5e-13, 1.0, 1.0 + 5e-13, 1.0 + 1e-10 }) {
            for (double anomaly : { -1.1, 0.0, 1.1 }) { // [rad]
                SCOPED_TRACE(::testing::Message() << scale << ", " << eccentricity << ", " << anomaly);
                ClassicElements elements = {};
                elements.a = eccentricity == 1.0 ? 0.0 : periapsis / (1.0 - eccentricity);
                elements.e = eccentricity;
                elements.rPeriap = periapsis;
                elements.f = anomaly;
                const double parameter = periapsis * (1.0 + eccentricity);
                const double radius = parameter / (1.0 + eccentricity * cos(anomaly));
                const double speed = sqrt(mu / parameter);
                double expectedR[3] = { radius * cos(anomaly), radius * sin(anomaly), 0.0 };                 // [L]
                double expectedV[3] = { -speed * sin(anomaly), speed * (eccentricity + cos(anomaly)), 0.0 }; // [L/s]
                double r[3], v[3];
                elem2rv(mu, &elements, r, v);
                EXPECT_VECTOR3_NEAR_REL(r, expectedR, 5e-15);
                EXPECT_VECTOR3_NEAR_REL(v, expectedV, 5e-15);
                ClassicElements recovered = {};
                rv2elem(mu, expectedR, expectedV, &recovered);
                EXPECT_NEAR_REL(recovered.rPeriap, periapsis, 5e-15);
                EXPECT_NEAR(recovered.e, eccentricity, 2e-15);
                double conditioning = 5e-15;
                if (eccentricity == 1.0) {
                    EXPECT_EQ(recovered.a, 0.0);
                    EXPECT_EQ(recovered.alpha, 0.0);
                    EXPECT_EQ(recovered.rApoap, 0.0);
                    EXPECT_EQ(recovered.e, 1.0);
                } else {
                    EXPECT_EQ(std::signbit(recovered.alpha), std::signbit(1.0 - eccentricity));
                    conditioning = 8.0 * std::numeric_limits<double>::epsilon() / fabs(1.0 - eccentricity);
                    EXPECT_NEAR_REL(recovered.a, elements.a, conditioning);
                }
                elem2rv(mu, &recovered, r, v);
                EXPECT_VECTOR3_NEAR_REL(r, expectedR, conditioning);
                EXPECT_VECTOR3_NEAR_REL(v, expectedV, conditioning);
            }
        }
    }
}

TEST(OrbitalMotion, rectilinearStates)
{
    for (double scale : { 1e-6, 1e3 }) {
        const double mu = MU_EARTH * scale * scale * scale; // [L^3/s^2]
        const double radius = 7500.0 * scale;               // [L]
        double directions[3][3] = { { 1.0, 0.0, 0.0 }, { 0.0, 0.0, -1.0 }, { 2.0, -3.0, 6.0 } };
        for (auto& direction : directions) {
            for (double speed : { -2.0, -sqrt(2.0), -0.6, 0.0, 0.6, sqrt(2.0), 2.0 }) { // [circular speed]
                SCOPED_TRACE(::testing::Message() << scale << ", " << speed);
                double unit[3], r[3], v[3], reconstructedR[3], reconstructedV[3];
                v3Normalize(direction, unit);
                v3Scale(radius, unit, r);
                v3Scale(speed * sqrt(mu / radius), unit, v);
                ClassicElements elements = {};
                rv2elem(mu, r, v, &elements);
                EXPECT_EQ(elements.e, 1.0);
                EXPECT_EQ(elements.rPeriap, 0.0);
                EXPECT_NEAR(elements.alpha * radius, 2.0 - speed * speed, 3e-15);
                if (elements.alpha == 0.0) {
                    EXPECT_EQ(elements.a, 0.0);
                    EXPECT_NEAR_REL(elements.rmag, radius, 3e-15);
                } else if (elements.a < 0.0) {
                    EXPECT_EQ(std::signbit(elements.f), std::signbit(speed));
                }
                elem2rv(mu, &elements, reconstructedR, reconstructedV);
                EXPECT_VECTOR3_NEAR_REL(reconstructedR, r, 5e-15);
                for (int index = 0; index < 3; ++index) {
                    EXPECT_NEAR(reconstructedV[index], v[index], 1e-13 * scale); // [L/s]
                }
            }
        }
    }
}

TEST(OrbitalMotion, rectilinearForward)
{
    for (double axis : { 7500.0, -7500.0 }) {               // [km]
        for (double anomaly : { -4.0, -1e-9, 1e-9, 4.0 }) { // [rad]
            ClassicElements elements = {};
            elements.a = axis;
            elements.e = 1.0;
            elements.f = anomaly;
            const double factor = axis > 0.0 ? sin(anomaly / 2.0) : sinh(anomaly / 2.0);
            const double radius = 2.0 * fabs(axis) * factor * factor;
            const double direction = axis > 0.0 ? sin(anomaly) : anomaly;
            const double speed = copysign(sqrt(MU_EARTH * (2.0 / radius - 1.0 / axis)), direction);
            double r[3], v[3];
            elem2rv(MU_EARTH, &elements, r, v);
            EXPECT_NEAR_REL(r[0], radius, 3e-15);
            EXPECT_NEAR_REL(v[0], speed, 3e-15);
            EXPECT_EQ(r[1], 0.0);
            EXPECT_EQ(r[2], 0.0);
            EXPECT_EQ(v[1], 0.0);
            EXPECT_EQ(v[2], 0.0);
        }
    }
}

TEST(OrbitalMotion, parabolicApoapsisAndInfinity)
{
    ClassicElements elements = {};
    elements.a = 7500.0; // [km]
    elements.e = std::nextafter(1.0, 0.0);
    elements.f = M_PI; // [rad]
    double r[3], v[3];
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_NEAR_REL(r[0], -elements.a * (1.0 + elements.e), 3e-15);
    EXPECT_NEAR_REL(v[1], -sqrt(MU_EARTH / elements.a * (1.0 - elements.e) / (1.0 + elements.e)), 3e-15);
    elements.a = 0.0; // [km]
    elements.e = 1.0;
    elements.rPeriap = 7500.0; // [km]
    elements.f = M_PI - 1e-8;  // [rad]
    elem2rv(MU_EARTH, &elements, r, v);
    const double barkerVariable = tan(elements.f / 2.0);
    EXPECT_NEAR_REL(v3Norm(r), elements.rPeriap * (1.0 + barkerVariable * barkerVariable), 3e-15);
    EXPECT_GT(v[1], 0.0);
    EXPECT_NEAR_REL(v3Dot(v, v), 2.0 * MU_EARTH / v3Norm(r), 3e-15);
}

TEST(OrbitalMotion, rejectsInvalidConics)
{
    // Each row gives a [km], e [-], f [rad], and periapsis [km].
    const double cases[][4] = { { 7500.0, 1.0, 0.0, 0.0 },   { -7500.0, 1.0, 0.0, 0.0 },  { 0.0, 1.0, M_PI, 7500.0 },
                                { -7500.0, 2.0, M_PI, 0.0 }, { 0.0, 1.0, 0.0, -1.0 },     { 7500.0, 2.0, 0.0, 0.0 },
                                { -7500.0, 0.5, 0.0, 0.0 },  { 0.0, 1.0, 0.0, 0.0 },      { -7500.0, 1.0, 1500.0, 0.0 },
                                { -7500.0, 2.0, 1e16, 0.0 }, { -7500.0, 2.0, -1e16, 0.0 } };
    for (const auto& values : cases) {
        ClassicElements elements = {};
        elements.a = values[0];
        elements.e = values[1];
        elements.f = values[2];
        elements.rPeriap = values[3];
        double r[3] = {}, v[3] = {};
        elem2rv(MU_EARTH, &elements, r, v);
        for (int index = 0; index < 3; ++index) {
            EXPECT_TRUE(std::isnan(r[index]));
            EXPECT_TRUE(std::isnan(v[index]));
        }
    }
}

TEST(OrbitalMotion, rejectsInvalidStates)
{
    for (int testCase = 0; testCase < 5; ++testCase) {
        double mu = 1.0;                 // [L^3/s^2]
        double r[3] = { 1.0, 0.0, 0.0 }; // [L]
        double v[3] = { 0.0, 1.0, 0.0 }; // [L/s]
        if (testCase == 0)
            mu = 0.0; // [L^3/s^2]
        if (testCase == 1)
            r[0] = 0.0; // [L]
        if (testCase == 2)
            r[0] = std::numeric_limits<double>::quiet_NaN();
        if (testCase == 3)
            v[0] = std::numeric_limits<double>::infinity();
        if (testCase == 4) {
            v[0] = 1.0;   // [L/s]
            v[1] = 1e-10; // [L/s]; finite energy, but e rounds to one
        }
        ClassicElements elements = {};
        rv2elem(mu, r, v, &elements);
        for (double value : { elements.a,
                              elements.e,
                              elements.i,
                              elements.Omega,
                              elements.omega,
                              elements.f,
                              elements.rmag,
                              elements.alpha,
                              elements.rPeriap,
                              elements.rApoap }) {
            EXPECT_TRUE(std::isnan(value));
        }
    }
}

TEST(OrbitalMotion, nearlyRectilinearState)
{
    double r[3] = { 1.0, 0.0, 0.0 };  // [L]
    double v[3] = { 1.0, 1e-3, 0.0 }; // [L/s]
    const double mu = 1.0;            // [L^3/s^2]
    ClassicElements elements = {};
    rv2elem(mu, r, v, &elements);
    EXPECT_LT(elements.e, 1.0);
    EXPECT_GT(elements.rPeriap, 0.0);
    double r2[3], v2[3];
    elem2rv(mu, &elements, r2, v2);
    EXPECT_VECTOR3_NEAR_REL(r2, r, 1e-9);
    EXPECT_VECTOR3_NEAR_REL(v2, v, 1e-9);
}

TEST(OrbitalMotion, nearlyRectilinearEnergyConsistency)
{
    double r[3] = { -0.7920102500247754, 0.3704143912399991, -0.48529675727125304 }; // [L]
    double v[3] = { -1.584020499625905, 0.7408287833858261, -0.9705935145425061 };   // [L/s]
    const double mu = 1.0;                                                           // [L^3/s^2]
    ClassicElements elements = {};
    rv2elem(mu, r, v, &elements);
    // e may round to either side of one; never return a mismatched conic.
    if (!std::isnan(elements.a)) {
        EXPECT_LT(elements.a, 0.0);
        EXPECT_GT(elements.e, 1.0);
    } else {
        EXPECT_TRUE(std::isnan(elements.e));
    }
}

TEST(OrbitalMotion, elem2rv1DHyperbolic)
{
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;
    elements.a = -7500.0;
    elements.e = 1.0;
    elements.i = 40.0 * D2R;
    elements.Omega = 133.0 * D2R;
    elements.omega = 113.0 * D2R;
    elements.f = 23.0 * D2R;
    elements.alpha = 1.0 / elements.a;
    elements.rPeriap = elements.a * (1. - elements.e);
    elements.rApoap = elements.a * (1. + elements.e);
    elem2rv(MU_EARTH, &elements, r, v);
    v3Set(-152.641873349816, -469.543156608544, 362.375968124408, r2);
    v3Set(-9.17378720883851, -28.2195763820421, 21.7788208976681, v3_2);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

class TwoDimensionHyperbolic : public ::testing::Test
{
  protected:
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;

    void SetUp() override
    {
        elements.a = -7500.0;
        elements.e = 1.4;
        elements.i = 40.0 * D2R;
        elements.Omega = 133.0 * D2R;
        elements.omega = 113.0 * D2R;
        elements.f = 23.0 * D2R;
        elements.alpha = 1.0 / elements.a;
        elements.rPeriap = elements.a * (1. - elements.e);
        elements.rApoap = elements.a * (1. + elements.e);

        v3Set(319.013136281857, -2796.71958333493, 1404.6919948109, r2);
        v3Set(15.3433051336115, -5.87012423567412, -6.05659420479213, v3_2);
    }
};

TEST_F(TwoDimensionHyperbolic, elem2rv)
{
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST_F(TwoDimensionHyperbolic, rv2elem)
{
    rv2elem(MU_EARTH, r2, v3_2, &elements);
    EXPECT_NEAR_REL(elements.a, -7500.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.e, 1.4, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.i, 40.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.Omega, 133.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.omega, 113.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.f, 23.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rmag, 3145.881340612725, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rPeriap, 3000.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rApoap, -18000., orbitalElementsAccuracy);
}

class TwoDimensionParabolic : public ::testing::Test
{
  protected:
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;

    void SetUp() override
    {
        elements.alpha = 0.0; /* zero orbit energy, i.e. parabolic */
        elements.a = 0.0;
        elements.rPeriap = 7500.;
        elements.e = 1.0;
        elements.i = 40.0 * D2R;
        elements.Omega = 133.0 * D2R;
        elements.omega = 113.0 * D2R;
        elements.f = 123.0 * D2R;
        v3Set(27862.6148209797, 795.70270010667, -17554.0435142669, r2);
        v3Set(3.06499561197954, 2.21344887266898, -3.14760065404514, v3_2);
    }
};

TEST_F(TwoDimensionParabolic, elem2rv)
{
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST_F(TwoDimensionParabolic, rv2elem)
{
    rv2elem(MU_EARTH, r2, v3_2, &elements);
    EXPECT_NEAR(elements.alpha, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.a, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.e, 1.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.i, 40.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.Omega, 133.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.omega, 113.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.f, 123.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rmag, 32940.89997480352, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rPeriap, 7500.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rApoap, 0.0, orbitalElementsAccuracy);
}

class TwoDimensionElliptical : public ::testing::Test
{
  protected:
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;

    void SetUp() override
    {
        elements.a = 7500.0;
        elements.e = 0.5;
        elements.i = 40.0 * D2R;
        elements.Omega = 133.0 * D2R;
        elements.omega = 113.0 * D2R;
        elements.f = 123.0 * D2R; /* true anomaly */
        elements.alpha = 1.0 / elements.a;
        elements.rPeriap = elements.a * (1. - elements.e);
        elements.rApoap = elements.a * (1. + elements.e);

        v3Set(6538.3506963942027, 186.7227227879431, -4119.3008399778619, r2);
        v3Set(1.4414106130924005, 5.588901415902356, -4.0828931566657038, v3_2);
    }
};

TEST_F(TwoDimensionElliptical, elem2rv)
{
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST_F(TwoDimensionElliptical, rv2elem)
{
    rv2elem(MU_EARTH, r2, v3_2, &elements);
    EXPECT_NEAR_REL(elements.a, 7500.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.e, 0.5, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.i, 40.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.Omega, 133.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.omega, 113.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.f, 123.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rmag, 7730.041048693483, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rPeriap, 3750.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rApoap, 11250.0, orbitalElementsAccuracy);
}

class NonCircularEquitorial : public ::testing::Test
{
  protected:
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;

    void SetUp() override
    {
        elements.a = 7500.0;
        elements.e = 0.5;
        elements.i = 0.0 * D2R;
        elements.Omega = 133.0 * D2R;
        elements.omega = 113.0 * D2R;
        elements.f = 123.0 * D2R;
        v3Set(7634.8714161163643, 1209.2448361913848, -0, r2);
        v3Set(2.5282399359829868, 6.6023861555546057, -0, v3_2);
    }
};

TEST_F(NonCircularEquitorial, elem2rv)
{
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST_F(NonCircularEquitorial, rv2elem)
{
    rv2elem(MU_EARTH, r2, v3_2, &elements);
    EXPECT_NEAR_REL(elements.a, 7500.00, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.e, 0.5, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.i, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.Omega, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.omega, (133 + 113) * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.f, 123.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rmag, 7730.041048693483, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rPeriap, 3750.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rApoap, 11250.0, orbitalElementsAccuracy);
}

class NonCircularNearEquitorial : public ::testing::Test
{
  protected:
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;
    double eps2 = 1e-12 * 0.5;

    void SetUp() override
    {
        elements.a = 7500.0;
        elements.e = 0.5;
        elements.i = eps2;
        elements.Omega = 133.0 * D2R;
        elements.omega = 113.0 * D2R;
        elements.f = 123.0 * D2R;
        v3Set(7634.87141611636, 1209.24483619139, -3.20424723337984e-09, r2);
        v3Set(2.52823993598298, 6.60238615555461, -3.17592708317508e-12, v3_2);
    }
};

TEST_F(NonCircularNearEquitorial, elem2rv)
{
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST_F(NonCircularNearEquitorial, rv2elem)
{
    rv2elem(MU_EARTH, r2, v3_2, &elements);
    EXPECT_NEAR_REL(elements.a, 7500.00, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.e, 0.5, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.i, eps2, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(wrapToPi(elements.Omega + elements.omega), (133 + 113 - 360.) * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.f, 123.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rmag, 7730.041048693483, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rPeriap, 3750.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rApoap, 11250.0, orbitalElementsAccuracy);
}

class NonCircularNearEquatorial180Degree : public ::testing::Test
{
  protected:
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;
    double eps2 = 1e-12 * 0.5; // [rad]

    void SetUp() override
    {
        elements.a = 7500.0;
        elements.e = 0.5;
        elements.i = M_PI - eps2; // [rad]
        elements.Omega = 133.0 * D2R;
        elements.omega = 113.0 * D2R;
        elements.f = 123.0 * D2R;
        v3Set(-1738.8808840249571, -7531.920597407997, -3.2053169070484574e-9, r2);   // [km]
        v3Set(-6.762664177427689, -2.061522094059298, -3.1769873027295912e-12, v3_2); // [km/s]
    }
};

TEST_F(NonCircularNearEquatorial180Degree, elem2rv)
{
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST_F(NonCircularNearEquatorial180Degree, rv2elem)
{
    rv2elem(MU_EARTH, r2, v3_2, &elements);
    EXPECT_NEAR_REL(elements.a, 7500.00, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.e, 0.5, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.i, M_PI - eps2, orbitalElementsAccuracy);                                          // [rad]
    EXPECT_NEAR_REL(wrapToPi(elements.omega - elements.Omega), (113 - 133) * D2R, orbitalElementsAccuracy); // [rad]
    EXPECT_NEAR_REL(elements.f, 123.0 * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rmag, 7730.041048693483, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rPeriap, 3750.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rApoap, 11250.0, orbitalElementsAccuracy);
}

class CircularInclined : public ::testing::Test
{
  protected:
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;

    void SetUp() override
    {
        elements.a = 7500.0;
        elements.e = 0.0;
        elements.i = 40.0 * D2R;
        elements.Omega = 133.0 * D2R;
        elements.omega = 113.0 * D2R;
        elements.f = 123.0 * D2R;
        v3Set(6343.7735859429586, 181.16597468085499, -3996.7130970223939, r2);
        v3Set(-1.8379619466304487, 6.5499717954886121, -2.6203988553352131, v3_2);
    }
};

TEST_F(CircularInclined, elem2rv)
{
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST_F(CircularInclined, rv2elem)
{
    rv2elem(MU_EARTH, r2, v3_2, &elements);
    EXPECT_NEAR_REL(elements.a, 7500.00, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.e, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.i, 40. * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.Omega, 133. * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(wrapToPi(elements.omega + elements.f), (113 + 123 - 360.) * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rmag, 7500.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rPeriap, 7500.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rApoap, 7500.0, orbitalElementsAccuracy);
}

class CircularEquitorial : public ::testing::Test
{
  protected:
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;

    void SetUp() override
    {
        elements.a = 7500.0;
        elements.e = 0.0;
        elements.i = 0.0;
        elements.Omega = 133.0 * D2R;
        elements.omega = 113.0 * D2R;
        elements.f = 123.0 * D2R;
        v3Set(7407.6625544635335, 1173.2584878017262, 0, r2);
        v3Set(-1.1404354122910105, 7.2004258117414572, 0, v3_2);
    }
};

TEST_F(CircularEquitorial, elem2rv)
{
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST_F(CircularEquitorial, rv2elem)
{
    rv2elem(MU_EARTH, r2, v3_2, &elements);
    EXPECT_NEAR_REL(elements.a, 7500.00, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.e, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.i, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.Omega, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.omega, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.f, (133 + 113 + 123 - 360) * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rmag, 7500.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rPeriap, 7500.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rApoap, 7500.0, orbitalElementsAccuracy);
}

class CircularEquitorialRetrograde : public ::testing::Test
{
  protected:
    double r[3];
    double v[3];
    double r2[3];
    double v3_2[3];
    ClassicElements elements;

    void SetUp() override
    {
        elements.a = 7500.0;
        elements.e = 0.0;
        elements.i = M_PI;
        elements.Omega = 133.0 * D2R;
        elements.omega = 113.0 * D2R;
        elements.f = 123.0 * D2R;
        v3Set(-1687.13290757899, -7307.77548588926, 0.0, r2);
        v3Set(-7.10333318346184, 1.63993368302803, 0.0, v3_2);
    }
};

TEST_F(CircularEquitorialRetrograde, elem2rv)
{
    elem2rv(MU_EARTH, &elements, r, v);
    EXPECT_VECTOR3_NEAR_REL(r, r2, orbitalElementsAccuracy);
    EXPECT_VECTOR3_NEAR_REL(v, v3_2, orbitalElementsAccuracy);
}

TEST_F(CircularEquitorialRetrograde, rv2elem)
{
    rv2elem(MU_EARTH, r2, v3_2, &elements);
    EXPECT_NEAR_REL(elements.a, 7500.00, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.e, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.i, M_PI, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.Omega, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR(elements.omega, 0.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.f, (-133 + 113 + 123) * D2R, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rmag, 7500.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rPeriap, 7500.0, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements.rApoap, 7500.0, orbitalElementsAccuracy);
}

TEST(OrbitalMotion, classicElementsToMeanElements)
{
    ClassicElements elements;
    elements.a = 1000.0;
    elements.e = 0.2;
    elements.i = 0.2;
    elements.Omega = 0.15;
    elements.omega = 0.5;
    elements.f = 0.2;
    double req = 300.0;
    double J2 = 1e-3;
    ClassicElements elements_p;
    clMeanOscMap(req, J2, &elements, &elements_p, 1);
    EXPECT_NEAR_REL(elements_p.a, 1000.07546442015950560744386166334152, orbitalElementsAccuracy);
    EXPECT_NEAR(elements_p.e, 0.20017786852908628358882481279579, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements_p.i, 0.20000333960738947425284095515963, orbitalElementsAccuracy);
    EXPECT_NEAR(elements_p.Omega, 0.15007256499303692209856819772540, orbitalElementsAccuracy);
    EXPECT_NEAR(elements_p.omega, 0.50011857315729335571319325026707, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements_p.f, 0.19982315726261962174348241205735, orbitalElementsAccuracy);
}

/** @brief Compare both equatorial orientations with an independent planar limit. */
TEST(OrbitalMotion, meanOscEquatorial)
{
    // Planar-limit references: delta(a)/a = 2*gamma*(a_r^3-eta^-3),
    // delta(mean longitude) = 3*gamma/eta^4*(f-M+e*sin(f)).
    // Eccentricity and mean anomaly are recovered from (e+delta(e), e*delta(M)).
    const double expectedR[2][3] = { { 4499.199053733382, 5668.85695301761, 0.0 },
                                     { 4488.47581038344, 5657.032670656745, 0.0 } }; // [km]
    const double expectedV[2][3] = { { -5.950694310784638, 4.990389813133058, 0.0 },
                                     { -5.964686867945799, 5.000352990770512, 0.0 } }; // [km/s]
    for (int direction : { -1, 1 }) {
        const int index = (direction + 1) / 2;
        for (bool retrograde : { false, true }) {
            ClassicElements elements = {};
            elements.a = 8000.0; // [km]
            elements.e = 0.1;
            elements.i = retrograde ? M_PI : 0.0;     // [rad]
            elements.Omega = retrograde ? -0.2 : 0.2; // [rad]
            elements.omega = 0.4;                     // [rad]
            elements.f = 0.3;                         // [rad]
            ClassicElements mapped = {};
            clMeanOscMap(REQ_EARTH, J2_EARTH, &elements, &mapped, direction);
            EXPECT_DOUBLE_EQ(mapped.i, elements.i);
            double position[3], velocity[3];
            elem2rv(MU_EARTH, &mapped, position, velocity);
            for (int component = 0; component < 3; ++component) {
                const double reflection = retrograde && component == 1 ? -1.0 : 1.0;
                EXPECT_NEAR(position[component], reflection * expectedR[index][component], 2e-10); // [km]
                EXPECT_NEAR(velocity[component], reflection * expectedV[index][component], 2e-13); // [km/s]
            }
        }
    }
}

/** @brief Preserve the exact unwrapped elements when J2 vanishes, including aliasing. */
TEST(OrbitalMotion, meanOscZeroJ2)
{
    const double inclinations[] = { 0.0, 1e-12, std::acos(1.0 / std::sqrt(5.0)), M_PI - 1e-12, M_PI }; // [rad]
    for (double inclination : inclinations) {
        for (double direction : { -1.0, 1.0 }) {
            ClassicElements elements = {};
            elements.a = 8000.0; // [km]
            elements.e = 0.1;
            elements.i = inclination;
            elements.Omega = 0.2;          // [rad]
            elements.omega = 0.4;          // [rad]
            elements.f = 8.0 * M_PI + 0.3; // [rad]
            ClassicElements mapped = {};
            clMeanOscMap(REQ_EARTH, 0.0, &elements, &mapped, direction);
            EXPECT_DOUBLE_EQ(mapped.a, elements.a);
            EXPECT_DOUBLE_EQ(mapped.e, elements.e);
            EXPECT_DOUBLE_EQ(mapped.i, elements.i);
            EXPECT_DOUBLE_EQ(mapped.Omega, elements.Omega);
            EXPECT_DOUBLE_EQ(mapped.omega, elements.omega);
            EXPECT_DOUBLE_EQ(mapped.f, elements.f);
            clMeanOscMap(REQ_EARTH, 0.0, &mapped, &mapped, direction);
            EXPECT_DOUBLE_EQ(mapped.f, elements.f);
            EXPECT_DOUBLE_EQ(mapped.i, elements.i);
        }
    }
}

/** @brief Whole turns must not change the mapped state in either direction or hemisphere. */
TEST(OrbitalMotion, meanOscRevolutions)
{
    const double inclinations[] = { 0.0, 1e-8, 0.7, M_PI - 1e-8, M_PI };             // [rad]
    const double anomalies[] = { 0.0, 0.3, -0.3, M_PI - 1e-10, M_PI, M_PI + 1e-10 }; // [rad]
    for (double inclination : inclinations) {
        for (double direction : { -1.0, 1.0 }) {
            for (double anomaly : anomalies) {
                ClassicElements elements = {};
                elements.a = 8000.0; // [km]
                elements.e = 0.1;
                elements.i = inclination;
                elements.Omega = 0.2; // [rad]
                elements.omega = 0.4; // [rad]
                elements.f = anomaly;
                ClassicElements mapped = {};
                clMeanOscMap(REQ_EARTH, J2_EARTH, &elements, &mapped, direction);
                double referenceR[3], referenceV[3];
                elem2rv(MU_EARTH, &mapped, referenceR, referenceV);
                for (int turns : { -100, -2, -1, 1, 2, 100 }) {
                    ClassicElements wrapped = elements;
                    wrapped.f += turns * 2.0 * M_PI; // [rad]
                    clMeanOscMap(REQ_EARTH, J2_EARTH, &wrapped, &wrapped, direction);
                    double position[3], velocity[3];
                    elem2rv(MU_EARTH, &wrapped, position, velocity);
                    for (int component = 0; component < 3; ++component) {
                        EXPECT_NEAR(position[component], referenceR[component], 3e-8);  // [km]
                        EXPECT_NEAR(velocity[component], referenceV[component], 3e-11); // [km/s]
                    }
                }
            }
        }
    }
}

TEST(OrbitalMotion, classicElementsToEquinoctialElements)
{
    ClassicElements elements;
    elements.a = 1000.0;
    elements.e = 0.2;
    elements.i = 0.2;
    elements.Omega = 0.15;
    elements.omega = 0.5;
    elements.f = 0.2;
    equinoctialElements elements_eq;
    clElem2eqElem(&elements, &elements_eq);

    EXPECT_NEAR_REL(elements_eq.a, 1000.00000000000000000000000000000000, orbitalElementsAccuracy);
    EXPECT_NEAR(elements_eq.P1, 0.12103728114720790909331071816268, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements_eq.P2, 0.15921675970981119530023306651856, orbitalElementsAccuracy);
    EXPECT_NEAR(elements_eq.Q1, 0.01499382601880069713906618034116, orbitalElementsAccuracy);
    EXPECT_NEAR(elements_eq.Q2, 0.09920802187229026125603326136115, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements_eq.l, 0.78093005232114087732497864635661, orbitalElementsAccuracy);
    EXPECT_NEAR_REL(elements_eq.L, 0.85000000000000008881784197001252, orbitalElementsAccuracy);
}

/** @brief Reject undefined Hill planes and invalid deputy states at every C entry point. */
TEST(OrbitalMotion, hillInvalidDomains)
{
    const double chiefs[][6] = {
        { 0, 0, 0, 0, 1, 0 },
        { 1, 0, 0, 0, 0, 0 },
        { 1, 0, 0, 1, 0, 0 },
        { 1, 0, 0, -1, 0, 0 },
        { 1, 0, 0, 1, 1e-16, 0 },
        { std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 1, 0 },
        { 1, 0, 0, 0, std::numeric_limits<double>::infinity(), 0 }
    };                                // Position [L], velocity [L/s].
    double relative[3] = { 1, 2, 3 }; // [L]
    double velocity[3] = {};          // [L/s]
    for (const auto& chief : chiefs) {
        double position[3], speed[3], frame[3][3], outputR[3], outputV[3];
        for (int index = 0; index < 3; index++) {
            position[index] = chief[index];
            speed[index] = chief[index + 3];
        }
        hillFrameChecked(position, speed, frame);
        for (const auto& row : frame) {
            for (double entry : row) {
                EXPECT_TRUE(std::isnan(entry));
            }
        }
        for (auto transform : { hill2rvChecked, rv2hillChecked }) {
            transform(position, speed, relative, velocity, outputR, outputV);
            for (int index = 0; index < 3; index++) {
                EXPECT_TRUE(std::isnan(outputR[index]));
                EXPECT_TRUE(std::isnan(outputV[index]));
            }
        }
    }
    for (auto transform : { hill2rvChecked, rv2hillChecked }) {
        for (int argument : { 0, 1 }) {
            double position[3] = { 7000, 0, 0 }; // [km]
            double speed[3] = { 0, 7, 0 };       // [km/s]
            double deputyR[3] = { 1, 2, 3 };     // [km]
            double deputyV[3] = {};              // [km/s]
            (argument == 0 ? deputyR : deputyV)[1] = std::numeric_limits<double>::quiet_NaN();
            double outputR[3], outputV[3];
            transform(position, speed, deputyR, deputyV, outputR, outputV);
            for (int index = 0; index < 3; index++) {
                EXPECT_TRUE(std::isnan(outputR[index]));
                EXPECT_TRUE(std::isnan(outputV[index]));
            }
        }
    }
}

/** @brief Hill geometry and rotating-frame velocity must not depend on distance scaling. */
TEST(OrbitalMotion, hillScaledRoundTrip)
{
    for (double scale : { 1e-200, 1.0, 1e160 }) {
        double position[3] = { 7000 * scale, 0, 0 };                           // [L]
        double velocity[3] = { scale, 0, 7 * scale };                          // [L/s]
        double relative[3] = { 2 * scale, 3 * scale, 4 * scale };              // [L]
        double relativeRate[3] = { 0.01 * scale, 0.02 * scale, 0.03 * scale }; // [L/s]
        double deputyR[3], deputyV[3], recoveredR[3], recoveredV[3], frame[3][3];
        hillFrame(position, velocity, frame);
        EXPECT_NEAR(frame[0][0], 1.0, 1e-15);
        EXPECT_NEAR(frame[1][2], 1.0, 1e-15);
        EXPECT_NEAR(frame[2][1], -1.0, 1e-15);
        hill2rv(position, velocity, relative, relativeRate, deputyR, deputyV);
        const double expectedR[3] = { 7002.0, -4.0, 3.0 };   // [L/scale]
        const double expectedV[3] = { 1.007, -0.03, 7.022 }; // [L/s/scale]
        rv2hill(position, velocity, deputyR, deputyV, recoveredR, recoveredV);
        for (int index = 0; index < 3; index++) {
            EXPECT_NEAR(deputyR[index] / scale, expectedR[index], 2e-12);
            EXPECT_NEAR(deputyV[index] / scale, expectedV[index], 2e-14);
            EXPECT_NEAR(recoveredR[index] / scale, relative[index] / scale, 2e-12);
            EXPECT_NEAR(recoveredV[index] / scale, relativeRate[index] / scale, 2e-14);
        }
    }
}

/** @brief Native compatibility entry points retain their old rejected-domain results. */
TEST(OrbitalMotion, legacyDomainFallbacks)
{
    double position[3] = { 7000.0, 0.0, 0.0 }; // [km]
    double velocity[3] = {};                   // [km/s]
    double frame[3][3];
    hillFrame(position, velocity, frame);
    EXPECT_DOUBLE_EQ(frame[0][0], 1.0);
    for (int row = 1; row < 3; row++) {
        for (double value : frame[row]) {
            EXPECT_DOUBLE_EQ(value, 0.0);
        }
    }
    ClassicElements elements = {};
    elements.a = 8000.0; // [km]
    elements.e = 0.1;
    elements.i = M_PI; // [rad]
    equinoctialElements output;
    clElem2eqElem(&elements, &output);
    EXPECT_DOUBLE_EQ(output.Q2, tan(M_PI / 2.0));
    clElem2eqElemChecked(&elements, &output);
    EXPECT_TRUE(std::isnan(output.Q2));
    elements.i = 0.7; // [rad]
    elements.f = 0.3; // [rad]
    ClassicElements mapped = {};
    mapped.rmag = 42.0; // [km]; verify auxiliary output fields are untouched.
    clMeanOscMap(REQ_EARTH, J2_EARTH, &elements, &mapped, 0);
    EXPECT_DOUBLE_EQ(mapped.a, elements.a);
    EXPECT_NEAR(mapped.f, elements.f, 2e-15);
    EXPECT_DOUBLE_EQ(mapped.rmag, 42.0);
    elements.rmag = 42.0; // [km]
    clMeanOscMap(REQ_EARTH, J2_EARTH, &elements, &elements, 0);
    EXPECT_NEAR(elements.f, mapped.f, 2e-15);
    EXPECT_DOUBLE_EQ(elements.rmag, 42.0);
}

/** @brief Inclination validation is intentional and applies to all public element converters. */
TEST(OrbitalMotion, inclinationInterval)
{
    ClassicElements elements = {};
    elements.a = 8000.0; // [km]
    elements.e = 0.1;
    for (double inclination : { -0.1, M_PI + 0.1, -2 * M_PI, 2 * M_PI }) { // [rad]
        elements.i = inclination;
        double position[3], velocity[3];
        elem2rv(MU_EARTH, &elements, position, velocity);
        EXPECT_TRUE(std::isnan(position[0]));
        EXPECT_TRUE(std::isnan(velocity[0]));
        ClassicElements mapped = {};
        clMeanOscMap(REQ_EARTH, J2_EARTH, &elements, &mapped, 1);
        EXPECT_TRUE(std::isnan(mapped.a));
        equinoctialElements output;
        clElem2eqElem(&elements, &output);
        EXPECT_TRUE(std::isnan(output.a));
    }
    for (double inclination : { 0.0, M_PI }) { // [rad]
        elements.i = inclination;
        double position[3], velocity[3];
        elem2rv(MU_EARTH, &elements, position, velocity);
        EXPECT_TRUE(std::isfinite(position[0]));
        EXPECT_TRUE(std::isfinite(velocity[0]));
    }
}
