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

#pragma once

#include "architecture/utilities/astroConstants.h"
#include <Eigen/Dense>
#include <architecture/utilities/orbitalMotion.h>
#include <cmath>

/*! @brief Represent an elliptical orbit with a coherent set of derived outputs.
 *
 * Position and velocity are computed with orbitalMotion. Constructors and element
 * or gravitational-parameter setters immediately refresh the affected outputs.
 * Use meters, seconds, and radians for inputs and outputs.
 *
 * @note The supported domain is a positive semi-major axis, a positive gravitational
 * parameter, and eccentricity in [0, 1). Hyperbolic, parabolic, and rectilinear
 * orbits are outside this class's scope.
 * @note E() and M() follow the f2E() and E2M() angle conventions without additional
 * normalization to [0, 2*pi). Negative and wrapped true anomalies are accepted;
 * these conversions do not preserve a count of completed revolutions.
 */
class KeplerianOrbit
{
  public:
    KeplerianOrbit();
    KeplerianOrbit(ClassicElements oe, const double mu);
    KeplerianOrbit(const KeplerianOrbit& orig);
    ~KeplerianOrbit();

    Eigen::Vector3d r_BP_P() const; //!< body position vector relative to planet
    Eigen::Vector3d v_BP_P() const; //!< body velocity vector relative to planet
    Eigen::Vector3d h_BP_P() const; //!< angular momentum of body relative to planet
    double M() const;               //!< Mean elliptic anomaly [rad], using E2M().
    double n() const;
    double P() const;
    double f() const;
    double fDot() const; //!< True anomaly rate [rad/s].
    double RAAN() const;
    double omega() const;
    double i() const;
    double e() const;
    double a() const;
    double h() const;
    double Energy();
    double r() const;
    double v() const;
    double r_a() const;
    double r_p() const;
    double fpa() const; //!< Flight-path angle [rad], negative for inward motion.
    double E() const;   //!< Eccentric anomaly [rad], using f2E().
    double p() const;
    double rDot() const; //!< Radial velocity [m/s], negative for inward motion.
    double c3() const;
    /*! @brief Return the current orbital elements and derived geometry.
     * @return Elements including the current radius in rmag [m], inverse semi-major
     * axis in alpha [1/m], and apsis radii in rPeriap and rApoap [m].
     */
    ClassicElements oe();
    /*! @brief Set the central body's gravitational parameter and refresh all derived outputs.
     * @param mu Gravitational parameter [m^3/s^2].
     */
    void set_mu(const double mu);
    /*! @brief Set the semi-major axis and refresh all derived outputs.
     * @param a Semi-major axis [m].
     */
    void set_a(double a);
    void set_e(double e);
    void set_i(double i);
    void set_omega(double omega);
    void set_RAAN(double RAAN);
    void set_f(double f);

  private:
    double mu = MU_EARTH * std::pow(10, 9); // [m^3/s^2]
    double semi_major_axis = 1E5 * 1000;    // convert to meters
    double eccentricity = 1E-5;
    double inclination{};
    double argument_of_periapsis{};
    double right_ascension{};
    double true_anomaly{};
    double true_anomaly_rate{};
    double orbital_period{};
    double orbital_energy{};
    double v_infinity{};
    double orbit_radius{};
    double radial_rate{};
    double r_apogee{};
    double r_perigee{};
    double semi_parameter{};
    double flight_path_angle{};
    double eccentric_anomaly{};
    double mean_motion{};
    double mean_anomaly{};
    Eigen::Vector3d orbital_angular_momentum_P;
    Eigen::Vector3d position_BP_P;
    Eigen::Vector3d velocity_BP_P;

  private:
    void change_orbit();
    void change_f();
};
