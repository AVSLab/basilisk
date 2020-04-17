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

#include <Eigen/Dense>
#include "../dynamics/_GeneralModuleFiles/gravityEffector.h"
#include <utilities/orbitalMotion.h>

/*! \addtogroup Sim Utility Group
 *  This group contains the simulation utilities that are used globally on the
 *  simulation side of the software.  Note that FSW should not generally use
 *  these utilities once we reach a CDR level of maturity on a project.
 * @{
 */

//! @brief The KeplerianOrbit class represents an elliptical orbit and provides a coherent set of
//! common outputs such as position and velocity, orbital period, semi-parameter, etc. It uses the
//! utility orbitalMotion to do orbital element to position and velocity conversion.
class KeplerianOrbit {
public:
    KeplerianOrbit();
    KeplerianOrbit(classicElements oe, GravBodyData* planet);
    KeplerianOrbit(const KeplerianOrbit &orig);
    ~KeplerianOrbit();
    
    Eigen::Vector3d r_BP_P(){return this->position_BP_P;};
    Eigen::Vector3d v_BP_P(){return this->velocity_BP_P;};
    Eigen::Vector3d h_BP_P(){return this->orbital_angular_momentum_P;};
    double M(){return this->mean_anomaly;};
    double n(){return this->mean_motion;};
    double P(){return this->orbital_period;};
    double f(){return this->true_anomaly;};
    double fDot(){return this->true_anomaly_rate;};
    double RAAN(){return this->right_ascension;};
    double omega(){return this->argument_of_periapsis;};
    double i(){return this->inclination;};
    double e(){return this->eccentricity;};
    double a(){return this->semi_major_axis;};
    double h(){return this->h_BP_P().norm();};
    double Energy(){return this->orbital_energy;};
    double r(){return this->r_BP_P().norm();};
    double v(){return this->v_BP_P().norm();};
    double r_a(){return this->r_apogee;};
    double r_p(){return this->r_perigee;};
    double fpa(){return this->flight_path_angle;};
    double E(){return this->eccentric_anomaly;};
    double p(){return this->semi_parameter;};
    double rDot(){return this->radial_rate;};
    double c3(){return this->v_infinity;};
    classicElements oe();
    void set_planet(GravBodyData* plt);
    void set_a(double a){this->semi_major_axis = a; this->change_orbit();};
    void set_e(double e){this->eccentricity = e; this->change_orbit();};
    void set_i(double i){this->inclination = i; this->change_orbit();};
    void set_omega(double omega){this->argument_of_periapsis = omega; this->change_orbit();};
    void set_RAAN(double RAAN){this->right_ascension = RAAN; this->change_orbit();};
    void set_f(double f){this->true_anomaly = f; this->change_f();};
    
private:
    GravBodyData* planet;
    double mu;
    double semi_major_axis; 
    double eccentricity;
    double inclination;
    double argument_of_periapsis;
    double right_ascension;
    double true_anomaly;
    double true_anomaly_rate;
    double orbital_period;
    double orbital_energy;
    double v_infinity;
    double orbit_radius;
    double radial_rate;
    double r_apogee;
    double r_perigee;
    double semi_parameter;
    double flight_path_angle;
    double eccentric_anomaly;
    double mean_motion;
    double mean_anomaly;
    Eigen::Vector3d orbital_angular_momentum_P;
    Eigen::Vector3d position_BP_P;
    Eigen::Vector3d velocity_BP_P;
private:
    void change_orbit();
    void change_f();
};

/*! @} */
