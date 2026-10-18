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

#include "keplerianOrbit.h"
#include <architecture/utilities/avsEigenSupport.h>
#include <cmath>
#include <stdexcept>

/*! This constructor initialized to an arbitrary orbit */
KeplerianOrbit::KeplerianOrbit()
{
    this->change_orbit();
}

/*! The constructor requires orbital elements and a gravitational constant value */
KeplerianOrbit::KeplerianOrbit(ClassicElements oe, const double mu)
    : mu(mu)
    , semi_major_axis(oe.a)
    , eccentricity(oe.e)
    , inclination(oe.i)
    , argument_of_periapsis(oe.omega)
    , right_ascension(oe.Omega)
    , true_anomaly(oe.f)
{
    this->change_orbit();
}

/*! The copy constructor works with python copy*/
KeplerianOrbit::KeplerianOrbit(const KeplerianOrbit& orig)
    : mu(orig.mu)
    , semi_major_axis(orig.a())
    , eccentricity(orig.e())
    , inclination(orig.i())
    , argument_of_periapsis(orig.omega())
    , right_ascension(orig.RAAN())
    , true_anomaly(orig.f())
{
    this->change_orbit();
}

/*! Generic Destructor */
KeplerianOrbit::~KeplerianOrbit() {}

/*!
    body position vector relative to planet
 */
Eigen::Vector3d KeplerianOrbit::r_BP_P() const
{
    return this->position_BP_P;
}

/*!
    body velocity vector relative to planet
 */
Eigen::Vector3d KeplerianOrbit::v_BP_P() const
{
    return this->velocity_BP_P;
}

/*!
    angular momentum of body relative to planet
 */
Eigen::Vector3d KeplerianOrbit::h_BP_P() const
{
    return this->orbital_angular_momentum_P;
}

double KeplerianOrbit::M() const
{
    return this->mean_anomaly;
}
/*! return mean orbit rate */
double KeplerianOrbit::n() const
{
    return this->mean_motion;
}; //!< return mean orbit rate
/*! return orbit period */
double KeplerianOrbit::P() const
{
    return this->orbital_period;
}; //!< return orbital period
/*! return true anomaly */
double KeplerianOrbit::f() const
{
    return this->true_anomaly;
}; //!< return true anomaly
double KeplerianOrbit::fDot() const
{
    return this->true_anomaly_rate;
};
/*! return right ascencion of the ascending node */
double KeplerianOrbit::RAAN() const
{
    return this->right_ascension;
};
/*! return argument of periapses */
double KeplerianOrbit::omega() const
{
    return this->argument_of_periapsis;
};
/*! return inclination angle */
double KeplerianOrbit::i() const
{
    return this->inclination;
};
/*! return eccentricty */
double KeplerianOrbit::e() const
{
    return this->eccentricity;
};
/*! return semi-major axis */
double KeplerianOrbit::a() const
{
    return this->semi_major_axis;
};
/*! return orbital angular momentum magnitude */
double KeplerianOrbit::h() const
{
    return this->h_BP_P().norm();
};
/*! return orbital energy */
double KeplerianOrbit::Energy()
{
    return this->orbital_energy;
};
/*! return orbit radius */
double KeplerianOrbit::r() const
{
    return this->r_BP_P().norm();
};
/*! return velocity magnitude */
double KeplerianOrbit::v() const
{
    return this->v_BP_P().norm();
};
/*! return radius at apoapses */
double KeplerianOrbit::r_a() const
{
    return this->r_apogee;
};
/*! return radius at periapses */
double KeplerianOrbit::r_p() const
{
    return this->r_perigee;
};
double KeplerianOrbit::fpa() const
{
    return this->flight_path_angle;
};
double KeplerianOrbit::E() const
{
    return this->eccentric_anomaly;
};
/*! return semi-latus rectum or the parameter */
double KeplerianOrbit::p() const
{
    return this->semi_parameter;
};
double KeplerianOrbit::rDot() const
{
    return this->radial_rate;
};
double KeplerianOrbit::c3() const
{
    return -this->mu / this->semi_major_axis;
};

void KeplerianOrbit::set_a(double a)
{
    ClassicElements elements = this->oe();
    elements.a = a;
    *this = KeplerianOrbit(elements, this->mu);
}
/*! set eccentricity */
void KeplerianOrbit::set_e(double e)
{
    ClassicElements elements = this->oe();
    elements.e = e;
    *this = KeplerianOrbit(elements, this->mu);
}
/*! set inclination angle */
void KeplerianOrbit::set_i(double i)
{
    ClassicElements elements = this->oe();
    elements.i = i;
    *this = KeplerianOrbit(elements, this->mu);
}
/*! set argument of periapsis */
void KeplerianOrbit::set_omega(double omega)
{
    ClassicElements elements = this->oe();
    elements.omega = omega;
    *this = KeplerianOrbit(elements, this->mu);
}
/*! set right ascension of the ascending node */
void KeplerianOrbit::set_RAAN(double RAAN)
{
    ClassicElements elements = this->oe();
    elements.Omega = RAAN;
    *this = KeplerianOrbit(elements, this->mu);
}
/*! set true anomaly angle */
void KeplerianOrbit::set_f(double f)
{
    ClassicElements elements = this->oe();
    elements.f = f;
    *this = KeplerianOrbit(elements, this->mu);
}

ClassicElements KeplerianOrbit::oe()
{
    ClassicElements elements;
    elements.a = this->semi_major_axis;
    elements.e = this->eccentricity;
    elements.i = this->inclination;
    elements.f = this->true_anomaly;
    elements.omega = this->argument_of_periapsis;
    elements.Omega = this->right_ascension;
    elements.rApoap = this->r_apogee;
    elements.rPeriap = this->r_perigee;
    elements.alpha = 1.0 / elements.a;
    elements.rmag = this->orbit_radius;
    return elements;
}

/*! @brief Recompute the derived state after changing orbital elements or gravity. */
void KeplerianOrbit::change_orbit()
{
    if (!std::isfinite(this->mu) || this->mu <= 0.0 || !std::isfinite(this->a()) || this->a() <= 0.0 ||
        !std::isfinite(this->e()) || this->e() < 0.0 || this->e() >= 1.0 || !std::isfinite(this->i()) ||
        this->i() < 0.0 || this->i() > M_PI || !std::isfinite(this->omega()) || !std::isfinite(this->RAAN()) ||
        !std::isfinite(this->f())) {
        throw std::invalid_argument(
            "KeplerianOrbit requires finite mu > 0, a > 0, 0 <= e < 1 and 0 <= i <= pi with finite angles.");
    }
    this->mean_motion = sqrt(this->mu / this->a()) / this->a();
    this->orbital_period = 2 * M_PI / this->mean_motion;
    this->orbital_energy = -this->mu / 2 / this->a();
    this->r_apogee = this->a() * (1 + this->e());
    this->r_perigee = this->a() * (1 - this->e());
    this->change_f();
    this->orbital_angular_momentum_P = this->position_BP_P.cross(this->velocity_BP_P);
    this->semi_parameter = this->a() * (1.0 - this->e()) * (1.0 + this->e());
    if (!this->position_BP_P.allFinite() || !this->velocity_BP_P.allFinite() ||
        !this->orbital_angular_momentum_P.allFinite() || !std::isfinite(this->h()) || !std::isfinite(this->v()) ||
        !std::isfinite(this->c3()) || !std::isfinite(this->orbital_period) || !std::isfinite(this->orbital_energy) ||
        !std::isfinite(this->r_apogee) || !std::isfinite(this->r_perigee) || !std::isfinite(this->orbit_radius) ||
        !std::isfinite(this->radial_rate) || !std::isfinite(this->true_anomaly_rate) ||
        !std::isfinite(this->eccentric_anomaly) || !std::isfinite(this->mean_anomaly) ||
        !std::isfinite(this->flight_path_angle) || !std::isfinite(this->semi_parameter) ||
        !std::isfinite(1.0 / this->a()) || this->mean_motion <= 0.0 || this->r_perigee <= 0.0 ||
        this->orbit_radius <= 0.0 || this->semi_parameter <= 0.0 || this->true_anomaly_rate <= 0.0) {
        throw std::invalid_argument("KeplerianOrbit derived state is outside the finite numerical range.");
    }
}
/*! @brief Recompute the outputs that depend on the current true anomaly. */
void KeplerianOrbit::change_f()
{
    double r[3];
    double v[3];
    ClassicElements oe = this->oe();
    elem2rv(this->mu, &oe, r, v);
    this->position_BP_P = cArray2EigenVector3d(r);
    this->velocity_BP_P = cArray2EigenVector3d(v);
    this->orbit_radius = this->r();
    this->true_anomaly_rate = this->position_BP_P.cross(this->velocity_BP_P).norm() / pow(this->orbit_radius, 2);
    this->radial_rate = this->position_BP_P.dot(this->velocity_BP_P) / this->orbit_radius;
    this->eccentric_anomaly = f2E(this->f(), this->e());
    this->mean_anomaly = E2M(this->E(), this->e());
    this->flight_path_angle = atan2(this->e() * sin(this->f()), 1 + this->e() * cos(this->f()));
}

void KeplerianOrbit::set_mu(const double mu)
{
    *this = KeplerianOrbit(this->oe(), mu);
}
