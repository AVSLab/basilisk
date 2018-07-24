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

#ifndef _ORBITAL_MOTION_0_H_
#define _ORBITAL_MOTION_0_H_

#define N_DEBYE_PARAMETERS 37

/*! \addtogroup Sim Utility Group
 * @{
 */

/* Celestial object being orbited */
typedef enum {
    CELESTIAL_MERCURY,
    CELESTIAL_VENUS,
    CELESTIAL_EARTH,
    CELESTIAL_MOON,
    CELESTIAL_MARS,
    CELESTIAL_PHOBOS,
    CELESTIAL_DEIMOS,
    CELESTIAL_JUPITER,
    CELESTIAL_SATURN,
    CELESTIAL_URANUS,
    CELESTIAL_NEPTUNE,
    CELESTIAL_PLUTO,
    CELESTIAL_SUN,
    MAX_CELESTIAL
} CelestialObject_t;

/*! This structure contains the set of Keplerian orbital elements that define the 
    spacecraft translational state.  It is operated on by the orbital element 
    routines and the OrbElemConvert module.
*/
typedef struct {
    double a;         //!< object semi-major axis
    double e;         //!< Eccentricity of the orbit
    double i;         //!< inclination of the orbital plane
    double Omega;     //!< Right ascension of the ascending node
    double omega;     //!< Argument of periapsis of the orbit
    double f;         //!< True anomaly of the orbit
    double rmag;      //!< Magnitude of the position vector (extra)
    double alpha;     //!< Inverted semi-major axis (extra)
	double rPeriap;   //!< Radius of periapsis (extra)
	double rApoap;    //!< Radius if apoapsis (extra)
} classicElements;

#ifdef __cplusplus
extern "C" {
#endif
    /*
     E = eccentric anomaly
     f = true anomaly
     M = mean anomaly
     H = hyperbolic anomaly
     N = mean hyperbolic anomaly
     */
    double  E2f(double E, double e);
    double  E2M(double E, double e);
    double  f2E(double f, double e);
    double  f2H(double f, double e);
    double  H2f(double H, double e);
    double  H2N(double H, double e);
    double  M2E(double M, double e);
    double  N2H(double N, double e);
    void    elem2rv(double mu, classicElements *elements, double *rVec, double *vVec);
    void    rv2elem(double mu, double *rVec, double *vVec, classicElements *elements);
    
    double  atmosphericDensity(double alt);
    double  debyeLength(double alt);
    void    atmosphericDrag(double Cd, double A, double m, double *rvec, double *vvec, double *advec);
    void    jPerturb(double *rvec, int num, double *ajtot, ...);
    void    solarRad(double A, double m, double *sunvec, double *arvec);
    
#ifdef __cplusplus
}
#endif
/*! @} */
#endif
