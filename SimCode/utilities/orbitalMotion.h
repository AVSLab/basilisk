/*
 *  orbitalMotion.h
 *
 *  Created by Hanspeter Schaub on 6/19/05.
 *
 *  This package provides various orbital
 *  mechanics subroutines using in astrodynamics calculations.
 *
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
    double a;         //!< (m) object semi-major axis
    double alpha;     //!< (1/m) Inverted semi-major axis (extra)
    double e;         //!< (-) Eccentricity of the orbit
    double i;         //!< (r) inclination of the orbital plane
    double Omega;     //!< (r) Right ascension of the ascending node
    double omega;     //!< (r) Argument of periapsis of the orbit
    double f;         //!< (r) True anomaly of the orbit
    double rmag;      //!< (m) Magnitude of the position vector
} classicElements;

#ifdef __cplusplus
extern "C" {
#endif
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
