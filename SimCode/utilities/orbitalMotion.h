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

typedef struct classicElem {
    double a;
    double alpha;
    double e;
    double i;
    double Omega;
    double omega;
    double f;
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
#endif
