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

#include "orbitalMotion.h"
#include <float.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "architecture/utilities/bsk_Print.h"
#include "astroConstants.h"
#include "linearAlgebra.h"

static int finiteVector3(const double vector[3])
{
    return isfinite(vector[0]) && isfinite(vector[1]) && isfinite(vector[2]);
}

/* Normalize before crossing so the frame test is independent of distance units. */
static int computeHillFrame(const double rc_N[3], const double vc_N[3], double HN[3][3], double* rate)
{
    double radius = hypot(hypot(rc_N[0], rc_N[1]), rc_N[2]);
    double speed = hypot(hypot(vc_N[0], vc_N[1]), vc_N[2]);
    double velocityDirection[3];
    double normal[3];
    if (!finiteVector3(rc_N) || !finiteVector3(vc_N) || !isfinite(radius) || radius <= 0.0 || !isfinite(speed) ||
        speed <= 0.0) {
        return 0;
    }
    for (int index = 0; index < 3; index++) {
        HN[0][index] = rc_N[index] / radius;
        velocityDirection[index] = vc_N[index] / speed;
    }
    v3Cross(HN[0], velocityDirection, normal);
    double sineAngle = v3Norm(normal);
    if (sineAngle <= 8.0 * DBL_EPSILON) {
        return 0;
    }
    v3Scale(1.0 / sineAngle, normal, HN[2]);
    v3Cross(HN[2], HN[0], HN[1]);
    if (rate != NULL) {
        *rate = (speed * sineAngle) / radius;
        if (!isfinite(*rate) || *rate <= 0.0) {
            return 0;
        }
    }
    return 1;
}

/* Shared checked implementation; report controls diagnostics. */
static void hillFrameImpl(double* rc_N, double* vc_N, double HN[3][3], int report)
{
    if (!computeHillFrame(rc_N, vc_N, HN, NULL)) {
        if (report) {
            BSK_PRINT(
                MSG_ERROR, "%s", "hillFrame() requires finite nonzero vectors defining a resolved orbital plane.");
        }
        for (int index = 0; index < 3; index++) {
            v3Set(nan(""), nan(""), nan(""), HN[index]);
        }
    }
}

/* Shared checked implementation; report controls diagnostics. */
static void
hill2rvImpl(double* rc_N, double* vc_N, double* rho_H, double* rhoPrime_H, double* rd_N, double* vd_N, int report)
{
    double HN[3][3];      /* DCM of Hill frame relative to inertial */
    double NH[3][3];      /* DCM of inertial frame relative to Hill */
    double fDot;          /* chief true anomaly rate */
    double omega_HN_H[3]; /* Hill frame angular velocity */
    double rho_N[3];      /* deputy relative to chief vector in N frame components */

    if (!finiteVector3(rho_H) || !finiteVector3(rhoPrime_H) || !computeHillFrame(rc_N, vc_N, HN, &fDot)) {
        goto invalidHillState;
    }
    m33Transpose(HN, NH);
    v3Set(0, 0, fDot, omega_HN_H);

    /* compute inertial deputy position */
    m33MultV3(NH, rho_H, rho_N);
    v3Add(rc_N, rho_N, rd_N);

    /* compute inertial deputy velocity */
    v3Cross(omega_HN_H, rho_H, vd_N);
    v3Add(vd_N, rhoPrime_H, vd_N);
    m33MultV3(NH, vd_N, vd_N);
    v3Add(vd_N, vc_N, vd_N);
    if (finiteVector3(rd_N) && finiteVector3(vd_N)) {
        return;
    }
invalidHillState:
    if (report) {
        BSK_PRINT(MSG_ERROR,
                  "%s",
                  "hill2rv() requires finite states, a resolved chief orbital plane and representable outputs.");
    }
    v3Set(nan(""), nan(""), nan(""), rd_N);
    v3Set(nan(""), nan(""), nan(""), vd_N);
}

/* Shared checked implementation; report controls diagnostics. */
static void
rv2hillImpl(double* rc_N, double* vc_N, double* rd_N, double* vd_N, double* rho_H, double* rhoPrime_H, int report)
{
    double HN[3][3];      /* DCM of Hill frame relative to inertial */
    double fDot;          /* chief true anomaly rate */
    double omega_HN_H[3]; /* Hill frame angular velocity */
    double rho_N[3];      /* deputy relative to chief vector in N frame components */
    double rhoDot_N[3];   /* inertial derivative of deputy/chief vector */
    double rhoDot_H[3];   /* inertial derivative of deputy/chief vector */

    if (!finiteVector3(rd_N) || !finiteVector3(vd_N) || !computeHillFrame(rc_N, vc_N, HN, &fDot)) {
        goto invalidHillState;
    }
    v3Set(0, 0, fDot, omega_HN_H);

    /* compute Hill frame position */
    v3Subtract(rd_N, rc_N, rho_N);
    m33MultV3(HN, rho_N, rho_H);

    /* compute Hill frame velocity */
    v3Subtract(vd_N, vc_N, rhoDot_N);
    m33MultV3(HN, rhoDot_N, rhoDot_H);
    v3Cross(omega_HN_H, rho_H, rhoPrime_H);
    v3Subtract(rhoDot_H, rhoPrime_H, rhoPrime_H);
    if (finiteVector3(rho_H) && finiteVector3(rhoPrime_H)) {
        return;
    }
invalidHillState:
    if (report) {
        BSK_PRINT(MSG_ERROR,
                  "%s",
                  "rv2hill() requires finite states, a resolved chief orbital plane and representable outputs.");
    }
    v3Set(nan(""), nan(""), nan(""), rho_H);
    v3Set(nan(""), nan(""), nan(""), rhoPrime_H);
}

/*!
 * Purpose: Maps eccentric anomaly angles into true anomaly angles.
 *   This function requires the orbit to be either circular or
 *   non-rectilinar elliptic orbit.
 * Inputs:
 *   Ecc = eccentric anomaly (rad)
 *   e = eccentricity (0 <= e < 1)
 * Outputs:
 *   f = true anomaly (rad)
 */
double E2f(double Ecc, double e)
{
    double f;

    if ((e >= 0) && (e < 1)) {
        f = 2 * atan2(sqrt(1 + e) * sin(Ecc / 2), sqrt(1 - e) * cos(Ecc / 2));
    } else {
        f = nan("");
        BSK_PRINT(MSG_ERROR, "E2f() received e = %g. The value of e should be 0 <= e < 1.", e);
    }

    return f;
}

/*!
 * Purpose: Maps the eccentric anomaly angle into the corresponding
 *   mean elliptic anomaly angle.  Both 2D and 1D elliptic
 *   orbit are allowed.
 * Inputs:
 *   Ecc = eccentric anomaly (rad)
 *   e = eccentricity (0 <= e < 1)
 * Outputs:
 *   M = mean elliptic anomaly (rad)
 */
double E2M(double Ecc, double e)
{
    double M;

    if ((e >= 0) && (e < 1)) {
        M = Ecc - e * sin(Ecc);
    } else {
        M = nan("");
        BSK_PRINT(MSG_ERROR, "E2M() received e = %g. The value of e should be 0 <= e < 1.", e);
    }

    return M;
}

/*!
 * Purpose: Maps true anomaly angles into eccentric anomaly angles.
 *   This function requires the orbit to be either circular or
 *   non-rectilinar elliptic orbit.
 * Inputs:
 *   f = true anomaly angle (rad)
 *   e = eccentricity (0 <= e < 1)
 * Outputs:
 *   Ecc = eccentric anomaly (rad)
 */
double f2E(double f, double e)
{
    double Ecc;

    if ((e >= 0) && (e < 1)) {
        Ecc = 2 * atan2(sqrt(1 - e) * sin(f / 2), sqrt(1 + e) * cos(f / 2));
    } else {
        Ecc = nan("");
        BSK_PRINT(MSG_ERROR, "f2E() received e = %g. The value of e should be 0 <= e < 1.", e);
    }

    return Ecc;
}

/*!
 * Purpose: Maps true anomaly angles into hyperbolic anomaly angles.
 *   This function requires the orbit to be hyperbolic
 * Inputs:
 *   f = true anomaly angle (rad)
 *   e = eccentricity (e > 1)
 * Outputs:
 *   H = hyperbolic anomaly (rad)
 */
double f2H(double f, double e)
{
    double H;

    if (e > 1) {
        H = 2 * atanh(sqrt((e - 1) / (e + 1)) * tan(f / 2));
    } else {
        H = nan("");
        BSK_PRINT(MSG_ERROR, "f2H() received e = %g. The value of e should be 1 < e.", e);
    }

    return H;
}

/*!
 * Purpose: Maps hyperbolic anomaly angles into true anomaly angles.
 *   This function requires the orbit to be hyperbolic
 * Inputs:
 *   H = hyperbolic anomaly (rad)
 *   e = eccentricity (e > 1)
 * Outputs:
 *   f = true anomaly angle (rad)
 */
double H2f(double H, double e)
{
    double f;

    if (e > 1) {
        f = 2 * atan(sqrt((e + 1) / (e - 1)) * tanh(H / 2));
    } else {
        f = nan("");
        BSK_PRINT(MSG_ERROR, "H2f() received e = %g. The value of e should be 1 < e.", e);
    }

    return f;
}

/*!
 * Purpose: Maps the hyperbolic anomaly angle H into the corresponding
 *   mean hyperbolic anomaly angle N.
 * Inputs:
 *   H = hyperbolic anomaly (rad)
 *   e = eccentricity (e > 1)
 * Outputs:
 *   N = mean hyperbolic anomaly (rad)
 */
double H2N(double H, double e)
{
    double N;

    if (e > 1) {
        N = e * sinh(H) - H;
    } else {
        N = nan("");
        BSK_PRINT(MSG_ERROR, "H2N() received e = %g. The value of e should be 1 < e.", e);
    }

    return N;
}

/**
 * @brief Evaluate E - sin(E) or sinh(H) - H without small-angle cancellation.
 * @param anomaly Nonnegative eccentric or hyperbolic anomaly [rad].
 * @param hyperbolic Nonzero for the hyperbolic equation, zero for the elliptic equation.
 * @return The nonlinear anomaly term [rad].
 */
static double keplerAnomalyDifference(double anomaly, int hyperbolic)
{
    if (anomaly > 0.5) {
        return hyperbolic ? sinh(anomaly) - anomaly : anomaly - sin(anomaly);
    }
    double multiplier = anomaly * anomaly * (hyperbolic ? 1.0 : -1.0);
    double term = anomaly * anomaly * anomaly / 6.0;
    double difference = term;
    /* Through order 17: the omitted term is below round-off for |anomaly| <= 0.5 rad. */
    for (int index = 2; index < 9; ++index) {
        term *= multiplier / ((2 * index) * (2 * index + 1));
        difference += term;
    }
    return difference;
}

/**
 * @brief Solve the elliptic Kepler equation using bracketed Newton iteration.
 * @param M Finite mean elliptic anomaly [rad].
 * @param e Finite eccentricity, 0 <= e < 1.
 * @return Eccentric anomaly [rad], or NaN with an error diagnostic for invalid
 * inputs or failure to converge within 200 iterations.
 * @note The result preserves the revolutions in M. Very large unwrapped anomalies
 * are limited by input/output angle resolution.
 */
double M2E(double M, double e)
{
    if (!isfinite(M) || !isfinite(e) || e < 0.0 || e >= 1.0) {
        BSK_PRINT(MSG_ERROR, "M2E() requires finite M and e with 0 <= e < 1; received M=%g, e=%g", M, e);
        return nan("");
    }
    if (e == 0.0 || M == 0.0) {
        return M;
    }

    /* Let libm reduce the angle: remainder(M, 2*pi) loses the small offset near
       whole revolutions, which is amplified when eccentricity approaches one. */
    double reducedMean = fabs(M) <= MPI ? M : atan2(sin(M), cos(M));
    double mean = fabs(reducedMean);
    double lower = mean;
    double upper = MPI; // [rad]
    double anomaly = mean;
    const double relativeTolerance = 4.0 * DBL_EPSILON;
    for (int count = 0; count < 200; ++count) {
        double residual = (1.0 - e) * anomaly + e * keplerAnomalyDifference(anomaly, 0) - mean;
        double halfSine = sin(0.5 * anomaly);
        double derivative = (1.0 - e) + 2.0 * e * halfSine * halfSine;
        double correction = residual / derivative;
        if (fabs(correction) <= relativeTolerance * fabs(anomaly)) {
            return M + copysign(anomaly - mean, reducedMean);
        }
        if (residual > 0.0) {
            upper = anomaly;
        } else {
            lower = anomaly;
        }
        double candidate = anomaly - correction;
        if (!(candidate > lower && candidate < upper)) {
            candidate = 0.5 * (lower + upper);
        }
        if (candidate == anomaly) {
            return M + copysign(anomaly - mean, reducedMean);
        }
        anomaly = candidate;
    }
    BSK_PRINT(MSG_ERROR, "M2E() failed to converge for M=%g, e=%g", M, e);
    return nan("");
}

/**
 * @brief Solve the hyperbolic Kepler equation using bracketed Newton iteration.
 * @param N Finite mean hyperbolic anomaly [rad].
 * @param e Finite eccentricity, e > 1.
 * @return Hyperbolic anomaly [rad], or NaN with an error diagnostic for invalid
 * inputs or failure to converge within 200 iterations.
 * @note The equation is scaled to avoid overflow for large finite mean anomalies
 * and evaluated without subtractive cancellation near the parabolic limit.
 */
double N2H(double N, double e)
{
    if (!isfinite(N) || !isfinite(e) || e <= 1.0) {
        BSK_PRINT(MSG_ERROR, "N2H() requires finite N and e with e > 1; received N=%g, e=%g", N, e);
        return nan("");
    }
    if (N == 0.0) {
        return N;
    }

    double mean = fabs(N) / e;
    double lower = asinh(mean);
    double upper = lower + 2.0; // [rad]; brackets the positive root for every e > 1
    double anomaly = lower;
    const double relativeTolerance = 4.0 * DBL_EPSILON;
    for (int count = 0; count < 200; ++count) {
        double residual;
        double derivative;
        if (anomaly <= 1.0) {
            /* Divide by e and retain the small linear term when e is close to one. */
            double linear = (e - 1.0) / e;
            residual = linear * anomaly + keplerAnomalyDifference(anomaly, 1) - mean;
            double halfSine = sinh(0.5 * anomaly);
            derivative = linear + 2.0 * halfSine * halfSine;
        } else {
            /* Divide both f and f' by e*exp(H), without forming either large factor. */
            double inverseExp = exp(-anomaly);
            residual = -0.5 * expm1(-2.0 * anomaly) - (anomaly / e + mean) * inverseExp;
            derivative = 0.5 * (1.0 + inverseExp * inverseExp) - inverseExp / e;
        }
        double correction = residual / derivative;
        if (fabs(correction) <= relativeTolerance * fabs(anomaly)) {
            return copysign(anomaly, N);
        }
        if (residual > 0.0) {
            upper = anomaly;
        } else {
            lower = anomaly;
        }
        double candidate = anomaly - correction;
        if (!(candidate > lower && candidate < upper)) {
            candidate = 0.5 * (lower + upper);
        }
        if (candidate == anomaly) {
            return copysign(anomaly, N);
        }
        anomaly = candidate;
    }
    BSK_PRINT(MSG_ERROR, "N2H() failed to converge for N=%g, e=%g", N, e);
    return nan("");
}

/**
 * @brief Convert classical elements to a finite inertial Cartesian state.
 * @param mu Positive gravitational parameter [L^3/s^2].
 * @param elements Orbital elements, with distances in L and angles in radians.
 * @param[out] rVec Inertial position vector [L].
 * @param[out] vVec Inertial velocity vector [L/s].
 * @note Inclination must satisfy 0 <= i <= pi. Ellipses use a > 0, e < 1; hyperbolas use a < 0, e > 1. No
 * eccentricity tolerance selects a conic. Planar parabolas use e = 1, a = 0,
 * rPeriap > 0 and true anomaly f.
 * @note Rectilinear ellipses use e = 1, a > 0 and f = eccentric anomaly E,
 * with outward motion for sin(E) > 0. Rectilinear hyperbolas use e = 1,
 * a < 0 and f = signed hyperbolic anomaly H, outward for H > 0.
 * Rectilinear parabolas use e = 1, a = rPeriap = 0, rmag > 0 and f = 0
 * for outward motion or f = pi for inward motion. For radial states the
 * perifocal x axis is the outward radial ray; the plane about it is arbitrary.
 * Extra fields are read only where required by these conventions.
 * @note Invalid/nonfinite inputs, collisions, open-conic anomalies at or beyond
 * infinity, and numerical overflow produce a diagnostic and all-NaN vectors.
 */
void elem2rv(double mu, ClassicElements* elements, double* rVec, double* vVec)
{
    const double a = elements->a;
    const double e = elements->e;
    const double i = elements->i;
    const double node = elements->Omega;
    const double periapsis = elements->omega;
    double anomaly = elements->f;
    double pHat[3], qHat[3];
    double radius, radialSpeed, parameter;
    if (!isfinite(mu) || mu <= 0.0 || !isfinite(a) || !isfinite(e) || e < 0.0 || !isfinite(i) || i < 0.0 || i > MPI ||
        !isfinite(node) || !isfinite(periapsis) || !isfinite(anomaly)) {
        goto invalid;
    }
    v3Set(cos(node) * cos(periapsis) - sin(node) * sin(periapsis) * cos(i),
          sin(node) * cos(periapsis) + cos(node) * sin(periapsis) * cos(i),
          sin(periapsis) * sin(i),
          pHat);
    v3Set(-cos(node) * sin(periapsis) - sin(node) * cos(periapsis) * cos(i),
          -sin(node) * sin(periapsis) + cos(node) * cos(periapsis) * cos(i),
          cos(periapsis) * sin(i),
          qHat);
    if (e == 1.0 && (a != 0.0 || elements->rPeriap == 0.0)) {
        if (a > 0.0) {
            /* Let libm reduce the angle; a rounded 2*pi divisor loses phase. */
            const double half = 0.5 * anomaly;
            const double factor = sin(half);
            radius = 2.0 * a * factor * factor;
            if (radius <= 0.0) {
                goto invalid;
            }
            radialSpeed = sqrt(mu / a) * cos(half) / factor;
        } else if (a < 0.0) {
            const double factor = sinh(0.5 * anomaly);
            radius = -2.0 * a * factor * factor;
            if (radius <= 0.0) {
                goto invalid;
            }
            radialSpeed = copysign(sqrt(mu / -a) / tanh(0.5 * fabs(anomaly)), anomaly);
        } else {
            radius = elements->rmag;
            if (!isfinite(radius) || radius <= 0.0 || (anomaly != 0.0 && anomaly != MPI)) {
                goto invalid;
            }
            radialSpeed = sqrt(2.0 * (mu / radius)) * (anomaly == 0.0 ? 1.0 : -1.0);
        }
        v3Scale(radius, pHat, rVec);
        v3Scale(radialSpeed, pHat, vVec);
    } else {
        if (e == 1.0 && a == 0.0) {
            if (!isfinite(elements->rPeriap) || elements->rPeriap <= 0.0) {
                goto invalid;
            }
            parameter = 2.0 * elements->rPeriap;
        } else {
            parameter = a * (1.0 - e) * (1.0 + e);
        }
        const double cosine = cos(anomaly);
        const double sine = sin(anomaly);
        const double principalAnomaly = atan2(sine, cosine);
        if (!isfinite(parameter) || parameter <= 0.0 || (e >= 1.0 && fabs(principalAnomaly) >= acos(-1.0 / e))) {
            goto invalid;
        }
        const double halfCosine = cos(0.5 * anomaly);
        /* Retain 1-e and e-1 near apoapsis/infinity. */
        const double denominator = cosine < 0.0 ? (1.0 - e) + 2.0 * e * halfCosine * halfCosine : 1.0 + e * cosine;
        if (denominator <= 0.0) {
            goto invalid;
        }
        radius = parameter / denominator;
        const double transverse = cosine < 0.0 ? (e - 1.0) + 2.0 * halfCosine * halfCosine : e + cosine;
        const double speed = sqrt(mu / parameter);
        for (int index = 0; index < 3; ++index) {
            rVec[index] = radius * (cosine * pHat[index] + sine * qHat[index]);
            vVec[index] = speed * (-sine * pHat[index] + transverse * qHat[index]);
        }
    }
    for (int index = 0; index < 3; ++index) {
        if (!isfinite(rVec[index]) || !isfinite(vVec[index])) {
            goto invalid;
        }
    }
    return;
invalid:
    BSK_PRINT(MSG_ERROR, "%s", "elem2rv() invalid conic, collision, anomaly at/beyond infinity, or numerical overflow");
    v3Set(nan(""), nan(""), nan(""), rVec);
    v3Set(nan(""), nan(""), nan(""), vVec);
}

/**
 * @brief Convert a Cartesian state to classical orbital elements.
 * @param mu Positive gravitational parameter [L^3/s^2].
 * @param rVec Nonzero inertial position vector [L].
 * @param vVec Inertial velocity vector [L/s].
 * @param[out] elements Orbital elements, with distances in L and angles in radians.
 * @note Inputs must be finite and use consistent distance units. Inclination is
 * in [0, pi]; Omega, omega, and nonradial f are in [0, 2*pi), including hyperbolic f.
 * Degenerate conics use the conventions documented in elem2rv().
 * @note A dimensionless tolerance of 1e-12 selects singular-angle conventions.
 * For e <= tolerance, omega = 0 and f is the argument of latitude. For
 * sin(i) <= tolerance, Omega = 0 and i is set to 0 or pi; omega measures
 * periapsis from the inertial x axis in the direction of orbital motion.
 * If both conditions hold, f measures position from that axis in the direction
 * of motion. Replacing unresolved directions introduces relative state errors
 * on the order of the singularity tolerance.
 * @note Parabolic energy and zero transverse velocity are detected within
 * 8 machine epsilons of the local energy and speed scales, respectively.
 * Radial states use a deterministic plane about the outward radial ray.
 * Near zero energy, relative accuracy of a and state reconstruction from
 * rounded a, e degrades as machine epsilon divided by abs(1-e).
 * Nonradial finite-energy states whose eccentricity rounds to one or the wrong
 * side of one cannot be encoded consistently. These, invalid inputs and numerical
 * overflow produce a diagnostic and all-NaN elements.
 */
void rv2elem(double mu, double* rVec, double* vVec, ClassicElements* elements)
{
    const double singularityTolerance = 1e-12; // [-]
    const double roundoff = 8.0 * DBL_EPSILON;
    double rHat[3], scaledV[3], scaledH[3], hHat[3], eVec[3], eHat[3], nHat[3], temp[3];
    double radius, speedScale, speedSquared, scaledEnergy, momentum, parameter, nodeMagnitude;
    int radial;
    if (!isfinite(mu) || mu <= 0.0) {
        goto invalid;
    }
    for (int index = 0; index < 3; ++index) {
        if (!isfinite(rVec[index]) || !isfinite(vVec[index])) {
            goto invalid;
        }
    }
    radius = hypot(hypot(rVec[0], rVec[1]), rVec[2]);
    if (!isfinite(radius) || radius <= 0.0) {
        goto invalid;
    }
    speedScale = sqrt(mu / radius);
    if (!isfinite(speedScale) || speedScale <= 0.0) {
        goto invalid;
    }
    for (int index = 0; index < 3; ++index) {
        rHat[index] = rVec[index] / radius;
        scaledV[index] = vVec[index] / speedScale;
    }
    speedSquared = v3Dot(scaledV, scaledV);
    if (!isfinite(speedSquared)) {
        goto invalid;
    }
    scaledEnergy = 2.0 - speedSquared;
    if (fabs(scaledEnergy) <= roundoff * fmax(2.0, speedSquared)) {
        scaledEnergy = 0.0; // [-]; unresolved energy at the parabolic limit
    }
    v3Cross(rHat, scaledV, scaledH);
    momentum = hypot(hypot(scaledH[0], scaledH[1]), scaledH[2]);
    radial = momentum <= roundoff * sqrt(speedSquared);
    elements->rmag = radius;
    elements->alpha = scaledEnergy / radius;
    elements->a = scaledEnergy != 0.0 ? radius / scaledEnergy : 0.0; // [L]
    parameter = radius * momentum * momentum;
    if (radial) {
        elements->e = 1.0;
        elements->rPeriap = 0.0; // [L]
        /* Choose the least-aligned inertial axis for a well-conditioned plane. */
        int axis = 0;
        for (int index = 1; index < 3; ++index) {
            if (fabs(rHat[index]) < fabs(rHat[axis])) {
                axis = index;
            }
        }
        v3SetZero(temp);
        temp[axis] = 1.0;
        v3Cross(rHat, temp, hHat);
        v3Normalize(hHat, hHat);
        v3Copy(rHat, eHat);
    } else {
        v3Scale(1.0 / momentum, scaledH, hHat);
        v3Cross(scaledV, scaledH, eVec);
        v3Subtract(eVec, rHat, eVec);
        elements->e = hypot(hypot(eVec[0], eVec[1]), eVec[2]);
        if (scaledEnergy == 0.0) {
            elements->e = 1.0;
        } else if ((scaledEnergy > 0.0 && elements->e >= 1.0) || (scaledEnergy < 0.0 && elements->e <= 1.0)) {
            BSK_PRINT(MSG_ERROR, "%s", "rv2elem() eccentricity cannot resolve the finite-energy conic");
            goto invalid;
        }
        elements->rPeriap = parameter / (1.0 + elements->e);
        v3Normalize(eVec, eHat);
    }
    elements->rApoap = scaledEnergy != 0.0 ? elements->a * (1.0 + elements->e) : 0.0; // [L]
    v3Set(-hHat[1], hHat[0], 0.0, nHat);
    nodeMagnitude = hypot(nHat[0], nHat[1]);
    if (nodeMagnitude <= singularityTolerance) {
        v3Set(1.0, 0.0, 0.0, nHat);
        elements->i = hHat[2] >= 0.0 ? 0.0 : MPI; // [rad]
    } else {
        v3Scale(1.0 / nodeMagnitude, nHat, nHat);
        elements->i = atan2(nodeMagnitude, hHat[2]);
    }
    if (elements->e <= singularityTolerance) {
        v3Copy(nHat, eHat);
    }
    elements->Omega = atan2(nHat[1], nHat[0]);
    v3Cross(nHat, eHat, temp);
    elements->omega = atan2(v3Dot(hHat, temp), v3Dot(nHat, eHat));
    v3Cross(eHat, rHat, temp);
    elements->f = atan2(v3Dot(hHat, temp), v3Dot(eHat, rHat));
    double* angles[3] = { &elements->Omega, &elements->omega, &elements->f };
    for (int index = 0; index < 3; ++index) {
        if (*angles[index] < 0.0) {
            *angles[index] += 2.0 * MPI;
        }
        if (*angles[index] >= 2.0 * MPI) {
            *angles[index] = 0.0; // [rad]; rounding can reach the upper endpoint
        }
    }
    if (radial) {
        const double radialSpeed = v3Dot(rHat, scaledV);
        if (scaledEnergy > 0.0) {
            elements->f = atan2(radialSpeed * sqrt(scaledEnergy), 1.0 - scaledEnergy);
            if (elements->f < 0.0) {
                elements->f += 2.0 * MPI;
            }
        } else if (scaledEnergy < 0.0) {
            elements->f = copysign(2.0 * asinh(sqrt(-0.5 * scaledEnergy)), radialSpeed);
        } else {
            elements->f = radialSpeed >= 0.0 ? 0.0 : MPI; // [rad]; radial direction flag
        }
    }
    if (!isfinite(elements->a) || !isfinite(elements->alpha) || !isfinite(elements->e) ||
        !isfinite(elements->rPeriap) || !isfinite(elements->rApoap) || !isfinite(elements->i) ||
        !isfinite(elements->Omega) || !isfinite(elements->omega) || !isfinite(elements->f)) {
        goto invalid;
    }
    return;
invalid:
    BSK_PRINT(MSG_ERROR, "%s", "rv2elem() invalid state or unrepresentable orbital elements");
    elements->a = elements->e = elements->i = elements->Omega = elements->omega = elements->f = nan("");
    elements->rmag = elements->alpha = elements->rPeriap = elements->rApoap = nan("");
}

/*!
 * Purpose: This program computes the atmospheric density based on altitude
 *   supplied by user.  This function uses a curve fit based on
 *   atmospheric data from the Standard Atmosphere 1976 Data. This
 *   function is valid for altitudes ranging from 100km to 1000km.
 *
 *   Note: This code can only be applied to spacecraft orbiting the Earth
 * Inputs:
 *   alt = altitude in km
 * Outputs:
 *   density = density at the given altitude in kg/m^3
 */
double atmosphericDensity(double alt)
{
    double logdensity;
    double density;
    double val;

    /* Smooth exponential drop-off after 1000 km */
    if (alt > 1000.) {
        logdensity = (-7e-05) * alt - 14.464;
        density = pow(10., logdensity);
        return density;
    }

    /* Calculating the density based on a scaled 6th order polynomial fit to the log of density */
    val = (alt - 526.8000) / 292.8563;
    logdensity = 0.34047 * pow(val, 6) - 0.5889 * pow(val, 5) - 0.5269 * pow(val, 4) + 1.0036 * pow(val, 3) +
                 0.60713 * pow(val, 2) - 2.3024 * val - 12.575;

    /* Calculating density by raising 10 to the log of density */
    density = pow(10., logdensity);

    return density;
}

/*!
 * @brief Compute the Debye length for a given altitude.
 * @param alt Altitude [km], valid from 200 km through 35000 km.
 * @return Debye length [m], or NaN if alt is non-finite or outside the valid range.
 * @note Values above 1000 km are highly speculative.
 */
double debyeLength(double alt)
{
    double debyedist;
    double a;
    double X[N_DEBYE_PARAMETERS] = { 200,  250,  300,  350,  400,  450,  500,  550,  600,  650,  700,  750,  800,
                                     850,  900,  950,  1000, 1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450,
                                     1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950, 2000 };
    double Y[N_DEBYE_PARAMETERS] = { 5.64E-03, 3.92E-03, 3.24E-03, 3.59E-03, 4.04E-03, 4.28E-03, 4.54E-03, 5.30E-03,
                                     6.55E-03, 7.30E-03, 8.31E-03, 8.38E-03, 8.45E-03, 9.84E-03, 1.22E-02, 1.37E-02,
                                     1.59E-02, 1.75E-02, 1.95E-02, 2.09E-02, 2.25E-02, 2.25E-02, 2.25E-02, 2.47E-02,
                                     2.76E-02, 2.76E-02, 2.76E-02, 2.76E-02, 2.76E-02, 2.76E-02, 2.76E-02, 3.21E-02,
                                     3.96E-02, 3.96E-02, 3.96E-02, 3.96E-02, 3.96E-02 };
    int i;

    /* Flat debyeLength length for altitudes above 2000 km */
    if ((alt > 2000.0) && (alt <= 30000.0)) {
        alt = 2000.0;
    } else if ((alt > 30000.0) && (alt <= 35000.0)) {
        debyedist = 0.1 * alt - 2999.7;
        return debyedist;
    } else if (!isfinite(alt) || (alt < 200.0) || (alt > 35000.0)) {
        BSK_PRINT(
            MSG_ERROR, "debyeLength() received alt = %g\nThe value of alt should be in the range of [200 35000]", alt);
        debyedist = nan("");
        return debyedist;
    }

    /* Interpolation of data */
    for (i = 0; i < N_DEBYE_PARAMETERS - 1; i++) {
        /* Include the upper endpoint to keep the final table lookup in bounds. */
        if (X[i + 1] >= alt) {
            break;
        }
    }
    a = (alt - X[i]) / (X[i + 1] - X[i]);
    debyedist = Y[i] + a * (Y[i + 1] - Y[i]);

    return debyedist;
}

/**
 * @brief Estimate Earth's atmospheric drag acceleration opposite the supplied velocity.
 * @param Cd Finite, nonnegative drag coefficient [-].
 * @param A Finite, nonnegative projected area [m^2].
 * @param m Finite, positive spacecraft mass [kg].
 * @param rvec Finite inertial position three-vector [km], above Earth's equatorial radius.
 * @param vvec Finite inertial velocity three-vector [km/s].
 * @param[out] advec Drag acceleration in the input frame [km/s^2].
 * @note Assumes an atmosphere at rest in the input inertial frame; no rotation or
 * wind correction is applied. Uses atmosphericDensity(), whose fit is valid from
 * 100 to 1000 km; positive altitudes below this range are extrapolated, and above
 * 1000 km the existing exponential tail is retained, without a hard cutoff.
 * @note Zero speed, area, coefficient, or computed density gives zero acceleration
 * after input validation. Invalid inputs, unrepresentable norms, or acceleration
 * overflow produce a diagnostic and three NaNs. Acceleration underflow is allowed.
 * The output may alias either input vector; all arrays must hold three doubles.
 */
void atmosphericDrag(double Cd, double A, double m, double* rvec, double* vvec, double* advec)
{
    if (!isfinite(Cd) || Cd < 0.0 || !isfinite(A) || A < 0.0 || !isfinite(m) || m <= 0.0) {
        BSK_PRINT(MSG_ERROR, "%s", "atmosphericDrag() requires finite Cd >= 0, A >= 0 and m > 0.");
        v3Set(nan(""), nan(""), nan(""), advec);
        return;
    }
    for (int index = 0; index < 3; ++index) {
        if (!isfinite(rvec[index]) || !isfinite(vvec[index])) {
            BSK_PRINT(MSG_ERROR, "%s", "atmosphericDrag() requires finite position and velocity three-vectors.");
            v3Set(nan(""), nan(""), nan(""), advec);
            return;
        }
    }
    const double radius = hypot(hypot(rvec[0], rvec[1]), rvec[2]);
    const double speed = hypot(hypot(vvec[0], vvec[1]), vvec[2]);
    if (!isfinite(radius) || !isfinite(speed) || radius <= REQ_EARTH) {
        BSK_PRINT(MSG_ERROR, "%s", "atmosphericDrag() requires finite vector norms and positive altitude above Earth.");
        v3Set(nan(""), nan(""), nan(""), advec);
        return;
    }
    if (Cd == 0.0 || A == 0.0 || speed == 0.0) {
        v3SetZero(advec);
        return;
    }
    const double density = atmosphericDensity(radius - REQ_EARTH);
    if (density == 0.0) {
        v3SetZero(advec);
        return;
    }

    // a = -0.5*kmToM*density*Cd*A/m*|v|*v: no division by speed.
    // Keep powers of two separate so intermediate products cannot overflow or
    // underflow when the final acceleration is representable.
    const double kmToM = 1000.0; // [m/km]
    const double factors[] = { 0.5 * kmToM, density, Cd, A, speed };
    double mantissa = 1.0;
    int exponent = 0;
    for (int index = 0; index < 5; ++index) {
        int factorExponent;
        mantissa *= frexp(factors[index], &factorExponent);
        exponent += factorExponent;
    }
    int massExponent;
    mantissa /= frexp(m, &massExponent);
    exponent -= massExponent;
    double acceleration[3];
    for (int index = 0; index < 3; ++index) {
        int velocityExponent;
        const double velocityMantissa = frexp(vvec[index], &velocityExponent);
        acceleration[index] = ldexp(-mantissa * velocityMantissa, exponent + velocityExponent);
        if (!isfinite(acceleration[index])) {
            BSK_PRINT(MSG_ERROR, "%s", "atmosphericDrag() acceleration is outside the finite numerical range.");
            v3Set(nan(""), nan(""), nan(""), advec);
            return;
        }
    }
    v3Copy(acceleration, advec);
}

/**
 * @brief Compute Earth's zonal perturbation acceleration through the requested degree.
 * @param rvec Finite, nonzero planet-centered position [km], with z along the spin axis.
 * @param num Maximum included degree, from 2 through 6 (inclusive).
 * @param[out] ajtot Perturbation acceleration in the input frame [km/s^2].
 * @note This fixed three-argument entry point selects Earth. For another body,
 * use jPerturbForPlanet(rvec, num, ajtot, planetID).
 * See jPerturbForPlanet() for model assumptions and failure behavior.
 */
void jPerturbEarth(double* rvec, int num, double* ajtot)
{
    jPerturbForPlanet(rvec, num, ajtot, CELESTIAL_EARTH);
}

/**
 * @brief Compute a selected body's zonal perturbation acceleration.
 * @param rvec Finite, nonzero planet-centered position [km], with z along the spin axis.
 * @param num Maximum included degree, from 2 through 6 (inclusive).
 * @param[out] ajtot Perturbation acceleration in the input frame [km/s^2].
 * @param planetID Body whose coefficients are used; unsupported identifiers are errors.
 * @note Earth includes J2 through J6. Mercury, Venus, Moon, Mars, Jupiter,
 * Saturn, Uranus and Neptune include J2 only. Sun and Pluto use a zero-zonal
 * approximation and return a zero vector. Phobos and Deimos are unsupported.
 * These approximations omit central gravity, tesseral terms and other forces.
 * @note The external gravity expansion is intended for positions outside the
 * body's reference radius; the surface boundary is not enforced. All inputs
 * are validated even for zero-zonal models. Invalid inputs or nonfinite results
 * produce a diagnostic and an all-NaN vector. Input/output arrays may alias.
 */
void jPerturbForPlanet(double* rvec, int num, double* ajtot, CelestialObject_t planetID)
{
    double mu;
    double req;
    double J2;
    double J3 = 0.0; // [-]
    double J4 = 0.0; // [-]
    double J5 = 0.0; // [-]
    double J6 = 0.0; // [-]
    const double x = rvec[0];
    const double y = rvec[1];
    const double z = rvec[2];
    const double r = hypot(hypot(x, y), z);
    double temp[3];
    double temp2[3];

    if (num < 2 || num > 6 || !isfinite(x) || !isfinite(y) || !isfinite(z) || !isfinite(r) || r <= 0.0) {
        BSK_PRINT(MSG_ERROR, "%s", "jPerturbForPlanet() requires degree 2..6 and a finite nonzero position");
        v3Set(nan(""), nan(""), nan(""), ajtot);
        return;
    }

    v3SetZero(ajtot);

    switch (planetID) {
        case CELESTIAL_MERCURY:
            mu = MU_MERCURY;
            req = REQ_MERCURY;
            J2 = J2_MERCURY;
            break;

        case CELESTIAL_VENUS:
            mu = MU_VENUS;
            req = REQ_VENUS;
            J2 = J2_VENUS;
            break;

        case CELESTIAL_MOON:
            mu = MU_MOON;
            req = REQ_MOON;
            J2 = J2_MOON;
            break;

        case CELESTIAL_MARS:
            mu = MU_MARS;
            req = REQ_MARS;
            J2 = J2_MARS;
            break;

        case CELESTIAL_JUPITER:
            mu = MU_JUPITER;
            req = REQ_JUPITER;
            J2 = J2_JUPITER;
            break;

        case CELESTIAL_SATURN:
            mu = MU_SATURN;
            req = REQ_SATURN;
            J2 = J2_SATURN;
            break;

        case CELESTIAL_URANUS:
            mu = MU_URANUS;
            req = REQ_URANUS;
            J2 = J2_URANUS;
            break;

        case CELESTIAL_NEPTUNE:
            mu = MU_NEPTUNE;
            req = REQ_NEPTUNE;
            J2 = J2_NEPTUNE;
            break;

        case CELESTIAL_PLUTO:
        case CELESTIAL_SUN:
            return;

        case CELESTIAL_EARTH:
            mu = MU_EARTH;
            req = REQ_EARTH;
            J2 = J2_EARTH;
            J3 = J3_EARTH;
            J4 = J4_EARTH;
            J5 = J5_EARTH;
            J6 = J6_EARTH;
            break;

        default:
            BSK_PRINT(MSG_ERROR, "jPerturbForPlanet() unsupported planet identifier %d", (int)planetID);
            v3Set(nan(""), nan(""), nan(""), ajtot);
            return;
    }

    /* Calculating the total acceleration based on user input */
    if (num >= 2) {
        v3Set((1.0 - 5.0 * pow(z / r, 2.0)) * (x / r),
              (1.0 - 5.0 * pow(z / r, 2.0)) * (y / r),
              (3.0 - 5.0 * pow(z / r, 2.0)) * (z / r),
              ajtot);
        v3Scale(-3.0 / 2.0 * J2 * (mu / r / r) * pow(req / r, 2.0), ajtot, ajtot);
    }
    if (num >= 3 && J3 != 0.0) {
        v3Set(5.0 * (7.0 * pow(z / r, 3.0) - 3.0 * (z / r)) * (x / r),
              5.0 * (7.0 * pow(z / r, 3.0) - 3.0 * (z / r)) * (y / r),
              -3.0 * (10.0 * pow(z / r, 2.0) - (35.0 / 3.0) * pow(z / r, 4.0) - 1.0),
              temp);
        v3Scale(1.0 / 2.0 * J3 * (mu / r / r) * pow(req / r, 3.0), temp, temp2);
        v3Add(ajtot, temp2, ajtot);
    }
    if (num >= 4 && J4 != 0.0) {
        v3Set((3.0 - 42.0 * pow(z / r, 2.0) + 63.0 * pow(z / r, 4.0)) * (x / r),
              (3.0 - 42.0 * pow(z / r, 2.0) + 63.0 * pow(z / r, 4.0)) * (y / r),
              (15.0 - 70.0 * pow(z / r, 2) + 63.0 * pow(z / r, 4.0)) * (z / r),
              temp);
        v3Scale(5.0 / 8.0 * J4 * (mu / r / r) * pow(req / r, 4.0), temp, temp2);
        v3Add(ajtot, temp2, ajtot);
    }
    if (num >= 5 && J5 != 0.0) {
        v3Set(3.0 * (35.0 * (z / r) - 210.0 * pow(z / r, 3.0) + 231.0 * pow(z / r, 5.0)) * (x / r),
              3.0 * (35.0 * (z / r) - 210.0 * pow(z / r, 3.0) + 231.0 * pow(z / r, 5.0)) * (y / r),
              -(15.0 - 315.0 * pow(z / r, 2.0) + 945.0 * pow(z / r, 4.0) - 693.0 * pow(z / r, 6.0)),
              temp);
        v3Scale(1.0 / 8.0 * J5 * (mu / r / r) * pow(req / r, 5.0), temp, temp2);
        v3Add(ajtot, temp2, ajtot);
    }
    if (num >= 6 && J6 != 0.0) {
        v3Set((35.0 - 945.0 * pow(z / r, 2) + 3465.0 * pow(z / r, 4.0) - 3003.0 * pow(z / r, 6.0)) * (x / r),
              (35.0 - 945.0 * pow(z / r, 2.0) + 3465.0 * pow(z / r, 4.0) - 3003.0 * pow(z / r, 6.0)) * (y / r),
              -(3003.0 * pow(z / r, 6.0) - 4851.0 * pow(z / r, 4.0) + 2205.0 * pow(z / r, 2.0) - 245.0) * (z / r),
              temp);
        v3Scale(-1.0 / 16.0 * J6 * (mu / r / r) * pow(req / r, 6.0), temp, temp2);
        v3Add(ajtot, temp2, ajtot);
    }

    if (!isfinite(ajtot[0]) || !isfinite(ajtot[1]) || !isfinite(ajtot[2])) {
        BSK_PRINT(MSG_ERROR, "%s", "jPerturbForPlanet() acceleration is outside the finite numerical range");
        v3Set(nan(""), nan(""), nan(""), ajtot);
    }
}

/*!
 * Purpose: Computes the inertial solar radiation force vectors
 *   based on cross-sectional Area and mass of the spacecraft
 *   and the position vector of the planet to the sun.
 *   Note: It is assumed that the solar radiation pressure decreases
 *   quadratically with distance from sun (in AU)
 *
 *   Solar Radiation Equations obtained from
 *   Earth Space and Planets Journal Vol. 51, 1999 pp. 979-986
 * Inputs:
 *   A = Cross-sectional area of the spacecraft that is facing
 *             the sun in m^2.
 *   m = The mass of the spacecraft in kg.
 *   sunvec = Position vector to the Sun in units of AU.
 *             Earth has a distance of 1 AU.
 * Outputs:
 *   arvec = The inertial acceleration vector due to the effects
 *       of Solar Radiation pressure in km/sec^2.  The vector
 *       components of the output are the same as the vector
 *       components of the sunvec input vector.
 */
void solarRad(double A, double m, double* sunvec, double* arvec)
{
    double c;
    double Cr;
    double sundist;

    /* Speed of light */
    c = 299792458.; /* m/s */

    /* Radiation pressure coefficient */
    Cr = 1.3;

    /* Magnitude of position vector */
    sundist = v3Norm(sunvec); /* AU */

    /* Computing the acceleration vector */
    v3Scale((-Cr * A * SOLAR_FLUX_EARTH) / (m * c * pow(sundist, 3)) / 1000., sunvec, arvec);
}

static int validEllipticElements(const ClassicElements* elements)
{
    return isfinite(elements->a) && elements->a > 0.0 && isfinite(elements->e) && elements->e >= 0.0 &&
           elements->e < 1.0 && isfinite(elements->i) && elements->i >= 0.0 && elements->i <= MPI &&
           isfinite(elements->Omega) && isfinite(elements->omega) && isfinite(elements->f);
}

/* Shared checked implementation; report controls diagnostics. */
static void clMeanOscMapImpl(double req,
                             double J2,
                             ClassicElements* elements,
                             ClassicElements* elements_p,
                             double sgn,
                             int report)
{
    if (!validEllipticElements(elements) || !isfinite(req) || req <= 0.0 || !isfinite(J2) ||
        (sgn != 1.0 && sgn != -1.0)) {
        goto invalidMeanOsc;
    }
    if (J2 == 0.0) {
        elements_p->a = elements->a;
        elements_p->e = elements->e;
        elements_p->i = elements->i;
        elements_p->Omega = elements->Omega;
        elements_p->omega = elements->omega;
        elements_p->f = elements->f;
        return;
    }

    double a = elements->a;
    double e = elements->e;
    double i = elements->i;
    double Omega = elements->Omega;
    double omega = elements->omega;
    // Reflection across the x-z plane preserves J2 dynamics. Work in the
    // prograde frame to resolve inclination near pi in the sin(i/2) recovery.
    int retrograde = i > MPI / 2.0;
    if (retrograde) {
        i = MPI - i;
        Omega = -Omega;
    }
    if (fabs(1.0 - 5.0 * cos(i) * cos(i)) <= sqrt(DBL_EPSILON)) {
        goto invalidMeanOsc;
    }
    // Put f, E and M on the same branch before forming the periodic f-M term.
    double f = atan2(sin(elements->f), cos(elements->f));
    double E = f2E(f, e);
    double M = E2M(E, e);

    double gamma2 = sgn * J2 / 2.0 * pow((req / a), 2.0); // (F.1),(F.2)
    double eta = sqrt(1.0 - pow(e, 2.0));
    double gamma2p = gamma2 / pow(eta, 4.0);         // (F.3)
    double a_r = (1.0 + e * cos(f)) / pow(eta, 2.0); // (F.6)
    double sinI = sin(i);
    double cosI = cos(i);
    // 1-11*c^2-40*c^4/(1-5*c^2) = sin(i)^2*(1-15*c^2)/(1-5*c^2).
    // Factoring cancels the removable 1/tan(i) singularity in (F.10).
    double inclinationRatio = (1.0 - 15.0 * cosI * cosI) / (1.0 - 5.0 * cosI * cosI);
    double inclinationFactor = sinI * sinI * inclinationRatio;

    double ap = a + a * gamma2 *
                        ((3.0 * pow(cos(i), 2.0) - 1.0) * (pow(a_r, 3.0) - 1.0 / pow(eta, 3.0)) +
                         3.0 * sinI * sinI * pow(a_r, 3.0) * cos(2.0 * omega + 2.0 * f)); // (F.7)

    double de1 = gamma2p / 8.0 * e * pow(eta, 2.0) * inclinationFactor * cos(2.0 * omega); // (F.8)

    double de = de1 + pow(eta, 2.0) / 2.0 *
                          (gamma2 * ((3.0 * pow(cos(i), 2.0) - 1.0) / pow(eta, 6.0) *
                                         (e * eta + e / (1.0 + eta) + 3.0 * cos(f) + 3.0 * e * pow(cos(f), 2.0) +
                                          pow(e, 2.0) * pow(cos(f), 3.0)) +
                                     3.0 * (1.0 - pow(cos(i), 2.0)) / pow(eta, 6.0) *
                                         (e + 3.0 * cos(f) + 3 * e * pow(cos(f), 2) + pow(e, 2) * pow(cos(f), 3)) *
                                         cos(2 * omega + 2 * f)) -
                           gamma2p * (1.0 - pow(cos(i), 2.0)) *
                               (3.0 * cos(2.0 * omega + f) + cos(2.0 * omega + 3.0 * f))); // (F.9)

    double di = gamma2p / 8.0 * sinI * cosI *
                (-e * e * inclinationRatio * cos(2.0 * omega) +
                 4.0 * (3.0 * cos(2.0 * omega + 2.0 * f) + 3.0 * e * cos(2.0 * omega + f) +
                        e * cos(2.0 * omega + 3.0 * f))); // (F.10), including the equatorial limit

    double MpopOp =
        M + omega + Omega + gamma2p / 8.0 * pow(eta, 3.0) * inclinationFactor * sin(2.0 * omega) -
        gamma2p / 16.0 *
            (2.0 + pow(e, 2.0) - 11.0 * (2.0 + 3.0 * pow(e, 2.0)) * pow(cos(i), 2.0) -
             40.0 * (2.0 + 5.0 * pow(e, 2.0)) * pow(cos(i), 4.0) / (1.0 - 5.0 * pow(cos(i), 2.0)) -
             400.0 * pow(e, 2.0) * pow(cos(i), 6.0) / pow(1.0 - 5.0 * pow(cos(i), 2.0), 2.0)) *
            sin(2.0 * omega) +
        gamma2p / 4.0 *
            (-6.0 * (1.0 - 5.0 * pow(cos(i), 2.0)) * (f - M + e * sin(f)) +
             (3.0 - 5.0 * pow(cos(i), 2.0)) *
                 (3.0 * sin(2.0 * omega + 2.0 * f) + 3.0 * e * sin(2.0 * omega + f) + e * sin(2.0 * omega + 3.0 * f))) -
        gamma2p / 8.0 * pow(e, 2.0) * cos(i) *
            (11.0 + 80.0 * pow(cos(i), 2.0) / (1.0 - 5.0 * pow(cos(i), 2.0)) +
             200.0 * pow(cos(i), 4.0) / pow(1.0 - 5.0 * pow(cos(i), 2.0), 2.0)) *
            sin(2.0 * omega) -
        gamma2p / 2.0 * cos(i) *
            (6.0 * (f - M + e * sin(f)) - 3.0 * sin(2.0 * omega + 2.0 * f) - 3.0 * e * sin(2.0 * omega + f) -
             e * sin(2.0 * omega + 3.0 * f)); // (F.11)

    double edM = gamma2p / 8.0 * e * pow(eta, 3.0) * inclinationFactor * sin(2.0 * omega) -
                 gamma2p / 4.0 * pow(eta, 3.0) *
                     (2.0 * (3.0 * pow(cos(i), 2.0) - 1.0) * (pow(a_r * eta, 2.0) + a_r + 1.0) * sin(f) +
                      3.0 * (1.0 - pow(cos(i), 2.0)) *
                          ((-pow(a_r * eta, 2.0) - a_r + 1.0) * sin(2.0 * omega + f) +
                           (pow(a_r * eta, 2.0) + a_r + 1.0 / 3.0) * sin(2.0 * omega + 3.0 * f))); // (F.12)

    double dOmega = -gamma2p / 8.0 * pow(e, 2.0) * cos(i) *
                        (11.0 + 80.0 * pow(cos(i), 2.0) / (1.0 - 5.0 * pow(cos(i), 2.0)) +
                         200.0 * pow(cos(i), 4.0) / pow(1.0 - 5.0 * pow(cos(i), 2.0), 2.0)) *
                        sin(2.0 * omega) -
                    gamma2p / 2.0 * cos(i) *
                        (6.0 * (f - M + e * sin(f)) - 3.0 * sin(2.0 * omega + 2.0 * f) -
                         3.0 * e * sin(2.0 * omega + f) - e * sin(2.0 * omega + 3.0 * f)); // (F.13)

    double d1 = (e + de) * sin(M) + edM * cos(M); // (F.14)
    double d2 = (e + de) * cos(M) - edM * sin(M); // (F.15)

    double Mp = atan2(d1, d2); // (F.16)
    double ep = hypot(d1, d2); // (F.17)

    double d3 = (sin(i / 2.0) + cos(i / 2.0) * di / 2.0) * sin(Omega) + sin(i / 2.0) * dOmega * cos(Omega); // (F.18)
    double d4 = (sin(i / 2.0) + cos(i / 2.0) * di / 2.0) * cos(Omega) - sin(i / 2.0) * dOmega * sin(Omega); // (F.19)

    double sinHalfIp = hypot(d3, d4);
    if (!isfinite(ap) || ap <= 0.0 || !isfinite(ep) || ep >= 1.0 || !isfinite(sinHalfIp) || sinHalfIp > 1.0 ||
        !isfinite(MpopOp)) {
        goto invalidMeanOsc;
    }
    double Omegap = atan2(d3, d4);        // (F.20)
    double ip = 2.0 * asin(sinHalfIp);    // (F.21)
    double omegap = MpopOp - Mp - Omegap; // (F.22)

    double Ep = M2E(Mp, ep);
    double fp = E2f(Ep, ep);
    if (!isfinite(fp) || !isfinite(omegap)) {
        goto invalidMeanOsc;
    }

    elements_p->a = ap;
    elements_p->e = ep;
    elements_p->i = retrograde ? MPI - ip : ip;
    elements_p->Omega = retrograde ? -Omegap : Omegap;
    elements_p->omega = omegap;
    elements_p->f = fp;
    return;

invalidMeanOsc:
    if (report) {
        BSK_PRINT(MSG_ERROR,
                  "%s",
                  "clMeanOscMap() requires valid elliptic inputs away from critical inclinations and finite elliptic "
                  "outputs.");
    }
    elements_p->a = elements_p->e = elements_p->i = elements_p->Omega = elements_p->omega = elements_p->f = nan("");
}

/* Shared checked implementation; report controls diagnostics. */
static void clElem2eqElemImpl(ClassicElements* elements_cl, equinoctialElements* elements_eq, int report)
{
    if (!validEllipticElements(elements_cl) || cos(elements_cl->i / 2.0) <= 8.0 * DBL_EPSILON ||
        !isfinite(elements_cl->Omega + elements_cl->omega) ||
        !isfinite(elements_cl->Omega + elements_cl->omega + elements_cl->f)) {
        if (report) {
            BSK_PRINT(
                MSG_ERROR, "%s", "clElem2eqElem() requires finite elliptic elements away from the retrograde equator.");
        }
        elements_eq->a = elements_eq->P1 = elements_eq->P2 = elements_eq->Q1 = elements_eq->Q2 = elements_eq->l =
            elements_eq->L = nan("");
        return;
    }
    // conversion
    // from classical orbital elements (a,e,i,Omega,omega,f)
    // to equinoctial orbital elements (a,P1,P2,Q1,Q2,l,L)
    elements_eq->a = elements_cl->a;
    elements_eq->P1 = elements_cl->e * sin(elements_cl->Omega + elements_cl->omega);
    elements_eq->P2 = elements_cl->e * cos(elements_cl->Omega + elements_cl->omega);
    elements_eq->Q1 = tan(elements_cl->i / 2.0) * sin(elements_cl->Omega);
    elements_eq->Q2 = tan(elements_cl->i / 2.0) * cos(elements_cl->Omega);
    double E = f2E(elements_cl->f, elements_cl->e);
    double M = E2M(E, elements_cl->e);
    elements_eq->l = elements_cl->Omega + elements_cl->omega + M;
    elements_eq->L = elements_cl->Omega + elements_cl->omega + elements_cl->f;
}

/* Preserve historical results outside the checked domain. */
static void legacyHillFrame(double* rc_N, double* vc_N, double HN[3][3])
{
    double ir_N[3];     /* orbit radial unit direction vector */
    double itheta_N[3]; /* along-track unit direction vector */
    double ih_N[3];     /* orbit plane normal unit direction vector */
    double hVec_N[3];   /* orbit angular momentum vector */

    v3Normalize(rc_N, ir_N);
    v3Cross(rc_N, vc_N, hVec_N);
    v3Normalize(hVec_N, ih_N);
    v3Cross(ih_N, ir_N, itheta_N);
    v3Copy(ir_N, HN[0]);
    v3Copy(itheta_N, HN[1]);
    v3Copy(ih_N, HN[2]);
}

/* Preserve historical results outside the checked domain. */
static void legacyHill2rv(double* rc_N, double* vc_N, double* rho_H, double* rhoPrime_H, double* rd_N, double* vd_N)
{
    double HN[3][3];      /* DCM of Hill frame relative to inertial */
    double NH[3][3];      /* DCM of inertial frame relative to Hill */
    double hVec_N[3];     /* orbit angular momentum vector */
    double rc;            /* chief orbit radius */
    double fDot;          /* chief true anomaly rate */
    double omega_HN_H[3]; /* Hill frame angular velocity */
    double rho_N[3];      /* deputy relative to chief vector in N frame components */

    legacyHillFrame(rc_N, vc_N, HN);
    m33Transpose(HN, NH);
    v3Cross(rc_N, vc_N, hVec_N);
    rc = v3Norm(rc_N);
    fDot = v3Norm(hVec_N) / rc / rc;
    v3Set(0, 0, fDot, omega_HN_H);

    /* compute inertial deputy position */
    m33MultV3(NH, rho_H, rho_N);
    v3Add(rc_N, rho_N, rd_N);

    /* compute inertial deputy velocity */
    v3Cross(omega_HN_H, rho_H, vd_N);
    v3Add(vd_N, rhoPrime_H, vd_N);
    m33MultV3(NH, vd_N, vd_N);
    v3Add(vd_N, vc_N, vd_N);
}

/* Preserve historical results outside the checked domain. */
static void legacyRv2hill(double* rc_N, double* vc_N, double* rd_N, double* vd_N, double* rho_H, double* rhoPrime_H)
{
    double HN[3][3];      /* DCM of Hill frame relative to inertial */
    double hVec_N[3];     /* orbit angular momentum vector */
    double rc;            /* chief orbit radius */
    double fDot;          /* chief true anomaly rate */
    double omega_HN_H[3]; /* Hill frame angular velocity */
    double rho_N[3];      /* deputy relative to chief vector in N frame components */
    double rhoDot_N[3];   /* inertial derivative of deputy/chief vector */
    double rhoDot_H[3];   /* inertial derivative of deputy/chief vector */

    legacyHillFrame(rc_N, vc_N, HN);
    v3Cross(rc_N, vc_N, hVec_N);
    rc = v3Norm(rc_N);
    fDot = v3Norm(hVec_N) / rc / rc;
    v3Set(0, 0, fDot, omega_HN_H);

    /* compute Hill frame position */
    v3Subtract(rd_N, rc_N, rho_N);
    m33MultV3(HN, rho_N, rho_H);

    /* compute Hill frame velocity */
    v3Subtract(vd_N, vc_N, rhoDot_N);
    m33MultV3(HN, rhoDot_N, rhoDot_H);
    v3Cross(omega_HN_H, rho_H, rhoPrime_H);
    v3Subtract(rhoDot_H, rhoPrime_H, rhoPrime_H);
}

/* Preserve historical results outside the checked domain. */
static void legacyClMeanOscMap(double req,
                               double J2,
                               ClassicElements* elements,
                               ClassicElements* elements_p,
                               double sgn)
{
    // Classical orbital elements = (a,e,i,Omega,omega,f)
    // First-order J2 Mapping Between Mean and Osculating Orbital Elements
    // sgn=1:mean2osc, sgn=-1:osc2mean
    // Analytical Mechanics of Space Systems
    // Hanspeter Schaub, John L. Junkins, 4th edition.
    // [m] or [km] should be the same both for req and elements->a

    double a = elements->a;
    double e = elements->e;
    double i = elements->i;
    double Omega = elements->Omega;
    double omega = elements->omega;
    double f = elements->f;
    double E = f2E(f, e);
    double M = E2M(E, e);

    double gamma2 = sgn * J2 / 2.0 * pow((req / a), 2.0); // (F.1),(F.2)
    double eta = sqrt(1.0 - pow(e, 2.0));
    double gamma2p = gamma2 / pow(eta, 4.0);         // (F.3)
    double a_r = (1.0 + e * cos(f)) / pow(eta, 2.0); // (F.6)

    double ap = a + a * gamma2 *
                        ((3.0 * pow(cos(i), 2.0) - 1.0) * (pow(a_r, 3.0) - 1.0 / pow(eta, 3.0)) +
                         3.0 * (1.0 - pow(cos(i), 2.0)) * pow(a_r, 3.0) * cos(2.0 * omega + 2.0 * f)); // (F.7)

    double de1 = gamma2p / 8.0 * e * pow(eta, 2.0) *
                 (1.0 - 11.0 * pow(cos(i), 2.0) - 40.0 * pow(cos(i), 4.0) / (1.0 - 5.0 * pow(cos(i), 2.0))) *
                 cos(2.0 * omega); // (F.8)

    double de = de1 + pow(eta, 2.0) / 2.0 *
                          (gamma2 * ((3.0 * pow(cos(i), 2.0) - 1.0) / pow(eta, 6.0) *
                                         (e * eta + e / (1.0 + eta) + 3.0 * cos(f) + 3.0 * e * pow(cos(f), 2.0) +
                                          pow(e, 2.0) * pow(cos(f), 3.0)) +
                                     3.0 * (1.0 - pow(cos(i), 2.0)) / pow(eta, 6.0) *
                                         (e + 3.0 * cos(f) + 3 * e * pow(cos(f), 2) + pow(e, 2) * pow(cos(f), 3)) *
                                         cos(2 * omega + 2 * f)) -
                           gamma2p * (1.0 - pow(cos(i), 2.0)) *
                               (3.0 * cos(2.0 * omega + f) + cos(2.0 * omega + 3.0 * f))); // (F.9)

    double di =
        -e * de1 / pow(eta, 2.0) / tan(i) + gamma2p / 2.0 * cos(i) * sqrt(1.0 - pow(cos(i), 2.0)) *
                                                (3.0 * cos(2.0 * omega + 2.0 * f) + 3.0 * e * cos(2.0 * omega + f) +
                                                 e * cos(2.0 * omega + 3.0 * f)); // (F.10)

    double MpopOp =
        M + omega + Omega +
        gamma2p / 8.0 * pow(eta, 3.0) *
            (1.0 - 11.0 * pow(cos(i), 2.0) - 40.0 * pow(cos(i), 4.0) / (1.0 - 5.0 * pow(cos(i), 2.0))) *
            sin(2.0 * omega) -
        gamma2p / 16.0 *
            (2.0 + pow(e, 2.0) - 11.0 * (2.0 + 3.0 * pow(e, 2.0)) * pow(cos(i), 2.0) -
             40.0 * (2.0 + 5.0 * pow(e, 2.0)) * pow(cos(i), 4.0) / (1.0 - 5.0 * pow(cos(i), 2.0)) -
             400.0 * pow(e, 2.0) * pow(cos(i), 6.0) / pow(1.0 - 5.0 * pow(cos(i), 2.0), 2.0)) *
            sin(2.0 * omega) +
        gamma2p / 4.0 *
            (-6.0 * (1.0 - 5.0 * pow(cos(i), 2.0)) * (f - M + e * sin(f)) +
             (3.0 - 5.0 * pow(cos(i), 2.0)) *
                 (3.0 * sin(2.0 * omega + 2.0 * f) + 3.0 * e * sin(2.0 * omega + f) + e * sin(2.0 * omega + 3.0 * f))) -
        gamma2p / 8.0 * pow(e, 2.0) * cos(i) *
            (11.0 + 80.0 * pow(cos(i), 2.0) / (1.0 - 5.0 * pow(cos(i), 2.0)) +
             200.0 * pow(cos(i), 4.0) / pow(1.0 - 5.0 * pow(cos(i), 2.0), 2.0)) *
            sin(2.0 * omega) -
        gamma2p / 2.0 * cos(i) *
            (6.0 * (f - M + e * sin(f)) - 3.0 * sin(2.0 * omega + 2.0 * f) - 3.0 * e * sin(2.0 * omega + f) -
             e * sin(2.0 * omega + 3.0 * f)); // (F.11)

    double edM = gamma2p / 8.0 * e * pow(eta, 3.0) *
                     (1.0 - 11.0 * pow(cos(i), 2.0) - 40.0 * pow(cos(i), 4.0) / (1.0 - 5.0 * pow(cos(i), 2.0))) *
                     sin(2.0 * omega) -
                 gamma2p / 4.0 * pow(eta, 3.0) *
                     (2.0 * (3.0 * pow(cos(i), 2.0) - 1.0) * (pow(a_r * eta, 2.0) + a_r + 1.0) * sin(f) +
                      3.0 * (1.0 - pow(cos(i), 2.0)) *
                          ((-pow(a_r * eta, 2.0) - a_r + 1.0) * sin(2.0 * omega + f) +
                           (pow(a_r * eta, 2.0) + a_r + 1.0 / 3.0) * sin(2.0 * omega + 3.0 * f))); // (F.12)

    double dOmega = -gamma2p / 8.0 * pow(e, 2.0) * cos(i) *
                        (11.0 + 80.0 * pow(cos(i), 2.0) / (1.0 - 5.0 * pow(cos(i), 2.0)) +
                         200.0 * pow(cos(i), 4.0) / pow(1.0 - 5.0 * pow(cos(i), 2.0), 2.0)) *
                        sin(2.0 * omega) -
                    gamma2p / 2.0 * cos(i) *
                        (6.0 * (f - M + e * sin(f)) - 3.0 * sin(2.0 * omega + 2.0 * f) -
                         3.0 * e * sin(2.0 * omega + f) - e * sin(2.0 * omega + 3.0 * f)); // (F.13)

    double d1 = (e + de) * sin(M) + edM * cos(M); // (F.14)
    double d2 = (e + de) * cos(M) - edM * sin(M); // (F.15)

    double Mp = atan2(d1, d2);           // (F.16)
    double ep = sqrt(d1 * d1 + d2 * d2); // (F.17)

    double d3 = (sin(i / 2.0) + cos(i / 2.0) * di / 2.0) * sin(Omega) + sin(i / 2.0) * dOmega * cos(Omega); // (F.18)
    double d4 = (sin(i / 2.0) + cos(i / 2.0) * di / 2.0) * cos(Omega) - sin(i / 2.0) * dOmega * sin(Omega); // (F.19)

    double Omegap = atan2(d3, d4);                       // (F.20)
    double ip = 2.0 * safeAsin(sqrt(d3 * d3 + d4 * d4)); // (F.21)
    double omegap = MpopOp - Mp - Omegap;                // (F.22)

    double Ep = M2E(Mp, ep);
    double fp = E2f(Ep, ep);

    elements_p->a = ap;
    elements_p->e = ep;
    elements_p->i = ip;
    elements_p->Omega = Omegap;
    elements_p->omega = omegap;
    elements_p->f = fp;
}

/* Preserve historical results outside the checked domain. */
static void legacyClElem2eqElem(ClassicElements* elements_cl, equinoctialElements* elements_eq)
{
    // conversion
    // from classical orbital elements (a,e,i,Omega,omega,f)
    // to equinoctial orbital elements (a,P1,P2,Q1,Q2,l,L)
    elements_eq->a = elements_cl->a;
    elements_eq->P1 = elements_cl->e * sin(elements_cl->Omega + elements_cl->omega);
    elements_eq->P2 = elements_cl->e * cos(elements_cl->Omega + elements_cl->omega);
    elements_eq->Q1 = tan(elements_cl->i / 2.0) * sin(elements_cl->Omega);
    elements_eq->Q2 = tan(elements_cl->i / 2.0) * cos(elements_cl->Omega);
    double E = f2E(elements_cl->f, elements_cl->e);
    double M = E2M(E, elements_cl->e);
    elements_eq->l = elements_cl->Omega + elements_cl->omega + M;
    elements_eq->L = elements_cl->Omega + elements_cl->omega + elements_cl->f;
}

/** @brief Compute the Hill DCM from a nonrectilinear chief state.
 * @param rc_N Finite inertial position [L], with finite nonzero norm.
 * @param vc_N Finite inertial velocity [L/s], with finite nonzero norm.
 * @param HN Output DCM mapping inertial components to Hill components.
 * @note Requires norm(cross(unit(rc_N), unit(vc_N))) > 8*DBL_EPSILON.
 * Invalid states produce a diagnostic and an all-NaN matrix.
 */
void hillFrameChecked(double* rc_N, double* vc_N, double HN[3][3])
{
    hillFrameImpl(rc_N, vc_N, HN, 1);
}

/** @brief Map a deputy state from the rotating chief Hill frame to inertial coordinates.
 * @param rc_N Chief inertial position [L], satisfying hillFrame() requirements.
 * @param vc_N Chief inertial velocity [L/s], satisfying hillFrame() requirements.
 * @param rho_H Finite Hill relative position [L].
 * @param rhoPrime_H Finite rotating-frame relative velocity [L/s].
 * @param rd_N Output deputy inertial position [L].
 * @param vd_N Output deputy inertial velocity [L/s].
 * @note Invalid states, unrepresentable positive angular rate, or nonfinite outputs
 * produce a diagnostic and two all-NaN vectors. Use consistent distance units.
 */
void hill2rvChecked(double* rc_N, double* vc_N, double* rho_H, double* rhoPrime_H, double* rd_N, double* vd_N)
{
    hill2rvImpl(rc_N, vc_N, rho_H, rhoPrime_H, rd_N, vd_N, 1);
}

/** @brief Express the deputy state relative to the chief in its rotating Hill frame.
 * @param rc_N Chief inertial position [L], satisfying hillFrame() requirements.
 * @param vc_N Chief inertial velocity [L/s], satisfying hillFrame() requirements.
 * @param rd_N Finite deputy inertial position [L].
 * @param vd_N Finite deputy inertial velocity [L/s].
 * @param rho_H Output Hill relative position [L].
 * @param rhoPrime_H Output rotating-frame relative velocity [L/s].
 * @note Invalid states, unrepresentable positive angular rate, or nonfinite outputs
 * produce a diagnostic and two all-NaN vectors. Use consistent distance units.
 */
void rv2hillChecked(double* rc_N, double* vc_N, double* rd_N, double* vd_N, double* rho_H, double* rhoPrime_H)
{
    rv2hillImpl(rc_N, vc_N, rd_N, vd_N, rho_H, rhoPrime_H, 1);
}

/**
 * @brief Apply the first-order J2 mean/osculating element mapping.
 * @param req Equatorial radius, in the same length unit as elements->a.
 * @param J2 Dimensionless second zonal harmonic coefficient.
 * @param elements Input classical elements, with angles in radians.
 * @param elements_p Output classical elements, with the same units as the input.
 * @param sgn +1 for mean to osculating, -1 for osculating to mean.
 * @note Follows Appendix F of Analytical Mechanics of Space Systems, Schaub and
 * Junkins, fourth edition. Inputs must be finite and elliptic with inclination
 * in [0, pi], away from the critical inclinations where 1 - 5*cos(i)^2 = 0.
 * The perturbation must be small; opposite mappings leave second-order J2 errors.
 * @note Both equatorial limits are supported. Whole revolutions in true anomaly
 * do not affect the mapped Cartesian state; output angles need not retain input
 * revolution counts. J2 = 0 copies the six classical elements exactly.
 * Only a, e, i, Omega, omega and f are written; auxiliary fields are unchanged.
 * Input and output may alias.
 * @note Requires req > 0 and sgn = +/-1. For nonzero J2, rejects
 * |1 - 5*cos(i)^2| <= sqrt(DBL_EPSILON) and nonphysical or nonfinite mapped
 * elements. This numerical guard does not establish perturbation-model accuracy.
 * Invalid inputs or results produce a diagnostic and six NaN classical elements.
 */
void clMeanOscMapChecked(double req, double J2, ClassicElements* elements, ClassicElements* elements_p, double sgn)
{
    clMeanOscMapImpl(req, J2, elements, elements_p, sgn, 1);
}

/** @brief Map finite elliptic classical elements to prograde equinoctial elements.
 * @param elements_cl Input with a > 0, 0 <= e < 1 and 0 <= i < pi; angles in radians.
 * @param elements_eq Output with the same length unit as the input and angles in radians.
 * @note The retrograde equator is singular. Rejects cos(i/2) <= 8*DBL_EPSILON.
 * Invalid inputs or nonfinite results produce a diagnostic and seven NaN outputs.
 */
void clElem2eqElemChecked(ClassicElements* elements_cl, equinoctialElements* elements_eq)
{
    clElem2eqElemImpl(elements_cl, elements_eq, 1);
}

/** @brief Compute the Hill DCM from a nonrectilinear chief state.
 * @param rc_N Finite inertial position [L], with finite nonzero norm.
 * @param vc_N Finite inertial velocity [L/s], with finite nonzero norm.
 * @param HN Output DCM mapping inertial components to Hill components.
 * @note See hillFrameChecked() for the supported domain. Rejected domains
 * retain historical results. Checked calls provide explicit domain validation.
 */
void hillFrame(double* rc_N, double* vc_N, double HN[3][3])
{
    double candidate[3][3];
    hillFrameImpl(rc_N, vc_N, candidate, 0);
    if (finiteVector3(candidate[0]) && finiteVector3(candidate[1]) && finiteVector3(candidate[2])) {
        m33Copy(candidate, HN);
    } else {
        legacyHillFrame(rc_N, vc_N, HN);
    }
}

/** @brief Map a deputy state from the rotating chief Hill frame to inertial coordinates.
 * @param rc_N Chief inertial position [L], satisfying hillFrameChecked() requirements.
 * @param vc_N Chief inertial velocity [L/s], satisfying hillFrameChecked() requirements.
 * @param rho_H Finite Hill relative position [L].
 * @param rhoPrime_H Finite rotating-frame relative velocity [L/s].
 * @param rd_N Output deputy inertial position [L].
 * @param vd_N Output deputy inertial velocity [L/s].
 * @note See hill2rvChecked() for the supported domain. Rejected domains
 * retain historical results. Checked calls provide explicit domain validation.
 */
void hill2rv(double* rc_N, double* vc_N, double* rho_H, double* rhoPrime_H, double* rd_N, double* vd_N)
{
    double position[3], velocity[3];
    hill2rvImpl(rc_N, vc_N, rho_H, rhoPrime_H, position, velocity, 0);
    if (finiteVector3(position) && finiteVector3(velocity)) {
        v3Copy(position, rd_N);
        v3Copy(velocity, vd_N);
    } else {
        legacyHill2rv(rc_N, vc_N, rho_H, rhoPrime_H, rd_N, vd_N);
    }
}

/** @brief Express the deputy state relative to the chief in its rotating Hill frame.
 * @param rc_N Chief inertial position [L], satisfying hillFrameChecked() requirements.
 * @param vc_N Chief inertial velocity [L/s], satisfying hillFrameChecked() requirements.
 * @param rd_N Finite deputy inertial position [L].
 * @param vd_N Finite deputy inertial velocity [L/s].
 * @param rho_H Output Hill relative position [L].
 * @param rhoPrime_H Output rotating-frame relative velocity [L/s].
 * @note See rv2hillChecked() for the supported domain. Rejected domains
 * retain historical results. Checked calls provide explicit domain validation.
 */
void rv2hill(double* rc_N, double* vc_N, double* rd_N, double* vd_N, double* rho_H, double* rhoPrime_H)
{
    double position[3], velocity[3];
    rv2hillImpl(rc_N, vc_N, rd_N, vd_N, position, velocity, 0);
    if (finiteVector3(position) && finiteVector3(velocity)) {
        v3Copy(position, rho_H);
        v3Copy(velocity, rhoPrime_H);
    } else {
        legacyRv2hill(rc_N, vc_N, rd_N, vd_N, rho_H, rhoPrime_H);
    }
}

/**
 * @brief Apply the first-order J2 mean/osculating element mapping.
 * @param req Equatorial radius, in the same length unit as elements->a.
 * @param J2 Dimensionless second zonal harmonic coefficient.
 * @param elements Input classical elements, with angles in radians.
 * @param elements_p Output classical elements, with the same units as the input.
 * @param sgn +1 for mean to osculating, -1 for osculating to mean.
 * @note See clMeanOscMapChecked() for the supported domain. Rejected domains
 * retain historical results. Checked calls provide explicit domain validation.
 * Invalid inclinations are always rejected; use 0 <= i <= pi.
 */
void clMeanOscMap(double req, double J2, ClassicElements* elements, ClassicElements* elements_p, double sgn)
{
    if (!isfinite(elements->i) || elements->i < 0.0 || elements->i > MPI) {
        clMeanOscMapChecked(req, J2, elements, elements_p, sgn);
        return;
    }
    ClassicElements candidate = { 0 };
    clMeanOscMapImpl(req, J2, elements, &candidate, sgn, 0);
    if (isfinite(candidate.a)) {
        elements_p->a = candidate.a;
        elements_p->e = candidate.e;
        elements_p->i = candidate.i;
        elements_p->Omega = candidate.Omega;
        elements_p->omega = candidate.omega;
        elements_p->f = candidate.f;
    } else {
        legacyClMeanOscMap(req, J2, elements, elements_p, sgn);
    }
}

/** @brief Map finite elliptic classical elements to prograde equinoctial elements.
 * @param elements_cl Input with a > 0, 0 <= e < 1 and 0 <= i < pi; angles in radians.
 * @param elements_eq Output with the same length unit as the input and angles in radians.
 * @note See clElem2eqElemChecked() for the supported domain. Rejected domains
 * retain historical results. Checked calls provide explicit domain validation.
 * Invalid inclinations are always rejected; use 0 <= i <= pi.
 */
void clElem2eqElem(ClassicElements* elements_cl, equinoctialElements* elements_eq)
{
    if (!isfinite(elements_cl->i) || elements_cl->i < 0.0 || elements_cl->i > MPI) {
        clElem2eqElemChecked(elements_cl, elements_eq);
        return;
    }
    equinoctialElements candidate;
    clElem2eqElemImpl(elements_cl, &candidate, 0);
    if (isfinite(candidate.a)) {
        *elements_eq = candidate;
    } else {
        legacyClElem2eqElem(elements_cl, elements_eq);
    }
}

/** @brief Explicit-planet adapter preserving the original jPerturb conventions.
 * @param rvec Position [km].
 * @param num Maximum zonal degree.
 * @param ajtot Output acceleration [km/s^2].
 * @param planetID Legacy planet identifier; unknown values select Earth.
 * @note Sun/Pluto retain the historical unchanged-output behavior. Use
 * jPerturbForPlanet() for checked selection and explicit zero-zonal outputs.
 */
void jPerturbLegacy(double* rvec, int num, double* ajtot, int planetID)
{
    if (planetID == CELESTIAL_SUN || planetID == CELESTIAL_PLUTO) {
        return;
    }
    if (planetID < CELESTIAL_MERCURY || planetID >= MAX_CELESTIAL || planetID == CELESTIAL_PHOBOS ||
        planetID == CELESTIAL_DEIMOS) {
        planetID = CELESTIAL_EARTH;
    }
    jPerturbForPlanet(rvec, num, ajtot, (CelestialObject_t)planetID);
}

/** @brief Retain the legacy variadic symbol for explicit-planet native callers.
 * @param rvec Position [km].
 * @param num Maximum zonal degree.
 * @param ajtot Output acceleration [km/s^2].
 * @note Exactly one trailing int planet identifier is required. Header-based
 * three-argument calls select Earth through the header adapter without reading varargs.
 */
void(jPerturb)(double* rvec, int num, double* ajtot, ...)
{
    va_list arguments;
    va_start(arguments, ajtot);
    int planetID = va_arg(arguments, int);
    va_end(arguments);
    jPerturbLegacy(rvec, num, ajtot, planetID);
}
