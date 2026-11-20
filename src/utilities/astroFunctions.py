#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


from calendar import monthrange
from datetime import date
from fractions import Fraction

import matplotlib.pyplot as plt
import numpy as np
from numpy import linalg as la
from Basilisk.architecture import astroConstants

# Reference solar flux at Earth [W/m^2]
solarFluxEarth = 1361.0

# ------------------------------------------------------------------------------------------------------------------- #
#                                           Normalize vector
#


def normalize(v):
    """normalize a vector"""
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v/norm

# ------------------------------------------------------------------------------------------------------------------- #
#                                           Orbital Period
#


def orbitalPeriod(a, mu):
    """Return the orbit period"""
    P = 2*astroConstants.MPI * np.sqrt(a*a*a / mu)
    return P


def orbitalSMA(P, mu):
    """Return the semi-major axis"""
    a3 = mu * np.power(P / (2 * astroConstants.MPI), 2)
    exp = 1./3
    a = np.power(a3, exp)
    return a

# ------------------------------------------------------------------------------------------------------------------- #
#                                           FROM OE 2 RV
#


def Earth_RV(JDE):
    """return Earth (r,v)"""
    (a, e, i, Omega, w, nu) = ephemeridesMeeus(JDE, 'EARTH')
    return OE2RV(astroConstants.MU_SUN, a, e, i, Omega, w, nu)


def Mars_RV(JDE):
    """return Mars (r,v)"""
    (a, e, i, Omega, w, nu) = ephemeridesMeeus(JDE, 'MARS')
    return OE2RV(astroConstants.MU_SUN, a, e, i, Omega, w, nu)


def Jupiter_RV(JDE):
    """return Jupiter (r,v)"""
    (a, e, i, Omega, w, nu) = ephemeridesMeeus(JDE, 'JUPITER')
    return OE2RV(astroConstants.MU_SUN, a, e, i, Omega, w, nu)


def Venus_RV(JDE):
    """return Venus (r,v)"""
    (a, e, i, Omega, w, nu) = ephemeridesMeeus(JDE, 'VENUS')
    return OE2RV(astroConstants.MU_SUN, a, e, i, Omega, w, nu)


def Pluto_RV(JDE):
    """return Pluto (r,v)"""
    (a, e, i, Omega, w, nu) = ephemeridesMeeus(JDE, 'PLUTO')
    return OE2RV(astroConstants.MU_SUN, a, e, i, Omega, w, nu)


def Uranus_RV(JDE):
    """return Uranus (r,v)"""
    (a, e, i, Omega, w, nu) = ephemeridesMeeus(JDE, 'URANUS')
    return OE2RV(astroConstants.MU_SUN, a, e, i, Omega, w, nu)


def Neptune_RV(JDE):
    """return Neptune (r,v)"""
    (a, e, i, Omega, w, nu) = ephemeridesMeeus(JDE, 'NEPTUNE')
    return OE2RV(astroConstants.MU_SUN, a, e, i, Omega, w, nu)


def Saturn_RV(JDE):
    """return Saturn (r,v)"""
    (a, e, i, Omega, w, nu) = ephemeridesMeeus(JDE, 'SATURN')
    return OE2RV(astroConstants.MU_SUN, a, e, i, Omega, w, nu)


def OE2RV(mu, a, e, i, Omega, w, nu):
    """OE to (r,v) conversion"""
    if e != 1:
        p = a*(1-e*e)
    else:
        print('ERROR: parabolic case')
        return
    c = np.cos(nu)
    s = np.sin(nu)
    r_PQW = np.array([p*c / (1 + e*c), p*s / (1 + e*c), 0])
    v_PQW = np.array([-s * np.sqrt(mu/p), (e + c)*np.sqrt(mu/p), 0])
    return PQW2IJK(Omega, i, w, r_PQW, v_PQW)


def quadrant4(x):
    return 2*astroConstants.MPI - x


def RV2OE(mu, r_IJK, v_IJK):
    """Convert an inertial Cartesian state to classical orbital elements.

    :param mu: positive gravitational parameter (L^3/s^2)
    :param r_IJK: nonzero inertial position vector (L)
    :param v_IJK: inertial velocity vector (L/s)
    :return: tuple ``(a, e, i, Omega, omega, nu)`` with ``a`` in L and angles in radians

    Zero angular momentum gives ``i = 0``. A zero node vector defaults
    ``Omega`` and ``omega`` to zero, and zero eccentricity defaults ``omega``
    and ``nu`` to zero. Legacy quadrant adjustments can express zero as ``2*pi``.
    These placeholders can discard orientation information for singular orbits.
    The angle tests use the computed vector magnitudes without a tolerance.

    Radial states have unit eccentricity. Their semimajor axis is positive
    below escape speed and negative above it. For any state whose
    specific orbital energy is zero to roundoff, the parabolic convention
    is ``a = 0``, ``e = 1``. ``OE2RV`` does not reconstruct unit-eccentricity states.
    All inputs use a common working precision of at least ``float64``.
    """
    r = np.asarray(r_IJK)
    v = np.asarray(v_IJK)
    dtype = np.result_type(r.dtype, v.dtype, np.asarray(mu).dtype, np.float64)
    r = r.astype(dtype, copy=False)
    v = v.astype(dtype, copy=False)
    mu = dtype.type(mu)
    K = np.array([0, 0, 1])
    h = np.cross(r, v)
    n = np.cross(K, h)
    hm = np.linalg.norm(h)
    nm = np.linalg.norm(n)
    rm = np.linalg.norm(r)

    mu_over_r = mu/rm
    if hm == 0:
        e_vec = -(r/rm)
        e = 1.0  # [-]
        i = 0.0  # [rad]
    else:
        vm = np.linalg.norm(v)
        e_vec = 1./mu * ((vm*vm - mu_over_r)*r - np.dot(r, v)*v)
        e = np.linalg.norm(e_vec)
        i = np.arccos(np.clip(h[2]/hm, -1.0, 1.0))
    if nm == 0:
        Omega = 0.0  # [rad]
    else:
        Omega = np.arccos(np.clip(n[0]/nm, -1.0, 1.0))
    if n[1] < 0:
        Omega = quadrant4(Omega)

    dotProd = np.dot(n, e_vec)
    if nm == 0 or e == 0:
        omega = 0.0  # [rad]
    else:
        omega = np.arccos(np.clip(dotProd/(nm*e), -1.0, 1.0))
    if e_vec[2] < 0:
        omega = quadrant4(omega)

    dotProd = np.dot(e_vec, r)
    if e == 0:
        nu = 0.0  # [rad]
    else:
        nu = np.arccos(np.clip(dotProd/(e*rm), -1.0, 1.0))
    if np.dot(r, v) < 0:
        nu = quadrant4(nu)

    kinetic = np.dot(0.5*v, v)
    energy = kinetic - mu_over_r
    # Match the relative escape-energy tolerance in orbitalMotion.rv2elem.
    roundoff = 8*np.finfo(dtype).eps
    if abs(energy) <= roundoff*max(kinetic, mu_over_r):
        a = 0.0  # [L] Parabolic convention.
        e = 1.0  # [-]
    else:
        a = -0.5*mu/energy
    return (a, e, i, Omega, omega, nu)


def PQW2IJK(Omega, i, w, r_PQW, v_PQW):
    sO = np.sin(Omega)
    cO = np.cos(Omega)
    si = np.sin(i)
    ci = np.cos(i)
    sw = np.sin(w)
    cw = np.cos(w)

    C11 = cO*cw - sO*sw*ci
    C12 = -cO*sw - sO*cw*ci
    C13 = sO*si
    C21 = sO*cw + cO*sw*ci
    C22 = -sO*sw + cO*cw*ci
    C23 = -cO*si
    C31 = sw*si
    C32 = cw*si
    C33 = ci
    C = np.array([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]])

    r_IJK = C.dot(r_PQW)
    v_IJK = C.dot(v_PQW)
    return (r_IJK, v_IJK)

# ------------------------------------------------------------------------------------------------------------------- #
#                                           MEEUS ALGORITHM
#


def ephemeridesMeeus(JDE, celestialBody):
    """Approximate heliocentric orbital elements in the mean ecliptic and equinox of J2000.0.

    The planetary ``*_RV`` helpers use these elements to return positions in km
    and velocities in km/s.

    :param JDE: Julian Ephemeris Day (TT), with J2000.0 at 2451545.0 days
    :param celestialBody: uppercase planet name (VENUS, EARTH, MARS, JUPITER,
        SATURN, URANUS, NEPTUNE, or PLUTO)
    :return: tuple ``(a, e, i, Omega, omega, nu)`` with semimajor axis in km,
        dimensionless eccentricity and angles in radians
    """
    T = (JDE - 2451545.0)/36525  # [Julian centuries since J2000.0]

    def ephem(vec, T):
        val = vec[0] + vec[1]*T + vec[2]*T*T + vec[3]*T*T*T
        return val

    if celestialBody == 'EARTH':
        L = 100.466449 + 35999.3728519 * T + (-0.00000568)*T*T + 0*T*T*T
        a = 1.000001018
        e = 0.01670862 + (-0.000042037)*T + (-0.0000001236) * \
            T*T + 0.00000000004*T*T*T
        i = 0 + 0.0130546*T + (-0.00000931)*T*T + (-0.000000034)*T*T*T
        Omega = 174.873174 + (-0.2410908)*T + 0.00004067 * \
            T*T + (-0.000001327)*T*T*T
        Pi = 102.937348 + 0.3225557*T + 0.00015026*T*T + 0.000000478*T*T*T
    elif celestialBody == 'MARS':
        L = 355.433275 + 19140.2993313 * T + \
            0.00000261*T*T + (-0.000000003)*T*T*T
        a = 1.523679342
        e = 0.09340062 + 0.000090483 * T + \
            (-0.0000000806)*T*T + (-0.00000000035)*T*T*T
        i = 1.849726 + (-0.0081479) * T + (-0.00002255) * \
            T*T + (-0.000000027)*T*T*T
        Omega = 49.558093 + (-0.2949846) * T + \
            (-0.00063993)*T*T + (-0.000002143)*T*T*T
        Pi = 336.060234 + 0.4438898 * T + \
            (- 0.00017321)*T*T + 0.000000300*T*T*T
    elif celestialBody == 'JUPITER':
        L = 34.351484 + 3034.9056746*T + (-0.00008501)*T*T + 0.000000004*T*T*T
        a = 5.202603191 + 0.0000001913*T
        e = 0.04849485 + 0.000163244*T + - \
            0.0000004719*T*T + (-0.00000000197)*T*T*T
        i = 1.303270 + (-0.0019872)*T + 0.00003318*T*T + 0.000000092*T*T*T
        Omega = 100.464441 + 0.1766828*T + \
            0.00090387*T*T + (-0.000007032)*T*T*T
        Pi = 14.331309 + 0.2155525*T + 0.00072252*T*T + (-0.000004590)*T*T*T
    elif celestialBody == 'VENUS':
        L = 181.979801 + 58517.8156760*T + \
            0.00000165*T*T + (-0.000000002)*T*T*T
        a = 0.72332982
        e = 0.00677188 + (-0.000047766)*T + 0.0000000975 * \
            T*T + 0.00000000044*T*T*T
        i = 3.394662 + (-0.0008568)*T + (-0.00003244)*T*T + 0.000000010*T*T*T
        Omega = 76.679920 + (-0.2780080)*T + (-0.00014256) * \
            T*T + (-0.000000198)*T*T*T
        Pi = 131.563707 + 0.0048646*T + \
            (-0.00138232)*T*T + (-0.000005332)*T*T*T
    elif celestialBody == 'PLUTO':
        L = 238.92903833 + 145.20780515*T
        a = 39.48211675 + (-0.00031596)*T
        e = 0.24882730 + 0.00005170*T
        i = 17.14001206 + 0.00004818*T
        Omega = 110.30393684 + (-0.01183482)*T
        Pi = 224.06891629 + (-0.04062942)*T
    elif celestialBody == 'SATURN':
        L_vec = np.array([50.077471, 1222.1137943, 0.00021004, -0.000000019])
        a_vec = np.array([9.554909596, -0.0000021389, 0.0, 0.0])
        e_vec = np.array([0.05550862, -0.000346818, -
                         0.0000006456, 0.00000000338])
        i_vec = np.array([2.488878,	0.0025515, -0.00004903, 0.000000018])
        Omega_vec = np.array(
            [113.665524, -0.2566649, -0.00018345, 0.000000357])
        Pi_vec = np.array([93.056787, 0.5665496, 0.00052809, 0.000004882])
        L = ephem(L_vec, T)
        a = ephem(a_vec, T)
        e = ephem(e_vec, T)
        i = ephem(i_vec, T)
        Omega = ephem(Omega_vec, T)
        Pi = ephem(Pi_vec, T)
    elif celestialBody == 'URANUS':
        # Angular polynomials for Uranus and Neptune are in degrees (Meeus, 1st ed., Table 30.B).
        L_vec = np.array([314.055005, 428.4669983, -0.00000486, 0.000000006])
        a_vec = np.array([19.218446062, -0.0000000372, 0.00000000098, 0.0])
        e_vec = np.array(
            [0.04629590, -0.000027337, 0.0000000790, 0.00000000025])
        i_vec = np.array([0.773196, -0.0016869, 0.00000349, 0.000000016])
        Omega_vec = np.array([74.005947, 0.0741461, 0.00040540, 0.000000104])
        Pi_vec = np.array([173.005159, 0.0893206, -0.00009470, 0.000000413])
        L = ephem(L_vec, T)
        a = ephem(a_vec, T)
        e = ephem(e_vec, T)
        i = ephem(i_vec, T)
        Omega = ephem(Omega_vec, T)
        Pi = ephem(Pi_vec, T)
    elif celestialBody == 'NEPTUNE':
        L_vec = np.array([304.348665, 218.4862002, 0.00000059, -0.000000002])
        a_vec = np.array([30.110386869,	-0.0000001663, 0.00000000069, 0.0])
        e_vec = np.array([0.00898809, 0.000006408, -
                         0.0000000008, -0.00000000005])
        i_vec = np.array([1.769952, 0.0002257, 0.00000023, 0.0])
        Omega_vec = np.array([131.784057, -0.0061651, -0.00000219, -0.000000078])
        Pi_vec = np.array([48.123691, 0.0291587, 0.00007051, -0.000000023])
        L = ephem(L_vec, T)
        a = ephem(a_vec, T)
        e = ephem(e_vec, T)
        i = ephem(i_vec, T)
        Omega = ephem(Omega_vec, T)
        Pi = ephem(Pi_vec, T)
    else:
        print("Meeus coefficients for " + celestialBody + " not defined")
        L = 0.
        a = 0.
        e = 0.
        i = 0.
        Omega = 0.
        Pi = 0.

    return computeCOE(L, a, e, i, Omega, Pi)


def computeCOE(L, a, e, i, Omega, Pi):
    """Convert mean planetary elements using the fifth-order equation of center.

    :param L: mean longitude in degrees
    :param a: semimajor axis in astronomical units
    :param e: dimensionless eccentricity
    :param i: inclination in degrees
    :param Omega: longitude of the ascending node in degrees
    :param Pi: longitude of perihelion in degrees
    :return: tuple ``(a, e, i, Omega, omega, nu)`` in the input reference frame,
        with semimajor axis in km and angles in radians. The last two angles
        are the argument of perihelion and the approximate true anomaly.
    """
    L = L*astroConstants.D2R
    a = a*astroConstants.AU
    i = i*astroConstants.D2R
    Omega = Omega*astroConstants.D2R
    Pi = Pi*astroConstants.D2R

    w = Pi - Omega
    M = L - Pi
    # Equation of center through fifth order in eccentricity.
    C_cen = (2*e - 1/4 * np.power(e, 3) + 5/96 * np.power(e, 5)) * np.sin(M) +\
            (5/4 * np.power(e, 2) - 11/24 * np.power(e, 4)) * np.sin(2*M) +\
            (13/12 * np.power(e, 3) - 43/64 * np.power(e, 5)) * np.sin(3*M) +\
        103/96 * np.power(e, 4) * np.sin(4*M) +\
        1097/960 * np.power(e, 5) * np.sin(5*M)
    nu = M + C_cen
    return (a, e, i, Omega, w, nu)

# ------------------------------------------------------------------------------------------------------------------- #
#                                             DATES CONVERSION

_JD_ORDINAL_OFFSET = 1721424.5  # [day] Gregorian ordinal 1 starts at JD 1721425.5.


def JulianDate(GDE):
    """Convert a proleptic Gregorian calendar date to a Julian date.

    :param GDE: ``[year, month, day]`` with integer year (1 through 9999) and
        month (1 through 12). The day may be a scalar or an array, with its
        fractional part measured from midnight.
    :return: Julian date in days, as a scalar or an array matching the shape
        of the input days. Integer Julian dates denote noon.
    """
    year, month, day = GDE[:3]
    if int(year) != year or int(month) != month:
        raise ValueError("year and month must be integers")
    first_day = date(int(year), int(month), 1)
    days_in_month = monthrange(first_day.year, first_day.month)[1]
    day = np.asanyarray(day)
    if np.iscomplexobj(day) or (day.dtype == object and any(map(np.iscomplexobj, day.flat))):
        raise TypeError("day must be real")
    dtype = np.result_type(day.dtype, np.float64) if day.dtype.kind == "f" else np.float64
    day = day.astype(dtype, copy=False)
    valid_days = (day >= 1) & (day < days_in_month + 1)
    if np.any(~valid_days):
        raise ValueError("day is out of range for month")
    return _JD_ORDINAL_OFFSET + first_day.toordinal() + (day - 1)


def _gregorian_day(jd):
    """Return the Gregorian date and whole seconds since midnight."""
    jd = np.asarray(jd).item()
    if isinstance(jd, (float, np.floating)):
        ordinal, fraction = divmod(jd - _JD_ORDINAL_OFFSET, 1)
        # Recover whole seconds rounded down at the precision of the Julian date.
        fraction += np.spacing(jd) / 2
    else:
        ordinal, fraction = divmod(Fraction(jd) - Fraction(_JD_ORDINAL_OFFSET), 1)
    return date.fromordinal(int(ordinal)), int(fraction * 86400)


def GregorianDate(JDE):
    """Return the proleptic Gregorian calendar date containing a Julian date.

    :param JDE: Julian date in days within Gregorian years 1 through 9999
    :return: NumPy integer array ``[year, month, day]``
    """
    day, _ = _gregorian_day(JDE)
    return np.array([day.year, day.month, day.day])


def exactGregorianDate(JD):
    """Format the proleptic Gregorian date and time to whole seconds.

    Fractional seconds are discarded. To recover a whole second that rounded
    down in a binary floating-point Julian date, the conversion allows a
    tolerance of half the spacing at the working precision (at least binary64).
    Decimal and rational inputs are treated as exact.

    :param JD: Julian date in days within Gregorian years 1 through 9999
    :return: String ``[year, month, day, hour:minute:second]``
    """
    day, seconds = _gregorian_day(JD)
    hour, remainder = divmod(seconds, 3600)
    minute, second = divmod(remainder, 60)
    return f"[{day.year}, {day.month}, {day.day}, {hour}:{minute}:{second}]"


def optimalDate(GD0, DaysPastDeparture, TOF):
    JD0 = JulianDate(GD0) + DaysPastDeparture
    print('Departure Date: ', GregorianDate(JD0))
    print('Arrival Date: ', GregorianDate(JD0 + TOF))
    return (JD0, JD0+TOF)


# ------------------------------------------------------------------------------------------------------------------- #
#                                             B PARAMETERS
#
def B_params_1(r, v, mu):
    rm = np.linalg.norm(r)
    vm = np.linalg.norm(v)
    h = np.cross(r, v)
    h_hat = normalize(h)
    e_vec = 1. / mu * ((vm * vm - mu / rm) * r - np.dot(r, v) * v)
    e = np.linalg.norm(e_vec)
    Betta = np.arccos(1. / e)
    a = 1. / (2 / rm - vm * vm / mu)
    b = np.abs(a) * np.sqrt(e * e - 1)

    k_hat = np.array([0, 0, 1])
    S_hat = np.cos(Betta)/e * e_vec + np.sin(Betta) * \
        normalize(np.cross(h_hat, e_vec))
    T_hat = normalize(np.cross(S_hat, k_hat))
    R_hat = np.cross(S_hat, T_hat)
    B_hat = np.cross(S_hat, h_hat)

    B = b*B_hat
    BT = np.dot(B, T_hat)
    BR = np.dot(B, R_hat)
    return (BT, BR)


def B_params_2(v_in, v_out, mu):
    vm_in = np.linalg.norm(v_in)
    vm_out = np.linalg.norm(v_out)

    S = normalize(v_in)
    h = normalize(np.cross(v_in, v_out))
    B_hat = normalize(np.cross(S, h))
    k = np.array([0, 0, 1])
    T = normalize(np.cross(S, k))
    R = normalize(np.cross(S, T))
    psi = np.arccos(np.dot(v_in, v_out) / (vm_in*vm_out))
    rp = mu / (vm_in * vm_in) * \
        (1.0 / np.cos(0.5*(astroConstants.MPI - psi)) - 1)
    temp = 1 + vm_in * vm_in * rp / mu
    B = mu / (vm_in * vm_in) * np.sqrt(temp*temp - 1)
    BT = B * np.dot(B_hat, T)
    BR = B * np.dot(B_hat, R)
    theta = np.arctan2(BR, BT)
    return (rp, psi*astroConstants.R2D, B, theta*astroConstants.R2D, BT, BR)

# ------------------------------------------------------------------------------------------------------------------- #
#                                             RESONANT ORBITS
#

# v_inf_in: sc hyperbolic velocity
# V1: Earth velocity at arrival (EGA1)
# R1: Earth position at arrival (EGA1)
# V2: Earth velocity at departure (EGA2)
# N: Number of resonances


def EarthResonantOrbit(v_inf_in, V1, R1, v_inf_out, V2, N):
    # compute P, a, v_scSun
    P = N * 365.242189 * 86400                      # Earth orbital period
    n = P / (2.0*astroConstants.MPI)
    exp = 1./3
    # Earth heliocentric orbit SMA
    a = np.power(n * n * astroConstants.MU_SUN, exp)
    rm_1 = la.norm(R1)
    vm_sc_h = np.sqrt(astroConstants.MU_SUN * (2./rm_1 - 1./a)
                      )      # s/c heliocentric velocity

    # compute theta
    vm_inf_in = la.norm(v_inf_in)
    vm_P = la.norm(V1)
    cosTheta = (vm_inf_in * vm_inf_in + vm_P * vm_P -
                vm_sc_h * vm_sc_h) / (2 * vm_inf_in * vm_P)
    theta = np.arccos(cosTheta)

    # Compute T_vnc: DCM from VNC to ecliptic
    V_hat = normalize(V1)
    N_hat = normalize(np.cross(R1, V1))
    C_hat = np.cross(V_hat, N_hat)
    T_vnc = np.array([V_hat, N_hat, C_hat]).T

    # Arrays to stack successful resonances that don't impact w/ the planet
    rp_GA1 = np.array([])
    PHI_GA1 = np.array([])
    rp_GA2 = np.array([])
    PHI_GA2 = np.array([])

    # Loop
    vm_inf_out = la.norm(v_inf_out)
    c = np.cos(astroConstants.MPI - theta)
    s = np.sin(astroConstants.MPI - theta)
    p = 360     # number of points
    phi_vec = np.linspace(0., 2*astroConstants.MPI, p)
    for phi in phi_vec:
        vGA_out_VNC = vm_inf_out * np.array([c, s*np.cos(phi), -s*np.sin(phi)])
        v_GA1_out_ecl = np.dot(T_vnc, vGA_out_VNC)
        vm_GA1_out = la.norm(v_GA1_out_ecl)
        (rp, psi, B, theta, BT, BR) = B_params_2(
            v_inf_in, v_GA1_out_ecl, astroConstants.MU_EARTH)
        if rp > astroConstants.REQ_EARTH or rp < astroConstants.REQ_EARTH:
            rp_GA1 = np.append(rp_GA1, rp)
            PHI_GA1 = np.append(PHI_GA1, phi)

        v_GA2_in_ecl = v_GA1_out_ecl + V1 - V2
        vm_GA2_in = la.norm(v_GA2_in_ecl)
        (rp, psi, B, theta, BT, BR) = B_params_2(
            v_GA2_in_ecl, v_inf_out, astroConstants.MU_EARTH)
        if rp > astroConstants.REQ_EARTH or rp < astroConstants.REQ_EARTH:
            rp_GA2 = np.append(rp_GA2, rp)
            PHI_GA2 = np.append(PHI_GA2, phi)

    plt.plot(PHI_GA1, rp_GA1, 'm', PHI_GA2, rp_GA2, 'b')
    plt.axhline(astroConstants.REQ_EARTH, color='k')
    plt.legend(['EGA 1', 'EGA 2', 'r$_{P, min}$'], loc='lower right')
    plt.xlabel(r'$\phi$ [rad]')
    plt.ylabel('Perigee Radius [km]')
    # plt.show()
    return


def testResonance():
    R1 = np.array([31161153575.0634, 143995536213.184, 3018421.6823707])
    R2 = np.array([31194146713.798,
                   143988869052.709,
                   2352102.45497072])
    V1 = np.array([
        -29599.9908905588,
        6189.32095554111,
        0.0729005972949417])
    V2 = np.array([-29598.5134550237,
                   6195.92283490717,
                   0.0568758149973483])

    v_inf_in = np.array([-0.320955575422136,
                         -8.79065711515436,
                         1.70894261814904])
    v_inf_out = np.array([-8.95399388012787,
                          -0.346120703226863,
                          -0.350867778719334])

    N = 2

    EarthResonantOrbit(v_inf_in, V1/1000, R1/1000, v_inf_out, V2/1000, N)


# ------------------------------------------------------------------------------------------------------------------- #
#                                           HELIOCENTRIC ORBIT PLOT
#
def plotPlanetOrbit(JD, planet, color, ax):
    rx = np.array([])
    ry = np.array([])
    rz = np.array([])
    (a, e, i, Omega, w, nu0) = ephemeridesMeeus(JD, planet)
    nu_vec = np.linspace(nu0, nu0 + 2*astroConstants.MPI, 360)
    for nu in nu_vec:
        (r, v) = OE2RV(astroConstants.MU_SUN, a, e, i, Omega, w, nu)
        rx = np.append(rx, r[0])
        ry = np.append(ry, r[1])
        rz = np.append(rz, r[2])
    ax.plot(rx, ry, rz, c=color)
    ax.scatter(rx[0], ry[0], rz[0], c=color)

    max_range = np.array([rx.max()-rx.min(), ry.max() -
                         ry.min(), rz.max()-rz.min()]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -
                                1:2:2][0].flatten() + 0.5*(rx.max()+rx.min())
    Yb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -
                                1:2:2][1].flatten() + 0.5*(ry.max()+ry.min())
    Zb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -
                                1:2:2][2].flatten() + 0.5*(rz.max()+rz.min())
    # Comment or uncomment following both lines to test the fake bounding box:
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')


def plotSolarSystem(JD, JD_ref):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plotPlanetOrbit(JD, 'EARTH', 'dodgerblue', ax)
    plotPlanetOrbit(JD, 'SATURN', 'orchid', ax)
    plotPlanetOrbit(JD, 'URANUS', 'lightgreen', ax)
    plt.title('Days Past JD_ref = ' + str(JD - JD_ref))
    plt.show()


# ------------------------------------------------------------------------------------------------------------------- #
#                                           TISSERAND PLOT
#

def V_circular(r, mu):
    return np.sqrt(mu / r)


def rotationMatrix(alpha):
    """Return the active rotation matrix about the positive z axis for ``alpha`` radians."""
    M = np.identity(3)
    M[0, 0] = np.cos(alpha)
    M[0, 1] = -np.sin(alpha)
    M[1, 0] = np.sin(alpha)
    M[1, 1] = np.cos(alpha)
    return M


def TisserandPlot(R_P, v_inf, color):
    RP = np.array([])
    RA = np.array([])
    R_P_vec = np.array([0., - R_P, 0.])
    V_P = V_circular(R_P, astroConstants.MU_SUN)
    V_P_vec = np.array([V_P, 0., 0.])
    points = 360
    alpha_vec = np.linspace(0., astroConstants.MPI, points)
    for alpha in alpha_vec:
        M = rotationMatrix(alpha)
        V_P_hat = normalize(V_P_vec)
        v_inf_vec = v_inf * np.dot(M, V_P_hat)
        V_inf_vec = V_P_vec + v_inf_vec
        (a, e, i, Omega, omega, nu) = RV2OE(
            astroConstants.MU_SUN, R_P_vec, V_inf_vec)
        rp = a * (1. - e)
        ra = a * (1. + e)
        RP = np.append(RP, rp/astroConstants.AU)
        RA = np.append(RA, ra/astroConstants.AU)
    plt.loglog(RP, RA, color)


def Tisserand_P1_2_P2(r_planet_vec, v_inf_P1_vec, v_inf_P2_vec):
    for r_planet in r_planet_vec:
        plt.loglog(r_planet/astroConstants.AU,
                   r_planet/astroConstants.AU, 'ko')

    color_vec = np.array(
        ['dodgerblue', 'lightgreen', 'magenta', 'lightsalmon', 'indigo'])
    color_vec = np.array(['k', 'k', 'k', 'k', 'k'])
    i = 0
    for v_inf in v_inf_P1_vec:
        TisserandPlot(r_planet_vec[0], v_inf, color_vec[i])
        i += 1
    j = 0
    for v_inf in v_inf_P2_vec:
        TisserandPlot(r_planet_vec[1], v_inf, color_vec[j])
        j += 1
    # x-axis plot limits
    l = len(r_planet_vec) - 1
    rp_min = r_planet_vec[0] / astroConstants.AU * 0.2
    rp_max = r_planet_vec[l] / astroConstants.AU * 1.8
    # y-axis plot limits
    ra_min = r_planet_vec[0] / astroConstants.AU * 0.2
    ra_max = r_planet_vec[l] / astroConstants.AU * 1.8
    plt.xlim([rp_min, rp_max])
    plt.ylim([ra_min, ra_max])
    plt.xlabel('Radius of Periapse [AU]')
    plt.ylabel('Radius of Apoapse [AU]')
    plt.title(r'$v_{\infty}^{Earth} $ = ' + str(v_inf_P1_vec) + r' km/s \n'
              r'$v_{\infty}^{Saturn} $ = ' + str(v_inf_P2_vec) + ' km/s')


def Tisserand_ESU():
    r_planet_vec = np.array(
        [astroConstants.SMA_EARTH, astroConstants.SMA_SATURN, astroConstants.SMA_URANUS])
    v_inf_E_vec = np.array([9, 11, 13, 15, 17])
    v_inf_S_vec = np.array([6, 7, 8, 9, 10])
    Tisserand_P1_2_P2(r_planet_vec, v_inf_E_vec, v_inf_S_vec)
    TisserandPlot(astroConstants.SMA_EARTH, v_inf_E_vec[2], 'cyan')
    TisserandPlot(astroConstants.SMA_SATURN, v_inf_S_vec[3], 'cyan')

    plt.show()


def main():
    Mars_RV(JulianDate([2018, 10, 16]))


if __name__ == '__main__':
    main()
