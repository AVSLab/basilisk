
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.




# import required modules:
import math

import numpy as np
from numpy import linalg as la


class ClassicElements(object):
    a = None
    e = None
    i = None
    Omega = None
    omega = None
    f = None
    rmag = None
    alpha = None
    rPeriap = None
    rApoap = None


class EquinoctialElements(object):
    a = None
    P1 = None
    P2 = None
    Q1 = None
    Q2 = None
    l = None
    L = None


N_DEBYE_PARAMETERS = 37  # orbitalMotion.h #

DB0_EPS = 1e-30
eps = 1e-13
maxIteration = 200
tolerance = 1e-15

AU = 149597870.693  # astronomical unit in units of kilometers #
D2R = (np.pi / 180.)

# Gravitational Constants mu = G*m, where m is the planet of the attracting planet.  All units are km^3/s^2.
# Values are obtained from SPICE kernels in http://naif.jpl.nasa.gov/pub/naif/generic_kernels/
MU_SUN = 132712440023.310
MU_MERCURY = 22032.080
MU_VENUS = 324858.599
MU_EARTH = 398600.436
MU_MOON = 4902.799
MU_MARS = 42828.314
MU_JUPITER = 126712767.881
MU_SATURN = 37940626.068
MU_URANUS = 5794559.128
MU_NEPTUNE = 6836534.065
MU_PLUTO = 983.055

# planet information for major solar system bodies. Units are in km.
# data taken from http://nssdc.gsfc.nasa.gov/planetary/planets.html
# Sun #
REQ_SUN = 695000.  # km #

# Mercury #
REQ_MERCURY = 2439.7  # km #
J2_MERCURY = 60.0e-6
SMA_MERCURY = 0.38709893 * AU
I_MERCURY = 7.00487 * D2R
E_MERCURY = 0.20563069

# Venus #
REQ_VENUS = 6051.8  # km #
J2_VENUS = 4.458e-6
SMA_VENUS = 0.72333199 * AU
I_VENUS = 3.39471 * D2R
E_VENUS = 0.00677323

# Earth #
REQ_EARTH = 6378.1366  # km, from SPICE #
RP_EARTH = 6356.7519  # km, from SPICE #
J2_EARTH = 1082.616e-6
J3_EARTH = -2.53881e-6
J4_EARTH = -1.65597e-6
J5_EARTH = -0.15e-6
J6_EARTH = 0.57e-6
SMA_EARTH = 1.00000011 * AU
I_EARTH = 0.00005 * D2R
E_EARTH = 0.01671022
OMEGA_EARTH = 0.00007292115  # Earth's planetary rotation rate, rad/sec #

# Moon #
REQ_MOON = 1737.4
J2_MOON = 202.7e-6
SMA_MOON = 0.3844e6
E_MOON = 0.0549

# Mars #
REQ_MARS = 3396.19  # km #
RP_MARS = 3376.2  # km #
J2_MARS = 1960.45e-6
SMA_MARS = 1.52366231 * AU
I_MARS = 1.85061 * D2R
E_MARS = 0.09341233
OMEGA_MARS = 0.00007262203  # Mars' polar rotation rate, rad/sec #

# Phobos #
REQ_PHOBOS = 11.2  # km #

# Deimos #
REQ_DEIMOS = 6.1  # km #

# Jupiter #
REQ_JUPITER = 71492.
J2_JUPITER = 14736.e-6
SMA_JUPITER = 5.20336301 * AU
I_JUPITER = 1.30530 * D2R
E_JUPITER = 0.04839266

# Saturn #
REQ_SATURN = 60268.
J2_SATURN = 16298.e-6
SMA_SATURN = 9.53707032 * AU
I_SATURN = 2.48446 * D2R
E_SATURN = 0.05415060

# Uranus #
REQ_URANUS = 25559.
J2_URANUS = 3343.43e-6
SMA_URANUS = 19.19126393 * AU
I_URANUS = 0.76986 * D2R
E_URANUS = 0.04716771

# Neptune #
REQ_NEPTUNE = 24746.
J2_NEPTUNE = 3411.e-6
SMA_NEPTUNE = 30.06896348 * AU
I_NEPTUNE = 1.76917 * D2R
E_NEPTUNE = 0.00858587

# Pluto #
REQ_PLUTO = 1137.
SMA_PLUTO = 39.48168677 * AU
I_PLUTO = 17.14175 * D2R
E_PLUTO = 0.24880766


def E2f(Ecc, e):
    """
    Maps eccentric anomaly angles into true anomaly angles
    This function requires the orbit to be either circular or
    non-rectilinear elliptic orbit

    :param Ecc: eccentric anomaly (rad)
    :param e: eccentric (0 <= e < 1)
    :return: f, true anomaly (rad)
    """
    if e >= 0.0 and e < 1.0:
        f = 2.0 * math.atan2(math.sqrt(1.0 + e) * math.sin(Ecc / 2.0), math.sqrt(1.0 - e) * math.cos(Ecc / 2.0))
        return f
    raise ValueError('Error: E2f() received e = {}, the value of e should be 0 <= e < 1'.format(str(e)))


def E2M(Ecc, e):
    """
    Maps the eccentric anomaly angle into the corresponding
    mean elliptic anomaly angle.  Both 2D and 1D elliptic
    orbit are allowed.

    :param Ecc: eccentric anomaly (rad)
    :param e: eccentricity (0 <= e < 1)
    :return: M, mean elliptic anomaly (rad)
    """
    if e >= 0.0 and e < 1.0:
        M = Ecc - e * math.sin(Ecc)
        return M
    raise ValueError('Error: E2M() received e = {}, the value of e should be 0 <= e < 1'.format(str(e)))


def f2E(f, e):
    """
    Maps true anomaly angles into eccentric anomaly angles.
    This function requires the orbit to be either circular or
    non-rectilinear elliptic orbit.

    :param f: true anomaly angle (rad)
    :param e: eccentricity (0 <= e < 1)
    :return: Ecc, eccentric anomaly (rad)
    """
    if e >= 0.0 and e < 1.0:
        Ecc = 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(f / 2.0), math.sqrt(1.0 + e) * math.cos(f / 2.0))
        return Ecc
    raise ValueError('Error: f2E() received e = {}, the value of e should be 0 <= e < 1'.format(str(e)))


def f2H(f, e):
    """
    Maps true anomaly angles into hyperbolic anomaly angles.
    This function requires the orbit to be hyperbolic

    :param f: true anomaly angle (rad)
    :param e: eccentricity (e > 1)
    :return:  H, hyperbolic anomaly (rad)
    """
    if e > 1.0:
        H = 2.0 * math.atanh(math.sqrt((e - 1.0) / (e + 1.0)) * math.tan(f / 2.0))
        return H
    raise ValueError('Error: f2H() received e = {}, the value of e should be 0 <= e < 1'.format(str(e)))


def H2f(H, e):
    """
    Maps hyperbolic anomaly angles into true anomaly angles.
    This function requires the orbit to be hyperbolic

    :param H: hyperbolic anomaly (rad)
    :param e: eccentricity (e > 1)
    :return: f, true anomaly angle (rad)
    """
    if e > 1.0:
        f = 2.0 * math.atan(math.sqrt((e + 1.0) / (e - 1.0)) * math.tanh(H / 2.0))
        return f
    raise ValueError('Error: H2f() received e = {}, the value of e should be 0 <= e < 1'.format(str(e)))


def H2N(H, e):
    """
    Maps the hyperbolic anomaly angle H into the corresponding
    mean hyperbolic anomaly angle N.

    :param H: hyperbolic anomaly (rad)
    :param e: eccentricity (e > 1)
    :return: N, mean hyperbolic anomaly (rad)
    """
    if e > 1.0:
        N = e * math.sinh(H) - H
        return N
    raise ValueError('Error: H2N() received e = {}, the value of e should be 0 <= e < 1'.format(str(e)))


def M2E(M, e):
    """
    Maps the mean elliptic anomaly angle into the corresponding
    eccentric anomaly angle.  Both 2D and 1D elliptic
    orbit are allowed.

    :param M: mean elliptic anomaly (rad)
    :param e: eccentricity (0 <= e < 1)
    :return: Ecc, eccentric anomaly (rad)
    """
    dE = 10.0 * eps
    E1 = M
    count = 0

    if e >= 0.0 and e < 1.0:
        while math.fabs(dE) > eps:
            dE = (E1 - e * math.sin(E1) - M) / (1 - e * math.cos(E1))
            E1 -= dE
            count += 1
            if count > maxIteration:
                print('Iteration error in M2E({},{})'.format(str(M), str(e)))
                dE = 0.0
        return E1
    raise ValueError('Error: M2E() received e = {}, the value of e should be 0 <= e < 1'.format(str(e)))


def N2H(N, e):
    """
    Maps the mean hyperbolic anomaly angle N into the corresponding
    hyperbolic anomaly angle H.

    :param N: mean hyperbolic anomaly (rad)
    :param e: eccentricity (e > 1)
    :return: H, hyperbolic anomaly (rad)
    """
    dH = 10.0 * eps
    H1 = N
    count = 0

    if e > 1.0:
        while math.fabs(dH) > eps:
            dH = (e * math.sinh(H1) - H1 - N) / (e * math.cosh(H1) - 1.0)
            H1 -= dH
            count += 1
            if count > maxIteration:
                print('Iteration error in M2E({},{})'.format(str(N), str(e)))
                dH = 0.
        return H1
    raise ValueError('Error: N2H() received e = {}, the value of e should be 0 <= e < 1'.format(str(e)))

def elem2rv_parab(mu, elements):
    """
    Translates the orbit elements:

    === ========================= =======
    a   semi-major axis           km
    e   eccentricity
    i   inclination               rad
    AN  ascending node            rad
    AP  argument of periapses     rad
    f   true anomaly angle        rad
    === ========================= =======

    to the inertial Cartesian position and velocity vectors.
    The attracting body is specified through the supplied
    gravitational constant mu (units of km^3/s^2).

    The code can handle the following cases:

    ================== ============  ===========   =======================
        circular:       e = 0           a > 0
        elliptical-2D:  0 < e < 1       a > 0
        elliptical-1D:  e = 1           a > 0        f = Ecc. Anom. here
        parabolic:      e = 1           rp = -a
        hyperbolic:     e > 1           a < 0
    ================== ============  ===========   =======================

    .. note::

        To handle the parabolic case and distinguish it form the
        rectilinear elliptical case, instead of passing along the
        semi-major axis a in the "a" input slot, the negative radius
        at periapses is supplied.  Having "a" be negative and e = 1
        is a then a unique identified for the code for the parabolic
        case.

    :param mu: gravitational parameter
    :param elements: orbital elements
    :return:   rVec = position vector, vVec = velocity vector
    """
    # map classical elements structure into local variables #
    a = elements.a
    e = elements.e
    i = elements.i
    AN = elements.Omega
    AP = elements.omega
    f = elements.f

    ir = np.zeros(3)
    rVec = np.zeros(3)
    vVec = np.zeros(3)

    # TODO: Might want to have an error band on this equality #
    if e == 1.0 and a > 0.0:  # rectilinear elliptic orbit case #
        Ecc = f  # f is treated as ecc. anomaly #
        r = a * (1.0 - e * math.cos(Ecc))  # orbit radius #
        v = math.sqrt(2.0 * mu / r - mu / a)
        ir[0] = math.cos(AN) * math.cos(AP) - math.sin(AN) * math.sin(AP) * math.cos(i)
        ir[1] = math.sin(AN) * math.cos(AP) + math.cos(AN) * math.sin(AP) * math.cos(i)
        ir[2] = math.sin(AP) * math.sin(i)
        rVec = r * ir
        if math.sin(Ecc) > 0.0:
            vVec = -v * ir
        else:
            vVec = v * ir

    else:
        if e == 1.0 and a < 0.0:    # parabolic case #
            rp = -a                  # radius at periapses #
            p = 2.0 * rp             # semi-latus rectum #
        else:                       # elliptic and hyperbolic cases #
            p = a * (1.0 - e * e)   # semi-latus rectum #

        r = p / (1.0 + e * math.cos(f))  # orbit radius #
        theta = AP + f                   # true latitude angle #
        h = math.sqrt(mu * p)            # orbit ang. momentum mag.

        rVec[0] = r * (math.cos(AN) * math.cos(theta) - math.sin(AN) * math.sin(theta) * math.cos(i))
        rVec[1] = r * (math.sin(AN) * math.cos(theta) + math.cos(AN) * math.sin(theta) * math.cos(i))
        rVec[2] = r * (math.sin(theta) * math.sin(i))

        vVec[0] = -mu / h * (math.cos(AN) * (math.sin(theta) + e * math.sin(AP)) + math.sin(AN) * (math.cos(
            theta) + e * math.cos(AP)) * math.cos(i))
        vVec[1] = -mu / h * (math.sin(AN) * (math.sin(theta) + e * math.sin(AP)) - math.cos(AN) * (math.cos(
            theta) + e * math.cos(AP)) * math.cos(i))
        vVec[2] = -mu / h * (-(math.cos(theta) + e * math.cos(AP)) * math.sin(i))

    return rVec, vVec

def elem2rv(mu, elements):
    """
    Translates the orbit elements:

    === ========================= =======
    a   semi-major axis           km
    e   eccentricity
    i   inclination               rad
    AN  ascending node            rad
    AP  argument of periapses     rad
    f   true anomaly angle        rad
    === ========================= =======

    to the inertial Cartesian position and velocity vectors.
    The attracting body is specified through the supplied
    gravitational constant mu (units of km^3/s^2).

    :param mu: gravitational parameter
    :param elements: orbital elements
    :return:   rVec, position vector
    :return:   vVec, velocity vector
    """
    rVec = np.zeros(3)
    vVec = np.zeros(3)

    if 1.0 + elements.e * math.cos(elements.f) < tolerance:
        print('WARNING: Radius is near infinite in elem2rv conversion.')

    # Calculate the semilatus rectum and the radius #
    p = elements.a * (1.0 - elements.e * elements.e)
    r = p / (1.0 + elements.e * math.cos(elements.f))
    theta = elements.omega + elements.f
    rVec[0] = r * (
        math.cos(theta) * math.cos(elements.Omega) - math.cos(elements.i) * math.sin(theta) * math.sin(
            elements.Omega))
    rVec[1] = r * (
        math.cos(theta) * math.sin(elements.Omega) + math.cos(elements.i) * math.sin(theta) * math.cos(
            elements.Omega))
    rVec[2] = r * (math.sin(theta) * math.sin(elements.i))

    if math.fabs(p) < tolerance:
        if math.fabs(1.0 - elements.e) < tolerance:
            # Rectilinear orbit #
            raise ValueError('elem2rv does not support rectilinear orbits')
        # Parabola #
        rp = -elements.a
        p = 2.0 * rp

    h = math.sqrt(mu * p)
    vVec[0] = -mu / h * (math.cos(elements.Omega) * (elements.e * math.sin(elements.omega) + math.sin(theta)) +
                         math.cos(elements.i) * (
                             elements.e * math.cos(elements.omega) + math.cos(theta)) * math.sin(elements.Omega))
    vVec[1] = -mu / h * (math.sin(elements.Omega) * (elements.e * math.sin(elements.omega) + math.sin(theta)) -
                         math.cos(elements.i) * (elements.e * math.cos(elements.omega) + math.cos(theta)) *
                         math.cos(elements.Omega))
    vVec[2] = mu / h * (elements.e * math.cos(elements.omega) + math.cos(theta)) * math.sin(elements.i)

    return rVec, vVec

def rv2elem_parab(mu, rVec, vVec):
    """
    Translates the orbit elements inertial Cartesian position
    vector rVec and velocity vector vVec into the corresponding
    classical orbit elements where

    === ========================= =======
    a   semi-major axis             km
    e   eccentricity
    i   inclination                 rad
    AN  ascending node              rad
    AP  argument of periapses       rad
    f   true anomaly angle          rad
    === ========================= =======

    If the orbit is rectilinear, then f will be the eccentric or hyperbolic anomaly

    The attracting body is specified through the supplied
    gravitational constant mu (units of km^3/s^2).

    The code can handle the following cases:

    ============== ============= ===========
    circular:       e = 0           a > 0
    elliptical-2D:  0 < e < 1       a > 0
    elliptical-1D:  e = 1           a > 0
    parabolic:      e = 1           a = -rp
    hyperbolic:     e > 1           a < 0
    ============== ============= ===========

    For the parabolic case the semi-major axis is not defined.
    In this case -rp (radius at periapses) is returned instead
    of a.  For the circular case, the AN and AP are ill-defined,
    along with the associated ie and ip unit direction vectors
    of the perifocal frame. In this circular orbit case, the
    unit vector ie is set equal to the normalized inertial
    position vector ir.

    :param   mu:  gravitational parameter
    :param   rVec:  position vector
    :param   vVec: velocity vector
    :return: orbital elements


    Todo: Modify this code to return true longitude of periapsis
    (non-circular, equatorial), argument of latitude (circular, inclined),
    and true longitude (circular, equatorial) when appropriate instead of
    simply zeroing out omega and Omega

    """

    dum = np.zeros(3)
    dum2 = np.zeros(3)
    ie = np.zeros(3)
    ip = np.zeros(3)
    ih = np.zeros(3)
    cVec = np.zeros(3)
    hVec = np.zeros(3)
    ir = np.zeros(3)
    elements = ClassicElements()

    # compute orbit radius #
    r = la.norm(rVec)
    elements.rmag = r
    ir = v3Normalize(rVec)

    # compute the angular momentum vector #
    hVec = np.cross(rVec, vVec)
    h = la.norm(hVec)

    # compute the eccentricity vector #
    cVec = np.cross(vVec, hVec)
    dum = (-mu / r) * rVec
    cVec = cVec + dum
    elements.e = la.norm(cVec) / mu

    # compute semi-major axis #
    elements.alpha = 2.0 / r - np.dot(vVec, vVec) / mu
    if math.fabs(elements.alpha) > eps:
        # elliptic or hyperbolic case #
        elements.a = 1.0 / elements.alpha
        elements.rPeriap = elements.a * (1.0 - elements.e)
        elements.rApoap = elements.a * (1.0 + elements.e)
    else:
        #  parabolic case #
        elements.alpha = 0.
        p = h * h / mu
        rp = p / 2.0
        elements.a = -rp  # a is not defined for parabola, so - rp is returned instead #
        elements.e = 1.0
        elements.rPeriap = rp
        elements.rApoap = -rp  # periapses radius doesn't exist, returning -rp instead #

    if h < eps:  # rectilinear motion case #
        dum = np.array([0.0, 0.0, 1.0])
        dum2 = np.array([0.0, 1.0, 0.0])
        ih = np.cross(ie, dum)
        ip = np.cross(ie, dum2)
        if la.norm(ih) > la.norm(ip):
            ih = v3Normalize(ih)
        else:
            ih = v3Normalize(ip)
        ip = np.cross(ih, ie)
    else:
        ih = v3Normalize(hVec)
        if math.fabs(elements.e) > eps:
            ie = (1.0 / mu / elements.e) * cVec
        else:
            ie = ir
        # circular orbit case.  Here ie, ip are arbitrary, as long as they #
        # are perpenticular to the ih vector. #
        ip = np.cross(ih, ie)

    # compute the 3-1-3 orbit plane orientation angles #
    elements.i = math.acos(ih[2])
    if elements.i > eps and elements.i < np.pi - eps:
        elements.Omega = math.atan2(ih[0], -ih[1])
        elements.omega = math.atan2(ie[2], ip[2])
    else:
        elements.Omega = 0.
        elements.omega = math.atan2(ie[1], ie[0])

    if h < eps:  # rectilinear motion case #
        if elements.alpha > 0:  # elliptic case #
            Ecc = math.acos(1 - r * elements.alpha)
            if np.dot(rVec, vVec) > 0:
                Ecc = 2.0 * np.pi - Ecc
            elements.f = Ecc  # for this mode the eccentric anomaly is returned #
        else:  # hyperbolic case #
            H = math.acosh(r * elements.alpha + 1)
            if np.dot(rVec, vVec) < 0:
                H = 2.0 * np.pi - H
            elements.f = H  # for this mode the hyperbolic anomaly is returned #
    else:
        # compute true anomaly #
        dum = np.cross(ie, ir)
        elements.f = math.atan2(np.dot(dum, ih), np.dot(ie, ir))

    return elements

def rv2elem(mu, rVec, vVec):
    """
    Translates the orbit elements inertial Cartesian position
    vector rVec and velocity vector vVec into the corresponding
    classical orbit elements where

    === ========================= =======
    a   semi-major axis           km
    e   eccentricity
    i   inclination               rad
    AN  ascending node            rad
    AP  argument of periapses     rad
    f   true anomaly angle        rad
    === ========================= =======

    If the orbit is rectilinear, then this will be the eccentric or hyperbolic anomaly

    The attracting body is specified through the supplied
    gravitational constant mu (units of km^3/s^2).

    :param mu:  gravitational parameter
    :param rVec: position vector
    :param vVec: velocity vector
    :return:  orbital elements
    """
    hVec = [0.0] * 3
    v3 = [0.0] * 3
    nVec = [0.0] * 3
    eVec = [0.0] * 3
    eVec2 = [0.0] * 3

    elements = ClassicElements()

    if (np.isnan(np.sum(rVec)) or np.isnan(np.sum(vVec))):
        print("ERROR: received NAN rVec or vVec values.")
        elements.a = np.nan
        elements.alpha = np.nan
        elements.e = np.nan
        elements.i = np.nan
        elements.AN = np.nan
        elements.AP = np.nan
        elements.f = np.nan
        elements.rmag = np.nan
        return


    # Calculate the specific angular momentum and its magnitude #
    hVec = np.cross(rVec, vVec)
    h = la.norm(hVec)
    p = h * h / mu

    # Calculate the line of nodes #
    v3 = np.array([0.0, 0.0, 1.0])
    nVec = np.cross(v3, hVec)
    n = la.norm(nVec)

    # Orbit eccentricity and energy #
    r = la.norm(rVec)
    v = la.norm(vVec)
    eVec = (v * v / mu - 1.0 / r) * rVec
    v3 = (np.dot(rVec, vVec) / mu) * vVec
    eVec = eVec - v3
    elements.e = la.norm(eVec)
    elements.rmag = r
    elements.rPeriap = p / (1.0 + elements.e)

    # compute semi-major axis #
    elements.alpha = 2.0 / r - v * v / mu
    if math.fabs(elements.alpha) > eps:
        # elliptic or hyperbolic case #
        elements.a = 1.0 / elements.alpha
        elements.rApoap = p / (1.0 - elements.e)
    else:
        rp = p / 2.
        elements.a = -rp
        elements.rApoap = -1.0

    # Calculate the inclination #
    elements.i = math.acos(hVec[2] / h)

    if elements.e >= 1e-11 and elements.i >= 1e-11:
        # Case 1: Non-circular, inclined orbit #
        elements.Omega = math.acos(nVec[0] / n)
        if nVec[1] < 0.0:
            elements.Omega = 2.0 * np.pi - elements.Omega
        elements.omega = math.acos(np.clip(np.dot(nVec, eVec) / n / elements.e, a_min=-1.0, a_max=1.0))
        if eVec[2] < 0.0:
            elements.omega = 2.0 * np.pi - elements.omega
        elements.f = math.acos(np.clip(np.dot(eVec, rVec) / elements.e / r, a_min=-1.0, a_max=1.0))
        if np.dot(rVec, vVec) < 0.0:
            elements.f = 2.0 * np.pi - elements.f
    elif elements.e >= 1e-11 and elements.i < 1e-11:
        # Case 2: Non-circular, equatorial orbit #
        # Equatorial orbit has no ascending node #
        elements.Omega = 0.0
        # True longitude of periapsis, omegatilde_true #
        elements.omega = math.acos(eVec[0] / elements.e)
        if eVec[1] < 0.0:
            elements.omega = 2.0 * np.pi - elements.omega
        elements.f = math.acos(np.clip(np.dot(eVec, rVec) / elements.e / r, a_min=-1.0, a_max=1.0))
        if np.dot(rVec, vVec) < 0.0:
            elements.f = 2.0 * np.pi - elements.f
    elif elements.e < 1e-11 and elements.i >= 1e-11:
        # Case 3: Circular, inclined orbit #
        elements.Omega = math.acos(nVec[0] / n)
        if nVec[1] < 0.0:
            elements.Omega = 2.0 * np.pi - elements.Omega
        elements.omega = 0.0
        # Argument of latitude, u = omega + f #
        elements.f = math.acos(np.clip(np.dot(nVec, rVec) / n / r, a_min=-1.0, a_max=1.0))
        if rVec[2] < 0.0:
            elements.f = 2.0 * np.pi - elements.f
    elif elements.e < 1e-11 and elements.i < 1e-11:
        # Case 4: Circular, equatorial orbit #
        elements.Omega = 0.0
        elements.omega = 0.0
        # True longitude, lambda_true #
        elements.f = math.acos(rVec[0] / r)
        if rVec[1] < 0:
            elements.f = 2.0 * np.pi - elements.f
    else:
        print("Error: rv2elem couldn't identify orbit type.")
    if elements.e > 1.0 and math.fabs(elements.f) > np.pi:
        twopiSigned = math.copysign(2.0 * np.pi, elements.f)
        elements.f -= twopiSigned

    return elements


def atmosphericDensity(alt):
    """
    This program computes the atmospheric density based on altitude
    supplied by user.  This function uses a curve fit based on
    atmospheric data from the Standard Atmosphere 1976 Data. This
    function is valid for altitudes ranging from 100km to 1000km.

    .. note::

        This code can only be applied to spacecraft orbiting the Earth

    :param alt: altitude in km
    :return:  density at the given altitude in kg/m^3
    """
    # Smooth exponential drop-off after 1000 km #
    if alt > 1000.:
        logdensity = (-7E-05) * alt - 14.464
        density = math.pow(10., logdensity)
        return density

    # Calculating the density based on a scaled 6th order polynomial fit to the log of density #
    val = (alt - 526.8000) / 292.8563
    logdensity = 0.34047 * math.pow(val, 6) - 0.5889 * math.pow(val, 5) - 0.5269 * math.pow(val, 4) \
                 + 1.0036 * math.pow(val, 3) + 0.60713 * math.pow(val, 2) - 2.3024 * val - 12.575

    # Calculating density by raising 10 to the log of density #
    density = math.pow(10., logdensity)

    return density


def debyeLength(alt):
    """
    This program computes the debyeLength length for a given
    altitude and is valid for altitudes ranging
    from 200 km to GEO (35000km).  However, all values above
    1000 km are HIGHLY speculative at this point.

    :param alt: altitude in km
    :return: debye length given in m
    """
    X = [200.0, 250.0, 300.0, 350.0, 400., 450., 500., 550., 600., 650., 700., 750., 800., 850.,
         900., 950., 1000., 1050., 1100., 1150., 1200., 1250., 1300., 1350., 1400., 1450.,
         1500., 1550., 1600., 1650., 1700., 1750., 1800., 1850., 1900., 1950., 2000.]

    Y = [5.64E-03, 3.92E-03, 3.24E-03, 3.59E-03, 4.04E-03, 4.28E-03, 4.54E-03, 5.30E-03, 6.55E-03,
         7.30E-03, 8.31E-03, 8.38E-03, 8.45E-03, 9.84E-03, 1.22E-02, 1.37E-02, 1.59E-02, 1.75E-02,
         1.95E-02, 2.09E-02, 2.25E-02, 2.25E-02, 2.25E-02, 2.47E-02, 2.76E-02, 2.76E-02, 2.76E-02,
         2.76E-02, 2.76E-02, 2.76E-02, 2.76E-02, 3.21E-02, 3.96E-02, 3.96E-02, 3.96E-02, 3.96E-02, 3.96E-02]

    # Flat debyeLength length for altitudes above 2000 km #
    if alt > 2000.0 and alt <= 30000.0:
        alt = 2000.0
    elif alt > 30000.0 and alt <= 35000.0:
        debyedist = 0.1 * alt - 2999.7
        return debyedist
    elif alt < 200.0 or alt > 35000.0:
        raise ValueError("ERROR: debyeLength() received alt = {}, the value of alt should be in the range "
                         "of [200 35000].".format(str(alt)))
    # Interpolation of data #
    i = 0
    for i in range(0, N_DEBYE_PARAMETERS - 1):
        if X[i + 1] > alt:
            break
    a = (alt - X[i]) / (X[i + 1] - X[i])
    debyedist = Y[i] + a * (Y[i + 1] - Y[i])

    return debyedist


def atmosphericDrag(Cd, A, m, rvec, vvec):
    """
     This program computes the atmospheric drag acceleration
     vector acting on a spacecraft.
     Note the acceleration vector output is inertial, and is
     only valid for altitudes up to 1000 km.
     Afterwards the drag force is zero. Only valid for Earth.

     :param Cd:  drag coefficient of the spacecraft
     :param A: cross-sectional area of the spacecraft in m^2
     :param m: mass of the spacecraft in kg
     :param rvec: Inertial position vector of the spacecraft in km  [x;y;z]
     :param vvec: Inertial velocity vector of the spacecraft in km/s [vx;vy;vz]
     :return: The inertial acceleration vector due to atmospheric drag in km/sec^2
    """
    # find the altitude and velocity #
    r = la.norm(rvec)
    v = la.norm(vvec)
    alt = r - REQ_EARTH
    advec = np.zeros(3)

    # Checking if user supplied a orbital position is insede the earth #
    if alt <= 0.:
        print("ERROR: atmosphericDrag() received rvec = [{} {} {}].". \
            format(str(rvec[1]), str(rvec[2]), str(rvec[3])))
        print('The value of rvec should produce a positive altitude for the Earth.')
        advec.fill(np.nan)
        return

    # get the Atmospheric density at the given altitude in kg/m^3 #
    density = atmosphericDensity(alt)

    # compute the magnitude of the drag acceleration #
    ad = ((-0.5) * density * (Cd * A / m) * (math.pow(v * 1000., 2))) / 1000.

    # computing the vector for drag acceleration #
    advec = (ad / v) * vvec

    return advec


def jPerturb(rvec, num, planet):
    """
    Computes the J2_EARTH-J6_EARTH zonal gravitational perturbation
    accelerations.

    :param rvec: Cartesian Position vector in kilometers [x;y;z].
    :param num: Corresponds to which J components to use,
                must be an integer between 2 and 6.
                (note: Additive- 2 corresponds to J2_EARTH while 3 will
                correspond to J2_EARTH + J3_EARTH)
    :param planet: planet variable, can be
                 CELESTIAL_MERCURY
                 CELESTIAL_VENUS
                 CELESTIAL_EARTH
                 CELESTIAL_MOON
                 CELESTIAL_MARS
                 CELESTIAL_JUPITER
                 CELESTIAL_URANUS
                 CELESTIAL_NEPTUNE
    :return: ajtot, The total acceleration vector due to the J
                    perturbations in km/sec^2 [accelx;accely;accelz]
    """

    ajtot = np.zeros(3)

    # default #
    mu = MU_EARTH
    req = REQ_EARTH
    J2 = J2_EARTH
    J3 = J3_EARTH
    J4 = J4_EARTH
    J5 = J5_EARTH
    J6 = J6_EARTH

    if planet == 'CELESTIAL_MERCURY':
        mu = MU_MERCURY
        req = REQ_MERCURY
        J2 = J2_MERCURY
        J3 = 0.0
        J4 = 0.0
        J5 = 0.0
        J6 = 0.0
    elif planet == 'CELESTIAL_VENUS':
        mu = MU_VENUS
        req = REQ_VENUS
        J2 = J2_VENUS
        J3 = 0.0
        J4 = 0.0
        J5 = 0.0
        J6 = 0.0
    elif planet == 'CELESTIAL_MOON':
        mu = MU_MOON
        req = REQ_MOON
        J2 = J2_MOON
        J3 = 0.0
        J4 = 0.0
        J5 = 0.0
        J6 = 0.0
    elif planet == 'CELESTIAL_MARS':
        mu = MU_MARS
        req = REQ_MARS
        J2 = J2_MARS
        J3 = 0.0
        J4 = 0.0
        J5 = 0.0
        J6 = 0.0
    elif planet == 'CELESTIAL_JUPITER':
        mu = MU_JUPITER
        req = REQ_JUPITER
        J2 = J2_JUPITER
        J3 = 0.0
        J4 = 0.0
        J5 = 0.0
        J6 = 0.0
    elif planet == 'CELESTIAL_URANUS':
        mu = MU_URANUS
        req = REQ_URANUS
        J2 = J2_URANUS
        J3 = 0.0
        J4 = 0.0
        J5 = 0.0
        J6 = 0.0
    elif planet == 'CELESTIAL_NEPTUNE':
        mu = MU_NEPTUNE
        req = REQ_NEPTUNE
        J2 = J2_NEPTUNE
        J3 = 0.0
        J4 = 0.0
        J5 = 0.0
        J6 = 0.0
    elif planet == 'CELESTIAL_PLUTO':
        return
    elif planet == 'CELESTIAL_SUN':
        return

    # Calculate the J perturbations #
    x = rvec[0]
    y = rvec[1]
    z = rvec[2]
    r = la.norm(rvec)

    # Error Checking #
    if num < 2 or num > 6:
        raise ValueError("ERROR: jPerturb() received num = {}.The value of num should be 2 <= num <= 6.".format(str(num)))

    # Calculating the total acceleration based on user input #
    if num >= 2:
        ajtot = np.array([(1.0 - 5.0 * math.pow(z / r, 2.0)) * (x / r),
                          (1.0 - 5.0 * math.pow(z / r, 2.0)) * (y / r),
                          (3.0 - 5.0 * math.pow(z / r, 2.0)) * (z / r)])
        ajtot = (-3.0 / 2.0 * J2 * (mu / math.pow(r, 2.0)) * math.pow(req / r, 2.0)) * ajtot
    if num >= 3:
        temp = np.array([5.0 * (7.0 * math.pow(z / r, 3.0) - 3.0 * (z / r)) * (x / r),
                         5.0 * (7.0 * math.pow(z / r, 3.0) - 3.0 * (z / r)) * (y / r),
                         -3.0 * (10.0 * math.pow(z / r, 2.0) - (35.0 / 3.0) * math.pow(z / r, 4.0) - 1.0)])
        temp2 = (1.0 / 2.0 * J3 * (mu / math.pow(r, 2.0)) * math.pow(req / r, 3.0)) * temp
        ajtot = ajtot + temp2
    if num >= 4:
        temp = np.array([(3.0 - 42.0 * math.pow(z / r, 2.0) + 63.0 * math.pow(z / r, 4.0)) * (x / r),
                         (3.0 - 42.0 * math.pow(z / r, 2.0) + 63.0 * math.pow(z / r, 4.0)) * (y / r),
                         (15.0 - 70.0 * math.pow(z / r, 2) + 63.0 * math.pow(z / r, 4.0)) * (z / r)])
        temp2 = (5.0 / 8.0 * J4 * (mu / math.pow(r, 2.0)) * math.pow(req / r, 4.0)) * temp
        ajtot = ajtot + temp2
    if num >= 5:
        temp = np.array([3.0 * (35.0 * (z / r) - 210.0 * math.pow(z / r, 3.0) + 231.0 * math.pow(z / r, 5.0)) * (x / r),
                         3.0 * (35.0 * (z / r) - 210.0 * math.pow(z / r, 3.0) + 231.0 * math.pow(z / r, 5.0)) * (y / r),
                         -(15.0 - 315.0 * math.pow(z / r, 2.0) + 945.0 * math.pow(z / r, 4.0) - 693.0 * math.pow(z / r,
                                                                                                                 6.0))])
        temp2 = (1.0 / 8.0 * J5 * (mu / math.pow(r, 2.0)) * math.pow(req / r, 5.0)) * temp
        ajtot = ajtot + temp2
    if num >= 6:
        temp = np.array([(35.0 - 945.0 * math.pow(z / r, 2) + 3465.0 * math.pow(z / r, 4.0) - 3003.0 * math.pow(z / r,
                                                                                                                6.0)) * (
                             x / r),
                         (35.0 - 945.0 * math.pow(z / r, 2.0) + 3465.0 * math.pow(z / r, 4.0) - 3003.0 * math.pow(z / r,
                                                                                                                  6.0)) * (
                             y / r),
                         -(3003.0 * math.pow(z / r, 6.0) - 4851.0 * math.pow(z / r, 4.0) + 2205.0 * math.pow(z / r,
                                                                                                             2.0) - 245.0) * (
                             z / r)])
        temp2 = (-1.0 / 16.0 * J6 * (mu / math.pow(r, 2.0)) * math.pow(req / r, 6.0)) * temp
        ajtot = ajtot + temp2

    return ajtot


def solarRad(A, m, sunvec):
    """
    Computes the inertial solar radiation force vectors
    based on cross-sectional Area and mass of the spacecraft
    and the position vector of the planet to the sun.

    .. note::

        It is assumed that the solar radiation pressure decreases quadratically with distance from sun (in AU)

    Solar Radiation Equations obtained from
    Earth Space and Planets Journal Vol. 51, 1999 pp. 979-986

    :param A: Cross-sectional area of the spacecraft that is facing the sun in m^2.
    :param m: The mass of the spacecraft in kg.
    :param sunvec: Position vector to the Sun in units of AU. Earth has a distance of 1 AU.
    :return:   arvec, The inertial acceleration vector due to the effects of Solar Radiation pressure in km/sec^2.  The vector
               components of the output are the same as the vector
               components of the sunvec input vector.
    """
    # Solar Radiation Flux #
    flux = 1372.5398

    # Speed of light #
    c = 299792458.

    # Radiation pressure coefficient #
    Cr = 1.3

    # Magnitude of position vector #
    sundist = la.norm(sunvec)

    # Computing the acceleration vector #
    arvec = ((-Cr * A * flux) / (m * c * math.pow(sundist, 3)) / 1000.) * sunvec

    return arvec


def v3Normalize(v):
    result = np.zeros(3)
    norm = la.norm(v)
    if norm > DB0_EPS:
        result = (1. / norm) * v
    return result


def clMeanOscMap(req, J2, oe, oep, sign):
    """
    First-order J2 Mapping Between Mean and Osculating Orbital Elements

    Analytical Mechanics of Space Systems, Hanspeter Schaub, John L. Junkins, 4th edition.
    [m] or [km] should be the same both for req and elements.a

    :param req: equatorial radius
    :param J2:
    :param oe: classical orbit element set
    :param oep:
    :param sign: sgn=1:mean to osc, sgn=-1:osc to mean

    """
    a       = oe.a
    e       = oe.e
    i       = oe.i
    Omega   = oe.Omega
    omega   = oe.omega
    f       = oe.f
    E       = f2E(f, e)
    M       = E2M(E, e)
    gamma2  = sign*J2/2*((req/oe.a)**2)
    eta     = math.sqrt(1-oe.e*oe.e)
    gamma2p = gamma2/(eta**4)
    a_r     = (1+oe.e*math.cos(oe.f))/(eta**2)
    # calculate oep.a
    ap = oe.a + oe.a*gamma2*((3*(math.cos(oe.i))**2-1)*(a_r**3-1/(eta**3)) \
       +3*(1-(math.cos(oe.i))**2)*(a_r**3)*math.cos(2*oe.omega+2*oe.f))  # (F.7)

    de1 = gamma2p/8*e*(eta**2)*(1-11*((math.cos(i))**2)-40*((math.cos(i)) **4) \
        /(1-5*((math.cos(i))**2)))*math.cos(2*omega)  # (F.8)

    de = de1 + (eta ** 2) / 2 * (gamma2 *((3 * ((math.cos(i)) ** 2) - 1) / (eta ** 6) \
       *(e * eta + e / (1 + eta) + 3 * math.cos(f) + 3 * e * ((math.cos(f)) ** 2) + (e ** 2) \
       *((math.cos(f)) ** 3)) + 3 * (1 - ((math.cos(i)) ** 2)) / (eta ** 6) \
       *(e + 3 * math.cos(f) + 3 * e * ((math.cos(f)) ** 2) + (e ** 2) * ((math.cos(f)) ** 3)) * math.cos(2 * omega + 2 * f)) \
       - gamma2p * (1 - ((math.cos(i)) ** 2)) *(3 * math.cos(2 * omega + f) + math.cos(2 * omega + 3 * f)))  # (F.9)

    di = -e*de1/(eta**2)/math.tan(i) + gamma2p/2*math.cos(i)*math.sqrt(1-((math.cos(i))**2)) \
       *(3*math.cos(2*omega+2*f) + 3*e*math.cos(2*omega+f)+e*math.cos(2*omega+3*f))  # (F.10)

    MpopOp = M + omega + Omega + gamma2p / 8 * (eta ** 3) * (1 - 11 * ((math.cos(i)) ** 2) \
           - 40 *((math.cos(i)) ** 4) / (1 - 5 * ((math.cos(i)) ** 2))) * math.sin(2 * omega) \
           - gamma2p / 16 * (2 + (e ** 2) - 11 * (2 + 3 * (e ** 2)) * ((math.cos(i)) ** 2) - 40 * (2 + 5 * (e ** 2)) \
           *((math.cos(i)) ** 4) / (1 - 5 * ((math.cos(i)) ** 2)) - 400 * (e ** 2) * ((math.cos(i)) ** 6) \
           /((1 - 5 * ((math.cos(i)) ** 2)) ** 2)) * math.sin(2 * omega) \
           + gamma2p / 4 * (-6 *(1 - 5 * ((math.cos(i)) ** 2)) * (f - M + e * math.sin(f)) + (3 - 5 * ((math.cos(i)) ** 2)) \
           *(3 * math.sin(2 * omega + 2 * f) + 3 * e * math.sin(2 * omega + f) + e * math.sin(2 * omega + 3 * f))) \
           - gamma2p / 8 * (e ** 2) * math.cos(i) * (11 + 80 *((math.cos(i)) ** 2) / (1 - 5 * ((math.cos(i)) ** 2)) \
           + 200 * ((math.cos(i)) ** 4) / ((1 - 5 * ((math.cos(i)) ** 2)) ** 2)) * math.sin(2 * omega) \
           - gamma2p / 2 * math.cos(i) * (6 *(f - M + e * math.sin(f)) - 3 * math.sin(2 * omega + 2 * f) \
           - 3 * e * math.sin(2 * omega + f) - e * math.sin(2 * omega + 3 * f))  # (F.11)

    edM = gamma2p / 8 * e * (eta ** 3) * (1 - 11 * ((math.cos(i)) ** 2) - 40 * ((math.cos(i)) ** 4) \
        /(1 - 5 * ((math.cos(i)) ** 2))) * math.sin(2 * omega) - gamma2p / 4 * (eta ** 3) * (2 * (3 * ((math.cos(i)) ** 2) - 1) \
        * ((a_r * eta) ** 2 + a_r + 1) * math.sin(f) + 3 *(1 - ((math.cos(i)) ** 2)) *((-(a_r * eta) ** 2 - a_r + 1) \
        * math.sin(2 * omega + f) + ((a_r * eta) ** 2 + a_r + 1 / 3) * math.sin(2 * omega + 3 * f)))  # (F.12)

    dOmega = -gamma2p / 8 * (e ** 2) * math.cos(i) * (11 + 80 * ((math.cos(i)) ** 2) /(1 - 5 * ((math.cos(i)) ** 2)) \
           + 200 * ((math.cos(i)) ** 4) /((1 - 5 * ((math.cos(i)) ** 2)) ** 2)) * math.sin(2 * omega) \
           - gamma2p / 2 * math.cos(i) * (6 *(f - M + e * math.sin(f)) - 3 * math.sin(2 * omega + 2 * f) \
           - 3 * e * math.sin(2 * omega + f) - e* math.sin(2 * omega + 3 * f))  # (F.13)

    d1 = (e+de)*math.sin(M) + edM*math.cos(M)  # (F.14)
    d2 = (e+de)*math.cos(M) - edM*math.sin(M)  # (F.15)

    Mp = math.atan2(d1, d2)  # (F.16)
    ep = math.sqrt(d1**2+d2**2)  # (F.17)

    d3 = (math.sin(i/2)+math.cos(i/2)*di/2)*math.sin(Omega) + math.sin(i/2)*dOmega*math.cos(Omega)  # (F.18)
    d4 = (math.sin(i/2)+math.cos(i/2)*di/2)*math.cos(Omega) - math.sin(i/2)*dOmega*math.sin(Omega)  # (F.19)

    Omegap = math.atan2(d3, d4)  # (F.20)
    ip = 2*math.asin(np.clip(math.sqrt(d3**2+d4**2), a_min=-1.0, a_max=1.0))  # (F.21)
    omegap = MpopOp - Mp - Omegap  # (F.22)

    Ep = M2E(Mp, ep)
    fp = E2f(Ep, ep)

    oep.a = ap
    oep.e = ep
    oep.i = ip
    oep.Omega = Omegap
    oep.omega = omegap
    oep.f = fp
    return


def clElem2eqElem(elements_cl, elements_eq):
    """
    conversion
    from classical orbital elements (a,e,i,Omega,omega,f)
    to equinoctial orbital elements (a,P1,P2,Q1,Q2,l,L)

    :param elements_cl: classical elements
    :return: elements_eq, equinoctial elements
    """
    elements_eq.a  = elements_cl.a
    elements_eq.P1 = elements_cl.e * math.sin(elements_cl.Omega + elements_cl.omega)
    elements_eq.P2 = elements_cl.e * math.cos(elements_cl.Omega + elements_cl.omega)
    elements_eq.Q1 = math.tan(elements_cl.i / 2) * math.sin(elements_cl.Omega)
    elements_eq.Q2 = math.tan(elements_cl.i / 2) * math.cos(elements_cl.Omega)
    E              = f2E(elements_cl.f, elements_cl.e)
    M              = E2M(E, elements_cl.e)
    elements_eq.l  = elements_cl.Omega + elements_cl.omega + M
    elements_eq.L  = elements_cl.Omega + elements_cl.omega + elements_cl.f
    return


def hillFrame(rc_N, vc_N):
    """
    Compute the Hill frame DCM HN
    :param rc_N: inertial position vector
    :param vc_N: inertial velocity vector
    :return: HN: DCM that maps from the inertial frame N to the Hill (i.e. orbit) frame H
    """
    ir = rc_N/la.norm(rc_N)
    h = np.cross(rc_N, vc_N)
    ih = h / la.norm(h)
    itheta = np.cross(ih, ir)

    return np.array([ir, itheta, ih])


def rv2hill(rc_N, vc_N, rd_N, vd_N):
    """
    Express the deputy position and velocity vector as chief by the chief Hill frame.

    :param rc_N: chief inertial position vector
    :param vc_N: chief inertial velocity vector
    :param rd_N: chief deputy position vector
    :param vd_N: chief deputy velocity vector
    :return: rho_H, rhoPrime_H: Hill frame relative position and velocity vectors
    """

    HN = hillFrame(rc_N, vc_N)
    fDot = la.norm(np.cross(rc_N, vc_N))/(la.norm(rc_N)**2)
    omega_HN_H = np.array([0, 0, fDot])
    rho_H = np.matmul(HN, rd_N - rc_N)
    rhoPrime_H = np.matmul(HN, vd_N - vc_N) - np.cross(omega_HN_H, rho_H)
    return rho_H, rhoPrime_H


def hill2rv(rc_N, vc_N, rho_H, rhoPrime_H):
    """
    Map the deputy position and velocity vector relative to the chief Hill frame to inertial frame.

    :param rc_N: chief inertial position vector
    :param vc_N: chief inertial velocity vector
    :param rho_H: deputy Hill relative position vector
    :param rhoPrime_H: deputy Hill relative velocity vector
    :return:  rd_N, vd_N: Deputy inertial position and velocity vectors
    """

    NH = hillFrame(rc_N, vc_N).transpose()
    fDot = la.norm(np.cross(rc_N, vc_N)) / (la.norm(rc_N) ** 2)
    omega_HN_H = np.array([0, 0, fDot])
    rd_N = rc_N + np.matmul(NH, rho_H)
    vd_N = vc_N + np.matmul(NH, rhoPrime_H + np.cross(omega_HN_H, rho_H))
    return rd_N, vd_N
