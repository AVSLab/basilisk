
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
from typing import Tuple



class ClassicElements(object):
    __slots__ = ['a', 'e', 'i', 'Omega', 'omega', 'f', 'rmag', 'alpha', 'rPeriap', 'rApoap']
    def __init__(self):
        self.a = None
        self.e = None
        self.i = None
        self.Omega = None
        self.omega = None
        self.f = None
        self.rmag = None
        self.alpha = None
        self.rPeriap = None
        self.rApoap = None


class EquinoctialElements(object):
    __slots__ = ['a', 'P1', 'P2', 'Q1', 'Q2', 'l', 'L']
    def __init__(self):
        self.a = None
        self.P1 = None
        self.P2 = None
        self.Q1 = None
        self.Q2 = None
        self.l = None
        self.L = None


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


def E2f(Ecc:float, e:float) -> float:
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


def E2M(Ecc:float, e:float) -> float:
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


def f2E(f:float, e:float) -> float:
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


def f2H(f:float, e:float) -> float:
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


def H2f(H:float, e:float) -> float:
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


def H2N(H:float, e:float) -> float:
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


def _kepler_anomaly_difference(anomaly: float, hyperbolic: bool) -> float:
    """Evaluate ``E - sin(E)`` or ``sinh(H) - H`` without small-angle cancellation."""
    if anomaly > 0.5:
        return math.sinh(anomaly) - anomaly if hyperbolic else anomaly - math.sin(anomaly)
    multiplier = anomaly * anomaly * (1.0 if hyperbolic else -1.0)
    term = anomaly * anomaly * anomaly / 6.0
    difference = term
    # Through order 17: the omitted term is below round-off for |anomaly| <= 0.5 rad.
    for index in range(2, 9):
        term *= multiplier / ((2 * index) * (2 * index + 1))
        difference += term
    return difference


def _kepler_result_type(mean, eccentricity):
    """Retain NumPy scalar and zero-dimensional array results without mutating inputs."""
    if isinstance(mean, np.ndarray) and mean.ndim == 0:
        dtype = np.result_type(mean, 0.0)
        return lambda value: np.asarray(value, dtype=dtype)
    if isinstance(mean, (np.generic, np.ndarray)) or isinstance(eccentricity, (np.generic, np.ndarray)):
        return np.result_type(mean, eccentricity, 0.0).type
    return float


def M2E(M: float, e: float) -> float:
    """
    Solve the elliptic Kepler equation using bracketed Newton iteration.

    The returned eccentric anomaly preserves the revolutions in ``M``. Very
    large unwrapped anomalies are limited by the resolution of the input and
    output angles.

    :param M: mean elliptic anomaly (rad)
    :param e: eccentricity (0 <= e < 1)
    :return: eccentric anomaly (rad); NumPy scalar types and zero-dimensional
        mean-array results are preserved
    :raises ValueError: if either input is non-finite or eccentricity is outside its domain
    :raises RuntimeError: if the iteration limit is reached without convergence
    """
    scalar_type = _kepler_result_type(M, e)
    M, e = float(M), float(e)
    if not math.isfinite(M) or not math.isfinite(e) or not 0.0 <= e < 1.0:
        raise ValueError('M2E() requires finite M and e with 0 <= e < 1')
    if e == 0.0 or M == 0.0:
        return scalar_type(M)

    # Let libm reduce the angle: remainder(M, 2*pi) loses the small offset near
    # whole revolutions, which is amplified when eccentricity approaches one.
    reduced_mean = M if abs(M) <= math.pi else math.atan2(math.sin(M), math.cos(M))
    mean = abs(reduced_mean)
    lower = mean
    upper = math.pi  # [rad]
    anomaly = mean
    relative_tolerance = 4.0 * math.ulp(1.0)
    for _ in range(maxIteration):
        residual = (1.0 - e) * anomaly + e * _kepler_anomaly_difference(anomaly, False) - mean
        derivative = (1.0 - e) + 2.0 * e * math.sin(0.5 * anomaly)**2
        correction = residual / derivative
        if abs(correction) <= relative_tolerance * abs(anomaly):
            return scalar_type(M + math.copysign(anomaly - mean, reduced_mean))
        if residual > 0.0:
            upper = anomaly
        else:
            lower = anomaly
        candidate = anomaly - correction
        if not lower < candidate < upper:
            candidate = 0.5 * (lower + upper)
        if candidate == anomaly:
            return scalar_type(M + math.copysign(anomaly - mean, reduced_mean))
        anomaly = candidate
    raise RuntimeError(f'M2E() failed to converge for M={M}, e={e}')


def N2H(N: float, e: float) -> float:
    """
    Solve the hyperbolic Kepler equation using bracketed Newton iteration.

    The equation is scaled to avoid overflow for large finite mean anomalies
    and evaluated without subtractive cancellation near the parabolic limit.

    :param N: mean hyperbolic anomaly (rad)
    :param e: eccentricity (e > 1)
    :return: hyperbolic anomaly (rad); NumPy scalar types and zero-dimensional
        mean-array results are preserved
    :raises ValueError: if either input is non-finite or eccentricity is outside its domain
    :raises RuntimeError: if the iteration limit is reached without convergence
    """
    scalar_type = _kepler_result_type(N, e)
    N, e = float(N), float(e)
    if not math.isfinite(N) or not math.isfinite(e) or e <= 1.0:
        raise ValueError('N2H() requires finite N and e with e > 1')
    if N == 0.0:
        return scalar_type(N)

    mean = abs(N) / e
    lower = math.asinh(mean)
    upper = lower + 2.0  # [rad]; brackets the positive root for every e > 1
    anomaly = lower
    relative_tolerance = 4.0 * math.ulp(1.0)
    for _ in range(maxIteration):
        if anomaly <= 1.0:
            # Divide by e and retain the small linear term when e is close to one.
            linear = (e - 1.0) / e
            residual = linear * anomaly + _kepler_anomaly_difference(anomaly, True) - mean
            derivative = linear + 2.0 * math.sinh(0.5 * anomaly)**2
        else:
            # Divide both f and f' by e*exp(H), without forming either large factor.
            inverse_exp = math.exp(-anomaly)
            residual = -0.5 * math.expm1(-2.0 * anomaly) - (anomaly / e + mean) * inverse_exp
            derivative = 0.5 * (1.0 + inverse_exp * inverse_exp) - inverse_exp / e
        correction = residual / derivative
        if abs(correction) <= relative_tolerance * abs(anomaly):
            result = math.copysign(anomaly, N)
            return scalar_type(result)
        if residual > 0.0:
            upper = anomaly
        else:
            lower = anomaly
        candidate = anomaly - correction
        if not lower < candidate < upper:
            candidate = 0.5 * (lower + upper)
        if candidate == anomaly:
            result = math.copysign(anomaly, N)
            return scalar_type(result)
        anomaly = candidate
    raise RuntimeError(f'N2H() failed to converge for N={N}, e={e}')


def elem2rv_parab(mu: float, elements: ClassicElements) -> Tuple[np.ndarray, np.ndarray]:
    """Convert elements using the legacy Python parabolic encoding.

    This adapter accepts ``e = 1, a < 0`` for a planar parabola with periapsis
    radius ``-a``, independently of auxiliary fields such as ``rPeriap``.
    Use :func:`elem2rv` for rectilinear hyperbolas. Other conventions follow it, except
    for rectilinear elliptic motion. This adapter retains inward velocity for
    positive sin(E) on rectilinear ellipses. This convention remains supported.
    The input object is not modified.

    :param mu: positive gravitational parameter (L^3/s^2)
    :param elements: orbital elements, with distances in L and angles in radians
    :return: inertial position (L) and velocity (L/s)
    :raises ValueError: if the elements do not describe a finite physical state
    """
    if elements.e == 1.0 and elements.a != 0.0:
        # A shallow copy of a SWIG wrapper still shares its native storage.
        source = elements
        elements = ClassicElements()
        for field in ("a", "e", "i", "Omega", "omega", "f"):
            setattr(elements, field, getattr(source, field))
        if elements.a < 0.0:
            elements.rPeriap = -elements.a
            elements.a = 0.0  # [L]; canonical planar parabolic encoding
        else:
            elements.f = -elements.f
    return elem2rv(mu, elements)


def elem2rv(mu: float, elements: ClassicElements) -> Tuple[np.ndarray, np.ndarray]:
    """Convert classical elements to a finite inertial Cartesian state.

    Inclination must be in [0, pi]. Distances use the same unit L as ``mu``. Ellipses have ``a > 0, e < 1``;
    hyperbolas have ``a < 0, e > 1``. No eccentricity tolerance selects a conic.
    The C and Python core conversions share these degenerate conventions:

    * Planar parabola: ``e = 1, a = 0, rPeriap > 0``; ``f`` is true anomaly.
    * Rectilinear ellipse: ``e = 1, a > 0``; ``f`` is eccentric anomaly E,
      with outward motion for ``sin(E) > 0``.
    * Rectilinear hyperbola: ``e = 1, a < 0``; ``f`` is signed hyperbolic
      anomaly H, with outward motion for ``H > 0``.
    * Rectilinear parabola: ``e = 1, a = rPeriap = 0``; ``rmag > 0`` supplies
      radius, and ``f = 0`` selects outward motion or ``f = pi`` inward motion.

    For rectilinear states, ``i, Omega, omega`` orient the outward radial ray
    along the perifocal x axis; the plane about that ray is arbitrary.
    Collisions and open-conic anomalies at or beyond infinity are rejected.
    The extra fields are read only where required by these conventions.

    :param mu: positive gravitational parameter (L^3/s^2)
    :param elements: orbital elements, with distances in L and angles in radians
    :return: inertial position (L) and velocity (L/s)
    :raises ValueError: if the elements do not describe a finite physical state
    """
    a, e, inc, node, periapsis, anomaly = (elements.a, elements.e, elements.i,
                                         elements.Omega, elements.omega, elements.f)
    if not all(value is not None and math.isfinite(value)
               for value in (mu, a, e, inc, node, periapsis, anomaly)) or mu <= 0.0 or e < 0.0 or not 0.0 <= inc <= math.pi:
        raise ValueError("elem2rv() requires finite elements, mu > 0, e >= 0 and 0 <= i <= pi")
    p_hat = np.array([math.cos(node) * math.cos(periapsis) - math.sin(node) * math.sin(periapsis) * math.cos(inc),
                      math.sin(node) * math.cos(periapsis) + math.cos(node) * math.sin(periapsis) * math.cos(inc),
                      math.sin(periapsis) * math.sin(inc)])
    q_hat = np.array([-math.cos(node) * math.sin(periapsis) - math.sin(node) * math.cos(periapsis) * math.cos(inc),
                      -math.sin(node) * math.sin(periapsis) + math.cos(node) * math.cos(periapsis) * math.cos(inc),
                      math.cos(periapsis) * math.sin(inc)])
    radial = e == 1.0 and (a != 0.0 or elements.rPeriap == 0.0)
    if radial:
        if a > 0.0:
            # Let libm reduce the angle; a rounded 2*pi divisor loses phase.
            half = 0.5 * anomaly
            factor = math.sin(half)
            radius = 2.0 * a * factor**2
            if radius <= 0.0:
                raise ValueError("elem2rv() cannot represent a rectilinear collision")
            radial_speed = math.sqrt(mu / a) * math.cos(half) / factor
        elif a < 0.0:
            try:
                factor = math.sinh(0.5 * anomaly)
                radius = -2.0 * a * factor * factor
            except OverflowError as error:
                raise ValueError("elem2rv() rectilinear radius overflow") from error
            if radius <= 0.0:
                raise ValueError("elem2rv() cannot represent a rectilinear collision")
            radial_speed = math.copysign(math.sqrt(mu / -a) / math.tanh(0.5 * abs(anomaly)), anomaly)
        else:
            radius = elements.rmag
            if radius is None or not math.isfinite(radius) or radius <= 0.0 or anomaly not in (0.0, math.pi):
                raise ValueError("elem2rv() radial parabola requires rmag > 0 and f = 0 or pi")
            radial_speed = math.sqrt(2.0 * (mu / radius)) * (1.0 if anomaly == 0.0 else -1.0)
        r_vec, v_vec = radius * p_hat, radial_speed * p_hat
    else:
        if e == 1.0 and a == 0.0:
            if elements.rPeriap is None or not math.isfinite(elements.rPeriap) or elements.rPeriap <= 0.0:
                raise ValueError("elem2rv() planar parabola requires rPeriap > 0")
            parameter = 2.0 * elements.rPeriap
        else:
            parameter = a * (1.0 - e) * (1.0 + e)
        cosine = math.cos(anomaly)
        sine = math.sin(anomaly)
        principal_anomaly = math.atan2(sine, cosine)
        if parameter <= 0.0 or not math.isfinite(parameter) or (e >= 1.0 and abs(principal_anomaly) >= math.acos(-1.0 / e)):
            raise ValueError("elem2rv() invalid conic or anomaly at/beyond infinity")
        # Half-angle expressions retain 1-e and e-1 near apoapsis/infinity.
        half_cosine_squared = math.cos(0.5 * anomaly)**2
        denominator = (1.0 - e) + 2.0 * e * half_cosine_squared if cosine < 0.0 else 1.0 + e * cosine
        if denominator <= 0.0:
            raise ValueError("elem2rv() anomaly at/beyond infinity")
        radius = parameter / denominator
        transverse = (e - 1.0) + 2.0 * half_cosine_squared if cosine < 0.0 else e + cosine
        r_vec = radius * (cosine * p_hat + sine * q_hat)
        v_vec = math.sqrt(mu / parameter) * (-sine * p_hat + transverse * q_hat)
    if not np.all(np.isfinite(r_vec)) or not np.all(np.isfinite(v_vec)):
        raise ValueError("elem2rv() state is outside the finite numerical range")
    return r_vec, v_vec


def rv2elem_parab(mu: float, rVec: np.ndarray, vVec: np.ndarray) -> ClassicElements:
    """Convert a Cartesian state using the legacy Python parabolic encoding.

    The computation and singular-angle conventions are those of :func:`rv2elem`.
    Planar parabolas return ``a = rApoap = -rPeriap``. Rectilinear states have
    ``rPeriap = 0``; use :func:`elem2rv` to reconstruct radial hyperbolas because
    :func:`elem2rv_parab` retains its negative-a planar encoding. Node, periapsis
    and nonradial true anomaly retain signed angles in [-pi, pi]; circular orbits
    place periapsis at the current position.
    Radial elliptic anomaly retains the supported inward-positive convention.

    :param mu: positive gravitational parameter (L^3/s^2)
    :param rVec: nonzero inertial position vector (L)
    :param vVec: inertial velocity vector (L/s)
    :return: orbital elements, with distances in L and angles in radians
    :raises ValueError: if the state is invalid or its elements are unrepresentable
    """
    elements = rv2elem(mu, rVec, vVec, canonical=True)
    if elements.a == 0.0 and elements.rPeriap > 0.0:
        elements.a = elements.rApoap = -elements.rPeriap
    if elements.rPeriap > 0.0 and elements.f > math.pi:
        elements.f -= 2.0 * math.pi
    for field in ("Omega", "omega"):
        angle = getattr(elements, field)
        setattr(elements, field, math.atan2(math.sin(angle), math.cos(angle)))
    if elements.rPeriap == 0.0 and elements.a > 0.0:
        elements.f = (-elements.f) % (2.0 * math.pi)
    elif elements.e <= 1e-12:
        # The legacy adapter places the perifocal x axis at the current position.
        elements.omega = math.atan2(math.sin(elements.omega + elements.f),
                                   math.cos(elements.omega + elements.f))
        elements.f = 0.0  # [rad]
    return elements


def rv2elem(mu: float, rVec: np.ndarray, vVec: np.ndarray, *, canonical: bool = False) -> ClassicElements:
    """Convert a Cartesian state to classical orbital elements.

    Inputs must be finite and use consistent distance units. Inclination ``i``
    is in [0, pi]; ``Omega`` and ``omega`` are in [0, 2*pi). Nonradial true
    anomaly ``f`` is in [0, 2*pi) for ellipses/parabolas and [-pi, pi] for
    hyperbolas. With ``canonical=True``, degenerate conics use the conventions
    of :func:`elem2rv`. The default retains the supported Python parabolic
    encoding ``a = -rPeriap, rApoap = -1``.

    A dimensionless tolerance of 1e-12 selects singular-angle conventions:

    * For ``e <= tolerance``, ``omega = 0`` and ``f`` is the argument of latitude.
    * For ``sin(i) <= tolerance``, ``Omega = 0`` and ``i`` is set to 0 or pi.
      ``omega`` measures periapsis from the inertial x axis in the direction of
      orbital motion. Equatorial retrograde longitudes therefore have the
      opposite sign to inertial azimuths.
    * If both conditions hold, ``f`` measures position from that axis in the
      direction of motion.

    Replacing unresolved directions introduces relative state errors on the
    order of this tolerance. Energy is classified as parabolic only within
    8 machine epsilons of the local kinetic/potential energy scale. Radial
    motion is detected within 8 machine epsilons of zero transverse velocity;
    the plane is chosen deterministically about the outward ray.
    Near zero energy, relative accuracy of ``a`` and state reconstruction from
    rounded ``a, e`` degrades as machine epsilon divided by ``abs(1-e)``.
    Nonradial finite-energy states whose eccentricity rounds to one or the
    wrong side of one cannot be encoded consistently and are rejected.

    :param mu: positive gravitational parameter (L^3/s^2)
    :param rVec: nonzero inertial position vector (L)
    :param vVec: inertial velocity vector (L/s)
    :param canonical: if True, use the C-compatible parabolic encoding; otherwise
        retain the original Python ``a = -rPeriap, rApoap = -1`` result
    :return: orbital elements, with distances in L and angles in radians; a, e,
        rmag, alpha, rPeriap and rApoap retain NumPy float64 scalar arithmetic
    :raises ValueError: if the state is invalid or its elements are unrepresentable
    """
    rVec = np.asarray(rVec, dtype=float)
    vVec = np.asarray(vVec, dtype=float)
    if (not math.isfinite(mu) or mu <= 0.0 or rVec.shape != (3,) or vVec.shape != (3,)
            or not np.all(np.isfinite(rVec)) or not np.all(np.isfinite(vVec))):
        raise ValueError("rv2elem() requires finite three-vectors and mu > 0")
    radius = math.hypot(*rVec)
    if radius <= 0.0 or not math.isfinite(radius):
        raise ValueError("rv2elem() requires a finite nonzero radius")
    speed_scale = math.sqrt(mu / radius)
    if speed_scale == 0.0 or not math.isfinite(speed_scale):
        raise ValueError("rv2elem() state is outside the finite numerical range")
    r_hat = rVec / radius
    scaled_v = vVec / speed_scale
    speed_squared = float(np.dot(scaled_v, scaled_v))
    if not math.isfinite(speed_squared):
        raise ValueError("rv2elem() state is outside the finite numerical range")
    scaled_energy = 2.0 - speed_squared
    roundoff = 8.0 * np.finfo(float).eps
    singularity_tolerance = 1e-12  # [-]
    if abs(scaled_energy) <= roundoff * max(2.0, speed_squared):
        scaled_energy = 0.0  # [-]; unresolved energy at the parabolic limit
    scaled_h = np.cross(r_hat, scaled_v)
    momentum = math.hypot(*scaled_h)
    radial = momentum <= roundoff * math.sqrt(speed_squared)
    elements = ClassicElements()
    elements.rmag = radius
    elements.alpha = scaled_energy / radius
    elements.a = radius / scaled_energy if scaled_energy != 0.0 else 0.0  # [L]
    parameter = radius * momentum * momentum
    if radial:
        elements.e = 1.0
        elements.rPeriap = 0.0  # [L]
        # The least-aligned inertial axis avoids a degenerate auxiliary plane.
        axis = np.eye(3)[np.argmin(np.abs(r_hat))]
        h_hat = np.cross(r_hat, axis)
        h_hat /= math.hypot(*h_hat)
        e_hat = r_hat
    else:
        h_hat = scaled_h / momentum
        e_vec = np.cross(scaled_v, scaled_h) - r_hat
        elements.e = math.hypot(*e_vec)
        if scaled_energy == 0.0:
            elements.e = 1.0
        elif (scaled_energy > 0.0 and elements.e >= 1.0) or (scaled_energy < 0.0 and elements.e <= 1.0):
            raise ValueError("rv2elem() eccentricity cannot resolve the finite-energy conic")
        elements.rPeriap = parameter / (1.0 + elements.e)
        e_hat = e_vec / math.hypot(*e_vec) if elements.e > singularity_tolerance else None
    elements.rApoap = elements.a * (1.0 + elements.e) if scaled_energy != 0.0 else 0.0  # [L]
    n_vec = np.array([-h_hat[1], h_hat[0], 0.0])
    node_magnitude = math.hypot(n_vec[0], n_vec[1])
    if node_magnitude <= singularity_tolerance:
        n_hat = np.array([1.0, 0.0, 0.0])
        elements.i = 0.0 if h_hat[2] >= 0.0 else math.pi  # [rad]
    else:
        n_hat = n_vec / node_magnitude
        elements.i = math.atan2(node_magnitude, h_hat[2])
    if e_hat is None:
        e_hat = n_hat
    elements.Omega = math.atan2(n_hat[1], n_hat[0])
    elements.omega = math.atan2(np.dot(h_hat, np.cross(n_hat, e_hat)), np.dot(n_hat, e_hat))
    elements.f = math.atan2(np.dot(h_hat, np.cross(e_hat, r_hat)), np.dot(e_hat, r_hat))
    for field in ("Omega", "omega", "f"):
        angle = getattr(elements, field) % (2.0 * math.pi)
        setattr(elements, field, 0.0 if angle >= 2.0 * math.pi else angle)
    if radial:
        radial_speed = float(np.dot(r_hat, scaled_v))
        if scaled_energy > 0.0:
            elements.f = math.atan2(radial_speed * math.sqrt(scaled_energy), 1.0 - scaled_energy)
            if elements.f < 0.0:
                elements.f += 2.0 * math.pi
        elif scaled_energy < 0.0:
            elements.f = math.copysign(2.0 * math.asinh(math.sqrt(-0.5 * scaled_energy)), radial_speed)
        else:
            elements.f = 0.0 if radial_speed >= 0.0 else math.pi  # [rad]; radial direction flag
    elif elements.e > 1.0 and elements.f > math.pi:
        elements.f -= 2.0 * math.pi
    if not all(math.isfinite(getattr(elements, field)) for field in ClassicElements.__slots__):
        raise ValueError("rv2elem() elements are outside the finite numerical range")
    if not canonical and elements.a == 0.0 and elements.rPeriap > 0.0:
        elements.a = -elements.rPeriap
        elements.rApoap = -1.0  # [L]; legacy sentinel, not an apoapsis radius
    # These fields historically carry NumPy scalar arithmetic (including reflected
    # list subtraction). Keep the public types independent of internal math helpers.
    for field in ("a", "e", "rmag", "alpha", "rPeriap", "rApoap"):
        setattr(elements, field, np.float64(getattr(elements, field)))
    return elements


def atmosphericDensity(alt: float) -> float:
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


DEBYE_ALTITUDES = [200.0, 250.0, 300.0, 350.0, 400., 450., 500., 550., 600., 650., 700., 750., 800., 850.,
         900., 950., 1000., 1050., 1100., 1150., 1200., 1250., 1300., 1350., 1400., 1450.,
         1500., 1550., 1600., 1650., 1700., 1750., 1800., 1850., 1900., 1950., 2000.]
DEBYE_LENGTHS = [5.64E-03, 3.92E-03, 3.24E-03, 3.59E-03, 4.04E-03, 4.28E-03, 4.54E-03, 5.30E-03, 6.55E-03,
         7.30E-03, 8.31E-03, 8.38E-03, 8.45E-03, 9.84E-03, 1.22E-02, 1.37E-02, 1.59E-02, 1.75E-02,
         1.95E-02, 2.09E-02, 2.25E-02, 2.25E-02, 2.25E-02, 2.47E-02, 2.76E-02, 2.76E-02, 2.76E-02,
         2.76E-02, 2.76E-02, 2.76E-02, 2.76E-02, 3.21E-02, 3.96E-02, 3.96E-02, 3.96E-02, 3.96E-02, 3.96E-02]

def debyeLength(alt: float) -> float:
    """
    This program computes the debyeLength length for a given
    altitude and is valid for altitudes ranging
    from 200 km to GEO (35000km).  However, all values above
    1000 km are HIGHLY speculative at this point.

    :param alt: altitude in km
    :return: debye length given in m
    """

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
        if DEBYE_ALTITUDES[i + 1] > alt:
            break
    a = (alt - DEBYE_ALTITUDES[i]) / (DEBYE_ALTITUDES[i + 1] - DEBYE_ALTITUDES[i])
    debyedist = DEBYE_LENGTHS[i] + a * (DEBYE_LENGTHS[i + 1] - DEBYE_LENGTHS[i])

    return debyedist


def atmosphericDrag(Cd: float, A: float, m: float, rvec: np.ndarray, vvec: np.ndarray) -> np.ndarray:
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


def jPerturb(rvec: np.ndarray, num: int, planet:str) -> np.ndarray:
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


def solarRad(A: float, m: float, sunvec: np.ndarray) -> np.ndarray:
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
    from Basilisk.utilities import astroFunctions

    # Solar Radiation Flux #
    flux = astroFunctions.solarFluxEarth

    # Speed of light #
    c = 299792458.

    # Radiation pressure coefficient #
    Cr = 1.3

    # Magnitude of position vector #
    sundist = la.norm(sunvec)

    # Computing the acceleration vector #
    arvec = ((-Cr * A * flux) / (m * c * math.pow(sundist, 3)) / 1000.) * sunvec

    return arvec


def v3Normalize(v: np.ndarray) -> np.ndarray:
    result = np.zeros(3)
    norm = la.norm(v)
    if norm > DB0_EPS:
        result = (1. / norm) * v
    return result


def clMeanOscMap(req: float, J2: float, oe: ClassicElements, oep:ClassicElements, sign: int):
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


def clElem2eqElem(elements_cl: ClassicElements, elements_eq: EquinoctialElements):
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


def hillFrame(rc_N: np.ndarray, vc_N: np.ndarray) -> np.ndarray:
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


def rv2hill(rc_N: np.ndarray, vc_N: np.ndarray, rd_N: np.ndarray, vd_N: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
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


def hill2rv(rc_N: np.ndarray, vc_N: np.ndarray, rho_H: np.ndarray, rhoPrime_H: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
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
