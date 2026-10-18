# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


"""Internal formulas preserving the original APIs' behavior outside checked domains.

These formulas preserve old exceptions and sentinel outputs only when the checked
implementation rejects an input. Public callers use :mod:`orbitalMotion`.
"""

from __future__ import annotations

import math
from typing import Tuple
import numpy as np
from numpy import linalg as la
from Basilisk.utilities.orbitalMotion import ClassicElements, EquinoctialElements, f2E, E2M, M2E, E2f

def clMeanOscMap(req: float, J2: float, oe: ClassicElements, oep:ClassicElements, sign: int):
    """Retain the historical formula for clMeanOscMap outside its checked domain.

    Distances use a consistent unit L, velocities L/s, and angles radians.
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
    """Retain the historical formula for clElem2eqElem outside its checked domain.

    Distances use a consistent unit L, velocities L/s, and angles radians.
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
    """Retain the historical formula for hillFrame outside its checked domain.

    Distances use a consistent unit L, velocities L/s, and angles radians.
    """
    ir = rc_N/la.norm(rc_N)
    h = np.cross(rc_N, vc_N)
    ih = h / la.norm(h)
    itheta = np.cross(ih, ir)

    return np.array([ir, itheta, ih])


def rv2hill(rc_N: np.ndarray, vc_N: np.ndarray, rd_N: np.ndarray, vd_N: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Retain the historical formula for rv2hill outside its checked domain.

    Distances use a consistent unit L, velocities L/s, and angles radians.
    """

    HN = hillFrame(rc_N, vc_N)
    fDot = la.norm(np.cross(rc_N, vc_N))/(la.norm(rc_N)**2)
    omega_HN_H = np.array([0, 0, fDot])
    rho_H = np.matmul(HN, rd_N - rc_N)
    rhoPrime_H = np.matmul(HN, vd_N - vc_N) - np.cross(omega_HN_H, rho_H)
    return rho_H, rhoPrime_H


def hill2rv(rc_N: np.ndarray, vc_N: np.ndarray, rho_H: np.ndarray, rhoPrime_H: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Retain the historical formula for hill2rv outside its checked domain.

    Distances use a consistent unit L, velocities L/s, and angles radians.
    """

    NH = hillFrame(rc_N, vc_N).transpose()
    fDot = la.norm(np.cross(rc_N, vc_N)) / (la.norm(rc_N) ** 2)
    omega_HN_H = np.array([0, 0, fDot])
    rd_N = rc_N + np.matmul(NH, rho_H)
    vd_N = vc_N + np.matmul(NH, rhoPrime_H + np.cross(omega_HN_H, rho_H))
    return rd_N, vd_N
