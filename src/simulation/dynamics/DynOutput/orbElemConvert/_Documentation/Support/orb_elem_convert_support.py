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

#
# Orb_Elem_Convert Support Script
#
# Purpose:  Illustrate the variation in discrepancy between
#           input and output semimajor axis values when approaching
#           eccentricity of 1.0.
#
# Author:   Gabriel Chapel
# Creation Date:  Aug. 25, 2017
#







import math

import matplotlib.pyplot as plt
# @cond DOXYGEN_IGNORE
import numpy
from Basilisk.utilities import macros as mc

a = 10000000.0
e = 0.99
i = 33.3*mc.D2R
AN = 48.2*mc.D2R
AP = 347.8*mc.D2R
f = 85.3*mc.D2R
mu = 0.3986004415E+15
ePlot = []
aInit = []
aFin = []
for g in range(1000):
    ######### Calculation of elem2rv #########
    if e == 1.0 and a > 0.0:  # rectilinear elliptic orbit case
        Ecc = f  # f is treated as ecc.anomaly
        r = a * (1 - e * math.cos(Ecc))  # orbit radius
        v = math.sqrt(2 * mu / r - mu / a)
        ir = numpy.zeros(3)
        ir[0] = math.cos(AN) * math.cos(AP) - math.sin(AN) * math.sin(AP) * math.cos(i)
        ir[1] = math.sin(AN) * math.cos(AP) + math.cos(AN) * math.sin(AP) * math.cos(i)
        ir[2] = math.sin(AP) * math.sin(i)
        rTruth = numpy.multiply(r, ir)
        if math.sin(Ecc) > 0:
            vTruth = numpy.multiply(-v, ir)
        else:
            vTruth = numpy.multiply(v, ir)
    else:
        if e == 1 and a < 0:  # parabolic case
            rp = -a  # radius at periapses
            p = 2 * rp  # semi-latus rectum
        else:  # elliptic and hyperbolic cases
            p = a * (1 - e * e)  # semi-latus rectum

        r = p / (1 + e * math.cos(f))  # orbit radius
        theta = AP + f  # true latitude angle
        h = math.sqrt(mu * p)  # orbit ang.momentum mag.
        rTruth = numpy.zeros(3)
        rTruth[0] = r * (math.cos(AN) * math.cos(theta) - math.sin(AN) * math.sin(theta) * math.cos(i))
        rTruth[1] = r * (math.sin(AN) * math.cos(theta) + math.cos(AN) * math.sin(theta) * math.cos(i))
        rTruth[2] = r * (math.sin(theta) * math.sin(i))

        vTruth = numpy.zeros(3)
        vTruth[0] = -mu / h * (math.cos(AN) * (math.sin(theta) + e * math.sin(AP)) + math.sin(AN) * (math.cos(
            theta) + e * math.cos(AP)) * math.cos(i))
        vTruth[1] = -mu / h * (math.sin(AN) * (math.sin(theta) + e * math.sin(AP)) - math.cos(AN) * (math.cos(
            theta) + e * math.cos(AP)) * math.cos(i))
        vTruth[2] = -mu / h * (-(math.cos(theta) + e * math.cos(AP)) * math.sin(i))

    ######### Calculate rv2elem #########
    # Calculate the specific angular momentum and its magnitude
    epsConv = 0.000000000001
    hVec = numpy.cross(rTruth, vTruth)
    h = numpy.linalg.norm(hVec)
    p = h * h / mu

    # Calculate the line of nodes
    v3 = numpy.array([0.0, 0.0, 1.0])
    nVec = numpy.cross(v3, hVec)
    n = numpy.linalg.norm(nVec)

    # Orbit eccentricity and energy
    r = numpy.linalg.norm(rTruth)
    v = numpy.linalg.norm(vTruth)
    eVec = numpy.multiply(v * v / mu - 1.0 / r, rTruth)
    v3 = numpy.multiply(numpy.dot(rTruth, vTruth) / mu, vTruth)
    eVec = numpy.subtract(eVec, v3)
    eO = numpy.linalg.norm(eVec)
    rmag = r
    rPeriap = p / (1.0 + eO)

    # compute semi - major axis
    alpha = 2.0 / r - v * v / mu
    if (math.fabs(alpha) > epsConv): # elliptic or hyperbolic case
        aO = 1.0 / alpha
        rApoap = p / (1.0 - eO)
    else:                        # parabolic case
        rp = p / 2.0
        aO = -rp # a is not defined for parabola, so -rp is returned instead
        rApoap = -1.0
    ePlot.append(e)
    aInit.append(a)
    aFin.append(aO)
    e += 0.00001
aDiff = numpy.subtract(aInit, aFin)
plt.figure()
plt.plot(ePlot, aDiff)
plt.xlabel('Eccentricity')
plt.ylabel('Semimajor Axis Discrepancy (km)')
plt.show()
