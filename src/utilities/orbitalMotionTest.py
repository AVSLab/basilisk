''' '''
'''
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

'''


from Basilisk.utilities import orbitalMotion
import numpy as np
e_count = 0
a_tol = 1e-14 # array tolerance
s_tol = 1e-6  #scale tolerance

def arrayEqualCheck(v1, v2, methodName):
    if len(v1) == len(v2):
        if np.allclose(v1, v2, atol=a_tol):
            return 0
        print methodName + ' failed'
        return 1
    else:
        print 'Mismatching lengths.'

def scaleEqualCheck(s1, s2, methodName):
    if np.allclose(s1, s2, atol = s_tol):
        return 0
    print methodName + ' failed'
    return 1

# Function E2f
f1 = orbitalMotion.E2f(180, 0.5)
E2FCValue = 3.70494  #input value (180,0.5)
e_count += scaleEqualCheck(f1, E2FCValue, 'E2f')

# Function E2M
M1 = orbitalMotion.E2M(180, 0.5)
E2MCValue = 180.401
e_count += scaleEqualCheck(M1, E2MCValue, 'E2M')

# Function f2E
Ecc1 = orbitalMotion.f2E(180, 0.5)
f2ECValue = 4.57142
e_count += scaleEqualCheck(Ecc1, f2ECValue, 'f2E')

# Function f2H
H1 = orbitalMotion.f2H(180, 1.5)
f2HCValue = -2.86601
e_count += scaleEqualCheck(H1, f2HCValue, 'f2H')

# Function H2f
H2f1 = orbitalMotion.H2f(180, 1.5)
H2fCValue = 2.30052
e_count += scaleEqualCheck(H2f1, H2fCValue, 'H2f')

# Function H2N
H2N1 = orbitalMotion.H2N(180, 1.5)
H2NCValue = 1.11704E78
e_count += scaleEqualCheck(H2N1, H2NCValue, 'H2N')

# Function M2E
M2E1 = orbitalMotion.M2E(180, 0.7)
M2ECValue = 179.629
e_count += scaleEqualCheck(M2E1, M2ECValue, 'M2E')

# Function N2H
N2H1 = orbitalMotion.N2H(180, 1.5)
N2HCValue = 5.51081
e_count += scaleEqualCheck(N2H1, N2HCValue, 'N2H')

# Function elem2rv_parab
mu = 1000
elements = orbitalMotion.ClassicElements()
elements.a = 1000
elements.e = 1.0
elements.Omega = 0.15
elements.omega = 0.5
elements.f = 0.2
elements.i = 0.2
rVec, vVec = orbitalMotion.elem2rv_parab(mu, elements)
rVecCValue = np.array([15.8971, 11.8751, 1.8986])
vVecCValue = np.array([-7.94852, -5.9375, -0.949294])
e_count += arrayEqualCheck(rVec, rVecCValue, 'elem2rv_parab')
e_count += arrayEqualCheck(vVec, vVecCValue, 'elem2rv_parab')

# Function elem2rv
mu2 = 1000
elements2 = orbitalMotion.ClassicElements()
elements2.a = -1000
elements2.e = 2
elements2.Omega = 0.15
elements2.omega = 0.5
elements2.f = 0.2
elements2.i = 0.2
rVec2, vVec2 = orbitalMotion.elem2rv(mu2, elements2)
rVecCValue2 = np.array([670.817, 748.53, 129.71])
vVecCValue2 = np.array([-1.12823, 1.2716, 0.289049])
e_count += arrayEqualCheck(rVec2, rVecCValue2, 'elem2rv')
e_count += arrayEqualCheck(vVec2, vVecCValue2, 'elem2rv')


# Function rv2elem_parab
mu3 = 1000
rVec3 = np.array([15.8971, 11.8751, 1.8986])
vVec3 = np.array([-7.94852, -5.9375, -0.949294])
elements3 = orbitalMotion.rv2elem_parab(mu3, rVec3, vVec3)
rv2elem_parabCValue = np.array([999.927, 1, 0.102545, -3.14159, -0.552287, -1.94593])
rv2elem_parab_Value = np.array([elements3.a, elements3.e, elements3.i, elements3.f, elements3.Omega, elements3.omega])
e_count += arrayEqualCheck(rv2elem_parab_Value, rv2elem_parabCValue, 'rv2elem_parab')

# Function rv2elem
mu4 = 1000
rVec4 = np.array([15.8971, 11.8751, 1.8986])
vVec4 = np.array([-7.94852, -5.9375, -0.949294])
elements4 = orbitalMotion.rv2elem_parab(mu4, rVec4, vVec4)
rv2elemCValue = np.array([999.927, 1, 0.102545, -3.14159, -0.552287, -1.94593])
rv2elemValue = np.array([elements4.a, elements4.e, elements4.i, elements4.f, elements4.Omega, elements4.omega])
e_count += arrayEqualCheck(rv2elemValue, rv2elemCValue, 'rv2elem')

# Function atmosphericDensity
density1 = orbitalMotion.atmosphericDensity(500)
density2 = orbitalMotion.atmosphericDensity(2000)
density1CValue = 4.36496E-13
density2CValue = 2.48886E-15
e_count += scaleEqualCheck(density1, density1CValue, 'atmosphericDensity')
e_count += scaleEqualCheck(density2, density2CValue, 'atmosphericDensity')


# Function debyeLength
debyedist1 = orbitalMotion.debyeLength(500)
debyedist2 = orbitalMotion.debyeLength(2000)
debyedist1CValue = 0.00454
debyedist2CValue = 0.0396
e_count += scaleEqualCheck(debyedist1, debyedist1CValue, 'debyeLength')


# Function atmosphericDrag
Cd = 0.5
A = 50
m = 5000
rVec3 = np.array([10000,20000,30000])
vVec3 = np.array([5,10,15])
advec = orbitalMotion.atmosphericDrag(Cd,  A, m, rVec3, vVec3)
advecCvalue = np.array([-5.39826E-15, -1.07965E-14, -1.61948E-14])
e_count += arrayEqualCheck(advec, advecCvalue, 'atmosphericDrag')


# Function jPerturb
rvec = np.array([10, 20, 30])
num = 6
ajtot1 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_MERCURY') # CASE MERCURY
adjtotCVaule1 = np.array([3.56356, 7.12713, 1.03458])
e_count += arrayEqualCheck(ajtot1, adjtotCVaule1, 'jPerturb')

ajtot2 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_VENUS') # CASE VENUS
adjtotCVaule2 = np.array([24.022, 48.0439, 6.97412])
e_count += arrayEqualCheck(ajtot2, adjtotCVaule2, 'jPerturb')

ajtot3 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_EARTH') # CASE EARTH
adjtotCVaule3 = np.array([-4.09853E9, -8.19707E9, -6.85662E9])
e_count += arrayEqualCheck(ajtot3, adjtotCVaule3, 'jPerturb')

ajtot4 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_MOON') # CASE MOON
adjtotCVaule4 = np.array([1.35863, 2.71727, 0.394442])
e_count += arrayEqualCheck(ajtot4, adjtotCVaule4, 'jPerturb')

ajtot5 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_MARS') # CASE MARS
adjtotCVaule5 = np.array([438.607, 877.214, 127.338])
e_count += arrayEqualCheck(ajtot5, adjtotCVaule5, 'jPerturb')

ajtot6 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_JUPITER') # CASE JUPITER
adjtotCVaule6 = np.array([4.32235E9, 8.6447E9, 1.25488E9])
e_count += arrayEqualCheck(ajtot6, adjtotCVaule6, 'jPerturb')

ajtot7 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_URANUS') # CASE URANUS
adjtotCVaule7 = np.array([5.73199E6, 1.1464E7, 1.66413E6])
e_count += arrayEqualCheck(ajtot7, adjtotCVaule7, 'jPerturb')

ajtot8 = orbitalMotion.jPerturb(rvec, num, 'CELESTIAL_NEPTUNE') # CASE NEPTUNE
adjtotCVaule8 = np.array([6.46744E6, 1.29349E7, 1.87764E6])
e_count += arrayEqualCheck(ajtot8, adjtotCVaule8, 'jPerturb')

# Function solarRad
A = 3
m = 500
sunvec = np.array([1.2, 1.2, 1.2])
arvec = orbitalMotion.solarRad(A, m, sunvec)
arvecCValue = np.array([-4.77259E-12, -4.77259E-12, -4.77259E-12])
e_count += arrayEqualCheck(arvec, arvecCValue, 'solarRad')

if e_count > 0:
    print str(e_count) + " functions failed"
