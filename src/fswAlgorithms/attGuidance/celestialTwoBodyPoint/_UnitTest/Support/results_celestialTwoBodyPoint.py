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
import numpy as np
from numpy import linalg as la
from numpy import sin, cos
np.set_printoptions(precision=12)
import sys, os, inspect





from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import astroFunctions as af

def normalize(v):
    norm=np.linalg.norm(v)
    if norm==0:
       return v
    return v/norm

def checkIndex(i):
    if i > 2:
        return 0
    else:
        return i


def computeCelestialTwoBodyPoint(R_P1, v_P1, a_P1, R_P2, v_P2, a_P2):

    # Beforehand computations
    R_n = np.cross(R_P1, R_P2)
    v_n = np.cross(v_P1, R_P2) + np.cross(R_P1, v_P2)
    a_n = np.cross(a_P1, R_P2) + np.cross(R_P1, a_P2) + 2 * np.cross(v_P1, v_P2)

    # Reference Frame generation
    r1_hat = normalize(R_P1)
    r3_hat = normalize(R_n)
    r2_hat = np.cross(r3_hat, r1_hat)
    RN = np.array([r1_hat, r2_hat, r3_hat])
    sigma_RN = rbk.C2MRP(RN)

    # Reference base-vectors first time-derivative
    I_33 = np.identity(3)
    C1 = I_33 - np.outer(r1_hat, r1_hat)
    dr1_hat = 1.0 / la.norm(R_P1) * np.dot(C1, v_P1)
    C3 = I_33 - np.outer(r3_hat, r3_hat)
    dr3_hat = 1.0 / la.norm(R_n) * np.dot(C3, v_n)
    dr2_hat = np.cross(dr3_hat, r1_hat) + np.cross(r3_hat, dr1_hat)

    # Angular Velocity computation
    omega_RN_R = np.array([
        np.dot(r3_hat, dr2_hat),
        np.dot(r1_hat, dr3_hat),
        np.dot(r2_hat, dr1_hat)
    ])
    omega_RN_N = np.dot(RN.T, omega_RN_R)

    # Reference base-vectors second time-derivative
    temp33_1 = 2 * np.outer(dr1_hat, r1_hat) + np.outer(r1_hat, dr1_hat)
    ddr1_hat = 1.0 / la.norm(R_P1) * (np.dot(C1, a_P1) - np.dot(temp33_1, v_P1))
    temp33_3 = 2 * np.outer(dr3_hat, r3_hat) + np.outer(r3_hat, dr3_hat)
    ddr3_hat = 1.0 / la.norm(R_n) * (np.dot(C3, a_n) - np.dot(temp33_3, v_n))
    ddr2_hat = np.cross(ddr3_hat, r1_hat) + np.cross(ddr1_hat, r3_hat) + 2 * np.cross(dr3_hat, dr1_hat)

    # Angular Acceleration computation
    domega_RN_R = np.array([
        np.dot(dr3_hat, dr2_hat) + np.dot(r3_hat, ddr2_hat) - np.dot(omega_RN_R, dr1_hat),
        np.dot(dr1_hat, dr3_hat) + np.dot(r1_hat, ddr3_hat) - np.dot(omega_RN_R, dr2_hat),
        np.dot(dr2_hat, dr1_hat) + np.dot(r2_hat, ddr1_hat) - np.dot(omega_RN_R, dr3_hat)

    ])
    domega_RN_N = np.dot(RN.T, domega_RN_R)

    # Print output
    print 'sigma_RN = ', sigma_RN
    print 'omega_RN_N = ', omega_RN_N
    print 'domega_RN_N = ', domega_RN_N

    return

# MAIN

# Initial Conditions (IC)
r_BN_N = np.array([500., 500., 1000.])
v_BN_N = np.array([0., 20., 0.])
celPositionVec = np.array([-500.,-500., 0.])
celVelocityVec = np.array([0., 0., 0.])
secPositionVec = np.array([500., 500., 500.])
secVelocityVec = np.array([0., 0., 0.])
singularityThresh = np.pi / 4.0

a = af.E_radius * 2.8
e = 0.0
i = 0.0
Omega = 0.0
omega = 0.0
f = 60 * af.D2R
(r, v) = af.OE2RV(af.mu_E, a, e, i, Omega, omega, f)
r_BN_N = np.array([0., 0., 0.])
v_BN_N = np.array([0., 0., 0.])
celPositionVec = r
celVelocityVec = v

# Begin Method
R_P1 = celPositionVec - r_BN_N
v_P1 = celVelocityVec - v_BN_N
a_P1 = np.array([0., 0., 0.])
a_P2 = np.array([0., 0., 0.])
boolSecondBody = True

boolSecondBody = False
boolValidConstraint = True

if (boolSecondBody == True):
    R_P2 = secPositionVec - r_BN_N
    v_P2 = secVelocityVec - v_BN_N
    R_P1_hat = normalize(R_P1)
    R_P2_hat = normalize(R_P2)
    dotProduct =  np.dot(R_P1_hat, R_P2_hat)
    if (dotProduct >= 1.0):
        platAngDiff = 0.0
    else:
        platAngDiff = np.arccos(np.dot(R_P1_hat, R_P2_hat))
    if np.abs(platAngDiff) < singularityThresh:
        boolValidConstraint = False

if np.logical_or(boolSecondBody == False, boolValidConstraint == False):
    R_P2 = np.cross(R_P1, v_P1)
    v_P2 = np.cross(R_P1, a_P1)
    a_P2 = np.cross(v_P1, a_P1)

computeCelestialTwoBodyPoint(R_P1, v_P1, a_P1, R_P2, v_P2, a_P2)












