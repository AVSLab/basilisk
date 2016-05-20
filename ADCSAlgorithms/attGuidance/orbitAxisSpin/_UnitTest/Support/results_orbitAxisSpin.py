import numpy as np
from numpy import linalg as la
from numpy import sin, cos
np.set_printoptions(precision=12)
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')
import RigidBodyKinematics as rbk
import astroFunctions as af

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

def computeInitialSpinAngle(o_spin, b_spin, sigma_R0N, sigma_BN):
    o1 = o_spin
    o2 = checkIndex(o1 + 1)

    b1 = b_spin
    b2 = checkIndex(b1 + 1)
    b3 = checkIndex(b2 + 1)

    BN = rbk.MRP2C(sigma_BN)
    R0N = rbk.MRP2C(sigma_R0N)

    RN = np.zeros([3,3])
    RN[b1] = R0N[o1]
    RN[b2] = R0N[o2]
    RN[b3] = np.cross(RN[b1], RN[b2])

    spin_align_angle = np.arccos(np.dot(BN[b1], RN[b1]))
    spin_align_axis = spin_align_angle * normalize(np.cross(BN[b1], RN[b1]))

    C_align = rbk.PRV2C(spin_align_axis)
    Temp33_1 = np.dot(C_align.T, BN)
    Temp33_2 = np.dot(RN, Temp33_1.T)
    PRV_algin = rbk.C2PRV(Temp33_2)
    phi_spin_0 = la.norm(PRV_algin)
    if np.dot(PRV_algin, RN[b1]) < 0:
        phi_spin_0 = - phi_spin_0
    return phi_spin_0


def printResults_orbitAxisSpin(sigma_R0N, omega_R0N_N, domega_R0N_N, params):
    o_spin = params[0]
    omega_spin = params[1]
    bool_initialize = params[2]
    phi_spin0 = params[3]
    callTime = params[4]

    R0N = rbk.MRP2C(sigma_R0N)
    omega_spin_vec = omega_spin * R0N[o_spin]
    omega_RN_N = omega_R0N_N +  omega_spin_vec
    domega_RN_N = domega_R0N_N + np.cross(omega_R0N_N, omega_spin_vec)
    if bool_initialize == False:
        phi_spin = phi_spin0 + callTime * omega_spin
    else:
        phi_spin = computeInitialSpinAngle(o_spin, b_spin, sigma_R0N, sigma_BN)
    Mi = rbk.Mi(phi_spin, o_spin + 1)
    sigma_RR0 = rbk.C2MRP(Mi)
    sigma_RN = sigma_RR0 + sigma_R0N

    print 'callTime = ', callTime
    print 'phi_spin = ', phi_spin
    print 'sigma_RN = ', sigma_RN
    print 'omega_RN_N = ', omega_RN_N
    print 'domega_RN_N = ', domega_RN_N
    print '\n'

    sigma_norm = la.norm(sigma_RN)
    RN = rbk.MRP2C(sigma_RN)
    end = True

    return (sigma_RN, omega_RN_N, domega_RN_N, phi_spin)

# Initial Conditions
# Attitude IC
sigma_R0N = np.array([-0.16367330847620223, -0.39514232112172260, -0.16367330847620221])
omega_R0N_N = np.array([-0.00383553448372837, 0.00383553448372837, 0.00000000000000000])
domega_R0N_N = np.array([0.00001786539057109, -0.00001786539057109, 0.00000000000000000])
sigma_BN = np.array([0.25, -0.45, 0.75])
# Spin Data
o_spin = 0
b_spin = 0
omega_spin = np.pi/8
bool_initialize = False
# Phi spin initialization
if bool_initialize == True: # compute the actual initial phi spin angle
    phi_spin0 = computeInitialSpinAngle(o_spin, b_spin, sigma_R0N, sigma_BN)
else: # begin process with a random phi spin angle
    phi_spin0 = np.pi/4

callTime = 0 # sec
params = (o_spin, omega_spin, bool_initialize, phi_spin0, callTime)
printResults_orbitAxisSpin(sigma_R0N, omega_R0N_N, domega_R0N_N, params)
callTime = 0.5 #sec
params = (o_spin, omega_spin, bool_initialize, phi_spin0, callTime)
printResults_orbitAxisSpin(sigma_R0N, omega_R0N_N, domega_R0N_N, params)








