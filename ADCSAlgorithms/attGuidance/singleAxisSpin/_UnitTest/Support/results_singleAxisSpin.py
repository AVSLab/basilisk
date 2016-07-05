import sys, os, inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import numpy as np
import RigidBodyKinematics as rbk
import macros as mc

np.set_printoptions(precision=12)

def printResultsSingleAxisSpin(dt, sigma_R0N, rotVector):
    R0N = rbk.MRP2C(sigma_R0N)
    PRV = dt * rotVector
    C = rbk.PRV2C(PRV)
    RN = np.dot(C, R0N)
    sigma_RN = rbk.C2MRP(RN)
    omega_RN = rotVector
    domega_RN = np.array([0., 0., 0.])

    print '\n'
    print 'sigma_RN = ', sigma_RN
    print 'omega_RN = ', omega_RN
    print 'domega_RN = ', domega_RN
    print '\n'
    return sigma_RN


sigma_RN = np.array([0.1, 0.2, 0.3])
omega_spin = np.array([1., 1., 1.]) * mc.D2R
print 'CallTime = 0.0'
dt = 0.0
sigma_RN = printResultsSingleAxisSpin(dt, sigma_RN, omega_spin)
print 'CallTime = 0.5'
dt = 0.5
sigma_RN = printResultsSingleAxisSpin(dt, sigma_RN, omega_spin)
print 'CallTime = 1.0'
sigma_RN = printResultsSingleAxisSpin(dt, sigma_RN, omega_spin)
