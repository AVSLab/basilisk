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

def printResults3DSpin(dt, sigma_RN, omega_RN_N):
    RN = rbk.MRP2C(sigma_RN)
    omega_RN_R = np.dot(RN, omega_RN_N)
    B = rbk.BmatMRP(sigma_RN)
    dsigma_RN = 0.25 * np.dot(B, omega_RN_R)
    sigma_RN += dsigma_RN * dt
    rbk.MRPswitch(sigma_RN, 1)
    print 'sigma_RN = ', sigma_RN
    print 'omega_RN_N = ', omega_RN_N
    print '\n'
    return (sigma_RN, omega_RN_N)


sigma_RN = np.array([0.1, 0.2, 0.3])
omega_RN_N = np.array([1., -1., 0.5]) * mc.D2R
print 'CallTime = 0.0'
dt = 0.0
(sigma_RN, omega_RN_N) = printResults3DSpin(dt, sigma_RN, omega_RN_N)
print 'CallTime = 0.5'
dt = 0.5
(sigma_RN, omega_RN_N) = printResults3DSpin(dt, sigma_RN, omega_RN_N)
print 'CallTime = 1.0'
(sigma_RN, omega_RN_N) = printResults3DSpin(dt, sigma_RN, omega_RN_N)
