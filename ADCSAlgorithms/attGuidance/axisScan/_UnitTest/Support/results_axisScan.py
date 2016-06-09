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

def initializeScan(sigma_R0N, theta0, psi0):
    M2 = rbk.Mi(-theta0, 2)
    M3 = rbk.Mi(psi0, 3)
    UR0 = np.dot(M2, M3)
    R0N = rbk.MRP2C(sigma_R0N)
    UN = np.dot(UR0, R0N)
    sigma_UN = rbk.C2MRP(UN)
    return sigma_UN

def printResults_axisScan(sigma_R0N, omega_R0N_N, domega_R0N_N, params, callTime):
    sigma_UN = params[0]
    psiDot = params[1]

    mnvrStartTime = 0.0
    if callTime == 0.0:
        mnvrStartTime = callTime
    currMnvrTime = callTime - mnvrStartTime
    RU = rbk.Mi(-psiDot * currMnvrTime, 3)
    UN = rbk.MRP2C(sigma_UN)
    RN = np.dot(RU, UN)
    sigma_RN = rbk.C2MRP(RN)

    omega_RU_R0 = np.array([0.0, 0.0, -psiDot])
    R0N = rbk.MRP2C(sigma_R0N)
    omega_RU_N = np.dot(R0N.T, omega_RU_R0)
    omega_RN_N = omega_RU_N + omega_R0N_N

    domega_RN_N = np.cross(omega_R0N_N, omega_RU_N) + domega_R0N_N

    print 'callTime = ', callTime
    print 'sigma_RN = ', sigma_RN
    print 'omega_RN_N = ', omega_RN_N
    print 'domega_RN_N = ', domega_RN_N
    print '\n'

    return UN

# Initial Conditions
sigma_R0N = np.array([ 0.1, 0.2, 0.3 ])
omega_R0N_N = np.array([0.1, 0.0, 0.0])
domega_R0N_N = np.array([0.0, 0.0, 0.0])
theta0 = 0.0
psi0 = 0.0
psiDot = 0.1

sigma_UN = initializeScan(sigma_R0N, theta0, psi0)
params = (sigma_UN, psiDot)
callTime = 0.0 # [sec]
printResults_axisScan(sigma_R0N, omega_R0N_N, domega_R0N_N, params, callTime)
callTime = 0.5 # [sec]
printResults_axisScan(sigma_R0N, omega_R0N_N, domega_R0N_N, params, callTime)
callTime = 1.0 # [sec]
printResults_axisScan(sigma_R0N, omega_R0N_N, domega_R0N_N, params, callTime)








