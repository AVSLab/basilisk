import numpy as np
from numpy import linalg as la
np.set_printoptions(precision=12)
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')
import RigidBodyKinematics as rbk

def normalize(v):
    norm=np.linalg.norm(v)
    if norm==0:
       return v
    return v/norm

def printResults_HillPoint(r_BN_N, v_BN_N, celBodyPosVec, celBodyVelVec):
    r = r_BN_N - celBodyPosVec
    v = v_BN_N - celBodyVelVec
    h = np.cross(r, v)
    i_r = normalize(r)
    i_h = normalize(h)
    i_theta = np.cross(i_h, i_r)
    HN = np.array([ i_r, i_theta, i_h ])
    sigma_HN = rbk.C2MRP(HN)

    hm = la.norm(h)
    rm = la.norm(r)
    drdt = np.dot(v, i_r)
    dfdt = hm / (rm * rm)
    ddfdt2 = -2.0 * drdt / rm * dfdt

    omega_HN_N = dfdt * i_h
    domega_HN_N = ddfdt2 * i_h

    print 'sigma_HN = ', sigma_HN
    print 'omega_HN_N = ', omega_HN_N
    print 'domega_HN_N = ', domega_HN_N

    return (sigma_HN, omega_HN_N, domega_HN_N)

# MAIN
# Initial Conditions (IC)
r_BN_N = np.array([500., 500., 1000.])
v_BN_N = np.array([0., 20., 0.])
celBodyPosVec = np.array([-500., -500., 0.])
celBodyVelVec = np.array([0., 0., 0.])
# Print generated Hill Frame for the given IC
printResults_HillPoint(r_BN_N, v_BN_N, celBodyPosVec, celBodyVelVec)
