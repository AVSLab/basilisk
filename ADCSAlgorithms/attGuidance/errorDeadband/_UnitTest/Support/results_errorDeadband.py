import sys, os, inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import numpy as np
from numpy import linalg as la
import RigidBodyKinematics as rbk
import macros as mc

np.set_printoptions(precision=12)


sigma_BR = np.array([0.03, -0.05, 0.0])
omega_BR_B = np.array([0.010, -0.020, 0.015])

innerThresh = 5 # [deg]
outerThresh = 20 # [deg]

attError = 4 * np.arctan(la.norm(sigma_BR))
rateError = la.norm(omega_BR_B)
error = np.sqrt(attError * attError + rateError * rateError)
print 'Error [deg] = ', error * mc.R2D
