import numpy as np
np.set_printoptions(precision=12)
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

# Initial Conditions
sigma_BR = np.array([0.3, -0.5, 0.7])
omega_BR_B = np.array([0.010, -0.020, 0.015])
omega_RN_B = np.array([-0.02, -0.01, 0.005])
domega_RN_B = np.array([0.0002, 0.0003, 0.0001])

I = np.array([
    [1000., 0., 0.],
    [0., 800., 0.],
    [0., 0., 800.]
])

K = 0.15
P = 150.0

# Begin Method
L = np.array([0., 0., 0.])
omega_BN_B = omega_BR_B + omega_RN_B
temp1 = np.dot(I, omega_BN_B)
temp2 = domega_RN_B - np.cross(omega_BN_B, omega_RN_B)
Lr = K * sigma_BR + P * omega_BR_B - np.cross(omega_BN_B, temp1) - np.dot(I, temp2)

# Print Results
print 'Lr = ', Lr
