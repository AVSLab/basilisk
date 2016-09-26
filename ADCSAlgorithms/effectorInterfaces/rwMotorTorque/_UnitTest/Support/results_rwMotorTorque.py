import numpy as np
from numpy import linalg as la

def controlAxes3D():
    C = np.array([
        [1.0, 0.0, 0.0]
        , [0.0, 1.0, 0.0]
        , [0.0, 0.0, 1.0]
    ])
    return C
def controlAxes2D():
    C = np.array([
        [1.0, 0.0, 0.0]
        , [0.0, 0.0, 1.0]
    ])
    return C
def controlAxes1D():
    C = np.array([
        [1.0, 0.0, 0.0]
    ])
    return C

Gs_B = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.5773502691896258, 0.5773502691896258, 0.5773502691896258]
]).T

JsList = np.array([0.1, 0.1, 0.1, 0.1])
numRW = 4
rwConfigParams = (Gs_B, JsList, numRW)

Lr = np.array([1.0, -0.5, 0.7])
rwAvailability = np.array([1, 1, 1, 1])

def computeTorqueU(C, Gs_B, Lr):
    CGs = np.dot(C, Gs_B)
    M = np.dot(CGs, CGs.T)
    M_inv = la.inv(M)
    A = np.dot(CGs.T, M_inv)
    CLr = np.dot(C, Lr)
    u_s = np.dot(A, CLr)

    print 'CGs = \n', CGs
    print 'A = \n', A
    print 'CLr = \n', CLr
    return u_s


print '3D Control'
u_s = computeTorqueU(controlAxes3D(), Gs_B, Lr)
print 'U_s = ', u_s, '\n'

print '2D Control'
u_s = computeTorqueU(controlAxes2D(), Gs_B, Lr)
print 'U_s = ', u_s

print '1D Control'
u_s = computeTorqueU(controlAxes1D(), Gs_B, Lr)
print 'U_s = ', u_s


