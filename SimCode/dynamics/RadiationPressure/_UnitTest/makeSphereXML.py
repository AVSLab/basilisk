''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#
# Make XML Radiation Pressure Lookup for spherical object to test against cannonball
#
# Purpose:  to Make lookup tables for "cannonball" method comparison
# Author:   Scott Carnahan
# Creation Date:  July 31, 2017
#

# @cond DOXYGEN_IGNORE
import sys
import os
import pytest
import inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')
# @endcond

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class unitVectorXYZ():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

numPoints = 40
radius = 1.                         # radius of unit vector sphere
theta = np.linspace(0,2*np.pi, numPoints) # angle about third axis
phi = np.linspace(-np.pi,np.pi, numPoints)   # elevation from x-y plane
xVec = np.zeros(len(theta)*len(phi))   # x component of unit vector
yVec = np.zeros(len(theta)*len(phi))   # y component of unit vector
zVec = np.zeros(len(theta)*len(phi))   # z component of unit vector
sHat_B = unitVectorXYZ(xVec,yVec,zVec)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(numPoints):
    for j in range(numPoints):
        z = radius * np.cos(phi[j])
        x = radius * np.cos(theta[i]) * np.sin(phi[j])
        y = radius * np.sin(theta[i]) * np.sin(phi[j])
        sHat_B.x[numPoints*i+j] = x
        sHat_B.y[numPoints*i+j] = y
        sHat_B.z[numPoints*i+j] = z
        ax.scatter(sHat_B.x[numPoints*i+j], sHat_B.y[numPoints*i+j], sHat_B.z[numPoints*i+j])

plt.show()
lookupFile = open('cannonballLookup.XML', 'w')
lookupFile.write()


lookupFile.close()