#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
# Make XML Radiation Pressure Lookup for spherical object to test against cannonball
#
# Purpose:  to Make lookup tables for "cannonball" method comparison
# Author:   Scott Carnahan
# Creation Date:  July 31, 2017
#

# @cond DOXYGEN_IGNORE


# @endcond

import numpy as np
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt

class unitVectorXYZ():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

numPoints = 64
radius = 1.                         # radius of unit vector sphere
theta = np.linspace(0., 2.*np.pi, numPoints, endpoint=False) # angle about third axis
phi = np.linspace(-np.pi, np.pi, numPoints, endpoint=False)   # elevation from x-y plane
xVec = np.zeros(len(theta)*len(phi))   # x component of unit vector
yVec = np.zeros(len(theta)*len(phi))   # y component of unit vector
zVec = np.zeros(len(theta)*len(phi))   # z component of unit vector
sHat_B = unitVectorXYZ(xVec,yVec,zVec)

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

for i in range(numPoints):
    for j in range(numPoints):
        print(np.cos(theta[i]), np.sin(phi[j]))
        x = radius * np.cos(theta[i]) * np.sin(phi[j])
        y = radius * np.sin(theta[i]) * np.sin(phi[j])
        z = radius * np.cos(phi[j])
        sHat_B.x[numPoints*i+j] = x
        sHat_B.y[numPoints*i+j] = y
        sHat_B.z[numPoints*i+j] = z
        # ax.scatter(sHat_B.x[numPoints*i+j], sHat_B.y[numPoints*i+j], sHat_B.z[numPoints*i+j])

# plt.show()
lookupFile = open('cannonballLookup2.xml', 'w')

nl = '\n'
singleTab = '   '
doubleTab = '       '
tripleTab = '           '
header = '<?xml version="1.0" encoding="utf-8"?>'
srp_values = '<srp_values>'
sHat_B_values = '   <sHatBValues>'
lines =header,nl,srp_values,nl,sHat_B_values,nl
lookupFile.writelines(lines)

for i in range(len(sHat_B.x)):
    top ='      <sHat_B index="' + str(i) + '">'
    value1 = '         <value_1>' + '{:1.16f}'.format(sHat_B.x[i]) + '</value_1>'
    value2 = '         <value_2>' + '{:1.16f}'.format(sHat_B.y[i]) + '</value_2>'
    value3 = '         <value_3>' + '{:1.16f}'.format(sHat_B.z[i]) + '</value_3>'
    bottom = '      </sHat_B>'
    lines = top,nl,value1,nl,value2,nl,value3,nl,bottom,nl
    lookupFile.writelines(lines)
endSHat_B_values ='   </sHatBValues>'


startForce_B_values = '   <forceBValues>'
lines = endSHat_B_values, nl, startForce_B_values, nl
lookupFile.writelines(lines)

for i in range(len(sHat_B.x)):
    top ='      <force_B index="' + str(i) + '">'
    value1 = '         <value_1>' + '{:1.16f}'.format(-sHat_B.x[i]) + '</value_1>'
    value2 = '         <value_2>' + '{:1.16f}'.format(-sHat_B.y[i]) + '</value_2>'
    value3 = '         <value_3>' + '{:1.16f}'.format(-sHat_B.z[i]) + '</value_3>'
    bottom = '      </force_B>'
    lines = top,nl,value1,nl,value2,nl,value3,nl,bottom,nl
    lookupFile.writelines(lines)
endForce_values ='   </forceBValues>'

startTorqueValues = '   <torqueBValues>'
lines = endForce_values, nl, startTorqueValues, nl
lookupFile.writelines(lines)

torqueValue = .0000000000000000


for i in range(len(sHat_B.x)):
    top ='      <torque_B index="' + str(i) + '">'
    value1 = '         <value_1>' + '{:1.16f}'.format(torqueValue) + '</value_1>'
    value2 = '         <value_2>' + '{:1.16f}'.format(torqueValue) + '</value_2>'
    value3 = '         <value_3>' + '{:1.16f}'.format(torqueValue) + '</value_3>'
    bottom = '      </torque_B>'
    lines = top,nl,value1,nl,value2,nl,value3,nl,bottom,nl
    lookupFile.writelines(lines)
endTorqueValues = '   </torqueBValues>'
endFile = '</srp_values>'
lines = endTorqueValues, nl, endFile
lookupFile.writelines(lines)

lookupFile.close()
