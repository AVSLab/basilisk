''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
import sys, os, inspect
import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros as mc
from Basilisk.utilities import unitTestSupport as sp

def plotResults(dataLr, dataSigmaBR, dataOmegaBR, dataPos, dataVel, dataSigmaBN):
    # Plot the results
    plt.close("all")        # clears out plots from earlier test runs
    timeLineFSW = dataSigmaBR[:, 0] * mc.NANO2MIN


    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    vectorData = sp.pullVectorSetFromData(dataSigmaBR)
    sNorm = np.array([np.linalg.norm(v) for v in vectorData])
    plt.plot(timeLineFSW, sNorm, color=sp.getLineColor(1,3),)
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error Norm $|\sigma_{B/R}|$')
    ax.set_yscale('log')

    plt.figure(2)
    for idx in range(1,4):
        plt.plot(timeLineFSW, dataLr[:, idx],
                 color=sp.getLineColor(idx,3),
                 label='$L_{r,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')

    plt.figure(3)
    for idx in range(1,4):
        plt.plot(timeLineFSW, dataOmegaBR[:, idx],
                 color=sp.getLineColor(idx,3),
                 label='$\omega_{BR,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    timeLineDYN = dataPos[:, 0] * mc.NANO2MIN
    vectorPosData = sp.pullVectorSetFromData(dataPos)
    vectorVelData = sp.pullVectorSetFromData(dataVel)
    vectorMRPData = sp.pullVectorSetFromData(dataSigmaBN)
    data = np.empty([len(vectorPosData),3])
    for idx in range(0,len(vectorPosData)):
        ir = vectorPosData[idx] / np.linalg.norm(vectorPosData[idx])
        hv = np.cross(vectorPosData[idx], vectorVelData[idx])
        ih = hv / np.linalg.norm(hv)
        itheta = np.cross(ih,ir)
        dcmBN = rbk.MRP2C(vectorMRPData[idx])
        data[idx] = [np.dot(ir, dcmBN[0]), np.dot(itheta, dcmBN[1]), np.dot(ih, dcmBN[2])]
    plt.figure(4)
    labelStrings = (r'$\hat\imath_r\cdot \hat b_1$'
                    , r'${\hat\imath}_{\theta}\cdot \hat b_2$'
                    , r'$\hat\imath_h\cdot \hat b_3$')
    for idx in range(0,3):
        plt.plot(timeLineDYN, data[:, idx],
                 color=sp.getLineColor(idx+1,3),
                 label=labelStrings[idx])
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Orientation Illustration')


    plt.show()
    plt.close("all")
    return
