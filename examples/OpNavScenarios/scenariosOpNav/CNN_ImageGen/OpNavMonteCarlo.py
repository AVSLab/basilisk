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
r"""
Overview
--------

The OpNav Monte-Carlo python scripts provides the capability to generate images and truth data in order to
train neural networks for image processing.

This script calls OpNavScenarios/CNN_ImageGen/scenario_CNNImages.py in order to generate the simulations.
The script can be called by running::

    python3 OpNavMonteCarlo.py

"""



import csv
import inspect
import os

import scenario_CNNImages as scenario

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities.MonteCarlo.Controller import Controller, RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import OrbitalElementDispersion, MRPDispersionPerAxis, UniformDispersion

# import simulation related support
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import macros
import matplotlib.pyplot as plt
import numpy as np


retainedMessageName1 = "scMsg"
retainedMessageName2 = "circlesMsg"
retainedRate = macros.sec2nano(10)
var1 = "r_BN_N"
var2 = "sigma_BN"
var3 = "valid"

def run(show_plots):
    """Main Simulation Method"""

    NUMBER_OF_RUNS = 10
    VERBOSE = True
    PROCESSES = 1
    RUN = True
    POST = True

    dirName = os.path.abspath(os.path.dirname(__file__)) + "/cnn_MC_data"
    if RUN:
        myExecutionFunction = scenario.run
        myCreationFunction = scenario.scenario_OpNav

        monteCarlo = Controller()
        monteCarlo.setShouldDisperseSeeds(True)
        monteCarlo.setExecutionFunction(myExecutionFunction)
        monteCarlo.setSimulationFunction(myCreationFunction)
        monteCarlo.setExecutionCount(NUMBER_OF_RUNS)
        monteCarlo.setThreadCount(PROCESSES)
        monteCarlo.setVerbose(True)
        monteCarlo.setArchiveDir(dirName)

        # Add some dispersions
        dispDict = {}
        dispDict["mu"] = 4.2828371901284001E+13
        dispDict["a"] = ["normal", 14000*1E3, 2500*1E3] # 12000
        dispDict["e"] = ["uniform", 0.2, 0.5]           # 0.4, 0.7
        dispDict["i"] = ["uniform", np.deg2rad(40), np.deg2rad(90)]
        dispDict["Omega"] = None
        dispDict["omega"] = None
        dispDict["f"] = ["uniform", np.deg2rad(0), np.deg2rad(359)]

        disp1Name = 'get_DynModel().scObject.hub.r_CN_NInit'
        disp2Name = 'get_DynModel().scObject.hub.v_CN_NInit'
        disp3Name = 'get_FswModel().trackingErrorCam.sigma_R0R'
        dispGauss = 'get_DynModel().cameraMod.gaussian'
        dispDC = 'get_DynModel().cameraMod.darkCurrent'
        dispSP = 'get_DynModel().cameraMod.saltPepper'
        dispCR = 'get_DynModel().cameraMod.cosmicRays'
        dispBlur = 'get_DynModel().cameraMod.blurParam'

        monteCarlo.addDispersion(OrbitalElementDispersion(disp1Name,disp2Name, dispDict))
        monteCarlo.addDispersion(MRPDispersionPerAxis(disp3Name, bounds=[[1./3-0.051, 1./3+0.051], [1./3-0.051, 1./3+0.051], [-1./3-0.051, -1./3+0.051]]))
        monteCarlo.addDispersion(UniformDispersion(dispGauss, [0,5]))
        monteCarlo.addDispersion(UniformDispersion(dispSP, [0,2.5]))
        monteCarlo.addDispersion(UniformDispersion(dispCR, [0.5,4]))
        monteCarlo.addDispersion(UniformDispersion(dispBlur, [1,6]))

        # Add retention policy
        retentionPolicy = RetentionPolicy()
        retentionPolicy.addMessageLog(retainedMessageName1, [var1, var2])
        retentionPolicy.addMessageLog(retainedMessageName2, [var3])
        monteCarlo.addRetentionPolicy(retentionPolicy)

        failures = monteCarlo.executeSimulations()
        assert len(failures) == 0, "No runs should fail"

    if POST:
        monteCarlo = Controller.load(dirName)
        for i in range(0,NUMBER_OF_RUNS):
            try:
                monteCarloData = monteCarlo.getRetainedData(i)
            except FileNotFoundError:
                print("File not found, ",  i)
                continue
        csvfile = open(dirName + "/run" + str(i) + "/data.csv", 'w')
        writer = csv.writer(csvfile)
        writer.writerow(['Filename', 'Valid', 'X_p', 'Y_p', 'rho_p', 'r_BN_N_1', 'r_BN_N_2', 'r_BN_N_3'])

        timeAxis = monteCarloData["messages"][retainedMessageName1 + ".times"]
        position_N = unitTestSupport.addTimeColumn(timeAxis,
                                                   monteCarloData["messages"][retainedMessageName1 + "." + var1])
        sigma_BN = unitTestSupport.addTimeColumn(timeAxis,
                                                 monteCarloData["messages"][retainedMessageName1 + "." + var2])
        validCircle = unitTestSupport.addTimeColumn(timeAxis,
                                                    monteCarloData["messages"][retainedMessageName2 + "." + var3])

        renderRate = 60*1E9
        sigma_CB = [0., 0., 0.]  # Arbitrary camera orientation
        sizeOfCam = [512, 512]
        sizeMM = [10. * 1E-3, 10. * 1E-3]  # in m
        fieldOfView = np.deg2rad(55)  # in degrees
        focal = sizeMM[0] / 2. / np.tan(fieldOfView / 2.)  # in m

        pixelSize = []
        pixelSize.append(sizeMM[0] / sizeOfCam[0])
        pixelSize.append(sizeMM[1] / sizeOfCam[1])

        dcm_CB = rbk.MRP2C(sigma_CB)
        # Plot results

        trueRhat_C = np.full([len(validCircle[:, 0]), 4], np.nan)
        trueCircles = np.full([len(validCircle[:, 0]), 4], np.nan)
        trueCircles[:, 0] = validCircle[:, 0]
        trueRhat_C[:, 0] = validCircle[:, 0]

        ModeIdx = 0
        Rmars = 3396.19 * 1E3
        for j in range(len(position_N[:, 0])):
            if position_N[j, 0] in validCircle[:, 0]:
                ModeIdx = j
                break
        for i in range(len(validCircle[:, 0])):
            if validCircle[i, 1] > 1E-5 or (validCircle[i, 0]%renderRate ==0 and validCircle[i, 0] > 1):
                trueRhat_C[i, 1:] = np.dot(np.dot(dcm_CB, rbk.MRP2C(sigma_BN[ModeIdx + i, 1:4])),
                                           position_N[ModeIdx + i, 1:4]) / np.linalg.norm(position_N[ModeIdx + i, 1:4])
                trueCircles[i, 3] = focal * np.tan(np.arcsin(Rmars / np.linalg.norm(position_N[ModeIdx + i, 1:4]))) / pixelSize[0]
                trueRhat_C[i, 1:] *= focal / trueRhat_C[i, 3]
                trueCircles[i, 1] = trueRhat_C[i, 1] / pixelSize[0] + sizeOfCam[0] / 2 - 0.5
                trueCircles[i, 2] = trueRhat_C[i, 2] / pixelSize[1] + sizeOfCam[1] / 2 - 0.5

                writer.writerow([str("{0:.6f}".format(position_N[i,0]*1E-9))+".jpg", validCircle[i, 1], trueCircles[i, 1], trueCircles[i, 2], trueCircles[i, 3], position_N[i,1], position_N[i,2], position_N[i,3]])
        csvfile.close()

    if show_plots:
        monteCarlo.executeCallbacks()
        plt.show()

    return


if __name__ == "__main__":
    pid = run(True)
