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

The OpNav Monte-Carlo python scripts provides the capability to run Monte-Carlo simulations using a OpNav scenario.
This script contains the dispersions applied, the plotting methods used, and the number of desired runs.

This script calls OpNavScenarios/OpNavMC/scenario_OpNavAttODMC.py (or others) in order to generate the simulations.
The script can be called by running::

    python3 MonteCarlo.py

"""


import os
import inspect
# import scenario_LimbAttOD as scenario
import scenario_OpNavAttODMC as scenario
from BSK_OpNav import BSKSim

Sim = BSKSim()
viz_path = Sim.vizPath

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities.MonteCarlo.Controller import Controller, RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import OrbitalElementDispersion, UniformDispersion
# import simulation related support
from Basilisk.utilities import macros
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

m2km = 1.0 / 1000.0
ns2min = 1/60.*1E-9

mpl.rcParams.update({'font.size' : 8 })
#seaborn-colorblind, 'seaborn-paper', 'bmh', 'tableau-colorblind10', 'seaborn-deep', 'myStyle', 'aiaa'

# plt.style.use("myStyle")
params = {'axes.labelsize': 8,'axes.titlesize':8, 'legend.fontsize': 8, 'xtick.labelsize': 7, 'ytick.labelsize': 7, 'text.usetex': True}
mpl.rcParams.update(params)

retainedMessageNameSc = "scMsg"
retainedMessageNameFilt = "filtMsg"
retainedMessageNameOpNav = "opnavMsg"
retainedRate = macros.sec2nano(10)

def displayPlots(data, retentionPolicy):
    mpl.rcParams['image.cmap'] = 'inferno'

    position_N = data["messages"][retainedMessageNameSc + ".r_BN_N"]
    vel_N = data["messages"][retainedMessageNameSc + ".v_BN_N"]
    states = data["messages"][retainedMessageNameFilt + ".state"]
    covar = data["messages"][retainedMessageNameFilt + ".covar"]
    valid = data["messages"][retainedMessageNameOpNav + ".valid"]

    truth = np.zeros([len(position_N[:, 0]), 7])
    truth[:, 0:4] = np.copy(position_N)
    truth[:, 4:7] = np.copy(vel_N[:, 1:])

    validIdx = []
    for i in range(len(valid[:,0])):
        if np.abs(valid[i,1] - 1) < 0.01:
            validIdx.append(i)
    diffPos = np.full([len(validIdx), 2], np.nan)
    diffVel = np.full([len(validIdx), 2], np.nan)
    covarPos = np.full([len(validIdx), 2], np.nan)
    covarVel = np.full([len(validIdx), 2], np.nan)

    m2km2 = m2km
    for i in range(len(validIdx)):
        diffPos[i,0] = states[validIdx[i],0]
        diffPos[i,1] = np.linalg.norm(states[validIdx[i],1:4] - truth[validIdx[i],1:4])/np.linalg.norm(truth[validIdx[i],1:4])*100
        diffVel[i,0] = states[validIdx[i],0]
        diffVel[i,1] = np.linalg.norm(states[validIdx[i],4:7] - truth[validIdx[i],4:7])/np.linalg.norm(truth[validIdx[i],4:7])*100
        covarPos[i,0] = states[validIdx[i],0]
        posVec = np.sqrt(np.array([covar[validIdx[i],1], covar[validIdx[i],1 + 6+1], covar[validIdx[i],1 + 2*(6+1)]]))
        covarPos[i,1] =  3*np.linalg.norm(posVec)/np.linalg.norm(truth[validIdx[i],1:4])*100
        covarVel[i,0] = states[validIdx[i],0]
        velVec = np.sqrt(np.array([covar[validIdx[i],1 + 3*(6+1)], covar[validIdx[i],1 + 4*(6+1)], covar[validIdx[i],1 + 5*(6+1)]]))
        covarVel[i,1] = 3*np.linalg.norm(velVec)/np.linalg.norm(truth[validIdx[i],4:7])*100
    # plt.figure(101, figsize=(2.7, 1.6), facecolor='w', edgecolor='k')
    plt.figure(101, figsize=(3.5, 2), facecolor='w', edgecolor='k')
    plt.plot(diffPos[:, 0] * ns2min, diffPos[:, 1])
    plt.ylabel("$\mathbf{r}_\mathrm{"+"Circ"+"}$ errors ($\%$)")
    plt.xlabel("Time (min)")
    # plt.ylim([0,3.5])
    plt.savefig('MCErrorPos.pdf')

    plt.figure(103, figsize=(3.5, 2), facecolor='w', edgecolor='k')
    plt.plot(covarPos[:, 0] * ns2min, covarPos[:,1], linestyle = '--')
    plt.ylabel("$\mathbf{r}_\mathrm{"+"Circ"+"}$ covar ($\%$)")
    plt.xlabel("Time (min)")
    # plt.ylim([0,3.5])
    plt.savefig('MCCovarPos.pdf')

    plt.figure(102, figsize=(3.5, 2.), facecolor='w', edgecolor='k')
    plt.plot(diffVel[:, 0] * ns2min, diffVel[:, 1])
    plt.ylabel("$\dot{\mathbf{r}}_\mathrm{"+"Circ"+ "}$ errors ($\%$)")
    plt.xlabel("Time (min)")
    plt.savefig('MCErrorVel.pdf')

    plt.figure(104, figsize=(3.5, 2.), facecolor='w', edgecolor='k')
    plt.plot(covarVel[:, 0] * ns2min, covarVel[:,1], linestyle = '--')
    plt.ylabel("$\dot{\mathbf{r}}_\mathrm{"+"Circ"+ "}$ covar ($\%$)")
    plt.xlabel("Time (min)")
    plt.savefig('MCCovarVel.pdf')


def run(show_plots):

    NUMBER_OF_RUNS = 2
    VERBOSE = True
    PROCESSES = 1
    RUN = True
    POST = False

    dirName = os.path.abspath(os.path.dirname(__file__)) + "/MC_data"
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
        dispDict["a"] = ["normal", 22000*1E3, 3000*1E3]
        dispDict["e"] = ["uniform", 0.2, 0.4]
        dispDict["i"] = ["uniform", -np.deg2rad(20), np.deg2rad(20)]
        dispDict["Omega"] = None
        dispDict["omega"] = None
        dispDict["f"] = ["uniform", 0., np.deg2rad(180)]

        disp1Name = 'get_DynModel().scObject.hub.r_CN_NInit'
        disp2Name = 'get_DynModel().scObject.hub.v_CN_NInit'
        dispFOV = 'get_DynModel().cameraMod.fieldOfView'
        dispNoise = 'get_FswModel().relativeOD.noiseSF'
        monteCarlo.addDispersion(UniformDispersion(dispNoise, [1, 10]))
        monteCarlo.addDispersion(UniformDispersion(dispFOV, [np.deg2rad(40) - np.deg2rad(0.001), np.deg2rad(40) + np.deg2rad(0.001)]))
        monteCarlo.addDispersion(OrbitalElementDispersion(disp1Name,disp2Name, dispDict))

        # Add retention policy
        retentionPolicy = RetentionPolicy()
        retentionPolicy.addMessageLog(retainedMessageNameSc, ["r_BN_N", "v_BN_N", "sigma_BN"])
        retentionPolicy.addMessageLog(retainedMessageNameOpNav, ["r_BN_N", "covar_N", "r_BN_C", "covar_C", "valid"])
        retentionPolicy.addMessageLog(retainedMessageNameFilt, ["state", "covar"])
        retentionPolicy.setDataCallback(displayPlots)
        monteCarlo.addRetentionPolicy(retentionPolicy)

        failures = monteCarlo.executeSimulations()
        assert len(failures) == 0, "No runs should fail"

        if show_plots:
            monteCarlo.executeCallbacks()
            plt.show()

    return


if __name__ == "__main__":
    run(False)


