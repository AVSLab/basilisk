import os
import inspect
# import scenario_LimbAttOD as scenario
import scenario_OpNavAttOD as scenario
from BSK_masters import BSKSim, BSKScenario
import BSK_OpNavDynamics, BSK_OpNavFsw
import csv, subprocess, signal

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities.MonteCarlo.Controller import Controller, RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import OrbitalElementDispersion, UniformDispersion, NormalDispersion, NormalVectorCartDispersion
# import simulation related support
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

m2km = 1.0 / 1000.0
ns2min = 1/60.*1E-9

mpl.rcParams.update({'font.size' : 8 })
#seaborn-colorblind, 'seaborn-paper', 'bmh', 'tableau-colorblind10', 'seaborn-deep', 'myStyle', 'aiaa'

plt.style.use("myStyle")
params = {'axes.labelsize': 8,'axes.titlesize':8, 'legend.fontsize': 8, 'xtick.labelsize': 7, 'ytick.labelsize': 7, 'text.usetex': True}
mpl.rcParams.update(params)

def displayPlots(data, retentionPolicy):
    mpl.rcParams['image.cmap'] = 'inferno'

    position_N = np.array(data["messages"]["inertial_state_output.r_BN_N"])
    vel_N = np.array(data["messages"]["inertial_state_output.v_BN_N"])
    states = np.array(data["messages"]["relod_filter_data.state"])
    covar = np.array(data["messages"]["relod_filter_data.covar"])
    valid = np.array(data["messages"]["output_nav_msg.valid"])

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



NUMBER_OF_RUNS = 100
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
    # disp3Name = 'get_FswModel().trackingErrorCamData.sigma_R0R'
    dispFOV = 'get_DynModel().cameraMod.fieldOfView'
    dispNoise = 'get_FswModel().relativeODData.noiseSF'
    # disp3Name = 'get_FswModel().trackingErrorCamData.sigma_R0R' = 5#7.5
    monteCarlo.addDispersion(UniformDispersion(dispNoise, [1, 10]))
    monteCarlo.addDispersion(UniformDispersion(dispFOV, [np.deg2rad(40) - np.deg2rad(0.001), np.deg2rad(40) + np.deg2rad(0.001)]))
    monteCarlo.addDispersion(OrbitalElementDispersion(disp1Name,disp2Name, dispDict))
    # monteCarlo.addDispersion(MRPDispersionPerAxis(disp3Name, bounds=[[1./3-0.05, 1./3+0.05], [1./3-0.05, 1./3+0.05], [-1./3-0.05, -1./3+0.05]]))

    # Add retention policy
    retentionPolicy = RetentionPolicy()
    retentionPolicy.addMessageLog("inertial_state_output", [("r_BN_N", range(3)), ("v_BN_N", range(3)), ("sigma_BN", range(3))], macros.sec2nano(10))
    retentionPolicy.addMessageLog("output_nav_msg", [("r_BN_N", range(3)), ("covar_N", range(3*3)), ("r_BN_C", range(3)), ("covar_C", range(3*3)), ("valid", range(1))], macros.sec2nano(10)) #horizon nav
    retentionPolicy.addMessageLog("relod_filter_data", [("state", range(6)), ("covar", range(6*6))], macros.sec2nano(10)) #horizon nav
    retentionPolicy.setDataCallback(displayPlots)
    monteCarlo.addRetentionPolicy(retentionPolicy)

    appPath = '/Applications/OpNavScene.app'
    child = subprocess.Popen(["open", appPath, "--args", "-opNavMode", "tcp://localhost:5556"])  # ,,"-batchmode",
    # os.system("nice -n -20 " + str(child.pid))
    print("Vizard spawned with PID = " + str(child.pid))

    failures = monteCarlo.executeSimulations()
    assert len(failures) == 0, "No runs should fail"

    monteCarlo.executeCallbacks()
    plt.show()

os.kill(child.pid + 1, signal.SIGKILL)

