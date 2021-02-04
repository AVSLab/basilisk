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
This script sets up a formation flying scenario with two spacecraft. 
The deputy spacecraft reconfigures its relative orbit in one orbit from one initial orbital element difference to
target orbital element difference.
This script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioFormationReconfig.py

The simulation layout is shown in the following illustration. 
Two spacecraft are orbiting the earth at close distance. No perturbation in assumed. 
Each spacecraft sends a :ref:`simpleNav` output message of type :ref:`NavAttMsg` message at a certain period
to :ref:`spacecraftReconfig`,
where burn scheduling is executed to achieve reconfiguration.

.. image:: /_images/static/test_scenarioFormationReconfig.svg
    :align: center

Illustration of Simulation Results
----------------------------------
::

    show_plots = True, useRefAttitude = False

In this case, reference attitude input is omitted.
Therefore, attitude control is executed to achieve thruster burn.
This resulting feedback control error is shown below.

.. image:: /_images/Scenarios/scenarioFormationReconfig10.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFormationReconfig20.svg
   :align: center

::

    show_plots = True, useRefAttitude = False

In this case, reference attitude input is included.
Therefore, attitude control is executed to both achieve thruster burn and reference attitude.
This resulting feedback control error is shown below.

.. image:: /_images/Scenarios/scenarioFormationReconfig11.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFormationReconfig21.svg
   :align: center
"""


import itertools
import numpy as np
import math
import matplotlib.pyplot as plt
import os

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import vizSupport
from Basilisk.architecture import sim_model
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import spacecraftReconfig
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, useRefAttitude):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        useRefAttitude (bool): Determines if reference attitude is used
    """
    scSim = SimulationBaseClass.SimBaseClass()

    # ----- dynamics ----- #
    dynProcessName = "dynProcess"
    dynTaskName = "dynTask"
    dynProcess = scSim.CreateNewProcess(dynProcessName)
    timeStep = 2.0
    dynTimeStep = macros.sec2nano(timeStep)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, dynTimeStep))

    # sc
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject2 = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "scObject"
    scObject2.ModelTag = "scObject2"

    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 500.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject2.hub.mHub = 500.0
    scObject2.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject2.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    scSim.AddModelToTask(dynTaskName, scObject, None, 2)
    scSim.AddModelToTask(dynTaskName, scObject2, None, 2)

    # grav
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth'])
    gravBodies['earth'].isCentralBody = True
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(
        list(gravFactory.gravBodies.values()))
    scObject2.gravField.gravBodies = spacecraftPlus.GravBodyVector(
        list(gravFactory.gravBodies.values()))

    # thruster
    thrusterEffector2 = thrusterDynamicEffector.ThrusterDynamicEffector()
    scSim.AddModelToTask(dynTaskName, thrusterEffector2, None, 3)
    thFactory2 = simIncludeThruster.thrusterFactory()
    location = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    direction = [[1.0, 1.0, 1.0], [1.0, 1.0, 1.0]]  # get thrust in +z direction
    for pos_B, dir_B in zip(location, direction):
        thFactory2.create('MOOG_Monarc_22_6', pos_B, dir_B, useMinPulseTime=False)
    thFactory2.addToSpacecraft(scObject2.ModelTag, thrusterEffector2, scObject2)

    # extObj
    extFTObject2 = extForceTorque.ExtForceTorque()
    extFTObject2.ModelTag = "externalDisturbance2"
    scObject2.addDynamicEffector(extFTObject2)
    scSim.AddModelToTask(dynTaskName, extFTObject2, None, 3)

    # simple nav
    simpleNavObject = simpleNav.SimpleNav()
    simpleNavObject2 = simpleNav.SimpleNav()
    simpleNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    simpleNavObject2.scStateInMsg.subscribeTo(scObject2.scStateOutMsg)
    scSim.AddModelToTask(dynTaskName, simpleNavObject, None, 1)
    scSim.AddModelToTask(dynTaskName, simpleNavObject2, None, 1)

    # ----- fsw ----- #
    fswProcessName = "fswProcess"
    fswTaskName = "fswTask"
    fswProcess = scSim.CreateNewProcess(fswProcessName)
    fswTimeStep = macros.sec2nano(timeStep)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    # inertial 3D target attitude
    inertial3DData = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DData)
    inertial3DWrap.ModelTag = "inertial_3D2"
    inertial3DData.sigma_R0N = [1.0, 0.0, 0.0]
    scSim.AddModelToTask(fswTaskName, inertial3DWrap, inertial3DData, 11)

    # thrusterConfigMsg
    fswSetupThrusters.clearSetup()
    for key, th in thFactory2.thrusterList.items():
        loc_B_tmp = list(itertools.chain.from_iterable(th.thrLoc_B))
        dir_B_tmp = list(itertools.chain.from_iterable(th.thrDir_B))
        fswSetupThrusters.create(loc_B_tmp, dir_B_tmp, th.MaxThrust)
    fswThrConfMsg = fswSetupThrusters.writeConfigMessage()

    # spacecraftReconfig
    spacecraftReconfigData = spacecraftReconfig.spacecraftReconfigConfig()
    spacecraftReconfigWrap = scSim.setModelDataWrap(spacecraftReconfigData)
    spacecraftReconfigWrap.ModelTag = "spacecraftReconfig"
    spacecraftReconfigData.chiefTransInMsg.subscribeTo(simpleNavObject.transOutMsg)
    spacecraftReconfigData.deputyTransInMsg.subscribeTo(simpleNavObject2.transOutMsg)
    if useRefAttitude:
        spacecraftReconfigData.attRefInMsg.subscribeTo(inertial3DData.attRefOutMsg)
    spacecraftReconfigData.thrustConfigInMsg.subscribeTo(fswThrConfMsg)
    thrusterEffector2.cmdsInMsg.subscribeTo(spacecraftReconfigData.onTimeOutMsg)
    spacecraftReconfigData.scMassDeputy = scObject2.hub.mHub  # [kg]
    spacecraftReconfigData.mu = orbitalMotion.MU_EARTH*1e9  # [m^3/s^2]
    spacecraftReconfigData.attControlTime = 400  # [s]
    spacecraftReconfigData.targetClassicOED = [0.0000, 0.0001, 0.0002, -0.0001, -0.0002, -0.0003]
    scSim.AddModelToTask(fswTaskName, spacecraftReconfigWrap, spacecraftReconfigData, 10)

    # att_Error
    attErrorData = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorData)
    attErrorWrap.ModelTag = "attError"
    scSim.AddModelToTask(fswTaskName, attErrorWrap, attErrorData, 9)
    attErrorData.attRefInMsg.subscribeTo(spacecraftReconfigData.attRefOutMsg)
    attErrorData.attNavInMsg.subscribeTo(simpleNavObject2.attOutMsg)

    # VehicleConfigFswMsg
    vehicleConfigOut2 = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut2.ISCPntB_B = I
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut2)

    # MRP_FeedBack
    mrpControlData = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlData)
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(fswTaskName, mrpControlWrap, mrpControlData, 8)
    mrpControlData.guidInMsg.subscribeTo(attErrorData.attGuidOutMsg)
    mrpControlData.vehConfigInMsg.subscribeTo(vcMsg)
    extFTObject2.cmdTorqueInMsg.subscribeTo(mrpControlData.cmdTorqueOutMsg)
    mrpControlData.K = 10
    mrpControlData.Ki = 0.0002
    mrpControlData.P = 50.0
    mrpControlData.integralLimit = 2. / mrpControlData.Ki * 0.1

    # ----- Setup spacecraft initial states ----- #
    mu = gravFactory.gravBodies['earth'].mu
    oe = orbitalMotion.ClassicElements()
    oe.a = 11000*1e3  # meters
    oe.e = 0.4
    oe.i = 60.0 * macros.D2R
    oe.Omega = 90 * macros.D2R
    oe.omega = 60 * macros.D2R
    M = 40 * macros.D2R
    E = orbitalMotion.M2E(M, oe.e)
    oe.f = orbitalMotion.E2f(E, oe.e)
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    orbitalMotion.rv2elem(mu, rN, vN)
    scObject.hub.r_CN_NInit = rN  # m
    scObject.hub.v_CN_NInit = vN  # m/s
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    oe2 = oe
    oe2.a = (1 + 0.0003)*oe2.a
    oe2.e = oe2.e - 0.0002
    oe2.i = oe2.i + 0.0001
    oe2.Omega = oe2.Omega + 0.0004
    oe2.omega = oe2.omega - 0.0001
    M2 = M + 0.0002
    E2 = orbitalMotion.M2E(M2, oe.e)
    oe2.f = orbitalMotion.E2f(E2, oe.e)
    rN2, vN2 = orbitalMotion.elem2rv(mu, oe2)
    scObject2.hub.r_CN_NInit = rN2  # m
    scObject2.hub.v_CN_NInit = vN2  # m/s
    scObject2.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_BN_B
    scObject2.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    # ----- log ----- #
    orbit_period = 2*math.pi/math.sqrt(mu/oe.a**3)
    simulationTime = orbit_period*1.1
    simulationTime = macros.sec2nano(simulationTime)
    numDataPoints = 1000
    samplingTime = unitTestSupport.samplingTime(simulationTime, dynTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    dataLog2 = scObject2.scStateOutMsg.recorder(samplingTime)
    attRefLog = spacecraftReconfigData.attRefOutMsg.recorder(samplingTime)
    thrCmdLog = spacecraftReconfigData.onTimeOutMsg.recorder(samplingTime)
    attErrLog = attErrorData.attGuidOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTaskName, dataLog)
    scSim.AddModelToTask(dynTaskName, dataLog2)
    scSim.AddModelToTask(dynTaskName, attRefLog)
    scSim.AddModelToTask(dynTaskName, thrCmdLog)
    scSim.AddModelToTask(dynTaskName, attErrLog)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, [scObject, scObject2]
                                              # , saveFile=fileName
                                             )

    # ----- execute sim ----- #
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # ----- pull ----- #
    pos = dataLog.r_BN_N
    vel = dataLog.v_BN_N
    pos2 = dataLog2.r_BN_N
    vel2 = dataLog2.v_BN_N
    attErr = attErrLog.sigma_BR
    timeData = dataLog.times()*macros.NANO2SEC/orbit_period

    # ----- plot ----- #
    # classic orbital element difference (figure1)
    plt.figure(1)
    oed = np.empty((len(pos[:, 0]), 6))
    for i in range(0, len(pos[:, 0])):
        oe_tmp = orbitalMotion.rv2elem(mu, pos[i], vel[i])
        oe2_tmp = orbitalMotion.rv2elem(mu, pos2[i], vel2[i])
        oed[i, 0] = (oe2_tmp.a - oe_tmp.a)/oe_tmp.a
        oed[i, 1] = oe2_tmp.e - oe_tmp.e
        oed[i, 2] = oe2_tmp.i - oe_tmp.i
        oed[i, 3] = oe2_tmp.Omega - oe_tmp.Omega
        oed[i, 4] = oe2_tmp.omega - oe_tmp.omega
        E_tmp = orbitalMotion.f2E(oe_tmp.f, oe_tmp.e)
        E2_tmp = orbitalMotion.f2E(oe2_tmp.f, oe2_tmp.e)
        oed[i, 5] = orbitalMotion.E2M(E2_tmp, oe2_tmp.e) - orbitalMotion.E2M(E_tmp, oe_tmp.e)
        for j in range(3, 6):
            if(oed[i, j] > math.pi):
                oed[i, j] = oed[i, j] - 2*math.pi
            if(oed[i, j] < -math.pi):
                oed[i, j] = oed[i, j] + 2*math.pi
    plt.plot(timeData, oed[:, 0], label="da")
    plt.plot(timeData, oed[:, 1], label="de")
    plt.plot(timeData, oed[:, 2], label="di")
    plt.plot(timeData, oed[:, 3], label="dOmega")
    plt.plot(timeData, oed[:, 4], label="domega")
    plt.plot(timeData, oed[:, 5], label="dM")
    plt.legend()
    plt.xlabel("time [orbit]")
    plt.ylabel("orbital element difference")
    figureList = {}
    pltName = fileName + "1" + str(int(useRefAttitude))
    figureList[pltName] = plt.figure(1)
    # attitude control error (figure2)
    plt.figure(2)
    plt.plot(timeData, attErr[:, 0])
    plt.plot(timeData, attErr[:, 1])
    plt.plot(timeData, attErr[:, 2])
    plt.xlabel("time [orbit]")
    plt.ylabel("MRP Error")
    pltName = fileName + "2" + str(int(useRefAttitude))
    figureList[pltName] = plt.figure(2)
    if(show_plots):
        plt.show()
    plt.close("all")

    return pos, vel, pos2, vel2, attErr, numDataPoints, figureList

if __name__ == "__main__":
    run(
        show_plots = True,  # show_plots
        useRefAttitude = False  # useRefAttitude
    )
