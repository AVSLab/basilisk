#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This script showcases how to create a Numba Basilisk module.

Demonstrates how to stabilize the attitude tumble without translational motion.
This script sets up a 6-DOF spacecraft, but without specifying any orbital motion.  Thus,
this scenario simulates the spacecraft translating in deep space.  The scenario is a
version of :ref:`scenarioAttitudePointingPy` where the Python MRP PD control module is
replaced with an equivalent :ref:`numbaModules`-based implementation.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttitudePointingNumba.py

As with :ref:`scenarioAttitudePointingPy`, when the simulation completes 3 plots are
shown for the MRP attitude history, the rate tracking errors, as well as the control
torque vector.

The MRP PD control module in this script is a class called ``NumbaMRPPD``.  It subclasses
``NumbaModel`` and implements the entire control law inside ``UpdateStateImpl``, which is
JIT-compiled by Numba to native code.  Gains ``K`` and ``P`` are stored in the persistent
``memory`` namespace and can be configured before ``InitializeSimulation`` is called::

    numMRPPD = NumbaMRPPD()
    numMRPPD.ModelTag = "numMRP_PD"
    numMRPPD.memory.K = 3.5
    numMRPPD.memory.P = 30.0
    numMRPPD.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    scSim.AddModelToTask(simTaskName, numMRPPD)

Illustration of Simulation Results
-----------------------------------

::

    show_plots = True

Here a small initial tumble is simulated.  The
resulting attitude and control torque histories are shown below.  The spacecraft quickly
regains a stable orientation without tumbling past 180 degrees.

.. image:: /_images/Scenarios/scenarioAttitudePointingNumba1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudePointingNumba2.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test showing how to set up and run a Numba BSK module with C/C++ modules
# Author:   AVS Lab
# Creation Date:  2026
#

import os
import time

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.architecture.numbaModel import NumbaModel
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    simTaskName = "simTask"
    simProcessName = "simProcess"

    scSim = SimulationBaseClass.SimBaseClass()

    simulationTime = macros.min2nano(10.)

    dynProcess = scSim.CreateNewProcess(simProcessName, 10)

    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]
    scSim.AddModelToTask(simTaskName, scObject)

    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]

    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # setup Numba MRP PD control module
    numMRPPD = NumbaMRPPD()
    numMRPPD.ModelTag = "numMRP_PD"
    numMRPPD.memory.K = 3.5
    numMRPPD.memory.P = 30.0
    numMRPPD.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    scSim.AddModelToTask(simTaskName, numMRPPD)

    numDataPoints = 50
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    mrpLog = numMRPPD.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, mrpLog)

    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(numMRPPD.cmdTorqueOutMsg)

    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)

    tic = time.time()
    scSim.ExecuteSimulation()
    toc = time.time()
    print("Run scenario in", toc-tic, "s")

    dataLr = mrpLog.torqueRequestBody
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    timeAxis = attErrorLog.times()
    np.set_printoptions(precision=16)

    plt.close("all")
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s]')

    if show_plots:
        plt.show()

    plt.close("all")

    return figureList


class NumbaMRPPD(NumbaModel):
    """MRP PD attitude controller implemented as a NumbaModel.

    The control law is::

        L_r = -(K * sigma_BR + P * omega_BR_B)

    Gains ``K`` and ``P`` are stored in the persistent ``memory`` namespace so they
    are accessible inside the JIT-compiled ``UpdateStateImpl``.
    """

    def __init__(self):
        super().__init__()
        self.guidInMsg = messaging.AttGuidMsgReader()
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        # Gains: configure before InitializeSimulation via module.memory.K = ...
        self.memory.K = 0.0
        self.memory.P = 0.0

    @staticmethod
    def UpdateStateImpl(guidInMsgPayload, cmdTorqueOutMsgPayload, memory):
        for i in range(3):
            lrCmd = (guidInMsgPayload.sigma_BR[i] * memory.K
                     + guidInMsgPayload.omega_BR_B[i] * memory.P)
            cmdTorqueOutMsgPayload.torqueRequestBody[i] = -lrCmd


#
# This statement below ensures that the unit test script can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    run(
        True  # show_plots
    )
