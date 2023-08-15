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

Demonstrates how to simulate an attitude control scenario without having any gravity
bodies present. In essence, the 3-U cube-sat spacecraft is hovering in deep space.  The goal is to
stabilize the tumble of a spacecraft and point it towards a fixed inertial direction.
This script sets up a 6-DOF spacecraft which is hovering in deep space.  The scenario duplicates
the scenario in :ref:`scenarioAttitudeFeedback` without adding
Earth gravity body to the simulation.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioAttitudeFeedbackNoEarth.py

When the simulation completes 3 plots are shown for the MRP attitude history, the rate
tracking errors, as well as the control torque vector.

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenarioAttitudeFeedbackNoEarth.svg
   :align: center

The spacecraft simulation is identical to :ref:`scenarioAttitudeFeedback`.
The primary difference is that the gravity body is not included.


Illustration of Simulation Results
----------------------------------
The following simulation runs should output identical results to the scenario runs in
:ref:`scenarioAttitudeFeedback` as the attitude pointing goal
is an inertial orientation, and thus independent of the satellite being in Earth orbit or
hovering in deep space.

::

    show_plots = True, useUnmodeledTorque = False, useIntGain = False, useKnownTorque = False

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackNoEarth1000.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackNoEarth2000.svg
   :align: center

::

    show_plots = True, useUnmodeledTorque = True, useIntGain = False, useKnownTorque = False

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackNoEarth1100.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackNoEarth2100.svg
   :align: center

::

    show_plots = True, useUnmodeledTorque = True, useIntGain = True, useKnownTorque = False

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackNoEarth1110.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackNoEarth2110.svg
   :align: center

::

    show_plots = True, useUnmodeledTorque = True, useIntGain = False, useKnownTorque = True

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackNoEarth1101.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackNoEarth2101.svg
   :align: center


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav() and
#           mrpFeedback() modules.  Illustrates spacecraft attitude control in deep
#           space without a planet or gravity body setup.
# Author:   Hanspeter Schaub
# Creation Date:  Sept. 13, 2018
#
import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
# import message declarations
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import inertial3D
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])



def run(show_plots, useUnmodeledTorque, useIntGain, useKnownTorque):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useUnmodeledTorque (bool): Specify if an external torque should be included
        useIntGain (bool): Specify if the feedback control uses an integral feedback term
        useKnownTorque (bool): Specify if the external torque is feed forward in the contro

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    # use the input flag to determine which external torque should be applied
    # Note that all variables are initialized to zero.  Thus, not setting this
    # vector would leave it's components all zero for the simulation.
    if useUnmodeledTorque:
        extFTObject.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.K = 3.5
    if useIntGain:
        mrpControl.Ki = 0.0002  # make value negative to turn off integral feedback
    else:
        mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1
    if useKnownTorque:
        mrpControl.knownTorquePntB_B = [0.25, -0.25, 0.1]

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    mrpLog = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, mrpLog)

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    configDataMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # The primary difference is that the gravity body is not included.
    # When initializing the spacecraft states, only the attitude states must be set.  The position and velocity
    # states are initialized automatically to zero.
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , modelDictionaryKeyList="3USat"
                                              # , saveFile=fileName
                                              )
    #
    # connect the messages to the modules
    #
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(configDataMsg)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataLr = mrpLog.torqueRequestBody
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    timeAxis = attErrorLog.times()
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1" + str(int(useUnmodeledTorque)) + str(int(useIntGain))+ str(int(useKnownTorque))
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    pltName = fileName + "2" + str(int(useUnmodeledTorque)) + str(int(useIntGain)) + str(int(useKnownTorque))
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False,  # useUnmodeledTorque
        False,  # useIntGain
        False  # useKnownTorque
    )
