#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/MT8Z_86-B4M" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

This scenario demonstrates how to layer attitude references. It starts with an inertial pointing attitude and then
adds a spiral scanning motion on top of it.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioInertialSpiral.py

The initial orientation is set to zero for all MRPs and rates, and the desired attitude is layered on top using
:ref:`eulerRotation`, which allows 3-2-1 Euler rotations to be specified. In this script, the first rotation
is 0.02 radians per second about the 1-axis and the second is 0.0001 radians per second about the 2-axis. This
creates an outward sweeping spiral motion. The inertial pointing module feeds its output message into the first
rotation, which feeds into the second, which in turn gives the attitude error that is used by the control law.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

Five plots are shown. The first three show the attitude error, control torque, and rate tracking error,
showing a transient response as the initial condition does not match the desired rates.

.. image:: /_images/Scenarios/scenarioInertialSpiral1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioInertialSpiral2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioInertialSpiral3.svg
   :align: center

The final two show the logged MRP data, converted back into Euler angles. The pitch and yaw and plotted
against time as well as against each other to show the resulting spiral.

.. image:: /_images/Scenarios/scenarioInertialSpiral4.svg
   :align: center

.. image:: /_images/Scenarios/scenarioInertialSpiral5.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the attitude navigation modules with eulerRotation()
#           module to demonstrate layering attitude references.
# Author:   Galen Bascom
# Creation Date:  February 9, 2022
#

import os

import numpy as np

np.set_printoptions(precision=16)

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simpleNav

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import eulerRotation

# import message declarations
from Basilisk.architecture import messaging

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenario can be run with the followings parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Define simulation run time and integration time step
    simulationTime = macros.min2nano(10.)
    simulationTimeStep = macros.sec2nano(0.1)

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # se tup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    # set up extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   set up the FSW algorithm tasks
    #

    # set up inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # we create 2 dynamic attitude reference modules as we want to do a 1-2 Euler angle rotation
    # and the modules provide a 3-2-1 sequence.  Thus, we do a 0-0-1 321-rotation and then a 0-1-0 321-rotation
    # get a 1-2 result.
    attGuidanceEuler1 = eulerRotation.eulerRotation()
    attGuidanceEuler1.ModelTag = "eulerRotation1"
    attGuidanceEuler1.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    scSim.AddModelToTask(simTaskName, attGuidanceEuler1)
    # Make rotation 1 be 0.02 rad/s about 1 axis
    attGuidanceEuler1.angleRates = [0.0, 0.0, 0.02]

    attGuidanceEuler2 = eulerRotation.eulerRotation()
    attGuidanceEuler2.ModelTag = "eulerRotation2"
    attGuidanceEuler2.attRefInMsg.subscribeTo(attGuidanceEuler1.attRefOutMsg)
    scSim.AddModelToTask(simTaskName, attGuidanceEuler2)
    # Make rotation 2 be 0.0001 rad/s about 2 axis
    attGuidanceEuler2.angleRates = [0.0, 0.0001, 0.0]
    
    # set up the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attError"
    scSim.AddModelToTask(simTaskName, attError)

    # set up the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.K = 3.5
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    #
    # create simulation messages
    #
    configData = messaging.VehicleConfigMsgPayload()
    configData.ISCPntB_B = I
    configDataMsg = messaging.VehicleConfigMsg().write(configData)

    #
    # connect the messages to the modules
    #
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(attGuidanceEuler2.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(configDataMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)

    #
    # Set up data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    snLog = sNavObject.scStateInMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    mrpLog = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, snLog)
    scSim.AddModelToTask(simTaskName, snAttLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, mrpLog)

    #
    #   set initial Spacecraft States
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.], [0.], [0.]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.], [0.], [0.]]  # rad/s - omega_BN_B

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    dataSigmaBN = snAttLog.sigma_BN
    
    dataEulerAnglesPitch = list()
    dataEulerAnglesYaw = list()
    dataEulerAnglesRoll = list()
    for sigma in dataSigmaBN:
        eulerAngle = RigidBodyKinematics.MRP2Euler321(sigma)
        dataEulerAnglesPitch.append(eulerAngle[0])
        dataEulerAnglesYaw.append(eulerAngle[1])
        dataEulerAnglesRoll.append(eulerAngle[2])
        
    timeLineSet = attErrorLog.times() * macros.NANO2MIN
    #
    #   plot the results
    #
    timeAxis = attErrorLog.times()
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attErrorLog.sigma_BR[:, idx],
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
        plt.plot(timeAxis * macros.NANO2MIN, mrpLog.torqueRequestBody[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='upper right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Control Torque $L_r$ [Nm]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attErrorLog.omega_BR_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    plt.plot(timeLineSet, dataEulerAnglesYaw,
                 color=unitTestSupport.getLineColor(0, 3),label=r'Yaw')
    plt.plot(timeLineSet, dataEulerAnglesPitch,
                 color=unitTestSupport.getLineColor(1, 3),label=r'Pitch')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Euler Angles [rad]')
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plt.figure(5)
    plt.plot(dataEulerAnglesPitch, dataEulerAnglesYaw)
    plt.xlabel('Pitch [rad]')
    plt.ylabel('Yaw [rad]')
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

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
        True  # show_plots
    )
