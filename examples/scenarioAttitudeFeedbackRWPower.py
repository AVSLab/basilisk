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

Illustrates how to add a :ref:`ReactionWheelPower` to the simulation to track the RW power usages.  Further,
a the RW power modules are connected to a battery to illustrate the energy usage during this maneuver.
This script expands on :ref:`scenarioAttitudeFeedbackRW`.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttitudeFeedbackRWPower.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.  The 3 separate :ref:`ReactionWheelPower` instances are created to model the RW power requirements.
For more examples on using the RW power module see :ref:`test_unitReactionWheelPower`.
Next, a battery module is created
using :ref:`simpleBattery`.  All the RW power draw messages are connected to the battery to model the total
energy usage.

.. image:: /_images/static/test_scenarioAttitudeFeedbackRWPower.svg
   :align: center

Illustration of Simulation Results
----------------------------------
The first simulation scenario is run with ``useRwPowerGeneration = False`` to model RW devices which require
electrical power to accelerate and decelerate the fly wheels.  The attitude history should be the same
as in :ref:`scenarioAttitudeFeedbackRW`.  Shown below are the resulting RW power requirements, as well as the
time history of the battery state.

::

    show_plots = True, useRwPowerGeneration = False

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRWPower3False.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRWPower4False.svg
   :align: center

The next simulation allows 50% of the breaking power to be returned to the power system.  You can see
how this will reduce the overall maneuver energy requirements.

::

    show_plots = True, useRwPowerGeneration = True

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRWPower3True.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRWPower4True.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated scenario using a RW feedback control law where the RW devices power consumption
#           is modeled, as well as the battery drain.
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 26, 2020
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    inertial3D, rwMotorTorque)
from Basilisk.simulation import ReactionWheelPower
from Basilisk.simulation import reactionWheelStateEffector, simpleNav, spacecraft
from Basilisk.simulation import simpleBattery
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
def plot_attitude_error(timeData, dataSigmaBR):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')

def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW):
    """Plot the RW actual motor torques."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
        plt.plot(timeData, dataRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

def plot_rw_power(timeData, dataRwPower, numRW):
    """Plot the RW actual motor torques."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, dataRwPower[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$p_{rw,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Power (W)')


def run(show_plots, useRwPowerGeneration):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useRwPowerGeneration (bool): Specify if the RW power generation ability is being model when breaking

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
    scSim.AddModelToTask(simTaskName, scObject, 1)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = messaging.BalancedWheels

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    RW1 = rwFactory.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=100.  # RPM
                           , RWModel=varRWModel
                           )
    RW2 = rwFactory.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=200.  # RPM
                           , RWModel=varRWModel
                           )
    RW3 = rwFactory.create('Honeywell_HR16', [0, 0, 1], maxMomentum=50., Omega=300.  # RPM
                           , rWB_B=[0.5, 0.5, 0.5]  # meters
                           , RWModel=varRWModel
                           )
    rwList = [RW1, RW2, RW3]

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # add RW power modules
    rwPowerList = []
    for c in range(numRW):
        powerRW = ReactionWheelPower.ReactionWheelPower()
        powerRW.ModelTag = scObject.ModelTag + "RWPower" + str(c)
        powerRW.basePowerNeed = 5.   # baseline power draw, Watts
        powerRW.rwStateInMsg.subscribeTo(rwStateEffector.rwOutMsgs[c])
        if useRwPowerGeneration:
            powerRW.mechToElecEfficiency = 0.5
        scSim.AddModelToTask(simTaskName, powerRW)
        rwPowerList.append(powerRW)

    # create battery module
    battery = simpleBattery.SimpleBattery()
    battery.ModelTag = scObject.ModelTag
    battery.storageCapacity = 300000  # W-s
    battery.storedCharge_Init = battery.storageCapacity * 0.8  # 20% depletion
    scSim.AddModelToTask(simTaskName, battery)
    # connect RW power to the battery module
    for c in range(numRW):
        battery.addPowerNodeToModel(rwPowerList[c].nodePowerOutMsg)

    #
    #   setup the FSW algorithm tasks
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # make the FSW RW configuration message
    fswRwMsg = rwFactory.getConfigMessage()

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    mrpControl.K = 3.5
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)
    # Initialize the test module msg names
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    rwCmdLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrLog = attError.attGuidOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, rwCmdLog)
    scSim.AddModelToTask(simTaskName, attErrLog)

    # To log the RW information, the following code is used:
    rwSpeedLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, rwSpeedLog)
    rwOutLog = []
    rwPowLog = []
    for c in range(numRW):
        rwOutLog.append(rwStateEffector.rwOutMsgs[c].recorder(samplingTime))
        rwPowLog.append(rwPowerList[c].nodePowerOutMsg.recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, rwOutLog[-1])
        scSim.AddModelToTask(simTaskName, rwPowLog[-1])

    batPowLog = battery.batPowerOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, batPowLog)

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
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              # , saveFile=fileName
                                              , rwEffectorList=rwStateEffector
                                              )

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
    dataUsReq = rwCmdLog.motorTorque[:, range(numRW)]
    dataSigmaBR = attErrLog.sigma_BR

    dataRW = []
    dataRwPower = []
    for c in range(0, numRW):
        dataRW.append(rwOutLog[c].u_current)
        dataRwPower.append(rwPowLog[c].netPower)
    batteryStorageLog = batPowLog.storageLevel

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeData = rwCmdLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    figureList = {}
    plot_attitude_error(timeData, dataSigmaBR)

    plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)

    plot_rw_power(timeData, dataRwPower, numRW)
    pltName = fileName + "3" + str(useRwPowerGeneration)
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    plt.plot(timeData, batteryStorageLog)
    plt.xlabel('Time [min]')
    plt.ylabel('Battery Storage (Ws)')
    pltName = fileName + "4" + str(useRwPowerGeneration)
    figureList[pltName] = plt.figure(4)

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
        True  # useRwPowerGeneration
    )
