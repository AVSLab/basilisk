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

Illustrates how to add a :ref:`PowerRW` to the simulation to track the RW power usages.  Further,
a the RW power modules are connected to a battery to illustrate the energy usage during this maneuver.
This script expands on :ref:`scenarioAttitudeFeedbackRW`.

The script is found in the folder ``src/examples`` and executed by using::

    python3 scenarioAttitudeFeedbackRWPower.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.  The 3 separate :ref:`PowerRW` modules are created to model the RW power requirements.
For more examples on using the RW power module see :ref:`test_unitPowerRW`.  Next, a battery module is created
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

import numpy as np
import os
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import (MRP_Feedback, attTrackingError, fswMessages,
                                    inertial3D, rwMotorTorque)
from Basilisk.simulation import reactionWheelStateEffector, simple_nav, spacecraftPlus
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)
from Basilisk.simulation import ReactionWheelPower
from Basilisk.simulation import simpleBattery

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
def plot_attitude_error(timeData, dataSigmaBR):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(1, 4):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')

def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW):
    """Plot the RW actual motor torques."""
    plt.figure(2)
    for idx in range(1, 4):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
        plt.plot(timeData, dataRW[idx - 1][:, 1],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

def plot_rw_power(timeData, dataRwPower, numRW):
    """Plot the RW actual motor torques."""
    plt.figure(3)
    for idx in range(1, 4):
        plt.plot(timeData, dataRwPower[idx - 1][:, 1],
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

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, None, 1)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = rwFactory.BalancedWheels

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

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.InputCmds = "reactionwheel_cmds"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(simTaskName, rwStateEffector, None, 2)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simple_nav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    # add RW power modules
    rwPowerList = []
    for c in range(0, numRW):
        powerRW = ReactionWheelPower.ReactionWheelPower()
        powerRW.ModelTag = scObject.ModelTag
        powerRW.basePowerNeed = 5.   # baseline power draw, Watts
        powerRW.rwStateInMsgName = powerRW.ModelTag + "_rw_config_" + str(c) + "_data"
        powerRW.nodePowerOutMsgName = "rwPower_" + str(c)
        if useRwPowerGeneration:
            powerRW.mechToElecEfficiency = 0.5
        scSim.AddModelToTask(simTaskName, powerRW)
        rwPowerList.append(powerRW)


    # create battery module
    battery = simpleBattery.SimpleBattery()
    battery.ModelTag = scObject.ModelTag
    battery.batPowerOutMsgName = 'battery_status'
    battery.storageCapacity = 300000  # W-s
    battery.storedCharge_Init = battery.storageCapacity * 0.8  # 20% depletion
    scSim.AddModelToTask(simTaskName, battery)
    # connect RW power to the battery module
    for c in range(0, numRW):
        battery.addPowerNodeToModel(rwPowerList[c].nodePowerOutMsgName)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation
    inertial3DConfig.outputDataName = "guidanceInertial3D"

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.outputDataName = "attErrorInertial3DMsg"
    attErrorConfig.inputRefName = inertial3DConfig.outputDataName
    attErrorConfig.inputNavName = sNavObject.outputAttName

    # setup the MRP Feedback control module
    mrpControlConfig = MRP_Feedback.MRP_FeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.inputGuidName = attErrorConfig.outputDataName
    mrpControlConfig.vehConfigInMsgName = "vehicleConfigName"
    mrpControlConfig.outputDataName = "LrRequested"
    mrpControlConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    mrpControlConfig.inputRWSpeedsName = rwStateEffector.OutputDataString
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
    rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
    rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueWrap, rwMotorTorqueConfig)
    # Initialize the test module msg names
    rwMotorTorqueConfig.outputDataName = rwStateEffector.InputCmds
    rwMotorTorqueConfig.inputVehControlName = mrpControlConfig.outputDataName
    rwMotorTorqueConfig.rwParamsInMsgName = mrpControlConfig.rwParamsInMsgName
    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    rwMotorTorqueConfig.controlAxes_B = controlAxes_B

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime // (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(rwMotorTorqueConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)
    # To log the RW information, the following code is used:
    scSim.TotalSim.logThisMessage(mrpControlConfig.inputRWSpeedsName, samplingTime)
    rwOutName = [scObject.ModelTag + "_rw_config_0_data",
                 scObject.ModelTag + "_rw_config_1_data",
                 scObject.ModelTag + "_rw_config_2_data"]
    for item in rwOutName:
        scSim.TotalSim.logThisMessage(item, samplingTime)
    for c in range(0,numRW):
        scSim.TotalSim.logThisMessage(rwPowerList[c].nodePowerOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(battery.batPowerOutMsgName, samplingTime)

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               mrpControlConfig.vehConfigInMsgName,
                               vehicleConfigOut)

    # make the FSW RW configuration message
    fswRwMsg = rwFactory.getConfigMessage()
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               mrpControlConfig.rwParamsInMsgName,
                               fswRwMsg)

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
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory, saveFile=fileName, numRW=numRW)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataUsReq = scSim.pullMessageLogData(rwMotorTorqueConfig.outputDataName + ".motorTorque", list(range(numRW)))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".sigma_BR", list(range(3)))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".omega_BR_B", list(range(3)))
    dataPos = scSim.pullMessageLogData(sNavObject.outputTransName + ".r_BN_N", list(range(3)))
    dataOmegaRW = scSim.pullMessageLogData(mrpControlConfig.inputRWSpeedsName + ".wheelSpeeds", list(range(numRW)))
    dataRW = []
    dataRwPower = []
    for i in range(0, numRW):
        dataRW.append(scSim.pullMessageLogData(rwOutName[i] + ".u_current", list(range(1))))
        dataRwPower.append(scSim.pullMessageLogData(rwPowerList[i].nodePowerOutMsgName + ".netPower"))
    batteryStorageLog = scSim.pullMessageLogData(battery.batPowerOutMsgName+".storageLevel")

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeData = dataUsReq[:, 0] * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    figureList = {}
    plot_attitude_error(timeData, dataSigmaBR)

    plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)

    plot_rw_power(timeData, dataRwPower, numRW)
    pltName = fileName + "3" + str(useRwPowerGeneration)
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    plt.plot(timeData, batteryStorageLog[:, 1])
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
