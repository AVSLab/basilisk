#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario illustrates how to add a RW temperature sensor and include some random noise
in the temperature measurement.  The RW-based attitude control component is based on
:ref:`scenarioAttitudeFeedbackRW`. This scenario uses the :ref:`motorThermal` module model
the true temperature data from reaction wheels and :ref:`tempMeasurement` module for
adding noise into the temperature readings.

.. caution::
    For the :ref:`tempMeasurement` module to provide random noise it is critical that the module
    variable ``RNGSeed`` is set to a unique value.  Setting it to the same value will result
    in the same random numbers being generated each run.


Illustration of Simulation Results
----------------------------------

::

    show_plots = True

The following plots illustrate the true temperature of the RWs and the measurement with noise.

.. image:: /_images/Scenarios/scenarioTempMeasurementAttitude1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTempMeasurementAttitude2.svg
   :align: center


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  This scenario illustrates the use of tempMeasurement module and generating random noise in the measurement.
# Author: Yumeka Nagano
# Creation Date:  May 10, 2024
#

import os

import matplotlib.pyplot as plt
import numpy as np
import time
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    inertial3D, rwMotorTorque)
from Basilisk.simulation import (reactionWheelStateEffector, simpleNav,
                                 spacecraft, motorThermal, tempMeasurement)
from Basilisk.utilities import (SimulationBaseClass, fswSetupRW, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
def plot_rw_temperature(timeData, dataTemp, numRW, id=None):
    """Plot the reaction wheel temperatures"""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataTemp[:,idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$T_{rw,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Temperatures [ºC]')
    return

def plot_rw_temp_measurement(timeData, dataTemp, numRW, id=None):
    """Plot the reaction wheel temperature measurements"""
    plt.figure(id)
    for idx in range(numRW):
        plt.plot(timeData, dataTemp[:,idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$T_{rw,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Temp Measurements [ºC]')
    return

def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
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
    # In this simulation the RW objects RW1, RW2 or RW3 are not modified further.  However, you can over-ride
    # any values generate in the `.create()` process using for example RW1.Omega_max = 100. to change the
    # maximum wheel speed.

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process.  This is required for the UpdateState() method
    # to be called which logs the RW states
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)

    # create temperature lists for each RW and create objects for each
    rwTempList = []
    tempMeasList = []
    for item in range(numRW):
        rwTempList.append(motorThermal.MotorThermal())
        tempMeasList.append(tempMeasurement.TempMeasurement())

        # initialize the RW temperature
        rwTempList[item].ModelTag = "rwThermals" + str(item)
        rwTempList[item].currentTemperature = 20  # [ºC]
        rwTempList[item].ambientTemperature = 20  # [ºC]
        rwTempList[item].efficiency = 0.7
        rwTempList[item].ambientThermalResistance = 5  # Air Thermal Resistance
        rwTempList[item].motorHeatCapacity = 50  # Motor (steel) Heat Capacity

        # initialize the temperature measurement
        tempMeasList[item].ModelTag = "tempMeasurementModel" + str(item)
        tempMeasList[item].senBias = 0.0 # [C] bias amount
        tempMeasList[item].senNoiseStd = 0.5  # [C] noise standard deviation
        tempMeasList[item].walkBounds = 0.1  # [C] noise wald bounds
        tempMeasList[item].stuckValue = 0.0  # [C] if the sensor gets stuck, stuck at 10 degrees C
        tempMeasList[item].spikeProbability = 0.0  # [-] 30% chance of spiking at each time step
        tempMeasList[item].spikeAmount = 0.0  # [-] 10x the actual sensed value if the spike happens
        tempMeasList[item].faultState = tempMeasurement.TEMP_FAULT_NOMINAL
        # tempMeasList[item].RNGSeed = 123 # Seed number (same seed)
        tempMeasList[item].RNGSeed = time.time_ns() % (2**32)  # Seed number (random for every run)

        # add RW temperature and measurement object array to the simulation process
        scSim.AddModelToTask(simTaskName, rwTempList[item], 2)
        scSim.AddModelToTask(simTaskName, tempMeasList[item], 2)

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
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)

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

    # A message is created that stores an array of the true temperature and temperature measurement. 
    # This is logged here to be plotted later on.
    rwTempLogs = []
    tempMeasLogs = []
    for item in range(numRW):
        rwTempLogs.append(rwTempList[item].temperatureOutMsg.recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, rwTempLogs[item])
        
        tempMeasLogs.append(tempMeasList[item].tempOutMsg.recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, tempMeasLogs[item])

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # set up the FSW RW configuration message.
    fswSetupRW.clearSetup()
    for key, rw in rwFactory.rwList.items():
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    fswRwParamMsg = fswSetupRW.writeConfigMessage()

    #
    #   set initial Spacecraft States
    #
    # set up the orbit using classical orbit elements
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

    # link messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

    for item in range(numRW):
        rwTempList[item].rwStateInMsg.subscribeTo(rwStateEffector.rwOutMsgs[item])
        tempMeasList[item].tempInMsg.subscribeTo(rwTempList[item].temperatureOutMsg)

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
    
    dataRWTemperature = []
    dataTempMeasurement = []
    for i in range(numRW):
        dataRWTemperature.append(rwTempLogs[i].temperature)
        dataTempMeasurement.append(tempMeasLogs[i].temperature)

    dataRWTemperature = np.array(dataRWTemperature).T
    dataTempMeasurement = np.array(dataTempMeasurement).T

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeData = rwTempLogs[0].times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    figureList = {}
    plot_rw_temperature(timeData, dataRWTemperature, numRW)
    pltName = fileName + "1" 
    figureList[pltName] = plt.figure(1)

    plot_rw_temp_measurement(timeData, dataTempMeasurement, numRW)
    pltName = fileName + "2" 
    figureList[pltName] = plt.figure(2)
    
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
