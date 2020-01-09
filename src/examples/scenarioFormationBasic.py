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

Demonstrates a basic method to simulate 3 satellites with 6-DOF motion and how to visualize the simlation
data in :ref:`Vizard <vizard>`.  One satellite is a 3-axis attitude controlled
satellite, while the second satellite is a tumbling space debris object.  The controlled satellite simulation components
are taken from :ref:`scenarioAttitudeFeedbackRW`. The purpose of this script is to show an explicit method to
setup multiple satellites, and also show how to store the Basilisk simulation data to be able to visualize
both satellite's motions within the :ref:`Vizard <vizard>` application.

The script is found in the folder ``src/examples`` and executed by using::

      python3 scenarioFormationBasic.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the servicer spacecraft and associated the Flight Software (FSW) algorithm
modules, as well as the first debris object that has 2 free-spinning RWs, and another debris object that
is an inert rigid body.

.. image:: /_images/static/test_scenarioFormationBasic.svg
   :align: center

When the simulation completes several plots are shown for the servicer MRP attitude history, the rate
tracking errors, the RW motor torque components, as well as the RW wheel speeds.

The simulation setups the spacecraft with 3 RW devices similar to :ref:`scenarioAttitudeFeedbackRW`.  One difference
is that here :ref:`hillPointing` is used to align the spacecraft with the Hill frame.  The two debris objects are
in a lead-follower configuration with the servicer and are 20m and 40m ahead respectively.

This simulation scripts illustrates how to use the :ref:`vizSupport` methods to record the simulation data such
that it can be viewed in the Vizard visualization.  Two methods of setting up the :ref:`vizInterface` module are shown.
If ``useMsgNameDefaults`` is set to true, then a default naming scheme is used to label the spacecraft state messages.
The first spacecraft, i.e. the servicer in this script, uses the default spacecraft state output message.
The following spacecraft have a number 2, 3, etc. attached to the default names. The script also illustrates
how to specify that each space object has a different number of RW devices.

If ``useMsgNameDefaults`` is set to false, then the simulation script illustrates how to manually configure
the spacecraft Data list of :ref:`vizInterface`.  This method is more verbose, but also allows to alternate
spacecraft state message naming schemes.



Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useMsgNameDefaults = True

Note that in the RW motor torque plot both the required control torque :math:`\hat u_B` and the true
motor torque :math:`u_B` are shown.  This illustrates that with this maneuver the RW devices are being
saturated, and the attitude still eventually stabilizes.

.. image:: /_images/Scenarios/scenarioFormationBasic1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFormationBasic2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFormationBasic3.svg
   :align: center


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing a servicer (3-axis attitude controlled) and a tumbling debris object.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 29, 2019
#

import numpy as np
import os
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import (MRP_Feedback, attTrackingError, fswMessages,
                                    rwMotorTorque, hillPoint)
from Basilisk.simulation import reactionWheelStateEffector, simple_nav, spacecraftPlus
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)
import copy
try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

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

def plot_rw_cmd_torque(timeData, dataUsReq, numRW):
    """Plot the RW command torques."""
    plt.figure(2)
    for idx in range(1, 4):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

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

def plot_rate_error(timeData, dataOmegaBR):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(3)
    for idx in range(1, 4):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s) ')

def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    """Plot the RW spin rates."""
    plt.figure(4)
    for idx in range(1, numRW + 1):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')


def run(show_plots, useMsgNameDefaults):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useMsgNameDefaults (bool): Specify if default message naming is used for the additional space objects

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

    # initialize servicer spacecraft object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "Servicer"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # create the debris object states
    scObject2 = spacecraftPlus.SpacecraftPlus()
    scObject2.ModelTag = "Debris"
    I2 = [600., 0., 0.,
          0., 650., 0.,
          0., 0, 450.]
    scObject2.hub.mHub = 350.0  # kg
    scObject2.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I2)

    # make another debris object */
    scObject3 = spacecraftPlus.SpacecraftPlus()
    scObject3.ModelTag = "DebrisSat"
    I3 = [600., 0., 0.,
          0., 650., 0.,
          0., 0, 450.]
    scObject3.hub.mHub = 350.0  # kg
    scObject3.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I3)

    # the debris object must have distinct output msg names from the servicer object
    # here the default names are augmented to make them unique
    if useMsgNameDefaults:
        scObject2.scStateOutMsgName = scObject.scStateOutMsgName + "2"
        scObject2.scMassStateOutMsgName = scObject.scMassStateOutMsgName + "2"
        scObject3.scStateOutMsgName = scObject.scStateOutMsgName + "3"
        scObject3.scMassStateOutMsgName = scObject.scMassStateOutMsgName + "3"
    else:
        scObject2.scStateOutMsgName = scObject.scStateOutMsgName + "Deputy"
        scObject2.scMassStateOutMsgName = scObject.scMassStateOutMsgName + "Deputy"
        scObject3.scStateOutMsgName = scObject.scStateOutMsgName + "Deputy2"
        scObject3.scMassStateOutMsgName = scObject.scMassStateOutMsgName + "Deputy2"



    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, None, 1)
    scSim.AddModelToTask(simTaskName, scObject2, None, 2)
    scSim.AddModelToTask(simTaskName, scObject3, None, 3)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObject2.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObject3.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

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
    # make sure the input and output names are unique to this spacecraft
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.InputCmds = scObject.ModelTag + "reactionwheel_cmds"
    rwStateEffector.OutputDataString = scObject.ModelTag + "_reactionwheel_output_states"

    if useMsgNameDefaults:
        rwPrefix = scObject.ModelTag
    else:
        rwPrefix = "chiefRW"
    rwFactory.addToSpacecraft(rwPrefix, rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(simTaskName, rwStateEffector, None, 4)


    # add free-spinning RWs to the debris object
    rwFactory2 = simIncludeRW.rwFactory()
    rwFactory2.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=1000.0)
    rwFactory2.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=-1000.0)
    numRW2 = rwFactory2.getNumOfDevices()
    rwStateEffector2 = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector2.InputCmds = ""  # empty string means there is no input command msg
    if useMsgNameDefaults:
        # use the sc name as default
        rw2Prefix = scObject2.ModelTag
    else:
        # provide a custom RW cluster name
        rw2Prefix = "debrisRW"
    rwFactory2.addToSpacecraft(rw2Prefix, rwStateEffector2, scObject2)
    scSim.AddModelToTask(simTaskName, rwStateEffector2, None, 5)


    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simple_nav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup hillPoint guidance module
    attGuidanceConfig = hillPoint.hillPointConfig()
    attGuidanceWrap = scSim.setModelDataWrap(attGuidanceConfig)
    attGuidanceWrap.ModelTag = "hillPoint"
    attGuidanceConfig.inputNavDataName = sNavObject.outputTransName
    # if you want to set attGuidanceConfig.inputCelMessName, then you need a planet ephemeris message of
    # type EphemerisIntMsg.  In the line below a non-existing message name is used to create an empty planet
    # ephemeris message which puts the earth at (0,0,0) origin with zero speed.
    attGuidanceConfig.inputCelMessName = "empty_earth_msg"
    attGuidanceConfig.outputDataName = "guidanceOut"
    scSim.AddModelToTask(simTaskName, attGuidanceWrap, attGuidanceConfig)

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    attErrorConfig.sigma_R0R = [0.414214, 0.0, 0.0]     # point the 3rd body axis in the along-track direction
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.outputDataName = "attErrorInertial3DMsg"
    attErrorConfig.inputRefName = attGuidanceConfig.outputDataName
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
    rwOutName = [rwPrefix + "_rw_config_0_data",
                 rwPrefix + "_rw_config_1_data",
                 rwPrefix + "_rw_config_2_data"]
    rw2OutName = [rw2Prefix + "_rw_config_0_data",
                  rw2Prefix + "_rw_config_1_data"]

    # A message is created that stores an array of the \Omega wheel speeds.  This is logged
    # here to be plotted later on.  However, RW specific messages are also being created which
    # contain a wealth of information.  Their default naming is automated and shown above.  This
    # allows us to log RW specific information such as the actual RW motor torque being applied.
    for item in rwOutName:
        scSim.TotalSim.logThisMessage(item, samplingTime)
    for item in rw2OutName:
        scSim.TotalSim.logThisMessage(item, samplingTime)


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

    # Two options are shown to setup the FSW RW configuration message.
    # First caseL: The FSW RW configuration message
    # uses the same RW states in the FSW algorithm as in the simulation.  In the following code
    # the fswSetupRW helper functions are used to individually add the RW states.  The benefit of this
    # method of the second method below is that it is easy to vary the FSW parameters slightly from the
    # simulation parameters.  In this script the second method is used, while the fist method is included
    # in a commented form to both options.
    # fswSetupRW.clearSetup()
    # for key, rw in rwFactory.rwList.items():
    #     fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    # fswSetupRW.writeConfigMessage(mrpControlConfig.rwParamsInMsgName, scSim.TotalSim, simProcessName)

    # Second case: If the exact same RW configuration states are to be used by the simulation and fsw, then the
    # following helper function is convenient to extract the fsw RW configuration message from the
    # rwFactory setup earlier.
    fswRwMsg = rwFactory.getConfigMessage()
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               mrpControlConfig.rwParamsInMsgName,
                               fswRwMsg)

    #
    #   set initial Spacecraft States
    #
    # setup the servicer orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.0
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    # setup 1st debris object states
    oe2 = copy.deepcopy(oe)
    oe2.f += 20./oe2.a
    r2N, v2N = orbitalMotion.elem2rv(mu, oe2)
    scObject2.hub.r_CN_NInit = r2N  # m   - r_CN_N
    scObject2.hub.v_CN_NInit = v2N  # m/s - v_CN_N
    scObject2.hub.sigma_BNInit = [[0.3], [0.1], [0.2]]  # sigma_CN_B
    scObject2.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # setup 2nd debris object states
    oe3 = copy.deepcopy(oe)
    oe3.f += 40./oe3.a
    r3N, v3N = orbitalMotion.elem2rv(mu, oe3)
    scObject3.hub.r_CN_NInit = r3N  # m   - r_CN_N
    scObject3.hub.v_CN_NInit = v3N  # m/s - v_CN_N
    scObject3.hub.sigma_BNInit = [[0.0], [-0.1], [0.2]]  # sigma_CN_B
    scObject3.hub.omega_BN_BInit = [[0.01], [-0.03], [-0.03]]  # rad/s - omega_CN_B


    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory,
                                              numRW=[numRW, numRW2, 0],
                                              # saveFile=fileName,
                                              scName=[scObject.ModelTag, scObject2.ModelTag, scObject3.ModelTag])

    # here the message are manually being set.  This allows for more customization
    if vizFound and not useMsgNameDefaults:
        # delete any existing list of vizInterface spacecraft data
        viz.scData.clear()

        # create a chief spacecraft info container
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject.ModelTag
        scData.numRW = numRW
        scData.scPlusInMsgName = "inertial_state_output"
        # the following command is required as we are deviating from the default naming of using the Model.Tag
        scData.rwInMsgName = vizInterface.StringVector([rwPrefix + "_rw_config_0_data",
                                                        rwPrefix + "_rw_config_1_data",
                                                        rwPrefix + "_rw_config_2_data"])
        viz.scData.push_back(scData)

        # create debris object info container
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject2.ModelTag
        scData.numRW = numRW2
        scData.scPlusInMsgName = "inertial_state_outputDeputy"
        # the following command is required as we are deviating from the default naming of using the Model.Tag
        scData.rwInMsgName = vizInterface.StringVector([rw2Prefix + "_rw_config_0_data",
                                                        rw2Prefix + "_rw_config_1_data"])
        viz.scData.push_back(scData)

        # create next debris object info container
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject3.ModelTag
        scData.numRW = 0
        scData.scPlusInMsgName = "inertial_state_outputDeputy2"
        viz.scData.push_back(scData)

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
    dataOmegaRW = scSim.pullMessageLogData(mrpControlConfig.inputRWSpeedsName + ".wheelSpeeds", list(range(numRW)))
    dataRW = []
    for i in range(0, numRW):
        dataRW.append(scSim.pullMessageLogData(rwOutName[i] + ".u_current", list(range(1))))
    np.set_printoptions(precision=16)
    omegaRW2 = []
    for i in range(0, numRW2):
        omegaRW2.append(scSim.pullMessageLogData(rw2OutName[i] + ".Omega", list(range(1))))

    #
    #   plot the results
    #
    timeData = dataUsReq[:, 0] * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_attitude_error(timeData, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_rate_error(timeData, dataOmegaBR)
    plot_rw_speeds(timeData, dataOmegaRW, numRW)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(4)

    plt.figure(5)
    for idx in range(1, 3):
        plt.plot(timeData, omegaRW2[idx - 1][:, 1]*60/(2*3.14159),
                 color=unitTestSupport.getLineColor(idx, numRW2),
                 label=r'$\Omega_{s,' + str(idx) + '}$')
    plt.xlabel('Time [min]')
    plt.ylabel('RW2 Omega (rpm)')


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
        False  # useMsgNameDefaults
    )
