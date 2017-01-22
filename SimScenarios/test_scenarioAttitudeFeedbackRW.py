''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraftPlus(), RWs, simpleNav() and
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
#           while using the RWs to do the attitude control actuation.
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 7, 2017
#



import pytest
import sys, os, inspect
import matplotlib
import numpy as np
import ctypes
import math
import csv
import logging

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
# if this script is run from a custom folder outside of the Basilisk folder, then uncomment the
# following line and specify the absolute bath to the Basilisk folder
#bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')
# @endcond

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import orbitalMotion

# import simulation related support
import spacecraftPlus
import simIncludeGravity
import simIncludeRW
import simple_nav
import reactionWheelStateEffector

# import FSW Algorithm related support
import MRP_Feedback
import inertial3D
import attTrackingError
import rwMotorTorque
import vehicleConfigData
import fswSetupRW





# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useJitterSimple", [
      (False)
    # , (True)
])

# provide a unique test method name, starting with test_
def test_bskAttitudeFeedbackMRP(show_plots, useJitterSimple):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, useJitterSimple)
    assert testResults < 1, testMessage



## \defgroup Tutorials_2_3
##   @{
## Demonstrates how to use RWs to stabilize the tumble of a spacecraft orbiting the
# Earth.
#
# Attitude Detumbling Simulation using RW Effectors {#scenarioAttitudeFeedbackMRP}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The goal is to
# illustrate how Reaction Wheel (RW) state effector can be added to the rigid
# spacecraftPlus() hub, and what flight algorithm module is used to control these RWs.
#  The scenario is
# setup to be run in multiple setups:
# Setup | useJitterSimple
# ----- | -------------------
# 1     | False
#
# To run the default scenario 1., call the python script from a Terminal window through
#
#       python test_scenarioAttitudeFeedbackRW.py
#
# When the simulation completes several plots are shown for the MRP attitude history, the rate
# tracking errors, as well as the RW motor torque components, as well as the RW wheel speeds.
#
#
# ### Setup Changes to Simulate RW Dynamic Effectors
#
# The fundamental simulation setup is the same as the one used in
# [test_scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback).
# The dynamics simulation is setup using a SpacecraftPlus() module to which an Earth gravity
# effector is attached.  The simple navigation module is still used to output the inertial attitude
# , angular rate, as well as position and velocity message. The simulation is setup to run in a single
# process again.  If the flight algorithms and simulation tasks are to run at different rates, then see
# [test_scenarioAttitudeFeedback2T.py](@ref scenarioAttitudeFeedback2T) on how to setup a 2 thread simulation.
#
# For the spacecraft simulation side of this script, the new element is adding RW effectors to the
# the rigid spacecraft hub.  The support macro `simIncludeRW.py` provides several convenient tools to facilitate
# this RW setup process.  This script allows the user to readily create RWs from a database of
# public RW specifications, customize them if needed, and add them to the spacecraftPlus() module.
# The specific code required is:
# ~~~~~~~~~~~~~{.py}
#     # add RW devices
#     # The clearRWSetup() is critical if the script is to run multiple times
#     simIncludeRW.clearSetup()
#     # the Honeywell HR16 comes in three momentum configuration, 100, 75 and 50 Nms
#     simIncludeRW.options.maxMomentum = 50
#     # create each RW by specifying the RW type, the spin axis gsHat and the initial wheel speed Omega
#     simIncludeRW.create(
#             'Honeywell_HR16',
#             [1, 0, 0],              # gsHat_S
#             100.0                     # RPM
#             )
#     simIncludeRW.create(
#             'Honeywell_HR16',
#             [0, 1, 0],              # gsHat_S
#             200.0                     # RPM
#             )
#     simIncludeRW.create(
#             'Honeywell_HR16',
#             [0, 0, 1],              # gsHat_S
#             300.0,                    # RPM
#             [0.5,0.5,0.5]           # r_S (optional argument)
#             )
#     numRW = simIncludeRW.getNumOfDevices()
#
#     # create RW object container and tie to spacecraft object
#     rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
#     simIncludeRW.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)
#
#     # add RW object array to the simulation process
#     scSim.AddModelToTask(simTaskName, rwStateEffector, None, 2)
# ~~~~~~~~~~~~~
# The first task is to call the `clearRWSetup()` function.  the `simIncludeRW.py` macro file creates a local
# array of RW devices as each is created.  This `clearRWSetup()` call clears this list.  It is a good
# practice to always call this function prior to setting up RWs.  In particular, if this script is
# called several times through a Monte-Carlo setup, or a multi-objective `py.test` run, then this ensures
# that each simulation run begins with a blank list of RW devices.
#
# The next step is to use create() to include a particular RW devices.  The `simIncludeRW.py` file contains several
# public specifications of RW devices which can be accessed by specifying the wheel name, `Honeywell_HR16`
# in this case.  The other 2 arguments provide the spin axis \f$\hat{\mathbf g}_s\f$ and initial RW spin \f$\Omega\f$
# in RPMs.  The 3rd argument is optional, and specifies the body-relative location of the RW center of mass.
# This information is only used if an off-balanced RW device is being modeled.
#
# Each RW device has several default options that can be customized if needed.  For example,
# the `Honeywell_HR16` comes in three different momentum storage configurations.  Before calling the
# `create()` command, the desired storage capacity must be specified.  Other options include specifying
# friction models (all turned off by default) or the imbalance models used (default is a balanced wheel).
#
# The command `addToSpacecraft()` adds all the created RWs to the spacecraftPlus() module.  The final step
# is as always to add the vector of RW effectors (called `rwStateEffector` above) to the list of simulation
# tasks.  However, note that the dynamic effector should be evaluated before the spacecraftPlus() module,
# which is why it is being added with a higher priority than the `scObject` task.  Generally speaking
# we should have the execution order:
#
#       effectors -> dynamics -> sensors
#
# As with spacecraftPlus(), the reactionWheelStateEffector() class is a sub-class of the StateEffector() class.
# As such, the RW states that get integrated are setup in a state class, and must be specified after the
# simulation is initialized.  The `create()` macros above already stored what desired RW speeds
# were to be simulated.  The macro call
# ~~~~~~~~~~~~~~~~~{.py}
#     # initialize the RW state effector states
#     simIncludeRW.setInitialStates(scObject)
# ~~~~~~~~~~~~~~~~~
# loops through all the setup RW effectors and sets the initial wheel speeds to the desired values.
#
# To log the RW information, the following code is used:
# ~~~~~~~~~~~~~~~~~{.py}
#     scSim.TotalSim.logThisMessage(mrpControlConfig.inputRWSpeedsName, samplingTime)
#     rwOutName = ["rw_bla0_data", "rw_bla1_data", "rw_bla2_data"]
#     for item in rwOutName:
#         scSim.TotalSim.logThisMessage(item, samplingTime)
# ~~~~~~~~~~~~~~~~~
# A message is created that stores an array of the \f$\Omega\f$ wheel speeds.  This is logged
# here to be plotted later on.  However, RW specific messages are also being created which
# contain a wealth of information.  Their default naming is automated and shown above.  This
# allows us to log RW specific information such as the actual RW motor torque being applied.
#
#
# ### Flight Algorithm Changes to Control RWs
#
# The general flight algorithm setup is the same as in the earlier simulation script. Here we
# use again the inertial3D() guidance module, teh attTrackingError() module to evaluate the
# tracking error states, and the MRP_Feedback() module to provide the desired \f${\mathbf L}_r\f$
# control torque vector.  In this simulation we want the MRP attitude control module to take
# advantage of the RW spin information.  This is achieved by adding the 2 extra lines:
# ~~~~~~~~~~~~~~~{.py}
#     mrpControlConfig.rwParamsInMsgName = "rwa_config_data_parsed"
#     mrpControlConfig.inputRWSpeedsName = rwStateEffector.OutputDataString
# ~~~~~~~~~~~~~~~
# The first line specifies the RW configuration flight message name, and the second name
# connects the RW speed output message as an input message to this control module.  This simulates
# the RW speed information being sensed and returned to this algorithm.  This this message names
# are not provided, then the BSK control modules automatically turn off any RW gyroscopic
# compensation.
#
# Instead of directly simulating this control torque vector, new
# algorithm modules are required to first map \f${\mathbf L}_r\f$ on the set of RW motor torques
# \f$u_s\f$.
# ~~~~~~~~~~~~~~~{.py}
#     # add module that maps the Lr control torque into the RW motor torques
#     rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
#     rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
#     rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
#     scSim.AddModelToTask(simTaskName, rwMotorTorqueWrap, rwMotorTorqueConfig)
#     # Initialize the test module msg names
#     rwMotorTorqueConfig.outputDataName = rwStateEffector.InputCmds
#     rwMotorTorqueConfig.inputVehControlName = mrpControlConfig.outputDataName
#     rwMotorTorqueConfig.rwParamsInMsgName = mrpControlConfig.rwParamsInMsgName
#     # Make the RW control all three body axes
#     controlAxes_B = [
#              1,0,0
#             ,0,1,0
#             ,0,0,1
#         ]
#     rwMotorTorqueConfig.controlAxes_B = controlAxes_B
# ~~~~~~~~~~~~~~~
# Note that the output vector of RW motoro torques \f$u_s\f$ is set to connect with
# the RW state effector command input message.  Further, this module inputs the typical
# vehicle configuration message, as well as a message containing the flight algorithm
# information of the RW devices.  This torque mapping module can map the full 3D \f${\mathbf L}_r\f$
# vector onto RW motor torques, or only a subset.  This is specified through the `controlAxes_B` variable
# which specifies up to 3 orthogonal axes to be controlled.  In this simulation the full 3D vector is
# mapped onto motor torques.
#
# The flight algorithm need to know how many RW devices are on the spacecraft and what their
# spin axis \f$\hat{\mathbf g}_s\f$ are.  This is set through a flight software message that is read
# in by flight algorithm modules that need this info.  To write the required flight RW configuration message
# a separate support macros called `fswSetupRW.py`  is used.
# ~~~~~~~~~~~~~~~~{.py}
#     # FSW RW configuration message
#     # use the same RW states in the FSW algorithm as in the simulation
#     fswSetupRW.clearSetup()
#     for rw in simIncludeRW.rwList:
#         fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_S), rw.Js)
#     fswSetupRW.writeConfigMessage(mrpControlConfig.rwParamsInMsgName, scSim.TotalSim, simProcessName)
# ~~~~~~~~~~~~~~~~
# Again a `clearSetup()` should be called first to clear out any pre-existing RW devices from an
# earlier simulation run.  Next, the script above uses the same RW information as what the simulation
# uses.  In this configuration we are simulation perfect RW device knowledge.  If imperfect RW knowledge
# is to be simulated, then the user would input the desired flight states rather than the true
# simulation states.  The support macro `writeConfigMessage()` creates the required RW flight configuration
# message.
#
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          False        # useJitterSimple
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The
# default scenario has the RW jitter turned off.  The
# resulting simulation illustrations are shown below.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedbackRW10.svg "MRP history")
# ![RW Motor Torque History](Images/Scenarios/scenarioAttitudeFeedbackRW20.svg "RW motor torque history")
# ![RW Spin History](Images/Scenarios/scenarioAttitudeFeedbackRW30.svg "RW Omega history")
# Note that in the RW motor torque plot both the required control torque \f$\hat u_s\f$ and the true
# motor torque \f$u_s\f$ are shown.  This illustrates that with this maneuver the RW devices are being
# saturated, and the attitude still eventually stabilizes.
#
##  @}
def run(doUnitTests, show_plots, useJitterSimple):
    '''Call this routine directly to run the tutorial scenario.'''
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    #
    #  From here on there scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

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
    scObject.hub.mHub = 750.0                   # kg - spacecraft mass
    scObject.hub.rBcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = True

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, None, 1)

    # clear prior gravitational body and SPICE setup definitions
    simIncludeGravity.clearSetup()

    # setup Earth Gravity Body
    simIncludeGravity.addEarth()
    simIncludeGravity.gravBodyList[-1].isCentralBody = True          # ensure this is the central gravitational body
    mu = simIncludeGravity.gravBodyList[-1].mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)


    # add RW devices
    # The clearRWSetup() is critical if the script is to run multiple times
    simIncludeRW.clearSetup()
    # the Honeywell HR16 comes in three momentum configuration, 100, 75 and 50 Nms
    simIncludeRW.options.maxMomentum = 50
    # create each RW by specifying the RW type, the spin axis gsHat and the initial wheel speed Omega
    simIncludeRW.create(
            'Honeywell_HR16',
            [1, 0, 0],              # gsHat_S
            100.0                     # RPM
            )
    simIncludeRW.create(
            'Honeywell_HR16',
            [0, 1, 0],              # gsHat_S
            200.0                     # RPM
            )
    if useJitterSimple:
        simIncludeRW.options.RWmodel = simIncludeRW.JitterSimple
    simIncludeRW.create(
            'Honeywell_HR16',
            [0, 0, 1],              # gsHat_S
            300.0,                    # RPM
            [0.5,0.5,0.5]           # r_S (optional argument)
            )
    numRW = simIncludeRW.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    simIncludeRW.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(simTaskName, rwStateEffector, None, 2)


    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simple_nav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)



    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]       # set the desired inertial orientation
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
    mrpControlConfig.inputGuidName  = attErrorConfig.outputDataName
    mrpControlConfig.vehConfigInMsgName  = "vehicleConfigName"
    mrpControlConfig.outputDataName = "LrRequested"
    mrpControlConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    mrpControlConfig.inputRWSpeedsName = rwStateEffector.OutputDataString
    mrpControlConfig.K  =   3.5
    mrpControlConfig.Ki =   -1          # make value negative to turn off integral feedback
    mrpControlConfig.P  = 30.0
    mrpControlConfig.integralLimit = 2./mrpControlConfig.Ki * 0.1
    mrpControlConfig.domega0 = [0.0, 0.0, 0.0]

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
             1,0,0
            ,0,1,0
            ,0,0,1
        ]
    rwMotorTorqueConfig.controlAxes_B = controlAxes_B


    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(rwMotorTorqueConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)
    scSim.TotalSim.logThisMessage(mrpControlConfig.inputRWSpeedsName, samplingTime)
    rwOutName = ["rw_bla0_data", "rw_bla1_data", "rw_bla2_data"]
    for item in rwOutName:
        scSim.TotalSim.logThisMessage(item, samplingTime)

    #
    # create simulation messages
    #
    simIncludeGravity.addDefaultEphemerisMsg(scSim.TotalSim, simProcessName)

    # create the FSW vehicle configuration message
    vehicleConfigOut = vehicleConfigData.vehicleConfigData()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               mrpControlConfig.vehConfigInMsgName,
                               vehicleConfigOut)

    # FSW RW configuration message
    # use the same RW states in the FSW algorithm as in the simulation
    fswSetupRW.clearSetup()
    for rw in simIncludeRW.rwList:
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_S), rw.Js)
    fswSetupRW.writeConfigMessage(mrpControlConfig.rwParamsInMsgName, scSim.TotalSim, simProcessName)


    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()


    #
    #   initialize Spacecraft States within the state manager
    #   this must occur after the initialization
    #
    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a     = 10000000.0           # meters
    oe.e     = 0.01
    oe.i     = 33.3*macros.D2R
    oe.Omega = 48.2*macros.D2R
    oe.omega = 347.8*macros.D2R
    oe.f     = 85.3*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    posRef.setState(unitTestSupport.np2EigenVectorXd(rN))  # m - r_BN_N
    velRef.setState(unitTestSupport.np2EigenVectorXd(vN))  # m - r_BN_N
    sigmaRef.setState([[0.1], [0.2], [-0.3]])       # sigma_BN_B
    omegaRef.setState([[0.001], [-0.01], [0.03]])   # rad/s - omega_BN_B

    # initialize the RW state effector states
    simIncludeRW.setInitialStates(scObject)

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataUsReq = scSim.pullMessageLogData(rwMotorTorqueConfig.outputDataName+".effectorRequest", range(numRW))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName+".sigma_BR", range(3))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName+".omega_BR_B", range(3))
    dataPos = scSim.pullMessageLogData(sNavObject.outputTransName+".r_BN_N", range(3))
    dataOmegaRW = scSim.pullMessageLogData(mrpControlConfig.inputRWSpeedsName+".wheelSpeeds", range(numRW))
    dataRW = []
    for i in range(0,numRW):
        dataRW.append(scSim.pullMessageLogData(rwOutName[i]+".u_current", range(1)))
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]
    timeData = dataUsReq[:, 0] * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(1,4):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$\sigma_'+str(idx)+'$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error $\sigma_{B/R}$')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"1"+str(int(useJitterSimple))
            , plt, path)

    plt.figure(2)
    for idx in range(1,4):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$\hat u_{s,'+str(idx)+'}$')
        plt.plot(timeData, dataRW[idx-1][:, 1],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"2"+str(int(useJitterSimple))
            , plt, path)

    plt.figure(3)
    for idx in range(1,4):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$\omega_{BR,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s) ')

    plt.figure(4)
    for idx in range(1,numRW+1):
        plt.plot(timeData, dataOmegaRW[:, idx]/macros.RPM,
                 color=unitTestSupport.getLineColor(idx,numRW),
                 label='$\Omega_{'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"3"+str(int(useJitterSimple))
            , plt, path)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    #
    #   the python code below is for the unit testing mode.  If you are studying the scenario
    #   to learn how to run BSK, you can stop reading below this line.
    #
    if doUnitTests:
        numTruthPoints = 5
        skipValue = int(numDataPoints/numTruthPoints)
        dataUsRed = dataUsReq[::skipValue]
        dataSigmaBRRed = dataSigmaBR[::skipValue]
        dataPosRed = dataPos[::skipValue]

        # setup truth data for unit test
        truePos = [
                      [-4.0203386903966456e+06, 7.4905667418525163e+06, 5.2482992115893615e+06]
                    , [-4.6421397265661405e+06, 7.0494536040548589e+06, 5.3596540365520352e+06]
                    , [-5.2364026851194846e+06, 6.5665185661712112e+06, 5.4392129624019405e+06]
                    , [-5.7996735881523984e+06, 6.0447162866713591e+06, 5.4865782619213760e+06]
                    , [-6.3286970190056376e+06, 5.4872170491069853e+06, 5.5015438477240102e+06]
                ]
        trueUs = trueSigmaBR = []

        if useJitterSimple == False:
            trueUs = [
                  [ 3.8000000000000000e-01, 4.0000000000000008e-01,-1.5000000000000013e-01]
                , [ 1.0886536396638849e-02,-5.1081088867427316e-01,-4.9465001721576522e-02]
                , [-5.3356020546124400e-02, 7.3280121862582176e-02, 2.3622678489553288e-02]
                , [ 2.4053273555142078e-02,-2.7877284619006338e-03, 1.0490688667807481e-04]
                , [-4.4666896866933491e-03,-3.0806563642653785e-03,-3.2335993502972866e-03]
            ]
            trueSigmaBR = [
                  [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [ 1.3881610052729310e-02,-1.5485878435765174e-01,-1.7589430807049264e-02]
                , [-2.7923740563112063e-02, 1.1269976169106372e-02, 4.7871422910631181e-04]
                , [ 6.1959342447429396e-03, 2.4918559180853771e-03, 3.7300409079442311e-03]
                , [ 1.5260133637049377e-05,-1.2491549414001010e-03,-1.4158582039329860e-03]
            ]
        # compare the results to the truth values
        accuracy = 1e-6

        testFailCount, testMessages = unitTestSupport.compareArray(
            trueUs, dataUsRed, accuracy, "Lr Vector",
            testFailCount, testMessages)

        testFailCount, testMessages = unitTestSupport.compareArray(
            truePos, dataPosRed, accuracy, "r_BN_N Vector",
            testFailCount, testMessages)

        testFailCount, testMessages = unitTestSupport.compareArray(
            trueSigmaBR, dataSigmaBRRed, accuracy, "sigma_BR Set",
            testFailCount, testMessages)

        #   print out success message if no error were found
        if testFailCount == 0:
            print "PASSED "
        else:
            print testFailCount
            print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run( False,       # do unit tests
         True,        # show_plots
         False        # useJitterSimple
       )

