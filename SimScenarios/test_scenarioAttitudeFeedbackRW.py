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
# Earth that is initially tumbling.
#
# Attitude Detumbling Simulation using RW Effectors {#scenarioAttitudeFeedbackMRP}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The scenario is
# setup to be run in four different setups:
# Setup | useJitterSimple
# ----- | -------------------
# 1     | False
# 2     | True
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioAttitudeFeedback.py
#
# When the simulation completes 3 plots are shown for the MRP attitude history, the rate
# tracking errors, as well as the control torque vector.
#
# The dynamics simulation is setup using a SpacecraftPlus() module to which a gravity
# effector is attached.  Note that both the rotational and translational degrees of
# freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.  For more
# information on how to setup orbit, see [test_scenarioBasicOrbit.py](@ref scenarioBasicOrbit)
#
# The control torque is simulated usign the ExtForceTorque() module.  This module can
# accept a torque in body frame components either through an input message, or through
# a module internal torque vector which can be set in python.  In this simulation, the
# flight software is providing the attitude control torque message which is connected to
# the torque input message of this module.  If an external torque is being simulated,
# then the module internal torque vector is set to a constant value.
#
# Lastly, the flight software algorithm module require a navigation message with the
# spacecraft orientation and attitude rates.  This is setup using the simple_nav()
# module. By just invoking a sensor module it is setup to run without any simulated
# corruptions.  Thus in this simulation it will return truth measurements.
#
# Next the flight software algorithms need to be setup.  The inertial pointing reference
# frame definition is provided through the simple inertial3D() module.  The only input
# it requires is the desired inertial heading.
#
# The reference frame states and the navigation message (output of simple_nav()) are fed
# into the attTrackingError() module.  It is setup to compute the attitude tracking error
# between the body frame *B* and the reference frame *R*.  If a body fixed frame other than *B*
# needs to be driven towards *R*, this could be configured as well in this module.
#
# Finally the tracking errors are fed to the classic MRP feedback control module.  The
# algorithm of this is discussed in the text book *Analytical Mechanics of Space Systems*
# (<http://arc.aiaa.org/doi/book/10.2514/4.102400>).  The control torque output vector message of this
# module is connected back to the input message of the extForceTorque() module to close
# the control loop.
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
#          False,       # useUnmodeledTorque
#          False,       # useIntGain
#          False        # useKnownTorque
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last 3 arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The
# default scenario has both the unmodeled torque and integral feedback turned off.  The
# resulting attitude and control torque histories are shown below.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback1000.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2000.svg "Torque history")
#
# Setup 2
# ------
#
# Here the python main function is changed to read:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          True,        # useUnmodeledTorque
#          False,       # useIntGain
#          False        # useKnownTorque
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  Note that, as expected,
# the orientation error doesn't settle to zero, but rather converges to a non-zero offset
# proportional to the unmodeled torque being simulated.  Also, the control torques settle on
# non-zero steady-state values.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback1100.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2100.svg "Torque history")
#
# Setup 3
# ------
#
# The 3rd scenario turns on both the unmodeled external torque and the integral
# feedback term:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          True,        # useUnmodeledTorque
#          False,       # useIntGain
#          False        # useKnownTorque
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  In this case
# the orientation error does settle to zero.  The integral term changes the control torque
# to settle on a value that matches the unmodeled external torque.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback1110.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2110.svg "Torque history")
#
# Setup 4
# ------
#
# The 4th scenario turns on the unmodeled external torque but keeps the integral
# feedback term off.  Instead, the external disturbance is fed forward in the
# attitude control solution.
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          True,        # useUnmodeledTorque
#          False,       # useIntGain
#          True         # useKnownTorque
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  In this case
# the orientation error does settle to zero as the feedforward term compensates for
# the external torque.  The control torque is now caused
# to settle on a value that matches the unmodeled external torque.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback1101.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2101.svg "Torque history")
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
    scSim.AddModelToTask(simTaskName, scObject)

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
    scSim.AddModelToTask(simTaskName, rwStateEffector)


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
    inputMessageSize = vehicleConfigOut.getStructSize()
    scSim.TotalSim.CreateNewMessage(simProcessName, mrpControlConfig.vehConfigInMsgName,
                                          inputMessageSize, 2)
    # use the same inertia in the FSW algorithm as in the simulation
    vehicleConfigOut.ISCPntB_B = I
    scSim.TotalSim.WriteMessageData(mrpControlConfig.vehConfigInMsgName,
                                    inputMessageSize,
                                    0, vehicleConfigOut)

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
        if useJitterSimple == True:
            trueUs = [
                  [-3.8540000000000002e-01,-3.5200000000000009e-01, 4.2000000000000121e-02]
                , [-2.3849697806730846e-01, 2.9471283787682012e-01,-1.3566545702259455e-01]
                , [-2.5271637424714444e-01, 2.3615511889142107e-01,-9.0488478286136861e-02]
                , [-2.4614222882341519e-01, 2.5067425482476591e-01,-9.8977162057449455e-02]
                , [-2.4977928591111503e-01, 2.4887615666172175e-01,-9.9881092081412159e-02]
            ]
            trueSigmaBR = [
                  [1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [2.2480494577949272e-02,-9.5531096658816039e-02, 7.0195707303957244e-02]
                , [2.2497104328479428e-02,-1.6693879988589459e-02, 2.1096813515555320e-02]
                , [5.5130423153784084e-03,-9.6647966447711703e-03, 5.2740482749995665e-03]
                , [1.9666952518230217e-03,-3.2953351361057178e-03, 2.7072233285654586e-03]
            ]
        if useJitterSimple == False:
            trueUs = [
                  [ 3.8000000000000000e-01, 4.0000000000000008e-01,-1.5000000000000013e-01]
                , [ 1.1231402312140565e-02,-5.1291709034434607e-01,-4.9996296037748973e-02]
                , [-5.3576899204811054e-02, 7.3722479933297710e-02, 2.3880144351365463e-02]
                , [ 2.4193559082756406e-02,-2.8516319358299399e-03, 2.6158801499764212e-06]
                , [-4.5358804715397785e-03,-3.0828353818758048e-03,-3.2251584952585274e-03]
            ]
            trueSigmaBR = [
                  [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [ 1.4061613716759677e-02,-1.5537401133724818e-01,-1.7736020110557204e-02]
                , [-2.8072554033139220e-02, 1.1328152717859542e-02, 4.8023651815939054e-04]
                , [ 6.2505180487499833e-03, 2.4908595924511279e-03, 3.7332111196198281e-03]
                , [-1.2999627747525804e-06,-1.2575327981617813e-03,-1.4238011880860959e-03]
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

