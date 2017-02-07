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
# Purpose:  Integrated test of the spacecraftPlus(), extForceTorque, simpleNav() and
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit.
#           This scenario is the same as test_scenarioAttitudeControl, but with the
#           difference that here the control and dynamics are executed at different
#           frequencies or time steps.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 25, 2016
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
import sim_model
import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import orbitalMotion

# import simulation related support
import spacecraftPlus
import ExtForceTorque
import simIncludeGravity
import simple_nav

# import FSW Algorithm related support
import MRP_Feedback
import inertial3D
import attTrackingError
import vehicleConfigData






# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useUnmodeledTorque, useIntGain", [
      (False, False)
    , (True, False)
    , (True, True)
])

# provide a unique test method name, starting with test_
def test_bskAttitudeFeedback2T(show_plots, useUnmodeledTorque, useIntGain):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, useUnmodeledTorque, useIntGain)
    assert testResults < 1, testMessage


## \defgroup Tutorials_2_1
##   @{
## Demonstrates how to stabilize the tumble of a spacecraft orbiting the
# Earth that is initially tumbling, but uses 2 separate threads.
#
# Attitude Detumbling Simulation in a Two Process Simulation Setup {#scenarioAttitudeFeedback2T}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth. This setup
# is similar to the [test_scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback),
# but here the dynamics
# simulation and the Flight Software (FSW) algorithms are run at different time steps.
# The scenario is again
# setup to be run in three different setups:
# Scenarios Simulation Setup Cases
#
# Setup | useUnmodeledTorque  | useIntGain
# ----- | ------------------- | -------------
# 1     | False               | False
# 2     | True                | False
# 3     | True                | True
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioAttitudeFeedback2T.py
#
# When the simulation completes 3 plots are shown for the MRP attitude history, the rate
# tracking errors, as well as the control torque vector.
#
# A key difference to the 1-process setup is that after the processes are created, the
# dynamics and FSW messages system must be linked to connect messages with the same name.
# Note that the interface references are added to the process that they are SUPPLYING data 
# to.  Reversing that setting is hard to detect as the data will still show up, it will just 
# have a single frame of latency.  This is done in the following two-step process:
#~~~~~~~~~~~~~~{.py}
#     dyn2FSWInterface = sim_model.SysInterface()
#     fsw2DynInterface = sim_model.SysInterface()
#
#     dyn2FSWInterface.addNewInterface(dynProcessName, fswProcessName)
#     fsw2DynInterface.addNewInterface(fswProcessName, dynProcessName)
#
#     dynProcess.addInterfaceRef(fsw2DynInterface)
#     fswProcess.addInterfaceRef(dyn2FSWInterface)
#~~~~~~~~~~~~~~
# Next, after the simulation has been initialized and the modules messages are created
# a discover process must be called that links messages that have the same name.
#~~~~~~~~~~~~~~{.py}
#     dyn2FSWInterface.discoverAllMessages()
#     fsw2DynInterface.discoverAllMessages()
#~~~~~~~~~~~~~~
#
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,       # show_plots
#          False,       # useUnmodeledTorque
#          False        # useIntGain
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last 2 arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The
# default scenario has both the unmodeled torque and integral feedback turned off.  The
# resulting attitude and control torque histories are shown below.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback2T100.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2T200.svg "Torque history")
# Note that now the FSW algorithms are called in a separate process, in the first time step the
# navigation message has not been copied over, and the initial FSW values for the tracking
# errors are zero.  This is why there is a slight difference in the resulting closed loop
# performance.
#
# Setup 2
# ------
#
# Here the python main function is changed to read:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,       # show_plots
#          True,       # useUnmodeledTorque
#          False        # useIntGain
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  Note that, as expected,
# the orientation error doesn't settle to zero, but rather converges to a non-zero offset
# proportional to the unmodeled torque being simulated.  Also, the control torques settle on
# non-zero steady-state values.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback2T110.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2T210.svg "Torque history")
#
# Setup 3
# ------
#
# The final scenario turns on both the unmodeled external torque and the integral
# feedback term:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,       # show_plots
#          True,       # useUnmodeledTorque
#          True        # useIntGain
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  In this case
# the orientation error does settle to zero.  The integral term changes the control torque
# to settle on a value that matches the unmodeled external torque.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback2T111.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2T211.svg "Torque history")
#
##  @}
def run(doUnitTests, show_plots, useUnmodeledTorque, useIntGain):
    '''Call this routine directly to run the tutorial scenario.'''
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    #
    #  From here on there scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

    # Create simulation variable names
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(dynProcessName)
    fswProcess = scSim.CreateNewProcess(fswProcessName)

    # Process message interfaces.
    # this step is used to copy messages between the dyn and fsw processes
    # as long as the message has the same name, it will get copied over automatically
    dyn2FSWInterface = sim_model.SysInterface()
    fsw2DynInterface = sim_model.SysInterface()

    dyn2FSWInterface.addNewInterface(dynProcessName, fswProcessName)
    fsw2DynInterface.addNewInterface(fswProcessName, dynProcessName)

    fswProcess.addInterfaceRef(dyn2FSWInterface)
    dynProcess.addInterfaceRef(fsw2DynInterface)


    # create the dynamics task and specify the integration update time
    simTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simTimeStep))
    fswTimeStep = macros.sec2nano(0.5)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))


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
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = True

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(dynTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    simIncludeGravity.clearSetup()

    # setup Earth Gravity Body
    simIncludeGravity.addEarth()
    simIncludeGravity.gravBodyList[-1].isCentralBody = True          # ensure this is the central gravitational body
    mu = simIncludeGravity.gravBodyList[-1].mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)


    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = ExtForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    # use the input flag to determine which external torque should be applied
    # Note that all variables are initialized to zero.  Thus, not setting this
    # vector would leave it's components all zero for the simulation.
    if useUnmodeledTorque:
        extFTObject.extTorquePntB_B = [[0.25],[-0.25],[0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(dynTaskName, extFTObject)


    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simple_nav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTaskName, sNavObject)



    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(fswTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]       # set the desired inertial orientation
    inertial3DConfig.outputDataName = "guidanceInertial3D"

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(fswTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.outputDataName = "attErrorInertial3DMsg"
    attErrorConfig.inputRefName = inertial3DConfig.outputDataName
    attErrorConfig.inputNavName = sNavObject.outputAttName

    # setup the MRP Feedback control module
    mrpControlConfig = MRP_Feedback.MRP_FeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(fswTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.inputGuidName  = attErrorConfig.outputDataName
    mrpControlConfig.vehConfigInMsgName  = "vehicleConfigName"
    mrpControlConfig.outputDataName = extFTObject.cmdTorqueInMsgName
    mrpControlConfig.K  =   3.5
    if useIntGain:
        mrpControlConfig.Ki =   0.0002      # make value negative to turn off integral feedback
    else:
        mrpControlConfig.Ki =   -1          # make value negative to turn off integral feedback
    mrpControlConfig.P  = 30.0
    mrpControlConfig.integralLimit = 2./mrpControlConfig.Ki * 0.1




    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(mrpControlConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputAttName, samplingTime)



    #
    # create dynamics simulation messages
    #
    simIncludeGravity.addDefaultEphemerisMsg(scSim.TotalSim, dynProcessName)

    #
    # create FSW simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = MRP_Feedback.VehicleConfigMessage()
    vehicleConfigOut.ISCPntB_B = I      # use the same inertia in the FSW algorithm as in the simulation
    unitTestSupport.setMessage(scSim.TotalSim,
                               fswProcessName,
                               mrpControlConfig.vehConfigInMsgName,
                               vehicleConfigOut)


    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    # this next call ensures that the FSW and Dynamics Message that have the same
    # name are copied over every time the simulation ticks forward.  This function
    # has to be called after the simulation is initialized to ensure that all modules
    # have created their own output/input messages declarations.
    dyn2FSWInterface.discoverAllMessages()
    fsw2DynInterface.discoverAllMessages()


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


    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataLr = scSim.pullMessageLogData(mrpControlConfig.outputDataName+".torqueRequestBody", range(3))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName+".sigma_BR", range(3))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName+".omega_BR_B", range(3))
    dataPos = scSim.pullMessageLogData(sNavObject.outputTransName+".r_BN_N", range(3))
    dataSigmaBN = scSim.pullMessageLogData(sNavObject.outputAttName+".sigma_BN", range(3))
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(1,4):
        plt.plot(dataSigmaBR[:, 0]*macros.NANO2MIN, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$\sigma_'+str(idx)+'$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error $\sigma_{B/R}$')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"1"+str(int(useUnmodeledTorque))+str(int(useIntGain)), plt, path)

    plt.figure(2)
    for idx in range(1,4):
        plt.plot(dataLr[:, 0]*macros.NANO2MIN, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$L_{r,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"2"+str(int(useUnmodeledTorque))+str(int(useIntGain)), plt, path)

    plt.figure(3)
    for idx in range(1,4):
        plt.plot(dataOmegaBR[:, 0]*macros.NANO2MIN, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$\omega_{BR,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    plt.figure(4)
    for idx in range(1,4):
        plt.plot(dataPos[:, 0]*macros.NANO2MIN, dataPos[:, idx]/1000,
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$r_{BN,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial Position [km] ')

    plt.figure(5)
    for idx in range(1,4):
        plt.plot(dataSigmaBN[:, 0]*macros.NANO2MIN, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$\sigma_{BN,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial MRP Attitude ')


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
        dataLrRed = dataLr[::skipValue]
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
        trueLr = trueSigmaBR = []
        if useUnmodeledTorque == True and useIntGain == True:
            trueLr = [
                  [-3.8540000000000002e-01,-3.5200000000000009e-01, 4.2000000000000121e-02]
                , [-2.4832413594005637e-01, 2.7423420741984977e-01,-1.2547995906140999e-01]
                , [-2.4963855704325660e-01, 2.4180503459076927e-01,-9.0824655654560021e-02]
                , [-2.4705814926163225e-01, 2.4984933912594240e-01,-9.9789581766606752e-02]
                , [-2.4992663484004582e-01, 2.4915540593910049e-01,-9.9730854856601880e-02]
            ]
            trueSigmaBR = [
                  [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [ 2.4124788077353267e-02,-8.8078468077718686e-02, 6.7556236560029792e-02]
                , [ 2.0013848145133590e-02,-1.5036479216989354e-02, 1.6743292993630865e-02]
                , [ 4.3556855886602566e-03,-8.2916392106937194e-03, 4.8022149157636237e-03]
                , [ 1.7102077355609178e-03,-2.5229471654219780e-03, 2.0963057897404771e-03]
            ]
        if useUnmodeledTorque == True and useIntGain == False:
            trueLr = [
                  [-3.8000000000000000e-01,-4.0000000000000008e-01, 1.5000000000000013e-01]
                , [-2.6967258574434960e-01, 2.3852492578210521e-01,-9.9303066167128723e-02]
                , [-2.4553483719840241e-01, 2.5582895110635612e-01,-9.9783874073020584e-02]
                , [-2.5082575869743895e-01, 2.4917049711833658e-01,-9.9921820727609134e-02]
                , [-2.4986476881781602e-01, 2.5008633794967206e-01,-1.0003104112824485e-01]
            ]
            trueSigmaBR = [
                  [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [ 6.3945338706459742e-02,-8.7562909724589022e-02, 4.4198807712487694e-02]
                , [ 7.1960157856111040e-02,-7.0992542477623266e-02, 2.7112368217686023e-02]
                , [ 7.1431247623012131e-02,-7.1324233641929718e-02, 2.8746725391756406e-02]
                , [ 7.1414708584927961e-02,-7.1455518150866384e-02, 2.8552586824521019e-02]
            ]
        if useUnmodeledTorque == False and useIntGain == False:
            trueLr = [
                  [-3.8000000000000000e-01,-4.0000000000000008e-01, 1.5000000000000013e-01]
                , [ 2.9018622651951317e-02,-2.3731740129077500e-03, 1.8500888075394767e-02]
                , [-1.4212083106812448e-03, 1.7754834987403689e-03,-1.3760887721230200e-03]
                , [-6.8638983386268455e-05,-2.6617199062440423e-04, 5.5312276312328556e-05]
                , [ 3.3478249139870173e-05, 2.8598845181088252e-05, -1.3792582437169445e-06]
            ]
            trueSigmaBR = [
                  [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [-1.6310559825609434e-02,-1.1632615581332386e-02, 9.0311147891659286e-03]
                , [ 1.8165879114118769e-03, 7.4688330133023404e-04,-1.2070602115782872e-04]
                , [-1.8335140530225598e-04,-2.9214999036672645e-05,-5.7216976124152215e-06]
                , [ 1.6181183222292735e-05,-1.0129144274203115e-06, 5.1639058023290004e-07]
            ]
        # compare the results to the truth values
        accuracy = 1e-6

        testFailCount, testMessages = unitTestSupport.compareArray(
            trueLr, dataLrRed, accuracy, "Lr Vector",
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
            print "# Errors:", testFailCount
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
         True,       # show_plots
         False,       # useUnmodeledTorque
         False        # useIntGain
       )

