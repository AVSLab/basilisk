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
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')
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


    sNavObject = simple_nav.SimpleNav()
    extFTObject = ExtForceTorque.ExtForceTorque()

    # #
    # #   setup the FSW algorithm tasks
    # #
    #
    # # setup inertial3D guidance module
    # inertial3DConfig = inertial3D.inertial3DConfig()
    # inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    # inertial3DWrap.ModelTag = "inertial3D"
    # scSim.AddModelToTask(fswTaskName, inertial3DWrap, inertial3DConfig)
    # inertial3DConfig.sigma_R0N = [0., 0., 0.]       # set the desired inertial orientation
    # inertial3DConfig.outputDataName = "guidanceInertial3D"
    #
    # # setup the attitude tracking error evaluation module
    # attErrorConfig = attTrackingError.attTrackingErrorConfig()
    # attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    # attErrorWrap.ModelTag = "attErrorInertial3D"
    # scSim.AddModelToTask(fswTaskName, attErrorWrap, attErrorConfig)
    # attErrorConfig.outputDataName = "attErrorInertial3DMsg"
    # attErrorConfig.inputRefName = inertial3DConfig.outputDataName
    # attErrorConfig.inputNavName = sNavObject.outputAttName
    #
    # # setup the MRP Feedback control module
    # mrpControlConfig = MRP_Feedback.MRP_FeedbackConfig()
    # mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    # mrpControlWrap.ModelTag = "MRP_Feedback"
    # scSim.AddModelToTask(fswTaskName, mrpControlWrap, mrpControlConfig)
    # mrpControlConfig.inputGuidName  = attErrorConfig.outputDataName
    # mrpControlConfig.vehConfigInMsgName  = "vehicleConfigName"
    # mrpControlConfig.outputDataName = extFTObject.cmdTorqueInMsgName
    # mrpControlConfig.K  =   3.5
    # if useIntGain:
    #     mrpControlConfig.Ki =   0.0002      # make value negative to turn off integral feedback
    # else:
    #     mrpControlConfig.Ki =   -1          # make value negative to turn off integral feedback
    # mrpControlConfig.P  = 30.0
    # mrpControlConfig.integralLimit = 2./mrpControlConfig.Ki * 0.1


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
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrixXd(I)
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
    # extFTObject = ExtForceTorque.ExtForceTorque()
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
    # sNavObject = simple_nav.SimpleNav()
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
    vehicleConfigOut = vehicleConfigData.vehicleConfigData()
    inputMessageSize = vehicleConfigOut.getStructSize()
    scSim.TotalSim.CreateNewMessage(fswProcessName, mrpControlConfig.vehConfigInMsgName,
                                          inputMessageSize, 2)
    # use the same inertia in the FSW algorithm as in the simulation
    vehicleConfigOut.ISCPntB_B = I
    scSim.TotalSim.WriteMessageData(mrpControlConfig.vehConfigInMsgName,
                                    inputMessageSize,
                                    0, vehicleConfigOut)


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

    posRef.setState(unitTestSupport.np2EigenVector3d(rN))  # m - r_BN_N
    velRef.setState(unitTestSupport.np2EigenVector3d(vN))  # m - r_BN_N
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
    print dataLr

    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]
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
                  [-0.0000000000000000e+00,-0.0000000000000000e+00,-0.0000000000000000e+00]
                , [-2.4892377027721430e-01, 2.7007071362935875e-01,-1.2455586615476819e-01]
                , [-2.4948148419074859e-01, 2.4266423610655152e-01,-9.0724080697132545e-02]
                , [-2.4709751666573707e-01, 2.4971648992319734e-01,-9.9844866410803840e-02]
                , [-2.4993438668027146e-01, 2.4918021098988458e-01,-9.9707495611305222e-02]
            ]
            trueSigmaBR = [
                  [ 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00]
                , [ 2.3644431439787103e-02,-8.7358892634732732e-02, 6.7461064071103832e-02]
                , [ 1.9970339954486448e-02,-1.5117904386027238e-02, 1.6821825977465451e-02]
                , [ 4.3062602140655935e-03,-8.2691329258470832e-03, 4.9242158392858145e-03]
                , [ 1.7087578443510938e-03,-2.5155867263617552e-03, 2.1073512814346112e-03]
            ]
        if useUnmodeledTorque == True and useIntGain == False:
            trueLr = [
                  [-0.0000000000000000e+00,-0.0000000000000000e+00,-0.0000000000000000e+00]
                , [-2.7048252116784532e-01, 2.3420894071876219e-01,-9.7869034276692218e-02]
                , [-2.4527391269230447e-01, 2.5632299647944368e-01,-9.9997257317616287e-02]
                , [-2.5087193660129020e-01, 2.4916683645057938e-01,-9.9874856904476755e-02]
                , [-2.4986168927326641e-01, 2.5007598529208708e-01,-1.0003845361180973e-01]
            ]
            trueSigmaBR = [
                  [ 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00]
                , [ 6.3924172727115203e-02,-8.6942497319127121e-02, 4.3767157675466670e-02]
                , [ 7.1891547087706412e-02,-7.1187991999062561e-02, 2.7137257010884548e-02]
                , [ 7.1454953842474911e-02,-7.1294076042318619e-02, 2.8739213985194037e-02]
                , [ 7.1409615380055996e-02,-7.1458240041155432e-02, 2.8555054893675037e-02]
            ]
        if useUnmodeledTorque == False and useIntGain == False:
            trueLr = [
                  [-0.0000000000000000e+00,-0.0000000000000000e+00,-0.0000000000000000e+00]
                , [ 2.8145728071642062e-02,-4.4493983666986023e-03, 1.9899718539106000e-02]
                , [-9.4715432578159221e-04, 2.0826149210831412e-03,-1.3480129941689915e-03]
                , [-1.6110621936204161e-04,-2.8737186957356785e-04, 4.6408348429500232e-05]
                , [ 4.6557322023934057e-05, 2.8313463938606023e-05,-7.7885686369882820e-07]
            ]
            trueSigmaBR = [
                  [ 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00]
                , [-1.6517323525879250e-02,-1.1277869301670424e-02, 8.3749349848387961e-03]
                , [ 1.7951805732487282e-03, 6.5110040458101078e-04,-6.9028191269068580e-05]
                , [-1.6987415329544191e-04,-1.5088365774999048e-05,-7.3239305809835457e-06]
                , [ 1.2989178503771199e-05,-2.4936275334020408e-06, 4.8933798362731636e-07]
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
         True,       # show_plots
         False,       # useUnmodeledTorque
         False        # useIntGain
       )

