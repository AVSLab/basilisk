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
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 19, 2016
#



import pytest
import sys, os, inspect
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
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
import unitTestSupport                  # general support file with common unit test functions
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
@pytest.mark.parametrize("useUnmodeledTorque, useIntGain, useKnownTorque", [
      (False, False, False)
    , (True, False, False)
    , (True, True, False)
    , (True, False, True)
])

# provide a unique test method name, starting with test_
def test_bskAttitudeFeedback(show_plots, useUnmodeledTorque, useIntGain, useKnownTorque):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, useUnmodeledTorque, useIntGain, useKnownTorque)
    assert testResults < 1, testMessage



## This scenario demonstrates how to stabilize the tumble of a spacecraft orbiting the
# Earth that is initially tumbling.
#
# Attitude Detumbling Simulation in a Single Simulation Process {#scenarioAttitudeFeedback}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The scenario is
# setup to be run in four different setups:
# Setup | useUnmodeledTorque  | useIntGain | useKnownTorque
# ----- | ------------------- | ---------- | --------------
# 1     | False               | False      | False
# 2     | True                | False      | False
# 3     | True                | True       | False
# 4     | True                | False      | True
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

def run(doUnitTests, show_plots, useUnmodeledTorque, useIntGain, useKnownTorque):
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
    simulationTime = macros.minute2nano(10.)

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
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrixXd(I)
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = True

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)


    # setup Earth Gravity Body
    earthGravBody, earthEphemData = simIncludeGravity.addEarth()
    earthGravBody.isCentralBody = True          # ensure this is the central gravitational body

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([earthGravBody])


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
    scSim.AddModelToTask(simTaskName, extFTObject)


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
    mrpControlConfig.outputDataName = extFTObject.cmdTorqueInMsgName
    mrpControlConfig.K  =   3.5
    if useIntGain:
        mrpControlConfig.Ki =   0.0002      # make value negative to turn off integral feedback
    else:
        mrpControlConfig.Ki =   -1          # make value negative to turn off integral feedback
    mrpControlConfig.P  = 30.0
    mrpControlConfig.integralLimit = 2./mrpControlConfig.Ki * 0.1
    mrpControlConfig.domega0 = [0.0, 0.0, 0.0]
    if useKnownTorque:
        mrpControlConfig.knownTorquePntB_B = [0.25,-0.25,0.1]



    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 50
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(mrpControlConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)



    #
    # create simulation messages
    #

    # create the gravity ephemerise message
    scSim.TotalSim.CreateNewMessage(simProcessName,
                                          earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 2)
    scSim.TotalSim.WriteMessageData(earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 0,
                                          earthEphemData)

    # create the FSW vehicle configuration message
    inputMessageSize = 18*8+8                           # 18 doubles + 1 32bit integer
    scSim.TotalSim.CreateNewMessage(simProcessName, mrpControlConfig.vehConfigInMsgName,
                                          inputMessageSize, 2)
    vehicleConfigOut = vehicleConfigData.vehicleConfigData()
    # use the same inertia in the FSW algorithm as in the simulation
    vehicleConfigOut.ISCPntB_B = I
    scSim.TotalSim.WriteMessageData(mrpControlConfig.vehConfigInMsgName,
                                    inputMessageSize,
                                    0, vehicleConfigOut)


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
    rN, vN = orbitalMotion.elem2rv(earthGravBody.mu, oe)

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
    np.set_printoptions(precision=16)


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
            fileNameString+"1"+str(int(useUnmodeledTorque))+str(int(useIntGain))
            +str(int(useKnownTorque))
            , plt, path)

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
            fileNameString+"2"+str(int(useUnmodeledTorque))+str(int(useIntGain))
            +str(int(useKnownTorque))
            , plt, path)

    plt.figure(3)
    for idx in range(1,4):
        plt.plot(dataOmegaBR[:, 0]*macros.NANO2MIN, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$\omega_{BR,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

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
                      [-4.0203386903966488e+06, 7.4905667418525126e+06, 5.2482992115893615e+06]
                    , [-4.6471275161781209e+06, 7.0456612350748070e+06, 5.3604360754118618e+06]
                    , [-5.2458956740931515e+06, 6.5582694131572684e+06, 5.4402498214923274e+06]
                    , [-5.8131059821266048e+06, 6.0314219867521739e+06, 5.4873352766347500e+06]
                    , [-6.3454267104367241e+06, 5.4683721059305444e+06, 5.5014852653880799e+06]
                ]
        trueLr = trueSigmaBR = []

        if useUnmodeledTorque == True and useIntGain == True and useKnownTorque == False:
            trueLr = [
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
        if useUnmodeledTorque == True and useIntGain == False and useKnownTorque == False:
            trueLr = [
                  [-3.8000000000000000e-01,-4.0000000000000008e-01, 1.5000000000000013e-01]
                , [-2.6249726949900559e-01, 2.5589984841560653e-01,-1.0917765643851718e-01]
                , [-2.4690643822771280e-01, 2.5434333217456701e-01,-9.8004180550054276e-02]
                , [-2.5063142579907921e-01, 2.4907026475177449e-01,-1.0026690856174182e-01]
                , [-2.4987503028354416e-01, 2.5015462259257626e-01,-9.9974532278102879e-02]
            ]
            trueSigmaBR = [
                  [1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [6.0923084226923496e-02,-9.2434292921154987e-02, 4.6511034647862895e-02]
                , [7.2857056494818079e-02,-6.9771873369940063e-02, 2.6850412008424970e-02]
                , [7.1220889524830688e-02,-7.1510953126517118e-02, 2.8814970926949179e-02]
                , [7.1456109973308091e-02,-7.1435046045892472e-02, 2.8534341637639557e-02]
            ]
        if useUnmodeledTorque == False and useIntGain == False and useKnownTorque == False:
            trueLr = [
                  [-3.8000000000000000e-01,-4.0000000000000008e-01, 1.5000000000000013e-01]
                , [ 4.3304295406265583e-02, 7.7970819853086931e-03, 1.2148680350980004e-02]
                , [-4.8427756513968068e-03, 2.6583725254198179e-04,-1.4133980386514184e-03]
                , [ 5.2386812124888282e-04,-1.3752464748227947e-04, 8.9786880165438401e-05]
                , [-5.3815258259032201e-05, 2.3975789622333814e-05,-4.5666337024320216e-06]
            ]
            trueSigmaBR = [
                  [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [-1.7700318439403492e-02,-1.4154347776578310e-02, 1.2434108941675513e-02]
                , [ 2.3210853655701645e-03, 1.3316275028241674e-03,-4.1569615433473430e-04]
                , [-3.0275893560215703e-04,-1.1614876733451711e-04, 8.6068784583440090e-06]
                , [ 3.9002194932293482e-05, 9.3813814117398300e-06, 1.5011853130355206e-07]
            ]
        if useUnmodeledTorque == True and useIntGain == False and useKnownTorque == True:
            trueLr = [
                  [-6.3000000000000000e-01,-1.5000000000000008e-01, 5.0000000000000128e-02]
                , [-2.0669233648787125e-01, 2.5765561543404975e-01,-8.7857327347252573e-02]
                , [-2.5484394926182946e-01, 2.5027978077275026e-01,-1.0141318726877116e-01]
                , [-2.4947595336005560e-01, 2.4986116764798483e-01,-9.9910231437738084e-02]
                , [-2.5005384173665213e-01, 2.5002409166692252e-01,-1.0000456570945075e-01]
            ]
            trueSigmaBR = [
                  [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [-1.7696313533933930e-02,-1.4143120263281624e-02, 1.2430697844911655e-02]
                , [ 2.3206734210692009e-03, 1.3296808140089579e-03,-4.1561415351098834e-04]
                , [-3.0275893560215703e-04,-1.1614876733451711e-04, 8.6068784583440090e-06]
                , [ 3.9002194932293482e-05, 9.3813814117398300e-06, 1.5011853130355206e-07]
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
         True,        # show_plots
         True,        # useUnmodeledTorque
         False,       # useIntGain
         True         # useKnownTorque
       )

