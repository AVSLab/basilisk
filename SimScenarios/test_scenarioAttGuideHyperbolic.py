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
# Purpose:  Integrated test of the spacecraftPlus(), extForceTorque, simpleNav(),
#           MRP_Feedback() with attitude navigation modules.  This script is a
#           spinoff from the attitude guidance tutorial, it implements a hyperbolic
#           trajectory and uses the velocityPoint module.
# Author:   Anne Bennett
# Creation Date:  Aug. 28th, 2017
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
import RigidBodyKinematics

# import simulation related support
import spacecraftPlus
import ExtForceTorque
import simIncludeGravBody
import simple_nav

# import FSW Algorithm related support
import MRP_Feedback
import velocityPoint
import attTrackingError

# import message declarations
import fswMessages

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useAltBodyFrame", [
      (False)
    , (True)
])

# provide a unique test method name, starting with test_
def test_bskAttGuide_Hyperbolic(show_plots, useAltBodyFrame):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, useAltBodyFrame)
    assert testResults < 1, testMessage


## \defgroup Tutorials_2_1_1
##   @{
## How to use guidance modules to align the spacecraft frame to the velocity-pointing frame.
#
# Attitude Alignment for a Spacecraft on a Hyperbolic Trajectory {#scenarioAttGuideHyperbolic}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is on a hyperbolic trajectory near Earth.
# It aligns the spacecraft to point along the velocity vector throughout the orbit.
#  The scenario is setup to be run in two different configurations:
# Setup | useAltBodyFrame
# ----- | -------------------
# 1     | False
# 2     | True
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioAttGuideHyperbolic.py
#
# The simulation layout is shown in the following illustration.  A single simulation process is created
# which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
# modules.
# ![Simulation Flow Diagram](Images/doc/test_scenarioAttGuideHyperbolic.svg "Illustration")
#
# When the simulation completes 4 plots are shown. This first three show the MRP attitude history, the rate
# tracking errors, and the control torque vector. The fourth shows the hyperbolic trajectory
# and the segment of that trajectory flown during the simulation.
#
# The basic simulation setup is the same as the one used in
# [test_scenarioAttitudeGuidance.py](@ref scenarioAttitudeGuidance).
# The dynamics simulation is setup using a SpacecraftPlus() module to which a gravity
# effector is attached.  Note that both the rotational and translational degrees of
# freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.  For more
# information on how to setup an orbit, see [test_scenarioBasicOrbit.py](@ref scenarioBasicOrbit)
#
# Where the Attitude Guidance Tutorial pointed the spacecraft relative to the Hill frame, this tutorial
# points it relative to the velocity vector.
# Note that mu must be assigned to attGuidanceConfig.mu when using the velocityPoint() module:
# ~~~~~~~~~~~~~{.py}
#     attGuidanceConfig = velocityPoint.velocityPointConfig()
#     attGuidanceWrap = scSim.setModelDataWrap(attGuidanceConfig)
#     attGuidanceWrap.ModelTag = "velocityPoint"
#     attGuidanceConfig.inputNavDataName = sNavObject.outputTransName
#     attGuidanceConfig.inputCelMessName = earth.bodyInMsgName
#     attGuidanceConfig.outputDataName = "guidanceOut"
#     attGuidanceConfig.mu = mu
#     scSim.AddModelToTask(simTaskName, attGuidanceWrap, attGuidanceConfig)
# ~~~~~~~~~~~~~
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
#          False        # useAltBodyFrame
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The remaining argument controls the
# simulation scenario flags to turn on or off certain simulation conditions.  The
# default scenario shown has the `useAltBodyFrame` flag turned off.  This means that we seek
# to align the body frame *B* with the velocity vector *V*.
# ![MRP Attitude History](Images/Scenarios/scenarioAttGuideHyperbolic10.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttGuideHyperbolic20.svg "Torque history")
# ![Rate Tracking Error](Images/Scenarios/scenarioAttGuideHyperbolic30.svg "Rate Tracking Error")
# ![Hyperbolic Orbit Illustration](Images/Scenarios/scenarioAttGuideHyperbolic40.svg "Hyperbolic Orbit Illustration")
#
#
# Setup 2
# -----
#
# To run the second scenario, change the main routine at the bottom of the file to read:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          True         # useAltBodyFrame
#        )
# ~~~~~~~~~~~~~
# Here the control should not align the principal body frame *B* with *V*, but rather an alternate,
# corrected body frame *Bc*.  For example, if a thruster is located on the \f$\hat b_1\f$ face, and it
# is desired to point it along the negative V-bar, this is achieved through:
# ~~~~~~~~~~~~~{.py}
#   attErrorConfig.sigma_R0R = [0,0,-1]
# ~~~~~~~~~~~~~
# This corrected body frame has an orientation which is rotated 180 degrees about \f$\hat b_3\f$,
# to point the correct face of the spacecraft along the negative V-bar.
#
# The resulting attitude and control torque histories are shown below.
# ![MRP Attitude History](Images/Scenarios/scenarioAttGuideHyperbolic11.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttGuideHyperbolic21.svg "Torque history")
# ![Rate Tracking Error](Images/Scenarios/scenarioAttGuideHyperbolic30.svg "Rate Tracking Error")
#
##  @}
def run(doUnitTests, show_plots, useAltBodyFrame):
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
    simulationTime = macros.sec2nano(750.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # unitTestSupport.enableVisualization(scSim, dynProcess, simProcessName, 'earth')  # The Viz only support 'earth', 'mars', or 'sun'

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
    scSim.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #
    #   initialize Spacecraft States with initialization variables
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a     = -150000.0 * 1000          # meters
    oe.e     = 1.5
    oe.i     = 33.3*macros.D2R
    oe.Omega = 48.2*macros.D2R
    oe.omega = 347.8*macros.D2R
    oe.f     = 30*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]       # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]   # rad/s - omega_BN_B

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = ExtForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    # use the input flag to determine which external torque should be applied
    # Note that all variables are initialized to zero.  Thus, not setting this
    # vector would leave it's components all zero for the simulation.
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

    # setup velocityPoint guidance module
    attGuidanceConfig = velocityPoint.velocityPointConfig()
    attGuidanceWrap = scSim.setModelDataWrap(attGuidanceConfig)
    attGuidanceWrap.ModelTag = "velocityPoint"
    attGuidanceConfig.inputNavDataName = sNavObject.outputTransName
    attGuidanceConfig.inputCelMessName = earth.bodyInMsgName
    attGuidanceConfig.outputDataName = "guidanceOut"
    attGuidanceConfig.mu = mu
    scSim.AddModelToTask(simTaskName, attGuidanceWrap, attGuidanceConfig)


    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.outputDataName = "attErrorMsg"
    if useAltBodyFrame:
        attErrorConfig.sigma_R0R = [0,0,-1]
    attErrorConfig.inputRefName = attGuidanceConfig.outputDataName
    attErrorConfig.inputNavName = sNavObject.outputAttName

    # setup the MRP Feedback control module
    mrpControlConfig = MRP_Feedback.MRP_FeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.inputGuidName  = attErrorConfig.outputDataName
    mrpControlConfig.vehConfigInMsgName  = "vehicleConfigName"
    mrpControlConfig.outputDataName = extFTObject.cmdTorqueInMsgName
    mrpControlConfig.K  =  3.5
    mrpControlConfig.Ki = -1.0      # make value negative to turn off integral feedback
    mrpControlConfig.P  = 30.0
    mrpControlConfig.integralLimit = 2./mrpControlConfig.Ki * 0.1
    mrpControlConfig.domega0 = [0.0, 0.0, 0.0]


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
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               mrpControlConfig.vehConfigInMsgName,
                               vehicleConfigOut)

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
    dataLr = scSim.pullMessageLogData(mrpControlConfig.outputDataName+".torqueRequestBody", range(3))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName+".sigma_BR", range(3))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName+".omega_BR_B", range(3))
    dataPos = scSim.pullMessageLogData(sNavObject.outputTransName+".r_BN_N", range(3))
    dataVel = scSim.pullMessageLogData(sNavObject.outputTransName+".v_BN_N", range(3))
    dataSigmaBN = scSim.pullMessageLogData(sNavObject.outputAttName + ".sigma_BN", range(3))
    np.set_printoptions(precision=16)



    #
    #   plot the results
    #
    timeLineSet = dataSigmaBR[:, 0]*macros.NANO2MIN
    fileNameString = filename[len(path)+6:-3]
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    vectorData = unitTestSupport.pullVectorSetFromData(dataSigmaBR)
    sNorm = np.array([np.linalg.norm(v) for v in vectorData])
    plt.plot(timeLineSet, sNorm,
             color=unitTestSupport.getLineColor(1,3),
             )
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error Norm $|\sigma_{B/R}|$')
    ax.set_yscale('log')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"1"+str(int(useAltBodyFrame))
            , plt, path)

    plt.figure(2)
    for idx in range(1,4):
        plt.plot(timeLineSet, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$L_{r,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"2"+str(int(useAltBodyFrame))
            , plt, path)

    plt.figure(3)
    for idx in range(1,4):
        plt.plot(timeLineSet, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$\omega_{BR,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"3"+str(int(useAltBodyFrame))
            , plt, path)


    # draw orbit in perifocal frame
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(4, figsize=np.array((1.0, 1.)) * 4.75, dpi=100)
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = '#008800'
    planet = gravFactory.createEarth()
    planetRadius = planet.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(dataPos)):
        oeData = orbitalMotion.rv2elem(mu, dataPos[idx, 1:4], dataVel[idx, 1:4])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000
             , color='#aa0000'
             , linewidth=3.0
             , label='Simulated Flight'
             )

    plt.axis(np.array([-1, 1, -1, 1]) * 1.25 * np.amax(rData) / 1000)

    # draw the full osculating orbit from the initial conditions
    tempAngle = (1./2.)*(np.pi-2*np.arcsin(1/oe.e))*1.01
    fData = np.linspace(np.pi-tempAngle, -np.pi+tempAngle, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000
             , '--'
             , color='#555555'
             , label='Orbit Track'
             )

    plt.xlabel('$i_e$ Cord. [km]')
    plt.ylabel('$i_p$ Cord. [km]')
    plt.legend(loc='lower left')
    plt.grid()

    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"4"+str(int(useAltBodyFrame))
            , plt, path)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    #
    #   the python code below is for the unit testing mode.  If you are studying the scenario
    #   to learn how to run BSK, you can stop reading below this line.
    # NEED TO UPDATE UNIT TEST DATA
    #
    if doUnitTests:
        numTruthPoints = 5
        skipValue = int(numDataPoints/numTruthPoints)
        dataSigmaBNRed = dataSigmaBN[::skipValue]
        dataPosRed = dataPos[::skipValue]

        # setup truth data for unit test
        truePos = [
                      [3.6223376821150966e+07, 7.1776505575846523e+07, 1.3687819378018096e+07]
                    , [3.5873290076594226e+07, 7.2092075260881290e+07, 1.3997417901516432e+07]
                    , [3.5522532862572916e+07, 7.2406297570323750e+07, 1.4306754823209373e+07]
                    , [3.5171116051793166e+07, 7.2719175431216419e+07, 1.4615826100429773e+07]
                    , [3.4819050453380756e+07, 7.3030711891436249e+07, 1.4924627774549646e+07]
                ]
        trueLr = trueSigmaBR = []
        if useAltBodyFrame == True:
            trueSigmaBN = [
                      [ 1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
                    , [-9.2494162977495867e-02, 1.9471395865807911e-01, -6.3717384535805643e-01]
                    , [-8.4160284482831221e-02, 1.8751522022305400e-01, -6.2862018070118753e-01]
                    , [-8.3717220192117484e-02, 1.8793830908990347e-01, -6.2761281563466287e-01]
                    , [-8.3427503754355453e-02, 1.8790862092331320e-01, -6.2675005457853550e-01]
            ]
        if useAltBodyFrame == False:
            trueSigmaBN = [
                      [ 1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
                    , [ 1.3870159058177514e-01, 6.5242458655457275e-02,  2.1071408452248369e-01]
                    , [ 1.3927887967605357e-01, 6.2240967986042707e-02,  2.0898043796751192e-01]
                    , [ 1.3967975559519039e-01, 6.2219318146119917e-02,  2.0946440039329009e-01]
                    , [ 1.3978125300497049e-01, 6.2060435748053963e-02,  2.1011602986235331e-01]
            ]
        # compare the results to the truth values
        accuracy = 1e-6

        testFailCount, testMessages = unitTestSupport.compareArray(
            truePos, dataPosRed, accuracy, "r_BN_N Vector",
            testFailCount, testMessages)

        testFailCount, testMessages = unitTestSupport.compareArray(
            trueSigmaBN, dataSigmaBNRed, accuracy, "sigma_BN Set",
            testFailCount, testMessages)

        #   print out success message if no error were found
        if testFailCount == 0:
            print "PASSED "

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
         False        # useAltBodyFrame
       )

