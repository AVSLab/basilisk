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
#           MRP_Feedback() with attitude navigation modules.  Illustrates how
#           attitude guidance behavior can be changed in a very modular manner.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 2, 2016
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
import hillPoint
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
def test_bskAttitudeGuidance(show_plots, useAltBodyFrame):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, useAltBodyFrame)
    assert testResults < 1, testMessage


## \defgroup Tutorials_2_1
##   @{
## How to use guidance modules to align the spacecraft frame to the orbit or Hill frame.
#
# Attitude Alignment with Hill Orbit Frame {#scenarioAttitudeGuidance}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The scenario is
# setup to be run in two different setups:
# Setup | useAltBodyFrame
# ----- | -------------------
# 1     | False
# 2     | True
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioAttitudeGuidance.py
#
# The simulation layout is shown in the following illustration.  A single simulation process is created
# which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
# modules.
# ![Simulation Flow Diagram](Images/doc/test_scenarioAttitudeGuidance.svg "Illustration")
#
# When the simulation completes 4 plots are shown for the MRP attitude history, the rate
# tracking errors, the control torque vector, as well as the projection of the body-frame B
# axes \f$\hat b_1\f$, b2 and b3 onto the respect Hill or Orbit frame axes \f$\hat\imath_r\f$,
# \f$\hat\imath_{\theta}\f$ and \f$\hat\imath_h\f$.  This latter plot illustrates how the body
# is being aligned with respect to this Hill frame.
#
# The basic simulation setup is the same as the one used in
# [test_scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback).
# The dynamics simulation is setup using a SpacecraftPlus() module to which a gravity
# effector is attached.  Note that both the rotational and translational degrees of
# freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.  For more
# information on how to setup orbit, see [test_scenarioBasicOrbit.py](@ref scenarioBasicOrbit)
#
# However, instead of doing an inertial pointing maneuver, here the hillFrame() attitude guidance module
# is used:
# ~~~~~~~~~~~~~{.py}
#     attGuidanceConfig = hillPoint.hillPointConfig()
#     attGuidanceWrap = scSim.setModelDataWrap(attGuidanceConfig)
#     attGuidanceWrap.ModelTag = "hillPoint"
#     attGuidanceConfig.inputNavDataName = sNavObject.outputTransName
#     attGuidanceConfig.inputCelMessName = earth.bodyInMsgName
#     attGuidanceConfig.outputDataName = "guidanceOut"
#     scSim.AddModelToTask(simTaskName, attGuidanceWrap, attGuidanceConfig)
# ~~~~~~~~~~~~~
#
# In contrast to the simple inertial pointing guidance module, this module also requires the
# spacecraft's position and velocity information.  The planet ephemeris message relative to which the Hill pointing
# is being achieved by setting the `inputCelMessName` message.
# This is useful, for example, if orbiting the sun, and wanting to point the spacecraft back at the
# Earth which is also orbiting the sun.  In this scenario, however, the spacecraft is to point at the
# Earth while already orbiting the Earth.  Thus, this planet ephemeris input message is not set, which
# in return zeros the planets position and velocity vector states in the guidance module.
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
# The first 2 arguments can be left as is.  The remaining argument(s) control the
# simulation scenario flags to turn on or off certain simulation conditions.  The
# default scenario shown has the `useAltBodyFrame` flag turned off.  This means that we seek
# to align the body frame *B* with the Hill reference frame *R*.    The
# resulting attitude and control torque histories are shown below.  Note that the projections
# of the body frame axes onto the Hill frame axes all converge to +1, indidcating that B becomes
# asympotically aligned with R as desired.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeGuidance10.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeGuidance20.svg "Torque history")
# ![Body/Hill Frame Axis Projections](Images/Scenarios/scenarioAttitudeGuidance40.svg "Axes Projection")
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
# Here the control should not align the principal body frame *B* with *R*, but rather an alternate,
# corrected body frame *Bc*.  For example, consider the Earth observing sensors to be mounted pointing in the
# positive \f$\hat b_1\f$ direction. In scenario 1 this sensor platform is actually pointing away from
# the Earth.  Thus, we define the corrected body frame orientation as a 180 deg rotation about
# \f$\hat b_2\f$.  This flips the orientation of the final first and third body axis.  This is achieved
# through:
# ~~~~~~~~~~~~~{.py}
#   attErrorConfig.sigma_R0R = [0,1,0]
# ~~~~~~~~~~~~~
#   The DCM \f$[R_0R]\f$ is the same as the body to corrected body DCM \f$[B_cB]\f$.
# The resulting attitude and control torque histories are shown below.  Note that the projections
# of the 2nd body frame axis onto the 2nd Hill frame axes converges to +1, while the other
# projections converge to -1.  This indicates that the desired asymptotic Earth observing attitude
# is achieved.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeGuidance11.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeGuidance21.svg "Torque history")
# ![Body/Hill Frame Axis Projections](Images/Scenarios/scenarioAttitudeGuidance41.svg "Axes Projection")
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
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.1)
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
    oe.a     = 10000000.0           # meters
    oe.e     = 0.1
    oe.i     = 33.3*macros.D2R
    oe.Omega = 48.2*macros.D2R
    oe.omega = 347.8*macros.D2R
    oe.f     = 85.3*macros.D2R
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

    # setup hillPoint guidance module
    attGuidanceConfig = hillPoint.hillPointConfig()
    attGuidanceWrap = scSim.setModelDataWrap(attGuidanceConfig)
    attGuidanceWrap.ModelTag = "hillPoint"
    attGuidanceConfig.inputNavDataName = sNavObject.outputTransName
    attGuidanceConfig.inputCelMessName = earth.bodyInMsgName
    attGuidanceConfig.outputDataName = "guidanceOut"
    scSim.AddModelToTask(simTaskName, attGuidanceWrap, attGuidanceConfig)

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.outputDataName = "attErrorMsg"
    if useAltBodyFrame:
        attErrorConfig.sigma_R0R = [0,1,0]
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

    vectorPosData = unitTestSupport.pullVectorSetFromData(dataPos)
    vectorVelData = unitTestSupport.pullVectorSetFromData(dataVel)
    vectorMRPData = unitTestSupport.pullVectorSetFromData(dataSigmaBN)
    data = np.empty([len(vectorPosData),3])
    for idx in range(0,len(vectorPosData)):
        ir = vectorPosData[idx] / np.linalg.norm(vectorPosData[idx])
        hv = np.cross(vectorPosData[idx], vectorVelData[idx])
        ih = hv / np.linalg.norm(hv)
        itheta = np.cross(ih,ir)
        dcmBN = RigidBodyKinematics.MRP2C(vectorMRPData[idx])
        data[idx] = [np.dot(ir, dcmBN[0]), np.dot(itheta, dcmBN[1]), np.dot(ih, dcmBN[2])]
    plt.figure(4)
    labelStrings = (r'$\hat\imath_r\cdot \hat b_1$'
                    , r'${\hat\imath}_{\theta}\cdot \hat b_2$'
                    , r'$\hat\imath_h\cdot \hat b_3$')
    for idx in range(0,3):
        plt.plot(timeLineSet, data[:, idx],
                 color=unitTestSupport.getLineColor(idx+1,3),
                 label=labelStrings[idx])
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Orientation Illustration')
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
    #
    if doUnitTests:
        numTruthPoints = 5
        skipValue = int(numDataPoints/numTruthPoints)
        dataSigmaBNRed = dataSigmaBN[::skipValue]
        dataPosRed = dataPos[::skipValue]


        # setup truth data for unit test
        truePos = [
                      [-3.9514176198221971e+06, 7.3621552027224889e+06, 5.1583270902798297e+06]
                    , [-4.6086027653051550e+06, 6.9672123694011206e+06, 5.3072237697180342e+06]
                    , [-5.2376104604200358e+06, 6.5296380777946832e+06, 5.4236570091631645e+06]
                    , [-5.8353304991197446e+06, 6.0530297355945092e+06, 5.5076788366235951e+06]
                    , [-6.3989830900466563e+06, 5.5410585900721485e+06, 5.5595354088057131e+06]
                ]
        trueLr = trueSigmaBR = []
        if useAltBodyFrame == True:
            trueSigmaBN = [
                      [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                    , [-6.4143845119742271e-01, 3.7549202067008880e-01, 1.6228422035818663e-01]
                    , [-8.2514275559858030e-01, 3.7431052486815464e-01, 2.6641953651279665e-01]
                    , [-8.0514621677426934e-01, 3.3744944030160068e-01, 2.4586406789433021e-01]
                    , [-8.1316266101544810e-01, 3.0421565940809858e-01, 2.4203891375413897e-01]
            ]
        if useAltBodyFrame == False:
            trueSigmaBN = [
                      [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                    , [ 1.9757381842655744e-01,-1.8325113332909412e-02, 5.3116118128700796e-01]
                    , [ 1.9401404616468543e-01,-6.2047093744322206e-02, 6.2244720069697612e-01]
                    , [ 1.9788907419526672e-01,-6.8298668119320893e-02, 6.4548524709461186e-01]
                    , [ 1.9984147378665409e-01,-7.7874650384175126e-02, 6.7169950976963932e-01]
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

