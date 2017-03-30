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
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in deep space.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 19, 2016
#



import pytest
import sys, os, inspect
import matplotlib
import numpy as np


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

# import simulation related support
import spacecraftPlus
import ExtForceTorque
import simple_nav

# import FSW Algorithm related support
import MRP_Feedback
import inertial3D
import attTrackingError

# import message declarations
import fswMessages

# import Viz messaging interface
import vis_message_interface





# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useLargeTumble", [
      (False)
    , (True)
])

# provide a unique test method name, starting with test_
def test_bskAttitudePointing(show_plots, useLargeTumble):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, useLargeTumble)
    assert testResults < 1, testMessage



## \defgroup Tutorials_2_1
##   @{
## Demonstrates how to stabilize the tumble without translational motion.
#
# Pure Attitude Detumbling Simulation in a Single Simulation Process {#scenarioAttitudePointing}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft, but without specifying any orbital motion.  Thus,
# this scenario simulates the spacecraft translating in deep space.  The scenario is a simplified
# version of [test_scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback) with the orbital setup
# removed.  The scenario is
# setup to be run in 2 different setups:
# Setup | useLargeTumble
# ----- | -------------------
# 1     | False
# 2     | True
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioAttitudePointing.py
#
# As with [test_scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback), when
# the simulation completes 3 plots are shown for the MRP attitude history, the rate
# tracking errors, as well as the control torque vector.
#
# Besides removing any orbit related code, the only difference is to turn of
# the spacecraftPlus() translational simulation mode.  This is done with:
# ~~~~~~~~~~~~~~~~{.py}
#     scObject.hub.useTranslation = False
#     scObject.hub.useRotation = True
# ~~~~~~~~~~~~~~~~
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          False        # useLargeTumble
#        )
# ~~~~~~~~~~~~~
# Here a small initial tumble is simulated.  The
# resulting attitude and control torque histories are shown below.  The spacecraft quickly
# regains a stable orientation without tumbling past 180 degrees.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudePointing10.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudePointing20.svg "Torque history")
#
# Setup 2
# ------
#
# Here the python main function is changed to read:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          True         # useLargeTumble
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  Note that, as expected,
# the orientation error tumbles past 180 degrees before stabilizing to zero.  The control
# torque effort is also much larger in this case.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudePointing11.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudePointing21.svg "Torque history")
#
#
##  @}
def run(doUnitTests, show_plots, useLargeTumble):
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

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    dynProcess.addTask(scSim.CreateNewTask("VisTask", macros.sec2nano(0.1)))
    viz = vis_message_interface.VisMessageInterface()
    scSim.AddModelToTask("VisTask", viz)

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
    scObject.hub.useTranslation = False
    scObject.hub.useRotation = True

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)


    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = ExtForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
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
    mrpControlConfig.Ki =   -1          # make value negative to turn off integral feedback
    mrpControlConfig.P  = 30.0
    mrpControlConfig.integralLimit = 2./mrpControlConfig.Ki * 0.1
    mrpControlConfig.domega0 = [0.0, 0.0, 0.0]



    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 50
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(mrpControlConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)



    #
    # create simulation messages
    #
    # simIncludeGravity.addDefaultEphemerisMsg(scSim.TotalSim, simProcessName)

    # create the FSW vehicle configuration message
    vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
    vehicleConfigOut.ISCPntB_B = I      # use the same inertia in the FSW algorithm as in the simulation
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               mrpControlConfig.vehConfigInMsgName,
                               vehicleConfigOut)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()


    #
    #   initialize Spacecraft States within the state manager
    #   this must occur after the initialization
    #
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")

    sigmaRef.setState([[0.1], [0.2], [-0.3]])       # sigma_BN_B
    if useLargeTumble:
        omegaRef.setState([[0.8], [-0.6], [0.5]])  # rad/s - omega_BN_B
    else:
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
    np.set_printoptions(precision=16)


    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]
    plt.close("all")        # clears out plots from earlier test runs
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
            fileNameString+"1"+str(int(useLargeTumble))
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
            fileNameString+"2"+str(int(useLargeTumble))
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

        trueLr = trueSigmaBR = []

        if useLargeTumble == True:
            trueLr = [
                  [-2.4350000000000001e+01, 1.7300000000000001e+01,-1.3949999999999999e+01]
                , [-1.8261025223247096e-01,-1.9802131477666673e-01,-2.2905552303763882e-01]
                , [-2.2703347936179175e-02, 2.8322384043503845e-02,-7.5383083954013580e-03]
                , [ 3.9685083651031109e-03,-4.6726997381575461e-03, 9.1702648415809018e-04]
                , [-6.5254418265915193e-04, 6.1478222187531318e-04,-6.2014699070663979e-05]
            ]
            trueSigmaBR = [
                  [ 1.0000000000000001e-01, 2.0000000000000001e-01,-2.9999999999999999e-01]
                , [ 1.5260971061679154e-01,-2.6346123607709682e-01, 1.9116787137307839e-01]
                , [-1.8707224538059040e-02, 1.9073543274306739e-02,-9.2763187341566734e-03]
                , [ 2.1576458281319776e-03,-1.5090989414394025e-03, 3.2041623116321299e-04]
                , [-2.4360871626616175e-04, 1.0266828769375566e-04,-7.5369979791638928e-06]
            ]
        if useLargeTumble == False:
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

      # compare the results to the truth values
        accuracy = 1e-6

        testFailCount, testMessages = unitTestSupport.compareArray(
            trueLr, dataLrRed, accuracy, "Lr Vector",
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
         True         # useLargeTumble
       )

