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
# Purpose:  Demonstrates how to setup CSS sensors on a rigid spacecraft
# Author:   Hanspeter Schaub
# Creation Date:  July 21, 2017
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



 


# @endcond

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.simulation import coarse_sun_sensor

# import simulation related support
from Basilisk.simulation import spacecraftPlus

# import message declarations
from Basilisk.simulation import simMessages





# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useCSSConstellation, usePlatform, useEclipse, useKelly", [
      (False, False, False, False)
    , (False, True, False, False)
    , (False, False, True, False)
    , (False, False, False, True)
    , (True, False, False, False)
])

# provide a unique test method name, starting with test_
def test_bskAttitudeFeedback(show_plots, useCSSConstellation, usePlatform, useEclipse, useKelly):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, useCSSConstellation, usePlatform, useEclipse, useKelly)
    assert testResults < 1, testMessage



## \defgroup Tutorials_4_0
##   @{
## Demonstrates how to add a Coarse Sun Sensor (CSS) sensor to a spacecraft.
#
# Coarse Sun Sensor (CSS) Simulation{#scenarioCSS}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft in deep space without any gravitational bodies. Only rotational
# motion is simulated.  The script illustrates how to setup CSS sensor units and log their data.  It is possible
# to setup individual CSS sensors, or setup a constellation or array of CSS sensors.  The scenario is
# setup to be run in four different setups:
# Setup | useCSSConstellation  | usePlatform  | useEclipse | useKelly
# ----- | -------------------- | ------------ | -----------|---------
# 1     | False                | False        | False      | False
# 2     | False                | True         | False      | False
# 3     | False                | False        | True       | False
# 4     | False                | False        | False      | True
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioCSS.py
#
# When the simulation completes a plot is shown for the CSS sensor signal history.
#
# The simulation layout options (A) and (B) are shown in the following illustration.  A single simulation process is created
# which contains both the spacecraft simulation module, as well as two individual CSS sensor units.  In scenario (A)
# the CSS units are individually executed by the simulation, while scenario (B) uses a CSS constellation class
# that executes a list of CSS evaluations at the same time.
# ![Simulation Flow Diagrams](Images/doc/test_scenarioCSS.svg "Illustration")
#
#
# The dynamics simulation is setup using a SpacecraftPlus() module where a specific spacecraft location
# is specified.  Note that both the rotational and translational degrees of
# freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.  The position
# vector is required when computing the relative heading between the sun and the spacecraft locations.  The
# spacecraft position is held fixed, while the orientation rotates constantly about the 3rd body axis.
# ~~~~~~~~~~~~~~~~{.py}
#     scObject.hub.r_CN_NInit = [[1000.0], [0.0], [0.0]]              # m   - r_CN_N
#     scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]                 # m/s - v_CN_N
#     scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]               # sigma_BN_B
#     scObject.hub.omega_BN_BInit = [[0.0], [0.0], [1.*macros.D2R]]   # rad/s - omega_BN_B
# ~~~~~~~~~~~~~~~~
#
# In both CSS simulation scenarios (A) and (B) the CSS modules must first be individuall created and configured.
# In this simulation each case uses two CSS sensors.  The minimum variables that must be set for each CSS
# includes:
# ~~~~~~~~~~~~~~~~{.py}
#     CSS1 = coarse_sun_sensor.CoarseSunSensor()
#     CSS1.ModelTag = "CSS1_sensor"
#     CSS1.fov = 80.*macros.D2R
#     CSS1.scaleFactor = 2.0
#     CSS1.OutputDataMsg = "CSS1_output"
#     CSS1.InputSunMsg = "sun_message"
# ~~~~~~~~~~~~~~~~
# The Field-Of-View variable fov must be specified.  This is the angle between the sensor bore-sight and
# the edge of the field of view.  Beyond this angle all sensor signals are set to zero. The
# scaleFactor variable scales a normalized CSS response to this value if facing the sun head on.
# The input message name InputSunMsg specifies an input message that contains the sun's position.
# If sensor
# corruptions are to be modeled, this can be set through the variables:
# ~~~~~~~~~~~~~~~~{.py}
#   CSS1.KellyFactor
#   CSS1.SenBias
#   CSS1.SenNoiseStd
# ~~~~~~~~~~~~~~~~
# The Kelly factor has values between 0 (off) and 1 and distorts the nominal cosine response.  The SenBias
# variable determines a normalized bias to be applied to the CSS model, and SenNoiseStd provides Gaussian noise.
#
# To create additional CSS sensor units, copies of the first CSS unit can be made.  This means only the parameters
# different in the additional units must be set.
# ~~~~~~~~~~~~~~~~{.py}
#   CSS2 = coarse_sun_sensor.CoarseSunSensor(CSS1)      # make copy of first CSS unit
#   CSS2.ModelTag = "CSS2_sensor"
#   CSS2.OutputDataMsg = "CSS2_output"
# ~~~~~~~~~~~~~~~~
#
# A key parameter that remains is the CSS sensor unit normal vector.  There are several options to set this
# vector (in body frame components).  The first method is to set \f$\hat{\mathbf n}\f$ or nHat_B directly.  This is
# done with:
# ~~~~~~~~~~~~~~~~{.py}
#   CSS1.nHat_B = [1.0, 0.0, 0.0]
#   CSS2.nHat_B = [0.0, -1.0, 0.0]
# ~~~~~~~~~~~~~~~~
# Another option is to use a frame associated relative to a common CSS platform \f$\cal P\f$.  The bundled CSS units are
# often symmetrically arranged on a platform such as in a pyramid configuration.  The the platform frame is
# specified through
# ~~~~~~~~~~~~~~~~{.py}
#   CSS1.setBodyToPlatformDCM(90.*macros.D2R, 0., 0.)
# ~~~~~~~~~~~~~~~~
# where the three orientation angles are 3-2-1 Euler angles.  These platform angles are initialized to zero.
# Next, the CSS unit direction vectors can be specified through the azimuth and elevation angles
# (\f$\phi\f$, \f$\theta\f$).  These are (3)-(-2) Euler angles.
# ~~~~~~~~~~~~~~~~{.py}
#   CSS1.phi = 90.*macros.D2R
#   CSS1.theta = 0.*macros.D2R
# ~~~~~~~~~~~~~~~~
# If no platform orientation is specified, then naturally these azimuth and elevation angles are
# measured relative to the body frame \f$\cal B\f$.
#
# An optional input message is the solar eclipse message.  If this message input name is specified for a CSS
# unit, then the eclipse information is taken into account.  If this message name is not set, then the CSS
# defaults to the spacecraft always being in the sun.
# ~~~~~~~~~~~~~~~~{.py}
#   CSS1.sunEclipseInMsgName = "eclipse_message"
# ~~~~~~~~~~~~~~~~
#
# In this scenario (A) setup the CSS unit are each evaluated separately through
# ~~~~~~~~~~~~~~~~{.py}
#   scSim.AddModelToTask(simTaskName, CSS1)
#   scSim.AddModelToTask(simTaskName, CSS2)
# ~~~~~~~~~~~~~~~~
# This means that each CSS unit creates a individual output messages.
#
# If instead a cluster of CSS units is to be evaluated as one, then the above individual CSS units
# can be grouped into a list, and added to the Basilisk execution stack as a single entity.  This is done with
# ~~~~~~~~~~~~~~~~{.py}
#   cssList = [CSS1, CSS2]
#   cssArray = coarse_sun_sensor.CSSConstellation()
#   cssArray.ModelTag = "css_array"
#   cssArray.sensorList = coarse_sun_sensor.CSSVector(cssList)
#   cssArray.outputConstellationMessage = "CSS_Array_output"
#   scSim.AddModelToTask(simTaskName, cssArray)
# ~~~~~~~~~~~~~~~~
# Here the CSSConstellation() module will call the individual CSS update functions, collect all the sensor
# signals, and store the output in a single output message containing an array of CSS sensor signals.
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          False,       # useCSSConstellation
#          False,       # usePlatform
#          False,       # useEclipse
#          False        # useKelly
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The remaining arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  This scenario
# simulates the CSS units being setup individually without any corruption.  The sensor unit normal
# axes are directly set, and no eclipse is modeled.  The
# resulting CSS sensor histories are shown below.
# ![CSS Sensor History](Images/Scenarios/scenarioCSS0000.svg "CSS history")
# The signals of the two CSS units range from a maximum of 2 if the CSS axis is pointing at the sun to zero.
# The limited field of view of 80 degrees causes the sensor signal to be clipped when the sun light incidence
# angle gets too small.
#
# Setup 2
# ------
#
# Here the python main function is changed to read:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          False,       # useCSSConstellation
#          True,        # usePlatform
#          False,       # useEclipse
#          False        # useKelly
#        )
# ~~~~~~~~~~~~~
# The resulting CSS sensor signals should be identical to the first scenario as the chosen
# platform orientation and CSS azimuth and elevation angles are chosen to yield the same
# senor normal unit axes.
# ![CSS Sensor History](Images/Scenarios/scenarioCSS0100.svg "CSS history")
#
# Setup 3
# ------
#
# The 3rd scenario connects a solar eclipse message to the CSS units through:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          False,       # useCSSConstellation
#          False,       # usePlatform
#          True,        # useEclipse
#          False        # useKelly
#        )
# ~~~~~~~~~~~~~
# The resulting CSS signals are scaled by a factor of 0.5 and are shown below.
# ![CSS Sensor History](Images/Scenarios/scenarioCSS0010.svg "CSS history")
#
# Setup 4
# ------
#
# The 4th scenario turns on Kelly corruption factor of the CSS units.
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          False,       # useCSSConstellation
#          False,       # usePlatform
#          False,       # useEclipse
#          True         # useKelly
#        )
# ~~~~~~~~~~~~~
# This causes the CSS signals to become slightly warped, and depart from the nominal cosine
# behavior.  The resulting simulation results are shown below.
# ![CSS Sensor History](Images/Scenarios/scenarioCSS0001.svg "CSS history")
#
# Setup 5
# ------
#
# The 5th scenario is identical to setup 1, but here the 2 CSS units are packaged inside the
# CSSConstellation() class.
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          True,        # useCSSConstellation
#          False,       # usePlatform
#          False,       # useEclipse
#          False        # useKelly
#        )
# ~~~~~~~~~~~~~
# The resulting simulation results are shown below to be identical to setup 1 as expected.
# ![CSS Sensor History](Images/Scenarios/scenarioCSS1000.svg "CSS history")
#

##  @}
def run(doUnitTests, show_plots, useCSSConstellation, usePlatform, useEclipse, useKelly):
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
    simulationTime = macros.sec2nano(120.)

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

    #
    # set initial spacecraft states
    #
    scObject.hub.r_CN_NInit = [[1000.0], [0.0], [0.0]]              # m   - r_CN_N
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]                 # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]               # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [1.*macros.D2R]]   # rad/s - omega_BN_B


    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # create CSS sensors
    CSS1 = coarse_sun_sensor.CoarseSunSensor()
    CSS1.ModelTag = "CSS1_sensor"
    CSS1.fov = 80.*macros.D2R
    CSS1.scaleFactor = 2.0
    CSS1.OutputDataMsg = "CSS1_output"
    CSS1.InputSunMsg = "sun_message"
    if useKelly:
        CSS1.KellyFactor = 0.2
    if useEclipse:
        CSS1.sunEclipseInMsgName = "eclipse_message"
    if usePlatform:
        CSS1.setBodyToPlatformDCM(90.*macros.D2R, 0., 0.)
        CSS1.theta = -90.*macros.D2R
        CSS1.phi = 0*macros.D2R
        CSS1.setUnitDirectionVectorWithPerturbation(0., 0.)
    else:
        CSS1.nHat_B = [1.0, 0.0, 0.0]

    CSS2 = coarse_sun_sensor.CoarseSunSensor(CSS1)      # make copy of first CSS unit
    CSS2.ModelTag = "CSS2_sensor"
    CSS2.OutputDataMsg = "CSS2_output"
    if usePlatform:
        CSS2.theta = 180.*macros.D2R
        CSS2.setUnitDirectionVectorWithPerturbation(0., 0.)
    else:
        CSS2.nHat_B = [0.0, -1.0, 0.0]

    if useCSSConstellation:
        cssList = [CSS1, CSS2]
        cssArray = coarse_sun_sensor.CSSConstellation()
        cssArray.ModelTag = "css_array"
        cssArray.sensorList = coarse_sun_sensor.CSSVector(cssList)
        cssArray.outputConstellationMessage = "CSS_Array_output"
        scSim.AddModelToTask(simTaskName, cssArray)
    else:
        scSim.AddModelToTask(simTaskName, CSS1)
        scSim.AddModelToTask(simTaskName, CSS2)

    #
    #   Setup data logging before the simulation is initialized
    #
    if useCSSConstellation:
        scSim.TotalSim.logThisMessage(cssArray.outputConstellationMessage, simulationTimeStep)
    else:
        scSim.TotalSim.logThisMessage(CSS1.OutputDataMsg, simulationTimeStep)
        scSim.TotalSim.logThisMessage(CSS2.OutputDataMsg, simulationTimeStep)



    #
    # create simulation messages
    #
    sunPositionMsg = simMessages.SpicePlanetStateSimMsg()
    sunPositionMsg.PositionVector = [2000.0, 0.0, 0.0]
    unitTestSupport.setMessage( scSim.TotalSim,
                                simProcessName,
                                CSS1.InputSunMsg,
                                sunPositionMsg)

    if useEclipse:
        eclipseMsg = simMessages.EclipseSimMsg()
        eclipseMsg.shadowFactor = 0.5
        unitTestSupport.setMessage(scSim.TotalSim,
                                   simProcessName,
                                   CSS1.sunEclipseInMsgName,
                                   eclipseMsg)
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
    if useCSSConstellation:
        dataCSSArray = scSim.pullMessageLogData(cssArray.outputConstellationMessage+".CosValue", range(len(cssList)))
    else:
        dataCSS1 = scSim.pullMessageLogData(CSS1.OutputDataMsg+".OutputData", range(1))
        dataCSS2 = scSim.pullMessageLogData(CSS2.OutputDataMsg+".OutputData", range(1))
    np.set_printoptions(precision=16)


    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]
    print fileNameString
    plt.close("all")        # clears out plots from earlier test runs
    plt.figure(1)
    if useCSSConstellation:
        for idx in range(1, len(cssList)+1):
            plt.plot(dataCSSArray[:, 0]*macros.NANO2SEC, dataCSSArray[:, idx],
                         color=unitTestSupport.getLineColor(idx,3),
                         label='CSS$_{'+str(idx)+'}$')
    else:
        plt.plot(dataCSS1[:, 0]*macros.NANO2SEC, dataCSS1[:, 1],
                 color=unitTestSupport.getLineColor(1,3),
                 label='CSS$_{1}$')
        plt.plot(dataCSS2[:, 0] * macros.NANO2SEC, dataCSS2[:, 1],
                 color=unitTestSupport.getLineColor(2, 3),
                 label='CSS$_{2}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('CSS Signals ')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+str(int(useCSSConstellation))+str(int(usePlatform))
            +str(int(useEclipse))+str(int(useKelly))
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
        skipValue = int(simulationTime*macros.NANO2SEC/numTruthPoints)
        if useCSSConstellation:
            dataCSSArrayRed = dataCSSArray[::skipValue]
            trueCSS = [
                  [ 2.0000000000000000e+00, 0.0000000000000000e+00]
                , [ 1.8270909152861943e+00, 8.1347328614937120e-01]
                , [ 1.3382612127209323e+00, 1.4862896509518926e+00]
                , [ 6.1803398875474258e-01, 1.9021130325887317e+00]
                , [ 0.0000000000000000e+00, 1.9890437907369840e+00]
                , [ 0.0000000000000000e+00, 1.7320508075693963e+00]
            ]

        else:
            dataCSS1red = dataCSS1[::skipValue]
            dataCSS2red = dataCSS2[::skipValue]

            # setup truth data for unit test
            if useEclipse == False and useKelly==False:
                trueCSS1 = [
                      [ 2.0000000000000000e+00]
                    , [ 1.8270909152861943e+00]
                    , [ 1.3382612127209323e+00]
                    , [ 6.1803398875474258e-01]
                    , [ 0.0000000000000000e+00]
                    , [ 0.0000000000000000e+00]
                ]
                trueCSS2 = [
                      [ 0.0000000000000000e+00]
                    , [ 8.1347328614937120e-01]
                    , [ 1.4862896509518926e+00]
                    , [ 1.9021130325887317e+00]
                    , [ 1.9890437907369840e+00]
                    , [ 1.7320508075693963e+00]
                ]
            if usePlatform == False and useEclipse == True and useKelly==False:
                trueCSS1 = [
                      [ 1.0000000000000000e+00]
                    , [ 9.1354545764309714e-01]
                    , [ 6.6913060636046617e-01]
                    , [ 3.0901699437737129e-01]
                    , [ 0.0000000000000000e+00]
                    , [ 0.0000000000000000e+00]
                ]
                trueCSS2 = [
                      [ 0.0000000000000000e+00]
                    , [ 4.0673664307468560e-01]
                    , [ 7.4314482547594629e-01]
                    , [ 9.5105651629436583e-01]
                    , [ 9.9452189536849200e-01]
                    , [ 8.6602540378469817e-01]
                ]
            if usePlatform == False and useEclipse == False and useKelly==True:
                trueCSS1 = [
                      [ 1.9865241060018290e+00]
                    , [ 1.7989379186517085e+00]
                    , [ 1.1956035766929121e+00]
                    , [ 2.3463126305909773e-01]
                    , [ 0.0000000000000000e+00]
                    , [ 0.0000000000000000e+00]
                ]
                trueCSS2 = [
                      [ 0.0000000000000000e+00]
                    , [ 4.5775481598362727e-01]
                    , [ 1.3923439498033288e+00]
                    , [ 1.8814534722288898e+00]
                    , [ 1.9748891817681278e+00]
                    , [ 1.6913168768673754e+00]
                ]
        # compare the results to the truth values
        accuracy = 1e-6

        if useCSSConstellation:
            testFailCount, testMessages = unitTestSupport.compareArrayND(
                trueCSS, dataCSSArrayRed, accuracy, "CSSarray", 2,
                testFailCount, testMessages)
        else:
            testFailCount, testMessages = unitTestSupport.compareDoubleArray(
                trueCSS1, dataCSS1red, accuracy, "CSS1",
                testFailCount, testMessages)
            testFailCount, testMessages = unitTestSupport.compareDoubleArray(
                trueCSS2, dataCSS2red, accuracy, "CSS2",
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
         False,       # useCSSConstellation
         False,       # usePlatform
         False,       # useEclipse
         False        # useKelly
       )

