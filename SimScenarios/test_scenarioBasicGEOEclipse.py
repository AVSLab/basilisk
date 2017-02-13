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
# Purpose:  Integrated test of the spacecraftPlus() and gravity modules.  Illustrates
#           a 6-DOF spacecraft in a typical GEO orbit.
# Author:   Patrick Kenneally
# Creation Date:  Feb. 8, 2017
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
bskPath = splitPath[0] + bskName + '/'
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
import gravityEffector
import spice_interface
import eclipse
import simIncludeGravity

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
# @pytest.mark.parametrize("useSphericalHarmonics, planetCase", [(False, 'Earth')])
# provide a unique test method name, starting with test_
def test_scenarioBasicGEOOrbit(show_plots):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots)
    assert testResults < 1, testMessage


def run(show_plots):
    '''Call this routine directly to run the tutorial scenario.'''
    #  Create a sim module as an empty container
    simContainer = SimulationBaseClass.SimBaseClass()
    simContainer.TotalSim.terminateSimulation()

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  create the simulation process
    dynProcess = simContainer.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(simContainer.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    # scObject.hub.useTranslation = True
    # scObject.hub.useRotation = False

    # add spacecraftPlus object to the simulation process
    simContainer.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    simIncludeGravity.clearSetup()

    # setup Gravity Bodies
    simIncludeGravity.addEarth()
    earth = simIncludeGravity.gravBodyList[-1]
    earth.isCentralBody = True
    simIncludeGravity.addSun()
    sun = simIncludeGravity.gravBodyList[-1]

    # simIncludeGravity.addMoon()
    # simIncludeGravity.addMars()
    # simIncludeGravity.addJupiter()

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)

    # setup SPICE interface
    simIncludeGravity.addSpiceInterface(bskPath, "2015 June 15, 00:00:00.0")
    simContainer.AddModelToTask(simTaskName, simIncludeGravity.spiceObject, None, 2)

    # by default the SPICE object will use the solar system barycenter as the inertial origin
    # If the spacecraftPlus() output is desired relative to another celestial object, the zeroBase string
    # name of the SPICE object needs to be changed.
    simIncludeGravity.spiceObject.zeroBase = 'Earth'

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rGEO = 42000.*1000  # meters
    oe.a = rGEO
    oe.e = 0.00001
    oe.i = 0.0*macros.D2R
    oe.Omega = 48.2*macros.D2R
    oe.omega = 0*macros.D2R
    oe.f = 0*macros.D2R
    # oe.f = 0
    rN, vN = orbitalMotion.elem2rv(earth.mu, oe)
    oe = orbitalMotion.rv2elem(earth.mu, rN, vN)

    # set the simulation time
    n = np.sqrt(earth.mu/oe.a/oe.a/oe.a)
    period = 2.*np.pi/n
    simulationTime = macros.sec2nano(10*period)

    #
    #   Setup data logging before the simulation is initialized
    numDataPoints = 400
    samplingTime = simulationTime / (numDataPoints-1)
    simContainer.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

    #
    #   initialize Simulation
    simContainer.InitializeSimulation()

    #
    #   initialize Spacecraft States within the state manager
    #   this must occur after the initialization
    #
    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    posRef.setState(unitTestSupport.np2EigenVectorXd(rN))  # [m - r_BN_N
    velRef.setState(unitTestSupport.np2EigenVectorXd(vN))  # m - v_BN_N
    simContainer.TotalSim.logThisMessage(eclipse.eclipseOutMsgIdAndName, samplingTime)

    #
    #   configure a simulation stop time time and execute the simulation run
    simContainer.ConfigureStopTime(simulationTime)
    simContainer.ExecuteSimulation()

    #
    #   retrieve the logged data
    posData = simContainer.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    velData = simContainer.pullMessageLogData(scObject.scStateOutMsgName+'.v_BN_N',range(3))
    srpForceData = simContainer.GetLogVariableData(srpDynEffector.ModelTag + ".forceExternal_B")
    srpTorqueData = simContainer.GetLogVariableData(srpDynEffector.ModelTag + ".torqueExternalPntB_B")

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]

    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(1,4):
        plt.plot(posData[:, 0]*macros.NANO2SEC/period, posData[:, idx]/1000.,
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$r_{BN,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')

    oeData = []
    for pos, vel in zip(posData, velData):
        oeData.append(orbitalMotion.rv2elem(mu_earth, pos[1:], vel[1:]))

    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    semiMajor = [oe.a/1000 for oe in oeData]
    plt.plot(posData[:, 0]*macros.NANO2SEC/period, semiMajor)
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Semi-major Axis [km]')

    plt.figure(4)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    srpForceMag = [np.linalg.norm(vec) for vec in srpForceData[:, 1:]]
    plt.plot(srpForceData[1:, 0] * macros.NANO2SEC / period, srpForceMag[1:])
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Magnitude SRP Force [N]')

    plt.figure(5)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    srpTorqueMag = [np.linalg.norm(vec) for vec in srpTorqueData[:, 1:]]
    plt.plot(srpTorqueData[1:, 0] * macros.NANO2SEC / period, srpTorqueMag[1:])
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Magnitude SRP Torque [Nm]')



    # only save off the figure if doing a unit test run
    # if doUnitTests:
    #     unitTestSupport.saveScenarioFigure(fileNameString+"2"+"GEO"+planetCase, plt, path)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    #
    #   the python code below is for the unit testing mode.  If you are studying the scenario
    #   to learn how to run BSK, you can stop reading below this line.
    #
    # if doUnitTests:
    #     numTruthPoints = 5
    #     skipValue = int(len(posData)/(numTruthPoints-1))
    #     dataPosRed = posData[::skipValue]
    #
    #     # setup truth data for unit test
    #     if orbitCase is 'LEO' and useSphericalHarmonics == False and planetCase is 'Earth':
    #         truePos = [
    #               [-2.8168016010234966e6,5.248174846916143e6,3.6771572646772987e6]
    #             , [-6.3710310400031125e6,-1.6053384413404597e6,2.4169406797143915e6]
    #             , [-1.970125344005881e6,-6.454584898598424e6,-1.8612676901068345e6]
    #             , [ 4.890526131271289e6,-3.2440700705588777e6,-3.815174368497354e6]
    #         ]
    #     if orbitCase is 'GTO' and useSphericalHarmonics == False and planetCase is 'Earth':
    #         truePos = [
    #               [-5.889529848066479e6,9.686574890007671e6,0.]
    #             , [-3.2026565710377645e7,-4.305001879844011e6,0.]
    #             , [-3.624269187139845e7,-1.8990291195663467e7,0.]
    #             , [-2.9802077401931673e7,-2.831957848900475e7,0.]
    #             , [-1.4932981196798025e7,-2.939523308237971e7,0.]
    #         ]
    #     if orbitCase is 'GEO' and useSphericalHarmonics == False and planetCase is 'Earth':
    #         truePos = [
    #               [-2.1819784817951165e7,3.588724145651873e7,0.]
    #             , [-4.16996933506621e7,-5.016611324503355e6,0.]
    #             , [-1.2686252555573342e7,-4.0038573722578734e7,0.]
    #             , [ 3.1201815137542922e7,-2.8114754297243357e7,0.]
    #             , [ 3.850428014786283e7,1.677456292503084e7,0.]
    #         ]
    #     if orbitCase is 'LEO' and useSphericalHarmonics == True and planetCase is 'Earth':
    #         truePos = [
    #               [-2.8168016010234915e6,5.248174846916147e6,3.677157264677297e6]
    #             , [ 5.787240887314784e6,3.7547029876434486e6,-1.1653623184693705e6]
    #             , [ 2.5908823579481775e6,-5.38042751586389e6,-3.6401355110844015e6]
    #             , [-5.905673984221732e6,-3.5332208726054016e6,1.2748483822117285e6]
    #             , [-2.3741237403798397e6,5.508156976353034e6,3.6085612280591857e6]
    #         ]
    #     if orbitCase is 'LEO' and useSphericalHarmonics == False and planetCase is 'Mars':
    #         truePos = [
    #               [-2.8168016010234966e6,5.248174846916143e6,3.6771572646772987e6]
    #             , [-6.370345938284969e6,-1.6147054668864955e6,2.412504030081398e6]
    #             , [-1.9520854768447054e6,-6.457181115789631e6,-1.8712382659451987e6]
    #             , [ 4.90876381054031e6,-3.2188851633259663e6,-3.8130784005532693e6]
    #         ]
    #
    #     # compare the results to the truth values
    #     accuracy = 1.0  # meters
    #
    #     testFailCount, testMessages = unitTestSupport.compareArray(
    #         truePos, dataPosRed, accuracy, "r_BN_N Vector",
    #         testFailCount, testMessages)
    #
    #     #   print out success message if no error were found
    #     if testFailCount == 0:
    #         print "PASSED "
    #     else:
    #         print testFailCount
    #         print testMessages
    #
    # # each test method requires a single assert method to be called
    # # this check below just makes sure no sub-test failures were found
    # return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(True, False, 'Earth')
    # do test, show_plots, planetCase (Earth, Mars)
