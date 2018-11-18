''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
# Basilisk Integrated Test of the Solar Radiation Pressure Evaluation
#
# Purpose:  Integrated test of the spacecraftPlus(), gravity modules and the solar
#           radiation pressure modeling.  Currently the cannonball model is only tested.
# Author:   Patrick Kenneally
# Creation Date:  June 11, 2018
#

import numpy as np
import matplotlib.pyplot as plt
import os
from Basilisk import __path__
bskPath = __path__[0]
from Basilisk.simulation import spacecraftPlus, radiation_pressure, spice_interface
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                unitTestSupport)
from Basilisk.utilities.simIncludeGravBody import gravBodyFactory

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_radiationPressureIntegratedTest(show_plots):
    [testResults, testMessage] = radiationPressureIntegratedTest(show_plots)
    assert testResults < 1, testMessage


def radiationPressureIntegratedTest(show_plots):
    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()
    sim.TotalSim.terminateSimulation()

    dynProcess = sim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.0)
    dynProcess.addTask(sim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    sim.AddModelToTask(simTaskName, scObject)

    srp = radiation_pressure.RadiationPressure() # default model is the SRP_CANNONBALL_MODEL
    srp.area = 1.0
    srp.coefficientReflection = 1.3
    sim.AddModelToTask(simTaskName, srp, None, -1)
    scObject.addDynamicEffector(srp)

    # setup Gravity Body
    gravFactory = gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True
    mu = planet.mu
    spice_path = bskPath + '/supportData/EphemerisData/'
    spice = gravFactory.createSpiceInterface(spice_path,
                                                '2021 MAY 04 07:47:49.965 (UTC)')
    spice.planetNames.append("sun")
    spice.ModelTag = "SpiceInterfaceData"
    spice.outputBufferCount = 100000
    sim.AddModelToTask(simTaskName, spice, None, -1)

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rGEO = 42000. * 1000     # meters
    oe.a = rGEO
    oe.e = 0.00001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
    # with circular or equatorial orbit, some angles are arbitrary

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(P)

    #   Setup data logging before the simulation is initialized
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    sim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    sim.TotalSim.logThisMessage('earth_planet_data', samplingTime)

    #
    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    #
    sim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    sim.ConfigureStopTime(simulationTime)
    sim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    earthEphm = sim.pullMessageLogData("earth_planet_data.PositionVector", range(3))
    posData = sim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))

    pos_rel_earth = posData[:, 1:4] - earthEphm[:, 1:4]
    pos_rel_earth = np.insert(pos_rel_earth, 0, posData[:, 0], axis=1)
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    numTruthPoints = 10
    skipValue = int(len(pos_rel_earth) / (numTruthPoints - 1))
    pos_rel_earth_parse = pos_rel_earth[::skipValue]
    print "earth pos:"
    print pos_rel_earth_parse

    # true position for un perturbed 2 body GEO orbit with cannonball SRP
    true_pos = np.array([[ -2.18197848e+07,  3.58872415e+07,  0.00000000e+00],
                     [ -3.98343483e+07,  1.33137624e+07, -7.09098053e+01],
                     [ -3.90149081e+07, -1.55551455e+07, -2.50040489e+02],
                     [ -1.97502034e+07, -3.70704301e+07, -4.53006371e+02],
                     [ 8.85077806e+06, -4.10621923e+07, -5.84144691e+02],
                     [ 3.32681060e+07, -2.56454922e+07, -5.81810165e+02],
                     [ 4.19603539e+07,  1.89241653e+06, -4.47449440e+02],
                     [ 3.08158618e+07,  2.85347563e+07, -2.44868263e+02],
                     [ 5.09902892e+06,  4.16822832e+07, -7.00894852e+01]])

    # compare the results to the truth values
    accuracy = 1.0  # meters

    testFailCount, testMessages = unitTestSupport.compareArray(
        true_pos, pos_rel_earth_parse, accuracy, "r_BN_N Vector",
        testFailCount, testMessages)

    #   print out success message if no error were found
    if testFailCount == 0:
        print "PASSED "
    else:
        print testFailCount
        print testMessages

    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(1, 4):
        plt.plot(pos_rel_earth_parse[:, 0] * macros.NANO2SEC / P, pos_rel_earth_parse[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')

    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')
    plt.title('Position Relative To Earth')
    if show_plots:
        plt.show()
        plt.close('all')

    figureList = {}
    fileName = os.path.basename(os.path.splitext(__file__)[0])
    pltName = fileName + "srp_integrated"
    figureList[pltName] = plt.figure(1)

    return testFailCount, testMessages


#
# This statement below ensures that the unit test script can be run as a stand-alone python script
#
if __name__ == "__main__":
    radiationPressureIntegratedTest(True)
