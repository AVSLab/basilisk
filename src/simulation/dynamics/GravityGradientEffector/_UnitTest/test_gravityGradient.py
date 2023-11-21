
# Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Test the gravity gradient effector module.
# Author:   Hanspeter Schaub
# Creation Date:  Jan 12, 2019
#

import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk import __path__
from Basilisk.simulation import GravityGradientEffector
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeGravBody, orbitalMotion, RigidBodyKinematics
from Basilisk.utilities import unitTestSupport

bskPath = __path__[0]

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("planetCase", [0, 1, 2, 3])
@pytest.mark.parametrize("cmOffset", [[[0.1], [0.15], [-0.1]], [[0.0], [0.0], [0.0]]])


# provide a unique test method name, starting with test_
def test_gravityGradientModule(show_plots, cmOffset, planetCase):
    r"""
    **Validation Test Description**

    This test creates a spacecraft in orbit about either Earth or Venus to check if the correct gravity gradient
    torque is evaluated.  Multiple test scenario combinations are possible where either a single or multiple
    gravity bodies are included, using either zero planet ephemeris for the single planet case, or using SPICE
    for the multi-planet scenario.

    **Test Parameters**

    The following list discusses in detail the various test parameters used. These are test tested in
    all possible permutations (except show_plots of course) which is turned off for ``pytest`` usage.

    :param show_plots:  flag to show some simulation plots
    :param cmOffset:    center of mass offset vector in meters
    :param planetCase: integer flag with values (0,1,2,3).  The cases consider the following simulation scenarios:

                        - Case 0 indicates a simulation with only Earth present at (0,0,0).
                        - Case 1 is a simulation with both Earth and Venus present using Spice, but the gravity
                          gradient torque is only evaluated using Earth.
                        - Case 2 is same as 1 but Venus is also included in the torque evaluation.
                        - Case 3 is like 2 but here the spacecraft is orbiting venus.
    :return: None

    **Description of Variables Being Tested**

    The gravity effector torque output message is compared against a python evaluated vector.

    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(
            show_plots, cmOffset, planetCase, 2.0)
    assert testResults < 1, testMessage


def truthGravityGradient(mu, rN, sigmaBN, hub):
    I = hub.IHubPntBc_B
    r = np.linalg.norm(rN)
    BN = RigidBodyKinematics.MRP2C(sigmaBN)
    rHatB = np.matmul(BN, rN) / r

    ggTorque = 3*mu/r/r/r * np.cross(rHatB, np.matmul(I, rHatB))

    return ggTorque

def run(show_plots, cmOffset, planetCase, simTime):
    """Call this routine directly to run the unit test."""
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages


    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))
    simulationTime = macros.sec2nano(simTime)

    # create Earth Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    if planetCase:
        # here all planet positions are provided by Spice, and both Earth and Venus are include
        # for gravity acceleration calculations
        venus = gravFactory.createVenus()
        timeInitString = "2012 MAY 1 00:28:30.0"
        gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                         timeInitString,
                                         epochInMsg=True)

        scSim.AddModelToTask(simTaskName, gravFactory.spiceObject, -1)

        if planetCase == 3:
            # orbit should be defined relative to Venus
            earth.isCentralBody = False
            venus.isCentralBody = True
            mu = venus.mu
            gravFactory.spiceObject.zeroBase = 'venus'  # spacecraft states are logged relative to Earth for plotting
        else:
            gravFactory.spiceObject.zeroBase = 'earth'  # spacecraft states are logged relative to Earth for plotting

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
                                                # with circular or equatorial orbit, some angles are arbitrary

    # setup basic spacecraft module
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskTestSat"
    IIC = [[500., 0., 0.]
           , [0., 800., 0.]
           , [0., 0., 350.]]
    scObject.hub.r_BcB_B = cmOffset
    scObject.hub.mHub = 100.0  # kg - spacecraft mass
    scObject.hub.IHubPntBc_B = IIC
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    scSim.AddModelToTask(simTaskName, scObject)

    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # add gravity gradient effector
    ggEff = GravityGradientEffector.GravityGradientEffector()
    ggEff.ModelTag = scObject.ModelTag
    ggEff.addPlanetName(earth.planetName)
    if planetCase >= 2:
        ggEff.addPlanetName(venus.planetName)
    scObject.addDynamicEffector(ggEff)
    scSim.AddModelToTask(simTaskName, ggEff)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 50
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    dataLogGG = ggEff.gravityGradientOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, dataLogGG)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    attData = dataLog.sigma_BN
    ggData = dataLogGG.gravityGradientTorque_B
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    if show_plots:
        plt.close("all")  # clears out plots from earlier test runs

        # draw the inertial position vector components
        plt.close("all")  # clears out plots from earlier test runs
        plt.figure(1)
        for idx in range(0, 3):
            plt.plot(dataLog.times() * macros.NANO2MIN, attData[:, idx],
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$\sigma_' + str(idx) + '$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [min]')
        plt.ylabel(r'MRP Attitude $\sigma_{B/N}$')

        plt.figure(2)
        for idx in range(0, 3):
            plt.plot(dataLog.times() * macros.NANO2MIN, posData[:, idx]/1000,
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$r_' + str(idx) + '$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [min]')
        plt.ylabel(r'Inertial Position coordinates [km]')

        plt.figure(3)
        for idx in range(0, 3):
            plt.plot(dataLogGG.times() * macros.NANO2MIN, ggData[:, idx] ,
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$r_' + str(idx) + '$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [min]')
        plt.ylabel(r'GG Torque [Nm]')

        plt.show()
        plt.close("all")

    # compare gravity gradient torque vector to the truth
    accuracy = 1e-10
    for rV, sV, ggV in zip(posData, attData, ggData):
        ggTruth = truthGravityGradient(mu, rV[0:3], sV[0:3], scObject.hub)
        testFailCount, testMessages = unitTestSupport.compareVector(ggV[0:3],
                                                                    ggTruth,
                                                                    accuracy,
                                                                    "gravityGradientTorque_B",
                                                                    testFailCount, testMessages)

    print("Accuracy used: " + str(accuracy))
    if testFailCount == 0:
        print("PASSED: Gravity Effector")
    else:
        print("Failed: Gravity Effector")

    return testFailCount, testMessages

    # close the plots being saved off to avoid over-writing old and new figures
if __name__ == '__main__':
    run(True,           # show_plots
        [[0.0], [0.0], [0.0]], # cmOffset
        3,              # planetCase
        3600)  # simTime (seconds)
