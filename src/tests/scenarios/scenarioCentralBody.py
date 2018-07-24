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
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstrate sim set up using isCentralBody=True and isCentralBody=False
# Author:   Scott Carnahan
# Creation Date:  Jul 11 2018
#

import os
import numpy as np

import matplotlib.pyplot as plt
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
# import simulation related support
from Basilisk.simulation import spacecraftPlus
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport)
from Basilisk.utilities import planetStates
from numpy import array
from numpy.linalg import norm



## \defgroup Tutorials_1_4
## @{
## Demonstration of using planetStates.planetPositionVelocity and isCentralBody to set
## spacecraft initial states in an absolute or relative frame
#
# Central Body Setup {#scenarioCentralBody}
# ====
#
# Scenario Description
# -----
# This script sets up a basic spacecraft in orbit about Earth. One option uses earth.isCentralBody = True and the
# other uses isCentralBody = False. The nuances of spacecraft position and velocity I/O in these cases is demonstrated.
# graphs are provided which show the outputs to be the same in each case
# parameters:
# Setup | showPlots           | useCentral
# ----- | ------------------- | ------------
# 1     | True                | False
# 2     | True                | True
#
# To run the default scenario 1 from the Basilisk/scenarios folder, call the python script through
#
#       python scenarioCentralBody.py
#
# Simulation Scenario Setup Details
# -----
# The basics of the spacecraft and simulation set up are shown in
# [scenarioBasicOrbit.py](@ref scenarioBasicOrbit) and only the particulars are discussed here.
#
# Spacecraft positions can be input (and integrated) only relative to the "central body" by
# using the isCentralBody flag. This is demonstrated in Setup 2. Generally, this is a good
# practice because it increases the accurracy of the integration.
#
# Alternatively, absolute positions and velocities can be input and integrated.
# To do so, it is useful to be able to get a planet position and velocity from spice
# to be able to modify your spacecraft inputs by these values. This can be done using the
# planetStates utility:
# ~~~~~~~~~~~~~{.py}
# from Basilisk.utilities import planetStates
# ~~~~~~~~~~~~~
# Then, the planetPositionVelocity() method can be used to get the needed information
# ~~~~~~~~~~~~~{.py}
# planetPosition, planetVelocity = planetStates.planetPositionVelocity('EARTH', UTCInit)
# ~~~~~~~~~~~~~
# In the above call, the first input is the planet to get the states of and the second is the UTC time
# to get the states at. Additional inputs are available in the method documentation. These states can then be added
# to the planet-relative states before setting them to the spacecraft initial statesL
# ~~~~~~~~~~~~~{.py}
# scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN + array(planetPosition))
# scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN + array(planetVelocity))
# ~~~~~~~~~~~~~
# This works without frame rotations because all planet states are given in the spice inertial system relative to the
# zero base. Additionally, it is assumed for this script that the Keplerian orbital elements were given relative
# to the Earth Centered Inertial Frame which is aligned with the spice inertial frame.
#
# Finally, note that the output position and velocity (when reading message logs) will be relative to the spice zero
# base, even when a central body is being used. So, to plot planet-relative orbits, the outputs are adjusted by the
# time history of the earth position and velocity.
#
# Plots found when running this scenario are the same as the basic orbit scenario and are included for visual inspection that the results are
# roughly the same regardless of the use of a central body.

## @}
def run(show_plots, useCentral):
    '''Call this routine directly to run the tutorial scenario.'''


    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    # unitTestSupport.enableVisualization(scSim, dynProcess, simProcessName, 'earth')
    # The Viz only support 'earth', 'mars', or 'sun'

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = useCentral          # ensure this is the central gravitational body
    mu = planet.mu

    # set up sun
    gravFactory.createSun()

    #Set up spice with spice time
    UTCInit = "2012 MAY 1 00:28:30.0"
    spiceObject = gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/', UTCInit)
    scSim.AddModelToTask(simTaskName, spiceObject)

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    oe.a = rLEO
    oe.e = 0.000001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    truth_r = norm(rN) #for test results
    truth_v = norm(vN) #for test results
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements wrt ECI frame

    #
    #   initialize Spacecraft States with the initialization variables
    #
    if useCentral:
        scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
        scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N
    else:
        #by default planetstates.planetPositionVelocity returns SSB central ICRS coordinates for the planet at the time
        # requested. also pck0010.tpc ephemeris file
        #look in the function for how to use other ephemeris files, reference frames, and observers
        planetPosition, planetVelocity = planetStates.planetPositionVelocity('EARTH', UTCInit)
        scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN + array(planetPosition))
        scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN + array(planetVelocity))

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(0.75*P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage("earth_planet_data", samplingTime)

    #
    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
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
    #Note: position and velocity are returned relative to the zerobase (SSB by default)
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))
    earthPositionHistory = scSim.pullMessageLogData("earth_planet_data.PositionVector", range(3))
    earthVelocityHistory = scSim.pullMessageLogData("earth_planet_data.VelocityVector", range(3))

    #bring the s/c pos, vel back to earth relative coordinates to plot
    posData[:, 1:4] -= earthPositionHistory[:, 1:4]
    velData[:, 1:4] -= earthVelocityHistory[:, 1:4]

    out_r = norm(posData[-1, 1:4])
    out_v = norm(velData[-1, 1:4])

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileName = os.path.basename(os.path.splitext(__file__)[0])

    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(1, 4):
        plt.plot(posData[:, 0] * macros.NANO2SEC / P, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')

    # draw orbit in perifocal frame
    b = oe.a * np.sqrt(1 - oe.e * oe.e)
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(2, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = '#008800'
    planetRadius = planet.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(posData[:,1] / 1000, posData[:,2] / 1000, color='#aa0000', linewidth=3.0)
    # draw the full osculating orbit from the initial conditions
    fData = np.linspace(0, 2 * np.pi, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555')
    plt.xlabel('$i_e$ Cord. [km]')
    plt.ylabel('$i_p$ Cord. [km]')
    plt.grid()

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return out_r, out_v, truth_r, truth_v

# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,        # show_plots
        False        # useCentral
    )
