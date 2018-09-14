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
# Purpose:  Integrated test of the spacecraftPlus() and gravity modules.  Illustrates
#           a 3-DOV spacecraft on a range of orbit types.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 26, 2016
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



## \defgroup Tutorials_1_0
## @{
## Demonstration of setup basic 3-DOF orbit simulation setup.
#
# Basic Orbit Setup and Translational Motion Simulation {#scenarioBasicOrbit}
# ====
#
# Scenario Description
# -----
# This script sets up a 3-DOF spacecraft which is orbiting a planet.  The purpose
# is to illustrate how to create a spacecraft, attach a gravity model, and run
# a basic Basilisk simulation.  The scenarios can be run with the followings setups
# parameters:
# Setup | orbitCase           | useSphericalHarmonics | planetCase
# ----- | ------------------- | --------------------- | -----------
# 1     | LEO                 | False                 | Earth
# 2     | GTO                 | False                 | Earth
# 3     | GEO                 | False                 | Earth
# 4     | LEO                 | True                  | Earth
# 5     | LEO                 | False                 | Mars
#
# To run the default scenario 1 from the Basilisk/scenarios folder, call the python script through
#
#       python scenarioBasicOrbit.py
#
# *However*, to play with any scenario scripts as tutorials, you should make a copy of them into a custom folder
# outside of the Basilisk directory.
#
# To copy them, first find the location of the Basilisk installation.
# After installing, you can find the installed location of Basilisk by opening a python interpreter and
# running the commands:
#
#~~~~~~~~~~~~~~{.py}
# import Basilisk
# basiliskPath = Basilisk.__path__[0]
# print basiliskPath
#~~~~~~~~~~~~~~
#
# Copy the folder `{basiliskPath}/tests` into a new folder in a different directory.
# Now, when you want to use a tutorial, navigate inside that folder, and edit and execute the *copied* integrated tests.
#
# Qt Visualization Option
# -----
# If you wish to transmit the simulation data to the Qt Visualization, then uncomment the following
# line (line 360 in the script) from the python scenario script.  If the Viz is running, and searching for a connection on
# 127.0.0.1 (using Open Connection command from the File menu), the simulation is visualized in
# realtime.  To enable this, uncomment line 360 in this tutorial script.
#~~~~~~~~~~~~~~{.py}
# unitTestSupport.enableVisualization(scSim, dynProcess, simProcessName, 'earth')  # The Viz only support 'earth', 'mars', or 'sun'
#~~~~~~~~~~~~~~
# The current Qt based visualiztion is only able to show orbits about either Earth, Mars or the sun.
# Be sure to match the celestial object name in the line above with the current simulation.
# Further, note that by default the Viz is running in realtime mode with a 1x speed up factor.  This Viz
# speed up factor can be increased in the Qt GUI by calling up the
#
#       View/Bottom Playback Controls
#
# The speed up factor is adusting in 2x steps up or down using the green arrows in this GUI.
# This simulation is only updated at the rate of the simulation.  Thus, for this orbit simulation with 10s
# time steps the Viz will be choppy at 1x.  Speeding up the Viz playback with the green arrows quickly illustrates
# the orbital motion.
#
#
# Simulation Scenario Setup Details
# -----
# The simulation layout is shown in the following illustration.  A single simulation process is created
# which contains the spacecraft object.  Gravity effectors are attached to the spacecraft dynamics to
# simulate the gravitational accelerations.
# ![Simulation Flow Diagram](Images/doc/test_scenarioBasicOrbit.svg "Illustration")
#
# When the simulation completes 2 plots are shown for each case.  One plot always shows
# the inertial position vector components, while the second plot either shows a planar
# orbit view relative to the perfocal frame (no spherical harmonics), or the
# semi-major axis time history plot (with spherical harmonics turned on).
#
# The dynamics simulation is setup using a SpacecraftPlus() module.
#~~~~~~~~~~~~~~~~~{.py}
#     scObject = spacecraftPlus.SpacecraftPlus()
#     scObject.ModelTag = "spacecraftBody"
#~~~~~~~~~~~~~~~~~
# Note that this module simulates both the translational and rotational motion of the spacecraft.
# In this scenario only the translational (i.e. orbital) motion is tracked.  This means the rotational motion
# remains at a default inertial frame orientation in this scenario.  There is no appreciable speed hit to
# simulate both the orbital and rotational motion for a single rigid body.  In the later scenarios
# the rotational motion is engaged by specifying rotational initial conditions, as well as rotation
# related effectors.  In this simple scenario only translational motion is setup and tracked.
# Further, the default spacecraft parameters, such as the unit mass and the principle inertia values are
# just fine for this orbit simulation as they don't impact the orbital dynamics in this case.
# This is true for all gravity force only orbital simulations. Later
# tutorials, such as [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback),
# illustrate how to over-ride default values with desired simulation values.
#
# Next, this module is attached to the simulation process
#~~~~~~~~~~~~~~~~~{.py}
#   scSim.AddModelToTask(simTaskName, scObject)
#~~~~~~~~~~~~~~~~~
# The first step to adding gravity objects is to create the gravity body factor class.  Note that
# this call will create an empty gravitational body list each time this script is called.  Thus, there
# is not need to clear any prior list of gravitational bodies.
#~~~~~~~~~~~~~~~~~{.py}
#   gravFactory = simIncludeGravBody.gravBodyFactory()
#~~~~~~~~~~~~~~~~~
# To attach an Earth gravity model to this spacecraft, the following macro is invoked:
#~~~~~~~~~~~~~~~~~{.py}
#     planet = gravFactory.createEarth()
#     planet.isCentralBody = True          # ensure this is the central gravitational body
#~~~~~~~~~~~~~~~~~
# The gravFactor() class stores the Earth gravitational object within the class, but it also returns a
# handler to this gravitational object as a convenience.  The celestial object position and velocity
# vectors are all defaulted to zero values.  If non-zero values are required, this can be manually
# overriden.  If multiple bodies are simulated, then their positions can be
# dynamically updated.  See [scenarioOrbitMultiBody.py](@ref scenarioOrbitMultiBody) to learn how this is
# done via a SPICE object.
#
# If extra customization is required, see the createEarth() macro to change additional values.
# For example, the spherical harmonics are turned off by default.  To engage them, the following code
# is used
#~~~~~~~~~~~~~~~~~{.py}
#     planet.useSphericalHarmParams = True
#     simIncludeGravBody.loadGravFromFile(Basilisk.__path__[0]+'/supportData/LocalGravData/GGM03S-J2-only.txt'
#                                         , planet.spherHarm
#                                         ,2
#                                         )
#~~~~~~~~~~~~~~~~~
# The value 2 indidates that the first two harmonics, excluding the 0th order harmonic,
# are included.  This harmonics data file only includes a zeroth order and J2 term.
#
# Finally, the gravitational body must be connected to the spacecraft object.  This is done with
#~~~~~~~~~~~~~~~~~{.py}
#     scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())
#~~~~~~~~~~~~~~~~~
# Here the complete list of gravitational bodies is automatically assigned to the spacecraft, regardless if
# it is only one body like Earth or Mars, or a list of multiple bodies.
#
# Note that the default planets position and velocity vectors in the gravitational body are set to zero.  If
# alternate position or velocity vectors are requried, this can be done by creating the planet ephemerise message
# that is connected to the gravity effector input message `bodyInMsgName`.
# If time varying planet ephemeris messages are to be included use the Spice module.  For non-zero messages
# the planet's default ephemeris would be replaced with the desired custom values.  How to use Spice to setup
# planet ephemerise is shown in the tutorial
# [scenarioOrbitMultiBody.py](@ref scenarioOrbitMultiBody).
#
# To set the spacecraft initial conditions, the following initial position and velocity variables are set:
#~~~~~~~~~~~~~~~~~{.py}
#    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
#    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
#~~~~~~~~~~~~~~~~~
# These vectors specify the inertial position and velocity vectors relative to the planet of the
# spacecraft center of mass location.  Note that there are 2 points that can be tracked.  The user always
# specifies the spacecraft center of mass location with the above code.  If the simulation output should be
# about another body fixed point B, this can be done as well.  This is useful in particular with more challenging
# dynamics where the center of mass moves relative to the body.  The following vector would specify the location of
# the spacecraft hub center of mass (Bc) relative to this body fixed point.
# ~~~~~~~~~~~~~~~~{.py}
#    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
# ~~~~~~~~~~~~~~~~
# If this vector is not specified, as in this tutorial scenario, then it defaults to zero.  If only a rigid hub
# is modeled, the Bc (hub center of mass) is the same as C (spacecraft center of mass).  If the spacecrat contains
# state effectors such as hinged panels, fuel slosh, imbalanced reaction wheels, etc., then the points Bc and C would
# not be the same.  Thus, in this simple simulation the body fixed point B and spacecraft center of mass are
# identical.
#
# Before the simulation is ready to run, it must be initialized.  The following code uses a convenient macro routine
# which initializes each BSK module (run self init, cross init and reset) and clears the BSK logging stack.
#~~~~~~~~~~~~~~~~~{.py}
#     scSim.InitializeSimulationAndDiscover()
#~~~~~~~~~~~~~~~~~
# If there are messages that are shared across multiple BSK threads, as shown in
# [scenarioAttitudeFeedback2T.py](@ref scenarioAttitudeFeedback2T), then this routine also
# auto-discovers these shared messages.
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # save figures to file
#          True,        # show_plots
#          'LEO',       # orbit Case
#          False,       # useSphericalHarmonics
#          'Earth'      # planet Case
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last 2 arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The default
# scenario places the spacecraft about the Earth in a LEO orbit and without considering
# gravitational spherical harmonics.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1LEO0Earth.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2LEO0Earth.svg "Orbit Illustration")
#
# Setup 2
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # save figures to file
#          True,        # show_plots
#          'GTO',       # orbit Case
#          False,       # useSphericalHarmonics
#          'Earth'      # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates an elliptical Geosynchronous Transfer Orbit (GTO) with zero orbit
# inclination.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1GTO0Earth.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2GTO0Earth.svg "Orbit Illustration")
#
# Setup 3
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # save figures to file
#          True,        # show_plots
#          'GEO',       # orbit Case
#          False,       # useSphericalHarmonics
#          'Earth'      # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular Geosynchronous Orbit (GEO) with zero orbit
# inclination.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1GEO0Earth.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2GEO0Earth.svg "Orbit Illustration")
#
#  Setup 4
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # save figures to file
#          True,        # show_plots
#          'LEO,        # orbit Case
#          True,        # useSphericalHarmonics
#          'Earth'      # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular LEO with a non-zero orbit
# inclination.  In this case the Earth's spherical harmonics are turned on.  The
# resulting position coordinates and semi-major axis time histories are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1LEO1Earth.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2LEO1Earth.svg "Orbit Illustration")
#
# Setup 5
# -------
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # save figures to file
#          True,        # show_plots
#          'LEO',       # orbit Case
#          True,        # useSphericalHarmonics
#          'Mars'       # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular Low Mars Orbit or LMO with a non-zero orbit
# inclination.  If you wish to visualize this simulation, be sure to change the celestial object name in
#~~~~~~~~~~~~~~{.py}
# unitTestSupport.enableVisualization(scSim, dynProcess, simProcessName, 'mars')
# # The Viz only support 'earth', 'mars', or 'sun'
#~~~~~~~~~~~~~~
# from 'earth' to 'mars'.  In this simulation setup the planet's spherical harmonics are turned on.  The
# resulting position coordinates and semi-major axis time histories are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1LEO0Mars.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2LEO0Mars.svg "Orbit Illustration")
#
## @}
def run(show_plots, orbitCase, useSphericalHarmonics, planetCase):
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
    if planetCase is 'Mars':
        planet = gravFactory.createMarsBarycenter()
        planet.isCentralBody = True           # ensure this is the central gravitational body
        if useSphericalHarmonics:
            planet.useSphericalHarmParams = True
            simIncludeGravBody.loadGravFromFile(bskPath + '/supportData/LocalGravData/GGM2BData.txt',
                                                planet.spherHarm, 100)
    else:  # Earth
        planet = gravFactory.createEarth()
        planet.isCentralBody = True          # ensure this is the central gravitational body
        if useSphericalHarmonics:
            planet.useSphericalHarmParams = True
            simIncludeGravBody.loadGravFromFile(bskPath + '/supportData/LocalGravData/GGM03S-J2-only.txt',
                                                planet.spherHarm, 2)
    mu = planet.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    rGEO = 42000. * 1000     # meters
    if orbitCase is 'GEO':
        oe.a = rGEO
        oe.e = 0.00001
        oe.i = 0.0 * macros.D2R
    elif orbitCase is 'GTO':
        oe.a = (rLEO + rGEO) / 2.0
        oe.e = 1.0 - rLEO / oe.a
        oe.i = 0.0 * macros.D2R
    else:                   # LEO case, default case 0
        oe.a = rLEO
        oe.e = 0.0001
        oe.i = 33.3 * macros.D2R
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
    if useSphericalHarmonics:
        simulationTime = macros.sec2nano(3. * P)
    else:
        simulationTime = macros.sec2nano(0.75 * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    if useSphericalHarmonics:
        numDataPoints = 400
    else:
        numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

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
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))

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
    figureList = {}
    pltName = fileName + "1" + orbitCase + str(int(useSphericalHarmonics))+ planetCase
    figureList[pltName] = plt.figure(1)

    if useSphericalHarmonics is False:
        # draw orbit in perifocal frame
        b = oe.a * np.sqrt(1 - oe.e * oe.e)
        p = oe.a * (1 - oe.e * oe.e)
        plt.figure(2, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
        plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
        # draw the planet
        fig = plt.gcf()
        ax = fig.gca()
        if planetCase == 'Mars':
            planetColor = '#884400'
        else:
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
        plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=3.0
                 )
        # draw the full osculating orbit from the initial conditions
        fData = np.linspace(0, 2 * np.pi, 100)
        rData = []
        for idx in range(0, len(fData)):
            rData.append(p / (1 + oe.e * np.cos(fData[idx])))
        plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555'
                 )
        plt.xlabel('$i_e$ Cord. [km]')
        plt.ylabel('$i_p$ Cord. [km]')
        plt.grid()

    else:
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        smaData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
            smaData.append(oeData.a / 1000.)
        plt.plot(posData[:, 0] * macros.NANO2SEC / P, smaData, color='#aa0000',
                 )
        plt.xlabel('Time [orbits]')
        plt.ylabel('SMA [km]')

    pltName = fileName + "2" + orbitCase + str(int(useSphericalHarmonics)) + planetCase
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return posData, figureList



#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,        # show_plots
        'LEO',       # orbit Case (LEO, GTO, GEO)
        False,       # useSphericalHarmonics
        'Earth'      # planetCase (Earth, Mars)
    )
