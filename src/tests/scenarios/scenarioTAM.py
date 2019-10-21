''' '''
'''
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
# Purpose:  Demonstrates how to setup TAM sensor on a rigid spacecraft
# Author:   Demet Cilden-Guler
# Creation Date:  September 20, 2019
#

import os
import numpy as np

import matplotlib.pyplot as plt
# The path to the location of Basilisk
# Used to get the location of supporting data.

from Basilisk import __path__
bskPath = __path__[0]

## \page scenarioTAMGroup
## @{
## Demonstrates how to add a Three-Axis Magnetometer (TAM) sensor to a spacecraft.
#
# Three-Axis Magnetometer (TAM) Simulation {#scenarioTAM}
# ====
#
# Scenario Description
# -----
# This script sets up a 3-DOF spacecraft which is orbiting a planet with a magnetic field.
# The orbit setup is similar to that used in [scenarioBasicOrbit.py](@ref scenarioBasicOrbit).
# The scenarios can be run with the followings setups parameters:
# Setup | orbitCase           | planetCase            | useBias               | useBounds
# ----- | ------------------- | --------------------- | --------------------- | ---------------------
# 1     | circular            | Earth                 | False                 | False
# 2     | elliptical          | Earth                 | True                  | False
# 3     | circular            | Jupiter               | False                 | True
# 4     | elliptical          | Jupiter               | False                 | False
#
# To run the default scenario 1 from the Basilisk/src/tests/scenarios folder, call the python script through
#
#       python3 scenarioTAM.py
#
# Simulation Scenario Setup Details
# -----
# The simulation layout is shown in the following illustration. A single simulation process is created
# which contains the spacecraft object. The spacecraft state and the magnetic field model
# (World Magnetic Model (WMM), Centered Dipole Magnetic Field etc.) messages  are connected to
# the magnetometer module which outputs the local magnetic field measurements in sensor frame components.
# ![Simulation Flow Diagram](Images/doc/test_scenario_magnetometer.svg "Illustration")
#
# When the simulation completes 2 plots are shown for each case. One plot shows the radius in km,
# while the second plot shows the magnetic field measurement vector components with respect to the sensor frame.
#
# The dynamics simulation is setup using a SpacecraftPlus() module.
# The magnetic field module is created using one of the magnetic field models, e.g.
# [MagneticFieldCenteredDipole](@ref scenarioMagneticFieldCenteredDipoleGroup)
# which requires setting appropriate parameters for the specific planet,
# [MagneticFieldWMM](@ref scenarioMagneticFieldWMMGroup) which is a model specific to only Earth.
# The TAM sensor unit is created using:
#~~~~~~~~~~~~~~~~~{.py}
#     TAM = magnetometer.Magnetometer()
#~~~~~~~~~~~~~~~~~
# The minimum variables that must be set for TAM includes:
# ~~~~~~~~~~~~~~~~{.py}
#     TAM.ModelTag = "TAM_sensor"
#     TAM.tamDataOutMsgName = "TAM_output"
# ~~~~~~~~~~~~~~~~
# Next, this module is attached to the simulation process with
#~~~~~~~~~~~~~~~~~{.py}
#   scSim.AddModelToTask(simTaskName, TAM)
#~~~~~~~~~~~~~~~~~
# The input message from the magnetic field model is specified in the data logging phase as:
# ~~~~~~~~~~~~~~~~~{.py}
#   TAM.magIntMsgName = magModule.envOutMsgNames[0]
# ~~~~~~~~~~~~~~~~~
# The Gaussian noise is setup as,
# ~~~~~~~~~~~~~~~~{.py}
#   TAM.SenNoiseStd = 100e-9  # Tesla
# ~~~~~~~~~~~~~~~~
# The continuous constant bias, and the bounds of the saturation are set for the useBias and useBounds conditions:
# ~~~~~~~~~~~~~~~~{.py}
#   TAM.SenBias = [0, 0, -1e-6]  # Tesla
#   TAM.minOutput = -3.5e-4  # Tesla
#   TAM.maxOutput = 3.5e-4  # Tesla
# ~~~~~~~~~~~~~~~~
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#  run(
#         True,        # show_plots
#         'circular',  # orbit Case (circular, elliptical)
#         'Earth',     # planetCase (Earth, Jupiter)
#          False,      # useBias
#          False       # useBounds
#     )
# ~~~~~~~~~~~~~
#
# This scenario places the spacecraft about the Earth in a circular LEO orbit.  The
# resulting position coordinates and magnetic field components are shown below.
# ![Orbit Radius History](Images/Scenarios/scenarioTAM1circularEarth.svg "Position history")
# ![Magnetic Field Measurements Illustration](Images/Scenarios/scenarioTAM2circularEarth.svg "Magnetic Field Measurements Illustration")
#
# Setup 2
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#  run(
#         True,          # show_plots
#         'elliptical',  # orbit Case (circular, elliptical)
#         'Earth'        # planetCase (Earth, Jupiter)
#          True,         # useBias
#          False         # useBounds
#     )
# ~~~~~~~~~~~~~
# TThis case illustrates an elliptical Earth orbit.
# The resulting radius and magnetic field measurements illustrations are shown below.
# ![Orbit Radius History](Images/Scenarios/scenarioTAM1ellipticalEarth.svg "Position history")
# ![Magnetic Field Measurements Illustration](Images/Scenarios/scenarioTAM2ellipticalEarth.svg "Magnetic Field Measurements Illustration")
#
#
# Setup 3
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#  run(
#         True,        # show_plots
#         'circular',  # orbit Case (circular, elliptical)
#         'Jupiter'    # planetCase (Earth, Jupiter)
#          False,      # useBias
#          True        # useBounds
#     )
# ~~~~~~~~~~~~~
# This case illustrates a circular orbit about Jupiter.
# The resulting radius and magnetic field measurements illustrations are shown below.
# ![Orbit Radius History](Images/Scenarios/scenarioTAM1circularJupiter.svg "Position history")
# ![Magnetic Field Measurements Illustration](Images/Scenarios/scenarioTAM2circularJupiter.svg "Magnetic Field Measurements Illustration")
#
# Setup 4
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#  run(
#         True,          # show_plots
#         'elliptical',  # orbit Case (circular, elliptical)
#         'Jupiter'      # planetCase (Earth, Jupiter)
#          False,        # useBias
#          False         # useBounds
#     )
# ~~~~~~~~~~~~~
# This case illustrates an elliptical orbit about Jupiter.
# The resulting radius and magnetic field measurements illustrations are shown below.
# ![Orbit Radius History](Images/Scenarios/scenarioTAM1ellipticalJupiter.svg "Position history")
# ![Magnetic Field Measurements Illustration](Images/Scenarios/scenarioTAM2ellipticalJupiter.svg "Magnetic Field Measurements Illustration")
#
## @}

# import simulation related support
from Basilisk.simulation import magneticFieldCenteredDipole
from Basilisk.simulation import magneticFieldWMM
from Basilisk.simulation import magnetometer

# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport)
from Basilisk.utilities import simSetPlanetEnvironment

# import simulation related support
from Basilisk.simulation import spacecraftPlus

#attempt to import vizard
from Basilisk.utilities import vizSupport
fileName = os.path.basename(os.path.splitext(__file__)[0])

def run(show_plots, orbitCase, planetCase, useBias, useBounds):
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
    if planetCase == 'Jupiter':
        planet = gravFactory.createJupiter()
        planet.isCentralBody = True           # ensure this is the central gravitational body
    elif planetCase == 'Earth':
        planet = gravFactory.createEarth()
        planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    req = planet.radEquator

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    if planetCase == 'Jupiter':
        magModule = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()  # default is Earth centered dipole module
        magModule.ModelTag = "CenteredDipole"
        # The following command is a support function that sets up the centered dipole parameters.
        # These parameters can also be setup manually
        simSetPlanetEnvironment.centeredDipoleMagField(magModule, 'jupiter')
    elif planetCase == 'Earth':
        magModule = magneticFieldWMM.MagneticFieldWMM()
        magModule.ModelTag = "WMM"
        magModule.dataPath = bskPath + '/supportData/MagneticField/'
        # set epoch date/time message
        magModule.epochInMsgName = "simEpoch"
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg('2019 June 27, 10:23:0.0 (UTC)')
        unitTestSupport.setMessage(scSim.TotalSim,
                                   simProcessName,
                                   magModule.epochInMsgName,
                                   epochMsg)
        if orbitCase == 'elliptical':
            magModule.envMinReach = 10000 * 1000.
            magModule.envMaxReach = 20000 * 1000.

    # add spacecraft to the magnetic field module so it can read the sc position messages
    magModule.addSpacecraftToModel(scObject.scStateOutMsgName)  # this command can be repeated if multiple

    # add the magnetic field module to the simulation task stack
    scSim.AddModelToTask(simTaskName, magModule)

    TAM = magnetometer.Magnetometer()
    TAM.ModelTag = "TAM_sensor"
    TAM.scaleFactor = 1.0
    TAM.tamDataOutMsgName = "TAM_output"
    TAM.stateIntMsgName = scObject.scStateOutMsgName
    TAM.senNoiseStd = 0
    if useBias:
        TAM.senBias = [0, 0, -1e-6]  # Tesla
    if useBounds:
        TAM.maxOutput = 3.5e-4  # Tesla
        TAM.minOutput = -3.5e-4  # Tesla

    scSim.AddModelToTask(simTaskName, TAM)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rPeriapses = req*1.1     # meters
    if orbitCase == 'circular':
        oe.a = rPeriapses
        oe.e = 0.0000
    elif orbitCase == 'elliptical':
        rApoapses = req*3.5
        oe.a = (rPeriapses + rApoapses) / 2.0
        oe.e = 1.0 - rPeriapses / oe.a
    else:
        print("Unsupported orbit type " + orbitCase + " selected")
        exit(1)
    oe.i = 85.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    # next lines stores consistent initial orbit elements
    # with circular or equatorial orbit, some angles are arbitrary
    oe = orbitalMotion.rv2elem(mu, rN, vN)

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(1. * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime // (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(magModule.envOutMsgNames[0], samplingTime)
    scSim.TotalSim.logThisMessage(TAM.tamDataOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    TAM.magIntMsgName = magModule.envOutMsgNames[0]  # get mag field model output message as an input message to TAM

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    # vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory, saveFile=fileName)

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
    magData = scSim.pullMessageLogData(magModule.envOutMsgNames[0] + '.magField_N', list(range(3)))
    tamData = scSim.pullMessageLogData(TAM.tamDataOutMsgName + '.OutputData', list(range(3)))
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', list(range(3)))

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs

    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    rData = []
    for idx in range(0, len(posData)):
        rMag = np.linalg.norm(posData[idx, 1:4])
        rData.append(rMag / 1000.)
    plt.plot(posData[:, 0] * macros.NANO2SEC / P, rData, color='#aa0000')
    if orbitCase == 'elliptical':
        plt.plot(posData[:, 0] * macros.NANO2SEC / P, [magModule.envMinReach / 1000.] * len(rData), color='#007700',
                 dashes=[5, 5, 5, 5])
        plt.plot(posData[:, 0] * macros.NANO2SEC / P, [magModule.envMaxReach / 1000.] * len(rData),
                 color='#007700', dashes=[5, 5, 5, 5])
    plt.xlabel('Time [orbits]')
    plt.ylabel('Radius [km]')
    plt.ylim(min(rData) * 0.9, max(rData) * 1.1)
    figureList = {}
    pltName = fileName + "1" + orbitCase + planetCase
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    for idx in range(1, 4):
        plt.plot(tamData[:, 0] * macros.NANO2SEC / P, tamData[:, idx] * 1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$TAM_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Magnetic Field [nT] ')
    pltName = fileName + "2" + orbitCase + planetCase
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return magData, tamData, figureList, simulationTime

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,          # show_plots (True, False)
        'circular',  # orbit Case (circular, elliptical)
        'Earth',    # planet Case (Earth, Jupiter)
        False,        # useBias
        False        # useBounds
    )
