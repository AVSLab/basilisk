#
#  ISC License
#
#  Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#
r"""
Overview
--------

Demonstrates how to add albedo module to coarse sun sensor. The script is found in the folder ``src/examples`` and
executed by using::

      python3 scenarioAlbedo.py

This script sets up a spacecraft with scenario options (1) and (2) shown in the following illustration.
In scenario (1), only earth is considered as a celestial body, and the spacecraft is orbiting around earth.
In scenario (2), there are two bodies (earth, moon) and spacecraft close to Earth increases its altitude up to moon
without considering the gravity effector. In both scenarios, rotational  motion is simulated too.

.. image:: /_images/static/test_scenarioAlbedo.svg
   :align: center

Simulation Scenario Setup Details
---------------------------------
A simulation process is created which contains both the spacecraft simulation module,
as well as one albedo module from :ref:`albedo` module using CSS configuration.
The dynamics simulation is setup using :ref:`SpacecraftPlus`.
The CSS module is created using :ref:`coarse_sun_sensor`.
The input message name sunPositionInMsgName specifies an input message that contains the sun's position.

Albedo module calculates the albedo value for any instrument location. The instrument configuration can be set
through the variables::

   fov
   nHat_B
   r_IB_B

which specify half field of view angle of the instrument, unit normal vector, and misalignment vector in meters.
The instrument configuration is added to the albedo module through::

    albModule.addInstrumentConfig(instInMsgName, configMsg)
    albModule.addInstrumentConfig(instInMsgName, fov, nHat_B, r_IB_B)

Both method can be used for the same purpose. Note that this commands can be repeated if the albedo should be computed
for different instruments.

Every time an instrument is added to the albedo module, an automated output message name is created.
For `albModule` is "AlbedoModule_0_data" as the ModelTag string is ``AlbedoModule`` and the instrument number is 0.
This output name is created in the  ``addInstrumentConfig()`` function.

Multiple planets can be added to the albedo module through::

    albModule.addPlanetandAlbedoAverageModel(planetSpiceName)
    albModule.addPlanetandAlbedoAverageModel(planetSpiceName, ALB_avg, numLat, numLon)
    albModule.addPlanetandAlbedoDataModel(planetSpiceName, dataPath, fileName)

based on the albedo model to be used for the planet. Note that this commands should be repeated for adding multiple
planets. However, the albedo module gives a summed albedo value at the instrument not a vector of values corresponding
to each planet. The albedo module is created using one of the albedo models, e.g.
``ALBEDO_DATA``
which is based on the albedo coefficient data using the specified fileName and dataPath, and
``ALBEDO_AVG``
which is a model based on an average albedo value that can be specified with ``ALB_avg`` variable,
and used for any planet. If ``ALB_avg`` is not specified albedo module uses the default value defined for each planet.

An optional input is the solar eclipse case ``eclipseCase``.
If it is specified as True, then the shadow factor is calculated by the albedo module automatically.

When the simulation completes, one to three plots are shown depending on the case. First one always show the
albedo value at the instrument (instrument's signal) with or without the instrument's total signal. In case of
using albedo data file, the albedo coefficient data map of the planet is shown. And, for the multiple planets case,
the radius of the spacecraft is shown.

albedoData, multipleInstrument, multiplePlanet, useEclipse

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, albedoData = True, multipleInstrument = False, multiplePlanet = False, useEclipse = False

.. image:: /_images/Scenarios/scenarioAlbedo11000.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAlbedo21000.svg
   :align: center

::

    show_plots = True, albedoData = False, multipleInstrument = True, multiplePlanet = False, useEclipse = False

.. image:: /_images/Scenarios/scenarioAlbedo10100.svg
   :align: center

::

    show_plots = True, albedoData = False, multipleInstrument = True, multiplePlanet = False, useEclipse = True

.. image:: /_images/Scenarios/scenarioAlbedo10101.svg
   :align: center

::

    show_plots = True, albedoData = False, multipleInstrument = True, multiplePlanet = True, useEclipse = False

.. image:: /_images/Scenarios/scenarioAlbedo10110.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAlbedo20110.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstrates how to setup albedo for CSS
# Author:   Demet Cilden-Guler
# Creation Date:  May 27, 2020
#

import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros, simIncludeGravBody
from Basilisk.simulation import coarse_sun_sensor
from Basilisk.simulation import albedo
from Basilisk.simulation import eclipse
from Basilisk.utilities import orbitalMotion as om

# import simulation related support
from Basilisk.simulation import spacecraftPlus

# import message declarations
from Basilisk.simulation import simMessages

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileNameString = os.path.basename(os.path.splitext(__file__)[0])

def run(show_plots, albedoData, multipleInstrument, multiplePlanet, useEclipse, simTimeStep):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots.
		albedoData (bool): Flag indicating if the albedo data based model should be used.
        multipleInstrument (bool): Flag indicating if multiple instrument should be used.
        multiplePlanet (bool): Flag specifying if multiple planets should be used.
        useEclipse (bool): Flag indicating if the partial eclipse at the incremental area is considered.
        simTimeStep (double): Flag specifying the simulation time step.

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"
    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)
    # Create the dynamics task
    if simTimeStep is None:
        simulationTimeStep = macros.sec2nano(10.)
    else:
        simulationTimeStep = macros.sec2nano(simTimeStep)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))
    # Create sun message
    sunPositionMsg = simMessages.SpicePlanetStateSimMsg()
    sunPositionMsg.PositionVector = [-om.AU * 1000., 0.0, 0.0]
    sunMessage = "sun_message"
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               sunMessage,
                               sunPositionMsg)
    # Create planet message (earth)
    gravFactory = simIncludeGravBody.gravBodyFactory()
    # Create planet message (earth)
    planetCase1 = 'earth'
    planet1 = gravFactory.createEarth()
    planet1.isCentralBody = True  # ensure this is the central gravitational body
    req1 = planet1.radEquator
    planetPositionMsg1 = simMessages.SpicePlanetStateSimMsg()
    planetPositionMsg1.PositionVector = [0., 0., 0.]
    planetPositionMsg1.PlanetName = planetCase1
    planetPositionMsg1.J20002Pfix = np.identity(3)
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               'earth_planet_data',
                               planetPositionMsg1)
    if multiplePlanet:
        # Create planet message (moon)
        planetCase2 = 'moon'
        planetPositionMsg2 = simMessages.SpicePlanetStateSimMsg()
        planetPositionMsg2.PositionVector = [0., 384400. * 1000, 0.]
        planetPositionMsg2.PlanetName = planetCase2
        planetPositionMsg2.J20002Pfix = np.identity(3)
        unitTestSupport.setMessage(scSim.TotalSim,
                                   simProcessName,
                                   'moon_planet_data',
                                   planetPositionMsg2)
    #
    # Initialize spacecraftPlus object and set properties
    #
    oe = om.ClassicElements()
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "SpacecraftBody1"
    scObject.scStateOutMsgName = "Spacecraft1"
    rLEO = req1 + 500 * 1000  # m
    # Define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    if multiplePlanet:
        # Set initial spacecraft states
        scObject.hub.r_CN_NInit = [[0.0], [rLEO], [0.0]]  # m - r_CN_N
        scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]  # m - v_CN_N
        scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_BN_B
        scObject.hub.omega_BN_BInit = [[0.0], [0.0], [1. * macros.D2R]]  # rad/s - omega_BN_B

    else:
        # Single planet case (earth)
        oe.a = rLEO
        oe.e = 0.0001
        oe.i = 0.0 * macros.D2R
        oe.Omega = 0.0 * macros.D2R
        oe.omega = 0.0 * macros.D2R
        oe.f = 180.0 * macros.D2R
        rN, vN = om.elem2rv(planet1.mu, oe)
        # set the simulation time
        n = np.sqrt(planet1.mu / oe.a / oe.a / oe.a)
        P = 2. * np.pi / n
        simulationTime = macros.sec2nano(0.5 * P)
        # Set initial spacecraft states
        scObject.hub.r_CN_NInit = rN  # m - r_CN_N
        scObject.hub.v_CN_NInit = vN  # m - v_CN_N
        scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_BN_B
        scObject.hub.omega_BN_BInit = [[0.0], [0.0], [.5 * macros.D2R]]  # rad/s - omega_BN_B
        scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, simulationTimeStep)
    #
    # Albedo Module
    #
    albModule = albedo.Albedo()
    albModule.ModelTag = "AlbedoModule"
    albModule.spacecraftStateInMsgName = scObject.scStateOutMsgName
    albModule.sunPositionInMsgName = sunMessage
    #
    # CSS-1
    #
    CSS1 = coarse_sun_sensor.CoarseSunSensor()
    CSS1.ModelTag = "CSS1"
    CSS1.cssDataOutMsgName = "CSS1_output"
    CSS1.stateInMsgName = albModule.spacecraftStateInMsgName
    CSS1.sunInMsgName = albModule.sunPositionInMsgName
    CSS1.fov = 80.*macros.D2R
    CSS1.maxOutput = 1.0
    CSS1.nHat_B = np.array([1., 0., 0.])
    #
    if useEclipse:
        albModule.eclipseCase = True
        eclipseObject = eclipse.Eclipse()
        eclipseObject.sunInMsgName = sunMessage
        eclipseObject.addPositionMsgName(scObject.scStateOutMsgName)
        eclipseObject.addPlanetName(planetCase1)
        scSim.AddModelToTask(simTaskName, eclipseObject)
        CSS1.sunEclipseInMsgName = "eclipse_data_0"
    #
    if albedoData:
        dataPath = os.path.abspath(bskPath + "/supportData/AlbedoData/")
        fileName = "Earth_ALB_2018_CERES_All_5x5.csv"
        albModule.addPlanetandAlbedoDataModel(planetCase1, dataPath, fileName)
    else:
        ALB_avg = 0.5
        numLat = 200
        numLon = 200
        albModule.addPlanetandAlbedoAverageModel(planetCase1, ALB_avg, numLat, numLon)
    #
    if multiplePlanet:
        albModule.addPlanetandAlbedoAverageModel(planetCase2)
    #
    # Add instrument to albedo module
    #
    config1 = albedo.instConfig_t()
    config1.fov = CSS1.fov
    config1.nHat_B = CSS1.nHat_B
    config1.r_IB_B = CSS1.r_PB_B
    albModule.addInstrumentConfig(CSS1.ModelTag, config1)
    # CSS albedo input message names should be defined after adding instrument to module
    CSS1.albedoInMsgName = albModule.albOutMsgNames[0]
    if multipleInstrument:
        # CSS-2
        CSS2 = coarse_sun_sensor.CoarseSunSensor(CSS1)
        CSS2.ModelTag = "CSS2"
        CSS2.cssDataOutMsgName = "CSS2_output"
        CSS2.nHat_B = np.array([-1., 0., 0.])
        albModule.addInstrumentConfig(CSS2.ModelTag, CSS2.fov, CSS2.nHat_B, CSS2.r_PB_B)
        scSim.TotalSim.logThisMessage(albModule.albOutMsgNames[1], simulationTimeStep)
        CSS2.albedoInMsgName = albModule.albOutMsgNames[1]
        # CSS-3
        CSS3 = coarse_sun_sensor.CoarseSunSensor(CSS1)
        CSS3.ModelTag = "CSS3"
        CSS3.cssDataOutMsgName = "CSS3_output"
        CSS3.nHat_B = np.array([0., -1., 0.])
        albModule.addInstrumentConfig(CSS3.ModelTag, CSS3.fov, CSS3.nHat_B, CSS3.r_PB_B)
        scSim.TotalSim.logThisMessage(albModule.albOutMsgNames[2], simulationTimeStep)
        CSS3.albedoInMsgName = albModule.albOutMsgNames[2]
    #
    # Add albedo and CSS to task and setup logging before the simulation is initialized
    #
    scSim.AddModelToTask(simTaskName, albModule)
    scSim.TotalSim.logThisMessage(albModule.albOutMsgNames[0], simulationTimeStep)
    scSim.AddModelToTask(simTaskName, CSS1)
    scSim.TotalSim.logThisMessage(CSS1.cssDataOutMsgName, simulationTimeStep)
    if multipleInstrument:
        scSim.TotalSim.logThisMessage(albModule.albOutMsgNames[1], simulationTimeStep)
        scSim.TotalSim.logThisMessage(albModule.albOutMsgNames[2], simulationTimeStep)
        scSim.AddModelToTask(simTaskName, CSS2)
        scSim.TotalSim.logThisMessage(CSS2.cssDataOutMsgName, simulationTimeStep)
        scSim.AddModelToTask(simTaskName, CSS3)
        scSim.TotalSim.logThisMessage(CSS3.cssDataOutMsgName, simulationTimeStep)
    #
    # Initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()
    #
    if multiplePlanet:
        velRef = scObject.dynManager.getStateObject("hubVelocity")
        # Configure a simulation stop time time and execute the simulation run
        T1 = macros.sec2nano(500.)
        scSim.ConfigureStopTime(T1)
        scSim.ExecuteSimulation()
        # get the current spacecraft states
        vVt = unitTestSupport.EigenVector3d2np(velRef.getState())
        T2 = macros.sec2nano(1000.)
        # Set second spacecraft states for decrease in altitude
        vVt = vVt + [0.0, 375300, 0.0]  # m - v_CN_N
        velRef.setState(unitTestSupport.np2EigenVectorXd(vVt))
        scSim.ConfigureStopTime(T1 + T2)
        scSim.ExecuteSimulation()
        # get the current spacecraft states
        T3 = macros.sec2nano(500.)
        # Set second spacecraft states for decrease in altitude
        vVt = [0.0, 0.0, 0.0]  # m - v_CN_N
        velRef.setState(unitTestSupport.np2EigenVectorXd(vVt))
        scSim.ConfigureStopTime(T1 + T2 + T3)
        scSim.ExecuteSimulation()
        simulationTime = T1 + T2 + T3
    else:
        # Configure a simulation stop time time and execute the simulation run
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()
    #
    # Retrieve the logged data
    #
    n = int(simulationTime/simulationTimeStep + 1)
    if multipleInstrument:
        dataCSS = np.zeros(shape=(n, 4))
        dataAlb = np.zeros(shape=(n, 4))
    else:
        dataCSS = np.zeros(shape=(n, 2))
        dataAlb = np.zeros(shape=(n, 2))
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', list(range(3)))
    dataCSS[:, 0:2] = scSim.pullMessageLogData(CSS1.cssDataOutMsgName + ".OutputData", list(range(1)))
    dataAlb[:, 0:2] = scSim.pullMessageLogData(albModule.albOutMsgNames[0] + ".albedoAtInstrument", list(range(1)))
    if multipleInstrument:
        dataCSS2 = scSim.pullMessageLogData(CSS2.cssDataOutMsgName + ".OutputData", list(range(1)))
        dataCSS3 = scSim.pullMessageLogData(CSS3.cssDataOutMsgName + ".OutputData", list(range(1)))
        dataAlbA2 = scSim.pullMessageLogData(albModule.albOutMsgNames[1] + ".albedoAtInstrument", list(range(1)))
        dataAlbA3 = scSim.pullMessageLogData(albModule.albOutMsgNames[2] + ".albedoAtInstrument", list(range(1)))
        dataCSS[:, 2] = dataCSS2[:, 1]
        dataCSS[:, 3] = dataCSS3[:, 1]
        dataAlb[:, 2] = dataAlbA2[:, 1]
        dataAlb[:, 3] = dataAlbA3[:, 1]
    np.set_printoptions(precision=16)
    #
    # Plot the results
    #
    plt.close("all")        # clears out plots from earlier test runs
    plt.figure(1)
    if multipleInstrument:
        for idx in range(1, 4):
            plt.plot(dataAlb[:, 0] * macros.NANO2SEC, dataAlb[:, idx],
                     linewidth=2, alpha=0.7, color=unitTestSupport.getLineColor(2 * idx, 6),
                     label='Albedo$_{'+str(idx)+'}$')
            if not multiplePlanet:
                plt.plot(dataCSS[:, 0]*macros.NANO2SEC, dataCSS[:, idx],
                         '--', linewidth=1.5, color=unitTestSupport.getLineColor(2 * idx - 1, 6),
                         label='CSS$_{'+str(idx)+'}$')
    else:
        plt.plot(dataAlb[:, 0] * macros.NANO2SEC, dataAlb[:, 1],
                 linewidth=2, alpha=0.7, color=unitTestSupport.getLineColor(1, 2),
                 label='Alb$_{1}$')
        if not multiplePlanet:
            plt.plot(dataCSS[:, 0] * macros.NANO2SEC, dataCSS[:, 1],
                     '--', linewidth=1.5, color=unitTestSupport.getLineColor(2, 2),
                     label='CSS$_{1}$')
    if multiplePlanet:
        plt.legend(loc='upper center')
    else:
        plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('Instrument\'s signal')
    figureList = {}
    pltName = fileNameString + str(1) + str(int(albedoData)) + str(int(multipleInstrument)) + str(int(multiplePlanet)) + str(
        int(useEclipse))
    figureList[pltName] = plt.figure(1)
    if multiplePlanet:
        # Show radius of SC
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        rData = np.linalg.norm(posData[:, 1:4], axis=1) / 1000.
        plt.plot(posData[:, 0] * macros.NANO2SEC, rData, color='#aa0000')
        plt.xlabel('Time [s]')
        plt.ylabel('Radius [km]')
        pltName = fileNameString + str(2) + str(int(albedoData)) + str(int(multipleInstrument)) + str(int(multiplePlanet)) + str(
            int(useEclipse))
        figureList[pltName] = plt.figure(2)

    if albedoData:
        filePath = os.path.abspath(dataPath + '/' + fileName)
        ALB1 = np.genfromtxt(filePath, delimiter=',')
        # ALB coefficient figures
        fig = plt.figure(2)
        ax = fig.add_subplot(111)
        ax.set_title('Earth Albedo Coefficients (All Sky)')
        ax.set(xlabel='Longitude (deg)', ylabel='Latitude (deg)')
        plt.imshow(ALB1, cmap='Reds', interpolation='none', extent=[-180, 180, 90, -90])
        plt.colorbar(orientation='vertical')
        ax.set_ylim(ax.get_ylim()[::-1])
        pltName = fileNameString + str(2) + str(int(albedoData)) + str(int(multipleInstrument)) + str(int(multiplePlanet)) + str(
            int(useEclipse))
        figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()
    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")
    return dataAlb, simulationTime, simulationTimeStep, figureList
#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,   # show_plots
        False,  # albedoData
        True,   # multipleInstrument
        False,  # multiplePlanet
        True,   # useEclipse
        None    # simTimeStep
       )
