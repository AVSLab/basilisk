#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

r"""
Overview
--------

This script sets up a 3-DOF spacecraft which is orbiting in Earth's magnetic field.
The purpose is to demonstrate how to add multiple Three-Axis Magnetometers (TAM) to a spacecraft and assign different
magnetic field representations, biases, and bounds to each.
The orbit setup is similar to that used in :ref:`scenarioBasicOrbit`.

The script is found in the folder ``Basilisk/examples`` and executed by using::

      python3 scenarioTAMcomparison.py

Simulation Scenario Setup Details
---------------------------------
The simulation layout is shown in the following illustration. A single simulation process is created which contains
the spacecraft object. The spacecraft state and the magnetic field models (World Magnetic Model (WMM) and Centered
Dipole Magnetic Field) messages are each connected to a :ref:`magnetometer` module which outputs the local magnetic field
measurements in sensor frame components.

When the simulation completes three plots are shown for each case. One plot shows the radius in km, the second shows
the :ref:`scenarioMagneticFieldCenteredDipole` measurements, and the third shows the :ref:`scenarioMagneticFieldWMM`
measurements. The magnetic models measurements are shown as vector components with respect to the sensor frame.

The dynamics simulation is setup using a :ref:`Spacecraft` module. The magnetometers are defined as Three Axis
Magnetometers based on :ref:`scenarioTAM`. The bounds for each magnetometer can be individually activated, and a bias
in the sensor z-direction can be individually activated. The Centered Dipole Model can be utilized for Jupiter
or Earth, but only the parameters for Earth are defined in this simulation. The World Magnetic Model (WMM) is specific
to Earth.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, orbitCase = 'circular', useBias1 = False, useBounds1 = True, useBias2 = False, useBounds2 = True

.. image:: /_images/Scenarios/scenarioTAMcomparison1circularFalseTrueFalseTrue.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTAMcomparison2circularFalseTrueFalseTrue.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTAMcomparison3circularFalseTrueFalseTrue.svg
   :align: center

::

    show_plots = True, orbitCase = 'elliptical', useBias1 = True, useBounds1 = False, useBias2 = True, useBounds2=False

.. image:: /_images/Scenarios/scenarioTAMcomparison1ellipticalTrueFalseTrueFalse.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTAMcomparison2ellipticalTrueFalseTrueFalse.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTAMcomparison3ellipticalTrueFalseTrueFalse.svg
   :align: center

::

    show_plots = True, orbitCase = 'elliptical', useBias1 = False, useBounds1 = False, useBias2 = False,
    useBounds2 = False

.. image:: /_images/Scenarios/scenarioTAMcomparison1ellipticalFalseFalseFalseFalse.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTAMcomparison2ellipticalFalseFalseFalseFalse.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTAMcomparison3ellipticalFalseFalseFalseFalse.svg
   :align: center

::

    show_plots = True, orbitCase = 'circular', useBias1 = False, useBounds1 = True, useBias2 = False, useBounds2 = False

.. image:: /_images/Scenarios/scenarioTAMcomparison1circularFalseTrueFalseFalse.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTAMcomparison2circularFalseTrueFalseFalse.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTAMcomparison3circularFalseTrueFalseFalse.svg
   :align: center
"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Compares TAM results using the WMM and centered Dipole Model
# Author:   Kaylee Champion
# Creation Date:  September 2021
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import magneticFieldCenteredDipole
from Basilisk.simulation import magneticFieldWMM
from Basilisk.simulation import magnetometer

# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport)
from Basilisk.utilities import simSetPlanetEnvironment

#attempt to import vizard
from Basilisk.utilities import vizSupport

def run(show_plots, orbitCase, useBias1, useBias2, useBounds1, useBounds2):
    """
    The scenarios can be run with the following setups parameters:
    
    Args:
        show_plots (bool): Determines if the script should display plots
        orbitCase (str):  Specify the type of orbit to be simulated {'elliptical','circular'}
        useBias1 (bool): Flag to use a sensor bias on TAM 1
        useBias2 (bool): Flag to use a sensor bias on TAM 2
        useBounds1 (bool): Flag to use TAM 1 sensor bounds
        useBounds2 (bool): Flag to use TAM 2 sensor bounds
    
    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    req = planet.radEquator

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    # create the centered dipole magnetic field
    #
    magModule1 = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()  # default is Earth centered dipole module
    magModule1.ModelTag = "CenteredDipole"
    simSetPlanetEnvironment.centeredDipoleMagField(magModule1, 'earth')

    if orbitCase == 'elliptical':
        # Note that more then one magnetic field can be attached to a planet.
        # In the elliptic Earth orbit scenario
        # a second magnetic field module `magModule2` is created with a
        # different custom dipole model.  It is connected to the
        # same spacecraft state message as the first magnetic field model.

        magModule2 = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
        magModule2.ModelTag = "CenteredDipole2"
        magModule2.addSpacecraftToModel(scObject.scStateOutMsg)
        # set the 2nd magnetic field through custom dipole settings
        magModule2.g10 = -30926.00 / 1e9 * 0.5  # Tesla
        magModule2.g11 = -2318.00 / 1e9 * 0.5  # Tesla
        magModule2.h11 = 5817.00 / 1e9 * 0.5  # Tesla
        magModule2.planetRadius = 6371.2 * 1000  # meters
        # set the reach variables such that the fields do not overlap
        magModule2.envMaxReach = req * 1.3
        magModule1.envMinReach = magModule2.envMaxReach
        scSim.AddModelToTask(simTaskName, magModule2)

    #
    # create the WMM magnetic field
    #

    magModule3 = magneticFieldWMM.MagneticFieldWMM()
    magModule3.ModelTag = "WMM"
    magModule3.dataPath = bskPath + '/supportData/MagneticField/'
    # set epoch date/time message
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg('2019 June 27, 10:23:0.0 (UTC)')
    magModule3.epochInMsg.subscribeTo(epochMsg)
    # set the minReach and maxReach values if on an elliptic orbit
    if orbitCase == 'elliptical':
        magModule3.envMinReach = 10000 * 1000.
        magModule3.envMaxReach = 20000 * 1000.

    # add spacecraft to the magnetic field modules so it can read the sc position messages
    magModule1.addSpacecraftToModel(scObject.scStateOutMsg)
    magModule3.addSpacecraftToModel(scObject.scStateOutMsg)

    # add the magnetic field modules to the simulation task stack
    scSim.AddModelToTask(simTaskName, magModule1)
    scSim.AddModelToTask(simTaskName, magModule3)

    # create the minimal TAM modules
    TAM1 = magnetometer.Magnetometer()
    TAM2 = magnetometer.Magnetometer()
    TAM1.ModelTag = "TAM1_sensor"
    TAM2.ModelTag = "TAM2_sensor"
    # specify the optional TAM variables
    TAM1.scaleFactor = 1.0
    TAM2.scaleFactor = 1.0
    TAM1.senNoiseStd = [100e-9, 100e-9, 100e-9]
    TAM2.senNoiseStd = [100e-9, 100e-9, 100e-9]

    if orbitCase == 'elliptical':
        TAM3 = magnetometer.Magnetometer()  # TAM3 is a dummy TAM used to plot Dipole Magnetic Model 2
        TAM3.ModelTag = "TAM3_sensor"
        TAM3.scaleFactor = 1.0
        TAM3.senNoiseStd = [100e-9, 100e-9, 100e-9]

    if useBias1:
        useBias1_str = 'True'
        TAM1.senBias = [0, 0, -1e-6]  # Tesla
        if orbitCase == 'elliptical':
            TAM3.senBias = [0, 0, -1e-6]  # Tesla
    else:
        useBias1_str = 'False'
    if useBounds1:
        useBounds1_str = 'True'
        TAM1.maxOutput = 2.5E-5  # Tesla
        TAM1.minOutput = -2.5E-5  # Tesla
        if orbitCase == 'elliptical':
            TAM3.maxOutput = 2.5E-5  # Tesla
            TAM3.minOutput = -2.5E-5  # Tesla
    else:
        useBounds1_str = 'False'
    TAM1.stateInMsg.subscribeTo(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, TAM1)
    if orbitCase == 'elliptical':
        TAM3.stateInMsg.subscribeTo(scObject.scStateOutMsg)
        scSim.AddModelToTask(simTaskName, TAM3)

    if useBias2:
        useBias2_str = 'True'
        TAM2.senBias = [0, 0, -1e-6]  # Tesla
    else:
        useBias2_str = 'False'
    if useBounds2:
        useBounds2_str = 'True'
        TAM2.maxOutput = 2.5E-5  # Tesla
        TAM2.minOutput = -2.5E-5  # Tesla
    else:
        useBounds2_str = 'False'
    TAM2.stateInMsg.subscribeTo(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, TAM2)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rPeriapses = req * 1.1  # meters
    if orbitCase == 'circular':
        oe.a = rPeriapses
        oe.e = 0.0000
    elif orbitCase == 'elliptical':
        rApoapses = req * 3.5
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
    oe = orbitalMotion.rv2elem(mu, rN, vN)  # this stores consistent initial orbit elements
    # with circular or equatorial orbit, some angles are arbitrary

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(1. * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    mag1Log = magModule1.envOutMsgs[0].recorder(samplingTime)
    mag3Log = magModule3.envOutMsgs[0].recorder(samplingTime)
    tam1Log = TAM1.tamDataOutMsg.recorder(samplingTime)
    tam2Log = TAM2.tamDataOutMsg.recorder(samplingTime)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, mag1Log)
    scSim.AddModelToTask(simTaskName, mag3Log)
    scSim.AddModelToTask(simTaskName, tam1Log)
    scSim.AddModelToTask(simTaskName, tam2Log)
    scSim.AddModelToTask(simTaskName, dataLog)
    TAM1.magInMsg.subscribeTo(magModule1.envOutMsgs[0])
    TAM2.magInMsg.subscribeTo(magModule3.envOutMsgs[0])
    if orbitCase == 'elliptical':
        mag2Log = magModule2.envOutMsgs[0].recorder(samplingTime)
        tam3Log = TAM3.tamDataOutMsg.recorder(samplingTime)
        scSim.AddModelToTask(simTaskName, mag2Log)
        scSim.AddModelToTask(simTaskName, tam3Log)
        TAM3.magInMsg.subscribeTo(magModule2.envOutMsgs[0])

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    mag1Data = mag1Log.magField_N
    mag3Data = mag3Log.magField_N
    tam1Data = tam1Log.tam_S
    tam2Data = tam2Log.tam_S
    if orbitCase == 'elliptical':
        mag2Data = mag2Log.magField_N
        tam3Data = tam3Log.tam_S
    posData = dataLog.r_BN_N

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
    timeAxis = dataLog.times()
    for idx in range(len(posData)):
        rMag = np.linalg.norm(posData[idx])
        rData.append(rMag / 1000.)
    plt.plot(timeAxis * macros.NANO2SEC / P, rData, color='#aa0000')
    if orbitCase == 'elliptical':
        plt.plot(timeAxis * macros.NANO2SEC / P, [magModule3.envMinReach / 1000.] * len(rData), color='#007700',
                 dashes=[5, 5, 5, 5])
        plt.plot(timeAxis * macros.NANO2SEC / P, [magModule3.envMaxReach / 1000.] * len(rData),
                 color='#007700', dashes=[5, 5, 5, 5])
    plt.xlabel('Time [orbits]')
    plt.ylabel('Radius [km]')
    plt.ylim(min(rData) * 0.9, max(rData) * 1.1)
    figureList = {}
    pltName = fileName + "1" + orbitCase + useBias1_str + useBounds1_str + useBias2_str + useBounds2_str
    figureList[pltName] = plt.figure(1)

    # plot 2
    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2SEC / P, tam1Data[:, idx] * 1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$TAM_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Magnetic Field [nT] ')
    plt.title('Centered Dipole Model')
    if orbitCase == 'elliptical':
        for idx in range(3):
            plt.plot(timeAxis * macros.NANO2SEC / P, tam3Data[:, idx] * 1e9, '--',
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$TAM_{' + str(idx) + '}$')
    pltName = fileName + "2" + orbitCase + useBias1_str + useBounds1_str + useBias2_str + useBounds2_str
    figureList[pltName] = plt.figure(2)

    # plot 3
    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2SEC / P, tam2Data[:, idx] * 1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$TAM_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Magnetic Field [nT] ')
    plt.title('WMM Model')
    pltName = fileName + "3" + orbitCase + useBias1_str + useBounds1_str + useBias2_str + useBounds2_str
    figureList[pltName] = plt.figure(3)


    if show_plots:
        plt.show()

        # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots (True,False)
        'circular',  # orbit Case (circular, elliptical)
        False,  #use Bias 1 (True,False)
        False,  #use Bias 2 (True,False)
        True,   #Use sensor bounds 1 (True,False)
        False   #Use sensor bounds 2 (True,False)
    )

