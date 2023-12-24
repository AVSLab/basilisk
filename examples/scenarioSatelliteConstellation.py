#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario demonstrates how to set up a Walker-Delta constellation of satellites. Note that this scenario 
uses the stand-alone Basilisk architecture rather than using the ''examples/FormationBskSim`` or ``examples/MultiSatBskSim``
architectures for simultaneously simulating multiple spacecraft.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioSatelliteConstellation.py

When designing a satellite constellation, symmetry provides repeatable performance and makes design and operations simpler. One 
such symmetric constellation design methodology is the Walker constellation which consists of circular orbits in evenly spaced 
planes at a chosen inclination. Walker constellation layouts are fully specified by four parameters written as "i:T/P/F" where:

    * i = inclination angle
    * T = total number of satellites in the constellation
    * P = numer of orbit planes evenly spaced in node angle
    * F = relative spacing between satellites in adjacent orbit planes

In order to simulate the constellation a semi-major axis 'a' is also specified as an input. The total number of satellites T must 
be specified as an integer multiple of the number of planes P. The relative spacing F must be an integer between 0 and (P-1).

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, a = 29994000, i = 56, T = 24, P = 3, F = 1

The default scenario is modelled after the Galileo constellation with Walker numbers 56:24/3/1 and a semi-major axis of 29,994 km.

.. image:: /_images/Scenarios/scenarioSatelliteConstellation.svg
   :align: center

Each line color corresponds to a different orbital plane, in this case 3 planes. Green circles mark the start of each of the 24 
individual satellite's ground tracks and a red circle marks their end.

Simulation Visualization In Vizard
----------------------------------

To add a visualization of each spacecraft's Earth nadir pointing, a cone aligned with the pointing direction
is created through the ``vizInterface``::

    vizSupport.createConeInOut(viz, fromBodyName=scList[i].ModelTag, toBodyName='earth', coneColor='teal',
                               normalVector_B=[1, 0, 0], incidenceAngle=30/macros.D2R, isKeepIn=False,
                               coneHeight=10.0, coneName='pointingCone')

.. image:: /_images/static/scenarioSatelliteConstellationVizard.jpg
   :scale: 50%
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstrates how to use the stand alone Basilisk architecture to automate 
#           the setup of a Walker constellation.
# Author:   Andrew Morell
# Creation Date:  Dec. 13, 2023
#

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.simulation import (spacecraft, ephemerisConverter)
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport)
from Basilisk.fswAlgorithms import locationPointing
from Basilisk.simulation import (simpleNav, planetEphemeris)

# always import the Basilisk messaging support

def run(show_plots, a, i, T, P, F):
    """
    The scenario can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        a (float): semi-major axis of all satellites [m]
        i (float): orbital inclination [rad]
        T (int): total number of satellites
        P (int): number of equally spaced orbital planes
        F (int): phasing between satellites in adjacent places

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # (Optional) If you want to see a simulation progress bar in the terminal window, the
    # use the following SetProgressBar(True) statement
    scSim.SetProgressBar(True)

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(30.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # Create the ephemeris converter module
    spiceObject = gravFactory.createSpiceInterface(time="2029 June 12 5:30:30.0", epochInMsg=True)
    spiceObject.zeroBase = 'Earth'
    scSim.AddModelToTask(simTaskName, spiceObject)
    ephemObject = ephemerisConverter.EphemerisConverter()
    ephemObject.ModelTag = "ephemData"
    ephemObject.addSpiceInputMsg(spiceObject.planetStateOutMsgs[0]) # Earth
    scSim.AddModelToTask(simTaskName, ephemObject)

    # spacecraft inertia
    I = [900., 0., 0.,
        0., 800., 0.,
        0., 0., 600.]

    # set the simulation time to be two orbit periods
    n = np.sqrt(mu / a / a / a)
    Pr = 2. * np.pi / n
    simulationTime = macros.sec2nano(1.5 * Pr)
    
    # create a logging task object of the spacecraft output message at the desired down sampling ratio
    numDataPoints = 500
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)

    if (np.mod(T,1) !=0 or np.mod(P,1) !=0 or np.mod(F,1) != 0 or T < 1 or P < 1 or F < 1):
        raise Exception('number of satellites T, number of planes P, and relative spaceing F must be positive integer numbers')
    if (np.mod(T,P) !=0):
        raise Exception('number of satellites T must be an integer multiple of P')

    # pre-compute Walker constellation parameters
    PU = 360/T # patter unit in degrees
    S = T/P # number of satellites in each plane
    delta_RAAN = S*PU # delta RAAN in degrees
    delta_nu = P*PU # in plane spacing of satellites
    oe = orbitalMotion.ClassicElements()
    oe.a = a
    oe.e = 0.01
    oe.i = i

    # initialize T spacecraft objects and set properties
    scList = []
    navList = []
    guideList = []
    dataRec = []
    for i in range(T):
        # initialize spacecraft object
        scList.append(spacecraft.Spacecraft())
        scList[i].ModelTag = "bsk-Sat-"+str(i)
        scSim.AddModelToTask(simTaskName, scList[i])

        # attach gravity model to spacecraft
        scList[i].gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

        # setup the orbit using classical orbit elements
        RAAN = delta_RAAN * np.floor(i/S)
        oe.Omega = RAAN * macros.D2R
        oe.omega = F * np.floor(i/S) * macros.D2R
        oe.f = delta_nu * np.mod(i,S) * macros.D2R
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        scList[i].hub.r_CN_NInit = rN  # [m]
        scList[i].hub.v_CN_NInit = vN  # [m/s]

        # set the spacecraft mass and inertia
        scList[i].hub.mHub = 750.0  # kg - spacecraft mass
        scList[i].hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

        # set up prescribed attitude guidance
        navList.append(simpleNav.SimpleNav())
        navList[i].ModelTag = "SimpleNav"+str(i)
        scSim.AddModelToTask(simTaskName, navList[i])
        navList[i].scStateInMsg.subscribeTo(scList[i].scStateOutMsg)

        # setup nadir pointing guidance module
        guideList.append(locationPointing.locationPointing())
        guideList[i].ModelTag = "nadirPoint"+str(i)
        guideList[i].pHat_B = [1, 0, 0]
        guideList[i].scTransInMsg.subscribeTo(navList[i].transOutMsg)
        guideList[i].scAttInMsg.subscribeTo(navList[i].attOutMsg)
        guideList[i].celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[0])
        scSim.AddModelToTask(simTaskName, guideList[i])
        scList[i].attRefInMsg.subscribeTo(guideList[i].attRefOutMsg)

        # record spacecraft states
        dataRec.append(scList[i].scStateOutMsg.recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, dataRec[i])

    # If you wish to transmit the simulation data to the United based Vizard Visualization application,
    # then uncomment the following saveFile line
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scList,
                                                  # saveFile=__file__
                                                  )
        for i in range(T):
            vizSupport.createConeInOut(viz, fromBodyName=scList[i].ModelTag, toBodyName='earth', coneColor='teal',
                           normalVector_B=[1, 0, 0], incidenceAngle=30/macros.D2R, isKeepIn=False,
                           coneHeight=10.0, coneName='pointingCone')

    # initialize simulation
    scSim.InitializeSimulation()

    # configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # plot the ground track
    plt.close("all")  # clears out plots from earlier test runs
    figureList = {}
    plt.figure(1)
    for n in range(T):
        # calculate latitude and longitude
        [lon, lat] = rv2latlon(dataRec[n])
        color = unitTestSupport.getLineColor(int(np.floor(n/S)), 3)
        plt.scatter(lat, lon, s=5, c=[color])
        plt.scatter(lat[0], lon[0], s=20, c='#00A300')
        plt.scatter(lat[-1], lon[-1], s=20, c='#A30000')
    plt.xlim([-180,180])
    plt.ylim([-90,90])
    plt.xticks(range(-180,181,30))
    plt.yticks(range(-90,91,15))
    plt.xlabel('longitude (degrees)')
    plt.ylabel('lattitude (degrees)')

    pltName = fileName
    figureList[pltName] = plt.figure(1)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList

def rv2latlon(dataRec):
    times = dataRec.times() * macros.NANO2SEC
    rECI = dataRec.r_BN_N
    lat = np.zeros(times.shape)
    lon = np.zeros(times.shape)
    for i in range(len(times)):
        theta = times[i]*planetEphemeris.OMEGA_EARTH
        dcm_ECI2ECEF = [[np.cos(theta), np.sin(theta), 0],
                        [-np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1]]
        rECEF = dcm_ECI2ECEF@rECI[i,:]
        lat[i] = np.arcsin(rECEF[2]/np.linalg.norm(rECEF)) * macros.R2D
        lon[i] = np.arctan2(rECEF[1],rECEF[0]) * macros.R2D

    return lat, lon

if __name__ == "__main__":
    run(
        True,       # show_plots
        29994000,   # semi-major axis [m]
        56,         # orbit inclination [deg]
        24,         # total number of satellites (int)
        3,          # number of orbit planes (int)
        1           # phasing (int)
    )
