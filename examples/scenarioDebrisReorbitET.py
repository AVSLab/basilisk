#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

Demonstrates a basic debris reorbit scenario from geostationary orbit using the Electrostatic Tractor (ET) concept and how to visualize the simulation
data in :ref:`Vizard <vizard>`. This scenario shows how to use the :ref:`etSphericalControl` module for ET relative
motion control and also illustrates the usage of the :ref:`msmForceTorque` to calculate the electrostatic forces with the Multi-Sphere Method (MSM). This simulation simply uses a single sphere to represent each spacecraft. The servicing satellite is charged to a positive electric potential, while the other satellite (the debris) is uncontrolled and charged to a negative potential. The purpose of this script is to show an explicit method to
setup the ET reorbit simulation, and also show how to store the Basilisk simulation data to be able to visualize
both satellite's motions within the :ref:`Vizard <vizard>` application.

The script is found in the folder ``src/examples`` and executed by using::

      python3 scenarioDebrisReorbitET.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the servicer spacecraft and the associated Flight Software (FSW) algorithm
module, as well as the debris object.

.. image:: /_images/static/test_scenarioDebrisReorbitET.svg
   :align: center

When the simulation completes several plots are shown for the separation distance of the two satellites, the inertial position of both spacecraft, and the semi-major axis change of the debris.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

.. image:: /_images/Scenarios/scenarioDebrisReorbitET1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDebrisReorbitET2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDebrisReorbitET3.svg
   :align: center


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing a servicer that reorbits a debris using electrostatic forces.
# Author:   Julian Hammerl
# Creation Date:  May 20, 2021
#

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import etSphericalControl
from Basilisk.simulation import simpleNav, spacecraft, extForceTorque, msmForceTorque
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                unitTestSupport, vizSupport)

try:
    from Basilisk.simulation import vizInterface

    vizFound = True
except ImportError:
    vizFound = False

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenarios can be run with the followings setup parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    # Create simulation variable names
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(dynProcessName, 1)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(300.0)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize servicer spacecraft object and set properties
    scObjectServicer = spacecraft.Spacecraft()
    scObjectServicer.ModelTag = "Servicer"
    scObjectServicer.hub.mHub = 500.0  # [kg] servicer mass

    # initialize servicer spacecraft object and set properties
    scObjectDebris = spacecraft.Spacecraft()
    scObjectDebris.ModelTag = "DebrisSat"
    scObjectDebris.hub.mHub = 2000.0  # kg

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(dynTaskName, scObjectServicer)
    scSim.AddModelToTask(dynTaskName, scObjectDebris)

    # Create VehicleConfig messages including the S/C mass (for etSphericalControl)
    servicerConfigOutData = messaging.VehicleConfigMsgPayload()
    servicerConfigOutData.massSC = scObjectServicer.hub.mHub
    servicerVehicleConfigMsg = messaging.VehicleConfigMsg().write(servicerConfigOutData)

    debrisConfigOutData = messaging.VehicleConfigMsgPayload()
    debrisConfigOutData.massSC = scObjectDebris.hub.mHub
    debrisVehicleConfigMsg = messaging.VehicleConfigMsg().write(debrisConfigOutData)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spaceCraftPlus
    scObjectServicer.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObjectDebris.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # setup MSM module
    MSMmodule = msmForceTorque.MsmForceTorque()
    MSMmodule.ModelTag = "msmForceTorqueTag"
    scSim.AddModelToTask(dynTaskName, MSMmodule)

    # define electric potentials
    voltServicerInMsgData = messaging.VoltMsgPayload()
    voltServicerInMsgData.voltage = 25000.  # [V] servicer potential
    voltServicerInMsg = messaging.VoltMsg().write(voltServicerInMsgData)

    voltDebrisInMsgData = messaging.VoltMsgPayload()
    voltDebrisInMsgData.voltage = -25000.  # [V] debris potential
    voltDebrisInMsg = messaging.VoltMsg().write(voltDebrisInMsgData)

    # create a list of sphere body-fixed locations and associated radii
    spPosListServicer = [[0., 0., 0.]]  # one sphere located at origin of body frame
    rListServicer = [2.]  # radius of sphere is 2m
    spPosListDebris = [[0., 0., 0.]]  # one sphere located at origin of body frame
    rListDebris = [3.]  # radius of sphere is 3m

    # add spacecraft to state
    MSMmodule.addSpacecraftToModel(scObjectServicer.scStateOutMsg, messaging.DoubleVector(rListServicer),
                                   unitTestSupport.npList2EigenXdVector(spPosListServicer))
    MSMmodule.addSpacecraftToModel(scObjectDebris.scStateOutMsg, messaging.DoubleVector(rListDebris),
                                   unitTestSupport.npList2EigenXdVector(spPosListDebris))

    # subscribe input messages to module
    MSMmodule.voltInMsgs[0].subscribeTo(voltServicerInMsg)
    MSMmodule.voltInMsgs[1].subscribeTo(voltDebrisInMsg)

    # setup extForceTorque module for Servicer
    # the electrostatic force from the MSM module is read in through the messaging system
    extFTObjectServicer = extForceTorque.ExtForceTorque()
    extFTObjectServicer.ModelTag = "eForceServicer"
    extFTObjectServicer.cmdForceInertialInMsg.subscribeTo(MSMmodule.eForceOutMsgs[0])
    scObjectServicer.addDynamicEffector(extFTObjectServicer)
    scSim.AddModelToTask(dynTaskName, extFTObjectServicer)

    # setup extForceTorque module for Debris
    # the electrostatic force from the MSM module is read in through the messaging system
    extFTObjectDebris = extForceTorque.ExtForceTorque()
    extFTObjectDebris.ModelTag = "eForceDebris"
    extFTObjectDebris.cmdForceInertialInMsg.subscribeTo(MSMmodule.eForceOutMsgs[1])
    scObjectDebris.addDynamicEffector(extFTObjectDebris)
    scSim.AddModelToTask(dynTaskName, extFTObjectDebris)

    # setup extForceTorque module
    # the control force from the ET relative motion control module is read in through the messaging system
    extFTObjectServicerControl = extForceTorque.ExtForceTorque()
    extFTObjectServicerControl.ModelTag = "controlServicer"
    scObjectServicer.addDynamicEffector(extFTObjectServicerControl)
    scSim.AddModelToTask(dynTaskName, extFTObjectServicerControl)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObjectServicer = simpleNav.SimpleNav()
    sNavObjectServicer.ModelTag = "SimpleNavigation"
    sNavObjectServicer.scStateInMsg.subscribeTo(scObjectServicer.scStateOutMsg)
    scSim.AddModelToTask(dynTaskName, sNavObjectServicer)

    sNavObjectDebris = simpleNav.SimpleNav()
    sNavObjectDebris.ModelTag = "SimpleNavigation3"
    sNavObjectDebris.scStateInMsg.subscribeTo(scObjectDebris.scStateOutMsg)
    scSim.AddModelToTask(dynTaskName, sNavObjectDebris)

    # ----- fsw ----- #
    fswProcessName = "fswProcess"
    fswTaskName = "fswTask"
    fswProcess = scSim.CreateNewProcess(fswProcessName, 1)
    fswTimeStep = macros.sec2nano(300.0)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    # setup ET Relative Motion Control module
    etSphericalControlObj = etSphericalControl.etSphericalControl()
    etSphericalControlObj.ModelTag = "ETcontrol"

    # connect required messages
    etSphericalControlObj.servicerTransInMsg.subscribeTo(sNavObjectServicer.transOutMsg)  # servicer translation
    etSphericalControlObj.debrisTransInMsg.subscribeTo(sNavObjectDebris.transOutMsg)  # debris translation
    etSphericalControlObj.servicerAttInMsg.subscribeTo(sNavObjectServicer.attOutMsg)  # servicer attitude
    etSphericalControlObj.servicerVehicleConfigInMsg.subscribeTo(servicerVehicleConfigMsg)  # servicer mass
    etSphericalControlObj.debrisVehicleConfigInMsg.subscribeTo(debrisVehicleConfigMsg)  # debris mass
    etSphericalControlObj.eForceInMsg.subscribeTo(MSMmodule.eForceOutMsgs[0])  # eForce on servicer (for feed-forward)

    # set module parameters
    # feedback gain matrices
    Ki = 4e-7
    Pi = 1.85 * Ki ** 0.5
    etSphericalControlObj.K = [Ki, 0.0, 0.0,
                                0.0, Ki, 0.0,
                                0.0, 0.0, Ki]
    etSphericalControlObj.P = [Pi, 0.0, 0.0,
                                0.0, Pi, 0.0,
                                0.0, 0.0, Pi]
    # desired relative position in spherical coordinates (reference state)
    etSphericalControlObj.L_r = 30.0  # separation distance
    etSphericalControlObj.theta_r = 0.  # in-plane rotation angle
    etSphericalControlObj.phi_r = 0.  # out-of-plane rotation angle
    etSphericalControlObj.mu = mu  # gravitational parameter

    # add module to fsw task
    scSim.AddModelToTask(fswTaskName, etSphericalControlObj)
    # connect output control thrust force with external force on servicer
    extFTObjectServicerControl.cmdForceInertialInMsg.subscribeTo(etSphericalControlObj.forceInertialOutMsg)

    #
    #   set initial Spacecraft States
    #
    # setup the servicer orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 42164. * 1e3  # [m] geostationary orbit
    oe.e = 0.
    oe.i = 0.
    oe.Omega = 0.
    oe.omega = 0
    oe.f = 0.
    r_SN, v_SN = orbitalMotion.elem2rv(mu, oe)
    scObjectServicer.hub.r_CN_NInit = r_SN  # m
    scObjectServicer.hub.v_CN_NInit = v_SN  # m/s
    oe = orbitalMotion.rv2elem(mu, r_SN, v_SN)

    # setup debris object states
    r_DS = np.array([0, -50.0, 0.0])  # relative position of debris, 50m behind servicer in along-track direction
    r_DN = r_DS + r_SN
    v_DN = v_SN
    scObjectDebris.hub.r_CN_NInit = r_DN  # m
    scObjectDebris.hub.v_CN_NInit = v_DN  # m/s

    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n  # orbit period

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 1000
    simulationTime = macros.sec2nano(1. * P)
    samplingTime = simulationTime // (numDataPoints - 1)
    dataRecS = scObjectServicer.scStateOutMsg.recorder(samplingTime)
    dataRecD = scObjectDebris.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTaskName, dataRecS)
    scSim.AddModelToTask(dynTaskName, dataRecD)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    if vizFound:
        # setup MSM information
        msmInfoServicer = vizInterface.MultiSphereInfo()
        msmInfoServicer.msmChargeInMsg.subscribeTo(MSMmodule.chargeMsmOutMsgs[0])
        msmServicerList = []
        for (pos, rad) in zip(spPosListServicer, rListServicer):
            msmServicer = vizInterface.MultiSphere()
            msmServicer.position = pos
            msmServicer.radius = rad
            msmServicer.isOn = 1
            msmServicer.maxValue = 30e-6  # Coulomb
            msmServicer.currentValue = 4e-6  # Coulomb
            msmServicerList.append(msmServicer)
        msmInfoServicer.msmList = vizInterface.MultiSphereVector(msmServicerList)

        msmInfoDebris = vizInterface.MultiSphereInfo()
        msmInfoDebris.msmChargeInMsg.subscribeTo(MSMmodule.chargeMsmOutMsgs[1])
        msmDebrisList = []
        for (pos, rad) in zip(spPosListDebris, rListDebris):
            msmDebris = vizInterface.MultiSphere()
            msmDebris.position = pos
            msmDebris.radius = rad
            msmDebris.isOn = 1
            msmDebris.maxValue = 30e-6  # Coulomb
            msmDebris.currentValue = 4e-6  # Coulomb
            msmDebris.neutralOpacity = 50  # opacity value between 0 and 255
            msmDebrisList.append(msmDebris)
        msmInfoDebris.msmList = vizInterface.MultiSphereVector(msmDebrisList)

        viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, [scObjectServicer, scObjectDebris]
                                                  # , saveFile=fileName
                                                  , msmInfoList=[msmInfoServicer, msmInfoDebris]
                                                  )

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   retrieve the logged data
    posDataS = dataRecS.r_BN_N
    velDataS = dataRecS.v_BN_N
    posDataD = dataRecD.r_BN_N
    velDataD = dataRecD.v_BN_N

    timeData = dataRecS.times()

    np.set_printoptions(precision=16)

    figureList = plotOrbits(timeData, posDataS, velDataS, posDataD, velDataD, oe, mu, P, earth)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


def plotOrbits(timeData, posDataS, velDataS, posDataD, velDataD, oe, mu, P, planet):
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    relPosData = posDataS[:, 0:3] - posDataD[:, 0:3]
    relPosMagn = np.linalg.norm(relPosData, axis=1)
    plt.plot(timeData * macros.NANO2SEC / P, relPosMagn[:])
    plt.xlabel('Time [orbits]')
    plt.ylabel('Separation [m]')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    # draw orbit in perifocal frame
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(2)
    plt.axis('equal')
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = '#008800'
    planetRadius = planet.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(posDataS)):
        oeData = orbitalMotion.rv2elem(mu, posDataS[idx, 0:3], velDataS[idx, 0:3])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=3.0)
    # draw the full osculating orbit from the initial conditions
    fData = np.linspace(0, 2 * np.pi, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555'
             )
    plt.xlabel('$x$ Cord. [km]')
    plt.ylabel('$y$ Cord. [km]')
    plt.grid()

    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    aData = []
    for idx in range(0, len(posDataS)):
        oeData = orbitalMotion.rv2elem(mu, posDataD[idx, 0:3], velDataD[idx, 0:3])
        aData.append(oeData.a)
    plt.plot(timeData * macros.NANO2SEC / P, (aData - oe.a) / 1000., color='#aa0000', linewidth=3.0)
    plt.xlabel('Time [orbits]')
    plt.ylabel('Increase of semi-major axis [km]')

    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
    )
