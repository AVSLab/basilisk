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

Demonstrates using :ref:`spacecraftLocation` to check for times where the antenna axis on the primary spacecraft
has access to another spacecraft.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioSpacecraftLocation.py

When the simulation completes a plot is shown with the access times illustrated.

.. image:: /_images/Scenarios/scenarioSpacecraftLocation1.svg
   :align: center


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing a servicer (3-axis attitude controlled) and a tumbling debris object.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 29, 2019
#

import copy
import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError, hillPoint)
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav, spacecraft
from Basilisk.simulation import spacecraftLocation
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
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(140.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize servicer spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "Servicer"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # create the debris object states
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "Debris"
    I2 = [600., 0., 0.,
          0., 650., 0.,
          0., 0, 450.]
    scObject2.hub.mHub = 350.0  # kg
    scObject2.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I2)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, scObject2)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObject2.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # add external control torque to scObject
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "extTorque"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, sNavObject)

    # setup spacecraft location access module
    scLocation = spacecraftLocation.SpacecraftLocation()
    scLocation.ModelTag = "scAccess"
    scLocation.addSpacecraftToModel(scObject2.scStateOutMsg)
    scLocation.primaryScStateInMsg.subscribeTo(scObject.scStateOutMsg)
    scLocation.rEquator = earth.radEquator
    scLocation.rPolar = earth.radEquator*0.98
    scLocation.aHat_B = [0, 1, 0]
    scLocation.theta = np.radians(10.)
    scLocation.maximumRange = 55.
    scSim.AddModelToTask(simTaskName, scLocation)

    #
    #   setup the FSW algorithm tasks
    #

    # setup hillPoint guidance module
    attGuidanceConfig = hillPoint.hillPointConfig()
    attGuidanceWrap = scSim.setModelDataWrap(attGuidanceConfig)
    attGuidanceWrap.ModelTag = "hillPoint"
    attGuidanceConfig.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    scSim.AddModelToTask(simTaskName, attGuidanceWrap, attGuidanceConfig)

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attError"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.attRefInMsg.subscribeTo(attGuidanceConfig.attRefOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    #
    #   Setup data logging before the simulation is initialized
    #
    accessRec = scLocation.accessOutMsgs[0].recorder()
    scSim.AddModelToTask(simTaskName, accessRec)

    #
    #   set initial Spacecraft States
    #
    # setup the servicer orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.0
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 90.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.02], [0.01]]  # rad/s - omega_CN_B

    # setup 1st debris object states
    oe2 = copy.deepcopy(oe)
    oe2.e += 0.000001
    oe2.f += 40./oe2.a
    r2N, v2N = orbitalMotion.elem2rv(mu, oe2)
    scObject2.hub.r_CN_NInit = r2N  # m   - r_CN_N
    scObject2.hub.v_CN_NInit = v2N  # m/s - v_CN_N
    scObject2.hub.sigma_BNInit = [[0.3], [0.1], [0.2]]  # sigma_CN_B
    scObject2.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    if vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                                  # , saveFile=fileName,
                                                  )
        vizSupport.addLocation(viz, stationName="antenna"
                               , parentBodyName='Servicer'
                               , r_GP_P=[0, 2, 0]
                               , gHat_P=[0, 1, 0]
                               , fieldOfView=2*scLocation.theta
                               , range=scLocation.maximumRange
                               , color='pink'
                               )
        viz.settings.showLocationCommLines = 1
        viz.settings.showLocationCones = 1
        viz.settings.showLocationLabels = 1
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
    #   plot the results
    #
    dataLog  = accessRec.hasAccess
    timeData = accessRec.times() * macros.NANO2MIN

    plt.figure(1)
    plt.plot(timeData, dataLog)
    plt.xlabel('Time [min]')
    plt.ylabel('Sat-Sat Access')

    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

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
        True   # show_plots
    )
