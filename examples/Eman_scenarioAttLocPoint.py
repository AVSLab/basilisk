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

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/AQAcHAmxcaU" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

Discusses how to use guidance modules to point a particular spacecraft axis towards an Earth
fixed location, Boulder in this example.
This script sets up a 6-DOF spacecraft which is orbiting the Earth.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttLocPoint.py

The simulation uses :ref:`groundLocation` to create an output message with Boulder's inertial
position.  This is fed to the 2D pointing module :ref:`locationPointing` which
directs the 3rd body axis to point towards Boulder.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

The following 2 plots illustrate the 2D pointing error and the external attitude control torque vector components.

.. image:: /_images/Scenarios/scenarioAttLocPoint1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttLocPoint2.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav(),
#           locationPoint() modules.  Will point a spacecraft axis at an Earth fixed location.
# Author:   Hanspeter Schaub
# Creation Date:  May 9, 2021
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
# import message declarations
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import locationPointing
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import groundLocation
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
def plot_attitude_error(timeLineSet, dataSigmaBR):
    """Plot the attitude result."""
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    vectorData = dataSigmaBR
    sNorm = np.array([np.linalg.norm(v) for v in vectorData])
    plt.plot(timeLineSet, sNorm,
             color=unitTestSupport.getLineColor(1, 3),
             )
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error Norm $|\sigma_{B/R}|$')
    ax.set_yscale('log')

def plot_control_torque(timeLineSet, dataLr):
    """Plot the control torque response."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeLineSet, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')


def plot_rate_error(timeLineSet, dataOmegaBR):
    """Plot the body angular velocity tracking error."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeLineSet, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')
    return


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
    simulationTime = macros.min2nano(20.)

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

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
   # define the simulation inertia
    I = [455.38, 2.99, -8.08082,
         2.99, 450.28, -0.5733578,
         -8.08082, -0.5733578, 260.56]
    scObject.hub.mHub = 719.942  # kg - spacecraft mass - MBZ 
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    #   initialize Spacecraft States with initialization variables
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = (6378 + 600)*1000.  # meters
    oe.e = 0.1
    oe.i = 63.3 * macros.D2R
    oe.Omega = 88.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 135.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    # use the input flag to determine which external torque should be applied
    # Note that all variables are initialized to zero.  Thus, not setting this
    # vector would leave it's components all zero for the simulation.
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Create the ground location
    groundStation = groundLocation.GroundLocation()
    groundStation.ModelTag = "BoulderGroundStation"
    groundStation.planetRadius = astroFunctions.E_radius*1e3  # meters
    groundStation.specifyLocation(np.radians(40.009971), np.radians(-105.243895), 1624)
    groundStation.minimumElevation = np.radians(10.)
    groundStation.maximumRange = 1e9  # meters
    groundStation.addSpacecraftToModel(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, groundStation)

    #
    #   setup the FSW algorithm tasks
    #

    # setup Boulder pointing guidance module
    locPointConfig = locationPointing.locationPointingConfig()
    locPointWrap = scSim.setModelDataWrap(locPointConfig)
    locPointWrap.ModelTag = "locPoint"
    scSim.AddModelToTask(simTaskName, locPointWrap, locPointConfig)
    locPointConfig.pHat_B = [0, 0, 1]
    locPointConfig.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    locPointConfig.useBoresightRateDamping = 1
    locPointConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    locPointConfig.locationInMsg.subscribeTo(groundStation.currentGroundStateOutMsg)
    # grMsgData = messaging.GroundStateMsgPayload()
    # grMsg = messaging.GroundStateMsg().write(grMsgData)
    # locPointConfig.locationInMsg.subscribeTo(grMsg)

    # setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.guidInMsg.subscribeTo(locPointConfig.attGuidOutMsg)
    mrpControlConfig.K = 5.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # connect torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    mrpLog = mrpControlConfig.cmdTorqueOutMsg.recorder(samplingTime)
    attErrLog = locPointConfig.attGuidOutMsg.recorder(samplingTime)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, mrpLog)
    scSim.AddModelToTask(simTaskName, attErrLog)
    scSim.AddModelToTask(simTaskName, snAttLog)
    scSim.AddModelToTask(simTaskName, snTransLog)

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    configDataMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    mrpControlConfig.vehConfigInMsg.subscribeTo(configDataMsg)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  # , saveFile=fileName
                                                  )
        vizSupport.addLocation(viz, stationName="Boulder Station"
                               , parentBodyName=earth.displayName
                               , r_GP_P=groundStation.r_LP_P_Init
                               , fieldOfView=np.radians(160.)
                               , color='pink'
                               , range=2000.0*1000  # meters
                               )
        viz.settings.spacecraftSizeMultiplier = 1.5
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
    #   retrieve the logged data
    #
    dataLr = mrpLog.torqueRequestBody
    dataSigmaBR = attErrLog.sigma_BR
    dataOmegaBR = attErrLog.omega_BR_B

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeLineSet = attErrLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_attitude_error(timeLineSet, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_control_torque(timeLineSet, dataLr)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_rate_error(timeLineSet, dataOmegaBR)

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
        True  # show_plots
    )
