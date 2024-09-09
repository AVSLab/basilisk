#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
This scenario simulates a single Hohmann Transfer about Earth. The spacecraft utilizes a Velocity Pointing frame
to align the thrusters with the velocity direction throughout the flight.

Compared to :ref:`scenarioOrbitManeuver` and :ref:`scenarioHohmann` which use impulsive orbit maneuvers to perform
:math:`\Delta v`'s this scenario uses the :ref:`thrusterStateEffector` module to perform
a finite time burn. Two thrusters with different thrust values are used to perform the GTO burn and circularization burn.

To calculate the duration of each finite burn an assumption of constant mass is used allowing the duration to be calculated
using:

.. math::
    t_{burn} = \frac{\Delta v}{a_{Thrust}}

where :math:`a_{Thrust}` is the acceleration due to thrust and is calculated using:

 .. math::
    a_{Thrust} = \frac{F_{thrust}}{M_{spacecraft}}

where :math:`F_{thrust}` is the thrust force and :math:`M_{spacecraft}` is the spacecraft mass.

The required :math:`\Delta v` for each burn is found using the same manner as discussed in :ref:`scenarioHohmann`.

To show that the :ref:`thrusterStateEffector` thruster module works with variable time step integrators, this scenario
uses an RKF45 integrator instead of the usual RK4.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioOrbitManeuverTH.py

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

.. image:: /_images/Scenarios/scenarioOrbitManeuverTH1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioOrbitManeuverTH2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioOrbitManeuverTH3.svg
    :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, velocityPoint(), simpleNav(),
#           attTrackingError(), thrusterStateEffector() and mrpFeedback() modules.  Illustrates
#           a 6-DOV spacecraft pointing itself in the velocity direction and using thrusters to
#           perform a Hohmann Transfer.
# Author:   William Schwend
# Creation Date:  Sep. 09, 2024
#

import math
import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
# import message declarations
from Basilisk.architecture import messaging
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import velocityPoint
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import thrusterStateEffector
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

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

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(2.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    #   initialize Spacecraft States with initialization variables
    #

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # meters
    rGEO = math.pow(mu / math.pow((2. * np.pi) / (24. * 3600.), 2), 1. / 3.)
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.03], [0.01]]  # rad/s - omega_BN_B

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(0.25 * P)

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalTorque"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # create the set of thruster in the dynamics task
    thrusterSet = thrusterStateEffector.ThrusterStateEffector()
    scSim.AddModelToTask(simTaskName, thrusterSet)

    # set the integrator to a variable time step of 4th-5th order
    integratorObject = svIntegrators.svIntegratorRKF45(scObject)
    scObject.setIntegrator(integratorObject)

    # Make a fresh thruster factory instance, this is critical to run multiple times
    thFactory = simIncludeThruster.thrusterFactory()

    # create arrays for thrusters' locations and directions
    location = [[0, 0, 1]]
    direction = [[0, 0, 1]]
    # create the thruster devices by specifying the thruster type and its location and direction
    for pos_B, dir_B in zip(location, direction):
        thFactory.create('Blank_Thruster', pos_B, dir_B, cutoffFrequency=.1, MaxThrust=3000.0,
                         areaNozzle=0.046759, steadyIsp=318.0)

    for pos_B, dir_B in zip(location, direction):
        thFactory.create('Blank_Thruster', pos_B, dir_B, cutoffFrequency=.1, MaxThrust=420.0,
                         areaNozzle=0.046759, steadyIsp=318.0)

    # create thruster object container and tie to spacecraft object
    thrModelTag = "GTOThrusterDynamics"
    thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)

    # Configure thruster on-time message
    ThrOnTimeMsgData = messaging.THRArrayOnTimeCmdMsgPayload()
    ThrOnTimeMsgData.OnTimeRequest = [0, 0]
    thrOnTimeMsg = messaging.THRArrayOnTimeCmdMsg().write(ThrOnTimeMsgData)

    #
    #   Setup the FSW algorithm tasks
    #

    # setup velocityPoint guidance module
    attGuidance = velocityPoint.velocityPoint()
    attGuidance.ModelTag = "velocityPoint"
    attGuidance.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    attGuidance.mu = mu
    scSim.AddModelToTask(simTaskName, attGuidance)

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorVelocityPoint"
    scSim.AddModelToTask(simTaskName, attError)
    attError.sigma_R0R = [np.tan(np.pi/8), 0,  0]
    attError.attRefInMsg.subscribeTo(attGuidance.attRefOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.K = 3.5
    mrpControl.Ki = -1.0  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # connect torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    mrpLog = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    attErrLog = attError.attGuidOutMsg.recorder(samplingTime)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    thrCmdRec0 = thrusterSet.thrusterOutMsgs[0].recorder()
    thrCmdRec1 = thrusterSet.thrusterOutMsgs[1].recorder()
    scSim.AddModelToTask(simTaskName, thrCmdRec0)
    scSim.AddModelToTask(simTaskName, thrCmdRec1)
    scSim.AddModelToTask(simTaskName, dataRec)
    scSim.AddModelToTask(simTaskName, mrpLog)
    scSim.AddModelToTask(simTaskName, attErrLog)
    scSim.AddModelToTask(simTaskName, snAttLog)
    scSim.AddModelToTask(simTaskName, snTransLog)

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # connect messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(attGuidance.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    thrusterSet.cmdsInMsg.subscribeTo(thrOnTimeMsg)


    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName,  scObject
                                                  # , saveFile=fileName
                                                  , thrEffectorList=thrusterSet
                                                  , thrColors=vizSupport.toRGBA255("red")
                                                  )
        vizSupport.setActuatorGuiSetting(viz, showThrusterLabels=True)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()
    scSim.SetProgressBar(True)

    #
    #  get access to dynManager translational states for future access to the states
    #
    posRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubPosition)
    velRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)

    # The dynamics simulation is setup using a Spacecraft() module with the Earth's
    # gravity module attached.  Note that the rotational motion simulation is turned off to leave
    # pure 3-DOF translation motion simulation.  After running the simulation for 1/4 of a period
    # the simulation is stopped to apply the larger thruster to change the inertial velocity vector.
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Next, the state manager objects are called to retrieve the latest inertial position and
    # velocity vector components:
    rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
    vVt = unitTestSupport.EigenVector3d2np(velRef.getState())

    # Hohmann Transfer to GEO
    v0 = np.linalg.norm(vVt)
    r0 = np.linalg.norm(rVt)
    at = (r0 + rGEO) * .5
    v0p = np.sqrt(mu / at * rGEO / r0)
    dv0 = v0p - v0

    # Calculate the burn time for the first manuever
    maxThrustTH1 = 3000
    t0Burn = dv0 / (maxThrustTH1 / scObject.hub.mHub)
    T2 = macros.sec2nano(t0Burn)

    # Update thruster on-time message
    ThrOnTimeMsgData.OnTimeRequest = [t0Burn, 0]
    thrOnTimeMsg.write(ThrOnTimeMsgData, time=simulationTime)

    # To start up the simulation again, note that the total simulation time must be provided,
    # not just the next incremental simulation time.
    scSim.ConfigureStopTime(simulationTime + T2)
    scSim.ExecuteSimulation()

    # get the current spacecraft states
    rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
    vVt = unitTestSupport.EigenVector3d2np(velRef.getState())

    # Find the time from current orbital position to apogee
    oe1 = orbitalMotion.rv2elem(mu, rVt, vVt)
    n1 = np.sqrt(mu / oe1.a / oe1.a / oe1.a)
    T3Full = (np.pi) / n1
    E1 = 2*np.arctan(np.sqrt((1-oe1.e)/(1+oe1.e))*np.tan(oe1.f/2))
    T3P = np.sqrt((oe1.a * oe1.a * oe1.a)/mu)*(E1-oe1.e*np.sin(E1))
    T3 = macros.sec2nano(T3Full - T3P)

    # Run the simulation for until apogee is reached
    scSim.ConfigureStopTime(simulationTime + T2 + T3)
    scSim.ExecuteSimulation()

    # get the current spacecraft states
    rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
    vVt = unitTestSupport.EigenVector3d2np(velRef.getState())

    # Compute second Delta-v maneuver
    v1 = np.linalg.norm(vVt)
    r1 = np.linalg.norm(rVt)
    v1p = np.sqrt(mu / r1)
    n2 = np.sqrt(mu / r1 / r1 / r1)
    dv1 = v1p - v1

    # Calculate the burn time for the second maneuver
    maxThrustTH2 = 420
    t1Burn = dv1 / (maxThrustTH2 / scObject.hub.mHub)
    T4 = macros.sec2nano(t1Burn)

    # Update thruster on-time message
    ThrOnTimeMsgData.OnTimeRequest = [0, t1Burn]
    currentTime = int(simulationTime + T2 + T3)
    thrOnTimeMsg.write(ThrOnTimeMsgData, time=currentTime)

    # Run the simulation for second burn and quarter orbit after
    T5 = macros.sec2nano(0.25 * (np.pi) / n2)
    scSim.ConfigureStopTime(simulationTime + T2 + T3 + T4 +T5)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataRec.r_BN_N
    velData = dataRec.v_BN_N

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(3):
        plt.plot(dataRec.times() * macros.NANO2HOUR, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [h]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    # show SMA
    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    rData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem_parab(mu, posData[idx], velData[idx])
        rData.append(oeData.rmag / 1000.)
    plt.plot(dataRec.times() * macros.NANO2HOUR, rData, color='#aa0000',
                 )
    plt.xlabel('Time [h]')
    plt.ylabel('Radius [km]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    plt.plot(thrCmdRec0.times()*macros.NANO2MIN/60, thrCmdRec0.thrustForce, color='#aa0000', label='Thruster 1')
    plt.plot(thrCmdRec1.times()*macros.NANO2MIN/60, thrCmdRec1.thrustForce, label='Thruster 2')
    plt.legend(loc='upper right')
    plt.xlabel('Time [h]')
    plt.ylabel('Force implemented[N]')
    pltName = fileName + "3"
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
        True,  # show_plots
    )
