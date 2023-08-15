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

Demonstrates how to use guidance modules to align the spacecraft frame to the velocity-pointing frame.
This script sets up a 6-DOF spacecraft which is on a hyperbolic trajectory near Earth.
It aligns the spacecraft to point along the velocity vector throughout the orbit.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttitudeGuidance.py


The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenarioAttGuideHyperbolic.svg
   :align: center

When the simulation completes 4 plots are shown. This first three show the MRP attitude history, the rate
tracking errors, and the control torque vector. The fourth shows the hyperbolic trajectory
and the segment of that trajectory flown during the simulation.

The basic simulation setup is the same as the one used in
:ref:`scenarioAttitudeGuidance`.
The dynamics simulation is setup using a :ref:`Spacecraft` module to which a gravity
effector is attached.  Note that both the rotational and translational degrees of
freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.  For more
information on how to setup an orbit, see :ref:`scenarioBasicOrbit`.

Where the Attitude Guidance Tutorial pointed the spacecraft relative to the Hill frame, this tutorial
points it relative to the velocity vector.  Note that in contrast to Hill pointing mode used in
:ref:`scenarioAttitudeGuidance`, the orbit velocity frame pointing
requires the attracting celestial body gravitational constant ``mu`` to be set.
Note that if the celestial body ephemeris input message is not connected then
a zero message is created which corresponds to the planet having a zero position and velocity vector.
If non-zero ephemeris information is required then the input name must point
to a message of type :ref:`EphemerisMsgPayload`.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useAltBodyFrame = False

This scenario shown has the ``useAltBodyFrame`` flag turned off.  This means that we seek
to align the body frame :math:`\cal B` with the velocity vector :math:`\cal V`.

.. image:: /_images/Scenarios/scenarioAttGuideHyperbolic10.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttGuideHyperbolic20.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttGuideHyperbolic30.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttGuideHyperbolic40.svg
   :align: center

::

    show_plots = True, useAltBodyFrame = True

Here the control should not align the principal body frame :math:`\cal B` with :math:`\cal V`, but rather an alternate,
corrected body frame *Bc*.  For example, if a thruster is located on the :\math:`\hat b_1` face, and it
is desired to point it along the negative V-bar, this is achieved through::

  attError.sigma_R0R = [0,0,-1]

This corrected body frame has an orientation which is rotated 180 degrees about :math:`\hat b_3`,
to point the correct face of the spacecraft along the negative V-bar.

.. image:: /_images/Scenarios/scenarioAttGuideHyperbolic11.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttGuideHyperbolic21.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttGuideHyperbolic31.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttGuideHyperbolic41.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav(),
#           mrpFeedback() with attitude navigation modules.  This script is a
#           spinoff from the attitude guidance tutorial, it implements a hyperbolic
#           trajectory and uses the velocityPoint module.
# Author:   Anne Bennett
# Creation Date:  Aug. 28th, 2017
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import mrpFeedback, attTrackingError, velocityPoint
from Basilisk.simulation import extForceTorque, simpleNav, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody, unitTestSupport
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

def plot_track_error_norm(timeLineSet, dataSigmaBR):
    """Plot the attitude tracking error norm value."""
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
    """Plot the attiude control torque effort."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeLineSet, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')

def plot_rate_error(timeLineSet, dataOmegaBR):
    """Plot the body angular velocity tracking errors."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeLineSet, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')


def plot_orbit(oe, mu, planet_radius, dataPos, dataVel):
    """Plot the spacecraft orbit trajectory."""
    # draw orbit in perifocal frame
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(4, figsize=np.array((1.0, 1.)) * 4.75, dpi=100)
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = '#008800'
    # planet = gravFactory.createEarth()
    planetRadius = planet_radius / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(dataPos)):
        oeData = orbitalMotion.rv2elem(mu, dataPos[idx], dataVel[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000,
             color='#aa0000', linewidth=3.0, label='Simulated Flight')

    plt.axis(np.array([-1, 1, -1, 1]) * 1.25 * np.amax(rData) / 1000)

    # draw the full osculating orbit from the initial conditions
    tempAngle = (1. / 2.) * (np.pi - 2 * np.arcsin(1 / oe.e)) * 1.01
    fData = np.linspace(np.pi - tempAngle, -np.pi + tempAngle, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555', label='Orbit Track')
    plt.xlabel('$i_e$ Cord. [km]')
    plt.ylabel('$i_p$ Cord. [km]')
    plt.legend(loc='lower left')
    plt.grid()


def run(show_plots, useAltBodyFrame):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useAltBodyFrame (bool): Specify if the alternate body frame should be aligned with Hill frame.

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.sec2nano(750.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
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
    oe.a = -150000.0 * 1000  # meters
    oe.e = 1.5
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 30 * macros.D2R
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

    #
    #   setup the FSW algorithm tasks
    #

    # setup velocityPoint guidance module
    attGuidance = velocityPoint.velocityPoint()
    attGuidance.ModelTag = "velocityPoint"
    attGuidance.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    # No celestial body input message is connect.  Thus, the default behavior is to create an empty planet
    # ephemeris message which puts the earth at (0,0,0) origin with zero speed.
    # Note that mu must be assigned to attGuidance.mu when using the velocityPoint() module:
    attGuidance.mu = mu
    scSim.AddModelToTask(simTaskName, attGuidance)

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)
    if useAltBodyFrame:
        attError.sigma_R0R = [0, 0, -1]
    attError.attRefInMsg.subscribeTo(attGuidance.attRefOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
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
    scSim.AddModelToTask(simTaskName, mrpLog)
    scSim.AddModelToTask(simTaskName, attErrLog)
    scSim.AddModelToTask(simTaskName, snAttLog)
    scSim.AddModelToTask(simTaskName, snTransLog)
    #
    # create simulation messages
    #

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              # , saveFile=fileName
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

    #
    #   retrieve the logged data
    #
    dataLr = mrpLog.torqueRequestBody
    dataSigmaBR = attErrLog.sigma_BR
    dataOmegaBR = attErrLog.omega_BR_B
    dataPos = snTransLog.r_BN_N
    dataVel = snTransLog.v_BN_N
    dataSigmaBN = snAttLog.sigma_BN
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeLineSet = attErrLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    figureList = {}
    plot_track_error_norm(timeLineSet, dataSigmaBR)
    pltName = fileName + "1" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(1)

    plot_control_torque(timeLineSet, dataLr)
    pltName = fileName + "2" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(2)

    plot_rate_error(timeLineSet, dataOmegaBR)
    pltName = fileName + "3" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(3)

    plot_orbit(oe, earth.mu, earth.radEquator, dataPos, dataVel)
    pltName = fileName + "4" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(4)

    if show_plots:
        plt.show()

    # close the plots to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False  # useAltBodyFrame
        )
