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

This script shows how to set up the :ref:`flybyPoint` to compute a guidance reference frame that allows to track the
direction of the celestial object being flown by. In this scenario, an Earth flyby is simulated. Effectively,
:ref:`flybyPoint` computes a Hill reference frame, which is however referred to a body that is not the spacecraft's
primary gravity body. To align a specific body-frame direction with the direction of the body, for example the direction
 of a camera that needs to take pictures of an asteroid, the user needst to specify the ``sigma_R0R'' parameter in the
 :ref:`attTrackingError`: this allows to introduce an offset rotation that aligns the desired axis with the celestial
 body, rather than the  body's :math:`x` axis. The script is run with the following input arguments:

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioFlybyPoint.py

Illustration of Simulation Results
----------------------------------

The scenario is run assuming to not neglect Earth's gravity. The relative position and velocity of the spacecraft w.r.t.
Earth are updated every 10 minutes. The following figures show the reference attitude, angular rates and angular
accelerations required to track Earth during the flyby, according to the rectilinear flyby model outlined in
:ref:`flybyPoint`.

.. image:: /_images/Scenarios/scenarioFlybyPoint3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlybyPoint4.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlybyPoint5.svg
   :align: center

Lastly, the following plot shows the error angles between the camera location, along the body axis :math:`[0,1,0]`, and
the desired out-of-plane body axis :math:`[1,0,0]`. It can be observed that both angle errors drop to zero soon after
the initial transient. Additionally, errors are observed around 175 minutes, when the spacecraft reaches its closes
approach to Earth. The errors grow due to the inaccuracy of the model of :ref:`flybyPoint`, which does not account for
the effects of Earth's gravity on the motion of the spacecraft.

.. image:: /_images/Scenarios/scenarioFlybyPoint6.svg
   :align: center

"""

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (flybyPoint, attTrackingError, mrpFeedback, rwMotorTorque)
from Basilisk.simulation import (simpleNav, spacecraft, reactionWheelStateEffector, planetEphemeris)
from Basilisk.utilities import (SimulationBaseClass, simIncludeRW, macros, unitTestSupport, orbitalMotion,
                                simIncludeGravBody)
from Basilisk.utilities import RigidBodyKinematics as rbk

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])




def run(show_plots, zeroEarthGravity, dtFilterData):

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Shows the simulation progress bar in the terminal
    scSim.SetProgressBar(True)

    # set the simulation time variable used later on
    simulationTime = macros.hour2nano(6.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.5)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # # Specify orbital parameters of the asteroid
    timeInitString = "2011 January 1 0:00:00.0"
    G = 6.67408 * (10 ** -11)  # m^3 / kg*s^2
    massBennu = 7.329 * (10 ** 10)  # kg
    muBennu = G * massBennu  # Bennu grav. parameter, m^3/s^2011

    # Create additional gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createBodies(["earth", "sun"])
    earth = gravFactory.gravBodies["earth"]
    earth.isCentralBody = True  # ensures the asteroid is the central gravitational body

    # Create and configure the default SPICE support module. The first step is to store
    # the date and time of the start of the simulation.
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)
    gravFactory.spiceObject.zeroBase = "earth"

    # Add the SPICE object to the simulation task list
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)

    # set Earth's gravity to zero to show rectilinear flyby
    if zeroEarthGravity:
        earth.mu = 0

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"

    # Connect all gravitational bodies to the spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.r_CN_NInit = [-2.08e8, 15e6, 0]  # m   - r_CN_N
    scObject.hub.v_CN_NInit = [2e4, 0, 0]  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    RW1 = rwFactory.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=100.)
    RW2 = rwFactory.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=200.)
    RW3 = rwFactory.create('Honeywell_HR16', [0, 0, 1], maxMomentum=50., Omega=300.)

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process.  This is required for the UpdateState() method
    # to be called which logs the RW states
    scSim.AddModelToTask(simTaskName, rwStateEffector, None, 2)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # se tup flybyPoint guidance module
    flybyGuid = flybyPoint.FlybyPoint()
    flybyGuid.ModelTag = "flybyPoint"
    flybyGuid.setTimeBetweenFilterData(dtFilterData)
    flybyGuid.setSignOfOrbitNormalFrameVector(-1)
    scSim.AddModelToTask(simTaskName, flybyGuid)

    cameraAxis_B = np.array([0, 1, 0])
    outOfPlaneAxis_B = np.array([flybyGuid.getSignOfOrbitNormalFrameVector(), 0, 0])
    crossProductAxis_B = np.cross(cameraAxis_B, outOfPlaneAxis_B)
    R0R = np.array([-cameraAxis_B,
                    crossProductAxis_B,
                    np.cross(crossProductAxis_B, cameraAxis_B)])

    # set up the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    attErrorConfig.sigma_R0R = rbk.C2MRP(R0R)
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)

    # setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
    rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
    rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueWrap, rwMotorTorqueConfig)

    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    rwMotorTorqueConfig.controlAxes_B = controlAxes_B

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 1000
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, snTransLog)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, snAttLog)
    attRefLog = flybyGuid.attRefOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, attRefLog)
    attGuidLog = attErrorConfig.attGuidOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, attGuidLog)
    torqueLog = mrpControlConfig.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, torqueLog)

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # If the exact same RW configuration states are to be used by the simulation and fsw, then the following helper
    # function is convenient to extract the fsw RW configuration message from the rwFactory setup earlier.
    fswRwParamMsg = rwFactory.getConfigMessage()

    #
    # create simulation messages
    #

    # link messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    flybyGuid.filterInMsg.subscribeTo(sNavObject.transOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attErrorConfig.attRefInMsg.subscribeTo(flybyGuid.attRefOutMsg)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControlConfig.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControlConfig.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwMotorTorqueConfig.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueConfig.vehControlInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueConfig.rwMotorTorqueOutMsg)

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
    dataPos = snTransLog.r_BN_N
    dataVel = snTransLog.v_BN_N
    att = snAttLog.sigma_BN
    refAtt = attRefLog.sigma_RN
    refRate = attRefLog.omega_RN_N
    refAcc = attRefLog.domega_RN_N
    guidAtt = attGuidLog.sigma_BR
    cmdTorque = torqueLog.torqueRequestBody

    cameraAxis_N = []
    outOfPlaneAxis_N = []
    pointingErrorCamera = []
    pointingErrorOutOfPlane = []
    for i in range(len(dataPos)):
        ur = dataPos[i] / np.linalg.norm(dataPos[i])
        uh = np.cross(dataPos[i], dataVel[i]) / np.linalg.norm(np.cross(dataPos[i], dataVel[i]))
        ut = np.cross(uh, ur)
        if flybyGuid.getSignOfOrbitNormalFrameVector() == -1:
            uh = np.cross(ut, ur)
        BN = rbk.MRP2C(att[i])
        NB = BN.transpose()
        cameraAxis_N.append(np.matmul(NB, cameraAxis_B))
        outOfPlaneAxis_N.append(np.matmul(NB, outOfPlaneAxis_B))
        pointingErrorCamera.append(np.arccos(min(max(-np.dot(ur, cameraAxis_N[i]), -1), 1)))
        pointingErrorOutOfPlane.append(np.arccos(min(max(np.dot(uh, outOfPlaneAxis_N[i]), -1), 1)))


    #
    #   plot the results
    #
    timeData = snTransLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    figureList = {}

    plot_position(timeData, dataPos)
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_velocity(timeData, dataVel)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_ref_attitude(timeData, refAtt)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plot_ref_rates(timeData, refRate)
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plot_ref_accelerations(timeData, refAcc)
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

    plot_angular_errors(timeData, pointingErrorCamera, pointingErrorOutOfPlane)
    pltName = fileName + "6"
    figureList[pltName] = plt.figure(6)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList

# Plotting functions
def plot_position(timeData, dataPos):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeData, dataPos[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$r_{BN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Inertial Position [m]')

def plot_velocity(timeData, dataVel):
    """Plot the attitude errors."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataVel[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$v_{BN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Inertial Velocity [m/s]')

def plot_ref_attitude(timeData, sigma_RN):
    """Plot the attitude errors."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, sigma_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_{RN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference attitude')

def plot_ref_rates(timeData, omega_RN):
    """Plot the attitude errors."""
    plt.figure(4)
    for idx in range(3):
        plt.plot(timeData, omega_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{RN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Ref. rates [rad/s]')

def plot_ref_accelerations(timeData, omegaDot_RN):
    """Plot the attitude errors."""
    plt.figure(5)
    for idx in range(3):
        plt.plot(timeData, omegaDot_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\dot{\omega}_{RN,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Ref. accelerations [rad/s^2]')

def plot_angular_errors(timeData, errCamera, errOutOfPlane):
    """Plot the attitude errors."""
    plt.figure(6)
    plt.plot(timeData, errCamera, label=r'$\Delta \theta_{camera}$')
    plt.plot(timeData, errOutOfPlane, label=r'$\Delta \theta_{out of plane}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Angular errors [rad]')

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False, # zeroEarthGravity
        600,   # dtFilterData
    )
