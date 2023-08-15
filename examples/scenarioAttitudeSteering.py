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

Demonstrates how to use the :ref:`mrpSteering` module to stabilize the attiude relative to the Hill Frame.
Details on the math of this module can be found in this `paper <http://doi.org/10.1016/j.actaastro.2018.03.022>`__.
This script sets up a spacecraft with 3 RWs which is orbiting the Earth.  The goal is to
illustrate how to use the :ref:`mrpSteering` module with a rate sub-servo system to control
the attitude.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttitudeSteering.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenarioAttitudeSteering.svg
   :align: center

The spacecraft is equipped with three RW, just as in the
:ref:`scenarioAttitudeFeedbackRW` tutorial.  The :ref:`hillPoint` guidance module is
used to align the body frame :math:`\cal B` to the Hill frame :math:`\cal H`.
The :ref:`rateServoFullNonlinear` module is
used to create the rate tracking sub-servo system.  How to setup the Hill frame
guidance module is discussed in
:ref:`scenarioAttitudeGuidance`.

When the simulation completes several plots are shown for the MRP attitude history, the rate
tracking errors, as well as the RW motor torque components, as well as the RW wheel speeds.

Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.

::

    show_plots = True, simCase = 0

Here an unknown external torque
is applied, but the integral feedback term is included as well.
Note that in the RW motor torque plot both the required control torque :math:`\hat u_B` and the true
motor torque :math:`u_B` are shown.  This illustrates that with this maneuver the RW devices are being
saturated, and the attitude still eventually stabilizes.

.. image:: /_images/Scenarios/scenarioAttitudeSteeringSigmaBR0.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeSteeringomegaBR0.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeSteeringrwUs0.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeSteeringOmega0.svg
   :align: center

In this simulation setup the integral feedback term is included, and the unknown external torque
is automatically compensated for to yield exponential convergence.  This convergence is despite having to track
a time-varying Hill frame on an elliptic orbit.  This illustrates that all the orbital motion is propoerly
feed-forward compensated.

::

    show_plots = True, simCase = 1

.. image:: /_images/Scenarios/scenarioAttitudeSteeringSigmaBR1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeSteeringomegaBR1.svg
   :align: center


::

    show_plots = True, simCase = 2

This setup investigates the small depature motion stability about the Hill frame.  Here only small initial
attitude and rate errors are introduced.  However, the outer loop feedback gain :math:`K_1` is increased such that
it violates the sub-servo loop separation principle.

.. image:: /_images/Scenarios/scenarioAttitudeSteeringSigmaBR2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeSteeringomegaBR2.svg
   :align: center

Here the local motion is now unstable, as predicted in this
`conference paper <http://hanspeterschaub.info/Papers/SchaubIAC2017.pdf>`__.


::

    show_plots = True, simCase = 3

This setup also investigates the small departure motion stability about the Hill frame.  However, in this case
the feedword term :math:`\omega'_{\cal B^\ast/R}` is omitted, which is predicted to yield locally stabilizing control
similar in performance to a standard proportional-derivative or PD feedback control.

.. image:: /_images/Scenarios/scenarioAttitudeSteeringSigmaBR3.svg
   :align: center


"""


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), RWs, simpleNav() and
#           MRP_Steering() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
#           while using the RWs to do the attitude control actuation.
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 7, 2017
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
# import message declarations
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import hillPoint
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpSteering
from Basilisk.fswAlgorithms import rateServoFullNonlinear
from Basilisk.fswAlgorithms import rwMotorTorque
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics as rb
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion as om
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def plot_attitude_error(timeData, dataSigmaBR):
    """Plot the attitude error."""
    plt.figure(1)
    for idx in range(3):
        plt.semilogy(timeData, np.abs(dataSigmaBR[:, idx]),
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$|\sigma_' + str(idx) + '|$')
    plt.legend(loc='upper right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')

def plot_rw_cmd_torque(timeData, dataUsReq, numRW):
    """plot the commanded RW torque."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

def plot_rw_motor_torque(timeData, dataRW, numRW):
    """Plot the actual RW motor torque."""
    plt.figure(2)
    for idx in range(3):
        plt.semilogy(timeData, np.abs(dataRW[idx]),
                     color=unitTestSupport.getLineColor(idx, numRW),
                     label='$|u_{s,' + str(idx) + '}|$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

def plot_rate_error(timeData, dataOmegaBR, dataOmegaBRAst):
    """Plot the body angular velocity tracking errors"""
    plt.figure(3)
    for idx in range(3):
        plt.semilogy(timeData, np.abs(dataOmegaBR[:, idx]) / macros.D2R,
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$|\omega_{BR,' + str(idx) + '}|$')
    for idx in range(3):
        plt.semilogy(timeData, np.abs(dataOmegaBRAst[:, idx]) / macros.D2R,
                     '--',
                     color=unitTestSupport.getLineColor(idx, 3)
                     )
    plt.legend(loc='upper right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (deg/s) ')


def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    """Plot the RW speeds."""
    plt.figure(4)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx) + '}$')
    plt.legend(loc='upper right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')




def run(show_plots, simCase):
    r"""
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        simCase (int):

            =======  ==============================================================
            simCase  Definition
            =======  ==============================================================
            0        Detumble with balanced gains (for inner- and outer-loop
                     separation principle), including integral feedback
            1        Detumble with balanced gains (for inner- and outer-loop
                     separation principle), without integral feedback
            2        Small detumble with strong steering gain violating
                     separation principle, with :math:`\omega'_{\cal B^{\ast}/R}`
            3        Small detumble with strong steering gain violating separation
                     principle, without :math:`\omega'_{\cal B^{\ast}/R}`
            =======  ==============================================================

            The first case has a scenario that should exponentially converge to zero,
            while the 2nd case will only provide a bounded
            (or Lagrange stable) response.  The latter two scenarios illustrate the
            performance if the outer loop feedback gain
            is too strong, violating the sub-servo separation principle, and how
            removing a particular term in case 3 can still
            lead to a locally stable response.

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [500., 0., 0.,
         0., 300., 0.,
         0., 0., 200.]
    scObject.hub.mHub = 750.0                   # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # add RW devices
    rwFactory = simIncludeRW.rwFactory()

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    initOmega = [100.0, 200.0, 300.0]
    RW1 = rwFactory.create('Honeywell_HR16'
                           , [1, 0, 0]
                           , maxMomentum=50.
                           , Omega=initOmega[0]  # RPM
                           )
    RW2 = rwFactory.create('Honeywell_HR16'
                           , [0, 1, 0]
                           , maxMomentum=50.
                           , Omega=initOmega[1]  # RPM
                           )
    RW3 = rwFactory.create('Honeywell_HR16'
                           , [0, 0, 1]
                           , maxMomentum=50.
                           , Omega=initOmega[2]  # RPM
                           )

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)
    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, 1)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    if simCase < 2:
        extFTObject = extForceTorque.ExtForceTorque()
        extFTObject.ModelTag = "externalDisturbance"
        extFTObject.extTorquePntB_B = [[0.01], [-0.01], [0.005]]
        scObject.addDynamicEffector(extFTObject)
        scSim.AddModelToTask(simTaskName, extFTObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup guidance module
    attGuidance = hillPoint.hillPoint()
    attGuidance.ModelTag = "hillPoint"
    scSim.AddModelToTask(simTaskName, attGuidance)

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # setup the MRP steering control module
    mrpControl = mrpSteering.mrpSteering()
    mrpControl.ModelTag = "MRP_Steering"

    scSim.AddModelToTask(simTaskName, mrpControl)

    if simCase < 2:
        mrpControl.K1 = 0.05
        mrpControl.ignoreOuterLoopFeedforward = False
    else:
        mrpControl.K1 = 2.2
        if simCase == 2:
            mrpControl.ignoreOuterLoopFeedforward = False
        else:
            mrpControl.ignoreOuterLoopFeedforward = True
    mrpControl.K3 = 0.75
    mrpControl.omega_max = 1. * macros.D2R

    # setup Rate servo module
    servo = rateServoFullNonlinear.rateServoFullNonlinear()
    servo.ModelTag = "rate_servo"

    if simCase == 1:
        servo.Ki = -1
    else:
        servo.Ki = 5.
    servo.P = 150.0
    servo.integralLimit = 2. / servo.Ki * 0.1
    servo.knownTorquePntB_B = [0., 0., 0.]

    scSim.AddModelToTask(simTaskName, servo)

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)

    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    ]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # FSW RW configuration message
    # use the same RW states in the FSW algorithm as in the simulation
    fswSetupRW.clearSetup()
    for key, rw in rwFactory.rwList.items():
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    fswRwParamMsg = fswSetupRW.writeConfigMessage()

    # setup message connections
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attGuidance.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(attGuidance.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    servo.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    servo.vehConfigInMsg.subscribeTo(vcMsg)
    servo.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    servo.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    servo.rateSteeringInMsg.subscribeTo(mrpControl.rateCmdOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(servo.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 200
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    rwStateLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    mrpLog = mrpControl.rateCmdOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, rwMotorLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, snTransLog)
    scSim.AddModelToTask(simTaskName, rwStateLog)
    scSim.AddModelToTask(simTaskName, mrpLog)
    rwLogs = []
    for item in range(numRW):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, rwLogs[item])

    #
    #   set initial Spacecraft States
    #
    oe = om.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = om.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    if simCase < 2:
        scObject.hub.sigma_BNInit = [[0.5], [0.6], [-0.3]]  # sigma_CN_B
        scObject.hub.omega_BN_BInit = [[0.01], [-0.01], [-0.01]]  # rad/s - omega_CN_B
    else:
        HN = rb.euler3132C([oe.Omega, oe.i, oe.omega + oe.f])
        sBR = [0.001, 0.002, -0.003]
        BN = rb.MRP2C([0.001, 0.002, -0.003])
        BH = BN * HN
        sBN = rb.C2MRP(BH)
        scObject.hub.sigma_BNInit = [[sBN[0]], [sBN[1]], [sBN[2]]]  # sigma_CN_B
        n = np.sqrt(mu / (oe.a * oe.a * oe.a))
        scObject.hub.omega_BN_BInit = [[n * HN[2, 0]], [n * HN[2, 1]], [n * HN[2, 2]]]  # rad/s - omega_CN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        , rwEffectorList=rwStateEffector
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
    dataUsReq = rwMotorLog.motorTorque[:, :numRW]
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    dataPos = snTransLog.r_BN_N
    dataOmegaRW = rwStateLog.wheelSpeeds[:, :numRW]
    dataOmegaBRAst = mrpLog.omega_BastR_B
    dataRW = []
    for i in range(numRW):
        dataRW.append(rwLogs[i].u_current)
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeData = attErrorLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_attitude_error(timeData, dataSigmaBR)
    figureList = {}
    pltName = fileName + "SigmaBR" + str(int(simCase))
    figureList[pltName] = plt.figure(1)

    plot_rw_motor_torque(timeData, dataRW, numRW)
    pltName = fileName + "rwUs" + str(int(simCase))
    figureList[pltName] = plt.figure(2)

    plot_rate_error(timeData, dataOmegaBR, dataOmegaBRAst)
    pltName = fileName + "omegaBR" + str(int(simCase))
    figureList[pltName] = plt.figure(3)

    plot_rw_speeds(timeData, dataOmegaRW, numRW)
    pltName = fileName + "Omega" + str(int(simCase))
    figureList[pltName] = plt.figure(4)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        0  # simCase
    )
