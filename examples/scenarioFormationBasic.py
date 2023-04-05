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

Demonstrates a basic method to simulate 3 satellites with 6-DOF motion and how to visualize the simulation
data in :ref:`Vizard <vizard>`.  One satellite is a 3-axis attitude controlled
satellite, while the second satellite is a tumbling space debris object.  The controlled satellite simulation components
are taken from :ref:`scenarioAttitudeFeedbackRW`. The purpose of this script is to show an explicit method to
setup multiple satellites, and also show how to store the Basilisk simulation data to be able to visualize
both satellite's motions within the :ref:`Vizard <vizard>` application.

Note, this scenario also illustrates how to ensure that the differential equations of motion of
the servicer and debris object are integrated at the same time.  This is not required in this scenario
as there are no direct satellite-to-satellite dynamic interactions.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioFormationBasic.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the servicer spacecraft and associated the Flight Software (FSW) algorithm
modules, as well as the first debris object that has 2 free-spinning RWs, and another debris object that
is an inert rigid body.

.. image:: /_images/static/test_scenarioFormationBasic.svg
   :align: center

When the simulation completes several plots are shown for the servicer MRP attitude history, the rate
tracking errors, the RW motor torque components, as well as the RW wheel speeds.

The simulation setups the spacecraft with 3 RW devices similar to :ref:`scenarioAttitudeFeedbackRW`.  One difference
is that here :ref:`hillPoint` is used to align the spacecraft with the Hill frame.  The two debris objects are
in a 2:1 centered ellipse and a lead-follower configuration with the servicer respectively.  The servicer camera
has a camera instrument attached that is pointing in the 3rd body axis direction.
The servicer has a light attached to illuminate the debris object.

By default, every :ref:`spacecraft` module instance will integrate its differential equations, and that of
all the associated state and dynamics effectors, during the module ``Update()`` method.  Thus, the
second spacecraft ODEs are integrated forward one time step after the first spacecraft, and so on.
If you require
both sets of spacecraft differential equations to be integrated at the same time, then the integration
of the second spacecraft can be synchronized with the integration of the first spacecraft using::

     scObject.syncDynamicsIntegration(scObject2)

This is illustrated in this example script where the debris satellite integration is sync'd with that
of the servicer satellite.  However, in this scenario this is not required as the ODEs of each spacecraft
are independent of each other.  If an effector is used that is connected to both spacecraft, then this
step will allow the effector force and torque evaluations to be properly applied to all sync'd objects.

This simulation scripts illustrates how to use the :ref:`vizSupport` methods to record the simulation data such
that it can be viewed in the Vizard visualization.


Illustration of Simulation Results
----------------------------------

::

    show_plots = True

Note that in the RW motor torque plot both the required control torque :math:`\hat u_B` and the true
motor torque :math:`u_B` are shown.  This illustrates that with this maneuver the RW devices are being
saturated, and the attitude still eventually stabilizes.

.. image:: /_images/Scenarios/scenarioFormationBasic1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFormationBasic2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFormationBasic3.svg
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
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    rwMotorTorque, hillPoint)
from Basilisk.simulation import reactionWheelStateEffector, simpleNav, spacecraft, svIntegrators
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)

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


# Plotting functions
def plot_attitude_error(timeData, dataSigmaBR):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')


def plot_rw_cmd_torque(timeData, dataUsReq, numRW):
    """Plot the RW command torques."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')


def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW):
    """Plot the RW actual motor torques."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
        plt.plot(timeData, dataRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')


def plot_rate_error(timeData, dataOmegaBR):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s) ')


def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    """Plot the RW spin rates."""
    plt.figure(4)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')


def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useMsgNameDefaults (bool): Specify if default message naming is used for the additional space objects

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(40.)

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
    # this next step is not required, just a demonstration how we can ensure that
    # the Servicer and Debris differential equations are integrated simultaneously
    scObject.syncDynamicsIntegration(scObject2)

    # Likewise, the following step is not required, as the default integrator
    # is already RK4. However, this illustrates that you can change the integrator
    # of the primary after calling sync, but not of the secondary!
    integratorObject = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integratorObject)
    # scObject2.setIntegrator(integratorObject) # <- Will raise an error!

    # make another debris object */
    scObject3 = spacecraft.Spacecraft()
    scObject3.ModelTag = "DebrisSat"
    I3 = [600., 0., 0.,
          0., 650., 0.,
          0., 0, 450.]
    scObject3.hub.mHub = 350.0  # kg
    scObject3.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I3)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, scObject2)
    scSim.AddModelToTask(simTaskName, scObject3)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObject2.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObject3.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = messaging.BalancedWheels

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    RW1 = rwFactory.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=100.  # RPM
                           , RWModel=varRWModel
                           )
    RW2 = rwFactory.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=200.  # RPM
                           , RWModel=varRWModel
                           )
    RW3 = rwFactory.create('Honeywell_HR16', [0, 0, 1], maxMomentum=50., Omega=300.  # RPM
                           , rWB_B=[0.5, 0.5, 0.5]  # meters
                           , RWModel=varRWModel
                           )

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    # make sure the input and output names are unique to this spacecraft
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory.addToSpacecraft("chiefRW", rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(simTaskName, rwStateEffector, None, 4)

    # add free-spinning RWs to the debris object
    rwFactory2 = simIncludeRW.rwFactory()
    rwFactory2.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=1000.0)
    rwFactory2.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=-1000.0)
    numRW2 = rwFactory2.getNumOfDevices()
    rwStateEffector2 = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory2.addToSpacecraft("debrisRW", rwStateEffector2, scObject2)
    scSim.AddModelToTask(simTaskName, rwStateEffector2, None, 5)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, sNavObject)

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
    attErrorWrap.ModelTag = "attErrorInertial3D"
    attErrorConfig.sigma_R0R = [0.414214, 0.0, 0.0]     # point the 3rd body axis in the along-track direction
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.attRefInMsg.subscribeTo(attGuidanceConfig.attRefOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # create FSW RW parameter msg
    fswRwMsg = rwFactory.getConfigMessage()

    # setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControlConfig.rwParamsInMsg.subscribeTo(fswRwMsg)
    mrpControlConfig.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
    rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
    rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueWrap, rwMotorTorqueConfig)
    # Initialize the test module msg names
    rwMotorTorqueConfig.vehControlInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)
    rwMotorTorqueConfig.rwParamsInMsg.subscribeTo(fswRwMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueConfig.rwMotorTorqueOutMsg)
    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    rwMotorTorqueConfig.controlAxes_B = controlAxes_B

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    rwCmdLog = rwMotorTorqueConfig.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrLog = attErrorConfig.attGuidOutMsg.recorder(samplingTime)
    sNavLog = sNavObject.transOutMsg.recorder(samplingTime)
    rwSpeedLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, rwCmdLog)
    scSim.AddModelToTask(simTaskName, attErrLog)
    scSim.AddModelToTask(simTaskName, sNavLog)
    scSim.AddModelToTask(simTaskName, rwSpeedLog)

    rwSc1Log = []
    for rw in rwStateEffector.rwOutMsgs:
        rwSc1Log.append(rw.recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, rwSc1Log[-1])
    rwSc2Log = []
    for rw in rwStateEffector2.rwOutMsgs:
        rwSc2Log.append(rw.recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, rwSc2Log[-1])

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
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    # setup 1st debris object states
    oe2 = copy.deepcopy(oe)
    oe2.e += 0.000001
    r2N, v2N = orbitalMotion.elem2rv(mu, oe2)
    scObject2.hub.r_CN_NInit = r2N  # m   - r_CN_N
    scObject2.hub.v_CN_NInit = v2N  # m/s - v_CN_N
    scObject2.hub.sigma_BNInit = [[0.3], [0.1], [0.2]]  # sigma_CN_B
    scObject2.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # setup 2nd debris object states
    oe3 = copy.deepcopy(oe)
    oe3.f += 40./oe3.a
    r3N, v3N = orbitalMotion.elem2rv(mu, oe3)
    scObject3.hub.r_CN_NInit = r3N  # m   - r_CN_N
    scObject3.hub.v_CN_NInit = v3N  # m/s - v_CN_N
    scObject3.hub.sigma_BNInit = [[0.0], [-0.1], [0.2]]  # sigma_CN_B
    scObject3.hub.omega_BN_BInit = [[0.01], [-0.03], [-0.03]]  # rad/s - omega_CN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    if vizFound:
        servicerLight = vizInterface.Light()
        servicerLight.label = "Main Light"
        servicerLight.position = [0.2, -1.0, 1.01]
        servicerLight.fieldOfView = 10.0 * macros.D2R
        servicerLight.normalVector = [0, 0, 1]
        servicerLight.range = 150.0
        servicerLight.markerDiameter = 0.1
        servicerLight.color = vizInterface.IntVector(vizSupport.toRGBA255("red"))

        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2, scObject3]
                                                  , rwEffectorList=[rwStateEffector, rwStateEffector2, None]
                                                  , lightList=[[servicerLight], None, None]
                                                  # , saveFile=fileName,
                                                  )
        # setup one-way instrument camera by having frameRate be 0
        vizSupport.createCameraConfigMsg(viz, parentName=scObject.ModelTag,
                                         cameraID=1, fieldOfView=40 * macros.D2R,
                                         resolution=[1024, 1024], renderRate=0.,
                                         cameraPos_B=[0., 0., 2.0], sigma_CB=[0., 0., 0.]
                                         )
        viz.settings.trueTrajectoryLinesOn = 1
        viz.settings.orbitLinesOn = 2

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
    dataUsReq = rwCmdLog.motorTorque[:, range(numRW)]
    dataSigmaBR = attErrLog.sigma_BR
    dataOmegaBR = attErrLog.omega_BR_B
    dataOmegaRW = rwSpeedLog.wheelSpeeds[:, range(numRW)]
    dataRW = []
    for i in range(numRW):
        dataRW.append(rwSc1Log[i].u_current)
    np.set_printoptions(precision=16)
    omegaRW2 = []
    for i in range(numRW2):
        omegaRW2.append(rwSc2Log[i].Omega)

    #
    #   plot the results
    #
    timeData = attErrLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_attitude_error(timeData, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_rate_error(timeData, dataOmegaBR)
    plot_rw_speeds(timeData, dataOmegaRW, numRW)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(4)

    plt.figure(5)
    for idx in range(numRW2):
        plt.plot(timeData, omegaRW2[idx]*60/(2*3.14159),
                 color=unitTestSupport.getLineColor(idx, numRW2),
                 label=r'$\Omega_{s,' + str(idx) + '}$')
    plt.xlabel('Time [min]')
    plt.ylabel('RW2 Omega (rpm)')

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
