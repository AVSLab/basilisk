#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario simulates two space objects, and active servicer approach another defund debris satellite.
The servicer spacecraft begin on a drifting approach trajectory after which it perfoms a drifting
safety-ellipse to circumnavigate the debris, and finally parks in a 2:1 bounded ellipse.

The script illustrates how the simulation can be run for fixed periods of time after which
some flight software modules change their input subscript to switch between the three possible
attitude pointing modes 1) Hill pointing, 2) spacecraft point at the debris object and
3) sun pointing of the solar panels.

To do relative motion maneuvers, methods are used to change the instantaneous relative
velocity.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioRendezVous.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the servicer spacecraft and associated the Flight Software (FSW) algorithm
modules, as well as the debris object.

.. image:: /_images/static/test_scenarioRendezVous.svg
   :align: center

When the simulation completes several plots are shown for the servicer MRP attitude history, the rate
tracking errors, the RW motor torque components, as well as the RW wheel speeds.

The simulation starts with the spacecraft entering Hill frame pointing in a along-track drift mode for
1/8th of an orbit.  Next, the logo side of the spacecraft is aimed at the debris object for 2 orbits.
Here a maneuver is performed to create a drifting circumnavigation.  Finally, the spacecraft
points the solar panels at the sun for 1/2 of an orbit.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

Note that in the RW motor torque plot both the required control torque :math:`\hat u_B` and the true
motor torque :math:`u_B` are shown.  This illustrates that with this maneuver the RW devices are being
saturated, and the attitude still eventually stabilizes.

.. image:: /_images/Scenarios/scenarioRendezVous1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRendezVous2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRendezVous3.svg
   :align: center


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Simulation of a servicer approaching a LEO spacecraft target
#
# Author:   Hanspeter Schaub
# Creation Date:  July 3, 2022
#

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import locationPointing
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    rwMotorTorque, hillPoint)
from Basilisk.simulation import reactionWheelStateEffector, simpleNav, spacecraft, ephemerisConverter
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)

try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass

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

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.SetProgressBar(True)

    # set the simulation time variable used later on
    simulationTime = 0

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize servicer spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "servicer"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # create the debris object states
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "debris"
    I2 = [600., 0., 0.,
          0., 650., 0.,
          0., 0, 450.]
    scObject2.hub.mHub = 350.0  # kg
    scObject2.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I2)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, 1)
    scSim.AddModelToTask(simTaskName, scObject2, 2)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    # earth = gravFactory.createEarth()
    # earth.isCentralBody = True  # ensure this is the central gravitational body
    gravBodies = gravFactory.createBodies('sun', 'earth')
    gravBodies['earth'].isCentralBody = True
    mu = gravBodies['earth'].mu
    sunIdx = 0
    earthIdx = 1

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)
    gravFactory.addBodiesTo(scObject2)

    # setup SPICE interface for celestial objects
    timeInitString = "2022 MAY 1 00:28:30.0"
    spiceObject = gravFactory.createSpiceInterface(time=timeInitString, epochInMsg=True)
    spiceObject.zeroBase = 'Earth'
    scSim.AddModelToTask(simTaskName, spiceObject)

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
    scSim.AddModelToTask(simTaskName, rwStateEffector, 4)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, sNavObject)

    sNavObject2 = simpleNav.SimpleNav()
    sNavObject2.ModelTag = "SimpleNavigation2"
    sNavObject2.scStateInMsg.subscribeTo(scObject2.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, sNavObject2)

    # Create an ephemeris converter to convert messages of type
    # 'SpicePlanetStateMsgPayload' to 'EphemerisMsgPayload'
    ephemObject = ephemerisConverter.EphemerisConverter()
    ephemObject.ModelTag = 'EphemData'
    ephemObject.addSpiceInputMsg(spiceObject.planetStateOutMsgs[sunIdx])
    ephemObject.addSpiceInputMsg(spiceObject.planetStateOutMsgs[earthIdx])
    scSim.AddModelToTask(simTaskName, ephemObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup hillPoint guidance module
    hillPointing = hillPoint.hillPoint()
    hillPointing.ModelTag = "hillPoint"
    hillPointing.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    hillPointing.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[earthIdx])
    scSim.AddModelToTask(simTaskName, hillPointing)

    # setup spacecraftPointing guidance module
    scPointing = locationPointing.locationPointing()
    scPointing.ModelTag = "scPointing"
    scPointing.pHat_B = [1, 0, 0]
    scPointing.useBoresightRateDamping = 1
    scPointing.scTargetInMsg.subscribeTo(sNavObject2.transOutMsg)
    scPointing.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    scPointing.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    scSim.AddModelToTask(simTaskName, scPointing)

    # setup sunPointing guidance module
    sunPointing = locationPointing.locationPointing()
    sunPointing.ModelTag = "scPointing"
    sunPointing.pHat_B = [0, 0, 1]
    sunPointing.useBoresightRateDamping = 1
    sunPointing.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[sunIdx])
    sunPointing.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    sunPointing.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    scSim.AddModelToTask(simTaskName, sunPointing)

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)
    attError.attRefInMsg.subscribeTo(scPointing.attRefOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # create FSW RW parameter msg
    fswRwMsg = rwFactory.getConfigMessage()

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    mrpControl.K = 3.5
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)
    # Initialize the test module msg names
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    rwCmdLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrLog = attError.attGuidOutMsg.recorder(samplingTime)
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

    #
    #   set initial Spacecraft States
    #
    # setup debris (chief) object states
    oe2 = orbitalMotion.ClassicElements()
    oe2.a = 10000000.0  # meters
    oe2.e = 0.0
    oe2.i = 33.3 * macros.D2R
    oe2.Omega = 48.2 * macros.D2R
    oe2.omega = 90.0 * macros.D2R
    oe2.f = 0.0 * macros.D2R
    r2N, v2N = orbitalMotion.elem2rv(mu, oe2)
    scObject2.hub.r_CN_NInit = r2N  # m   - r_CN_N
    scObject2.hub.v_CN_NInit = v2N  # m/s - v_CN_N
    scObject2.hub.sigma_BNInit = [[0.3], [0.1], [0.2]]  # sigma_CN_B
    scObject2.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B
    n = np.sqrt(mu/oe2.a/oe2.a/oe2.a)
    orbitPeriod = 2*np.pi / n   # in seconds

    # setup the servicer (deputy) orbit using Hill frame coordinates
    # oe = copy.deepcopy(oe2)
    # oe.e = 0.000001
    # rN, vN = orbitalMotion.elem2rv(mu, oe)
    rho_H = [-10.0, -40.0, 0.0]
    rho_Prime_H = [0, -1.5*n*rho_H[0], 0]
    rN, vN = orbitalMotion.hill2rv(r2N, v2N, rho_H, rho_Prime_H)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    if vizSupport.vizFound:
        servicerLight = vizInterface.Light()
        servicerLight.label = "Main Light"
        servicerLight.position = [1.0, 0.0, 0.00]
        servicerLight.fieldOfView = 10.0 * macros.D2R
        servicerLight.normalVector = [1, 0, 0]
        servicerLight.range = 150.0
        servicerLight.markerDiameter = 0.1
        servicerLight.color = vizInterface.IntVector(vizSupport.toRGBA255("white"))

        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                                  , rwEffectorList=[rwStateEffector, None]
                                                  , lightList=[[servicerLight], None]
                                                  # , saveFile=fileName
                                                  )

        viz.settings.trueTrajectoryLinesOn = -1
        viz.settings.orbitLinesOn = 2
        viz.settings.mainCameraTarget = "debris"

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    debrisPos = scObject2.dynManager.getStateObject(scObject2.hub.nameOfHubPosition)
    debrisVel = scObject2.dynManager.getStateObject(scObject2.hub.nameOfHubVelocity)
    servicerPos = scObject.dynManager.getStateObject(scObject.hub.nameOfHubPosition)
    servicerVel = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)

    # Set up flight modes
    def fswTargetPointing():
        attError.attRefInMsg.subscribeTo(scPointing.attRefOutMsg)
        attError.sigma_R0R = [0, 0, 0]

    def fswSunPointing():
        attError.attRefInMsg.subscribeTo(sunPointing.attRefOutMsg)
        attError.sigma_R0R = [0, 0, 0]

    def fswHillPointing():
        attError.attRefInMsg.subscribeTo(hillPointing.attRefOutMsg)
        attError.sigma_R0R = [0.0, 0.0, -0.414214]

    def runSim(simTime):
        nonlocal simulationTime
        simulationTime += macros.sec2nano(simTime)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

    def relOrbDrift():
        rd = unitTestSupport.EigenVector3d2np(servicerPos.getState())
        vd = unitTestSupport.EigenVector3d2np(servicerVel.getState())
        rc = unitTestSupport.EigenVector3d2np(debrisPos.getState())
        vc = unitTestSupport.EigenVector3d2np(debrisVel.getState())
        rho_H, rho_Prime_H = orbitalMotion.rv2hill(rc, vc, rd, vd)
        xOff = rho_H[0]
        rho_Prime_H[0] = 0.0
        rho_Prime_H[1] = -1.5*n*xOff
        unusedPos, vd = orbitalMotion.hill2rv(rc, vc, rho_H, rho_Prime_H)
        servicerVel.setState(vd)

    def relativeEllipse(A0, xOff, B0=-1):
        rd = unitTestSupport.EigenVector3d2np(servicerPos.getState())
        vd = unitTestSupport.EigenVector3d2np(servicerVel.getState())
        rc = unitTestSupport.EigenVector3d2np(debrisPos.getState())
        vc = unitTestSupport.EigenVector3d2np(debrisVel.getState())
        rho_H, rho_Prime_H = orbitalMotion.rv2hill(rc, vc, rd, vd)
        alpha = np.arccos((rho_H[0] - xOff)/A0)
        if B0 > 0.01:
            beta = np.arccos(rho_H[2]/B0)
        else:
            if B0 == 0.0:
                beta = 0.0
        # yOff = rho_H[1] + 2*A0*np.sin(alpha)
        rho_Prime_H[0] = -A0*n*np.sin(alpha)
        rho_Prime_H[1] = -2*A0*n*np.cos(alpha) - 1.5*n*xOff
        if B0 >= 0.0:
            rho_Prime_H[2] = -B0*n*np.sin(beta)
        unusedPos, vd = orbitalMotion.hill2rv(rc, vc, rho_H, rho_Prime_H)
        servicerVel.setState(vd)

    #
    #   configure a simulation stop time and execute the simulation run
    #
    fswHillPointing()
    relOrbDrift()
    runSim(orbitPeriod / 8)

    fswTargetPointing()
    relativeEllipse(10.0, -1.0, 10)
    runSim(orbitPeriod * 2)

    fswSunPointing()
    relativeEllipse(10.0, 0.0, 0.0)
    runSim(orbitPeriod / 2)

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

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    gravFactory.unloadSpiceKernels()

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True      # show_plots
    )
