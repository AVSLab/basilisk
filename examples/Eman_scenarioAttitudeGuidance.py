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

    <iframe width="560" height="315" src="https://www.youtube.com/embed/-tG4OZzGJfQ" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

Discusses how to use guidance modules to align the spacecraft frame to the orbit or Hill frame.
This script sets up a 6-DOF spacecraft which is orbiting the Earth.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttitudeGuidance.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenarioAttitudeGuidance.svg
   :align: center

When the simulation completes 4 plots are shown for the MRP attitude history, the rate
tracking errors, the control torque vector, as well as the projection of the body-frame B
axes :math:`\hat b_1`, :math:`\hat b_2` and :math:`\hat b_3` onto the respect
Hill or Orbit frame axes :math:`\hat\imath_r`,
:math:`\hat\imath_{\theta}` and :math:`\hat\imath_h`.  This latter plot illustrates how the body
is being aligned with respect to this Hill frame.

The basic simulation setup is the same as the one used in
:ref:`scenarioAttitudeFeedback`.
The dynamics simulation is setup using a :ref:`Spacecraft` module to which a gravity
effector is attached.  Note that both the rotational and translational degrees of
freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.  For more
information on how to setup orbit, see :ref:`scenarioBasicOrbit`.

In contrast to the simple inertial pointing guidance example :ref:`scenarioAttitudeFeedback`,
this module also requires the
spacecraft's position and velocity information.  The planet ephemeris message relative to which the Hill pointing
is being achieved by connecting the ``celBodyInMsg`` message.
This is useful, for example, if orbiting the sun, and wanting to point the spacecraft back at the
Earth which is also orbiting the sun.
Note that if the celestial body ephemeris input message is not connected then
a zero message is created which corresponds to the planet having a zero position and velocity vector.
If non-zero ephemeris information is required then the input name must point
to a message of type :ref:`EphemerisMsgPayload`.
In this scenario, however, the spacecraft is to point at the Earth while already orbiting the Earth and the input
message name is set to a dummy message.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useAltBodyFrame = False

The default scenario shown has the ``useAltBodyFrame`` flag turned off.  This means that we seek
to align the body frame *B* with the Hill reference frame :math:`\cal R`.    The
resulting attitude and control torque histories are shown below.  Note that the projections
of the body frame axes onto the Hill frame axes all converge to +1, indicating that :math:`\cal B` becomes
asymptotically aligned with :math:`\cal R` as desired.

.. image:: /_images/Scenarios/scenarioAttitudeGuidance10.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeGuidance20.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeGuidance40.svg
   :align: center

::

    show_plots = True, useAltBodyFrame = True

Here the control should not align the principal body frame *B* with *R*, but rather an alternate,
corrected body frame :math:`{\cal B}_c`.  For example, consider the Earth observing sensors
to be mounted pointing in the
positive :math:`\hat b_1` direction. In earlier scenario this sensor platform is actually pointing away from
the Earth.  Thus, we define the corrected body frame orientation as a 180 deg rotation about
:math:`\hat b_2`.  This flips the orientation of the final first and third body axis.  This is achieved
through::

  attErrorConfig.sigma_R0R = [0,1,0]

The DCM :math:`[R_0R]` is the same as the body to corrected body DCM :math:`[B_cB]`.
The resulting attitude and control torque histories are shown below.  Note that the projections
of the 2nd body frame axis onto the 2nd Hill frame axes converges to +1, while the other
projections converge to -1.  This indicates that the desired asymptotic Earth observing attitude
is achieved.

.. image:: /_images/Scenarios/scenarioAttitudeGuidance11.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeGuidance21.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeGuidance41.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav(),
#           mrpFeedback() with attitude navigation modules.  Illustrates how
#           attitude guidance behavior can be changed in a very modular manner.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 2, 2016
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
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import sunSafePoint
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import rwMotorTorque
from Basilisk.fswAlgorithms import rwMotorVoltage
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.simulation import groundLocation
# import simulation related support
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import motorVoltageInterface
# import general simulation support files
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import astroFunctions
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

def plot_orientation(timeLineSet, dataPos, dataVel, dataSigmaBN):
    """Plot the spacecraft orientation."""
    vectorPosData = dataPos
    vectorVelData = dataVel
    vectorMRPData = dataSigmaBN
    data = np.empty([len(vectorPosData), 3])
    for idx in range(0, len(vectorPosData)):
        ir = vectorPosData[idx] / np.linalg.norm(vectorPosData[idx])
        hv = np.cross(vectorPosData[idx], vectorVelData[idx])
        ih = hv / np.linalg.norm(hv)
        itheta = np.cross(ih, ir)
        dcmBN = RigidBodyKinematics.MRP2C(vectorMRPData[idx])
        data[idx] = [np.dot(ir, dcmBN[0]), np.dot(itheta, dcmBN[1]), np.dot(ih, dcmBN[2])]
    plt.figure(4)
    labelStrings = (r'$\hat\imath_r\cdot \hat b_1$'
                    , r'${\hat\imath}_{\theta}\cdot \hat b_2$'
                    , r'$\hat\imath_h\cdot \hat b_3$')
    for idx in range(3):
        plt.plot(timeLineSet, data[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=labelStrings[idx])
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Orientation Illustration')


def run(show_plots, useJitterSimple, useAltBodyFrame):
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
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))


    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    
    # define the simulation inertia -MBZ
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
    #edited by Ahmed
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 512044.6  # meters -MBZ
    oe.e = 0.00074680
    oe.i = 97.4 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.02], [0.02], [0.02]]  # rad/s - omega_BN_B


 # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = messaging.BalancedWheels
    if useJitterSimple:
        varRWModel = messaging.JitterSimple

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    RW1 = rwFactory.create('Rockwell_RSI215', [1, 0, 0], maxMomentum=15.
                           , RWModel=varRWModel
                           )
    RW2 = rwFactory.create('Rockwell_RSI215', [0, 1, 0], maxMomentum=15.
                           , RWModel=varRWModel
                           )
    RW3 = rwFactory.create('Rockwell_RSI215', [0, 0, 1], maxMomentum=15.
                           , rWB_B=[0.5, 0.5, 0.5]  # meters
                           , RWModel=varRWModel
                           )
    RW4 = rwFactory.create('Rockwell_RSI215', [0, 0, 1], maxMomentum=15.
                           , rWB_B=[0.5, 0.5, 0.5]  # meters
                           , RWModel=varRWModel
                           )
    # In this simulation the RW objects RW1, RW2 or RW3 are not modified further.  However, you can over-ride
    # any values generate in the `.create()` process using for example RW1.Omega_max = 100. to change the
    # maximum wheel speed.

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process.  This is required for the UpdateState() method
    # to be called which logs the RW states
    scSim.AddModelToTask(simTaskName, rwStateEffector, None, 2)


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
    groundStation.ModelTag = "MBRSCGroundStation"
    groundStation.planetRadius = astroFunctions.E_radius*1e3  # meters
    groundStation.specifyLocation(np.radians(25.225907), np.radians(55.465201), 11)
    groundStation.minimumElevation = np.radians(10.)
    groundStation.maximumRange = 1e9  # meters
    groundStation.addSpacecraftToModel(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, groundStation)

    #
    #   setup the simulation tasks/objects
    #
    # setup MBRSC pointing guidance module
    locPointConfig = locationPointing.locationPointingConfig()
    locPointWrap = scSim.setModelDataWrap(locPointConfig)
    locPointWrap.ModelTag = "locPoint"
    scSim.AddModelToTask(simTaskName, locPointWrap, locPointConfig)
    locPointConfig.pHat_B = [0, 0, 1]
    locPointConfig.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    locPointConfig.useBoresightRateDamping = 1
    locPointConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    locPointConfig.locationInMsg.subscribeTo(groundStation.currentGroundStateOutMsg)

    #
    #   setup the FSW algorithm tasks
    #
    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation


    # setup sunSafePoint guidance module
    attGuidanceConfig = sunSafePoint.sunSafePointConfig()
    attGuidanceWrap = scSim.setModelDataWrap(attGuidanceConfig)
    attGuidanceWrap.ModelTag = "sunSafePoint"
    # attGuidanceConfig.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    # if you want to connect attGuidanceConfig.celBodyInMsg, then you need a planet ephemeris message of
    # type EphemerisMsgPayload.  In this simulation the input message is not connected to create an empty planet
    # ephemeris message which puts the earth at (0,0,0) origin with zero speed.
    CelBodyData = messaging.NavAttMsgPayload() # make zero'd planet ephemeris message
    sunDirectionInMsg = messaging.NavAttMsg().write(CelBodyData)
    attGuidanceConfig.sunDirectionInMsg.subscribeTo(sunDirectionInMsg)
    scSim.AddModelToTask(simTaskName, attGuidanceWrap, attGuidanceConfig)
    

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    if useAltBodyFrame:
        attErrorConfig.sigma_R0R = [0, 1, 0]
    #attErrorConfig.attRefInMsg.subscribeTo(attGuidanceConfig.attRefOutMsg)
    attErrorConfig.attGuidOutMsg.subscribeTo(attGuidanceConfig.attGuidanceOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(attGuidanceConfig.sunDirectionInMsg)
    attErrorConfig.attNavInMsg.subscribeTo(attGuidanceConfig.imuInMsg)

    # setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1.0  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1
    
    # connect torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    # # add module that maps the Lr control torque into the RW motor torques - Ahmed Added
    # rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
    # rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
    # rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
    # scSim.AddModelToTask(simTaskName, rwMotorTorqueWrap, rwMotorTorqueConfig)

    # # Make the RW control all three body axes
    # controlAxes_B = [
    #     1, 0, 0, 0, 1, 0, 0, 0, 1
    # ]
    # rwMotorTorqueConfig.controlAxes_B = controlAxes_B

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    mrpLog = mrpControlConfig.cmdTorqueOutMsg.recorder(samplingTime)
    attErrLog = attErrorConfig.attGuidOutMsg.recorder(samplingTime)
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

    # fswSetupRW.clearSetup()
    # for key, rw in rwFactory.rwList.items():
    #     fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    # fswRwParamMsg1 = fswSetupRW.writeConfigMessage()

    # # Second case: If the exact same RW configuration states are to be used by the simulation and fsw, then the
    # # following helper function is convenient to extract the fsw RW configuration message from the
    # # rwFactory setup earlier.
    # fswRwParamMsg2 = rwFactory.getConfigMessage()
    # fswRwParamMsg = fswRwParamMsg2

    # vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
    #                                           , saveFile=fileName
    #                                           )
    # vizSupport.addLocation(viz, stationName="MBRSC Station"
    #                            , parentBodyName=earth.displayName
    #                            , r_GP_P=groundStation.r_LP_P_Init
    #                            , fieldOfView=np.radians(160.)
    #                            , color='pink'
    #                            , range=2000.0*1000  # meters
    #                            )
    # viz.settings.spacecraftSizeMultiplier = 1.5
    # viz.settings.showLocationCommLines = 1
    # viz.settings.showLocationCones = 1
    # viz.settings.showLocationLabels = 1

    # link messages
    # sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    # attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    # attErrorConfig.attRefInMsg.subscribeTo(inertial3DConfig.attRefOutMsg)
    # mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    # mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    # mrpControlConfig.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    # mrpControlConfig.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    # rwMotorTorqueConfig.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    # rwMotorTorqueConfig.vehControlInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)
    
    # rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueConfig.rwMotorTorqueOutMsg)


    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()


    # execute BSK for a single step:
    dynProcess.disableAllTasks()
    print("all tasks disabled")
    scSim.TotalSim.SingleStepProcesses()
    print("BSK executed a single simulation step")

    scSim.enableTask("locPoint")
    scSim.TotalSim.SingleStepProcesses()
    print("BSK executed a single simulation step - locPoint")

    scSim.disableTask("locPoint")
    scSim.TotalSim.SingleStepProcesses()
    print("locPoint task diabled")    

    scSim.enableTask("sunSafePoint")
    scSim.TotalSim.SingleStepProcesses()
    print("BSK executed a single simulation step - SunSafe")

    scSim.disableTask("sunSafePoint")
    scSim.TotalSim.SingleStepProcesses()
    print("sunSafePoint task diabled")  

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

    plot_attitude_error(timeLineSet, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(1)

    plot_control_torque(timeLineSet, dataLr)
    pltName = fileName + "2" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(2)

    plot_rate_error(timeLineSet, dataOmegaBR)

    plot_orientation(timeLineSet, dataPos, dataVel, dataSigmaBN)
    pltName = fileName + "4" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(4)

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
        True,  # useJitterSample
        True # useAltBodyFrame
    )
