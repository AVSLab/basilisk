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

This scenario is an exact replica of :ref:`scenarioAttitudeFeedback2T_TH`. The only difference lies in the fact that
this scenario uses the :ref:`thrusterStateEffector` module instead of :ref:`thrusterDynamicEffector`. The performance
and results should be nearly identical to the original scenario, with the small difference that the thrusters do not
have an on-off behavior, but instead behave like a first-order filter. For more information on the scenario setup, see
:ref:`scenarioAttitudeFeedback2T_TH`.

To show that the :ref:`thrusterStateEffector` thruster module works with variable time step integrators, this scenario
uses an RKF78 integrator instead of the usual RK4.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useDVThrusters = False

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH10.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH20.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH30.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH40.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH50.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH60.svg
   :align: center


::

    show_plots = True, useDVThrusters = True

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH11.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH21.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH31.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH41.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH51.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T_stateEffTH61.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav(), thrusterDynamicEffector() and
#           mrpFeedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit, while using thrusters
#           to do the attitude control actuation.
# Author: João Vaz Carneiro
# Creation Date:  July 27, 2022
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
from Basilisk.fswAlgorithms import inertial3D
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import thrFiringSchmitt
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import thrusterStateEffector
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
def plot_attitude_error(timeDataFSW, dataSigmaBR):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeDataFSW, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + r'$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')


def plot_rate_error(timeDataFSW, dataOmegaBR):
    """Plot the body angular velocity tracking errors."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeDataFSW, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + r'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')


def plot_requested_torque(timeDataFSW, dataLr):
    """Plot the commanded attitude control torque."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeDataFSW, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Control Torque $L_r$ [Nm]')


def plot_thrForce(timeDataFSW, dataMap, numTh):
    """Plot the Thruster force values."""
    plt.figure(4)
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataMap[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label=r'$thrForce_{' + str(idx) + r'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Force requested [N]')


def plot_OnTimeRequest(timeDataFSW, dataSchm, numTh):
    """Plot the thruster on time requests."""
    plt.figure(5)
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataSchm[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label=r'$OnTimeRequest_{' + str(idx) + r'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('OnTimeRequest [sec]')


def plot_trueThrForce(timeDataFSW, dataMap, numTh):
    """Plot the Thruster force values."""
    plt.figure(6)
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataMap[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label=r'$thrForce_{' + str(idx) + r'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Force implemented[N]')


def run(show_plots, useDVThrusters):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useDVThrusters (bool): Use 6 DV thrusters instead of the default 8 ACS thrusters.

    """

    # Create simulation variable names
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(dynProcessName)
    fswProcess = scSim.CreateNewProcess(fswProcessName)

    # create the dynamics task and specify the integration update time
    simTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simTimeStep))
    fswTimeStep = macros.sec2nano(0.5)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

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
    scSim.AddModelToTask(dynTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(dynTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTaskName, sNavObject)

    # create arrays for thrusters' locations and directions
    if useDVThrusters:

        location = [
            [
                0,
                0.95,
                -1.1
            ],
            [
                0.8227241335952166,
                0.4750000000000003,
                -1.1
            ],
            [
                0.8227241335952168,
                -0.47499999999999976,
                -1.1
            ],
            [
                0,
                -0.95,
                -1.1
            ],
            [
                -0.8227241335952165,
                -0.4750000000000004,
                -1.1
            ],
            [
                -0.822724133595217,
                0.4749999999999993,
                -1.1
            ]
        ]

        direction = [[0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0],
                     [0.0, 0.0, 1.0]]
    else:

        location = [
            [
                3.874945160902288e-2,
                -1.206182747348013,
                0.85245
            ],
            [
                3.874945160902288e-2,
                -1.206182747348013,
                -0.85245
            ],
            [
                -3.8749451609022656e-2,
                -1.206182747348013,
                0.85245
            ],
            [
                -3.8749451609022656e-2,
                -1.206182747348013,
                -0.85245
            ],
            [
                -3.874945160902288e-2,
                1.206182747348013,
                0.85245
            ],
            [
                -3.874945160902288e-2,
                1.206182747348013,
                -0.85245
            ],
            [
                3.8749451609022656e-2,
                1.206182747348013,
                0.85245
            ],
            [
                3.8749451609022656e-2,
                1.206182747348013,
                -0.85245
            ]
        ]

        direction = [
            [
                -0.7071067811865476,
                0.7071067811865475,
                0.0
            ],
            [
                -0.7071067811865476,
                0.7071067811865475,
                0.0
            ],
            [
                0.7071067811865475,
                0.7071067811865476,
                0.0
            ],
            [
                0.7071067811865475,
                0.7071067811865476,
                0.0
            ],
            [
                0.7071067811865476,
                -0.7071067811865475,
                0.0
            ],
            [
                0.7071067811865476,
                -0.7071067811865475,
                0.0
            ],
            [
                -0.7071067811865475,
                -0.7071067811865476,
                0.0
            ],
            [
                -0.7071067811865475,
                -0.7071067811865476,
                0.0
            ]
        ]

    # create the set of thruster in the dynamics task
    thrusterSet = thrusterStateEffector.ThrusterStateEffector()
    scSim.AddModelToTask(dynTaskName, thrusterSet)

    # set the integrator to a variable time step of 7th-8th order
    integratorObject = svIntegrators.svIntegratorRKF78(scObject)
    scObject.setIntegrator(integratorObject)

    # Make a fresh thruster factory instance, this is critical to run multiple times
    thFactory = simIncludeThruster.thrusterFactory()

    # create the thruster devices by specifying the thruster type and its location and direction
    for pos_B, dir_B in zip(location, direction):
        if useDVThrusters:
            thFactory.create('MOOG_Monarc_22_6', pos_B, dir_B, cutoffFrequency=1.)
        else:
            thFactory.create('MOOG_Monarc_1', pos_B, dir_B, cutoffFrequency=1.)

    # get number of thruster devices
    numTh = thFactory.getNumOfDevices()

    # create thruster object container and tie to spacecraft object
    thrModelTag = "ACSThrusterDynamics"
    thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation
    scSim.AddModelToTask(fswTaskName, inertial3DObj)

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(fswTaskName, attError)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(fswTaskName, mrpControl)
    mrpControl.K = 3.5 * 10.0
    mrpControl.Ki = 0.0002  # make value negative to turn off integral feedback
    mrpControl.P = 30.0 * 10.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # setup the thruster force mapping module
    thrForceMappingObj = thrForceMapping.thrForceMapping()
    thrForceMappingObj.ModelTag = "thrForceMapping"
    scSim.AddModelToTask(fswTaskName, thrForceMappingObj)

    if useDVThrusters:
        controlAxes_B = [1, 0, 0,
                         0, 1, 0]
        thrForceMappingObj.thrForceSign = -1
    else:
        controlAxes_B = [1, 0, 0,
                         0, 1, 0,
                         0, 0, 1]
        thrForceMappingObj.thrForceSign = +1
    thrForceMappingObj.controlAxes_B = controlAxes_B

    # setup the Schmitt trigger thruster firing logic module
    thrFiringSchmittObj = thrFiringSchmitt.thrFiringSchmitt()
    thrFiringSchmittObj.ModelTag = "thrFiringSchmitt"
    scSim.AddModelToTask(fswTaskName, thrFiringSchmittObj)
    thrFiringSchmittObj.thrMinFireTime = 0.002
    thrFiringSchmittObj.level_on = .75
    thrFiringSchmittObj.level_off = .25
    if useDVThrusters:
        thrFiringSchmittObj.baseThrustState = 1

    #
    #   Setup data logging before the simulation is initialized
    #

    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, fswTimeStep, numDataPoints)
    mrpTorqueLog = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    thrMapLog = thrForceMappingObj.thrForceCmdOutMsg.recorder(samplingTime)
    thrTrigLog = thrFiringSchmittObj.onTimeOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(fswTaskName, mrpTorqueLog)
    scSim.AddModelToTask(fswTaskName, attErrorLog)
    scSim.AddModelToTask(fswTaskName, snTransLog)
    scSim.AddModelToTask(fswTaskName, snAttLog)
    scSim.AddModelToTask(fswTaskName, thrMapLog)
    scSim.AddModelToTask(fswTaskName, thrTrigLog)

    thrForceLog = []
    for i in range(numTh):
        thrForceLog.append(thrusterSet.thrusterOutMsgs[i].recorder(samplingTime))
        scSim.AddModelToTask(fswTaskName, thrForceLog[i])

    #
    # create FSW simulation messages
    #

    # create the FSW vehicle configuration message

    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # create the FSW Thruster configuration message
    if useDVThrusters:
        maxThrust = 22
    else:
        maxThrust = 1

    # A `clearSetup()` should be called first to clear out any pre-existing devices from an
    # earlier simulation run.  Next, the `maxThrust` value should be specified and used in the macro `create()`,
    # together with the locations and directions, and looped through a for cycle to consider all the thrusters.
    # The support macro `writeConfigMessage()` creates the required thrusters flight configuration message.
    fswSetupThrusters.clearSetup()
    for pos_B, dir_B in zip(location, direction):
        fswSetupThrusters.create(pos_B, dir_B, maxThrust)
    fswThrConfigMsg = fswSetupThrusters.writeConfigMessage()
    # an alternate method to pull un-modifed SIM Thruster configuration and create the corresponding FSW
    # configuration message is:
    fswThrConfigMsg = thFactory.getConfigMessage()

    #   set initial Spacecraft States
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # connect messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    thrForceMappingObj.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    thrForceMappingObj.thrConfigInMsg.subscribeTo(fswThrConfigMsg)
    thrForceMappingObj.vehConfigInMsg.subscribeTo(vcMsg)
    thrFiringSchmittObj.thrConfInMsg.subscribeTo(fswThrConfigMsg)
    thrFiringSchmittObj.thrForceInMsg.subscribeTo(thrForceMappingObj.thrForceCmdOutMsg)
    thrusterSet.cmdsInMsg.subscribeTo(thrFiringSchmittObj.onTimeOutMsg)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName,  scObject
                                              # , saveFile=fileName
                                              , thrEffectorList=thrusterSet
                                              , thrColors=vizSupport.toRGBA255("red")
                                              )
    vizSupport.setActuatorGuiSetting(viz, showThrusterLabels=True)

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
    dataLr = mrpTorqueLog.torqueRequestBody
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    dataMap = thrMapLog.thrForce
    dataSchm = thrTrigLog.OnTimeRequest

    dataThrust = []
    for i in range(numTh):
        dataThrust.append(np.array(thrForceLog[i].thrustForce))
    dataThrust = np.stack(np.transpose(dataThrust))

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeDataFSW = attErrorLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_requested_torque(timeDataFSW, dataLr)
    figureList = {}
    pltName = fileName + "1" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(1)

    plot_rate_error(timeDataFSW, dataOmegaBR)
    pltName = fileName + "2" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(2)

    plot_attitude_error(timeDataFSW, dataSigmaBR)
    pltName = fileName + "3" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(3)

    plot_thrForce(timeDataFSW, dataMap, numTh)
    pltName = fileName + "4" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(4)

    plot_OnTimeRequest(timeDataFSW, dataSchm, numTh)
    pltName = fileName + "5" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(5)

    plot_trueThrForce(timeDataFSW, dataThrust, numTh)
    pltName = fileName + "6" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(6)

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
        False,  # useDVThrusters
    )
