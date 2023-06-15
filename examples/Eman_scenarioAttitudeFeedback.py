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

    <iframe width="560" height="315" src="https://www.youtube.com/embed/As68r55iGY0" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

Demonstrates how to stabilize the tumble of a spacecraft orbiting the
Earth that is initially tumbling.
This script sets up a 6-DOF spacecraft which is orbiting the Earth.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioAttitudeFeedback.py

The simulation layout is
shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenarioAttitudeFeedback.svg
   :align: center

The dynamics simulation is setup using a :ref:`spacecraft` module to which a gravity
effector is attached.  Note that both the rotational and translational degrees of
freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.  For more
information on how to setup orbit, see :ref:`scenarioBasicOrbit`.

The control torque is simulated using the :ref:`extForceTorque` module.  This module can
accept a torque in body frame components either through an input message, or through
a module internal torque vector which can be set in python.  In this simulation, the
flight software is providing the attitude control torque message which is connected to
the torque input message of this module.  If an external torque is being simulated,
then the module internal torque vector is set to a constant value.

The flight software algorithm module require a navigation message with the
spacecraft orientation and attitude rates.  This is setup using the :ref:`simpleNav`
module. By just invoking a sensor module it is setup to run without any simulated
corruptions.  Thus in this simulation it will return truth measurements.

Next the flight software algorithms need to be setup.  The inertial pointing reference
frame definition is provided through the simple :ref:`inertial3D` module.  The only input
it requires is the desired inertial heading.

The reference frame states and the navigation message (output of :ref:`simpleNav` are fed
into the :ref:`attTrackingError` module.  It is setup to compute the attitude tracking error
between the body frame :math:`\cal B` and the reference frame :math:`\cal R`.
If a body fixed frame other than :math:`\cal B`
needs to be driven towards *R*, this could be configured as well in this module.

Finally the tracking errors are fed to the classic MRP feedback control module.  The
algorithm of this is discussed in the text book
`Analytical Mechanics of Space Systems <http://dx.doi.org/10.2514/4.105210>`__.
The control torque output vector message of this
module is connected back to the input message of the :ref:`extForceTorque` module to close
the control loop.

While the nominal simulation has set ``useCMsg`` flag to False, with it set to ``True`` it illustrates two things.
First it shows how to create a C-wrapped C-message in Python and write to it.  This is done with the
``VehicleConfigMsg`` message.  Second, it illustrates how instead of writing to a module internal
output message (see ``mrpControlConfig.cmdTorqueOutMsg``) we can re-direct the module to write to a
stand-alone message ``cmdTorqueMsg`` instead.  This is useful if we need to have multiple module be writing
to a single output message such as if several flight software stacks are being setup.

When the simulation completes 3 plots are shown for the MRP attitude history, the rate
tracking errors, as well as the control torque vector.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useUnmodeledTorque = False, useIntGain = False, useKnownTorque = False, useCMsg = False

.. image:: /_images/Scenarios/scenarioAttitudeFeedback1000.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2000.svg
   :align: center

::

    show_plots = True, useUnmodeledTorque = True, useIntGain = False, useKnownTorque = False, useCMsg = False

Note that, as expected,
the orientation error doesn't settle to zero, but rather converges to a non-zero offset
proportional to the un-modeled torque being simulated.  Also, the control torques settle on
non-zero steady-state values.

.. image:: /_images/Scenarios/scenarioAttitudeFeedback1100.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2100.svg
   :align: center

::

    show_plots = True, useUnmodeledTorque = True, useIntGain = True, useKnownTorque = False, useCMsg = False

In this case the orientation error does settle to zero.  The integral term changes the control torque
to settle on a value that matches the un-modeled external torque.

.. image:: /_images/Scenarios/scenarioAttitudeFeedback1110.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2110.svg
   :align: center

::

    show_plots = True, useUnmodeledTorque = True, useIntGain = False, useKnownTorque = True, useCMsg = False

In this case the orientation error does settle to zero as the feed-forward term compensates for
the external torque.  The control torque is now caused
to settle on a value that matches the un-modeled external torque.

.. image:: /_images/Scenarios/scenarioAttitudeFeedback1101.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2101.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav() and
#           mrpFeedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 19, 2016
#
import os

import numpy as np

np.set_printoptions(precision=16)

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeRW

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simpleNav
from Basilisk.simulation import reactionWheelStateEffector

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError

# import message declarations
from Basilisk.architecture import messaging

# attempt to import vizard
from Basilisk.utilities import vizSupport


# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, useJitterSimple, useUnmodeledTorque, useIntGain, useKnownTorque, useCMsg):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useUnmodeledTorque (bool): Specify if an external torque should be included
        useIntGain (bool): Specify if the feedback control uses an integral feedback term
        useKnownTorque (bool): Specify if the external torque is feed forward in the control
        useCMsg (bool): Specify if the C-based stand-alone messages should be used

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

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    # use the input flag to determine which external torque should be applied
    # Note that all variables are initialized to zero.  Thus, not setting this
    # vector would leave it's components all zero for the simulation.
    if useUnmodeledTorque:
        extFTObject.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)

    # setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.K = 3.5
    if useIntGain:
        mrpControlConfig.Ki = 0.0002  # make value negative to turn off integral feedback
    else:
        mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1
    if useKnownTorque:
        mrpControlConfig.knownTorquePntB_B = [0.25, -0.25, 0.1]

    #
    # create simulation messages
    #
    # The MRP Feedback algorithm requires the vehicle configuration structure. This defines various spacecraft
    # related states such as the inertia tensor and the position vector between the primary Body-fixed frame
    # B origin and the center of mass (defaulted to zero).  The message payload is created through
    configData = messaging.VehicleConfigMsgPayload()
    configData.ISCPntB_B = I
    # Two methods are shown to create either a C++ or C wrapped msg object in python.  The
    # preferred method is to just create C++ wrapped messages.
    if not useCMsg:
        # create standard C++ wrapped C-msg copy
        configDataMsg = messaging.VehicleConfigMsg()
    else:
        # example of how to create a C-wrapped C-msg in Python, have it init and write the data
        configDataMsg = messaging.VehicleConfigMsg_C().init(configData)
        # example of how to create a C-wrapped C-msg in Python that is initialized
        configDataMsg = messaging.VehicleConfigMsg_C().init()
    configDataMsg.write(configData)

    #
    # connect the messages to the modules
    #
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attErrorConfig.attRefInMsg.subscribeTo(inertial3DConfig.attRefOutMsg)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(configDataMsg)
    if useCMsg:
        cmdTorqueMsg = messaging.CmdTorqueBodyMsg_C()
        # connect to external commanded torque msg
        messaging.CmdTorqueBodyMsg_C_addAuthor(mrpControlConfig.cmdTorqueOutMsg, cmdTorqueMsg)
        extFTObject.cmdTorqueInMsg.subscribeTo(cmdTorqueMsg)
    else:
        # connect to module-internal commanded torque msg
        extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    #
    # Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    snLog = scObject.scStateOutMsg.recorder(samplingTime)
    # instead of recording the contents of a C++ output message, you can also recording
    # the incoming contents of a C++ input message.  However, note that you must setup the
    # input message recorder after this input message has been subscribed to another message.
    # Otherwise, you are reading an uninitialized msg which leads to lovely segmentation faults.
    snLog = sNavObject.scStateInMsg.recorder(samplingTime)
    attErrorLog = attErrorConfig.attGuidOutMsg.recorder(samplingTime)
    if useCMsg:
        # create stand-along commanded torque msg and setup recorder()
        mrpLog = cmdTorqueMsg.recorder(samplingTime)
    else:
        mrpLog = mrpControlConfig.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, snLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, mrpLog)

    #
    #   set initial Spacecraft States
    #
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


    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
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
    #   plot the results
    #
    timeAxis = attErrorLog.times()
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attErrorLog.sigma_BR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1" + str(int(useUnmodeledTorque)) + str(int(useIntGain)) + str(int(useKnownTorque))
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, mrpLog.torqueRequestBody[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Control Torque $L_r$ [Nm]')
    pltName = fileName + "2" + str(int(useUnmodeledTorque)) + str(int(useIntGain)) + str(int(useKnownTorque))
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attErrorLog.omega_BR_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    plt.figure(4)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, snLog.r_BN_N[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial Position [km]')

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
        True,  #useJitterSimple
        False,  # useUnmodeledTorque
        False,  # useIntGain
        False,  # useKnownTorque
        False
    )
