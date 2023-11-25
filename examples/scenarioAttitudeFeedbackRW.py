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

    <iframe width="560" height="315" src="https://www.youtube.com/embed/wyDw7NEanqk" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

Demonstrates how to use RWs to stabilize the tumble of a spacecraft orbiting the Earth.
This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The goal is to
illustrate how the Reaction Wheel (RW) state effector can be added to the rigid :ref:`spacecraft` hub,
and what flight algorithm module is used to control these RWs.

The first setup runs the RW control to produce a desired set of RW motor torques
which are then connected directly to the RW device input states.  The second setup illustrates
how to setup voltage based I/O modules to the RW devices, both on the FSW and SIM side.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioAttitudeFeedbackRW.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenarioAttitudeFeedbackRW.svg
   :align: center

When the simulation completes several plots are shown for the MRP attitude history, the rate
tracking errors, as well as the RW motor torque components, as well as the RW wheel speeds.

The fundamental simulation setup is the same as the one used in
:ref:`scenarioAttitudeFeedback`.
The dynamics simulation is setup using a :ref:`Spacecraft` module to which an Earth gravity
effector is attached.  The simple navigation module is still used to output the inertial attitude,
angular rate, as well as position and velocity message. The simulation is setup to run in a single
process again.  If the flight algorithms and simulation tasks are to run at different rates, then see
:ref:`scenarioAttitudeFeedback2T` on how to setup a 2 thread simulation.

How to Add RW Devices to the Spacecraft Simulation
--------------------------------------------------

For the spacecraft simulation side of this script, the new element is adding RW effectors to the
the rigid spacecraft hub.  The support macro ``simIncludeRW.py`` provides several convenient tools to facilitate
this simulated RW setup process.  This script allows the user to readily create RWs from a database of
public RW specifications, customize them if needed, and add them to the :ref:`spacecraft` module.

The first task is to create a fresh instance of the RW factory class ``rwFactory()``.  This factory is able
to create a list of RW devices, and return copies that can easily be manipulated and customized if needed.
The next step in this code is to store the correct ``RWModel`` state.  This can be either a balanced wheel,
a wheel with a simple jitter model, or a wheel with a fully coupled model.


The next step in this simulation setup is to use create() to include a particular RW devices.
The `rwFactory` class in :ref:`simIncludeRW` contains several
public specifications of RW devices which can be accessed by specifying the wheel name, ``Honeywell_HR16``
in this case.  The  2nd required argument is the spin axis :math:`\hat{\mathbf g}_B`.  It is a unit
vector expressed in the :math:`\cal B`-frame.  The remaining arguments are all optional.  In this simulation
each RW is given a different initial RW spin :math:`\Omega`
in units of RPMs.  The 3rd RW specifies the body-relative location of the RW center of mass.  The
other two RWs use a default value which is a zero vector.
This last position vector is only used if an off-balanced RW device is being modeled.

Each RW device has several default options that can be customized if needed.  For example,
the ``Honeywell_HR16`` comes in three different momentum storage configurations.  When calling the
`create()` command, the desired storage capacity must be specified through the ``maxMomentum`` argument.

The following table provides a comprehensive list of all the optional arguments of the ``create()``
command.  This table list the arguments, default values, as well as expected units.

+---------------------+-------+----------+----------------------------------------+--------------------+
|  Argument           | Units | Type     | Description                            | Default            |
+=====================+=======+==========+========================================+====================+
| RWModel             |       | String   | flag indicating the RW dynamical model.| ``BalancedWheels`` |
|                     |       |          | Options are ``BalancedWheels``,        |                    |
|                     |       |          | ``JitterSimple`` and                   |                    |
|                     |       |          | ``JitterFullyCoupled``                 |                    |
+---------------------+-------+----------+----------------------------------------+--------------------+
| Omega               | RPM   | Float    | initial Wheel speed                    | 0.0                |
+---------------------+-------+----------+----------------------------------------+--------------------+
| maxMomentum         | Nms   | Float    | maximum RW angular momentum storage    | 0.0                |
+---------------------+-------+----------+----------------------------------------+--------------------+
| useRWfriction       |       | Bool     | flag to turn on RW wheel friction      | False              |
+---------------------+-------+----------+----------------------------------------+--------------------+
| useMinTorque        |       | Bool     | flag to turn on a min. RW torque       | False              |
+---------------------+-------+----------+----------------------------------------+--------------------+
| useMaxTorque        |       | Bool     | flag to turn on RW torque saturation   | True               |
+---------------------+-------+----------+----------------------------------------+--------------------+
| linearFrictionRatio |       | Float    | Coulomb static friction value to model | -1 (Off)           |
|                     |       |          | stickage, negative values turn off this|                    |
|                     |       |          | feature                                |                    |
+---------------------+-------+----------+----------------------------------------+--------------------+
| rWB_B               | m     | Float(3) | RW center of mass location relative to | [0.0, 0.0, 0.0]    |
|                     |       |          | B, in :math`\cal B`-frame components   |                    |
+---------------------+-------+----------+----------------------------------------+--------------------+
| label               |       | String   | unique device label, must be not exceed| RWi                |
|                     |       |          | 10 characters.  If not provided, the   |                    |
|                     |       |          | function will auto-generate names using|                    |
|                     |       |          | RWi where i is the RW wheel index      |                    |
|                     |       |          | starting with 1.                       |                    |
+---------------------+-------+----------+----------------------------------------+--------------------+

The command ``addToSpacecraft()`` adds all the created RWs to the :ref:`spacecraft` module.  The final step
is as always to add the vector of RW effectors (called ``rwStateEffector`` above) to the list of simulation
tasks.  However, note that the dynamic effector should be evaluated before the :ref:`spacecraft` module,
which is why it is being added with a higher priority than the ``scObject`` task.  Generally speaking
we should have the execution order::

    effectors -> dynamics -> sensors


If you want to configure or customize the RWs, the ``rwFactor()`` class is very convenient. Assume you want
to override the default value of the maximum RW speed from 6000RPM to 10,000RPM.  After declaring the RW
and keeping a copy named RW1, ``Omega_max`` stage is changed using::

    RW1.Omega_max = 10000.0*macros.RPM

These changes must be made before adding the RWs to the spacecraft.  If the `RW1` handler is not
stored when the RW is create, any setup RW devices can be recalled through the device label.
For example, the above modification could also be done with::

    rwFactory.rwList['RW1'].Omega_max = 10000.0*macros.RPM

Flight Algorithm Changes to Control RWs
---------------------------------------

The general flight algorithm setup is the same as in the earlier simulation script. Here we
use again the :ref:`inertial3D` guidance module, the :ref:`attTrackingError` module to evaluate the
tracking error states, and the :ref:`mrpFeedback` module to provide the desired :math:`{\mathbf L}_r`
control torque vector.  In this simulation we want the MRP attitude control module to take
advantage of the RW spin information.  This is achieved by adding the 2 extra lines::

    mrpControl.rwParamsInMsg.subscribeTo(rwParamMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwSpeedsMsg)

The first line specifies the RW configuration flight message name, and the second name
connects the RW speed output message as an input message to this control module.  This simulates
the RW speed information being sensed and returned to this algorithm.  This message names
are not provided, then the BSK control modules automatically turn off any RW gyroscopic
compensation.

Instead of directly simulating this control torque vector, new
algorithm modules are required to first map :math:`{\mathbf L}_r` on the set of RW motor torques
:math:`u_B`.  This is achieved by adding the :ref:`rwMotorTorque` module.
Note that the output vector of RW motor torques :math:`u_B` is set to connect with
the RW state effector command input message.  Further, this module inputs the typical
vehicle configuration message, as well as a message containing the flight algorithm
information of the RW devices.  This torque mapping module can map the full 3D :math:`{\mathbf L}_r`
vector onto RW motor torques, or only a subset.  This is specified through the ``controlAxes_B`` variable
which specifies up to 3 orthogonal axes to be controlled.  In this simulation the full 3D vector is
mapped onto motor torques.

The flight algorithm needs to know how many RW devices are on the spacecraft and what their
spin axis :math:`\hat{\mathbf g}_B` are.  This is set through a flight software message that is read
in by flight algorithm modules that need this info.  Two options are shown in the code how to achieve this.

First, the required flight RW configuration message can be written using
a separate support macros called ``fswSetupRW.py``.
The a ``clearSetup()`` should be called first to clear out any pre-existing RW devices from an
earlier simulation run.  Next, the script above uses the same RW information as what the simulation
uses.  In this configuration we are simulating perfect RW device knowledge.  If imperfect RW knowledge
is to be simulated, then the user would input the desired flight states rather than the true
simulation states.  The support macro ``writeConfigMessage()`` creates the required RW flight configuration
message.

Second, the ``rwFactory`` class method ``getConfigMessage()`` can be used to extract the desired
:ref:`RWArrayConfigMsgPayload` message.  Using this approach an exact copy is ensured between the FSW and
simulation RW configuration states, but it is less convenient in introduce differences.

Setting up an Analog RW Interface Module
----------------------------------------

The scenario illustrates how to setup a RW analog I/O module.  This is illustrated in the updated
flow diagram illustration.

.. image:: /_images/static/test_scenarioAttitudeFeedbackRWc2.svg
   :align: center

The default interface voltage output name will connect with the default RW
input message name.  Naturally, these can be customized if needed.  The one parameter
that must be set is the voltage to torque conversion gain of the electric motor being modeled.

On the FSW side, the commanded motor torques must be directed towards a new module that converts
the required torque to a commanded voltage.  The first step is to re-direct the
rWMotorTorque() output to not directly be the input to the RW SIM devices, but rather the
input to the RW voltage command module:


Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useJitterSimple = False, useRWVoltageIO = False

Note that in the RW motor torque plot both the required control torque :math:`\hat u_B` and the true
motor torque :math:`u_B` are shown.  This illustrates that with this maneuver the RW devices are being
saturated, and the attitude still eventually stabilizes.

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRW100.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRW200.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRW300.svg
   :align: center

::

    show_plots = True, useJitterSimple = True, useRWVoltageIO = False

Here the simple RW jitter model is engaged for each of the RWs. Change this option before the RW is created.
As this is set before any of the RW created in this
scenario, all the RWs have jitter engaged if this '`useJitterSimple`' flag is set.
The impact of the RW jitter is very small, naturally.  The plots for this case look very similar to
the balanced RW case.  But there is a distinct numerical difference.

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRW110.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRW210.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRW310.svg
   :align: center

::

    show_plots = True, useJitterSimple = False, useRWVoltageIO = True

Here the simple RW jitter model is engaged for each of the RWs. Change this option before the RW is created.
As this is set before any of the RW created in this
scenario, all the RWs have jitter engaged if this '`useJitterSimple`' flag is set.
The impact of the RW jitter is very small, naturally.  The plots for this case look very similar to
the balanced RW case.  But there is a distinct numerical difference.

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRW101.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRW201.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRW301.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), RWs, simpleNav() and
#           mrpFeedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
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
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    inertial3D, rwMotorTorque, rwMotorVoltage)
from Basilisk.simulation import reactionWheelStateEffector, motorVoltageInterface, simpleNav, spacecraft
from Basilisk.utilities import (SimulationBaseClass, fswSetupRW, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)

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


def plot_rw_voltages(timeData, dataVolt, numRW):
    """Plot the RW voltage inputs."""
    plt.figure(5)
    for idx in range(numRW):
        plt.plot(timeData, dataVolt[:, idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$V_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Voltage (V)')

def run(show_plots, useJitterSimple, useRWVoltageIO):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useJitterSimple (bool): Specify if the RW simple jitter model should be included
        useRWVoltageIO (bool): Specify if the RW voltage interface should be simulated.

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
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, 1)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = messaging.BalancedWheels
    if useJitterSimple:
        varRWModel = messaging.JitterSimple

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
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)

    if useRWVoltageIO:
        rwVoltageIO = motorVoltageInterface.MotorVoltageInterface()
        rwVoltageIO.ModelTag = "rwVoltageInterface"

        # set module parameters(s)
        rwVoltageIO.setGains(np.array([0.2 / 10.] * 3)) # [Nm/V] conversion gain

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, rwVoltageIO)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.K = 3.5
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)

    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B

    if useRWVoltageIO:
        fswRWVoltage = rwMotorVoltage.rwMotorVoltage()
        fswRWVoltage.ModelTag = "rwMotorVoltage"

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, fswRWVoltage)

        # set module parameters
        fswRWVoltage.VMin = 0.0  # Volts
        fswRWVoltage.VMax = 10.0  # Volts

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, rwMotorLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, snTransLog)

    # To log the RW information, the following code is used:
    mrpLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, mrpLog)

    # A message is created that stores an array of the \f$\Omega\f$ wheel speeds.  This is logged
    # here to be plotted later on.  However, RW specific messages are also being created which
    # contain a wealth of information.  The vector of messages is ordered as they were added.  This
    # allows us to log RW specific information such as the actual RW motor torque being applied.
    rwLogs = []
    for item in range(numRW):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, rwLogs[item])
    if useRWVoltageIO:
        rwVoltLog = fswRWVoltage.voltageOutMsg.recorder(samplingTime)
        scSim.AddModelToTask(simTaskName, rwVoltLog)

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Two options are shown to setup the FSW RW configuration message.
    # First case: The FSW RW configuration message
    # uses the same RW states in the FSW algorithm as in the simulation.  In the following code
    # the fswSetupRW helper functions are used to individually add the RW states.  The benefit of this
    # method of the second method below is that it is easy to vary the FSW parameters slightly from the
    # simulation parameters.  In this script the second method is used, while the fist method is included
    # to show both options.
    fswSetupRW.clearSetup()
    for key, rw in rwFactory.rwList.items():
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    fswRwParamMsg1 = fswSetupRW.writeConfigMessage()

    # Second case: If the exact same RW configuration states are to be used by the simulation and fsw, then the
    # following helper function is convenient to extract the fsw RW configuration message from the
    # rwFactory setup earlier.
    fswRwParamMsg2 = rwFactory.getConfigMessage()
    fswRwParamMsg = fswRwParamMsg2

    #
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
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              # , saveFile=fileName
                                              , rwEffectorList=rwStateEffector
                                              )
    # link messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    if useRWVoltageIO:
        fswRWVoltage.torqueInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
        fswRWVoltage.rwParamsInMsg.subscribeTo(fswRwParamMsg)
        rwVoltageIO.motorVoltageInMsg.subscribeTo(fswRWVoltage.voltageOutMsg)
        rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwVoltageIO.motorTorqueOutMsg)
    else:
        rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

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
    dataUsReq = rwMotorLog.motorTorque
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    dataPos = snTransLog.r_BN_N
    dataOmegaRW = mrpLog.wheelSpeeds

    dataRW = []
    for i in range(numRW):
        dataRW.append(rwLogs[i].u_current)
    if useRWVoltageIO:
        dataVolt = rwVoltLog.voltage
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeData = rwMotorLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_attitude_error(timeData, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1" + str(int(useJitterSimple)) + str(int(useRWVoltageIO))
    figureList[pltName] = plt.figure(1)

    plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)
    pltName = fileName + "2" + str(int(useJitterSimple)) + str(int(useRWVoltageIO))
    figureList[pltName] = plt.figure(2)

    plot_rate_error(timeData, dataOmegaBR)
    plot_rw_speeds(timeData, dataOmegaRW, numRW)
    pltName = fileName + "3" + str(int(useJitterSimple)) + str(int(useRWVoltageIO))
    figureList[pltName] = plt.figure(4)

    if useRWVoltageIO:
        plot_rw_voltages(timeData, dataVolt, numRW)
        pltName = fileName + "4" + str(int(useJitterSimple)) + str(int(useRWVoltageIO))
        figureList[pltName] = plt.figure(5)

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
        False,  # useJitterSimple
        True  # useRWVoltageIO
    )
