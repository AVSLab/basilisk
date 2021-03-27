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

Demonstrates how to use RWs to stabilize the tumble of a spacecraft orbiting the Earth.
This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The goal is to
illustrate how the Reaction Wheel (RW) state effector can be added to the rigid :ref:`spacecraft` hub,
and what flight algorithm module is used to control these RWs.

The first setup runs the RW control to produce a desired set of RW motor torques
which are then connected directly to the RW device input states.  The second setup illustrates
how to setup voltage based I/O modules to the RW devices, both on the FSW and SIM side.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioMtbMomentumManagement.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenarioAttitudeFeedbackRW.svg
   :align: center

When the simulation completes several plots are shown for the MRP attitude history, the rate
tracking errors, as well as the RW motor torque components, as well as the RW wheel speeds.

The fundamental simulation setup is the same as the one used in
:ref:`scenarioAttitudeFeedback`.
The dynamics simulation is setup using a :ref:`spacecraft` module to which an Earth gravity
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
a wheel with a simple jitter model, or a wheel with a fully couped model.


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

    mrpControlConfig.rwParamsInMsg.subscribeTo(rwParamMsg)
    mrpControlConfig.rwSpeedsInMsg.subscribeTo(rwSpeedsMsg)

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
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
#           while using the RWs to do the attitude control actuation.
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 7, 2017
#

import sys
import numpy as np
import os
import matplotlib.pyplot as plt
from Basilisk.utilities import macros

from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    inertial3D, rwMotorTorque, rwMotorVoltage,
                                    tamComm, mtbMomentumManagement)
from Basilisk.simulation import (reactionWheelStateEffector, 
                                 motorVoltageInterface, simpleNav, 
                                 magneticFieldCenteredDipole, 
                                 magneticFieldWMM, magnetometer, MtbEffector, 
                                 spacecraft)
from Basilisk.utilities import (SimulationBaseClass, fswSetupRW, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport,
                                simSetPlanetEnvironment)
from Basilisk.architecture import messaging

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
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
    plt.grid(True)

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
    plt.ylabel('RW Motor Torque [Nm]')
    plt.grid(True)
    
def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW):
    """Plot the RW actual motor torques."""
    plt.figure(2)
    for idx in range(numRW):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
        plt.plot(timeData, dataRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque [Nm]')
    plt.grid(True)

def plot_rate_error(timeData, dataOmegaBR):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')
    plt.grid(True)
    
def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    """Plot the RW spin rates."""
    plt.figure(4)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed [RPM] ')
    plt.grid(True)
    
def plot_magnetic_field(timeData, dataMagField):
    """Plot the magnetic field."""
    plt.figure(5)
    for idx in range(3):
        plt.plot(timeData, dataMagField[:, idx] * 1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$B\_N_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Magnetic Field [nT]')
    plt.grid(True)

def plot_data_tam(timeData, dataTam):
    """Plot the magnetic field."""
    plt.figure(6)
    for idx in range(3):
        plt.plot(timeData, dataTam[:, idx] * 1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$TAM\_S_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Magnetic Field [nT]')
    plt.grid(True)
    
def plot_data_tam_comm(timeData, dataTamComm):
    """Plot the magnetic field."""
    plt.figure(7)
    for idx in range(3):
        plt.plot(timeData, dataTamComm[:, idx] * 1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$TAM\_B_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Magnetic Field [nT]')
    plt.grid(True)
    
def plot_data_mtb_momentum_management(timeData, dataMtbMomentumManegement, numMTB):
    """Plot the magnetic field."""
    plt.figure(8)
    for idx in range(numMTB):
        plt.plot(timeData, dataMtbMomentumManegement[:, idx],
                 color=unitTestSupport.getLineColor(idx, numMTB),
                 label=r'$MTB\_T_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Torque Rod Dipoles [A-m2]')
    plt.grid(True)
    

def plot_data_rw_motor_torque_desired(dataUsReq, tauRequested_W, numRW):
    """Plot the RW desired motor torques."""
    plt.figure(9)
    for idx in range(numRW):
        '''
        plt.plot(dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
        '''
        plt.plot(tauRequested_W[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')
    plt.grid(True)
    
def run(show_plots):
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
    simulationTime = macros.min2nano(45.)

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

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [10.5, 0., 0.,
         0., 8., 0.,
         0., 0., 6.75]
    scObject.hub.mHub = 10.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, None, 1)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = messaging.BalancedWheels


    beta = 52. * np.pi / 180.
    Gs = np.array([
            [0.,            0.,             np.cos(beta), -np.cos(beta)],
            [np.cos(beta),  np.sin(beta),  -np.sin(beta), -np.sin(beta)],
            [np.sin(beta), -np.cos(beta),   0.,             0.]])

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    RW1 = rwFactory.create('BCT_RWP015', Gs[:, 0], Omega=5000.  # RPM
                           , RWModel=varRWModel, useRWfriction=False,
                           )
    RW2 = rwFactory.create('BCT_RWP015', Gs[:, 1], Omega=5000.  # RPM
                           , RWModel=varRWModel, useRWfriction=False,
                           )
    RW3 = rwFactory.create('BCT_RWP015', Gs[:, 2], Omega=5000.  # RPM
                           , RWModel=varRWModel, useRWfriction=False,
                           )
    
    RW4 = rwFactory.create('BCT_RWP015', Gs[:, 3], Omega=5000.  # RPM
                           , RWModel=varRWModel, useRWfriction=False,
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


    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    # create magnetic field module
    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.ModelTag = "WMM"
    magModule.dataPath = bskPath + '/supportData/MagneticField/'
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg('2019 June 27, 10:23:0.0 (UTC)')
    magModule.epochInMsg.subscribeTo(epochMsg)
    magModule.addSpacecraftToModel(scObject.scStateOutMsg)  # this command can be repeated if multiple
    scSim.AddModelToTask(simTaskName, magModule)
    
    # add magnetic torque bar effector
    mtbEff = MtbEffector.MtbEffector()
    mtbEff.ModelTag = "MtbEff"
    scObject.addDynamicEffector(mtbEff)
    scSim.AddModelToTask(simTaskName, mtbEff)
    
    
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
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    
    mrpControlConfig.K = 0.
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 0.
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

    # create the minimal TAM module
    TAM = magnetometer.Magnetometer()
    TAM.ModelTag = "TAM_sensor"
    # specify the optional TAM variables
    TAM.scaleFactor = 1.0
    
    # changed this from 1e-9
    TAM.senNoiseStd = [0.0,  0.0, 0.0]
    '''
    if useBias:
        TAM.senBias = [0, 0, -1e-6]  # Tesla
    if useBounds:
        TAM.maxOutput = 3.5e-4  # Tesla
        TAM.minOutput = -3.5e-4  # Tesla
    '''
    scSim.AddModelToTask(simTaskName, TAM)
    
    # setup tamComm module
    tamCommConfig = tamComm.tamConfigData()
    tamCommConfig.dcm_BS = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    tamCommWrap = scSim.setModelDataWrap(tamCommConfig)
    tamCommWrap.ModelTag = "tamComm"
    scSim.AddModelToTask(simTaskName, tamCommWrap, tamCommConfig)
    
    # setup mtbMomentumManagement module
    mtbMomentumManagementConfig = mtbMomentumManagement.mtbMomentumManagementConfig()
    mtbMomentumManagementConfig.wheelSpeedBiases = [500. * macros.rpm2radsec, 500. * macros.rpm2radsec, 500. * macros.rpm2radsec, 500. * macros.rpm2radsec]
    mtbMomentumManagementConfig.cGain = 0.003
    mtbMomentumManagementWrap = scSim.setModelDataWrap(mtbMomentumManagementConfig)
    mtbMomentumManagementWrap.ModelTag = "mtbMomentumManagement"          
    scSim.AddModelToTask(simTaskName, mtbMomentumManagementWrap, mtbMomentumManagementConfig)
    
    # mtbConfigData message
    mtbConfigParams = messaging.MTBArrayConfigMsgPayload()
    mtbConfigParams.numMTB = 3
    mtbConfigParams.GtMatrix_B = [
        1., 0., 0.,
        0., 1., 0.,
        0., 0., 1.]
    PLACEHOLDER = 0.
    mtbConfigParams.muMax_W = [PLACEHOLDER, PLACEHOLDER, PLACEHOLDER, PLACEHOLDER]
    mtbParamsInMsg = messaging.MTBArrayConfigMsg().write(mtbConfigParams)
    
    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 1000
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    rwMotorLog = rwMotorTorqueConfig.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrorLog = attErrorConfig.attGuidOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    magLog = magModule.envOutMsgs[0].recorder(samplingTime)
    tamLog = TAM.tamDataOutMsg.recorder(samplingTime)
    tamCommLog = tamCommConfig.tamOutMsg.recorder(samplingTime)
    mtbDipoleCmdsLog = mtbMomentumManagementConfig.mtbCmdOutMsg.recorder(samplingTime)
    
    scSim.AddModelToTask(simTaskName, rwMotorLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, snTransLog)
    scSim.AddModelToTask(simTaskName, magLog)
    scSim.AddModelToTask(simTaskName, tamLog)
    scSim.AddModelToTask(simTaskName, tamCommLog)
    scSim.AddModelToTask(simTaskName, mtbDipoleCmdsLog)


    
    scSim.AddVariableForLogging(mtbMomentumManagementWrap.ModelTag + ".tauDesiredRW_W", macros.sec2nano(0.1), 0, numRW)
    
    
    
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
    oe.a = 6778.14 * 1000.  # meters
    oe.e = 0.00
    oe.i = 45. * macros.D2R
    oe.Omega = 60. * macros.D2R
    oe.omega = 0. * macros.D2R
    oe.f = 0. * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # link messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attErrorConfig.attRefInMsg.subscribeTo(inertial3DConfig.attRefOutMsg)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControlConfig.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControlConfig.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)

    TAM.stateInMsg.subscribeTo(scObject.scStateOutMsg)
    TAM.magInMsg.subscribeTo(magModule.envOutMsgs[0])
    tamCommConfig.tamInMsg.subscribeTo(TAM.tamDataOutMsg)
    
    rwMotorTorqueConfig.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueConfig.vehControlInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)
    
    mtbMomentumManagementConfig.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mtbMomentumManagementConfig.mtbParamsInMsg.subscribeTo(mtbParamsInMsg)
    mtbMomentumManagementConfig.tamSensorBodyInMsg.subscribeTo(tamCommConfig.tamOutMsg)
    mtbMomentumManagementConfig.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    #mtbMomentumManagementConfig.vehControlInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)
    mtbMomentumManagementConfig.rwMotorTorqueInMsg.subscribeTo(rwMotorTorqueConfig.rwMotorTorqueOutMsg)
    
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(mtbMomentumManagementConfig.rwMotorTorqueOutMsg)
    
    mtbEff.mtbCmdInMsg.subscribeTo(mtbMomentumManagementConfig.mtbCmdOutMsg)
    mtbEff.mtbParamsInMsg.subscribeTo(mtbParamsInMsg)
    mtbEff.magInMsg.subscribeTo(magModule.envOutMsgs[0])
    
    
    # initialize configure and execute sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   retrieve the logged data
    dataUsReq = rwMotorLog.motorTorque
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    dataPos = snTransLog.r_BN_N
    dataOmegaRW = mrpLog.wheelSpeeds
    dataRW = []
    for i in range(numRW):
        dataRW.append(rwLogs[i].u_current)
    dataMagField = magLog.magField_N
    dataTam = tamLog.tam_S
    dataTamComm = tamCommLog.tam_B
    dataMtbDipoleCmds = mtbDipoleCmdsLog.mtbDipoleCmds
    
    
    tauRequested_W = scSim.GetLogVariableData(mtbMomentumManagementWrap.ModelTag + ".tauDesiredRW_W")
    np.set_printoptions(precision=16)

    #   plot the results
    timeData = rwMotorLog.times() * macros.NANO2MIN
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

    plot_magnetic_field(timeData, dataMagField)
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(5)
    
    plot_data_tam(timeData, dataTam)
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(6)
    
    plot_data_tam_comm(timeData, dataTamComm)
    pltName = fileName + "6"
    figureList[pltName] = plt.figure(7)
    
    plot_data_mtb_momentum_management(timeData, dataMtbDipoleCmds, mtbConfigParams.numMTB)
    pltName = fileName + "7"
    figureList[pltName] = plt.figure(8)
    
    
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
