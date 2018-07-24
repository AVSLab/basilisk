''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraftPlus(), RWs, simpleNav() and
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
#           while using the RWs to do the attitude control actuation.
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 7, 2017
#


import numpy as np
import os
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import (MRP_Feedback, attTrackingError, fswMessages,
                                    inertial3D, rwMotorTorque, rwMotorVoltage)
from Basilisk.simulation import reactionWheelStateEffector, rwVoltageInterface, simple_nav, spacecraftPlus
from Basilisk.utilities import (SimulationBaseClass, fswSetupRW, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport)

# Plotting functions
def plot_attitude_error(timeData, dataSigmaBR):
    plt.figure(1)
    for idx in range(1, 4):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error $\sigma_{B/R}$')

def plot_rw_cmd_torque(timeData, dataUsReq, numRW):
    plt.figure(2)
    for idx in range(1, 4):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$\hat u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW):
    plt.figure(2)
    for idx in range(1, 4):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$\hat u_{s,' + str(idx) + '}$')
        plt.plot(timeData, dataRW[idx - 1][:, 1],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

def plot_rate_error(timeData, dataOmegaBR):
    plt.figure(3)
    for idx in range(1, 4):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s) ')

def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    plt.figure(4)
    for idx in range(1, numRW + 1):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$\Omega_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')


def plot_rw_voltages(timeData, dataVolt, numRW):
    plt.figure(5)
    for idx in range(1, numRW + 1):
        plt.plot(timeData, dataVolt[:, idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$V_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Voltage (V)')


## \defgroup Tutorials_2_2
## @{
# Demonstrates how to use RWs to stabilize the tumble of a spacecraft orbiting the
# Earth.
#
# Attitude Detumbling Simulation using RW Effectors {#scenarioAttitudeFeedbackRW}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The goal is to
# illustrate how Reaction Wheel (RW) state effector can be added to the rigid
# spacecraftPlus() hub, and what flight algorithm module is used to control these RWs.
#  The scenario is
# setup to be run in multiple configurations:
# Setup | useJitterSimple    | useRWVoltageIO
# ----- | -------------------|----------------
# 1     | False              | False
# 2     | True               | False
# 3     | False              | True
#
# The first setup runs the RW control to produce a desired set of RW motor torques
# which are then connected directly to the RW device input states.  The second setup illustrates
# how to setup voltage based I/O modules to the RW devices, both on the FSW and SIM side.
#
# To run the default scenario 1., call the python script from a Terminal window through
#
#       python scenarioAttitudeFeedbackRW.py
#
# The simulation layout is shown in the following illustration.  A single simulation process is created
# which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
# modules.
# ![Simulation Flow Diagram](Images/doc/test_scenarioAttitudeFeedbackRW.svg "Illustration")
#
# When the simulation completes several plots are shown for the MRP attitude history, the rate
# tracking errors, as well as the RW motor torque components, as well as the RW wheel speeds.
#
#
# ### Setup Changes to Simulate RW Dynamic Effectors
#
# The fundamental simulation setup is the same as the one used in
# [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback).
# The dynamics simulation is setup using a SpacecraftPlus() module to which an Earth gravity
# effector is attached.  The simple navigation module is still used to output the inertial attitude
# , angular rate, as well as position and velocity message. The simulation is setup to run in a single
# process again.  If the flight algorithms and simulation tasks are to run at different rates, then see
# [scenarioAttitudeFeedback2T.py](@ref scenarioAttitudeFeedback2T) on how to setup a 2 thread simulation.
#
# For the spacecraft simulation side of this script, the new element is adding RW effectors to the
# the rigid spacecraft hub.  The support macro `simIncludeRW.py` provides several convenient tools to facilitate
# this RW setup process.  This script allows the user to readily create RWs from a database of
# public RW specifications, customize them if needed, and add them to the spacecraftPlus() module.
# The specific code required is:
# ~~~~~~~~~~~~~{.py}
#     # Make a fresh RW factory instance, this is critical to run multiple times
#     rwFactory = simIncludeRW.rwFactory()
#
#     # store the RW dynamical model type
#     varRWModel = rwFactory.BalancedWheels
#     if useJitterSimple:
#         varRWModel = rwFactory.JitterSimple
#
#     # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
#     RW1 = rwFactory.create('Honeywell_HR16'
#                            , [1, 0, 0]
#                            , maxMomentum=50.
#                            , Omega=100.                 # RPM
#                            , RWModel= varRWModel
#                            )
#     RW2 = rwFactory.create('Honeywell_HR16'
#                            , [0, 1, 0]
#                            , maxMomentum=50.
#                            , Omega=200.                 # RPM
#                            , RWModel= varRWModel
#                            )
#
#     RW3 = rwFactory.create('Honeywell_HR16'
#                            , [0, 0, 1]
#                            , maxMomentum=50.
#                            , Omega=300.                 # RPM
#                            , rWB_B = [0.5, 0.5, 0.5]    # meters
#                            , RWModel= varRWModel
#                            )
#
#     numRW = rwFactory.getNumOfDevices()
#
#     # create RW object container and tie to spacecraft object
#     rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
#     rwFactory.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)
#
#     # add RW object array to the simulation process
#     scSim.AddModelToTask(simTaskName, rwStateEffector, None, 2)
# ~~~~~~~~~~~~~
# The first task is to create a fresh instance of the RW factory class `rwFactorY()`.  This factory is able
# to create a list of RW devices, and return copies that can easily be manipulated and custumized if needed.
# The next step in this code is to store the correct `RWModel` state.  This can be either a balanced wheel,
# a wheel with a simple jitter model, or a wheel with a fully coupled model.
#
# The next step in this simulation setup is to use create() to include a particular RW devices.
# The `rwFactory()` class contains several
# public specifications of RW devices which can be accessed by specifying the wheel name, `Honeywell_HR16`
# in this case.  The  2nd required argument is the spin axis \f$\hat{\mathbf g}_B\f$.  It is a unit
# vector expressed in the \f$\cal B\f$-frame.  The remaining arguments are all optional.  In this simulation
# each RW is given a different initial RW spin \f$\Omega\f$
# in units of RPMs.  The 3rd RW specifies the body-relative location of the RW center of mass.  The
# other two RWs use a default value which is a zero vector.
# This last position vector is only used if an off-balanced RW device is being modeled.
#
# Each RW device has several default options that can be customized if needed.  For example,
# the `Honeywell_HR16` comes in three different momentum storage configurations.  When calling the
# `create()` command, the desired storage capacity must be specified through the `maxMomentum` argument.
#
# The following table provides a comprehensive list of all the optional arguments of the `create()`
# command.  This table list the arguments, default values, as well as expected units.
#
# Argument      | Units    | Type | Description  | Default
# ------------- | ---------|------|------------- |-------------
# RWModel       | -        | Integer | flag indicating the RW dynamical model.  Options are BalancedWheels, JitterSimple and  JitterFullyCoupled | BalancedWheels
# Omega         | RPM      | Float | initial Wheel speed | 0.0
# maxMomentum   | Nms      | Float | maximum RW angular momentum storage  | 0.0
# useRWfriction | -        | Bool  | flag to turn on RW wheel friction | False
# useMinTorque  | -        | Bool  | flag to turn on a min. RW torque | False
# useMaxTorque  | -        | Bool  | flag to turn on RW torque saturation | True
# linearFrictionRatio | -  | Float | Coulomb static friction value to model stickage, negative values turn off this feature | -1.0 (turned off)
# rWB_B         | m        | 3x1 Float | RW center of mass location relative to B, in \f$\cal B\f$-frame components | [0.0, 0.0, 0.0]
# label         | -        | string | unique device label, must be not exceed 10 characters.  If not provided, the function will autogenerate names using RWi where i is the RW wheel index starting with 1. | RWi
#
# The command `addToSpacecraft()` adds all the created RWs to the spacecraftPlus() module.  The final step
# is as always to add the vector of RW effectors (called `rwStateEffector` above) to the list of simulation
# tasks.  However, note that the dynamic effector should be evaluated before the spacecraftPlus() module,
# which is why it is being added with a higher priority than the `scObject` task.  Generally speaking
# we should have the execution order:
#
#       effectors -> dynamics -> sensors
#
#
# To log the RW information, the following code is used:
# ~~~~~~~~~~~~~~~~~{.py}
#     scSim.TotalSim.logThisMessage(mrpControlConfig.inputRWSpeedsName, samplingTime)
#     rwOutName = ["rw_config_0_data", "rw_config_1_data", "rw_config_2_data"]
#     for item in rwOutName:
#         scSim.TotalSim.logThisMessage(item, samplingTime)
# ~~~~~~~~~~~~~~~~~
# A message is created that stores an array of the \f$\Omega\f$ wheel speeds.  This is logged
# here to be plotted later on.  However, RW specific messages are also being created which
# contain a wealth of information.  Their default naming is automated and shown above.  This
# allows us to log RW specific information such as the actual RW motor torque being applied.
#
# If you want to configure or customize the RWs, the `rwFactor()` class is very convenient. Assume you want
# to override the default value of the maximum RW speed from 6000RPM to 10,000RPM.  After declaring the RW
# and keeping a copy named RW1, `Omega_max` stage is changed using:
# ~~~~~~~~~~~~~{.py}
# RW1.Omega_max = 10000.0*macros.RPM
# ~~~~~~~~~~~~~
# These changes must be made before adding the RWs to the spacecraft.  If the `RW1` handler is not
# stored when the RW is create, any setup RW devices can be recalled through the device label.
# For example, the above modification could also be done with
# ~~~~~~~~~~~~~{.py}
# rwFactory.rwList['RW1'].Omega_max = 10000.0*macros.RPM
# ~~~~~~~~~~~~~
#
# ### Flight Algorithm Changes to Control RWs
#
# The general flight algorithm setup is the same as in the earlier simulation script. Here we
# use again the inertial3D() guidance module, the attTrackingError() module to evaluate the
# tracking error states, and the MRP_Feedback() module to provide the desired \f${\mathbf L}_r\f$
# control torque vector.  In this simulation we want the MRP attitude control module to take
# advantage of the RW spin information.  This is achieved by adding the 2 extra lines:
# ~~~~~~~~~~~~~~~{.py}
#     mrpControlConfig.rwParamsInMsgName = "rwa_config_data_parsed"
#     mrpControlConfig.inputRWSpeedsName = rwStateEffector.OutputDataString
# ~~~~~~~~~~~~~~~
# The first line specifies the RW configuration flight message name, and the second name
# connects the RW speed output message as an input message to this control module.  This simulates
# the RW speed information being sensed and returned to this algorithm.  This message names
# are not provided, then the BSK control modules automatically turn off any RW gyroscopic
# compensation.
#
# Instead of directly simulating this control torque vector, new
# algorithm modules are required to first map \f${\mathbf L}_r\f$ on the set of RW motor torques
# \f$u_B\f$.
# ~~~~~~~~~~~~~~~{.py}
#     # add module that maps the Lr control torque into the RW motor torques
#     rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
#     rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
#     rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
#     scSim.AddModelToTask(simTaskName, rwMotorTorqueWrap, rwMotorTorqueConfig)
#     # Initialize the test module msg names
#     rwMotorTorqueConfig.outputDataName = rwStateEffector.InputCmds
#     rwMotorTorqueConfig.inputVehControlName = mrpControlConfig.outputDataName
#     rwMotorTorqueConfig.rwParamsInMsgName = mrpControlConfig.rwParamsInMsgName
#     # Make the RW control all three body axes
#     controlAxes_B = [
#              1,0,0
#             ,0,1,0
#             ,0,0,1
#         ]
#     rwMotorTorqueConfig.controlAxes_B = controlAxes_B
# ~~~~~~~~~~~~~~~
# Note that the output vector of RW motor torques \f$u_B\f$ is set to connect with
# the RW state effector command input message.  Further, this module inputs the typical
# vehicle configuration message, as well as a message containing the flight algorithm
# information of the RW devices.  This torque mapping module can map the full 3D \f${\mathbf L}_r\f$
# vector onto RW motor torques, or only a subset.  This is specified through the `controlAxes_B` variable
# which specifies up to 3 orthogonal axes to be controlled.  In this simulation the full 3D vector is
# mapped onto motor torques.
#
# The flight algorithm need to know how many RW devices are on the spacecraft and what their
# spin axis \f$\hat{\mathbf g}_B\f$ are.  This is set through a flight software message that is read
# in by flight algorithm modules that need this info.  To write the required flight RW configuration message
# a separate support macros called `fswSetupRW.py`  is used.
# ~~~~~~~~~~~~~~~~{.py}
#     # FSW RW configuration message
#     # use the same RW states in the FSW algorithm as in the simulation
#     fswSetupRW.clearSetup()
#     for key, rw in rwFactory.rwList.iteritems():
#         fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js)
#     fswSetupRW.writeConfigMessage(mrpControlConfig.rwParamsInMsgName, scSim.TotalSim, simProcessName)
# ~~~~~~~~~~~~~~~~
# Again a `clearSetup()` should be called first to clear out any pre-existing RW devices from an
# earlier simulation run.  Next, the script above uses the same RW information as what the simulation
# uses.  In this configuration we are simulating perfect RW device knowledge.  If imperfect RW knowledge
# is to be simulated, then the user would input the desired flight states rather than the true
# simulation states.  The support macro `writeConfigMessage()` creates the required RW flight configuration
# message.
#
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#        True,        # show_plots
#        False,       # useJitterSimple
#        False        # useRWVoltageIO
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The
# default scenario has the RW jitter turned off.  The
# resulting simulation illustrations are shown below.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedbackRW100.svg "MRP history")
# ![RW Motor Torque History](Images/Scenarios/scenarioAttitudeFeedbackRW200.svg "RW motor torque history")
# ![RW Spin History](Images/Scenarios/scenarioAttitudeFeedbackRW300.svg "RW Omega history")
# Note that in the RW motor torque plot both the required control torque \f$\hat u_B\f$ and the true
# motor torque \f$u_B\f$ are shown.  This illustrates that with this maneuver the RW devices are being
# saturated, and the attitude still eventually stabilizes.
#
#
# Setup 2
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#        True,        # show_plots
#        True,        # useJitterSimple
#        False        # useRWVoltageIO
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  Here the simple RW jitter
# model is engaged for each of the RWs.  To turn this on, the command
# ~~~~~~~~~~~~~~{.py}
#     simIncludeRW.options.RWModel = simIncludeRW.JitterSimple
# ~~~~~~~~~~~~~~
# Change this option before the RW is created.  As this is set before any of the RW created in this
# scenario, all the RWs have jitter engaged if this 'useJitterSimple' flag is set. The
# resulting simulation illustrations are shown below.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedbackRW110.svg "MRP history")
# ![RW Motor Torque History](Images/Scenarios/scenarioAttitudeFeedbackRW210.svg "RW motor torque history")
# ![RW Spin History](Images/Scenarios/scenarioAttitudeFeedbackRW310.svg "RW Omega history")
# The impact of the RW jitter is very small, naturally.  The plots for this case look very similar to
# the balanced RW case.  But there is a distinct numerical difference.
#
# Setup 3
# -----
# The second scenario illustrates how to setup RW analog I/O modules.  This is illustrated in the updated
# flow diagram illustration.
# ![Simulation Flow Diagram](Images/doc/test_scenarioAttitudeFeedbackRWc2.svg "Illustration")
#
# On the simulation side, the
# voltage interface is setup by adding
# ~~~~~~~~~~~~~~{.py}
#     rwVoltageIO = rwVoltageInterface.RWVoltageInterface()
#     rwVoltageIO.ModelTag = "rwVoltageInterface"
#
#     # set module parameters(s)
#     rwVoltageIO.voltage2TorqueGain = 0.2/10.  # [Nm/V] conversion gain
#
#     # Add test module to runtime call list
#     scSim.AddModelToTask(simTaskName, rwVoltageIO)
# ~~~~~~~~~~~~~~
# The default interface voltage output name will connect with the default RW
# input message name.  Naturally, these can be customized if needed.  The one parameter
# that must be set is the voltage to torque conversion gain of the electric motor being modeled.
#
# On the FSW side, the commanded motor torques must be directed towards a new module that converts
# the required torque to a commanded voltage.  The first step is to re-direct the
# rWMotorTorque() output to not directly be the input to the RW SIM devices, but rather the
# input to the RW voltage command module:
# ~~~~~~~~~~~~~~{.py}
# rwMotorTorqueConfig.outputDataName = "rw_torque_Lr"
# ~~~~~~~~~~~~~~
# Next, the rwMotorVoltage() module is setup using:
# ~~~~~~~~~~~~~~{.py}
#       fswRWVoltageConfig = rwMotorVoltage.rwMotorVoltageConfig()
#       fswRWVoltageWrap = scSim.setModelDataWrap(fswRWVoltageConfig)
#       fswRWVoltageWrap.ModelTag = "rwMotorVoltage"
#
#       # Add test module to runtime call list
#       scSim.AddModelToTask(simTaskName, fswRWVoltageWrap, fswRWVoltageConfig)
#
#       # Initialize the test module configuration data
#       fswRWVoltageConfig.torqueInMsgName = rwMotorTorqueConfig.outputDataName
#       fswRWVoltageConfig.rwParamsInMsgName = mrpControlConfig.rwParamsInMsgName
#       fswRWVoltageConfig.voltageOutMsgName = rwVoltageIO.rwVoltageInMsgName
#
#       # set module parameters
#       fswRWVoltageConfig.VMin = 0.0  # Volts
#       fswRWVoltageConfig.VMax = 10.0  # Volts
# ~~~~~~~~~~~~~~
# Note that the rwMotorTorque() output is connect here as the input, and the
# SIM RW voltage input name is used as the output name of this FSW module to connect
# the FSW voltage command the the SIM RW voltage motor module.
#
# To run this scenario, modify the bottom of the script to read:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#        True,        # show_plots
#        False,       # useJitterSimple
#        True         # useRWVoltageIO
#        )
# ~~~~~~~~~~~~~
# The resulting simulation illustrations are shown below.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedbackRW101.svg "MRP history")
# ![RW Motor Torque History](Images/Scenarios/scenarioAttitudeFeedbackRW201.svg "RW motor torque history")
# ![RW Spin History](Images/Scenarios/scenarioAttitudeFeedbackRW301.svg "RW Omega history")
# ![RW Voltage History](Images/Scenarios/scenarioAttitudeFeedbackRW401.svg "RW Voltage history")
# Note that the rwMotorVoltage() module is run here in a simple open-loop manner.  See the
# [rwMotorVoltage documentation](../fswAlgorithms/effectorInterfaces/rwMotorVoltage/_Documentation/Basilisk-rwMotorVoltage-20170113.pdf "PDF Doc")
# for more info.  By connecting the RW availability message it is possible turn
# off the voltage command for particular wheels.  Also, by specifying the RW speed message input
# name it is possible to turn on a torque tracking feedback loop in this module.
## @}
def run(show_plots, useJitterSimple, useRWVoltageIO):
    '''Call this routine directly to run the tutorial scenario.'''

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # unitTestSupport.enableVisualization(scSim, dynProcess, simProcessName, 'earth')
    # The Viz only support 'earth', 'mars', or 'sun'

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, None, 1)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = rwFactory.BalancedWheels
    if useJitterSimple:
        varRWModel = rwFactory.JitterSimple

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
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.InputCmds = "reactionwheel_cmds"
    rwFactory.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(simTaskName, rwStateEffector, None, 2)

    if useRWVoltageIO:
        rwVoltageIO = rwVoltageInterface.RWVoltageInterface()
        rwVoltageIO.ModelTag = "rwVoltageInterface"

        # set module parameters(s)
        rwVoltageIO.setGains(np.array([0.2 / 10.] * 3)) # [Nm/V] conversion gain

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, rwVoltageIO)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simple_nav.SimpleNav()
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
    inertial3DConfig.outputDataName = "guidanceInertial3D"

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.outputDataName = "attErrorInertial3DMsg"
    attErrorConfig.inputRefName = inertial3DConfig.outputDataName
    attErrorConfig.inputNavName = sNavObject.outputAttName

    # setup the MRP Feedback control module
    mrpControlConfig = MRP_Feedback.MRP_FeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.inputGuidName = attErrorConfig.outputDataName
    mrpControlConfig.vehConfigInMsgName = "vehicleConfigName"
    mrpControlConfig.outputDataName = "LrRequested"
    mrpControlConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    mrpControlConfig.inputRWSpeedsName = rwStateEffector.OutputDataString
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1
    mrpControlConfig.domega0 = [0.0, 0.0, 0.0]

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
    rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
    rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueWrap, rwMotorTorqueConfig)
    # Initialize the test module msg names
    if useRWVoltageIO:
        rwMotorTorqueConfig.outputDataName = "rw_torque_Lr"
    else:
        rwMotorTorqueConfig.outputDataName = rwStateEffector.InputCmds
    rwMotorTorqueConfig.inputVehControlName = mrpControlConfig.outputDataName
    rwMotorTorqueConfig.rwParamsInMsgName = mrpControlConfig.rwParamsInMsgName
    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    rwMotorTorqueConfig.controlAxes_B = controlAxes_B

    if useRWVoltageIO:
        fswRWVoltageConfig = rwMotorVoltage.rwMotorVoltageConfig()
        fswRWVoltageWrap = scSim.setModelDataWrap(fswRWVoltageConfig)
        fswRWVoltageWrap.ModelTag = "rwMotorVoltage"

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, fswRWVoltageWrap, fswRWVoltageConfig)

        # Initialize the test module configuration data
        fswRWVoltageConfig.torqueInMsgName = rwMotorTorqueConfig.outputDataName
        fswRWVoltageConfig.rwParamsInMsgName = mrpControlConfig.rwParamsInMsgName
        fswRWVoltageConfig.voltageOutMsgName = rwVoltageIO.rwVoltageInMsgName

        # set module parameters
        fswRWVoltageConfig.VMin = 0.0  # Volts
        fswRWVoltageConfig.VMax = 10.0  # Volts

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(rwMotorTorqueConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)
    scSim.TotalSim.logThisMessage(mrpControlConfig.inputRWSpeedsName, samplingTime)
    rwOutName = ["rw_config_0_data", "rw_config_1_data", "rw_config_2_data"]
    for item in rwOutName:
        scSim.TotalSim.logThisMessage(item, samplingTime)
    if useRWVoltageIO:
        scSim.TotalSim.logThisMessage(fswRWVoltageConfig.voltageOutMsgName, samplingTime)

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               mrpControlConfig.vehConfigInMsgName,
                               vehicleConfigOut)

    # FSW RW configuration message
    # use the same RW states in the FSW algorithm as in the simulation
    fswSetupRW.clearSetup()
    for key, rw in rwFactory.rwList.iteritems():
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    fswSetupRW.writeConfigMessage(mrpControlConfig.rwParamsInMsgName, scSim.TotalSim, simProcessName)

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
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataUsReq = scSim.pullMessageLogData(rwMotorTorqueConfig.outputDataName + ".motorTorque", range(numRW))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".sigma_BR", range(3))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".omega_BR_B", range(3))
    dataPos = scSim.pullMessageLogData(sNavObject.outputTransName + ".r_BN_N", range(3))
    dataOmegaRW = scSim.pullMessageLogData(mrpControlConfig.inputRWSpeedsName + ".wheelSpeeds", range(numRW))
    dataRW = []
    for i in range(0, numRW):
        dataRW.append(scSim.pullMessageLogData(rwOutName[i] + ".u_current", range(1)))
    if useRWVoltageIO:
        dataVolt = scSim.pullMessageLogData(fswRWVoltageConfig.voltageOutMsgName + ".voltage", range(numRW))
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileName = os.path.basename(os.path.splitext(__file__)[0])

    timeData = dataUsReq[:, 0] * macros.NANO2MIN
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

    return dataPos, dataSigmaBR, dataUsReq, numDataPoints, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False,  # useJitterSimple
        False  # useRWVoltageIO
    )
