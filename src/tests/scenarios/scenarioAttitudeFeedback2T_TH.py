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
# Purpose:  Integrated test of the spacecraftPlus(), extForceTorque, simpleNav(), thrusterDynamicEffector() and
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit, while using thrusters
#           to do the attitude control actuation.
# Author: Giulio Napolitano
# Creation Date:  June 26, 2019
#

import sys
import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import sim_model
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import simIncludeThruster

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simple_nav
from Basilisk.simulation import thrusterDynamicEffector

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import MRP_Feedback
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.fswAlgorithms import thrFiringSchmitt

# import message declarations
from Basilisk.fswAlgorithms import fswMessages

# attempt to import vizard
from Basilisk.utilities import vizSupport

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

# Plotting functions
def plot_attitude_error(timeDataFSW, dataSigmaBR):
    plt.figure(1)
    for idx in range(1, 4):
        plt.plot(timeDataFSW, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error $\sigma_{B/R}$')

def plot_rate_error(timeDataFSW, dataOmegaBR):
    plt.figure(2)
    for idx in range(1, 4):
        plt.plot(timeDataFSW, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

def plot_requested_torque(timeDataFSW, dataLr):
    plt.figure(3)
    for idx in range(1, 4):
        plt.plot(timeDataFSW, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')

def plot_thrForce(timeDataFSW, dataMap, numTh):
    plt.figure(4)
    for idx in range(1, numTh + 1):
        plt.plot(timeDataFSW, dataMap[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label='$thrForce,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Force requested [N]')

def plot_OnTimeRequest(timeDataFSW, dataSchm, numTh):
    plt.figure(5)
    for idx in range(1, numTh + 1):
        plt.plot(timeDataFSW, dataSchm[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label='$OnTimeRequest,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('OnTimeRequest [sec]')

## \defgroup scenarioAttitudeFeedback2T_THGroup
## @{
# Demonstrates how to use thrusters to stabilize the tumble of a spacecraft orbiting the
# Earth, using two separate threads.
#
# Attitude Detumbling Simulation using Thruster Effectors in a Two Process Simulation Setup  {#scenarioAttitudeFeedback2T_TH}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The goal is to
# illustrate how a set of thrusters can be added to the rigid SpacecraftPlus() hub, and what
# FSW modules are needed to control these thrusters. The simulation setup is performed with two
# processes, similarly to [scenarioAttitudeFeedback2T.py](@ref scenarioAttitudeFeedback2T),
# in which the dynamics and the FSW algorithms are run at different time steps.  The control setup is the same
# as in [scenarioAttitudeFeedbackRW.py](@ref scenarioAttitudeFeedbackRW), but here the RW actuation is replaced with
# thruster based control torque solution.
#
# To run the scenario, call the python script from a Terminal window through:
#
#       python scenarioAttitudeFeedback2T_TH.py
#
# The simulation layout is shown in the following illustration. The two processes (SIM and FSW) are simulated
# and run at different time rates. Interface messages are shared across SIM and
# FSW message passing interfaces (MPIs).
# ![Simulation Flow Diagram](Images/doc/test_scenarioAttitudeFeedback2T_TH.svg "Illustration")
#
# When the simulation completes several plots are shown for the MRP attitude history, the rate
# tracking errors, the requested torque, the requested forces for each thruster and the On-Time commands.
#
#
# ### Setup Changes to Simulate Thrusters Dynamic Effectors
#
# At the beginning of the script all the plot functions are declared. Then the fundamental simulation setup is the same
# as the one used in [scenarioAttitudeFeedback2T.py](@ref scenarioAttitudeFeedback2T).
# The dynamics simulation is setup using a SpacecraftPlus() module to which an Earth gravity
# effector is attached.  The simple navigation module is still used to output the inertial attitude,
# angular rate, as well as position and velocity messages.
#
# The Thruster Dynamic Effector is added to the the rigid spacecraft hub, similarly to
# [scenarioAttitudeFeedbackRW.py](@ref scenarioAttitudeFeedbackRW).  The support macro `simIncludeThruster.py`
# provides several convenient tools to facilitate the setup process.  This script allows the user to
# readily create thrusters from a database of public specifications, customize them if needed, and add
# them to the SpacecraftPlus() module.
# The specific code required is:
# ~~~~~~~~~~~~~{.py}
#     # create the set of thrusters in the dynamics task
#     thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
#     scSim.AddModelToTask(dynTaskName, thrusterSet)
#
#     # Make a fresh thruster factory instance, this is critical to run multiple times
#     thFactory = simIncludeThruster.thrusterFactory()
#
#     # create the thruster devices by specifying the thruster type and its location and direction
#     for pos_B, dir_B in zip(location, direction):
#
#         if useDVThrusters:
#             thFactory.create('MOOG_Monarc_22_6', pos_B, dir_B)
#         else:
#             thFactory.create('MOOG_Monarc_1', pos_B, dir_B)
#
#     # get number of thruster devices
#     numTh = thFactory.getNumOfDevices()
#
#     # create thruster object container and tie to spacecraft object
#     thFactory.addToSpacecraft("ACSThrusterDynamics", thrusterSet, scObject)
# ~~~~~~~~~~~~~
#
# The first thing to do is to create the (empty) set of thrusters that will later contain all the devices. Then
# a fresh instance of the thruster factory class `thrusterFactory()` is created.  This factory is able
# to create a list of thruster devices, and return copies that can easily be manipulated and customized if needed.
# The next step in this simulation setup is to use `create()` to include a particular thruster device.
# The `thrusterFactory()` class contains several
# public specifications of thruster devices which can be accessed by specifying their name. In our case we will consider
# `MOOG_Monarc_1` for the ACS Thrusters configuration, and `MOOG_Monarc_22_6` for the DV Thrusters one.
# The  2nd and 3rd required arguments are respectively the location of the
# thruster \f$\hat{\mathbf r}\f$ and the direction of its force \f$\hat{\mathbf g}_t\f$.  Both
# vectors are expressed in the \f$\cal B\f$-frame.  The remaining arguments are all optional.
# The thrusters are generated by using the `create()` command inside a 'for' loop, which has the
# job of assigning the respective location and direction arguments to each thruster, by cycling through the two
# pre-defined arrays 'location' and 'direction'.
#
# The following table provides a comprehensive list of all the optional arguments of the `create()`
# command.  This table list the arguments, default values, as well as expected units.
#
# Argument         | Units    | Type  | Description  | Default
# -------------    | ---------|------ |------------- |-------------
# useMinPulseTime  | -        | Bool  | flag if the thruster model should use a minimum impulse time | False
# areaNozzle       | m^2      | Float | thruster nozzle exhaust cone exit area | 0.1
# steadyIsp        | s        | Float | thruster fuel efficiency in Isp  | 100.0
# MaxThrust        | N        | Float | maximum thruster force | 0.200
# thrusterMagDisp  | %        | Float | thruster dispersion percentage | 0.0
# MinOnTime        | s        | Float | thruster minimum on time | 0.020
#
# The command `addToSpacecraft()` adds all the created thrusters to the `spacecraftPlus()` module.  The final step
# is to add the `thrusterDynamicEffector` to the list of simulation tasks.
#
# To log the information requred for the plots, the following code is used:
# ~~~~~~~~~~~~~~~~~{.py}
#     numDataPoints = 100
#     samplingTime = simulationTime / (numDataPoints - 1)
#     scSim.TotalSim.logThisMessage(mrpControlConfig.outputDataName, samplingTime)
#     scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
#     scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)
#     scSim.TotalSim.logThisMessage(sNavObject.outputAttName, samplingTime)
#     scSim.TotalSim.logThisMessage(thrForceMappingConfig.outputDataName, samplingTime)
#     scSim.TotalSim.logThisMessage(thrFiringSchmittConfig.onTimeOutMsgName, samplingTime)
# ~~~~~~~~~~~~~~~~~
#
# ### Flight Algorithm Changes to Control Thrusters
#
# The general flight algorithm setup is the same as in the earlier simulation scripts. Here we
# use again the `inertial3D()` guidance module, the `attTrackingError()` module to evaluate the
# tracking error states, and the `MRP_Feedback()` module to provide the desired \f${\mathbf L}_r\f$
# control torque vector.  In addition, this time, we have to add two more modules: `thrForceMapping()`
# and `thrFiringSchmitt()`.
#
# ~~~~~~~~~~~~~~~{.py}
#     # setup the thruster force mapping module
#     thrForceMappingConfig = thrForceMapping.thrForceMappingConfig()
#     thrForceMappingWrap = scSim.setModelDataWrap(thrForceMappingConfig)
#     thrForceMappingWrap.ModelTag = "thrForceMapping"
#     scSim.AddModelToTask(fswTaskName, thrForceMappingWrap, thrForceMappingConfig)
#     thrForceMappingConfig.inputVehControlName = mrpControlConfig.outputDataName
#     thrForceMappingConfig.inputThrusterConfName = "ThrustersConfig"
#     thrForceMappingConfig.outputDataName = "Lr_command"
#     thrForceMappingConfig.inputVehicleConfigDataName = mrpControlConfig.vehConfigInMsgName
#     if useDVThrusters:
#         controlAxes_B = [1, 0, 0,
#                          0, 1, 0]
#         thrForceMappingConfig.thrForceSign = -1
#     else:
#         controlAxes_B = [1, 0, 0,
#                          0, 1, 0,
#                          0, 0, 1]
#         thrForceMappingConfig.thrForceSign = +1
#     thrForceMappingConfig.controlAxes_B = controlAxes_B
# ~~~~~~~~~~~~~~~
#
# The `thrForceMapping()` module takes a commanded attitude control torque vector and determines a set of desired
# thruster force values to implement this torque. It is assumed that the nominal thruster configuration is such that
# pure torque solutions are possible. The module supports both on- and off-pulsing solutions, including
# cases where the thruster solutions are saturated due to a large commanded attitude control torque.
# The module set up is done in an analogous way as the previous ones. It can be noted that one of the inputs
# corresponds to the output of the `MRP_Feedback()`, being the commanded control torque. The other ones are the
# information on the thrusters configuration and spacecraft inertia, whose messages will be created later in the script.
# In addition, the control axes are specified using the full identity matrix for ACS thrusters, and its first
# two rows for the DV ones, since in the latter case we are not able to control one axis (z in our case),
# due to the geometrical configuration. The last value to specify is the `thrForceSign`, which will have the
# value of -1 if off-pulsing DV thrusters are employed, and +1 with the on-pulsing ACS configuration.
#
# ~~~~~~~~~~~~~~~{.py}
#     # setup the Schmitt trigger thruster firing logic module
#     thrFiringSchmittConfig = thrFiringSchmitt.thrFiringSchmittConfig()
#     thrFiringSchmittWrap = scSim.setModelDataWrap(thrFiringSchmittConfig)
#     thrFiringSchmittWrap.ModelTag = "thrFiringSchmitt"
#     scSim.AddModelToTask(fswTaskName, thrFiringSchmittWrap, thrFiringSchmittConfig)
#     thrFiringSchmittConfig.thrMinFireTime = 0.002
#     thrFiringSchmittConfig.level_on = .75
#     thrFiringSchmittConfig.level_off = .25
#     if useDVThrusters:
#         thrFiringSchmittConfig.baseThrustState = 1
#     thrFiringSchmittConfig.thrConfInMsgName = "ThrustersConfig"
#     thrFiringSchmittConfig.thrForceInMsgName = thrForceMappingConfig.outputDataName
#     thrFiringSchmittConfig.onTimeOutMsgName = "acs_thruster_cmds"
# ~~~~~~~~~~~~~~~
#
# The last needed FSW module is `thrFiringSchmitt()`. A Schmitt trigger logic is implemented to map a desired
# thruster force value into a thruster on command time. The module reads in the attitude control thruster
# force values for both on- and off-pulsing scenarios, and then maps this into a time which specifies how
# long a thruster should be on. Four values are specified: `thrMinFireTime` (minimum thruster on-time in seconds),
# `level_on` (Upper duty cycle percentage threshold relative to t min to turn on thrusters), `level_off`
# (upper duty cycle percentage threshold relative to t min to turn on thrusters), and `baseThrustState` (0 by default
# and set to 1 for DV thrusters). As expected, the thrusters force
# input is directly the output of `thrForceMapping`, and also in this case we will need the thrusters configuration
# message. It can be noted how the output of this module ends up to be the input commands for the
# `thrustersDynamicEffector()`, by using the default message name "acs_thruster_cmds".
#
# The flight algorithm needs to know how many thruster devices are on the spacecraft and what their
# location and direction are.  This is set through a flight software message that is read
# in by flight algorithm modules that need this info.  To write the required flight thrusters configuration message
# a separate support macros called `fswSetupThrusters.py` is used.
# ~~~~~~~~~~~~~~~~{.py}
#     # create the FSW Thruster configuration message
#     if useDVThrusters:
#         maxThrust = 22
#     else:
#         maxThrust = 1
#
#     fswSetupThrusters.clearSetup()
#     for pos_B, dir_B in zip(location, direction):
#         fswSetupThrusters.create(pos_B, dir_B, maxThrust)
#     fswSetupThrusters.writeConfigMessage("ThrustersConfig", scSim.TotalSim, fswProcessName)
# ~~~~~~~~~~~~~~~~
# A `clearSetup()` should be called first to clear out any pre-existing devices from an
# earlier simulation run.  Next, the `maxThrust` value should be specified and used in the macro `create()`,
# together with the locations and directions, and looped through a for cycle to consider all the thrusters.
# The support macro `writeConfigMessage()` creates the required thrusters flight configuration message.
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
#        False,       # useDVThrusters
#        )
# ~~~~~~~~~~~~~
# The second argument control whether the ACS Thrusters or DV Thrusters are implemented.  The default scenario
# has the 8 ACS Thrusters.  The resulting simulation illustrations are shown below.
# ![Attitude Error History](Images/Scenarios/scenarioAttitudeFeedback2T_TH10.svg "Attitude Error History")
# ![Rate Error History](Images/Scenarios/scenarioAttitudeFeedback2T_TH20.svg "Rate Error History")
# ![Requested Torque History](Images/Scenarios/scenarioAttitudeFeedback2T_TH30.svg "Requested Torque History")
# ![Requested Force for each Thruster](Images/Scenarios/scenarioAttitudeFeedback2T_TH40.svg "Requested Force for each Thruster")
# ![On Time Commands] (Images/Scenarios/scenarioAttitudeFeedback2T_TH50.svg "On Time Commands")
# By looking at the plots we can see how every axis is controlled, and the detumbling action is perfectly performed.
# We can also see how the requested force for each thruster (third plot) never reaches the 1 N limit (apart from during
# the initial transitory), which means that they are never saturated.
#
# Setup 2
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#        True,        # show_plots
#        True,       # useDVThrusters
#        )
# ~~~~~~~~~~~~~
# In this setup we use the 6 DV Thrusters configuration. In this case, given the spacial configuration of the thrusters,
# it is impossible to control the third axis, so the result is a spacecraft attitude in which the x, y axes are
# controlled, but with a tumbling motion in the z axis. The resulting simulation illustrations are shown below.
# ![Attitude Error History](Images/Scenarios/scenarioAttitudeFeedback2T_TH11.svg "Attitude Error History")
# ![Rate Error History](Images/Scenarios/scenarioAttitudeFeedback2T_TH21.svg "Rate Error History")
# ![Requested Torque History](Images/Scenarios/scenarioAttitudeFeedback2T_TH31.svg "Requested Torque History")
# ![Requested Force for each Thruster](Images/Scenarios/scenarioAttitudeFeedback2T_TH41.svg "Requested Force for each
#  Thruster")
# ![On Time Commands] (Images/Scenarios/scenarioAttitudeFeedback2T_TH51.svg "On Time Commands")
# In this setup it can be clearly seen how the control action is performed just on the x and y axes, leaving the
# spacecraft tumbling around the z one. Another important remark is that, since the default state of the DV thrusters
# is 'on', the requested thruster force is always negative, as it can be seen in the plot.
## @}

def run(show_plots, useDVThrusters):
    '''Call this routine directly to run the tutorial scenario.'''

    # Create simulation variable names
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(dynProcessName)
    fswProcess = scSim.CreateNewProcess(fswProcessName)

    # Process message interfaces.
    # this step is used to copy messages between the dyn and fsw processes
    # as long as the message has the same name, it will get copied over automatically
    dyn2FSWInterface = sim_model.SysInterface()
    fsw2DynInterface = sim_model.SysInterface()

    dyn2FSWInterface.addNewInterface(dynProcessName, fswProcessName)
    fsw2DynInterface.addNewInterface(fswProcessName, dynProcessName)

    fswProcess.addInterfaceRef(dyn2FSWInterface)
    dynProcess.addInterfaceRef(fsw2DynInterface)

    # create the dynamics task and specify the integration update time
    simTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simTimeStep))
    fswTimeStep = macros.sec2nano(0.5)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

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
    scSim.AddModelToTask(dynTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(dynTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simple_nav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTaskName, sNavObject)
    sNavObject.outputAttName = "SpacecraftAttitude"

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
    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    scSim.AddModelToTask(dynTaskName, thrusterSet)

    # Make a fresh thruster factory instance, this is critical to run multiple times
    thFactory = simIncludeThruster.thrusterFactory()

    # create the thruster devices by specifying the thruster type and its location and direction
    for pos_B, dir_B in zip(location, direction):

        if useDVThrusters:
            thFactory.create('MOOG_Monarc_22_6', pos_B, dir_B)
        else:
            thFactory.create('MOOG_Monarc_1', pos_B, dir_B)

    # get number of thruster devices
    numTh = thFactory.getNumOfDevices()

    # create thruster object container and tie to spacecraft object
    thrModelTag = "ACSThrusterDynamics"
    thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation
    scSim.AddModelToTask(fswTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.outputDataName = "guidanceInertial3D"

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(fswTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.outputDataName = "attErrorInertial3DMsg"
    attErrorConfig.inputRefName = inertial3DConfig.outputDataName
    attErrorConfig.inputNavName = sNavObject.outputAttName

    # setup the MRP Feedback control module
    mrpControlConfig = MRP_Feedback.MRP_FeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(fswTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.inputGuidName = attErrorConfig.outputDataName
    mrpControlConfig.vehConfigInMsgName = "vehicleConfigName"
    mrpControlConfig.outputDataName = "Lr_requested"
    mrpControlConfig.K = 3.5*10.0
    mrpControlConfig.Ki = 0.0002  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0*10.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # setup the thruster force mapping module
    thrForceMappingConfig = thrForceMapping.thrForceMappingConfig()
    thrForceMappingWrap = scSim.setModelDataWrap(thrForceMappingConfig)
    thrForceMappingWrap.ModelTag = "thrForceMapping"
    scSim.AddModelToTask(fswTaskName, thrForceMappingWrap, thrForceMappingConfig)
    thrForceMappingConfig.inputVehControlName = mrpControlConfig.outputDataName
    thrForceMappingConfig.inputThrusterConfName = "ThrustersConfig"
    thrForceMappingConfig.outputDataName = "Lr_command"
    thrForceMappingConfig.inputVehicleConfigDataName = mrpControlConfig.vehConfigInMsgName
    if useDVThrusters:
        controlAxes_B = [1, 0, 0,
                         0, 1, 0]
        thrForceMappingConfig.thrForceSign = -1
    else:
        controlAxes_B = [1, 0, 0,
                         0, 1, 0,
                         0, 0, 1]
        thrForceMappingConfig.thrForceSign = +1
    thrForceMappingConfig.controlAxes_B = controlAxes_B

    # setup the Schmitt trigger thruster firing logic module
    thrFiringSchmittConfig = thrFiringSchmitt.thrFiringSchmittConfig()
    thrFiringSchmittWrap = scSim.setModelDataWrap(thrFiringSchmittConfig)
    thrFiringSchmittWrap.ModelTag = "thrFiringSchmitt"
    scSim.AddModelToTask(fswTaskName, thrFiringSchmittWrap, thrFiringSchmittConfig)
    thrFiringSchmittConfig.thrMinFireTime = 0.002
    thrFiringSchmittConfig.level_on = .75
    thrFiringSchmittConfig.level_off = .25
    if useDVThrusters:
        thrFiringSchmittConfig.baseThrustState = 1
    thrFiringSchmittConfig.thrConfInMsgName = "ThrustersConfig"
    thrFiringSchmittConfig.thrForceInMsgName = thrForceMappingConfig.outputDataName
    thrFiringSchmittConfig.onTimeOutMsgName = "acs_thruster_cmds"

    #
    #   Setup data logging before the simulation is initialized
    #

    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(mrpControlConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputAttName, samplingTime)
    scSim.TotalSim.logThisMessage(thrForceMappingConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(thrFiringSchmittConfig.onTimeOutMsgName, samplingTime)


    #
    # create FSW simulation messages
    #

    # create the FSW vehicle configuration message

    vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    unitTestSupport.setMessage(scSim.TotalSim,
                               fswProcessName,
                               mrpControlConfig.vehConfigInMsgName,
                               vehicleConfigOut)

    # create the FSW Thruster configuration message
    if useDVThrusters:
        maxThrust = 22
    else:
        maxThrust = 1

    fswSetupThrusters.clearSetup()
    for pos_B, dir_B in zip(location, direction):
        fswSetupThrusters.create(pos_B, dir_B, maxThrust)
    fswSetupThrusters.writeConfigMessage("ThrustersConfig", scSim.TotalSim, fswProcessName)

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
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # vizSupport.enableUnityVisualization(scSim, dynTaskName,  dynProcessName, gravBodies = gravFactory, saveFile=fileName, thrDevices=[(thFactory.getNumOfDevices(), thrModelTag)])

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    # this next call ensures that the FSW and Dynamics Message that have the same
    # name are copied over every time the simulation ticks forward.  This function
    # has to be called after the simulation is initialized to ensure that all modules
    # have created their own output/input messages declarations.
    # dyn2FSWInterface.discoverAllMessages()
    # fsw2DynInterface.discoverAllMessages()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataLr = scSim.pullMessageLogData(mrpControlConfig.outputDataName + ".torqueRequestBody", list(range(3)))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".sigma_BR", list(range(3)))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".omega_BR_B", list(range(3)))
    dataMap = scSim.pullMessageLogData(thrForceMappingConfig.outputDataName + ".thrForce", list(range(numTh)))
    dataSchm = scSim.pullMessageLogData(thrFiringSchmittConfig.onTimeOutMsgName + ".OnTimeRequest", list(range(numTh)))
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    timeDataFSW = dataLr[:, 0] * macros.NANO2MIN
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

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return numDataPoints, figureList

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False,  # useDVThrusters
    )