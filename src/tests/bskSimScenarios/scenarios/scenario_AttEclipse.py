''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
## \defgroup Tutorials_2_2_1
## @{
# Demonstrates how to sun safe pointing in conjunction with the Eclipse, RW, and CSS modules to stabilize the tumble of
# a spacecraft passing through an eclipse while orbiting the Earth.
#
# Attitude Guidance Eclipse Simulation using RW Effectors {#scenario_AttitudeEclipse}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The goal is to
# illustrate 1) how Reaction Wheel (RW) state effector be added to the rigid
# spacecraftPlus() hub, 2) how Coarse Sun Sensor (CSS) constellations and IMU sensors can be added
# to the rigid body spacecraftPlus() hub, 3) how the Eclipse module can be used in conjunction with
# gravityEffectors to cast shadows over the CSS constellation, and 4) what flight algorithm modules
# are used to control these RWs in these circumstances.
#
#
# To run the default scenario, call the python script from a Terminal window through
#
#       python scenario_AttitudeEclipse.py
#
# The simulation layout is shown in the following illustration.  A single simulation process is created
# which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
# modules.
# ![Simulation Flow Diagram](Images/doc/test_scenarioAttitudeFeedbackRW.svg "Illustration")
#
# When the simulation completes several plots are shown for the spacecraft position, MRP attitude history, the sun pointing
# vector, as well as the eclipse shadow factor.
#
#
# ### Setup Changes to Simulate RW Dynamic Effectors
#
# The fundamental simulation setup is the same as the one used in
# [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback).
# The dynamics simulation is setup using a SpacecraftPlus() module to which an Earth gravity
# effector is attached. The simulation is setup to run in a two simultaenous processes.
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
#     # specify RW momentum capacity
#     maxRWMomentum = 50. # Nms
#
#     # Define orthogonal RW pyramid
#     # -- Pointing directions
#     rwElAngle = np.array([40.0, 40.0, 40.0, 40.0])*mc.D2R
#     rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0])*mc.D2R
#     rwPosVector = [[0.8, 0.8, 1.79070],
#                    [0.8, -0.8, 1.79070],
#                    [-0.8, -0.8, 1.79070],
#                    [-0.8, 0.8, 1.79070]
#                    ]
#
#      for elAngle, azAngle, posVector in zip(rwElAngle, rwAzimuthAngle, rwPosVector):
#          gsHat = (rbk.Mi(-azAngle,3).dot(rbk.Mi(elAngle,2))).dot(np.array([1,0,0]))
#          rwFactory.create('Honeywell_HR16',
#                          gsHat,
#                          maxMomentum=maxRWMomentum,
#                          rWB_B=posVector)
#
#     rwFactory.addToSpacecraft("RWStateEffector", self.rwStateEffector, self.scObject)
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


# Import utilities
from Basilisk.utilities import orbitalMotion, macros, unitTestSupport

# Get current file path
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario

# Import plotting file for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

sys.path.append(path + '/../../scenarios')
import scenarioAttitudeFeedbackRW as scene_plt

# Create your own scenario child class
class scenario_AttitudeEclipse(BSKScenario):
    def __init__(self, masterSim):
        super(scenario_AttitudeEclipse, self).__init__(masterSim)
        self.name = 'scenario_AttitudeEclipse'
        self.masterSim = masterSim

    def configure_initial_conditions(self):
        print '%s: configure_initial_conditions' % self.name
        # Configure FSW mode
        self.masterSim.modeRequest = 'sunSafePoint'

        # Configure Dynamics initial conditions

        oe = orbitalMotion.ClassicElements()
        oe.a = 7000000.0  # meters
        oe.e = 0.0
        oe.i = 0.0 #33.3 * macros.D2R
        oe.Omega = 0.0# 48.2 * macros.D2R
        oe.omega = 0.0# 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        mu = self.masterSim.DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        self.masterSim.DynModels.scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
        self.masterSim.DynModels.scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
        self.masterSim.DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        self.masterSim.DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B


    def log_outputs(self):
        print '%s: log_outputs' % self.name
        samplingTime = self.masterSim.DynModels.processTasksTimeStep

        # Dynamics process outputs: log messages below if desired.
        self.masterSim.TotalSim.logThisMessage(self.masterSim.DynModels.scObject.scStateOutMsgName, samplingTime)
        self.masterSim.TotalSim.logThisMessage("eclipse_data_0", samplingTime)

        # FSW process outputs
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.mrpFeedbackRWsData.inputRWSpeedsName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.rwMotorTorqueData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.trackingErrorData.outputDataName, samplingTime)
        self.masterSim.TotalSim.logThisMessage(self.masterSim.FSWModels.sunSafePointData.sunDirectionInMsgName, samplingTime)
        return

    def pull_outputs(self):
        print '%s: pull_outputs' % self.name
        num_RW = 4 # number of wheels used in the scenario

        # Dynamics process outputs: pull log messages below if any
        r_BN_N = self.masterSim.pullMessageLogData(self.masterSim.DynModels.scObject.scStateOutMsgName + ".r_BN_N", range(3))
        shadowFactor = self.masterSim.pullMessageLogData("eclipse_data_0.shadowFactor")

        # FSW process outputs
        dataUsReq = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.rwMotorTorqueData.outputDataName + ".motorTorque", range(num_RW))
        sigma_BR = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.trackingErrorData.outputDataName + ".sigma_BR", range(3))
        omega_BR_B = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
        RW_speeds = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.mrpFeedbackRWsData.inputRWSpeedsName + ".wheelSpeeds", range(num_RW))
        sunPoint = self.masterSim.pullMessageLogData(
            self.masterSim.FSWModels.sunSafePointData.sunDirectionInMsgName + ".vehSunPntBdy", range(3))

        # Plot results
        timeData = dataUsReq[:, 0] * macros.NANO2MIN
        #scene_plt.plot_attitude_error(timeData, sigma_BR)
        #scene_plt.plot_rw_cmd_torque(timeData, dataUsReq, num_RW)
        #scene_plt.plot_rate_error(timeData, omega_BR_B)
        #scene_plt.plot_rw_speeds(timeData, RW_speeds, num_RW)

        BSK_plt.plot_shadow_factor(timeData, shadowFactor)
        BSK_plt.plot_orbit(r_BN_N)
        BSK_plt.plot_sun_point(timeData, sunPoint)
        #BSK_plt.plot3components(r_BN_N)
        BSK_plt.show_all_plots()



def run(showPlots):
    # Instantiate base simulation

    '''
    CURRENT STATE, THE SPICE DATA MODULE DOES WELL BUT HAS NO ECLIPSE FUNCTIONALITY

    THE SPICE OBJECT ALSO RUNS, BUT THERE IS NO SPACECRAFT ORBIT ANY LONGER SO...THAT NEEDS TO BE FIXED
    '''
    TheBSKSim = BSKSim()

    # Configure an scenario in the base simulation
    TheScenario = scenario_AttitudeEclipse(TheBSKSim)
    TheScenario.log_outputs()
    TheScenario.configure_initial_conditions()

    # Initialize simulation
    TheBSKSim.InitializeSimulationAndDiscover()

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(60.0)
    TheBSKSim.ConfigureStopTime(simulationTime)
    print 'Starting Execution'
    TheBSKSim.ExecuteSimulation()
    print 'Finished Execution. Post-processing results'

    # Pull the results of the base simulation running the chosen scenario
    if showPlots:
        TheScenario.pull_outputs()


if __name__ == "__main__":
    run(True)
