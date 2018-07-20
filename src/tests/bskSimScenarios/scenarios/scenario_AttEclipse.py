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
# Demonstrates how to sun safe pointing in conjunction with the Eclipse, RW, CSS Weighted Least Squares Estimator and
# CSS modules to stabilize the tumble of a spacecraft passing through an eclipse while orbiting the Earth.
#
# Sun Safe Pointing Attitude Guidance Simulation with Eclipse Module {#scenario_AttitudeEclipse}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth.  The goal is to
# illustrate 1) how Reaction Wheel (RW) state effector be added to the rigid
# spacecraftPlus() hub, 2) how Coarse Sun Sensor (CSS) constellations and IMU sensors can be added
# to the rigid body spacecraftPlus() hub, 3) how the eclipse module can be used in conjunction with
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
# When the simulation completes several plots are shown for the spacecraft position, MRP attitude history, the sun
# pointing vector, as well as the eclipse shadow factor.
#
#
# ### Setup Changes for Spacecraft Dynamics
#
# The fundamental simulation setup is a combination of the setups used in
# [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback) and [scenarioCSS.py](@ref scenarioCSS).
# The dynamics simulation is setup using a SpacecraftPlus() module to which an Earth gravity
# effector is attached.
#
# For the spacecraft simulation side of this script, the new element is adding the IMU sensor to the spacecraft, and
# the eclipse module to the simulation environment. The specific code required to build the IMU sensor is:
# ~~~~~~~~~~~~~{.py}
#     imuObject = imu_sensor.ImuSensor()
#     imuObject.InputStateMsg = self.scObject.scStateOutMsgName
#     imuObject.OutputDataMsg = "imu_sensor_output"
# ~~~~~~~~~~~~~
# and the code required to attach it to the simulation is:
# ~~~~~~~~~~~~~{.py}
#     SimBase.AddModelToTask(self.taskName, self.imuObject, None, 205)
# ~~~~~~~~~~~~~
#
# ### Dynamics Changes to Configure Eclipse Module
#
# The scenario also requires the user to establish a eclipse module.
# This module is used to simulate a shadow casing over the CSS constellation to interfere with the sun direction vector.
# To establish this module, use the following code:
#
# ~~~~~~~~~~~~~{.py}
#         self.eclipseObject.sunInMsgName = 'sun_planet_data'
#         self.eclipseObject.addPlanetName('earth')
#         self.eclipseObject.addPlanetName('moon')
#         self.eclipseObject.addPositionMsgName(self.scObject.scStateOutMsgName)
# ~~~~~~~~~~~~~
#
# The module requires spice data regarding the location of the sun, the planets to be monitored for shadow-casting
# effects, and the location of the spacecraft. In combination these inputs can produce an output that is attached to the
# CSS constellation which simulates a shadow. The eclipse object output is called using:
#
#         eclipse_data_0
#
# where the number at the end corresponds to order of eclipse objects linked to the simulation.
#
#
#
# ### Flight Algorithm Changes to Configure Sun Safe Pointing Guidance
#
# The general flight algorithm setup is different than the earlier simulation scripts. Here we
# use the sunSafePoint() guidance module, the CSSWlsEst() module to evaluate the
# sun pointing vector, and the MRP_Feedback() module to provide the desired \f${\mathbf L}_r\f$
# control torque vector.
#
# The sunSafePoint() guidance module is used to steer the spacecraft to point towards the sun direction vector.
# This is used for functionality like safe mode, or a power generation mode. The inputs of the module are the
# sun direction vector (as provided by the CSSWlsEst module), as well as the body rate information (as provided by the
#  IMU). The guidance module can be
# configured using:
#
# ~~~~~~~~~~~~~{.py}
# self.sunSafePointData = sunSafePoint.sunSafePointConfig()
# self.sunSafePointWrap = SimBase.setModelDataWrap(self.sunSafePointData)
# self.sunSafePointWrap.ModelTag = "sunSafePoint"
# SimBase.fswProc.addTask(SimBase.CreateNewTask("sunSafePointTask", self.processTasksTimeStep), 20)
# self.sunSafePointData.attGuidanceOutMsgName = "guidanceOut"
# self.sunSafePointData.imuInMsgName = SimBase.DynModels.imuObject.OutputDataMsg
# self.sunSafePointData.sunDirectionInMsgName = self.cssWlsEstData.navStateOutMsgName
# self.sunSafePointData.sHatBdyCmd = [1.0, 0.0, 0.0]
# ~~~~~~~~~~~~~
# The sHatBdyCmd defines the desired body pointing vector that will align with the sun direction vector.
# The sun direction vector itself is calculated through the use of a CSS constellation and the CSSWlsEst module. The
# setup for the CSS constellation can be found in the [scenarioCSS.py](@ref scenarioCSS) scenario. The CSSWlsEst module
# is a weighted least-squares minimum-norm algorithm used to estimate the body-relative sun heading using a cluster of
# coarse sun sensors. The algorithm requires a minimum of three CSS to operate correctly. The corresponding I/O for the
# CSS is called using:
# ~~~~~~~~~~~~~{.py}
#         cssConfig = fswMessages.CSSConfigFswMsg()
#         totalCSSList = []
#         nHat_B_vec = [
#                       [1. / np.sqrt(2.), 0., -1. / np.sqrt(2.)],
#                       [1. / np.sqrt(2.), 1. / np.sqrt(2.), 0.],
#                       [1. / np.sqrt(2.), 0., 1. / np.sqrt(2)],
#                       [1. / np.sqrt(2.), -1. / np.sqrt(2.), 0.]
#                       ]
#         for CSSHat in nHat_B_vec:
#             CSSConfigElement = fswMessages.CSSUnitConfigFswMsg()
#             CSSConfigElement.CBias = 1.0
#             CSSConfigElement.nHat_B = CSSHat
#             totalCSSList.append(CSSConfigElement)
#             cssConfig.cssVals = totalCSSList
#
#         cssConfig.nCSS = len(SimBase.DynModels.CSSConstellationObject.sensorList)
#         cssConfigSize = cssConfig.getStructSize()
#         SimBase.TotalSim.CreateNewMessage("FSWProcess", "css_config_data", cssConfigSize, 2, "CSSConstellation")
#         SimBase.TotalSim.WriteMessageData("css_config_data", cssConfigSize, 0, cssConfig)
# ~~~~~~~~~~~~~
# and the way to interface with the CSSWlsEst algorithm is using:
# ~~~~~~~~~~~~~{.py}
#         self.cssWlsEstData.cssDataInMsgName = SimBase.DynModels.CSSConstellationObject.outputConstellationMessage
#         self.cssWlsEstData.cssConfigInMsgName = "css_config_data"
#         self.cssWlsEstData.navStateOutMsgName = "sun_point_data"
# ~~~~~~~~~~~~~
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
# ![RW Spin History](Images/Scenarios/scenarioAttitudeFeedbackRW300.svg "RW Omega history")
# Note that in the RW motor torque plot both the required control torque \f$\hat u_B\f$ and the true
# motor torque \f$u_B\f$ are shown.  This illustrates that with this maneuver the RW devices are being
# saturated, and the attitude still eventually stabilizes.
#
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
