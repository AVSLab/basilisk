''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
# Purpose:  Demonstrates how to setup and use sun heading filters
# Author:   Thibaud Teil
# Creation Date:  November 20, 2017
#



import pytest
import numpy as np
import time

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
import matplotlib.pyplot as plt
from Basilisk.utilities import orbitalMotion as om
from Basilisk.utilities import RigidBodyKinematics as rbk

from Basilisk.simulation import spacecraftPlus, spice_interface, coarse_sun_sensor
from Basilisk.fswAlgorithms import sunlineUKF, sunlineEKF, okeefeEKF, vehicleConfigData

import SunLineKF_test_utilities as Fplot

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("FilterType, simTime", [
      ('uKF', 200)
    , ('EKF', 200)
    , ('OEKF', 200)
])

# provide a unique test method name, starting with test_
def test_Filters(show_plots, FilterType, simTime):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, FilterType, simTime)
    assert testResults < 1, testMessage


def setupUKFData(filterObject):
    filterObject.navStateOutMsgName = "sunline_state_estimate"
    filterObject.filtDataOutMsgName = "sunline_filter_data"
    filterObject.cssDataInMsgName = "css_sensors_data"
    filterObject.cssConfInMsgName = "css_config_data"

    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0

    filterObject.state = [1.0, 0.1, 0.0, 0.0, 0.01, 0.0]
    filterObject.covar = [1., 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 1., 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 1., 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
    qNoiseIn = np.identity(6)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3]*0.017*0.017
    qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6]*0.0017*0.0017
    filterObject.qNoise = qNoiseIn.reshape(36).tolist()
    filterObject.qObsVal = 0.017**2
    filterObject.sensorUseThresh = np.sqrt(filterObject.qObsVal)*5


def setupEKFData(filterObject):
    filterObject.navStateOutMsgName = "sunline_state_estimate"
    filterObject.filtDataOutMsgName = "sunline_filter_data"
    filterObject.cssDataInMsgName = "css_sensors_data"
    filterObject.cssConfInMsgName = "css_config_data"

    filterObject.states = [1.0, 0.1, 0.0, 0.0, 0.01, 0.0]
    filterObject.x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    filterObject.covar = [1., 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 1., 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 1., 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.02]

    filterObject.qProcVal = 0.001**2
    filterObject.qObsVal = 0.017**2
    filterObject.sensorUseThresh = np.sqrt(filterObject.qObsVal)*5

    filterObject.eKFSwitch = 5. #If low (0-5), the CKF kicks in easily, if high (>10) it's mostly only EKF

def setupOEKFData(filterObject):
    filterObject.navStateOutMsgName = "sunline_state_estimate"
    filterObject.filtDataOutMsgName = "sunline_filter_data"
    filterObject.cssDataInMsgName = "css_sensors_data"
    filterObject.cssConfInMsgName = "css_config_data"

    filterObject.omega = [0., 0., 0.]
    filterObject.states = [1.0, 0.1, 0.0]
    filterObject.x = [0.0, 0.0, 0.0]
    filterObject.covar = [1., 0.0, 0.0,
                          0.0, 1., 0.0,
                          0.0, 0.0, 1.]

    filterObject.qProcVal = 0.1**2
    filterObject.qObsVal = 0.017 ** 2
    filterObject.sensorUseThresh = np.sqrt(filterObject.qObsVal)*5

    filterObject.eKFSwitch = 5. #If low (0-5), the CKF kicks in easily, if high (>10) it's mostly only EKF


## \defgroup Tutorials_4_0_1
##   @{
## Demonstrates how Estimate spacecraft attitude using Coarse Sun Sensors Filters.
#
# Coarse Sun Sensor (CSS) Filters {#scenarioCSSFilters}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft in deep space without any gravitational bodies. Only rotational
# motion is simulated.  The script illustrates how to setup attitude filters that use measurements from the Coarse Sun Sensors (CSS).
# A constellation of CSS are setup, and different filters are used to compare their performances.
#
# Setup | Filter               |
# ----- | -------------------- |
# 1     | uKF                  |
# 2     | EKF                  |
# 3     | EKF V2               |
#
# To run the default scenario, call the python script through
#
#       python test_scenarioCSSFilters.py
#
# When the simulation completes several plots are written summarizing the filter performances.
#
# The simulation reads the Sun's poistion from SpiceInterface(). By creating this spice object and adding it to the
# task, the spice object automatically writes out the ephemeris messages.
# The date used is of no importance for this scenario.
# ~~~~~~~~~~~~~~~~{.py}
#     spiceObject = spice_interface.SpiceInterface()
#     spiceObject.planetNames = spice_interface.StringVector(["sun"])
#     spiceObject.ModelTag = "SpiceInterfaceData"
#     spiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
#     spiceObject.outputBufferCount = 100000
#     spiceObject.UTCCalInit = '2021 MAY 04 07:47:49.965 (UTC)'
#     scSim.AddModelToTask(simTaskName, spiceObject)
# ~~~~~~~~~~~~~~~~
#
# The dynamics simulation is setup using a SpacecraftPlus() module where a specific spacecraft location
# is specified.  Note that both the rotational and translational degrees of
# freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.  The position
# vector is required when computing the relative heading between the sun and the spacecraft locations.  The
# spacecraft position is held fixed, while the orientation rotates constantly about the 3rd body axis.
# ~~~~~~~~~~~~~~~~{.py}
#     scObject.hub.r_CN_NInit = [[-om.AU*1000.], [0.0], [0.0]]        # m   - r_CN_N
#     scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]                 # m/s - v_CN_N
#     scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]               # sigma_BN_B
#     scObject.hub.omega_BN_BInit = [[0.5*macros.D2R], [-1.*macros.D2R], [1.*macros.D2R]]   # rad/s - omega_BN_B
# ~~~~~~~~~~~~~~~~
#
# The CSS modules must first be individual created and configured.
# This simulation uses 8 sun sensors, in 2 pyramids of 4 units. The code that sets up a constellation displays another
# method than used in [test_scenarioCSS.py](@ref scenarioCSS).
# In this case instead of creating a list of CSS and adding the list to the constellation, the "appendCSS" command is used.
#
# ~~~~~~~~~~~~~~~~{.py}
# cssConstelation = coarse_sun_sensor.CSSConstellation()
# for CSSHat in CSSOrientationList:
#     newCSS = coarse_sun_sensor.CoarseSunSensor()
#     newCSS.nHat_B = CSSHat
#     cssConstelation.appendCSS(newCSS)
# cssConstelation.outputConstellationMessage = 'css_sensors_data'
# scSim.AddModelToTask(simTaskName, cssConstelation)
# ~~~~~~~~~~~~~~~~
#
# The constellation characteristics are summarized in the following table. This table shows the individual unit vectors
# for each sensor, named nHat_B in the code.
#
# CSS   | normal vector          |
# ----- | ---------------------- |
# 1     | [sqrt(2)/2, -0.5, 0.5] |
# 2     | [sqrt(2)/2, -0.5, -0.5]|
# 3     | [sqrt(2)/2, 0.5, -0.5] |
# 4     | [sqrt(2)/2,  0.5, 0.5]     |
# 5     | [-sqrt(2)/2, 0, sqrt(2)/2] |
# 6     | [-sqrt(2)/2, sqrt(2)/2, 0] |
# 7     | [-sqrt(2)/2, 0, -sqrt(2)/2] |
# 8     | [-sqrt(2)/2, -sqrt(2)/2, 0] |
#
#
# An additional message must be written for the configuration of the CSS for the Flight Software modules. This is done with vehicleConfigData,
# a message that is read once at the start of a simulation. This message also allows the user to set different values between the
# simulation and the flight software parameters, which could corrupt the simulation, and reproduce an imperfect spacecraft
# construction process.
#
# ~~~~~~~~~~~~~~~~{.py}
# cssConstVehicle = vehicleConfigData.CSSConstConfig()
#
# totalCSSList = []
# for CSSHat in CSSOrientationList:
#     newCSS = vehicleConfigData.CSSConfigurationElement()
#     newCSS.nHat_B = CSSHat
#     totalCSSList.append(newCSS)
# cssConstVehicle.nCSS = len(CSSOrientationList)
# cssConstVehicle.cssVals = totalCSSList
#
# ~~~~~~~~~~~~~~~~
# This allows us to write the CSS config message, using the unitTestSupport function "setMessage"
# ~~~~~~~~~~~~~~~~{.py}
#unitTestSupport.setMessage(scSim.TotalSim, simProcessName, "css_config_data", cssConstVehicle)
# ~~~~~~~~~~~~~~~~
#
# This sets up the spacecraft, it's sun sensors, and the sun direction. The filters can now be initialized.
# These are configured very similarly, but the nature of the filters lead to slight differences.
# All of the filters output a Navigation message which outputs the sun heading for other modules to use,
# but they also output a filtering message (sunlineFilterFswMsg.h), containing observations, post-fit residuals,
# covariances, and full states.
# This allows users to check in on filter performances efficiently, and is used in this tutorial. This first allows us
# to see when observations occur throughout the scenario over which we are comparing performance:
#
# ![CSS Observations](Images/Scenarios/scenario_Filters_ObsOEKF.svg "Number of CSS measurements")
#
#
# Setup 1 - ukF
# -----
#
# In the first run, we use an square root unscented Kalman Filter. This filter has the following states:
#
# States         |     notation |
# -------------- | ------------ |
# Sunheading     |      d       |
# Sunheading Rate|      d_dot   |
#
# This filter estimates sunheading, and the sunheading's rate of change. As a unscented filter, it also has the
# the following parameters:
#
#
# Name       | Value        |
# -----------| ------------ |
# alpha      |    0.02      |
# beta       |      2       |
# kappa      |      0       |
#
# The covariance is then set, as well as the measurement noise:
#
#
# Parameter                                 |       Value       |
# ----------------------------------------  | ----------------- |
# covariance on  heading vector  components |       0.2         |
# covariance on heading rate  components    |       0.02        |
# noise on heading measurements             |       0.017 ** 2  |
# noise on heading measurements             |       0.0017 ** 2 |
#
#
# This is all initialized in the following code. Alpha, Beta, and Kappa are varaibles specific to uKF propagation.
# The state vector and the covariance are named "state" and "covar" respectively, while the measurement noise
# is given by the qNoiseIn , matrix. The standard deviation of the measurement noise is of 0.017**2 on the sun heading
# components, and 0.0017**2 and the sun heading rate. The qObsVal is used in the filter to create noise matrices.
#  ~~~~~~~~~~~~~{.py}
# filterObject.navStateOutMsgName = "sunline_state_estimate"
# filterObject.filtDataOutMsgName = "sunline_filter_data"
# filterObject.cssDataInMsgName = "css_sensors_data"
# filterObject.cssConfInMsgName = "css_config_data"
#
# filterObject.alpha = 0.02
# filterObject.beta = 2.0
# filterObject.kappa = 0.0
#
# filterObject.state = [1.0, 0.1, 0.0, 0.0, 0.01, 0.0]
# filterObject.covar = [1., 0.0, 0.0, 0.0, 0.0, 0.0,
#                       0.0, 1., 0.0, 0.0, 0.0, 0.0,
#                       0.0, 0.0, 1., 0.0, 0.0, 0.0,
#                       0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
#                       0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
#                       0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
# qNoiseIn = np.identity(6)
# qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 0.017 * 0.017
# qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 0.0017 * 0.0017
# filterObject.qNoise = qNoiseIn.reshape(36).tolist()
# filterObject.qObsVal = 0.017 * 0.017
# ~~~~~~~~~~~~~
#
# The resulting plots of the states, their covariance envelopes, as compared to the true state
# are plotted. Further documentation can be found in the _Documentation folder in the module directory, the paper
# of interested found in '/src/fswAlgorithms/attDetermination/sunlineUKF/_Documentation/sunlineUKF_DesignDescription.pdf'.
# ![uKF Performance](Images/Scenarios/scenario_Filters_StatesExpecteduKF.svg "States vs Truth")
#
# These plots show good state estimation throughout the simulation. The mean stays close to the truth, the states do
# appear slightly noisy at times.
#
# The post fit residuals, show a fully functional filter, with no issues of observabilty:
# ![uKF Post Fit](Images/Scenarios/scenario_Filters_PostFituKF.svg "Post Fit Residuals")
#
# Setup 2 - EKF
# ------
#
# The following filter tested is an Extended Kalman filter. This filter uses all the same values for initialization
#  as the uKF (aside from the uKF specific alpha, beta, kappa variables). A couple variables are added:
#
# Name          | Value        |
# -----------   | ------------ |
# Process noise |    0.001**2  |
# CKF switch    |      5       |
#
# The process noise is the noise added on the dynamics. This allows to account for dynamical uncertainties, and
# avoid filter saturation.
#
# The CKF switch is the number of measurements that are processed using a classical, linear Kalman filter when the
# filter is first run. This allows for the covariance to shrink before employing the EKF, increasing the robustness.
#
# These variables are setup as previously, with the addition of the state error vector, called 'x'.
# ~~~~~~~~~~~~~{.py}
# filterObject.navStateOutMsgName = "sunline_state_estimate"
# filterObject.filtDataOutMsgName = "sunline_filter_data"
# filterObject.cssDataInMsgName = "css_sensors_data"
# filterObject.cssConfInMsgName = "css_config_data"
#
# filterObject.sensorUseThresh = 0.
# filterObject.states = [1.0, 0.1, 0.0, 0.0, 0.01, 0.0]
# filterObject.x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# filterObject.covar = [1., 0.0, 0.0, 0.0, 0.0, 0.0,
#                       0.0, 1., 0.0, 0.0, 0.0, 0.0,
#                       0.0, 0.0, 1., 0.0, 0.0, 0.0,
#                       0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
#                       0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
#                       0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
#
# filterObject.qProcVal = 0.001 ** 2
# filterObject.qObsVal = 0.017 ** 2
# filterObject.eKFSwitch = 5.  # If low (0-5), the CKF kicks in easily, if high (>10) it's mostly only EKF
# ~~~~~~~~~~~~~
# The states vs expected states are plotted, as well as the state error plots along with the covariance
# envelopes. Further documentation can be found in the _Documentation folder in the module directory:
#'/src/fswAlgorithms/attDetermination/sunlineEKF/_Documentation/'.
# ![EKF State Errors](Images/Scenarios/scenario_Filters_StatesPlotEKF.svg "State Error and Covariances")
# ![EKF Filter performance](Images/Scenarios/scenario_Filters_StatesExpectedEKF.svg "States vs Truth")
#
# These plots show good state estimation throughout the simulation and despite the patches of time with
# fewer measurements. The covariance stays close to the mean, without exesive noise.
#
# The post fit residuals, give further confirmation of a working filter:
# ![EKF Post Fit](Images/Scenarios/scenario_Filters_PostFitEKF.svg "Post Fit Residuals")
#
# Setup 3 -OEKF
# ------
#
# The 3rd scenario uses a second type of Extended Kalman Filter (named Okeefe-EKF). This filter takes in fewer states
# as it only estimates the sunheading. In order to propagate it, it estimates the omega vector from the two last
# measurements.
#
# The set up is nearly identical to the EKF, with the exception of the size of the vectors and matrices (only 3 states
# are estimated now). Furthermore, the rotation rate of the spacecraft, omega, is initialized.
#
# ~~~~~~~~~~~~~{.py}
# filterObject.navStateOutMsgName = "sunline_state_estimate"
# filterObject.filtDataOutMsgName = "sunline_filter_data"
# filterObject.cssDataInMsgName = "css_sensors_data"
# filterObject.cssConfInMsgName = "css_config_data"
#
# filterObject.sensorUseThresh = 0.
# filterObject.omega = [0., 0., 0.]
# filterObject.states = [1.0, 0.1, 0.0]
# filterObject.x = [0.0, 0.0, 0.0]
# filterObject.covar = [1., 0.0, 0.0,
#                       0.0, 1., 0.0,
#                       0.0, 0.0, 1.]
#
# filterObject.qProcVal = 0.1 ** 2
# filterObject.qObsVal = 0.017 ** 2
# filterObject.eKFSwitch = 5.  # If low (0-5), the CKF kicks in easily, if high (>10) it's mostly only EKF
# ~~~~~~~~~~~~~
# More in-depth documentation on the filter specifics are found in '/src/fswAlgorithms/attDetermination/okeefeEKF/_Documentation/'.
# The results from this filter are plotted:
# ![OEKF State Errors](Images/Scenarios/scenario_Filters_StatesPlotOEKF.svg "State Error and Covariances")
# ![OEKF Filter performance](Images/Scenarios/scenario_Filters_StatesExpectedOEKF.svg "States vs Truth")
#
# These plots show poorer state estimation throughout the simulation. As measurements stop, the filter doesn't
# propagate the states sufficiently well. This is due to the absence of rate in the states, and the compensation
# with the computation of omega can lead to noisy estimates.
#
# The post fit residuals, do show that the filter is working, just with difficulties when measurements become sparse:
# ![OEKF Post Fit](Images/Scenarios/scenario_Filters_PostFitOEKF.svg "Post Fit Residuals")
#
##  @}
def run(show_plots, FilterType, simTime):
    '''Call this routine directly to run the tutorial scenario.'''
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    #
    #  From here on there scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    # set the simulation time variable used later on
    simulationTime = macros.sec2nano(simTime)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.5)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #
    spiceObject = spice_interface.SpiceInterface()
    spiceObject.planetNames = spice_interface.StringVector(["sun"])
    spiceObject.ModelTag = "SpiceInterfaceData"
    spiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
    spiceObject.outputBufferCount = 100000
    spiceObject.UTCCalInit = '2021 MAY 04 07:47:49.965 (UTC)'

    scSim.TotalSim.logThisMessage('sun_planet_data', simulationTimeStep)
    scSim.AddModelToTask(simTaskName, spiceObject)


    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0                   # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = True

    #
    # set initial spacecraft states
    #
    scObject.hub.r_CN_NInit = [[-om.AU*1000.], [0.0], [0.0]]              # m   - r_CN_N
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]                 # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.]]               # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[-0.1*macros.D2R], [0.5*macros.D2R], [0.5*macros.D2R]]   # rad/s - omega_BN_B

    scSim.TotalSim.logThisMessage('inertial_state_output', simulationTimeStep)
    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # Make a CSS constelation
    cssConstelation = coarse_sun_sensor.CSSConstellation()
    CSSOrientationList = [
        [0.70710678118654746, -0.5, 0.5],
        [0.70710678118654746, -0.5, -0.5],
        [0.70710678118654746, 0.5, -0.5],
        [0.70710678118654746, 0.5, 0.5],
        [-0.70710678118654746, 0, 0.70710678118654757],
        [-0.70710678118654746, 0.70710678118654757, 0.0],
        [-0.70710678118654746, 0, -0.70710678118654757],
        [-0.70710678118654746, -0.70710678118654757, 0.0]
    ]
    for CSSHat in CSSOrientationList:
        newCSS = coarse_sun_sensor.CoarseSunSensor()
        newCSS.minOutput = 0.
        newCSS.SenNoiseStd = 0.017
        newCSS.nHat_B = CSSHat
        cssConstelation.appendCSS(newCSS)
    cssConstelation.outputConstellationMessage = 'css_sensors_data'
    scSim.AddModelToTask(simTaskName, cssConstelation)

    #
    #   Add the normals to the vehicle Config data struct
    #
    cssConstVehicle = vehicleConfigData.CSSConstConfig()

    totalCSSList = []
    for CSSHat in CSSOrientationList:
        newCSS = vehicleConfigData.CSSConfigurationElement()
        newCSS.nHat_B = CSSHat
        totalCSSList.append(newCSS)
    cssConstVehicle.nCSS = len(CSSOrientationList)
    cssConstVehicle.cssVals = totalCSSList
    #
    # Setup filter
    #

    numStates = 6
    if FilterType == 'EKF':
        moduleConfig = sunlineEKF.sunlineEKFConfig()
        moduleWrap = scSim.setModelDataWrap(moduleConfig)
        moduleWrap.ModelTag = "SunlineEKF"
        setupEKFData(moduleConfig)

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, moduleWrap, moduleConfig)

    if FilterType == 'OEKF':
        numStates = 3

        moduleConfig = okeefeEKF.okeefeEKFConfig()
        moduleWrap = scSim.setModelDataWrap(moduleConfig)
        moduleWrap.ModelTag = "okeefeEKF"
        setupOEKFData(moduleConfig)

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, moduleWrap, moduleConfig)


    if FilterType == 'uKF':
        moduleConfig = sunlineUKF.SunlineUKFConfig()
        moduleWrap = scSim.setModelDataWrap(moduleConfig)
        moduleWrap.ModelTag = "SunlineUKF"
        setupUKFData(moduleConfig)

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, moduleWrap, moduleConfig)


    scSim.TotalSim.logThisMessage('sunline_state_estimate', simulationTimeStep)
    scSim.TotalSim.logThisMessage('sunline_filter_data', simulationTimeStep)

    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, "css_config_data", cssConstVehicle)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)

    # Time the runs for performance comparisons
    timeStart = time.time()
    scSim.ExecuteSimulation()
    timeEnd = time.time()


    #
    #   retrieve the logged data
    #

    # Get messages that will make true data
    OutSunPos = scSim.pullMessageLogData('sun_planet_data' + ".PositionVector", range(3))
    Outr_BN_N = scSim.pullMessageLogData('inertial_state_output' + ".r_BN_N", range(3))
    OutSigma_BN = scSim.pullMessageLogData('inertial_state_output' + ".sigma_BN", range(3))
    Outomega_BN = scSim.pullMessageLogData('inertial_state_output' + ".omega_BN_B", range(3))

    # Get the filter outputs through the messages
    sunPnt_B = scSim.pullMessageLogData('sunline_state_estimate' + ".vehSunPntBdy", range(3))
    stateErrorLog = scSim.pullMessageLogData('sunline_filter_data' + ".stateError", range(numStates))
    stateLog = scSim.pullMessageLogData('sunline_filter_data' + ".state", range(numStates))
    postFitLog = scSim.pullMessageLogData('sunline_filter_data' + ".postFitRes", range(8))
    covarLog = scSim.pullMessageLogData('sunline_filter_data' + ".covar", range(numStates*numStates))
    obsLog = scSim.pullMessageLogData('sunline_filter_data' + ".numObs", range(1))

    sHat_B = np.zeros(np.shape(OutSunPos))
    sHatDot_B = np.zeros(np.shape(OutSunPos))
    for i in range(len(OutSunPos[:,0])):
        sHat_N = (OutSunPos[i,1:] - Outr_BN_N[i,1:])/np.linalg.norm(OutSunPos[i,1:] - Outr_BN_N[i,1:])
        dcm_BN = rbk.MRP2C(OutSigma_BN[i,1:])
        sHat_B[i,0] = sHatDot_B[i,0]= OutSunPos[i,0]
        sHat_B[i,1:] = np.dot(dcm_BN, sHat_N)
        sHatDot_B[i,1:] = - np.cross(Outomega_BN[i,1:], sHat_B[i,1:] )

    expected = np.zeros(np.shape(stateLog))
    expected[:,0:4] = sHat_B
    # The OEKF has fewer states
    if FilterType != 'OEKF':
        expected[:, 4:] = sHatDot_B[:,1:]

    #
    #   plot the results
    #
    errorVsTruth = np.copy(stateLog)
    errorVsTruth[:,1:] -= expected[:,1:]

    Fplot.StateErrorCovarPlot(errorVsTruth, covarLog, FilterType, show_plots)
    Fplot.StatesVsExpected(stateLog, covarLog, expected, FilterType, show_plots)
    Fplot.PostFitResiduals(postFitLog, np.sqrt(moduleConfig.qObsVal), FilterType, show_plots)
    Fplot.numMeasurements(obsLog, FilterType, show_plots)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run( True,      # show_plots
        'uKF',
         400
       )

