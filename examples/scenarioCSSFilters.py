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

This script sets up a 6-DOF spacecraft in deep space without any gravitational
bodies. Only rotational motion is simulated.  The script illustrates how to
setup attitude filters that use measurements from the Coarse Sun Sensors (CSS).

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioCSSFilters.py

When the simulation completes several plots are written summarizing the filter performances.

The simulation reads the Sun's position from :ref:`SpiceInterface`. By creating this
spice object and adding it to the task, the spice object automatically writes out
the ephemeris messages.

The dynamics simulation is setup using a :ref:`Spacecraft` module where a specific
spacecraft location is specified.  Note that both the rotational and translational
degrees of freedom of the spacecraft hub are turned on here to get a 6-DOF simulation.
The position vector is required when computing the relative heading between the sun
and the spacecraft locations.  The spacecraft position is held fixed, while the
orientation rotates constantly about the 3rd body axis.

The CSS modules must first be individual created and configured. This simulation
uses 8 sun sensors, in 2 pyramids of 4 units. The code that sets up a constellation
displays another method that is used in :ref:`scenarioCSS`.  In this
case instead of creating a list of CSS and adding the list to the constellation,
the ``appendCSS`` command is used.

The constellation characteristics are summarized in the following table.
This table shows the individual unit vectors for each sensor, named ``nHat_B`` in the code.

========  ============================
  CSS     normal vector
========  ============================
  1       [:math:`\sqrt{2}`/2, -0.5, 0.5]
  2       [:math:`\sqrt{2}`/2, -0.5, -0.5]
  3       [:math:`\sqrt{2}`/2, 0.5, -0.5]
  4       [:math:`\sqrt{2}`/2,  0.5, 0.5]
  5       [-:math:`\sqrt{2}`/2, 0, :math:`\sqrt{2}`/2]
  6       [-:math:`\sqrt{2}`/2, :math:`\sqrt{2}`/2, 0]
  7       [-:math:`\sqrt{2}`/2, 0, -:math:`\sqrt{2}`/2]
  8       [-:math:`\sqrt{2}`/2, -:math:`\sqrt{2}`/2, 0]
========  ============================

An additional message must be written for the configuration of the CSS for the
Flight Software modules. This is done with ``vehicleConfigData``, a message that is
read once at the start of a simulation. This message also allows the user to set
different values between the  simulation and the flight software parameters,
which could corrupt the simulation, and reproduce an imperfect spacecraft
construction process.

The filters can now be initialized.  These are configured very similarly, but
the nature of the filters lead to slight differences. All of the filters output
a Navigation message which outputs the sun heading for other modules to use,
but they also output a filtering message (:ref:`sunlineFilterMsgPayload`), containing
observations, post-fit residuals, covariances, and full states. This allows users
to check in on filter performances efficiently, and is used in this tutorial.
This first allows us to see when observations occur throughout the scenario
over which we are comparing performance:

.. image:: /_images/Scenarios/scenario_Filters_ObsOEKF.svg
   :align: center

Setup 1 - ukF
-------------

In the first run, we use an square root unscented Kalman Filter (:ref:`sunlineUKF`).
This filter has the following states:

================  =============
States            notation
================  =============
Sunheading        ``d``
Sunheading Rate|  ``d_dot``
================  =============


This filter estimates sunheading, and the sunheading's rate of change.
As a unscented filter, it also has the the following parameters:

=============  =============
  Name         Value
=============  =============
  ``alpha``     0.02
  ``beta``      2
  ``kappa``     0
=============  =============

The covariance is then set, as well as the measurement noise:

=============================================  ==================
  Parameter                                         Value
=============================================  ==================
  covariance on  heading vector  components         0.2
  covariance on heading rate  components            0.02
  noise on heading measurements                     0.017 ** 2
  noise on heading measurements                     0.0017 ** 2
=============================================  ==================

The resulting plots of the states, their covariance envelopes, as compared
to the true state are plotted. Further documentation can be found in :ref:`sunlineUKF`.

.. image:: /_images/Scenarios/scenario_Filters_StatesPlotuKF.svg
   :align: center

.. image:: /_images/Scenarios/scenario_Filters_StatesExpecteduKF.svg
   :align: center

These plots show good state estimation throughout the simulation.
The mean stays close to the truth, the states do appear slightly noisy at times.

The post fit residuals, show a fully functional filter, with no issues of observability:

.. image:: /_images/Scenarios/scenario_Filters_PostFituKF.svg
   :align: center


Setup 2 - EKF
-------------

The following filter tested is an Extended Kalman filter (:ref:`sunlineEKF`).
This filter uses all the same values for initialization as the uKF (aside
from the uKF specific alpha, beta, kappa variables). A couple variables are added:

===============  ===============
  Name            Value
===============  ===============
  Process noise      0.001**2
  CKF switch           5
===============  ===============

The process noise is the noise added on the dynamics. This allows to account
for dynamical uncertainties, and avoid filter saturation.

The CKF switch is the number of measurements that are processed using a classical,
linear Kalman filter when the filter is first run. This allows for the covariance
to shrink before employing the EKF, increasing the robustness.

The states vs expected states are plotted, as well as the state error plots along with the covariance
envelopes. Further documentation can be found in :ref:`sunlineEKF`

.. image:: /_images/Scenarios/scenario_Filters_StatesPlotEKF.svg
   :align: center

.. image:: /_images/Scenarios/scenario_Filters_StatesExpectedEKF.svg
   :align: center

These plots show good state estimation throughout the simulation and despite the patches of time with
fewer measurements. The covariance stays close to the mean, without exesive noise.

The post fit residuals, give further confirmation of a working filter:

.. image:: /_images/Scenarios/scenario_Filters_PostFitEKF.svg
   :align: center


Setup 3 - OEKF
--------------

The 3rd scenario uses a second type of Extended Kalman Filter (:ref:`okeefeEKF`).
This filter takes in fewer states as it only estimates the sun heading. In order to
propagate it, it estimates the omega vector from the two last measurements.

The set up is nearly identical to the EKF, with the exception of the size of the
vectors and matrices (only 3 states are estimated now). Furthermore, the
rotation rate of the spacecraft, omega, is initialized. More in-depth documentation on the filter
specifics are found in :ref:`okeefeEKF`.

.. image:: /_images/Scenarios/scenario_Filters_StatesPlotOEKF.svg
   :align: center

.. image:: /_images/Scenarios/scenario_Filters_StatesExpectedOEKF.svg
   :align: center

These plots show poorer state estimation throughout the simulation. As measurements stop,
the filter doesn't propagate the states sufficiently well. This is due to the absence of
rate in the states, and the compensation  with the computation of omega can lead to
noisy estimates.

The post fit residuals, do show that the filter is working, just with difficulties
when measurements become sparse:

.. image:: /_images/Scenarios/scenario_Filters_PostFitOEKF.svg
   :align: center


Setup 4 -Switch-EKF
-------------------

The 4th scenario uses a Switch formulation to extract the observable rates as
well as estimate the sun heading (:ref:`sunlineSEKF`).

.. image:: /_images/Scenarios/scenario_Filters_StatesPlotSEKF.svg
   :align: center

.. image:: /_images/Scenarios/scenario_Filters_StatesExpectedSEKF.svg
   :align: center

These plots show poorer state estimation throughout the simulation.

The post fit residuals show that the filter is working, just with difficulties
when measurements become sparse.

.. image:: /_images/Scenarios/scenario_Filters_PostFitSEKF.svg
   :align: center



Setup 5 -Switch-uKF
-------------------

The 5th scenario uses the same Switch formulation but in a square-root uKF (:ref:`sunlineSuKF`).
This one has an additional state: the sun intensity (equal to 1 at 1AU). This state
has low process noise and low initial covariance given it's well determined nature generally.

.. image:: /_images/Scenarios/scenario_Filters_StatesPlotSuKF.svg
   :align: center

.. image:: /_images/Scenarios/scenario_Filters_StatesExpectedSuKF.svg
   :align: center

These plots show good state estimation throughout the simulation.
The mean stays close to the truth.

The post fit residuals, show a fully functional filter, with no issues of observabilty:

.. image:: /_images/Scenarios/scenario_Filters_PostFitSuKF.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstrates how to setup and use sun heading filters
# Author:   Thibaud Teil
# Creation Date:  November 20, 2017
#



import numpy as np

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
import matplotlib.pyplot as plt
from Basilisk.utilities import orbitalMotion as om
from Basilisk.utilities import RigidBodyKinematics as rbk

from Basilisk.simulation import spacecraft, coarseSunSensor
from Basilisk.fswAlgorithms import sunlineUKF, sunlineEKF, okeefeEKF, sunlineSEKF, sunlineSuKF
from Basilisk.architecture import messaging

import SunLineKF_test_utilities as Fplot


def setupUKFData(filterObject):
    """Setup UKF Filter Data"""
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
    """Setup EKF Filter Data"""
    filterObject.state = [1.0, 0.1, 0.0, 0.0, 0.01, 0.0]
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
    """Setup OEKF Filter Data"""
    filterObject.omega = [0., 0., 0.]
    filterObject.state = [1.0, 0.1, 0.0]
    filterObject.x = [0.0, 0.0, 0.0]
    filterObject.covar = [1., 0.0, 0.0,
                          0.0, 1., 0.0,
                          0.0, 0.0, 1.]

    filterObject.qProcVal = 0.1**2
    filterObject.qObsVal = 0.017 ** 2
    filterObject.sensorUseThresh = np.sqrt(filterObject.qObsVal)*5

    filterObject.eKFSwitch = 5. #If low (0-5), the CKF kicks in easily, if high (>10) it's mostly only EKF

def setupSEKFData(filterObject):
    """Setup SEKF Filter Data"""
    filterObject.state = [1.0, 0.1, 0., 0., 0.]
    filterObject.x = [0.0, 0.0, 0.0, 0.0, 0.0]
    filterObject.covar = [1., 0.0, 0.0, 0.0, 0.0,
                          0.0, 1., 0.0, 0.0, 0.0,
                          0.0, 0.0, 1., 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.01, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.01]

    filterObject.qProcVal = 0.001**2
    filterObject.qObsVal = 0.017 ** 2
    filterObject.sensorUseThresh = np.sqrt(filterObject.qObsVal)*5

    filterObject.eKFSwitch = 5. #If low (0-5), the CKF kicks in easily, if high (>10) it's mostly only EKF


def setupSuKFData(filterObject):
    """Setup SuKF Filter Data"""
    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0

    filterObject.stateInit = [1.0, 0.1, 0., 0., 0., 1.]
    filterObject.covarInit = [1., 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 1., 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 1., 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]

    qNoiseIn = np.identity(6)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3]*0.001**2
    qNoiseIn[3:5, 3:5] = qNoiseIn[3:5, 3:5]*0.0001**2
    qNoiseIn[5, 5] = qNoiseIn[5, 5]*0.000001**2
    filterObject.qNoise = qNoiseIn.reshape(36).tolist()
    filterObject.qObsVal = 0.017**2
    filterObject.sensorUseThresh = np.sqrt(filterObject.qObsVal)*5


def run(saveFigures, show_plots, FilterType, simTime):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        saveFigures (bool): flag to save off the figures
        show_plots (bool): Determines if the script should display plots
        FilterType (str): {'uKF', 'EKF', 'OEKF', 'SEKF', 'SuKF'}
        simTime (float): The length of the simulation time

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

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
    # create sun position message at origin
    sunMsgData = messaging.SpicePlanetStateMsgPayload()
    sunMsg = messaging.SpicePlanetStateMsg().write(sunMsgData)
    sunLog = sunMsg.recorder()
    scSim.AddModelToTask(simTaskName, sunLog)

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0                   # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    #
    # set initial spacecraft states
    #
    scObject.hub.r_CN_NInit = [[-om.AU*1000.], [0.0], [0.0]]              # m   - r_CN_N
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]                 # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.]]               # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[-0.1*macros.D2R], [0.5*macros.D2R], [0.5*macros.D2R]]   # rad/s - omega_BN_B

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    dataLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, dataLog)


    # Make a CSS constelation
    cssConstelation = coarseSunSensor.CSSConstellation()
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
    counter = 0
    def setupCSS(CSS):
        CSS.minOutput = 0.
        CSS.senNoiseStd = 0.017
        CSS.sunInMsg.subscribeTo(sunMsg)
        CSS.stateInMsg.subscribeTo(scObject.scStateOutMsg)
        CSS.this.disown()
    for CSSHat in CSSOrientationList:
        newCSS = coarseSunSensor.CoarseSunSensor()
        newCSS.ModelTag = "CSS" + str(counter)
        counter += 1
        setupCSS(newCSS)
        newCSS.nHat_B = CSSHat
        cssConstelation.appendCSS(newCSS)
    scSim.AddModelToTask(simTaskName, cssConstelation)

    #
    #   add the FSW CSS information
    #
    cssConstVehicle = messaging.CSSConfigMsgPayload()

    totalCSSList = []
    for CSSHat in CSSOrientationList:
        newCSS = messaging.CSSUnitConfigMsgPayload()
        newCSS.nHat_B = CSSHat
        newCSS.CBias = 1.0
        totalCSSList.append(newCSS)
    cssConstVehicle.nCSS = len(CSSOrientationList)
    cssConstVehicle.cssVals = totalCSSList

    cssConstMsg = messaging.CSSConfigMsg().write(cssConstVehicle)

    #
    # Setup filter
    #
    numStates = 6
    bVecLogger = None
    if FilterType == 'EKF':
        module = sunlineEKF.sunlineEKF()
        module.ModelTag = "SunlineEKF"
        setupEKFData(module)

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, module)

    if FilterType == 'OEKF':
        numStates = 3

        module = okeefeEKF.okeefeEKF()
        module.ModelTag = "okeefeEKF"
        setupOEKFData(module)

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, module)

    if FilterType == 'uKF':
        module = sunlineUKF.sunlineUKF()
        module.ModelTag = "SunlineUKF"
        setupUKFData(module)

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, module)

    if FilterType == 'SEKF':
        numStates = 5

        module = sunlineSEKF.sunlineSEKF()
        module.ModelTag = "SunlineSEKF"
        setupSEKFData(module)

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, module)
        bVecLogger = module.logger('bVec_B', simulationTimeStep)
        scSim.AddModelToTask(simTaskName, bVecLogger)

    if FilterType == 'SuKF':
        numStates = 6
        module = sunlineSuKF.sunlineSuKF()
        module.ModelTag = "SunlineSuKF"
        setupSuKFData(module)

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, module)
        bVecLogger = module.logger('bVec_B', simulationTimeStep)
        scSim.AddModelToTask(simTaskName, bVecLogger)

    module.cssDataInMsg.subscribeTo(cssConstelation.constellationOutMsg)
    module.cssConfigInMsg.subscribeTo(cssConstMsg)

    navLog = module.navStateOutMsg.recorder()
    filtLog = module.filtDataOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, navLog)
    scSim.AddModelToTask(simTaskName, filtLog)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)

    # Time the runs for performance comparisons
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    def addTimeColumn(time, data):
        return np.transpose(np.vstack([[time], np.transpose(data)]))

    # Get messages that will make true data
    timeAxis = dataLog.times()
    OutSunPos = addTimeColumn(timeAxis, sunLog.PositionVector)
    Outr_BN_N = addTimeColumn(timeAxis, dataLog.r_BN_N)
    OutSigma_BN = addTimeColumn(timeAxis, dataLog.sigma_BN)
    Outomega_BN = addTimeColumn(timeAxis, dataLog.omega_BN_B)

    # Get the filter outputs through the messages
    stateLog = addTimeColumn(timeAxis, filtLog.state[:, range(numStates)])
    postFitLog = addTimeColumn(timeAxis, filtLog.postFitRes[:, :8])
    covarLog = addTimeColumn(timeAxis, filtLog.covar[:, range(numStates*numStates)])
    obsLog = addTimeColumn(timeAxis, filtLog.numObs)

    # Get bVec_B through the variable logger
    bVecLog = None if bVecLogger is None else addTimeColumn(timeAxis, bVecLogger.bVec_B)

    dcmLog = np.zeros([len(stateLog[:,0]),3,3])
    omegaExp = np.zeros([len(stateLog[:,0]),3])
    if FilterType == 'SEKF':
        dcm = sunlineSEKF.new_doubleArray(3 * 3)
        for j in range(9):
            sunlineSEKF.doubleArray_setitem(dcm, j, 0)
        for i in range(len(stateLog[:,0])):
            sunlineSEKF.sunlineSEKFComputeDCM_BS(stateLog[i,1:4].tolist(), bVecLog[i, 1:4].tolist(), dcm)
            dcmOut = []
            for j in range(9):
                dcmOut.append(sunlineSEKF.doubleArray_getitem(dcm, j))
            dcmLog[i,:,:] = np.array(dcmOut).reshape([3,3])
            omegaExp[i,:] = -np.dot(dcmLog[i,:,:], np.array([0, stateLog[i,4], stateLog[i,5]]))
    if FilterType == 'SuKF':
        dcm = sunlineSuKF.new_doubleArray(3 * 3)
        for j in range(9):
            sunlineSuKF.doubleArray_setitem(dcm, j, 0)
        for i in range(len(stateLog[:,0])):
            sunlineSuKF.sunlineSuKFComputeDCM_BS(stateLog[i,1:4].tolist(), bVecLog[i, 1:4].tolist(), dcm)
            dcmOut = []
            for j in range(9):
                dcmOut.append(sunlineSuKF.doubleArray_getitem(dcm, j))
            dcmLog[i,:,:] = np.array(dcmOut).reshape([3,3])
            omegaExp[i,:] = np.dot(dcmLog[i,:,:].T,Outomega_BN[i,1:])


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
    if FilterType != 'OEKF' and FilterType != 'SEKF' and FilterType != 'SuKF':
        expected[:, 4:] = sHatDot_B[:,1:]
    if FilterType == 'SEKF' or FilterType == 'SuKF':
        for i in range(len(stateLog[:, 0])):
            expected[i, 4] = omegaExp[i,1]
            expected[i, 5] = omegaExp[i,2]

    #   plot the results
    #
    errorVsTruth = np.copy(stateLog)
    errorVsTruth[:,1:] -= expected[:,1:]

    Fplot.StateErrorCovarPlot(errorVsTruth, covarLog, FilterType, show_plots, saveFigures)
    Fplot.StatesVsExpected(stateLog, covarLog, expected, FilterType, show_plots, saveFigures)
    Fplot.PostFitResiduals(postFitLog, np.sqrt(module.qObsVal), FilterType, show_plots, saveFigures)
    Fplot.numMeasurements(obsLog, FilterType, show_plots, saveFigures)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(False,       # save figures to file
        True,      # show_plots
        'SuKF',
         400
       )
