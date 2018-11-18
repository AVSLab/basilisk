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
# Purpose:  Integrated test of the spacecraftPlus(), extForceTorque, simpleNav() and
#           MRP_Feedback() modules.  Illustrates spacecraft attitude control in deep
#           space without a planet or gravity body setup.
# Author:   Hanspeter Schaub
# Creation Date:  Sept. 13, 2018
#

import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simple_nav

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import MRP_Feedback
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError

# import message declarations
from Basilisk.fswAlgorithms import fswMessages



## \defgroup Tutorials_2_0_4
##   @{
# Demonstrates how to simulate an attitude control scenario without having any gravity
# bodies present. In essence, the spacecraft is hovering in deep space.  The goal is to
#  stabilize the tumble of a spacecraft and point it towards a fixed inertial direction.
#
# Attitude Detumbling Simulation Without any Gravity Bodies {#scenarioAttitudeFeedbackNoEarth}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is hovering in deep space.  The scenario duplicates
# the scenario in [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedbackNoEarth) without adding
# Earth gravity body to the simulation.  The scenario is
# setup to be run in four different setups:
# Setup | useUnmodeledTorque  | useIntGain | useKnownTorque
# ----- | ------------------- | ---------- | --------------
# 1     | False               | False      | False
# 2     | True                | False      | False
# 3     | True                | True       | False
# 4     | True                | False      | True
#
# To run the default scenario 1., call the python script through
#
#       python scenarioAttitudeFeedbackNoEarth.py
#
# When the simulation completes 3 plots are shown for the MRP attitude history, the rate
# tracking errors, as well as the control torque vector.
#
# The simulation layout is shown in the following illustration.  A single simulation process is created
# which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
# modules.
# ![Simulation Flow Diagram](Images/doc/test_scenarioAttitudeFeedbackNoEarth.svg "Illustration")
#
# The spacecraft simulation is identical to [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedbackNoEarth)
# with identical reaction wheel included.  The primary difference is that the gravity body is not included.
# When initializing the spacecraft states, only the attitude states must be set.  The position and velocity
# states are initialized automatically to zero.
# ~~~~~~~~~~~~~~~~{.py}
#     scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
#     scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B
# ~~~~~~~~~~~~~~~~
#
# The following simulation runs should output identical results to the scenario runs in
# [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedbackNoEarth) as the attitude pointing goal
# is an inertial orientation, and thus independent of the satellite being in Earth orbit or
# hovering in deep space.
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,        # show_plots
#          False,       # useUnmodeledTorque
#          False,       # useIntGain
#          False        # useKnownTorque
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last 3 arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The
# default scenario has both the unmodeled torque and integral feedback turned off.  The
# resulting attitude and control torque histories are shown below.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedbackNoEarth1000.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedbackNoEarth2000.svg "Torque history")
#
# Setup 2
# ------
#
# Here the python main function is changed to read:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,        # show_plots
#          True,        # useUnmodeledTorque
#          False,       # useIntGain
#          False        # useKnownTorque
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  Note that, as expected,
# the orientation error doesn't settle to zero, but rather converges to a non-zero offset
# proportional to the unmodeled torque being simulated.  Also, the control torques settle on
# non-zero steady-state values.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedbackNoEarth1100.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedbackNoEarth2100.svg "Torque history")
#
# Setup 3
# ------
#
# The 3rd scenario turns on both the unmodeled external torque and the integral
# feedback term:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,        # show_plots
#          True,        # useUnmodeledTorque
#          False,       # useIntGain
#          False        # useKnownTorque
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  In this case
# the orientation error does settle to zero.  The integral term changes the control torque
# to settle on a value that matches the unmodeled external torque.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedbackNoEarth1110.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedbackNoEarth2110.svg "Torque history")
#
# Setup 4
# ------
#
# The 4th scenario turns on the unmodeled external torque but keeps the integral
# feedback term off.  Instead, the external disturbance is fed forward in the
# attitude control solution.
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,        # show_plots
#          True,        # useUnmodeledTorque
#          False,       # useIntGain
#          True         # useKnownTorque
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  In this case
# the orientation error does settle to zero as the feedforward term compensates for
# the external torque.  The control torque is now caused
# to settle on a value that matches the unmodeled external torque.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedbackNoEarth1101.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedbackNoEarth2101.svg "Torque history")
#
##  @}
def run(show_plots, useUnmodeledTorque, useIntGain, useKnownTorque):
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
    scSim.AddModelToTask(simTaskName, scObject)


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
    mrpControlConfig.outputDataName = extFTObject.cmdTorqueInMsgName
    mrpControlConfig.K = 3.5
    if useIntGain:
        mrpControlConfig.Ki = 0.0002  # make value negative to turn off integral feedback
    else:
        mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1
    mrpControlConfig.domega0 = [0.0, 0.0, 0.0]
    if useKnownTorque:
        mrpControlConfig.knownTorquePntB_B = [0.25, -0.25, 0.1]

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 50
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(mrpControlConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)

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

    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

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
    dataLr = scSim.pullMessageLogData(mrpControlConfig.outputDataName + ".torqueRequestBody", range(3))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".sigma_BR", range(3))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".omega_BR_B", range(3))
    dataPos = scSim.pullMessageLogData(sNavObject.outputTransName + ".r_BN_N", range(3))
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileName = os.path.basename(os.path.splitext(__file__)[0])

    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(1, 4):
        plt.plot(dataSigmaBR[:, 0] * macros.NANO2MIN, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1" + str(int(useUnmodeledTorque)) + str(int(useIntGain))+ str(int(useKnownTorque))
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(1, 4):
        plt.plot(dataLr[:, 0] * macros.NANO2MIN, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    pltName = fileName + "2" + str(int(useUnmodeledTorque)) + str(int(useIntGain)) + str(int(useKnownTorque))
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(1, 4):
        plt.plot(dataOmegaBR[:, 0] * macros.NANO2MIN, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    print dataPos

    return dataPos, dataSigmaBR, dataLr, numDataPoints, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False,  # useUnmodeledTorque
        False,  # useIntGain
        False  # useKnownTorque
    )
