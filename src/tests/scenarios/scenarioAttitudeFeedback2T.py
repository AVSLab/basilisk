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
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit.
#           This scenario is the same as scenarioAttitudeControl, but with the
#           difference that here the control and dynamics are executed at different
#           frequencies or time steps.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 25, 2016
#


import pytest
import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import sim_model
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



## \defgroup Tutorials_2_0_2
##   @{
## Demonstrates how to stabilize the tumble of a spacecraft orbiting the
# Earth that is initially tumbling, but uses 2 separate threads.
#
# Attitude Detumbling Simulation in a Two Process Simulation Setup {#scenarioAttitudeFeedback2T}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth. This setup
# is similar to the [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback),
# but here the dynamics
# simulation and the Flight Software (FSW) algorithms are run at different time steps.
# The scenario is again
# setup to be run in three different setups:
# Scenarios Simulation Setup Cases
#
# Setup | useUnmodeledTorque  | useIntGain
# ----- | ------------------- | -------------
# 1     | False               | False
# 2     | True                | False
# 3     | True                | True
#
# To run the default scenario 1., call the python script through
#
#       python scenarioAttitudeFeedback2T.py
#
# When the simulation completes 3 plots are shown for the MRP attitude history, the rate
# tracking errors, as well as the control torque vector.
#
# The simulation layout is shown in the following illustration.  Both a simulation process is created
# which contains the spacecraft simulation modules.  A separate FSW algorithm process is run
# at a different updated rate to evaluate the Flight Software (FSW) algorithm
# modules.  Interface messages are now shared across SIM and FSW message passing interfaces (MPIs).
# ![Simulation Flow Diagram](Images/doc/test_scenarioAttitudeFeedback2T.svg "Illustration")
#
# A key difference to the 1-process setup is that after the processes are created, the
# dynamics and FSW messages system must be linked to connect messages with the same name.
# Note that the interface references are added to the process that they are SUPPLYING data
# to.  Reversing that setting is hard to detect as the data will still show up, it will just
# have a single frame of latency.  This is done in the following two-step process:
# ~~~~~~~~~~~~~~{.py}
#     dyn2FSWInterface = sim_model.SysInterface()
#     fsw2DynInterface = sim_model.SysInterface()
#
#     dyn2FSWInterface.addNewInterface(dynProcessName, fswProcessName)
#     fsw2DynInterface.addNewInterface(fswProcessName, dynProcessName)
#
#     dynProcess.addInterfaceRef(fsw2DynInterface)
#     fswProcess.addInterfaceRef(dyn2FSWInterface)
# ~~~~~~~~~~~~~~
# Next, after the simulation has been initialized and the modules messages are created
# a discover process must be called that links messages that have the same name.  This is
# achieved through the combined initialization and message discovery macro.
# ~~~~~~~~~~~~~~{.py}
#     scSim.InitializeSimulationAndDiscover()
# ~~~~~~~~~~~~~~
#
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,       # show_plots
#          False,       # useUnmodeledTorque
#          False        # useIntGain
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last 2 arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The
# default scenario has both the unmodeled torque and integral feedback turned off.  The
# resulting attitude and control torque histories are shown below.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback2T100.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2T200.svg "Torque history")
# Note that now the FSW algorithms are called in a separate process, in the first time step the
# navigation message has not been copied over, and the initial FSW values for the tracking
# errors are zero.  This is why there is a slight difference in the resulting closed loop
# performance.
#
# Setup 2
# ------
#
# Here the python main function is changed to read:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,       # show_plots
#          True,       # useUnmodeledTorque
#          False        # useIntGain
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  Note that, as expected,
# the orientation error doesn't settle to zero, but rather converges to a non-zero offset
# proportional to the unmodeled torque being simulated.  Also, the control torques settle on
# non-zero steady-state values.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback2T110.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2T210.svg "Torque history")
#
# Setup 3
# ------
#
# The final scenario turns on both the unmodeled external torque and the integral
# feedback term:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,       # show_plots
#          True,       # useUnmodeledTorque
#          True        # useIntGain
#        )
# ~~~~~~~~~~~~~
# The resulting attitude and control torques are shown below.  In this case
# the orientation error does settle to zero.  The integral term changes the control torque
# to settle on a value that matches the unmodeled external torque.
# ![MRP Attitude History](Images/Scenarios/scenarioAttitudeFeedback2T111.svg "MRP history")
# ![Control Torque History](Images/Scenarios/scenarioAttitudeFeedback2T211.svg "Torque history")
#
##  @}
def run(show_plots, useUnmodeledTorque, useIntGain):
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

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # unitTestSupport.enableVisualization(scSim, dynProcess, simProcessName, 'earth')  # The Viz only support 'earth', 'mars', or 'sun'

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
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

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
    scSim.AddModelToTask(dynTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simple_nav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(fswTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation
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
    mrpControlConfig.outputDataName = extFTObject.cmdTorqueInMsgName
    mrpControlConfig.K = 3.5
    if useIntGain:
        mrpControlConfig.Ki = 0.0002  # make value negative to turn off integral feedback
    else:
        mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(mrpControlConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputAttName, samplingTime)

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
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

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
    dataLr = scSim.pullMessageLogData(mrpControlConfig.outputDataName + ".torqueRequestBody", range(3))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".sigma_BR", range(3))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".omega_BR_B", range(3))
    dataPos = scSim.pullMessageLogData(sNavObject.outputTransName + ".r_BN_N", range(3))
    dataSigmaBN = scSim.pullMessageLogData(sNavObject.outputAttName + ".sigma_BN", range(3))
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
    pltName = fileName + "1" + str(int(useUnmodeledTorque)) + str(int(useIntGain))
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(1, 4):
        plt.plot(dataLr[:, 0] * macros.NANO2MIN, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    pltName = fileName + "2" + str(int(useUnmodeledTorque)) + str(int(useIntGain))
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(1, 4):
        plt.plot(dataOmegaBR[:, 0] * macros.NANO2MIN, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    plt.figure(4)
    for idx in range(1, 4):
        plt.plot(dataPos[:, 0] * macros.NANO2MIN, dataPos[:, idx] / 1000,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial Position [km] ')

    plt.figure(5)
    for idx in range(1, 4):
        plt.plot(dataSigmaBN[:, 0] * macros.NANO2MIN, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$\sigma_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial MRP Attitude ')

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return dataPos, dataSigmaBR, dataLr, numDataPoints, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False,  # useUnmodeledTorque
        False  # useIntGain
        )
