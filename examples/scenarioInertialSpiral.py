#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

Discusses how to layer attitude references. Starts with an inertial pointing attitude and then adds a spiral scanning motion on top of it.


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the attitude navigation modules with eulerRotation()
#           module to demonstrate layering attitude references.
# Author:   Galen Bascom
# Creation Date:  ___ ___, 2022
#

import sys
import os
import numpy as np
np.set_printoptions(precision=16)

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simpleNav

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import eulerRotation

# import message declarations
from Basilisk.architecture import messaging

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenarios can be run with the followings parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Define simulation run time and integration time step
    simulationTime = macros.min2nano(10.)
    simulationTimeStep = macros.sec2nano(0.1)

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # se tup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # set up extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    # use the input flag to determine which external torque should be applied
    # Note that all variables are initialized to zero.  Thus, not setting this
    # vector would leave its components all zero for the simulation.
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   set up the FSW algorithm tasks
    #

    # set up inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    attGuidanceConfigEuler1 = eulerRotation.eulerRotationConfig()
    attGuidanceWrapEuler1 = scSim.setModelDataWrap(attGuidanceConfigEuler1)
    attGuidanceWrapEuler1.ModelTag = "eulerRotation1"
    attGuidanceConfigEuler1.attRefInMsg.subscribeTo(inertial3DConfig.attRefOutMsg)
    scSim.AddModelToTask(simTaskName, attGuidanceWrapEuler1, attGuidanceConfigEuler1)
    
    attGuidanceConfigEuler2 = eulerRotation.eulerRotationConfig()
    attGuidanceWrapEuler2 = scSim.setModelDataWrap(attGuidanceConfigEuler2)
    attGuidanceWrapEuler2.ModelTag = "eulerRotation2"
    attGuidanceConfigEuler2.attRefInMsg.subscribeTo(attGuidanceConfigEuler1.attRefOutMsg)
    scSim.AddModelToTask(simTaskName, attGuidanceWrapEuler2, attGuidanceConfigEuler2)

    # Set up Euler angles.
    # Make rotation 1 be 0.02 rad/s about 1 axis
    # Make rotation 2 be 0.0001 rad/s about 2 axis
    
    attGuidanceConfigEuler1.angleRates = [0,0,.02]
    attGuidanceConfigEuler2.angleRates = [0,0.0001,0]
    
    # set up the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attError"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)

    # set up the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    #
    # create simulation messages
    #
    # The MRP Feedback algorithm requires the vehicle configuration structure. This defines various spacecraft
    # related states such as the inertia tensor and the position vector between the primary Body-fixed frame
    # B origin and the center of mass (defaulted to zero).  The message payload is created through
    configData = messaging.VehicleConfigMsgPayload()
    configData.ISCPntB_B = I
    # Two methods are shown to create either a C++ or C wrapped msg object in python.  The
    # preferred method is to just create C++ wrapped messages.
    configDataMsg = messaging.VehicleConfigMsg()
    configDataMsg.write(configData)

    #
    # connect the messages to the modules
    #
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attErrorConfig.attRefInMsg.subscribeTo(attGuidanceConfigEuler2.attRefOutMsg)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(configDataMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    #
    # Set up data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    snLog = scObject.scStateOutMsg.recorder(samplingTime)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    # instead of recording the contents of a C++ output message, you can also recording
    # the incoming contents of a C++ input message.  However, note that you must set up the
    # input message recorder after this input message has been subscribed to another message.
    # Otherwise, you are reading an uninitialized msg which leads to lovely segmentation faults.
    snLog = sNavObject.scStateInMsg.recorder(samplingTime)
    attErrorLog = attErrorConfig.attGuidOutMsg.recorder(samplingTime)
    mrpLog = mrpControlConfig.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, snLog)
    scSim.AddModelToTask(simTaskName, snAttLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, mrpLog)

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
    scObject.hub.sigma_BNInit = [[0.], [0.], [0.]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.], [0.], [0.]]  # rad/s - omega_BN_B

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    dataSigmaBN = snAttLog.sigma_BN
    
    dataEulerAnglesPitch = list()
    dataEulerAnglesYaw = list()
    dataEulerAnglesRoll = list()
    for sigma in dataSigmaBN:
        eulerAngle = RigidBodyKinematics.MRP2Euler321(sigma)
        dataEulerAnglesPitch.append(eulerAngle[0])
        dataEulerAnglesYaw.append(eulerAngle[1])
        dataEulerAnglesRoll.append(eulerAngle[2])
        
    timeLineSet = attErrorLog.times() * macros.NANO2MIN
    #
    #   plot the results
    #
    timeAxis = attErrorLog.times()
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attErrorLog.sigma_BR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, mrpLog.torqueRequestBody[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='upper right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Control Torque $L_r$ [Nm]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attErrorLog.omega_BR_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    plt.figure(4)
    plt.plot(timeLineSet, dataEulerAnglesYaw,
                 color=unitTestSupport.getLineColor(0, 3),label=r'Yaw')
    plt.plot(timeLineSet, dataEulerAnglesPitch,
                 color=unitTestSupport.getLineColor(1, 3),label=r'Pitch')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Euler Angles [rad]')

    plt.figure(5)
    plt.plot(dataEulerAnglesPitch, dataEulerAnglesYaw)
    plt.xlabel('Pitch [rad]')
    plt.ylabel('Yaw [rad]')

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
        True  # show_plots
    )
