#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This script shows how to perform momentum dumping when the momentum accumulated on the reaction wheels
is above a user-defined threshold. In this case, such threshold is set at 80 Nms. The dumping is performed 
by a set of 8 thrusters that can provide control about the three principal axes of the spacecraft. 
To perform the momentum dumping, three concatenated modules are used:

- :ref:`thrMomentumManagement`: computes the amount of momentum to be dumped, based on current stored momentum 
  and the user-defined threshold. It is important to notice that, for the three concatenated modules to work
  correctly, this first module cannot be run at simulation time :math:`t = 0`. In this script, the method 
  ``Reset`` is called on :ref:`thrMomentumManagement` at :math:`t = 10` s, which coincides to the time at which 
  the first desaturating impulse is fired.
- :ref:`thrForceMapping`: maps the amout of momentum to be dumped into impulses that must be delivered by each
  thruster. This module is originally implemented to map a requested torque into forces imparted by the thrusters,
  but it can be applied in this case as well, because the math is the same. The only caveat is that, in this case,
  the output should not be scaled by the thruster maximum torque capability, since the desired output is an impulse
  and not a torque. To deactivate the output scaling, the ``angErrThresh`` input variable for this module must be 
  set to a value larger than :math:`\pi`, as specified in the module documentation.
- :ref:`thrMomentumDumping`: computes the thruster on-times required to deliver the desired impulse. A 
  ``maxCounterValue`` of 100 is used in this example to allow the spacecraft to maneuver back to the desired attitude
  after each time the thrusters fire. 

For this script to work as intended, it is necessary to run the flight software and the dynamics at two different 
frequencies. In this example, the simulation time step for the flight software is 1 second, whereas for the dynamics
it is 0.1 seconds. This is necessary because the :ref:`thrMomentumDumping` automatically uses the task time step as 
control period for the firing. However, if the dynamics is integrated at the same frequency, this does not give 
enough time resolution to appreciate the variation in the momentum.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioMomentumDumping.py

Illustration of Simulation Results
----------------------------------

In this examples, the spacecraft is already at the desired attitude, but the four reaction wheels are saturated (the total
angular momentum exceeds the threshold). The desaturation happens at :math:`t = 10` when the :ref:`thrMomentumManagement` is
reset. Three firings are sufficient to dump the momentum below the set threshold. The following figures illustrate the change
in momentum for the four wheels :math:`H_i` for :math:`i = 1,...,4` and the total angular momentum :math:`\|H\|`, and the 
attitude errors, as functions of time, with respect to the desired target attitude.

.. image:: /_images/Scenarios/scenarioMomentumDumping3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMomentumDumping1.svg
   :align: center

The plots show that the momentum is dumped below the threshold. Also, the desired attitude is recovered between the first and
second firing, and after the third, but between the second and the third there is not enough time for the spacecraft to slew
back to that attitude.

The next two plots show the amount of impulse [Ns] requested for each thruster, and the times during which each thruster is 
operational. As expected, 100 control times pass between each firing: because the control time coincides with the flight 
software simulation time step of 1 s, this means that firings are 100 seconds apart.

.. image:: /_images/Scenarios/scenarioMomentumDumping5.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMomentumDumping7.svg
   :align: center

"""

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError, inertial3D, rwMotorTorque,
                                    thrMomentumManagement, thrForceMapping, thrMomentumDumping)
from Basilisk.simulation import (reactionWheelStateEffector, thrusterDynamicEffector, simpleNav, spacecraft)
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, simIncludeThruster, unitTestSupport, vizSupport)

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])



def run(show_plots):

    # Create simulation variable names
    fswTask = "fswTask"
    dynTask = "dynTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    scSim.SetProgressBar(False)

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the simulation time and integration update time
    simulationTime = macros.min2nano(5)
    simulationTimeStepFsw = macros.sec2nano(1)
    simulationTimeStepDyn = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(dynTask, simulationTimeStepDyn))
    dynProcess.addTask(scSim.CreateNewTask(fswTask, simulationTimeStepFsw))
    
    #
    # setup the simulation tasks/objects
    # 

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "Max-SC"

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(dynTask, scObject, None, 1)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # Next a series of gravitational bodies are included
    gravBodies = gravFactory.createBodies(['earth', 'sun'])
    planet = gravBodies['earth']
    planet.isCentralBody = True

    mu = planet.mu
    # The configured gravitational bodies are added to the spacecraft dynamics with the usual command:
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Next, the default SPICE support module is created and configured.
    timeInitString = "2022 JUNE 27 00:00:00.0"

    # The following is a support macro that creates a `gravFactory.spiceObject` instance
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)

    # Earth is gravity center
    gravFactory.spiceObject.zeroBase = 'Earth'

    # The SPICE object is added to the simulation task list.
    scSim.AddModelToTask(fswTask, gravFactory.spiceObject, None, 2)

    # The gravitational body is connected to the spacecraft object
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 7000. * 1000      # meters
    oe.e = 0.0001
    oe.i = 33.3 * macros.D2R
    oe.Omega = 148.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 335 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN                          # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN                          # m/s - v_BN_N
    scObject.hub.sigma_BNInit = [0, 0., 0.]              # MRP set to customize initial inertial attitude
    scObject.hub.omega_BN_BInit = [[0.], [0.], [0.]]      # rad/s - omega_CN_B
    
    # define the simulation inertia
    I = [1700,  0.,    0.,
         0.,    1700,  0.,
         0.,    0.,    1800]
    scObject.hub.mHub = 2500  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.28]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    #
    # add RW devices
    #
    # Make RW factory instance
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = messaging.BalancedWheels

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    c = 2**(-0.5)
    RW1 = rwFactory.create('Honeywell_HR16', [c, 0, c], maxMomentum=100., Omega=4000.  # RPM
                           , RWModel=varRWModel)
    RW2 = rwFactory.create('Honeywell_HR16', [0, c, c], maxMomentum=100., Omega=2000.  # RPM
                           , RWModel=varRWModel)
    RW3 = rwFactory.create('Honeywell_HR16', [-c, 0, c], maxMomentum=100., Omega=3500.  # RPM
                           , RWModel=varRWModel)
    RW4 = rwFactory.create('Honeywell_HR16', [0, -c, c], maxMomentum=100., Omega=0.  # RPM
                           , RWModel=varRWModel)

    numRW = rwFactory.getNumOfDevices()
    RW = [RW1, RW2, RW3, RW4]

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(dynTask, rwStateEffector, None, 2)

    # Setup the FSW RW configuration message.
    fswRwParamMsg = rwFactory.getConfigMessage()

    # create arrays for thrusters' locations and directions
    a = 1
    b = 1.28
    location = [
        [-a, -a,  b],
        [ a, -a, -b],
        [ a, -a,  b],
        [ a,  a, -b],
        [ a,  a,  b],
        [-a,  a, -b],
        [-a,  a,  b],
        [-a, -a, -b]
    ]
    direction = [
        [ 1,  0,  0],
        [-1,  0,  0],
        [ 0,  1,  0],
        [ 0, -1,  0],
        [-1,  0,  0],
        [ 1,  0,  0],
        [ 0, -1,  0],
        [ 0,  1,  0]
    ]

    # create the set of thruster in the dynamics task
    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    scSim.AddModelToTask(dynTask, thrusterSet)

    # Make a fresh thruster factory instance, this is critical to run multiple times
    thFactory = simIncludeThruster.thrusterFactory()

    # create the thruster devices by specifying the thruster type and its location and direction
    for pos_B, dir_B in zip(location, direction):
        thFactory.create('MOOG_Monarc_5', pos_B, dir_B, MaxThrust=5.0)

    # get number of thruster devices
    numTh = thFactory.getNumOfDevices()

    # create thruster object container and tie to spacecraft object
    thrModelTag = "ACSThrusterDynamics"
    thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)

    # add the simple Navigation sensor module
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTask, sNavObject)

    # Setup the FSW thruster configuration message.
    fswThrConfigMsg = thFactory.getConfigMessage()

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(fswTask, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(fswTask, attErrorWrap, attErrorConfig)

    # setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(fswTask, mrpControlWrap, mrpControlConfig)
    decayTime = 10.0
    xi = 1.0
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 3*np.max(I)/decayTime
    mrpControlConfig.K = (mrpControlConfig.P/xi)*(mrpControlConfig.P/xi)/np.max(I)
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
    rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
    rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(fswTask, rwMotorTorqueWrap, rwMotorTorqueConfig)
    # Make the RW control all three body axes
    rwMotorTorqueConfig.controlAxes_B = controlAxes_B

    # Momentum dumping configuration
    thrDesatControlConfig = thrMomentumManagement.thrMomentumManagementConfig()
    thrDesatControlWrap = scSim.setModelDataWrap(thrDesatControlConfig)
    thrDesatControlWrap.ModelTag = "thrMomentumManagement"
    scSim.AddModelToTask(fswTask, thrDesatControlWrap, thrDesatControlConfig)
    thrDesatControlConfig.hs_min = 80   # Nms  :  maximum wheel momentum 

    # setup the thruster force mapping module
    thrForceMappingConfig = thrForceMapping.thrForceMappingConfig()
    thrForceMappingWrap = scSim.setModelDataWrap(thrForceMappingConfig)
    thrForceMappingWrap.ModelTag = "thrForceMapping"
    scSim.AddModelToTask(fswTask, thrForceMappingWrap, thrForceMappingConfig)
    thrForceMappingConfig.controlAxes_B = controlAxes_B
    thrForceMappingConfig.thrForceSign = 1
    thrForceMappingConfig.angErrThresh = 3.15    # this needs to be larger than pi (180 deg) for the module to work in the momentum dumping scenario

    # setup the thruster momentum dumping module
    thrDumpConfig = thrMomentumDumping.thrMomentumDumpingConfig()
    thrDumpWrap = scSim.setModelDataWrap(thrDumpConfig)
    thrDumpWrap.ModelTag = "thrDump"
    scSim.AddModelToTask(fswTask, thrDumpWrap, thrDumpConfig)
    thrDumpConfig.maxCounterValue = 100          # number of control periods (simulationTimeStepFsw) to wait between two subsequent on-times
    thrDumpConfig.thrMinFireTime = 0.02          # thruster firing resolution

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, dynTask, scObject
                                              # , saveFile=fileName
                                              , rwEffectorList=rwStateEffector
                                              , thrEffectorList=thrusterSet
                                              )
    vizSupport.setActuatorGuiSetting(viz, viewRWPanel=True,
                                     viewRWHUD=True,
                                     viewThrusterPanel=True,
                                     viewThrusterHUD=True
                                     )

    # link messages
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attErrorConfig.attRefInMsg.subscribeTo(inertial3DConfig.attRefOutMsg)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControlConfig.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControlConfig.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwMotorTorqueConfig.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueConfig.vehControlInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueConfig.rwMotorTorqueOutMsg)
    thrDesatControlConfig.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    thrDesatControlConfig.rwConfigDataInMsg.subscribeTo(fswRwParamMsg)
    thrForceMappingConfig.cmdTorqueInMsg.subscribeTo(thrDesatControlConfig.deltaHOutMsg)
    thrForceMappingConfig.thrConfigInMsg.subscribeTo(fswThrConfigMsg)
    thrForceMappingConfig.vehConfigInMsg.subscribeTo(vcMsg)
    thrDumpConfig.thrusterConfInMsg.subscribeTo(fswThrConfigMsg)
    thrDumpConfig.deltaHInMsg.subscribeTo(thrDesatControlConfig.deltaHOutMsg)
    thrDumpConfig.thrusterImpulseInMsg.subscribeTo(thrForceMappingConfig.thrForceCmdOutMsg)
    thrusterSet.cmdsInMsg.subscribeTo(thrDumpConfig.thrusterOnTimeOutMsg)


    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 5000
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStepDyn, numDataPoints)
    sNavRec = sNavObject.attOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, sNavRec)
    dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, dataRec)
    rwMotorLog = rwMotorTorqueConfig.rwMotorTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, rwMotorLog)
    attErrorLog = attErrorConfig.attGuidOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, attErrorLog)
    deltaHLog  = thrDesatControlConfig.deltaHOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, deltaHLog)
    thrMapLog = thrForceMappingConfig.thrForceCmdOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, thrMapLog)
    onTimeLog = thrDumpConfig.thrusterOnTimeOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, onTimeLog)
    # thrLog = fswThrConfigMsg.recorder(samplingTime)
    # scSim.AddModelToTask(simTaskName, thrLog)

    # To log the RW information, the following code is used:
    mrpLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(dynTask, mrpLog)

    # A message is created that stores an array of the Omega wheel speeds
    rwLogs = []
    for item in range(numRW):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
        scSim.AddModelToTask(dynTask, rwLogs[item])

    # To log the thruster information, the following code is used:
    thrLog = []
    for item in range(numTh):
        thrLog.append(thrusterSet.thrusterOutMsgs[item].recorder(samplingTime))
        scSim.AddModelToTask(dynTask, thrLog[item])

    # initialize Simulation:  This function runs the self_init()
    # cross_init() and reset() routines on each module.
    scSim.InitializeSimulation()

    # configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(10.0))
    scSim.ExecuteSimulation()

    # reset thrDesat module after 10 seconds because momentum cannot be dumped at t = 0
    thrDesatControlWrap.Reset(macros.sec2nano(10.0))

    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve the logged data

    # RW
    dataUsReq = rwMotorLog.motorTorque
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    dataOmegaRW = mrpLog.wheelSpeeds
    dataDH = deltaHLog.torqueRequestBody
    dataMap = thrMapLog.thrForce
    dataOnTime = onTimeLog.OnTimeRequest

    dataRW = []
    for i in range(numRW):
        dataRW.append(rwLogs[i].u_current)

    dataThr = []
    for i in range(numTh):
        dataThr.append(thrLog[i].thrustForce)

    np.set_printoptions(precision=16)


    # Displays the plots relative to the S/C attitude and rates errors, wheel momenta, thruster impulses, on times, and thruster firing intervals  
    
    timeData = rwMotorLog.times() * macros.NANO2SEC

    plot_attitude_error(timeData, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_rate_error(timeData, dataOmegaBR)
    pltName = fileName + "2"    
    figureList[pltName] = plt.figure(2)

    plot_rw_momenta(timeData, dataOmegaRW, RW, numRW)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plot_DH(timeData, dataDH)
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plot_thrImpulse(timeData, dataMap, numTh)
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

    plot_OnTimeRequest(timeData, dataOnTime, numTh)
    pltName = fileName + "6"
    figureList[pltName] = plt.figure(6)

    plot_thrForce(timeData, dataThr, numTh)
    pltName = fileName + "7"
    figureList[pltName] = plt.figure(7)

    if show_plots:  
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


# Plotting RW functions
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

def plot_rate_error(timeData, dataOmegaBR):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s) ')

def plot_rw_momenta(timeData, dataOmegaRw, RW, numRW):
    """Plot the RW momenta."""
    totMomentumNorm = []
    for j in range(len(timeData)):
        totMomentum = np.array([0,0,0])
        for idx in range(numRW):
            for k in range(3):
                totMomentum[k] = totMomentum[k] + dataOmegaRw[j, idx] * RW[idx].Js * RW[idx].gsHat_B[k][0]
        totMomentumNorm.append(np.linalg.norm(totMomentum))
    plt.figure(3)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRw[:, idx] * RW[idx].Js,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$H_' + str(idx+1) + '}$')
    plt.plot(timeData, totMomentumNorm, '--',
             label=r'$\|H\|$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Momentum (Nms)')

def plot_DH(timeData, dataDH):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(4)
    for idx in range(3):
        plt.plot(timeData, dataDH[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\Delta H_' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Dumped momentum (Nms) ')

def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    """Plot the RW spin rates."""
    plt.figure(5)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')

def plot_thrImpulse(timeDataFSW, dataMap, numTh):
    """Plot the Thruster force values."""
    plt.figure(5)
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataMap[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label='$thrImpulse,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Impulse requested [Ns]')

def plot_OnTimeRequest(timeData, dataOnTime, numTh):
    """Plot the thruster on time requests."""
    plt.figure(6)
    for idx in range(numTh):
        plt.plot(timeData, dataOnTime[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label='$OnTimeRequest,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('OnTimeRequest [sec]')

def plot_thrForce(timeDataFSW, dataThr, numTh):
    """Plot the Thruster force values."""
    plt.figure(7)
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataThr[idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label='$thrForce,' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Thruster force [N]')

if __name__ == "__main__":
    run(True)
