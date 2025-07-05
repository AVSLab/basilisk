# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder"

r"""
This scenario is adapted from scenarioAttitudeFeedback.py to compare the runtime of two filters:
1. (baseline) InertialUKF, and 2. (new) inertialAttitudeUKF.
See _Documents within for more details of each filter.

Runtime results are reported below:

Filter               |  Runtime (s)
-----------------------------------
InertialUKF          |        1.157
inertialAttitudeUKF  |        1.134
"""

import os
import time
import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    inertial3D, rwMotorTorque, rwMotorVoltage)
from Basilisk.simulation import reactionWheelStateEffector, motorVoltageInterface, simpleNav, spacecraft
from Basilisk.utilities import (SimulationBaseClass, fswSetupRW, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)
from Basilisk.fswAlgorithms import inertialUKF
# from Basilisk.fswAlgorithms import inertialAttitudeUkf

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

def setup_filter_data(filterObject):
    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0
    filterObject.switchMag = 1.2

    filterObject.stateInit = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    filterObject.covarInit = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    qNoise = np.identity(6)
    qNoise[0:3, 0:3] = qNoise[0:3, 0:3]*0.0017*0.0017
    qNoise[3:6, 3:6] = qNoise[3:6, 3:6]*0.00017*0.00017
    filterObject.qNoise = qNoise.reshape(36).tolist()

def run(showPlots, useJitterSimple, useRWVoltageIO, filterType, simTime=20.0):
    """
    The scenarios can
    be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots
        useJitterSimple (bool): Specify if the RW simple jitter model should be included
        useRWVoltageIO (bool): Specify if the RW voltage interface should be simulated.
        filterType (str): {'UKF', 'AttUKF'}
        simTime (float): The length of the simulation time in minutes

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(simTime)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"

    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, 1)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = messaging.BalancedWheels
    if useJitterSimple:
        varRWModel = messaging.JitterSimple

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
    # In this simulation the RW objects RW1, RW2 or RW3 are not modified further.  However, you can over-ride
    # any values generate in the `.create()` process using for example RW1.Omega_max = 100. to change the
    # maximum wheel speed.

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process.  This is required for the updateState() method
    # to be called which logs the RW states
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)

    if useRWVoltageIO:
        rwVoltageIO = motorVoltageInterface.MotorVoltageInterface()
        rwVoltageIO.ModelTag = "rwVoltageInterface"

        # set module parameters(s)
        rwVoltageIO.setGains(np.array([0.2 / 10.] * 3)) # [Nm/V] conversion gain

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, rwVoltageIO)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.K = 3.5
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)

    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B

    if useRWVoltageIO:
        fswRWVoltage = rwMotorVoltage.rwMotorVoltage()
        fswRWVoltage.ModelTag = "rwMotorVoltage"

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, fswRWVoltage)

        # set module parameters
        fswRWVoltage.VMin = 0.0  # Volts
        fswRWVoltage.VMax = 10.0  # Volts

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, rwMotorLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, snTransLog)

    # To log the RW information, the following code is used:
    mrpLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, mrpLog)

    # A message is created that stores an array of the \f$\Omega\f$ wheel speeds.  This is logged
    # here to be plotted later on.  However, RW specific messages are also being created which
    # contain a wealth of information.  The vector of messages is ordered as they were added.  This
    # allows us to log RW specific information such as the actual RW motor torque being applied.
    rwLogs = []
    for item in range(numRW):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, rwLogs[item])
    if useRWVoltageIO:
        rwVoltLog = fswRWVoltage.voltageOutMsg.recorder(samplingTime)
        scSim.AddModelToTask(simTaskName, rwVoltLog)

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Two options are shown to setup the FSW RW configuration message.
    # First case: The FSW RW configuration message
    # uses the same RW states in the FSW algorithm as in the simulation.  In the following code
    # the fswSetupRW helper functions are used to individually add the RW states.  The benefit of this
    # method of the second method below is that it is easy to vary the FSW parameters slightly from the
    # simulation parameters.  In this script the second method is used, while the fist method is included
    # to show both options.
    fswSetupRW.clearSetup()
    for key, rw in rwFactory.rwList.items():
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    fswRwParamMsg1 = fswSetupRW.writeConfigMessage()

    # Second case: If the exact same RW configuration states are to be used by the simulation and fsw, then the
    # following helper function is convenient to extract the fsw RW configuration message from the
    # rwFactory setup earlier.
    fswRwParamMsg2 = rwFactory.getConfigMessage()
    fswRwParamMsg = fswRwParamMsg2

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
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              # , saveFile=fileName
                                              , rwEffectorList=rwStateEffector
                                              )
    # link messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    if useRWVoltageIO:
        fswRWVoltage.torqueInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
        fswRWVoltage.rwParamsInMsg.subscribeTo(fswRwParamMsg)
        rwVoltageIO.motorVoltageInMsg.subscribeTo(fswRWVoltage.voltageOutMsg)
        rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwVoltageIO.motorTorqueOutMsg)
    else:
        rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

    #
    #   [test] 1 star tracker
    #
    stMessage1 = messaging.STAttMsgPayload()
    stMessage1.timeTag = 0
    stMessage1.MRP_BdyInrtl = [0.3, 0.4, 0.5]
    st1InMsg = messaging.STAttMsg().write(stMessage1)

    #
    #   [test] setup the FSW algorithm tasks
    #
    if(filterType == 'UKF'):
        attEstimator = inertialUKF.inertialUKF()
        scSim.AddModelToTask(simTaskName, attEstimator)
        setup_filter_data(attEstimator)

        ST1Data = inertialUKF.STMessage()
        ST1Data.noise = [0.00017 * 0.00017, 0.0, 0.0,
                         0.0, 0.00017 * 0.00017, 0.0,
                         0.0, 0.0, 0.00017 * 0.00017]

        STList = [ST1Data]
        attEstimator.STDatasStruct.STMessages = STList
        attEstimator.STDatasStruct.numST = len(STList)
        attEstimator.STDatasStruct.STMessages[0].stInMsg.subscribeTo(st1InMsg)

        gyroInMsg = messaging.AccDataMsg()
        attEstimator.gyrBuffInMsg.subscribeTo(gyroInMsg)
        attEstimator.massPropsInMsg.subscribeTo(vcMsg)
        attEstimator.rwParamsInMsg.subscribeTo(fswRwParamMsg)
        attEstimator.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)

        att_estimatorLog = attEstimator.logger(["covar", "state"], simulationTimeStep)
        scSim.AddModelToTask(simTaskName, att_estimatorLog)
    elif(filterType == 'AttUKF'):
        print("inertialAttitudeUKF not yet included")
        # starOnly = inertialAttitudeUkf.AttitudeFilterMethod_StarOnly
        # attEstimator = inertialAttitudeUkf.InertialAttitudeUkf(starOnly)
        # scSim.AddModelToTask(simTaskName, attEstimator)
        # setup_filter_data(attEstimator)
        #
        # ST1Data = inertialAttitudeUkf.StarTrackerMessage()
        # ST1Data.measurementNoise = [[0.00017 * 0.00017, 0.0, 0.0],
        #                  [0.0, 0.00017 * 0.00017, 0.0],
        #                  [0.0, 0.0, 0.00017 * 0.00017]]
        # ST1Data.starTrackerMsg.subscribeTo(st1InMsg)
        # attEstimator.addStarTrackerInput(ST1Data)
        #
        # gyroInMsg = messaging.AccDataMsg()
        # attEstimator.accelDataMsg.subscribeTo(gyroInMsg)
        # attEstimator.vehicleConfigMsg.subscribeTo(vcMsg)
        # attEstimator.rwArrayConfigMsg.subscribeTo(fswRwParamMsg)
        # attEstimator.rwSpeedMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
        #
        # att_estimatorLog = attEstimator.inertialFilterOutputMsg.recorder()
        # scSim.AddModelToTask(simTaskName, att_estimatorLog)
    else:
        # assert error message: non-implemented filter
        raise NotImplementedError("non-implemented filter")

    #
    #   true attitude log
    #
    attNavLog = sNavObject.attOutMsg.recorder(simulationTimeStep)
    scSim.AddModelToTask(simTaskName, attNavLog)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   [test] configure a simulation stop time and execute the simulation run
    #
    currentTime = 0
    startTime = time.time()
    while currentTime < simulationTime:
        nav_payload = sNavObject.attOutMsg.read()
        stMessage1.timeTag = currentTime
        stMessage1.MRP_BdyInrtl = nav_payload.sigma_BN + np.random.normal(0, 5e-3, 3)
        st1InMsg.write(stMessage1, currentTime)

        stopTime = currentTime + simulationTimeStep
        scSim.ConfigureStopTime(stopTime)
        scSim.ExecuteSimulation()
        currentTime = stopTime
    endTime = time.time() - startTime

    #
    #   plot logged data
    #
    if showPlots:
        plt.figure()
        plt.plot(att_estimatorLog.times()*macros.NANO2MIN, att_estimatorLog.state,
                 linestyle="-", linewidth=3.0, alpha=0.6)
        plt.plot(attNavLog.times()*macros.NANO2MIN, attNavLog.sigma_BN, linestyle="--", color='k')
        plt.plot(attNavLog.times()*macros.NANO2MIN, attNavLog.omega_BN_B, linestyle="--", color='k')
        plt.xlabel('t, min')
        plt.legend([r'$\hat{x}_1$',r'$\hat{x}_2$',r'$\hat{x}_3$',
                    r'$\hat{x}_4$',r'$\hat{x}_5$',r'$\hat{x}_6$',
                    r'$x_1$',r'$x_2$',r'$x_3$',
                    r'$x_4$',r'$x_5$',r'$x_6$'],
                   ncol=2)
        plt.title(filterType)
        plt.grid()
        plt.show()

    return endTime


def compare_runtime(showPlots=False):
    runtimeUkf = run(showPlots, True, True, 'UKF')
    # runtimeAttukf = run(showPlots, True, True, 'AttUKF')

    # static width version
    colWidth = 20

    # Header
    print(f"{'Filter':<{colWidth}} | {'Runtime (s)':>12}")
    print("-" * (colWidth + 3 + 12))

    # Rows
    print(f"{'InertialUKF':<{colWidth}} | {runtimeUkf:12.3f}")
    # print(f"{'inertialAttitudeUKF':<{colWidth}} | {runtimeAttukf:12.3f}")

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    compare_runtime(showPlots=True)