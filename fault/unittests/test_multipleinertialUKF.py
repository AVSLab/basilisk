"""
Test if the inertialUKF is setup properly to estimate attitude states (MRP, angularrate)
"""

import os
import pytest
import matplotlib.pyplot as plt
import numpy as np
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
from scipy.stats import chi2
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    inertial3D, rwMotorTorque)
from Basilisk.simulation import reactionWheelStateEffector, simpleNav, spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)
from Basilisk.fswAlgorithms import inertialUKF

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


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

def plot_filter_result_sigma(filter_key, timeData, state, state_est, cov_est):
    timeData = timeData[1:]
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    
    for idx, ax in enumerate(axs):
        color = unitTestSupport.getLineColor(idx, 3)
        
        # True state
        ax.plot(timeData, state[1:, idx],
                color=color,
                label=rf'$x_{{{idx+1}}}$')
        
        # Estimated state
        ax.plot(timeData, state_est[1:, idx],
                color=color,
                linestyle='--',
                label=rf'$\hat{{x}}_{{{idx+1}}}$')
        
        # ±6 std‐dev band
        std5  = 6 * np.sqrt(cov_est[1:, idx])
        upper = state_est[1:, idx] + std5
        lower = state_est[1:, idx] - std5
        ax.fill_between(timeData, lower, upper,
                        color=color,
                        alpha=0.3,
                        label=r'$\pm6\sigma$')
        
        ax.set_ylabel(rf'$x_{{{idx+1}}}$')      # y-label per subplot
        ax.legend(loc='upper right', fontsize='small')
        # margin = 0.0
        # ax.set_ylim([state[:, idx].min()-margin, state[:, idx].max()+margin])
    # Common x‑label on the bottom subplot
    axs[-1].set_xlabel('Time [min]')
    fig.suptitle("["+filter_key+" filter] result", fontsize=14)

def plot_filter_result_omega(filter_key, timeData, state, state_est, cov_est):
    timeData = timeData[1:]
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    
    for idx, ax in enumerate(axs):
        color = unitTestSupport.getLineColor(idx, 3)
        
        # True state
        ax.plot(timeData, state[1:, idx],
                color=color,
                label=rf'$x_{{{idx+3}}}$')
        
        # Estimated state
        ax.plot(timeData, state_est[1:, idx],
                color=color,
                linestyle='--',
                label=rf'$\hat{{x}}_{{{idx+3}}}$')
        
        # ±5 std‐dev band
        std5  = 6 * np.sqrt(cov_est[1:, idx])
        upper = state_est[1:, idx] + std5
        lower = state_est[1:, idx] - std5
        ax.fill_between(timeData, lower, upper,
                        color=color,
                        alpha=0.3,
                        label=r'$\pm6\sigma$')
        
        ax.set_ylabel(rf'$x_{{{idx+3}}}$')      # y-label per subplot
        ax.legend(loc='upper right', fontsize='small')
        # ax.set_ylim([state[:, idx].min(), state[:, idx].max()])
    # Common x‑label on the bottom subplot
    axs[-1].set_xlabel('Time [min]')
    fig.suptitle("["+filter_key+" filter] result", fontsize=14)

def plot_rw_cmd_torque(timeData, dataUsReq, numRW):
    """Plot the RW command torques."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW):
    """Plot the RW actual motor torques."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
        plt.plot(timeData, dataRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')

def plot_rate_error(timeData, dataOmegaBR):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s) ')

def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    """Plot the RW spin rates."""
    plt.figure(4)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')

def plot_filter_chisquare(dataChiSquare):
    p = 0.05      # confidence level
    dof = 3       # degrees of freedom
    chi_ub = chi2.ppf(1-0.5*p, dof)
    chi_lb = chi2.ppf(0.5*p, dof)
    plt.figure()
    plt.scatter(np.arange(len(dataChiSquare)), dataChiSquare, color="black", s=10, label=r"$\chi^2$")
    plt.axhline(y=chi_ub, color='r', linestyle='--', label=r"$\chi^2$ upper threshold")
    plt.axhline(y=chi_lb, color='b', linestyle='--', label=r"$\chi^2$ lower threshold")
    plt.legend(loc='upper right')


def compute_chisquare(dataFilterCov_S, dataFilterInno, threshold=1e12):
    """
    Computes the Mahalanobis distance (chi-square) for a sequence of innovation and covariance matrices,
    and plots the resulting values if the covariance is well-conditioned.

    Parameters:
    - dataFilterCov_S: np.ndarray of shape (N, 9), flattened 3x3 covariance matrices
    - dataFilterInno: np.ndarray of shape (N, 3), innovation vectors
    - threshold: float, condition number threshold for filtering singular matrices

    Returns:
    - dataChiSquare: np.ndarray of Mahalanobis distances
    """
    dataChiSquare = []
    for i in range(dataFilterInno.shape[0]):
        cov_i = dataFilterCov_S[i, :].reshape(3, 3)
        cond = np.linalg.cond(cov_i)
        if cond < threshold:
            mahalanobis = dataFilterInno[i, :].T @ np.linalg.inv(cov_i) @ dataFilterInno[i, :]
            dataChiSquare.append(mahalanobis)
    dataChiSquare = np.array(dataChiSquare)
    return dataChiSquare

def setup_inertialattfilter(filterObject):
    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0
    filterObject.switchMag = 1.2
    filterObject.stateInit = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    filterObject.covarInit = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
    sigmaMrpSquare = (1E-3) ** 2
    sigmaRateSquare = (5E-4) ** 2
    qNoise = np.identity(6)
    qNoise[0:3, 0:3] = qNoise[0:3, 0:3]*sigmaMrpSquare
    qNoise[3:6, 3:6] = qNoise[3:6, 3:6]*sigmaRateSquare
    filterObject.qNoise = qNoise.reshape(36).tolist()

def configure_inertialattfilter(filterObject, config, measurement_message):
    setup_inertialattfilter(filterObject)
    vcMsg = config["vcMsg"]
    rwStateEffector = config["rwStateEffector"]
    inertialAttFilterRwParamMsg = config["inertialAttFilterRwParamMsg"]
    gyroInMsg = config["gyroInMsg"]
    st_cov = config["st_cov"]
    # connect message to spacecraft and RW configurations
    filterObject.massPropsInMsg.subscribeTo(vcMsg)
    filterObject.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    filterObject.rwParamsInMsg.subscribeTo(inertialAttFilterRwParamMsg)
    filterObject.gyrBuffInMsg.subscribeTo(gyroInMsg)
    # setup measurement model in the filter
    starTracker1 = inertialUKF.STMessage()
    starTracker1.noise = [st_cov, 0.0, 0.0,
                          0.0, st_cov, 0.0,
                          0.0, 0.0, st_cov]
    star_tracker_list = [starTracker1]
    filterObject.STDatasStruct.STMessages = star_tracker_list
    filterObject.STDatasStruct.numST = len(star_tracker_list)
    # connect filter star tracker to the true inertial attitude measurement
    filterObject.STDatasStruct.STMessages[0].stInMsg.subscribeTo(measurement_message)


def _test_multipleinertialUkf(show_plots=False):
    """
    Args:
        show_plots (bool): Determines if the script should display plots
    """

    # --- Create Simulation ---
    simTaskName = "simTask"
    simProcessName = "simProcess"
    # create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    # set the simulation time variable used later on
    simTimeSec = 600
    simulationTime = macros.sec2nano(simTimeSec)
    # create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)
    # create the dynamics task and specify the integration update time
    simTimeStepSec = 0.1
    simulationTimeStep = macros.sec2nano(simTimeStepSec)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # --- Setup Gravity ---
    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # --- Create Spacecraft ---
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the spacecraft inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    # define the spacecraft initial state
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
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B
    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, 1)
    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    # --- Setup Reaction Wheels ---
    # make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()
    varRWModel = messaging.BalancedWheels
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

    # --- Connect Reaction Wheels to Spacecraft via rwStateEffector ---
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)
    # add RW object array to the simulation process.  This is required for the UpdateState() method
    # to be called which logs the RW states
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)

    # --- Setup Navigation --- 
    # NOTE add the simple Navigation sensor module.  
    # This sets the SC attitude, rate, position velocity navigation message
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

    # --- Connect messages ---
    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    # create the FSW reaction wheel configuration message
    fswRwParamMsg = rwFactory.getConfigMessage()
    # create the inertialUKF reaction wheel configuration message
    inertialAttFilterRwParamMsg = rwFactory.getConfigMessage()
    # connect navigation to spacecraft
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    # connect att control error
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    # connect mrp control law
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    # create and connect RW motor torqe
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)
    #   make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 
        0, 1, 0, 
        0, 0, 1
    ]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    # connect rwStateEffector to rwMotorTorque
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

    # --- Create the holder for the true inertial attitude measurement ---
    st_1_data = messaging.STAttMsgPayload()
    st_1_data.timeTag = 0
    attitude_measurement_msg = messaging.STAttMsg().write(st_1_data)
    st_cov = 1e-4

    # --- Create multiple inertialUKF (state = MRP, angular_rate) ---
    # create an empty gyro measurement
    gyroBufferData = messaging.AccDataMsgPayload()
    gyroInMsg = messaging.AccDataMsg()
    gyroInMsg.write(gyroBufferData, 0)

    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)

    # 0: nominal filter
    inertialAttFilter = inertialUKF.inertialUKF()
    scSim.AddModelToTask(simTaskName, inertialAttFilter)
    config = {
        "vcMsg": vcMsg,
        "rwStateEffector": rwStateEffector, 
        "inertialAttFilterRwParamMsg": inertialAttFilterRwParamMsg, 
        "gyroInMsg": gyroInMsg,
        "st_cov": st_cov,
    }
    configure_inertialattfilter(inertialAttFilter, config, attitude_measurement_msg)
    inertialAttFilterLog = inertialAttFilter.logger(["covar", "state", "cov_S", "innovation"], samplingTime)
    scSim.AddModelToTask(simTaskName, inertialAttFilterLog)

    # 1: fault1 filter
    inertialAttFilter1 = inertialUKF.inertialUKF()
    scSim.AddModelToTask(simTaskName, inertialAttFilter1)
    rwFactory_fault1 = simIncludeRW.rwFactory()
    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    rwFactory_fault1.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=100.  # RPM
                           , RWModel=varRWModel, 
                           Omega_max = 3000.0*macros.RPM # the default Honeywell_HR16 has 6000.0*macros.RPM
                           )
    rwFactory_fault1.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=200.  # RPM
                           , RWModel=varRWModel
                           )
    rwFactory_fault1.create('Honeywell_HR16', [0, 0, 1], maxMomentum=50., Omega=300.  # RPM
                           , rWB_B=[0.5, 0.5, 0.5]  # meters
                           , RWModel=varRWModel,
                           )
    config1 = {
        "vcMsg": vcMsg,
        "rwStateEffector": rwStateEffector, 
        "inertialAttFilterRwParamMsg": rwFactory_fault1.getConfigMessage(), 
        "gyroInMsg": gyroInMsg,
        "st_cov": st_cov,
    }
    configure_inertialattfilter(inertialAttFilter1, config1, attitude_measurement_msg)
    inertialAttFilter1Log = inertialAttFilter1.logger(["covar", "state", "cov_S", "innovation"], samplingTime)
    scSim.AddModelToTask(simTaskName, inertialAttFilter1Log)

    # 2: fault2 filter
    inertialAttFilter2 = inertialUKF.inertialUKF()
    scSim.AddModelToTask(simTaskName, inertialAttFilter2)
    rwFactory_fault2 = simIncludeRW.rwFactory()
    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    rwFactory_fault2.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=100.  # RPM
                           , RWModel=varRWModel, 
                           )
    rwFactory_fault2.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=200.  # RPM
                           , RWModel=varRWModel,
                           Omega_max = 3000.0*macros.RPM # the default Honeywell_HR16 has 6000.0*macros.RPM
                           )
    rwFactory_fault2.create('Honeywell_HR16', [0, 0, 1], maxMomentum=50., Omega=300.  # RPM
                           , rWB_B=[0.5, 0.5, 0.5]  # meters
                           , RWModel=varRWModel,
                           )
    config2 = {
        "vcMsg": vcMsg,
        "rwStateEffector": rwStateEffector, 
        "inertialAttFilterRwParamMsg": rwFactory_fault2.getConfigMessage(), 
        "gyroInMsg": gyroInMsg,
        "st_cov": st_cov,
    }
    configure_inertialattfilter(inertialAttFilter2, config2, attitude_measurement_msg)
    inertialAttFilter2Log = inertialAttFilter2.logger(["covar", "state", "cov_S", "innovation"], samplingTime)
    scSim.AddModelToTask(simTaskName, inertialAttFilter2Log)

    # 3: fault3 filter
    inertialAttFilter3 = inertialUKF.inertialUKF()
    scSim.AddModelToTask(simTaskName, inertialAttFilter3)
    rwFactory_fault3 = simIncludeRW.rwFactory()
    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    rwFactory_fault3.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=100.  # RPM
                           , RWModel=varRWModel, 
                           )
    rwFactory_fault3.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=200.  # RPM
                           , RWModel=varRWModel,
                           )
    rwFactory_fault3.create('Honeywell_HR16', [0, 0, 1], maxMomentum=50., Omega=300.  # RPM
                           , rWB_B=[0.5, 0.5, 0.5]  # meters
                           , RWModel=varRWModel,
                           Omega_max = 3000.0*macros.RPM # the default Honeywell_HR16 has 6000.0*macros.RPM
                           )
    config3 = {
        "vcMsg": vcMsg,
        "rwStateEffector": rwStateEffector, 
        "inertialAttFilterRwParamMsg": rwFactory_fault3.getConfigMessage(), 
        "gyroInMsg": gyroInMsg,
        "st_cov": st_cov,
    }
    configure_inertialattfilter(inertialAttFilter3, config3, attitude_measurement_msg)
    inertialAttFilter3Log = inertialAttFilter3.logger(["covar", "state", "cov_S", "innovation"], samplingTime)
    scSim.AddModelToTask(simTaskName, inertialAttFilter3Log)


    # collect filter log
    inertialAttFilterLog_dict = {
        "nominal": inertialAttFilterLog, 
        "fault1": inertialAttFilter1Log,
        "fault2": inertialAttFilter2Log,
        "fault3": inertialAttFilter3Log,
    }
    
    # --- Setup Data Logging ---
    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, rwMotorLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, snTransLog)
    # add the true attitude state log
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, snAttLog)
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

    # --- Setup 3D Visualiztion
    # viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
    #                                           , saveFile=fileName
    #                                           , rwEffectorList=rwStateEffector
    #                                           )

    # --- Initialize Simulation ---
    scSim.InitializeSimulation()
    timeSpan = np.arange(0, simTimeSec + simTimeStepSec, simTimeStepSec)

    # --- Run Simulation ---
    for i in range(len(timeSpan)-1):
        # propagate to next time
        scSim.ConfigureStopTime(macros.sec2nano((timeSpan[i+1])))
        scSim.ExecuteSimulation()
        # obtain true star tracker measurement
        if(snAttLog.sigma_BN.shape[0] > 0):
            true_att = snAttLog.sigma_BN[-1,:]
            true_att_with_noise = true_att + np.random.normal(0, np.sqrt(st_cov), 3)
            st_1_data.valid = True
            st_1_data.timeTag = int(timeSpan[i+1]*1E9)
            st_1_data.MRP_BdyInrtl = true_att_with_noise
        attitude_measurement_msg.write(st_1_data, int(timeSpan[i+1]*1E9))

    # --- Retrieve Logged Data ---
    dataUsReq = rwMotorLog.motorTorque
    dataSigmaBN = snAttLog.sigma_BN
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBN = snAttLog.omega_BN_B
    dataOmegaBR = attErrorLog.omega_BR_B
    dataPos = snTransLog.r_BN_N
    dataOmegaRW = mrpLog.wheelSpeeds
    dataRW = []
    for i in range(numRW):
        dataRW.append(rwLogs[i].u_current)

    # --- Plot Results ---
    np.set_printoptions(precision=16)
    timeData = rwMotorLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    # plot_attitude_error(timeData, dataSigmaBR)

    # plot_rate_error(timeData, dataOmegaBR)

    # plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)

    # plot_rw_speeds(timeData, dataOmegaRW, numRW)

    # show all filter results
    for key, filterlog in inertialAttFilterLog_dict.items():
        dataFilterState = filterlog.state
        dataFilterCov = filterlog.covar
        dataFilterCov_S = filterlog.cov_S
        dataFilterInno = filterlog.innovation
        dataFilterSigmaBN = dataFilterState[:, 0:3]
        dataFilterOmegaBN = dataFilterState[:, 3:]
        # assert equalt shape of the true and filter estimated states
        np.testing.assert_equal(dataSigmaBN.shape, dataFilterSigmaBN.shape)
        _tmp = dataFilterCov.reshape(-1, 6, 6)
        dataFilterSigmaDiagCov = np.diagonal(_tmp, 
                                            axis1=1, axis2=2)[:, :3]
        dataFilterOmegaDiagCov = np.diagonal(_tmp, 
                                            axis1=1, axis2=2)[:, 3:]
        if(key == "nominal"): # assuming the true hypothesis is the nominal dynamics
            # assert the filter estimated states are within 6 standard deviations
            np.testing.assert_array_less(np.abs(dataSigmaBN-dataFilterSigmaBN)[-10:, :], 
                                        6*np.sqrt(dataFilterSigmaDiagCov)[-10:, :])
            np.testing.assert_array_less(np.abs(dataOmegaBN-dataFilterOmegaBN)[-10:, :], 
                                        6*np.sqrt(dataFilterOmegaDiagCov)[-10:, :])
            
        dataChiSquare = compute_chisquare(dataFilterCov_S, dataFilterInno)
        plot_filter_chisquare(dataChiSquare)
        # plot_filter_result_sigma(key, timeData, dataSigmaBN, dataFilterSigmaBN, dataFilterSigmaDiagCov)
        # plot_filter_result_omega(key, timeData, dataOmegaBN, dataFilterOmegaBN, dataFilterOmegaDiagCov)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


if __name__ == "__main__":
    _test_multipleinertialUkf(
        show_plots=True,  # show_plots
    )