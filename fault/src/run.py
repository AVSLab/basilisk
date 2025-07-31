import os
import matplotlib.pyplot as plt
import numpy as np
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    inertial3D, rwMotorTorque)
from Basilisk.utilities import (macros,simIncludeRW, unitTestSupport)
from Basilisk.fswAlgorithms import inertialUKF

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from .spacecraft import setup_spacecraft_sim
from .navigation import setup_navigation_and_control
from .ukf import compute_chisquare, configure_inertialattfilter
from .messages import setup_messages
from .log import setup_logging


from .plots import (
    plot_attitude_error,
    plot_filter_result_sigma,
    plot_filter_result_omega,
    plot_rw_cmd_torque,
    plot_rw_motor_torque,
    plot_rate_error,
    plot_rw_speeds,
    plot_filter_chisquare
)


def multipleinertialUkf(show_plots=False):
    """
    Args:
        show_plots (bool): Determines if the script should display plots
    """

    # Setup spacecraft and simulation module
    (scSim, scObject,simTaskName, simTimeSec, simTimeStepSec, simulationTime, simulationTimeStep,
        varRWModel, rwFactory, rwStateEffector, numRW, I) = setup_spacecraft_sim()

    # Setup navigation module
    sNavObject, inertial3DObj, attError, mrpControl = setup_navigation_and_control(scSim, simTaskName)

    # Connect messages
    vcMsg, inertialAttFilterRwParamMsg, attitude_measurement_msg, st_cov, rwMotorTorqueObj, st_1_data \
        = setup_messages(scSim, simTaskName, I, rwFactory, scObject, sNavObject, attError, inertial3DObj, mrpControl, rwStateEffector)
    fswRwParamMsg = rwFactory.getConfigMessage()
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)


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
    
    # Setup logs
    rwMotorLog, attErrorLog, snTransLog, snAttLog, mrpLog, rwLogs \
          = setup_logging(scSim, simTaskName, samplingTime, rwMotorTorqueObj, attError, sNavObject, rwStateEffector, numRW)


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