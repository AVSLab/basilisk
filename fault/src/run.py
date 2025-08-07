import os
import matplotlib.pyplot as plt
import numpy as np
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.utilities import macros, simIncludeRW, unitTestSupport
from Basilisk.fswAlgorithms import inertialUKF

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from .utils.spacecraft import setup_spacecraft_sim
from .utils.navigation import setup_navigation_and_control
from .ukf import configure_inertialattfilter
from .utils.messages import setup_messages
from .utils.log import setup_logging, process_filter
from .passive import passive_fault_id

def run(moving_window, terminate = True, true_mode = 0, show_plots=False):
    """
    Args:
        show_plots (bool): Determines if the script should display plots
    """

    # Setup spacecraft and simulation module
    (scSim, scObject,simTaskName, simTimeSec, simTimeStepSec, simulationTime, simulationTimeStep,
        varRWModel, rwFactory, rwStateEffector, numRW, I) = setup_spacecraft_sim(true_mode=true_mode)

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
    snAttLog, rwLogs = setup_logging(scSim, simTaskName, samplingTime, rwMotorTorqueObj, 
                                     attError, sNavObject, rwStateEffector, numRW)

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

    # process_filter(snAttLog, rwLogs, inertialAttFilterLog_dict, numRW)

    # Perform passive fault ID
    H_hist, hypotheses, k_end, fail, id_mode = passive_fault_id(inertialAttFilterLog_dict, moving_window, terminate=terminate, true_mode = true_mode)

    H_hist = np.array(H_hist)  # convert to numpy array (timesteps x hypotheses)

    if show_plots:
        print("Hypothesis history over time:")
        for t, h in enumerate(H_hist):
            print(f"Time step {t}: {h}")

        plt.figure()
        for idx, h in enumerate(hypotheses):
            plt.plot(H_hist[:, idx], label=h)
        plt.xlabel("Time step")
        plt.ylabel("Belief")
        plt.title("Passive Fault Identification Belief Evolution")
        plt.legend()
        plt.grid(True)
        plt.show()

        if show_plots:
            plt.show()

        # close the plots being saved off to avoid over-writing old and new figures
        plt.close("all")

    print(f"Returning: true_mode={true_mode}, id_mode={id_mode}, equal={id_mode == true_mode}")

    return {
        "true_mode": true_mode,
        "identified_mode": id_mode,
        "correct": id_mode == true_mode,
        "id_time": k_end
    }