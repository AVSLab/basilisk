import os
import matplotlib.pyplot as plt
import numpy as np
import scipy.io
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.utilities import macros, simIncludeRW, unitTestSupport
from Basilisk.fswAlgorithms import inertialUKF

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from utils.spacecraft import setup_spacecraft_sim
from utils.navigation import setup_navigation_and_control
from ukf import configure_inertialattfilter
from utils.messages import setup_messages
from utils.log import setup_logging, process_filter
from passive import passive_fault_id

def run_synthetic(sweep_window, show_plots=False):
    (scSim, scObject,simTaskName, simTimeSec, simTimeStepSec, simulationTime, simulationTimeStep,
        varRWModel, rwFactory, rwStateEffector, numRW, I) = setup_spacecraft_sim()

    sNavObject, inertial3DObj, attError, mrpControl = setup_navigation_and_control(scSim, simTaskName)

    vcMsg, inertialAttFilterRwParamMsg, attitude_measurement_msg, st_cov, rwMotorTorqueObj, st_1_data \
        = setup_messages(scSim, simTaskName, I, rwFactory, scObject, sNavObject, attError, inertial3DObj, mrpControl, rwStateEffector)
    fswRwParamMsg = rwFactory.getConfigMessage()
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)

    gyroBufferData = messaging.AccDataMsgPayload()
    gyroInMsg = messaging.AccDataMsg()
    gyroInMsg.write(gyroBufferData, 0)

    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)

    def make_filter(name, config_msg):
        filt = inertialUKF.inertialUKF()
        scSim.AddModelToTask(simTaskName, filt)
        config = {
            "vcMsg": vcMsg,
            "rwStateEffector": rwStateEffector, 
            "inertialAttFilterRwParamMsg": config_msg, 
            "gyroInMsg": gyroInMsg,
            "st_cov": st_cov,
        }
        configure_inertialattfilter(filt, config, attitude_measurement_msg)
        log = filt.logger(["covar", "state", "cov_S", "innovation"], samplingTime)
        scSim.AddModelToTask(simTaskName, log)
        return filt, log

    inertialAttFilter, inertialAttFilterLog = make_filter("nominal", inertialAttFilterRwParamMsg)

    def make_fault_rw_factory(modify_idx):
        factory = simIncludeRW.rwFactory()
        for i, gsHat in enumerate([[1,0,0],[0,1,0],[0,0,1]]):
            kwargs = dict(RWModel=varRWModel)
            if i == 2:
                kwargs['rWB_B'] = [0.5, 0.5, 0.5]
            if i == modify_idx:
                kwargs['Omega_max'] = 3000. * macros.RPM
            factory.create('Honeywell_HR16', gsHat, maxMomentum=50., Omega=(i+1)*100., **kwargs)
        return factory

    filters = {'nominal': inertialAttFilterLog}
    for i in range(3):
        fault_factory = make_fault_rw_factory(i)
        filt, log = make_filter(f"fault{i+1}", fault_factory.getConfigMessage())
        filters[f"fault{i+1}"] = log

    snAttLog, rwLogs = setup_logging(scSim, simTaskName, samplingTime, rwMotorTorqueObj, 
                                     attError, sNavObject, rwStateEffector, numRW)

    scSim.InitializeSimulation()
    timeSpan = np.arange(0, simTimeSec + simTimeStepSec, simTimeStepSec)

    for i in range(len(timeSpan)-1):
        scSim.ConfigureStopTime(macros.sec2nano((timeSpan[i+1])))
        scSim.ExecuteSimulation()
        if snAttLog.sigma_BN.shape[0] > 0:
            true_att = snAttLog.sigma_BN[-1,:]
            true_att_with_noise = true_att + np.random.normal(0, np.sqrt(st_cov), 3)
            st_1_data.valid = True
            st_1_data.timeTag = int(timeSpan[i+1]*1E9)
            st_1_data.MRP_BdyInrtl = true_att_with_noise
        attitude_measurement_msg.write(st_1_data, int(timeSpan[i+1]*1E9))

    process_filter(snAttLog, rwLogs, filters, numRW)

    H_hist, hypotheses = passive_fault_id(filters, moving_window=sweep_window)
    H_hist = np.array(H_hist)
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
    if show_plots:
        plt.show()
    plt.close("all")

    # Save .mat file
    import scipy.io
    save_dict = {
        "time": snAttLog.times() * 1e-9,
        "attitude_mrp": snAttLog.sigma_BN,
        "angular_rate": snAttLog.omega_BN_B
    }
    scipy.io.savemat(f"{fileName}_output.mat", save_dict)
    print(f"Saved time series to {fileName}_output.mat")



if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--sweep', 
        type=int, 
        nargs='*', 
        default=[10]
    )
    args = parser.parse_args()

    for sweep_window in args.sweep:
        print(f"Running FID with sweep window = {sweep_window}")
        run_synthetic(sweep_window, show_plots=True)