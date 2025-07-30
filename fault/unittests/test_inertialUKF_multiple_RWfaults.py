import os
import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError, inertial3D, rwMotorTorque, inertialUKF)
from Basilisk.simulation import reactionWheelStateEffector, simpleNav, spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport)


def setup_inertialattfilter(filterObject):
    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0
    filterObject.switchMag = 1.2
    filterObject.stateInit = [0.0]*6
    filterObject.covarInit = [1.0 if i % 7 == 0 else 0.0 for i in range(36)]
    sigmaMrpSquare = (1E-3)**2
    sigmaRateSquare = (5E-4)**2
    qNoise = np.identity(6)
    qNoise[0:3, 0:3] *= sigmaMrpSquare
    qNoise[3:6, 3:6] *= sigmaRateSquare
    filterObject.qNoise = qNoise.reshape(36).tolist()


def run_inertialUkf_mode(mode_id, rw_scales, save_dir="logs"):
    simTaskName = "simTask"
    simProcessName = "simProcess"
    simTimeSec = 600
    simTimeStepSec = 0.1
    simulationTime = macros.sec2nano(simTimeSec)
    simulationTimeStep = macros.sec2nano(simTimeStepSec)

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Gravity
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu

    # Spacecraft
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    I = [900., 0., 0., 0., 800., 0., 0., 0., 600.]
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    scSim.AddModelToTask(simTaskName, scObject, 1)
    gravFactory.addBodiesTo(scObject)

    # Reaction Wheels
    rwFactory = simIncludeRW.rwFactory()
    varRWModel = messaging.BalancedWheels
    RW1 = rwFactory.create('Honeywell_HR16', [1, 0, 0], maxMomentum=100. * rw_scales[0], Omega=100., RWModel=varRWModel)
    RW2 = rwFactory.create('Honeywell_HR16', [0, 1, 0], maxMomentum=100. * rw_scales[1], Omega=200., RWModel=varRWModel)
    RW3 = rwFactory.create('Honeywell_HR16', [0, 0, 1], maxMomentum=100. * rw_scales[2], Omega=300., RWModel=varRWModel)

    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)
    numRW = rwFactory.getNumOfDevices()

    # Navigation
    sNavObject = simpleNav.SimpleNav()
    scSim.AddModelToTask(simTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Filter Setup
    ukf = inertialUKF.inertialUKF()
    ukf.ModelTag = f"inertialUKF_mode{mode_id}"
    setup_inertialattfilter(ukf)
    scSim.AddModelToTask(simTaskName, ukf)

    vcPayload = messaging.VehicleConfigMsgPayload()
    vcPayload.ISCPntB_B = I
    vcMsg = messaging.VehicleConfigMsg().write(vcPayload)
    fswRwParamMsg = rwFactory.getConfigMessage()
    ukf.massPropsInMsg.subscribeTo(vcMsg)
    ukf.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    ukf.rwParamsInMsg.subscribeTo(fswRwParamMsg)

    # Gyro measurement input - create empty message with zero data and subscribe
    gyroInMsg = messaging.AccDataMsg().write(messaging.AccDataMsgPayload(), 0)
    ukf.gyrBuffInMsg.subscribeTo(gyroInMsg)

    st_cov = 1e-4
    st = inertialUKF.STMessage()
    st.noise = [st_cov if i % 4 == 0 else 0.0 for i in range(9)]
    st_data = messaging.STAttMsgPayload()
    st_data.timeTag = 0
    st_msg = messaging.STAttMsg().write(st_data)
    st.stInMsg.subscribeTo(st_msg)
    ukf.STDatasStruct.STMessages = [st]
    ukf.STDatasStruct.numST = 1

    # Logging setup
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, 100)
    attLog = sNavObject.attOutMsg.recorder(samplingTime)
    filtLog = ukf.logger(["covar", "state"], samplingTime)
    scSim.AddModelToTask(simTaskName, attLog)
    scSim.AddModelToTask(simTaskName, filtLog)

    scSim.InitializeSimulation()
    timeVec = np.arange(0, simTimeSec + simTimeStepSec, simTimeStepSec)

    for i in range(len(timeVec) - 1):
        scSim.ConfigureStopTime(macros.sec2nano(timeVec[i+1]))
        scSim.ExecuteSimulation()
        if attLog.sigma_BN.shape[0] > 0:
            # Simulate ST measurement with noise
            meas = attLog.sigma_BN[-1, :] + np.random.normal(0, np.sqrt(st_cov), 3)
            st_data.MRP_BdyInrtl = meas
            st_data.timeTag = int(timeVec[i+1] * 1e9)
            st_data.valid = True
            st_msg.write(st_data, st_data.timeTag)

    # Save logs
    output_dir = os.path.join(save_dir, f"mode_{mode_id}")
    os.makedirs(output_dir, exist_ok=True)
    np.save(os.path.join(output_dir, "true_sigmaBN.npy"), attLog.sigma_BN)
    np.save(os.path.join(output_dir, "filt_state.npy"), filtLog.state)
    np.save(os.path.join(output_dir, "filt_cov.npy"), filtLog.covar)

    print(f"Saved logs for mode {mode_id} at {output_dir}")


def load_all_logs(save_dir="logs", modes=[0,1,2,3,4]):
    data = {}
    for mode in modes:
        mode_dir = os.path.join(save_dir, f"mode_{mode}")
        true_sigma = np.load(os.path.join(mode_dir, "true_sigmaBN.npy"))
        filt_state = np.load(os.path.join(mode_dir, "filt_state.npy"))
        filt_cov = np.load(os.path.join(mode_dir, "filt_cov.npy"))
        data[mode] = {
            "true_sigma": true_sigma,
            "filt_state": filt_state,
            "filt_cov": filt_cov
        }
    return data


def plot_all_ukfs(data):
    # Data keys: mode -> dict with true_sigma, filt_state, filt_cov
    modes = sorted(data.keys())
    n_states = data[modes[0]]["filt_state"].shape[1]  # should be 6

    plt.figure(figsize=(12, 8))
    colors = plt.cm.get_cmap('tab10', len(modes))

    for state_idx in range(n_states):
        plt.subplot(n_states, 1, state_idx+1)
        for mode_i, mode in enumerate(modes):
            filt_state = data[mode]["filt_state"]
            if mode == 0:
                plt.plot(range(filt_state.shape[0]), filt_state[:, state_idx],
                         label=f"Mode {mode} (True Mode)", color="k", linewidth=2.5)
            else:
                plt.plot(range(filt_state.shape[0]), filt_state[:, state_idx],
                         label=f"Mode {mode}", color=colors(mode_i), linestyle="--")
        plt.ylabel(f"State {state_idx}")
        if state_idx == 0:
            plt.title("UKF State Estimates Over Time Steps (True Mode = 0)")
        if state_idx == n_states - 1:
            plt.xlabel("Time Step")
        if state_idx == 0:
            plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    modes = {
        0: [1.0, 1.0, 1.0],
        1: [0.5, 1.0, 1.0],
        2: [1.0, 0.5, 1.0],
        3: [1.0, 1.0, 0.5],
        4: [0.5, 0.5, 0.5]
    }
    save_dir = "logs"

    for mode_id, scales in modes.items():
        print(f"Running UKF Mode {mode_id} with RW scales {scales}")
        run_inertialUkf_mode(mode_id, scales, save_dir=save_dir)

    # Load logs and plot
    data = load_all_logs(save_dir=save_dir, modes=list(modes.keys()))
    plot_all_ukfs(data)

if __name__ == "__main__":
    main()
