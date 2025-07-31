import numpy as np
from Basilisk.fswAlgorithms import inertialUKF
from Basilisk.utilities import (macros, simIncludeRW)

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


def create_filter(simTaskName, scSim, mode, vcMsg, rwStateEffector, gyroInMsg, st_cov, attitude_measurement_msg, samplingTime, varRWModel):
    filter_instance = inertialUKF.inertialUKF()
    scSim.AddModelToTask(simTaskName, filter_instance)

    rwFactory = simIncludeRW.rwFactory()
    # Define RW configs based on mode
    rw_configs = [
        {"Omega_max": None, "rWB_B": None},  # Nominal
        {"Omega_max": 3000.0 * macros.RPM, "rWB_B": None},  # Fault 1
        {"Omega_max": None, "rWB_B": [0.5, 0.5, 0.5]},       # Fault 2
        {"Omega_max": 3000.0 * macros.RPM, "rWB_B": [0.5, 0.5, 0.5]}  # Fault 3
    ]

    config = rw_configs[mode]
    Omega_max = config.get("Omega_max", None)
    rWB_B = config.get("rWB_B", None)

    for i, gsHat in enumerate([[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
        rw_kwargs = {
            "RWModel": varRWModel,
            "Omega": 100. * (i + 1),  # 100, 200, 300 RPM
            "maxMomentum": 50.
        }
        if i == mode and Omega_max is not None:
            rw_kwargs["Omega_max"] = Omega_max
        if rWB_B is not None:
            rw_kwargs["rWB_B"] = rWB_B

        rwFactory.create('Honeywell_HR16', gsHat, **rw_kwargs)

    rwParamMsg = rwFactory.getConfigMessage()
    config_dict = {
        "vcMsg": vcMsg,
        "rwStateEffector": rwStateEffector,
        "inertialAttFilterRwParamMsg": rwParamMsg,
        "gyroInMsg": gyroInMsg,
        "st_cov": st_cov,
    }

    configure_inertialattfilter(filter_instance, config_dict, attitude_measurement_msg)
    filter_logger = filter_instance.logger(["covar", "state", "cov_S", "innovation"], samplingTime)
    scSim.AddModelToTask(simTaskName, filter_logger)

    return filter_logger
