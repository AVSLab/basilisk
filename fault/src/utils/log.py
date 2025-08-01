import numpy as np
from ..ukf import compute_chisquare
from .plots import plot_filter_chisquare

def setup_logging(scSim, simTaskName, samplingTime, rwMotorTorqueObj, attError, sNavObject, rwStateEffector, numRW):
    # Setup Data Logging
    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, rwMotorLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, snTransLog)
    
    # Add the true attitude state log
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, snAttLog)
    
    # Log the RW speed information
    mrpLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, mrpLog)
    
    # Log each RW specific message
    rwLogs = []
    for i in range(numRW):
        log = rwStateEffector.rwOutMsgs[i].recorder(samplingTime)
        rwLogs.append(log)
        scSim.AddModelToTask(simTaskName, log)

    # Return all logs for external access
    return snAttLog, rwLogs

def process_filter(snAttLog, rwLogs, inertialAttFilterLog_dict, numRW):
    """
    Process logged data and plot filter results.
    
    Args:
        snAttLog: navigation star tracker attitude log (with sigma_BN, omega_BN_B)
        rwLogs: list of reaction wheel logs (each with u_current)
        inertialAttFilterLog_dict: dict of filter logs by hypothesis name
        numRW: number of reaction wheels
    
    Returns:
        dataSigmaBN, dataOmegaBN, dataRW (raw logged data arrays)
    """
    # Retrieve logged data
    dataSigmaBN = snAttLog.sigma_BN
    dataOmegaBN = snAttLog.omega_BN_B
    dataRW = [rwLogs[i].u_current for i in range(numRW)]

    # Process and plot each filter log
    for key, filterlog in inertialAttFilterLog_dict.items():
        dataFilterState = filterlog.state
        dataFilterCov = filterlog.covar
        dataFilterCov_S = filterlog.cov_S
        dataFilterInno = filterlog.innovation
        
        dataFilterSigmaBN = dataFilterState[:, 0:3]
        dataFilterOmegaBN = dataFilterState[:, 3:]
        
        # Assert equal shape of true and filter estimated states
        np.testing.assert_equal(dataSigmaBN.shape, dataFilterSigmaBN.shape)
        
        _tmp = dataFilterCov.reshape(-1, 6, 6)
        dataFilterSigmaDiagCov = np.diagonal(_tmp, axis1=1, axis2=2)[:, :3]
        dataFilterOmegaDiagCov = np.diagonal(_tmp, axis1=1, axis2=2)[:, 3:]
        
        if key == "nominal":  # assuming nominal is true hypothesis
            np.testing.assert_array_less(
                np.abs(dataSigmaBN - dataFilterSigmaBN)[-10:, :],
                6 * np.sqrt(dataFilterSigmaDiagCov)[-10:, :]
            )
            np.testing.assert_array_less(
                np.abs(dataOmegaBN - dataFilterOmegaBN)[-10:, :],
                6 * np.sqrt(dataFilterOmegaDiagCov)[-10:, :]
            )
        
        # Compute and plot Chi-Square results
        dataChiSquare = compute_chisquare(dataFilterCov_S, dataFilterInno)
        plot_filter_chisquare(dataChiSquare)
