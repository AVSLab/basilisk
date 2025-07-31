from Basilisk.architecture import messaging

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
    return rwMotorLog, attErrorLog, snTransLog, snAttLog, mrpLog, rwLogs
