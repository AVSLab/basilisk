from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import rwMotorTorque

def setup_messages(scSim, simTaskName, I, rwFactory, scObject, sNavObject, attError, inertial3DObj, mrpControl, rwStateEffector):
    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    
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
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    
    # create and connect RW motor torque
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)
    
    # make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    ]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    
    # connect rwStateEffector to rwMotorTorque
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
    
    # --- Create the holder for the true inertial attitude measurement ---
    st_1_data = messaging.STAttMsgPayload()
    st_1_data.timeTag = 0
    attitude_measurement_msg = messaging.STAttMsg().write(st_1_data)
    st_cov = 1e-4
    
    # Return the important messages and objects
    return vcMsg, inertialAttFilterRwParamMsg, attitude_measurement_msg, st_cov, rwMotorTorqueObj, st_1_data