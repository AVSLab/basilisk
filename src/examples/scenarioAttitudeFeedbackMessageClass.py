''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
import numpy as np
np.set_printoptions(precision=16)

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simple_nav
from Basilisk.simulation import message

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import MRP_Feedback
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError

# import message declarations
from Basilisk.simulation.c_messages import VehicleConfigFswMsg_C

# I absolutely do not understand at the moment but we have to import vizSupport for new messages to work...
from Basilisk.utilities import vizSupport


def run(show_plots):
    """This is a sandbox to demonstrate how a new messaging system could work

    It does everything we currently do in a single bsk process"""

    simTaskName = "simTask"
    simProcessName = "simProcess"

    scSim = SimulationBaseClass.SimBaseClass()

    simulationTime = macros.min2nano(10.)

    dynProcess = scSim.CreateNewProcess(simProcessName)

    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    scObject = spacecraftPlus.SpacecraftPlus()
    I = [900., 0., 0., 0., 800., 0., 0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scSim.AddModelToTask(simTaskName, scObject)

    earth = simIncludeGravBody.gravityEffector.GravBodyData()
    earth.mu = 0.3986004415E+15  # meters^3/s^2
    earth.radEquator = 6378136.6  # meters
    earth.isCentralBody = True
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([earth])

    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    sNavObject = simple_nav.SimpleNav()
    scSim.AddModelToTask(simTaskName, sNavObject)

    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    scSim.AddModelToTask(simTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)

    mrpControlConfig = MRP_Feedback.MRP_FeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(earth.mu, oe)
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    sNavObject.readInputState.subscribeTo(scObject.stateOutMsg)
    snLog = scObject.stateOutMsg.log()
    scSim.AddModelToTask(simTaskName, snLog)

    attErrorConfig.inputNavMessage.subscribeTo(sNavObject.outputAttMessage)
    attErrorConfig.inputRefMessage.subscribeTo(inertial3DConfig.outMsg)
    attErrorLog = attErrorConfig.outputDataMessage.log()
    scSim.AddModelToTask(simTaskName, attErrorLog)

    mrpControlConfig.inputGuidanceMessage.subscribeTo(attErrorConfig.outputDataMessage)
    extFTObject.readCmdTorque.subscribeTo(mrpControlConfig.outputMessage)
    extFTLog = extFTObject.readCmdTorque.log()
    scSim.AddModelToTask(simTaskName, extFTLog)
    mrpLog = mrpControlConfig.outputMessage.log()
    scSim.AddModelToTask(simTaskName, mrpLog)

    configData = VehicleConfigFswMsg_C().userMessage()
    configData.payload.ISCPntB_B = I
    mrpControlConfig.vehicleConfigMessage.subscribeTo(configData)

    scSim.InitializeSimulationAndDiscover()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    sigma_error_fig = plt.figure()
    ax = sigma_error_fig.add_subplot('111')
    for idx in range(3):
        ax.plot(attErrorLog.times() * macros.NANO2MIN, attErrorLog.sigma_BR[:, idx],
                 color=unitTestSupport.getLineColor(idx+1, 3),
                 label='$\sigma_' + str(idx) + '$')
    ax.legend(loc='lower right')
    ax.set_xlabel('Time [min]')
    ax.set_ylabel('Attitude Error $\sigma_{B/R}$')

    tq_fig = plt.figure()
    ax = tq_fig.add_subplot('111')
    for idx in range(3):
        ax.plot(np.array(mrpLog.times()) * macros.NANO2MIN,
                 mrpLog.torqueRequestBody[:, idx],
                 color=unitTestSupport.getLineColor(idx+1, 3),
                 label='$L_{r,' + str(idx) + '}$')
    ax.legend(loc='lower right')
    ax.set_xlabel('Time [min]')
    ax.set_ylabel('Control Torque $L_r$ [Nm]')

    omErr_fig = plt.figure()
    ax = omErr_fig.add_subplot('111')
    for idx in range(3):
        ax.plot(attErrorLog.times() * macros.NANO2MIN, attErrorLog.omega_BR_B[:, idx],
                 color=unitTestSupport.getLineColor(idx+1, 3),
                 label='$\omega_{BR,' + str(idx) + '}$')
    ax.legend(loc='lower right')
    ax.set_xlabel('Time [min]')
    ax.set_ylabel('Rate Tracking Error [rad/s] ')

    if show_plots:
        plt.show()
    plt.close("all")

if __name__ == "__main__":
    run(True)
