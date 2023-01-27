# 
#  ISC License
# 
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
# 
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
# 
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
# 
# 

import os

import matplotlib.pyplot as plt
import numpy as np
import pytest
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.simulation import MtbEffector
from Basilisk.simulation import spacecraft, magneticFieldWMM
from Basilisk.utilities import SimulationBaseClass, simIncludeGravBody, orbitalMotion, RigidBodyKinematics
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def truthMagneticTorque(bField_N, sigmaBN, mtbCmds, GtMatrix, numMTB, maxDipole):
    
    temp = np.asarray(GtMatrix[0:3*numMTB])
    GtMatrix = np.reshape(temp, (3, numMTB))
    bField_N = np.asarray(bField_N)
    BN = RigidBodyKinematics.MRP2C(sigmaBN)
    bField_B = BN @ bField_N
    for i in range(len(mtbCmds)):
        if mtbCmds[i] > maxDipole:
            mtbCmds[i] = maxDipole
        if mtbCmds[i] < -maxDipole:
            mtbCmds[i] = -maxDipole

    mtbCmds = np.asarray(mtbCmds[0:numMTB])
    bTilde = np.asarray(RigidBodyKinematics.v3Tilde(bField_B))
    tauNet = - bTilde @ GtMatrix @ mtbCmds
    return tauNet


@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize("maxDipole", [10., 0.1])
def test_MtbEffector(show_plots, accuracy, maxDipole):
    r"""
    **Validation Test Description**

    Compose a general description of what is being tested in this unit test script.

    **Test Parameters**

    Discuss the test parameters used.

    Args:
        param1 (int): Dummy test parameter for this parameterized unit test
        param2 (int): Dummy test parameter for this parameterized unit test
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    Here discuss what variables and states are being checked. 
    """
    [testResults, testMessage] = MtbEffectorTestFunction(show_plots, accuracy, maxDipole)
    assert testResults < 1, testMessage


def MtbEffectorTestFunction(show_plots, accuracy, maxDipole):
    """Call this routine directly to run the unit test."""
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
   
    # create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"


    # create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))
    simTime = 3.
    simulationTime = macros.sec2nano(simTime)


    # create Earth Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu
    
    
    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskTestSat"

    I = [10.5, 0., 0.,
         0., 8., 0.,
         0., 0., 6.75]
    scObject.hub.mHub = 10.0  # kg - spacecraft mass (arbitrary)
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    
    oe = orbitalMotion.ClassicElements()
    oe.a = 6778.14 * 1000.  # meters
    oe.e = 0.0
    oe.i = 45. * macros.D2R
    oe.Omega = 60. * macros.D2R
    oe.omega = 0. * macros.D2R
    oe.f = 0. * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    # add spacecraft object
    scSim.AddModelToTask(simTaskName, scObject)
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    
    
    # add magnetic field module
    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.ModelTag = "WMM"
    magModule.dataPath = bskPath + '/supportData/MagneticField/'
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg('2020 May 12, 00:00:0.0 (UTC)')
    magModule.epochInMsg.subscribeTo(epochMsg)
    magModule.addSpacecraftToModel(scObject.scStateOutMsg)  # this command can be repeated if multiple
    scSim.AddModelToTask(simTaskName, magModule)
    
    
    # add magnetic torque bar effector
    mtbEff = MtbEffector.MtbEffector()
    mtbEff.ModelTag = "MtbEff"
    scObject.addDynamicEffector(mtbEff)
    scSim.AddModelToTask(simTaskName, mtbEff)
    
    
    # mtbConfigData message
    mtbConfigParams = messaging.MTBArrayConfigMsgPayload()
    mtbConfigParams.numMTB = 3
    # row major toque bar alignments
    mtbConfigParams.GtMatrix_B = [
        1., 0., 0.,
        0., 1., 0.,
        0., 0., 1.
    ]
    mtbConfigParams.maxMtbDipoles = [maxDipole]*4
    mtbParamsInMsg = messaging.MTBArrayConfigMsg().write(mtbConfigParams)
    
    
    # MTBCmdMsgPayload, mtbCmdInMsg
    mtbCmdInMsgContainer = messaging.MTBCmdMsgPayload()
    mtbCmdInMsgContainer.mtbDipoleCmds = [0.2, 0.2, 0.2]
    mtbCmdInMsg = messaging.MTBCmdMsg().write(mtbCmdInMsgContainer)
    
    # subscribe to mesaages 
    mtbEff.mtbCmdInMsg.subscribeTo(mtbCmdInMsg)
    mtbEff.mtbParamsInMsg.subscribeTo(mtbParamsInMsg)
    mtbEff.magInMsg.subscribeTo(magModule.envOutMsgs[0])
    
    
    # Setup data logging before the simulation is initialized
    numDataPoints = 3600
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    dataLogMag = magModule.envOutMsgs[0].recorder(samplingTime)
    dataLogMTB = mtbEff.mtbOutMsg.recorder(samplingTime)
    
    scSim.AddModelToTask(simTaskName, dataLogMTB)
    scSim.AddModelToTask(simTaskName, dataLogMag)
    scSim.AddModelToTask(simTaskName, dataLog)
    

    # initialize Simulation
    scSim.InitializeSimulation()

    # configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve the logged data
    attData = dataLog.sigma_BN
    mtbData = dataLogMTB.mtbNetTorque_B
    dataMagField = dataLogMag.magField_N
    np.set_printoptions(precision=16)
     
    # plot net mtb torque
    if show_plots:
        plt.close("all")  # clears out plots from earlier test runs
        plt.figure(1)
        for idx in range(0, 3):
            plt.plot(dataLogMTB.times() * macros.NANO2SEC, mtbData[:, idx],
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$\tau_' + str(idx) + '$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [s]')
        plt.ylabel(r'MTB Net Torque $\tau_{B}$ [Nm]')

        # plot magnetic field data in the Inertial frame
        plt.figure(2)
        for idx in range(3):
            plt.plot(dataLogMag.times() * macros.NANO2SEC, dataMagField[:, idx] * 1e9,
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$B\_N_{' + str(idx) + '}$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [s]')
        plt.ylabel('Magnetic Field [nT]')

        # plot the Body relative to Inertial attitude
        plt.figure(3)
        for idx in range(0, 3):
            plt.plot(dataLog.times() * macros.NANO2SEC, attData[:, idx],
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$\sigma_' + str(idx) + '$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [s]')
        plt.ylabel(r'MRP Attitude $\sigma_{B/N}$')

        plt.show()

    # compare gravity gradient torque vector to the truth

    # use mtbData[1:, :] since mtbData is the torque from prev iterations
    for sV, mtbTauV, bV in zip(attData[1:, :], mtbData[1:, :], dataMagField):
        mtbTauTruth = truthMagneticTorque(bV, sV, mtbCmdInMsgContainer.mtbDipoleCmds,
                                          mtbConfigParams.GtMatrix_B,
                                          mtbConfigParams.numMTB,
                                          maxDipole
                                          )
        
        testFailCount, testMessages = unitTestSupport.compareVector(mtbTauV,
                                                                    mtbTauTruth,
                                                                    accuracy,
                                                                    "mtbTorque_B",
                                                                    testFailCount, testMessages)
     
    if testFailCount == 0:
        print("PASSED: Mtb Effector")
    else:
        print("Failed: Mtb Effector")

    return testFailCount, testMessages
if __name__ == "__main__":
    test_MtbEffector(
        False,
        1e-12,
        0.1
    )


