# ISC License
#
# Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import inspect
import math
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import simHelpers
from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros, RigidBodyKinematics as rbk
from Basilisk.simulation import spacecraft, svIntegrators, thrusterStateEffector
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
import matplotlib.pyplot as plt


class ResultsStore:
    def __init__(self):
        self.PassFail = []

    def texSnippet(self):
        for i in range(len(self.PassFail)):
            snippetName = 'Result' + str(i)
            if self.PassFail[i] == 'PASSED':
                textColor = 'ForestGreen'
            elif self.PassFail[i] == 'FAILED':
                textColor = 'Red'
            texSnippet = r'\textcolor{' + textColor + '}{' + self.PassFail[i] + '}'
            simHelpers.writeTeXSnippet(snippetName, texSnippet, path)


@pytest.fixture(scope="module")
def testFixture():
    listRes = ResultsStore()
    yield listRes
    listRes.texSnippet()


def thrusterEffectorAllTests(show_plots):
    [testResults, testMessage] = test_unitThrusters(show_plots)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)


@pytest.mark.parametrize("thrustNumber, initialConditions, duration, long_angle, lat_angle, location, swirlTorque, rate, attachBody", [
    (1, 0., 2.0, 30., 15., [[1.125], [0.5], [2.0]], 0.0, macros.sec2nano(0.01), "OFF"),  # 1 thruster, thrust on
    (1, 1., 0.0, 30., 15., [[1.125], [0.5], [2.0]], 0.0, macros.sec2nano(0.01), "OFF"),  # 1 thruster, thrust off
    (1, 0., 2.0, 60., -15., [[-1.125], [0.5], [-2.0]], 0.0, macros.sec2nano(0.01), "OFF"),  # 1 thruster, thrust on, different orientation and location
    (1, 1., 0.0, 60., -15., [[-1.125], [0.5], [-2.0]], 0.0, macros.sec2nano(0.01), "OFF"),  # 1 thruster, thrust off, different orientation and location
    (2, 0., 2.0, 30., 15., [[1.125], [0.5], [2.0]], 0.0, macros.sec2nano(0.01), "OFF"),  # 2 thrusters, thrust on
    (2, 1., 0.0, 30., 15., [[1.125], [0.5], [2.0]], 0.0, macros.sec2nano(0.01), "OFF"),  # 2 thrusters, thrust off
    (2, 0., 2.0, 30., 15., [[1.125], [0.5], [2.0]], 2.0, macros.sec2nano(0.01), "OFF"),  # 2 thrusters, thrust on, swirl torque
    (2, 0., 2.0, 30., 15., [[1.125], [0.5], [2.0]], 0.0, macros.sec2nano(0.01), "ON")  # 2 thrusters, attached body
])
# provide a unique test method name, starting with test_
def test_unitThrusters(testFixture, show_plots, thrustNumber, initialConditions, duration, long_angle, lat_angle, location, swirlTorque, rate, attachBody):
    r"""
    **Validation Test Description**

    This unit test script tests the stateEffector implementation of thrusters. It sets up the thruster module and runs
    a combination of 6 different scenarios. Each scenario uses either one or two thrusters, while also changing the
    thruster's locations and whether thruster 1 is firing or not.

    For information on how the thruster module works and what the closed-form solution for the ``thrustFactor`` variable
    is, see :ref:`thrusterStateEffector`. Given the ``thrustFactor`` :math:`\kappa`, the thrust is computed as follows:

    .. math::
        \textbf{F} = \kappa \cdot F_{\mathrm{max}} \cdot \hat{n}

    where :math:`\hat{n}` is the thruster's direction vector. The torque is computed by:

    .. math::
        \textbf{T} = \textbf{r}\times\textbf{F} + \kappa \cdot T_{\mathrm{maxSwirl}} \cdot \hat{n}

    where :math:`\textbf{r}` corresponds to the thruster's position relative to the spacecraft's center of mass and the
    second term represents the swirl torque. The mass flow rate is given by:

    .. math::
        \dot{m} = \dfrac{F}{g\cdot I_{sp}}

    where :math:`g` is Earth's gravitational acceleration and :math:`I_{sp}` is the thruster's specific impulse.

    **Test Parameters**

    Args:
        thrustNumber (int): number of thrusters used in the simulation
        initialConditions (float): initial value of the ``thrustFactor`` variable for thruster 1. Thruster always starts
        off.
        duration (float): duration of the thrust in seconds.
        long_angle (float): longitude angle in degrees for thruster 1. Thruster 2 is also impacted by this value.
        lat_angle (float): latitude angle in degrees for thruster 1. Thruster 2 is also impacted by this value.
        location (float): location of thruster 1.
        swirlTorque (float): maximum value of the swirl torque on the thruster.
        rate (int): simulation rate in nanoseconds.
        attachBody (flag): whether the thruster is attached to the hub or to a different body.

    **Description of Variables Being Tested**

    In this file we are checking the values of the variables

    - ``thrForce``
    - ``thrTorque``
    - ``mDot``

    All these variables are compared to the true values from the closed-form expressions given in :ref:`thrusterStateEffector`.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitThrusters(testFixture, show_plots, thrustNumber, initialConditions, duration, long_angle,
                                               lat_angle, location, swirlTorque, rate, attachBody)
    assert testResults < 1, testMessage


@pytest.mark.parametrize("mount", ["hub", "hub-message", "external"])
def test_accepted_thrust_after_rejected_rkf45_step(mount):
    """Nozzle loads follow the accepted thrust factor and mount pose through rejection and retry."""
    sc = spacecraft.Spacecraft()
    sc.hub.mHub = 10.0  # [kg]
    sc.hub.IHubPntBc_B = (100.0 * np.eye(3)).tolist()  # [kg*m^2]
    sc.hub.v_CN_NInit = [0, 1, 0]  # [m/s]
    thrusters = thrusterStateEffector.ThrusterStateEffector()
    nozzle = thrusterStateEffector.THRSimConfig()
    nozzle.thrDir_B = [1, 0, 0]  # [-]
    nozzle.thrLoc_B = [0, 2, 0]  # [m]
    nozzle.MaxThrust = 2.0  # [N]
    nozzle.thrusterMagDisp = 0.25  # [-]
    nozzle.MaxSwirlTorque = 0.3  # [N*m]
    nozzle.cutoffFrequency = 1.0  # [1/s]
    nozzle.steadyIsp = 220.0  # [s]
    force = np.array([2.5, 0, 0])  # [N] at full thrust
    torque = np.array([0.3, 0, -5])  # [N*m] at full thrust
    if mount == "external":
        # Prescribe the hub pose so the expected nozzle loads can be calculated by inspection.
        position = messaging.TransRefMsgPayload()
        position.r_RN_N = [10, 0, 0]  # [m]
        position_msg = messaging.TransRefMsg().write(position)
        sc.transRefInMsg.subscribeTo(position_msg)
        attitude = messaging.AttRefMsgPayload()
        attitude.sigma_RN = [0, 0, math.tan(math.pi / 8)]  # [-] 90 degrees about +Z
        sc.attRefInMsg.subscribeTo(messaging.AttRefMsg().write(attitude))
        pose = messaging.SCStatesMsgPayload()
        pose.r_BN_N = [10, 4, 0]  # [m] mount origin is 4 m along hub +X
        pose.sigma_BN = [0, 0, 1]  # [-] mount +X points along hub +Y
        mount_msg = messaging.SCStatesMsg().write(pose)
        thrusters.addThruster(nozzle, mount_msg)
        force = np.array([0, 2.5, 0])  # [N]
        torque = np.array([0, 0.3, 5])  # [N*m] nozzle at (2, 0, 0) m in the hub frame
    elif mount == "hub-message":
        # A mount coincident with the moving hub must still have zero offset after a rejected trial.
        thrusters.addThruster(nozzle, sc.scStateOutMsg)
    else:
        thrusters.addThruster(nozzle)
    thrusters.kappaInit = messaging.DoubleVector([1.0])  # [-]
    sc.addStateEffector(thrusters)
    output = thrusters.thrusterOutMsgs[0].addSubscriber()
    rk = svIntegrators.svIntegratorRKF45(sc)
    rk.relTol = rk.absTol = 1e-12
    sc.setIntegrator(rk)
    thrusters.Reset(0)
    sc.Reset(0)
    state = sc.dynManager.getStateObject(thrusters.nameOfKappaState)
    step = macros.sec2nano(1.0)  # [ns]

    def check(tick):
        assert sc.timeBeforeNanos == output.timeWritten() == tick
        factor = state.getState()[0][0]
        assert factor == pytest.approx(math.exp(-tick * macros.NANO2SEC), abs=1e-9, rel=0)
        msg = output()
        assert msg.thrustFactor == factor
        assert msg.thrustForce == pytest.approx(2.5 * factor)  # [N]
        np.testing.assert_allclose(msg.thrustForce_B, force * factor, atol=1e-14)  # [N]
        np.testing.assert_allclose(msg.thrustTorquePntB_B, torque * factor, atol=1e-14)  # [N*m]

    check(0)  # Spacecraft publication before the first scheduled effector update.
    sc.UpdateState(0)
    thrusters.UpdateState(0)
    check(0)
    if mount == "external":
        # The older mount sample and the new hub position must not be combined into a different moment arm.
        position.r_RN_N = [10, 1, 0]  # [m]
        position_msg.write(position, step)
    sc.UpdateState(step)
    check(step)
    thrusters.UpdateState(step)
    factors = rk.safetyFactorForNextStepSize, rk.minimumFactorDecreaseForNextStepSize
    rk.safetyFactorForNextStepSize = rk.minimumFactorDecreaseForNextStepSize = 0.0
    with pytest.raises(RuntimeError, match="representable forward progress"):
        sc.UpdateState(2 * step)
    check(step)
    if mount == "external":
        # Even a replacement stamped at the accepted tick cannot be sampled against a rejected hub pose.
        pose.r_BN_N = [10, 1, 3]  # [m] mount origin is 3 m along hub +Z
        pose.sigma_BN = [0, 0, 0]  # [-] mount +X now points along hub -Y
        mount_msg.write(pose, step)
    thrusters.moduleID += 1  # Distinguish a fresh write at the accepted tick from the previous message.
    thrusters.UpdateState(step)
    assert output.moduleID() == thrusters.moduleID
    check(step)
    if mount == "external":
        mount_msg.write(pose, 2 * step)
        force = np.array([0, -2.5, 0])  # [N]
        torque = np.array([7.5, -0.3, -5])  # [N*m] nozzle at (2, 0, 3) m in the hub frame
    rk.safetyFactorForNextStepSize, rk.minimumFactorDecreaseForNextStepSize = factors
    sc.UpdateState(2 * step)
    check(2 * step)


def test_thruster_first_timestamps_after_rejected_rkf45_step():
    """A thruster scheduled before its spacecraft publishes the accepted tick through rejection and retry."""
    step = macros.sec2nano(1.0)  # [ns]
    sim = SimulationBaseClass.SimBaseClass()
    sim.CreateNewProcess("dynamics").addTask(sim.CreateNewTask("dynamicsTask", step))
    sc = spacecraft.Spacecraft()
    rk = svIntegrators.svIntegratorRKF45(sc)
    rk.relTol = rk.absTol = 1e-12
    sc.setIntegrator(rk)
    thrusters = thrusterStateEffector.ThrusterStateEffector()
    nozzle = thrusterStateEffector.THRSimConfig()
    nozzle.thrLoc_B = [0, 0, 0]  # [m]
    nozzle.thrDir_B = [1, 0, 0]  # [-]
    nozzle.MaxThrust = 1.0  # [N]
    nozzle.steadyIsp = 220.0  # [s]
    nozzle.cutoffFrequency = 1.0  # [1/s]
    thrusters.addThruster(nozzle)
    thrusters.kappaInit = messaging.DoubleVector([1.0])  # [-]
    sc.addStateEffector(thrusters)
    output = thrusters.thrusterOutMsgs[0].addSubscriber()
    before_spacecraft = thrusters.thrusterOutMsgs[0].recorder()
    sim.AddModelToTask("dynamicsTask", thrusters, 3)
    sim.AddModelToTask("dynamicsTask", before_spacecraft, 2)
    sim.AddModelToTask("dynamicsTask", sc, 1)
    sim.InitializeSimulation()
    sim.TotalSim.SingleStepProcesses()  # Initial tick.
    sim.TotalSim.SingleStepProcesses()  # First positive step.
    assert output.timeWritten() == sc.timeBeforeNanos == step
    accepted_factor = output().thrustFactor
    assert accepted_factor == pytest.approx(math.exp(-1.0))

    factors = rk.safetyFactorForNextStepSize, rk.minimumFactorDecreaseForNextStepSize
    rk.safetyFactorForNextStepSize = rk.minimumFactorDecreaseForNextStepSize = 0.0
    with pytest.raises(RuntimeError, match="representable forward progress"):
        sim.TotalSim.SingleStepProcesses()
    assert output.timeWritten() == sc.timeBeforeNanos == step
    assert output().thrustFactor == accepted_factor

    rk.safetyFactorForNextStepSize, rk.minimumFactorDecreaseForNextStepSize = factors
    sim.TotalSim.SingleStepProcesses()
    # The recorder sees the effector's own write before the spacecraft on both attempts.
    np.testing.assert_array_equal(before_spacecraft.timesWritten(), [0, 0, step, step])
    assert output.timeWritten() == sc.timeBeforeNanos == 2 * step
    assert output().thrustFactor == pytest.approx(math.exp(-2.0))


# Run the test
def unitThrusters(testFixture, show_plots, thrustNumber, initialConditions, duration, long_angle, lat_angle, location, swirlTorque, rate, attachBody):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    #  Create a simulation and set the rate
    TotalSim = SimulationBaseClass.SimBaseClass()
    testRate = int(rate)  # Parametrized rate of test

    # breakpoint()

    # Create the process and task
    unitTaskName1 = "unitTask1"  # arbitrary name (don't change)
    unitTaskName2 = "unitTask2"  # arbitrary name (don't change)
    unitTaskName3 = "unitTask3"  # arbitrary name (don't change)
    unitProcessName1 = "TestProcess1"  # arbitrary name (don't change)
    unitProcessName2 = "TestProcess2"  # arbitrary name (don't change)
    unitProcessName3 = "TestProcess3"  # arbitrary name (don't change)
    testProc1 = TotalSim.CreateNewProcess(unitProcessName1, 10)
    testProc1.addTask(TotalSim.CreateNewTask(unitTaskName1, testRate))
    testProc2 = TotalSim.CreateNewProcess(unitProcessName2, 0)
    testProc2.addTask(TotalSim.CreateNewTask(unitTaskName2, testRate))
    testProc3 = TotalSim.CreateNewProcess(unitProcessName3, 5)
    testProc3.addTask(TotalSim.CreateNewTask(unitTaskName3, testRate))

    # Create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    # Constants for thruster creation
    g = 9.80665
    Isp = 226.7

    #  Create the thrusters
    thrusterSet = thrusterStateEffector.ThrusterStateEffector()
    thrusterSet.ModelTag = "ACSThrusterDynamics"

    #  Create thruster characteristic parameters (position, angle thrust, ISP, time of thrust) for thruster 1
    long_angle_deg = long_angle  # Parametrized angle of thrust
    lat_angle_deg = lat_angle
    long_angle_rad = long_angle_deg * math.pi / 180.0
    lat_angle_rad = lat_angle_deg * math.pi / 180.0
    thruster1 = thrusterStateEffector.THRSimConfig()
    thruster1.thrLoc_B = location  # Parametrized location for thruster
    thruster1.thrDir_B = [[math.cos(long_angle_rad) * math.cos(lat_angle_rad)],
                          [math.sin(long_angle_rad) * math.cos(lat_angle_rad)], [math.sin(lat_angle_rad)]]
    thruster1.MaxThrust = 10.0
    thruster1.steadyIsp = 226.7
    thruster1.MinOnTime = 0.006
    thruster1.cutoffFrequency = 5
    thruster1.MaxSwirlTorque = swirlTorque
    thrusterSet.addThruster(thruster1)

    loc1 = np.array([thruster1.thrLoc_B[0][0], thruster1.thrLoc_B[1][0], thruster1.thrLoc_B[2][0]])
    dir1 = np.array([thruster1.thrDir_B[0][0], thruster1.thrDir_B[1][0], thruster1.thrDir_B[2][0]])

    #  Create thruster characteristic parameters for thruster 2
    if thrustNumber == 2:
        thruster2 = thrusterStateEffector.THRSimConfig()
        thruster2.thrLoc_B = np.array([[1.], [0.0], [0.0]]).reshape([3, 1])
        thruster2.thrDir_B = np.array(
            [[math.cos(long_angle_rad + math.pi / 4.) * math.cos(lat_angle_rad - math.pi / 4.)],
             [math.sin(long_angle_rad + math.pi / 4.) * math.cos(lat_angle_rad - math.pi / 4.)],
             [math.sin(lat_angle_rad - math.pi / 4.)]]).reshape([3, 1])
        thruster2.MaxThrust = 20.0
        thruster2.steadyIsp = 226.7
        thruster2.MinOnTime = 0.006
        thruster2.cutoffFrequency = 2

        loc2 = np.array([thruster2.thrLoc_B[0][0], thruster2.thrLoc_B[1][0], thruster2.thrLoc_B[2][0]])
        dir2 = np.array([thruster2.thrDir_B[0][0], thruster2.thrDir_B[1][0], thruster2.thrDir_B[2][0]])

        if attachBody == "ON":
            # Set up the dcm and location
            dcm_BF = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
            r_FB_B = [0, 0, 1]

            # Create the module
            pyModule = attachedBodyModule(dcm_BF, r_FB_B, True, 100)
            pyModule.ModelTag = "attachedBody"
            TotalSim.AddModelToTask(unitTaskName3, pyModule)

            # Attach messages
            pyModule.scInMsg.subscribeTo(scObject.scStateOutMsg)

            # Update the direction and location of the thruster
            dir2 = dcm_BF.dot(dir2)
            loc2 = dcm_BF.dot(loc2) + r_FB_B

            # Attach thruster
            thrusterSet.addThruster(thruster2, pyModule.bodyOutMsg)
        else:
            thrusterSet.addThruster(thruster2)

    # Set the initial conditions
    thrusterSet.kappaInit = messaging.DoubleVector([initialConditions] + [0.0] * (thrustNumber - 1))  # [-]

    # Attach thrusters and add the effector to the spacecraft
    scObject.addStateEffector(thrusterSet)

    # Save state
    dataRec = thrusterSet.thrusterOutMsgs[0].recorder(testRate)

    # Add both modules and the recorder to tasks
    TotalSim.AddModelToTask(unitTaskName1, scObject)
    TotalSim.AddModelToTask(unitTaskName2, thrusterSet)
    TotalSim.AddModelToTask(unitTaskName2, dataRec)

    #  Define the start of the thrust and its duration
    thrDurationTime = macros.sec2nano(2.0)

    # Log variables of interest
    thrusterSetLog = thrusterSet.logger(["forceOnBody_B", "torqueOnBodyPntB_B", "mDotTotal"])
    TotalSim.AddModelToTask(unitTaskName2, thrusterSetLog)

    #  Configure a single thruster firing, create a message for it
    ThrustMessage = messaging.THRArrayOnTimeCmdMsgPayload()
    if thrustNumber == 1:
        ThrustMessage.OnTimeRequest = [duration]
    if thrustNumber == 2:
        ThrustMessage.OnTimeRequest = [duration, 2.]
    thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(ThrustMessage)
    thrusterSet.cmdsInMsg.subscribeTo(thrCmdMsg)

    # Initialize the simulation
    TotalSim.InitializeSimulation()

    # Close all plots
    plt.close("all")

    # Run the simulation
    TotalSim.ConfigureStopTime(TotalSim.TotalSim.CurrentNanos + int(thrDurationTime))
    TotalSim.ExecuteSimulation()

    # Plot the thrust factor if needed
    dataThrustFactor = dataRec.thrustFactor
    plt.figure(1)
    plt.plot(dataRec.times() * macros.NANO2SEC, dataThrustFactor)
    plt.xlabel('Time [s]')
    plt.ylabel('Thrust Factor')
    if show_plots:
        plt.show()

    # Gather the Force, Torque and Mass Rate results
    thrForce = simHelpers.addTimeColumn(thrusterSetLog.times(), thrusterSetLog.forceOnBody_B)
    thrTorque = simHelpers.addTimeColumn(thrusterSetLog.times(), thrusterSetLog.torqueOnBodyPntB_B)
    mDot = simHelpers.addTimeColumn(thrusterSetLog.times(), thrusterSetLog.mDotTotal)

    # Save the time vector
    timeSec = dataRec.times() * macros.NANO2SEC

    # Generate the truth data (force, torque and mass rate)
    expectedThrustData = np.zeros([3, np.shape(thrForce)[0]])
    expectedTorqueData = np.zeros([3, np.shape(thrTorque)[0]])
    expectedMDot = np.zeros([1, np.shape(mDot)[0]])
    for i in range(np.shape(thrForce)[0]):
        if thrustNumber == 1:
            # Compute the thrust force
            if duration == 0.:
                thrustFactor1 = initialConditions * np.exp(- thruster1.cutoffFrequency * timeSec[i])
                force1 = thrustFactor1 * thruster1.MaxThrust * dir1
                expectedThrustData[0:3, i] = force1
            else:
                thrustFactor1 = (1.0 + (initialConditions - 1.0) * np.exp(- thruster1.cutoffFrequency * timeSec[i]))
                force1 = thrustFactor1 * thruster1.MaxThrust * dir1
                expectedThrustData[0:3, i] = force1
            # Compute the torque
            expectedTorqueData[0:3, i] = np.cross(loc1, force1) + thrustFactor1 * swirlTorque * dir1
            # Compute the mass flow rate
            expectedMDot[0, i] = thruster1.MaxThrust * thrustFactor1 / (g * Isp)
        else:
            # Compute the thrust force
            if duration == 0.:
                thrustFactor1 = initialConditions * np.exp(- thruster1.cutoffFrequency * timeSec[i])
                thrustFactor2 = (1.0 - np.exp(- thruster2.cutoffFrequency * timeSec[i]))
                force1 = thrustFactor1 * thruster1.MaxThrust * dir1
                force2 = thrustFactor2 * thruster2.MaxThrust * dir2
                expectedThrustData[0:3, i] = force1 + force2
            else:
                thrustFactor1 = (1.0 + (initialConditions - 1.0) * np.exp(- thruster1.cutoffFrequency * timeSec[i]))
                thrustFactor2 = (1.0 - np.exp(- thruster2.cutoffFrequency * timeSec[i]))
                force1 = thrustFactor1 * thruster1.MaxThrust * dir1
                force2 = thrustFactor2 * thruster2.MaxThrust * dir2
                expectedThrustData[0:3, i] = force1 + force2
            # Compute the torque
            expectedTorqueData[0:3, i] = np.cross(loc1, force1) + thrustFactor1 * swirlTorque * dir1 + np.cross(loc2, force2)
            # Compute the mass flow rate
            expectedMDot[0, i] = (thruster1.MaxThrust * thrustFactor1 + thruster2.MaxThrust * thrustFactor2) / (g * Isp)

    # Modify expected values for comparison and define errorTolerance
    TruthForce = np.transpose(expectedThrustData)
    TruthTorque = np.transpose(expectedTorqueData)
    TruthMDot = np.transpose(expectedMDot)
    ErrTolerance = 1E-3

    # Compare Force values (exclude first element because of python process priority)
    thrForce = np.delete(thrForce, 0, axis=1)  # remove time column
    testFailCount, testMessages = unitTestSupport.compareArray(TruthForce[1:, :], thrForce[1:, :], ErrTolerance, "Force",
                                                               testFailCount, testMessages)

    # Compare Torque values (exclude first element because of python process priority)
    thrTorque = np.delete(thrTorque, 0, axis=1)  # remove time column
    testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque[1:, :], thrTorque[1:, :], ErrTolerance, "Torque",
                                                               testFailCount, testMessages)

    # Compare mass flow rate values
    mDot = np.delete(mDot, 0, axis=1)
    ErrTolerance = 1E-6
    testFailCount, testMessages = unitTestSupport.compareArray(np.transpose(TruthMDot), np.transpose(mDot), ErrTolerance, "MDot",
                                                               testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED")
        testFixture.PassFail.append("PASSED")
    else:
        testFixture.PassFail.append("FAILED")
        print(testMessages)

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


class attachedBodyModule(sysModel.SysModel):
    def __init__(self, dcm_BF, r_FB_B, modelActive=True, modelPriority=-1):
        super(attachedBodyModule, self).__init__()

        # Input spacecraft state structure message
        self.scInMsg = messaging.SCStatesMsgReader()
        self.scMsgBuffer = None

        # Output body state message
        self.bodyOutMsg = messaging.SCStatesMsg()

        # Save dcm and location
        self.dcm_BF = dcm_BF
        self.r_FB_B = r_FB_B

    def UpdateState(self, CurrentSimNanos):
        # Read input message
        self.scMsgBuffer = self.scInMsg()

        # Write output message
        self.writeOutputMsg(CurrentSimNanos)

    def writeOutputMsg(self, CurrentSimNanos):
        # Create output message buffer
        bodyOutMsgBuffer = messaging.SCStatesMsgPayload()

        # Grab the spacecraft hub states
        sigma_BN = self.scMsgBuffer.sigma_BN
        dcm_BN = rbk.MRP2C(sigma_BN)
        omega_BN_B = self.scMsgBuffer.omega_BN_B
        r_BN_N = self.scMsgBuffer.r_BN_N

        # Compute the attached body states relative to the hub
        dcm_FB = np.transpose(self.dcm_BF)
        sigma_FB = rbk.C2MRP(dcm_FB)
        sigma_FN = rbk.addMRP(np.array(sigma_BN), sigma_FB)
        omega_FB_F = dcm_FB.dot(omega_BN_B)
        r_FN_N = r_BN_N + np.transpose(dcm_BN).dot(np.array(self.r_FB_B))

        # Write the output message information
        bodyOutMsgBuffer.sigma_BN = sigma_FN
        bodyOutMsgBuffer.omega_BN_B = omega_FB_F
        bodyOutMsgBuffer.r_BN_N = r_FN_N
        self.bodyOutMsg.write(bodyOutMsgBuffer, CurrentSimNanos, self.moduleID)


if __name__ == "__main__":
    unitThrusters(ResultsStore(), False, 2, 0., 2.0, 30., 15., [[1.125], [0.5], [2.0]], 0.0, macros.sec2nano(0.01), "ON")
