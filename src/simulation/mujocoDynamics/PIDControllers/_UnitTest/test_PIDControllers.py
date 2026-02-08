import pytest
import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.utilities import pythonVariableLogger

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJPIDControllers

    couldImportMujoco = True
except:
    couldImportMujoco = False

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_jointPIDController(showPlots: bool = False):
    """
    Unit test for JointPIDController.
    Verifies that the controller computes the correct output for given input messages.
    Plots joint angle, velocity, and control torque if plot=True.
    """
    dt = .1
    tf = 50

    desiredPosition = np.pi/2
    desiredVelocity = desiredPosition / tf

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create the MJScene from a simple cannonball body
    scene = mujoco.MJScene(
r"""
<mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
        <body name="arm">
            <geom name="arm" type="capsule" size="0.1" fromto="0 0 0 0 0 1"/>
            <joint name="arm" axis="1 0 0"/>
        </body>
    </worldbody>
</mujoco>
"""
    )
    scene.extraEoMCall = True
    scSim.AddModelToTask("test", scene)

    armJoint: mujoco.MJScalarJoint = scene.getBody("arm").getScalarJoint("arm")
    armMotor: mujoco.MJSingleActuator = scene.addJointSingleActuator("arm", armJoint)

    desiredPosInterpolator = mujoco.ScalarJointStateInterpolator()
    desiredPosInterpolator.setDataPoints([
        [0, 0],
        [macros.sec2nano(tf), desiredPosition]
    ])
    scene.AddModelToDynamicsTask(desiredPosInterpolator)

    armController = MJPIDControllers.JointPIDController()
    armController.setProportionalGain(1)
    armController.setDerivativeGain(2)
    armController.setIntegralGain(0.02)
    scene.AddModelToDynamicsTask(armController)

    payload = messaging.ScalarJointStateMsgPayload()
    payload.state = desiredVelocity # constant
    desiredVelMsg = messaging.ScalarJointStateMsg().write(payload)

    armController.measuredPosInMsg.subscribeTo( armJoint.stateOutMsg )
    armController.measuredVelInMsg.subscribeTo( armJoint.stateDotOutMsg )

    armController.desiredPosInMsg.subscribeTo( desiredPosInterpolator.interpolatedOutMsg )
    armController.desiredVelInMsg.subscribeTo( desiredVelMsg )

    armMotor.actuatorInMsg.subscribeTo( armController.outputOutMsg )

    desiredJointAngleRecorder = desiredPosInterpolator.interpolatedOutMsg.recorder()
    jointAngleRecorder = armJoint.stateOutMsg.recorder()
    jointVelocityRecorder = armJoint.stateDotOutMsg.recorder()
    torqueRecorder = armController.outputOutMsg.recorder()
    integralErrorLogger = pythonVariableLogger.PythonVariableLogger(
        {"error": lambda _: armController.getIntegralError()}
    )

    scSim.AddModelToTask("test", desiredJointAngleRecorder)
    scSim.AddModelToTask("test", jointAngleRecorder)
    scSim.AddModelToTask("test", jointVelocityRecorder)
    scSim.AddModelToTask("test", torqueRecorder)
    scSim.AddModelToTask("test", integralErrorLogger)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    t = jointAngleRecorder.times()
    pos  = jointAngleRecorder.state
    desiredPos = desiredJointAngleRecorder.state
    vel = jointVelocityRecorder.state
    torque = torqueRecorder.input
    integralError = integralErrorLogger.error

    # Plot results
    if showPlots:
        plt.figure(figsize=(10,6))
        plt.subplot(3,1,1)
        plt.plot(t, pos, label="Joint Angle")
        plt.plot(t, desiredPos, color='r', linestyle='--', label="Desired Angle")
        plt.ylabel("Angle [rad]")
        plt.legend()
        plt.subplot(3,1,2)
        plt.plot(t, vel, label="Joint Velocity")
        plt.plot(t[:-1], np.diff(pos)/np.diff(t), linestyle=":", label="dx/dt")
        plt.axhline(desiredVelocity, color='r', linestyle='--', label="Desired Velocity")
        plt.ylabel("Velocity [rad/s]")
        plt.legend()
        plt.subplot(3,1,3)
        plt.plot(t, torque, label="Control Torque")
        plt.ylabel("Torque [Nm]")
        plt.xlabel("Time [s]")
        plt.legend()
        plt.tight_layout()
        plt.show()

    # verify PID formula
    for i in range(len(torque)):
        expected_output = (
            armController.getProportionalGain() * (desiredPos[i] - pos[i]) +
            armController.getDerivativeGain() * (desiredVelocity - vel[i]) +
            armController.getIntegralGain() * integralError[i]
        )
        assert torque[i] == pytest.approx(expected_output, abs=1e-6), \
            f"At t={t[i]:.2f}s, PID output {torque[i]} != expected {expected_output}"

    # Assert final joint position is close to desired
    assert pos[-1] == pytest.approx(desiredPosition, rel=1e-2), \
        f"Final joint position {pos[-1]} not close to desired {desiredPosition}"

    # Assert velocity settles near zero
    assert vel[-1] == pytest.approx(desiredVelocity, rel=1e-2), \
        f"Final joint velocity {vel[-1]} not close to desired {desiredVelocity}"

if __name__ == "__main__":
    test_jointPIDController(True)
