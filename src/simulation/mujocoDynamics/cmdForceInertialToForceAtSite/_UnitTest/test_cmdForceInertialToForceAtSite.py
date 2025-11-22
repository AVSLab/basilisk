import pytest
import numpy as np
import numpy.testing as npt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJCmdForceInertialToForceAtSite
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_cmdForceInertialToForceAtSiteSCStates():
    dt = 1.0

    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("t", scene)

    # Inputs
    F_N = np.array([10.0, -2.0, 5.0])  # N in inertial frame
    sigma_BN = np.array([0.1, 0.2, 0.3])  # MRPs for S=B wrt N

    cmdMsg = messaging.CmdForceInertialMsg()
    cmdMsg.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=F_N.tolist()))

    scMsg = messaging.SCStatesMsg()
    scMsg.write(messaging.SCStatesMsgPayload(sigma_BN=sigma_BN.tolist()))

    mod = MJCmdForceInertialToForceAtSite.CmdForceInertialToForceAtSite()
    mod.cmdForceInertialInMsg.subscribeTo(cmdMsg)
    mod.siteFrameStateInMsg.subscribeTo(scMsg)
    scene.AddModelToDynamicsTask(mod)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(dt))
    scSim.ExecuteSimulation()

    F_S = np.array(mod.forceOutMsg.read().force_S, dtype=float)

    # Expected: F_S = C_SN * F_N, and C_SN == C_BN from MRPs
    C_BN = np.array(rbk.MRP2C(sigma_BN))
    F_S_exp = C_BN.dot(F_N)

    npt.assert_allclose(F_S, F_S_exp, rtol=0.0, atol=1e-12)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_cmdForceInertialToForceAtSiteNavAtt():
    dt = 1.0

    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("t", scene)

    F_N = np.array([3.0, 4.0, 0.0])
    sigmaAtt_BN = np.array([0.05, -0.02, 0.01])  # will be used
    sigmaState_BN = np.array([0.3, 0.0, 0.0])    # should be ignored if NavAtt is linked

    cmdMsg = messaging.CmdForceInertialMsg()
    cmdMsg.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=F_N.tolist()))

    attMsg = messaging.NavAttMsg()
    attMsg.write(messaging.NavAttMsgPayload(sigma_BN=sigmaAtt_BN.tolist()))

    scMsg = messaging.SCStatesMsg()
    scMsg.write(messaging.SCStatesMsgPayload(sigma_BN=sigmaState_BN.tolist()))

    mod = MJCmdForceInertialToForceAtSite.CmdForceInertialToForceAtSite()
    mod.cmdForceInertialInMsg.subscribeTo(cmdMsg)
    mod.siteAttInMsg.subscribeTo(attMsg)          # preferred
    mod.siteFrameStateInMsg.subscribeTo(scMsg)    # fallback
    scene.AddModelToDynamicsTask(mod)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(dt))
    scSim.ExecuteSimulation()

    F_S = np.array(mod.forceOutMsg.read().force_S, dtype=float)

    C_BN_att = np.array(rbk.MRP2C(sigmaAtt_BN))
    F_S_exp = C_BN_att.dot(F_N)

    npt.assert_allclose(F_S, F_S_exp, rtol=0.0, atol=1e-12)


if __name__ == "__main__":
    assert couldImportMujoco
    test_cmdForceInertialToForceAtSiteSCStates()
    test_cmdForceInertialToForceAtSiteNavAtt()
