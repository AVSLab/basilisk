#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import os
import pytest

from Basilisk import hasBuildFeature

mujocoEnabled = hasBuildFeature("mujoco")
pytestmark = pytest.mark.skipif(
    not mujocoEnabled,
    reason="Requires Basilisk built with --mujoco True",
)
if mujocoEnabled:
    from Basilisk.simulation import linearTranslationOneDOFStateEffector, mujoco, spacecraft, svIntegrators

from Basilisk.architecture import messaging, sysModel
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

import numpy as np
import pytest

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{TEST_FOLDER}/sat_hub_only.xml"

ROTATED_SITE_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
      <site name="rotated" quat="0.7071067811865476 0 0 0.7071067811865476"/>
    </body>
  </worldbody>
</mujoco>
"""

SLIDER_SITE_XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="slider">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.1" mass="1"/>
      <site name="tip"/>
    </body>
  </worldbody>
</mujoco>
"""


class PublicationProbe(sysModel.SysModel):
    """Record site-message header state from within integration stages."""

    def __init__(self, reader):
        super().__init__()
        self.reader = reader
        self.samples = []

    def UpdateState(self, CurrentSimNanos):
        """Record whether the site message was written at this stage time."""
        is_written = self.reader.isWritten()
        write_time = self.reader.timeWritten() if is_written else None
        self.samples.append((CurrentSimNanos, is_written, write_time))


class DirectJointStateMutator(sysModel.SysModel):
    """Mutate joint records without using scene-aware joint setters."""

    def __init__(self, joint):
        super().__init__()
        self.joint = joint

    def UpdateState(self, CurrentSimNanos):
        self.joint.getPositionState().setState([[2.5]])
        self.joint.getVelocityState().setState([[-0.75]])


class SiteStateProbe(sysModel.SysModel):
    def __init__(self, site):
        super().__init__()
        self.site = site
        self.samples = []

    def UpdateState(self, CurrentSimNanos):
        state = self.site.stateOutMsg.read()
        self.samples.append(
            (state.r_BN_N[0], state.v_BN_N[0])
        )


def test_siteVelocity():
    """Test the velocity output of sites."""

    dt = 1 # s
    tf = 30 # s

    # Create sim, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create scene
    scene = mujoco.MJScene.fromFile(XML_PATH)
    scSim.AddModelToTask("test", scene)

    # Set initial conditions
    initialPosition = [1.5,-1,0.75]
    initialVelocity = [5e-3,-5e-3,5e-3]
    initialAttitude = [0.15,-0.1,0.1]
    initialAttitudeRate = [0.01,-0.01,0.005]

    # Save body state through recorder
    bodyStateRecorder = scene.getBody("hub").getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    # Initialize sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))

    # Set initial states
    scene.getBody("hub").setPosition(initialPosition)
    scene.getBody("hub").setVelocity(initialVelocity)
    scene.getBody("hub").setAttitude(initialAttitude)
    scene.getBody("hub").setAttitudeRate(initialAttitudeRate)

    # run sim
    scSim.ExecuteSimulation()

    # validate the simulation output values
    v_BN_N = bodyStateRecorder.v_BN_N[-1]
    omega_BN_B = bodyStateRecorder.omega_BN_B[-1]

    np.testing.assert_allclose(v_BN_N, np.array(initialVelocity), rtol=1e-8,
                               err_msg=f"Linear velocity {v_BN_N} not close to expected {initialVelocity}")
    np.testing.assert_allclose(omega_BN_B, np.array(initialAttitudeRate), rtol=1e-8,
                               err_msg=f"Angular velocity {omega_BN_B} not close to expected {initialAttitudeRate}")


def test_rotated_site_angular_velocity():
    """A rotated site reports global angular velocity in its local frame."""
    dt = 0.1  # [s]
    sc_sim = SimulationBaseClass.SimBaseClass()
    process = sc_sim.CreateNewProcess("test")
    process.addTask(sc_sim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene(ROTATED_SITE_XML)
    sc_sim.AddModelToTask("test", scene)
    sc_sim.InitializeSimulation()
    scene.getBody("hub").setAttitudeRate([0.1, 0.2, 0.3])
    sc_sim.ConfigureStopTime(macros.sec2nano(dt))
    sc_sim.ExecuteSimulation()

    state = scene.getSite("rotated").stateOutMsg.read()
    np.testing.assert_allclose(
        state.omega_BN_B,
        [0.2, -0.1, 0.3],
        atol=1e-14,
    )


def test_explicit_forward_detects_direct_joint_state_mutation():
    scene = mujoco.MJScene(SLIDER_SITE_XML)
    joint = scene.getBody("slider").getScalarJoint("slide")
    site = scene.getSite("tip")
    mutator = DirectJointStateMutator(joint)
    probe = SiteStateProbe(site)

    scene.AddModelToDynamicsTask(mutator, 100)
    scene.AddFwdKinematicsToDynamicsTask(50)
    scene.AddModelToDynamicsTask(probe, 0)
    scene.Reset(0)

    scene.integrateState(1)

    assert probe.samples
    np.testing.assert_allclose(
        probe.samples,
        np.tile([2.5, -0.75], (len(probe.samples), 1)),
        atol=1e-14,
    )


@pytest.mark.parametrize("extra_eom_call", [False, True])
def test_site_messages_publish_stage_and_final_states(extra_eom_call):
    """Joint-state integration preserves stage and final site publication."""
    dt = 0.1  # [s]
    sc_sim = SimulationBaseClass.SimBaseClass()
    process = sc_sim.CreateNewProcess("test")
    process.addTask(sc_sim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene(ROTATED_SITE_XML)
    scene.extraEoMCall = extra_eom_call
    sc_sim.AddModelToTask("test", scene)

    site = scene.getSite("rotated")
    reader = site.stateOutMsg.addSubscriber()
    probe = PublicationProbe(reader)
    scene.AddModelToDynamicsTask(probe, 0)

    sc_sim.InitializeSimulation()
    scene.getBody("hub").setPosition([2.0, 0.0, 0.0])  # [m]
    scene.getBody("hub").setVelocity([1.0, 0.0, 0.0])  # [m/s]
    sc_sim.ConfigureStopTime(macros.sec2nano(dt))
    sc_sim.ExecuteSimulation()

    positive_stage_samples = [
        sample for sample in probe.samples if sample[0] > 0
    ]
    assert positive_stage_samples
    assert all(
        is_written and write_time == stage_time
        for stage_time, is_written, write_time in positive_stage_samples
    )
    assert reader.isWritten()
    final_state = site.stateOutMsg.read()
    assert reader.timeWritten() == macros.sec2nano(dt)
    assert final_state.r_BN_N[0] == pytest.approx(2.0 + dt)


@pytest.mark.parametrize("extra_eom_call", [False, True])
def test_accepted_clocks_at_multi_year_epoch(extra_eom_call):
    """Check the integer publications and the native clock through a reset and an Euler step."""
    scene = mujoco.MJScene(SLIDER_SITE_XML)
    scene.extraEoMCall = extra_eom_call
    scene.setIntegrator(svIntegrators.svIntegratorEuler(scene))
    body = scene.getBody("slider")
    joint = body.getScalarJoint("slide")
    site = body.getOrigin().stateOutMsg.addSubscriber()
    mass = body.massPropertiesOutMsg.addSubscriber()
    mass_rate = messaging.SCMassPropsMsgPayload()
    mass_rate.massSC = 0.25  # [kg/s]
    mass_rate_msg = messaging.SCMassPropsMsg().write(mass_rate)
    body.derivativeMassPropertiesInMsg.subscribeTo(mass_rate_msg)
    readers = [site, scene.stateOutMsg.addSubscriber(), mass,
               joint.stateOutMsg.addSubscriber(), joint.stateDotOutMsg.addSubscriber()]
    samples = []

    class ClockProbe(sysModel.SysModel):
        def Reset(self, tick):
            samples.append((scene.getEvaluationTime(), tick))

        def UpdateState(self, tick):
            samples.append((scene.getEvaluationTime(), tick))

    probe = ClockProbe()
    scene.AddModelToDynamicsTask(probe)
    scene.AddModelToDiffusionDynamicsTask(probe)
    start = 126_230_400_000_000_003  # [ns] four Julian years plus three nanoseconds
    step = 1_000_000_001  # [ns]
    scene.Reset(start)
    assert samples == [(start / 10**9, start)]
    samples.clear()
    joint.setVelocity(1.0)  # [m/s]
    scene.UpdateState(start)
    assert samples == [(start / 10**9, start)] * (3 if extra_eom_call else 1)
    samples.clear()
    end = start + step
    scene.UpdateState(end)
    assert samples == [(start / 10**9, start)] + ([(end / 10**9, end)] * 2 if extra_eom_call else [])
    assert scene.getEvaluationTime() == end / 10**9
    assert all(reader.timeWritten() == end for reader in readers)
    assert site().r_BN_N[0] == pytest.approx(step * macros.NANO2SEC, rel=0, abs=1e-14)
    assert mass().massSC == pytest.approx(1.0 + mass_rate.massSC * step * macros.NANO2SEC)  # [kg]


@pytest.mark.parametrize("offset, step", [(0, 3_906_250), (8, 1_000_000_000), (3, 5)])  # [ns]
def test_rk4_stage_timestamps_at_multi_year_epoch(offset, step):
    """RK4 task and site timestamps follow the integer interval even when stage seconds coincide."""
    primary, secondary = mujoco.MJScene(SLIDER_SITE_XML), mujoco.MJScene(SLIDER_SITE_XML)
    primary.setIntegrator(svIntegrators.svIntegratorRK4(primary))
    primary.syncDynamicsIntegration(secondary)

    class ClockProbe(sysModel.SysModel):
        def __init__(self, scene):
            super().__init__()
            self.scene = scene
            self.site = scene.getSite("tip").stateOutMsg.addSubscriber()
            self.samples = []

        def UpdateState(self, tick):
            self.samples.append((tick, self.site.timeWritten(), self.scene.getEvaluationTime()))

    start = 126_230_400_000_000_000 + offset  # [ns] four Julian years plus the offset
    probes = [ClockProbe(scene) for scene in (primary, secondary)]
    for probe in probes:
        probe.scene.AddModelToDynamicsTask(probe)
        probe.scene.Reset(start)
    primary.UpdateState(start + step)
    stages = [start, start + step // 2, start + step // 2, start + step]
    seconds = [start / 10**9 + fraction * (step * macros.NANO2SEC) for fraction in (0.0, 0.5, 0.5, 1.0)]
    for probe in probes:
        assert probe.samples == list(zip(stages, stages, seconds))
        assert probe.site.timeWritten() == start + step


@pytest.mark.parametrize("solver", ["svIntegratorEuler", "svStochasticIntegratorMayurama"])
def test_invalid_final_mass_rolls_back_synchronized_step(solver):
    """Reject the secondary's final candidate and retry the same tick from both entry states."""
    primary, secondary = mujoco.MJScene(SLIDER_SITE_XML), mujoco.MJScene(SLIDER_SITE_XML)
    primary.setIntegrator(getattr(svIntegrators, solver)(primary))
    primary.syncDynamicsIntegration(secondary)
    rates = messaging.SCMassPropsMsg()
    rate = messaging.SCMassPropsMsgPayload()
    rates.write(rate)
    secondary.getBody("slider").derivativeMassPropertiesInMsg.subscribeTo(rates)
    speeds = (1.0, 2.0)  # [m/s]
    for scene, speed in zip((primary, secondary), speeds):
        scene.Reset(0)
        scene.getBody("slider").getScalarJoint("slide").setVelocity(speed)

    def check(tick):
        for scene, speed in zip((primary, secondary), speeds):
            body = scene.getBody("slider")
            state = scene.stateOutMsg.addSubscriber()
            mass = body.massPropertiesOutMsg.addSubscriber()
            site = scene.getSite("tip").stateOutMsg.addSubscriber()
            x = speed * tick * macros.NANO2SEC  # [m]
            assert scene.timeBeforeNanos == state.timeWritten() == mass.timeWritten() == site.timeWritten() == tick
            assert scene.getEvaluationTime() == tick * macros.NANO2SEC
            assert body.getMass() == mass().massSC == 1.0  # [kg]
            assert body.getScalarJoint("slide").getPositionState().getState()[0][0] == x
            assert state().qpos[0][0] == site().r_BN_N[0] == x

    step = macros.sec2nano(1.0)  # [ns]
    primary.UpdateState(step)
    check(step)
    rate.massSC = -2.0  # [kg/s] both solvers evaluate the derivative at the valid entry mass
    rates.write(rate)
    with pytest.raises(RuntimeError, match="finite and nonnegative"):
        primary.UpdateState(2 * step)
    check(step)
    rate.massSC = 0.0  # [kg/s]
    rates.write(rate)
    primary.UpdateState(2 * step)
    check(2 * step)


@pytest.mark.parametrize("primary_is_spacecraft", [True, False])
@pytest.mark.parametrize("link_during_stage", [False, True])
def test_late_point_mass_attitude_reference(primary_is_spacecraft, link_during_stage):
    """Reject an unsupported reference before either model commits and allow retry after disconnect."""
    sc, scene = spacecraft.Spacecraft(), mujoco.MJScene(SLIDER_SITE_XML)
    sc.pointMassTranslationalOnly = True
    sc.hub.mHub = 1.0  # [kg]
    hub_speed, slide_speed = 1.0, -2.0  # [m/s]
    sc.hub.v_CN_NInit = [hub_speed, 0.0, 0.0]  # [m/s]
    primary, secondary = (sc, scene) if primary_is_spacecraft else (scene, sc)
    primary.setIntegrator(svIntegrators.svIntegratorRK4(primary))
    primary.syncDynamicsIntegration(secondary)
    scene.extraEoMCall = False
    reference = messaging.AttRefMsg()
    stages = []
    link_in_callback = False

    class StageProbe(sysModel.SysModel):
        def UpdateState(self, tick):
            stages.append(tick)
            if link_in_callback:
                sc.attRefInMsg.subscribeTo(reference)

    probe = StageProbe()
    scene.AddModelToDynamicsTask(probe)
    hub = sc.scStateOutMsg.addSubscriber()
    state = scene.stateOutMsg.addSubscriber()
    sc.Reset(0)
    scene.Reset(0)
    slide = scene.getBody("slider").getScalarJoint("slide")
    slide.setVelocity(slide_speed)
    position = sc.dynManager.getStateObject(sc.hub.nameOfHubPosition)
    velocity = sc.dynManager.getStateObject(sc.hub.nameOfHubVelocity)

    def check(tick):
        t = tick * macros.NANO2SEC  # [s]
        assert sc.timeBeforeNanos == scene.timeBeforeNanos == hub.timeWritten() == state.timeWritten() == tick
        assert scene.getEvaluationTime() == t
        assert hub().r_BN_N[0] == position.getState()[0][0] == pytest.approx(hub_speed * t)
        assert hub().v_BN_N[0] == velocity.getState()[0][0] == hub_speed
        assert state().qpos[0][0] == slide.getPositionState().getState()[0][0] == pytest.approx(slide_speed * t)
        assert state().qvel[0][0] == slide.getVelocityState().getState()[0][0] == slide_speed

    step = macros.sec2nano(1.0)  # [ns]
    primary.UpdateState(step)
    check(step)
    stages.clear()
    if link_during_stage:
        reference.write(messaging.AttRefMsgPayload(), step)
        link_in_callback = True
    else:
        sc.attRefInMsg.subscribeTo(reference)  # Even an unwritten attitude reference is unsupported.
    with pytest.raises(BasiliskError, match="does not support attitude reference"):
        primary.UpdateState(2 * step)
    assert bool(stages) == link_during_stage
    check(step)
    link_in_callback = False
    sc.attRefInMsg.unsubscribe()
    primary.UpdateState(2 * step)
    check(2 * step)


@pytest.mark.parametrize("primary_is_spacecraft", [True, False])
def test_rejected_same_tick_hub_references(primary_is_spacecraft):
    """Withdrawing references after rejection preserves the accepted hub pose and velocity on retry."""
    sc, scene = spacecraft.Spacecraft(), mujoco.MJScene(SLIDER_SITE_XML)
    sc.hub.mHub = 1.0  # [kg]
    sc.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
    sc.hub.r_CN_NInit = [1.0, 2.0, 3.0]  # [m]
    sc.hub.v_CN_NInit = [0.25, 0.0, 0.0]  # [m/s]
    sc.hub.sigma_BNInit = [0.1, 0.0, 0.0]  # [-]
    sc.hub.omega_BN_BInit = [0.05, 0.0, 0.0]  # [rad/s]
    primary, secondary = (sc, scene) if primary_is_spacecraft else (scene, sc)
    primary.setIntegrator(svIntegrators.svIntegratorRK4(primary))
    primary.syncDynamicsIntegration(secondary)
    hub = sc.scStateOutMsg.addSubscriber()

    def hub_states():
        names = (sc.hub.nameOfHubPosition, sc.hub.nameOfHubVelocity, sc.hub.nameOfHubSigma, sc.hub.nameOfHubOmega)
        return np.array([sc.dynManager.getStateObject(name).getState() for name in names]).reshape(4, 3)

    def published():
        msg = hub()
        return np.array([msg.r_BN_N, msg.v_BN_N, msg.sigma_BN, msg.omega_BN_B])

    class RejectTrial(sysModel.SysModel):
        def UpdateState(self, tick):
            if reject:
                trial.append(hub_states())
                raise RuntimeError("reject same-tick stage")

    reject, trial = False, []
    probe = RejectTrial()
    scene.AddModelToDynamicsTask(probe)
    scene.extraEoMCall = False
    sc.Reset(0)
    scene.Reset(0)
    step = macros.sec2nano(1.0)  # [ns]
    primary.UpdateState(step)
    accepted = published()
    np.testing.assert_array_equal(hub_states(), accepted)

    trans = messaging.TransRefMsgPayload()
    trans.r_RN_N = [10.0, 0.0, 0.0]  # [m]
    trans.v_RN_N = [0.0, 2.0, 0.0]  # [m/s]
    att = messaging.AttRefMsgPayload()
    att.sigma_RN = [0.0, 0.0, 0.1]  # [-]
    att.omega_RN_N = [0.0, 0.0, 0.5]  # [rad/s] along the rotation axis
    trans_msg = messaging.TransRefMsg().write(trans, step)
    att_msg = messaging.AttRefMsg().write(att, step)
    sc.transRefInMsg.subscribeTo(trans_msg)
    sc.attRefInMsg.subscribeTo(att_msg)
    reject = True
    with pytest.raises(RuntimeError, match="director method error"):
        primary.UpdateState(step)
    np.testing.assert_allclose(trial[0], [trans.r_RN_N, trans.v_RN_N, att.sigma_RN, att.omega_RN_N])
    np.testing.assert_array_equal(hub_states(), accepted)
    np.testing.assert_array_equal(published(), accepted)
    assert sc.timeBeforeNanos == scene.timeBeforeNanos == hub.timeWritten() == step

    sc.transRefInMsg.unsubscribe()
    sc.attRefInMsg.unsubscribe()
    reject = False
    sc.moduleID += 1  # Distinguish a fresh publication at the same timestamp.
    primary.UpdateState(step)
    assert hub.moduleID() == sc.moduleID
    np.testing.assert_array_equal(hub_states(), accepted)
    np.testing.assert_array_equal(published(), accepted)


@pytest.mark.parametrize("failure", [None, "drift", "diffusion", "invalid_mass"])
def test_accepted_callback_reads_published_peer(failure):
    """Accepted callbacks see the peer and leave local joint outputs consistent even on failure."""
    primary, secondary = mujoco.MJScene(SLIDER_SITE_XML), mujoco.MJScene(SLIDER_SITE_XML)
    primary.setIntegrator(svIntegrators.svIntegratorEuler(primary))
    primary.syncDynamicsIntegration(secondary)
    primary.extraEoMCall = True
    site = secondary.getSite("tip").stateOutMsg.addSubscriber()
    state = secondary.stateOutMsg.addSubscriber()
    joint = primary.getBody("slider").getScalarJoint("slide")
    position, velocity = 3.0, -0.75  # [m], [m/s]
    samples = []

    class PeerProbe(sysModel.SysModel):
        def __init__(self, task):
            super().__init__()
            self.task = task

        def UpdateState(self, tick):
            if tick == end:
                samples.append((site.timeWritten(), site().r_BN_N[0], state.timeWritten(),
                                state().qpos[0][0], secondary.getEvaluationTime()))
                if self.task == "drift":
                    joint.getPositionState().setState([[position]])
                    joint.getVelocityState().setState([[velocity]])
                if self.task == "diffusion" and failure == "invalid_mass":
                    primary.getMassState().setState([[0.0], [-1.0]])  # [kg]
                if self.task == failure:
                    raise RuntimeError(f"deliberate accepted {failure} callback failure")

    end = macros.sec2nano(1.0)  # [ns]
    drift, diffusion = PeerProbe("drift"), PeerProbe("diffusion")
    primary.AddModelToDynamicsTask(drift)
    primary.AddModelToDiffusionDynamicsTask(diffusion)
    for scene in (primary, secondary):
        scene.Reset(0)
    secondary.getBody("slider").getScalarJoint("slide").setVelocity(2.0)  # [m/s]
    primary.UpdateState(0)
    if failure:
        error = "finite and nonnegative" if failure == "invalid_mass" else "director method error"
        with pytest.raises(RuntimeError, match=error):
            primary.UpdateState(end)
    else:
        primary.UpdateState(end)
    assert samples == [(end, 2.0, end, 2.0, 1.0)] * (1 if failure == "drift" else 2)  # [ns], [m], [ns], [m], [s]
    for scene in (primary, secondary):
        assert scene.timeBeforeNanos == scene.stateOutMsg.addSubscriber().timeWritten() == end
    scene_state = primary.stateOutMsg.addSubscriber()
    primary_site = primary.getSite("tip").stateOutMsg.addSubscriber()
    q, v = joint.stateOutMsg.addSubscriber(), joint.stateDotOutMsg.addSubscriber()
    assert all(reader.timeWritten() == end for reader in (scene_state, primary_site, q, v))
    assert joint.getPositionState().getState()[0][0] == scene_state().qpos[0][0] == position
    assert primary_site().r_BN_N[0] == q().state == position
    assert joint.getVelocityState().getState()[0][0] == scene_state().qvel[0][0] == velocity
    assert primary_site().v_BN_N[0] == v().state == velocity
    body = primary.getBody("slider")
    mass = body.massPropertiesOutMsg.addSubscriber()
    assert mass.timeWritten() == end
    assert mass().massSC == body.getMass() == 1.0  # [kg]


@pytest.mark.parametrize("primary_is_spacecraft", [True, False])
@pytest.mark.parametrize("callback_fails", [False, True])
def test_accepted_callback_reads_spacecraft_and_effector(primary_is_spacecraft, callback_fails):
    """A MuJoCo callback sees the accepted hub and slider telemetry, even when it raises."""
    sc, scene = spacecraft.Spacecraft(), mujoco.MJScene(SLIDER_SITE_XML)
    sc.hub.mHub = 1.0  # [kg]
    sc.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
    sc.hub.v_CN_NInit = [0.0, 2.0, 0.0]  # [m/s]
    slider = linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector()
    slider.setMass(1.0)  # [kg]
    slider.setRhoDotInit(1.0)  # [m/s] along the hub's x axis
    sc.addStateEffector(slider)
    primary, secondary = (sc, scene) if primary_is_spacecraft else (scene, sc)
    primary.setIntegrator(svIntegrators.svIntegratorEuler(primary))
    primary.syncDynamicsIntegration(secondary)
    scene.extraEoMCall = True

    hub = sc.scStateOutMsg.addSubscriber()
    mass = sc.scMassOutMsg.addSubscriber()
    joint = slider.translatingBodyOutMsg.addSubscriber()
    body = slider.translatingBodyConfigLogOutMsg.addSubscriber()

    def read_telemetry():
        ticks = [reader.timeWritten() for reader in (hub, mass, joint, body)]
        return ticks, [hub().r_BN_N[1], mass().c_B[0], joint().rho, body().r_BN_N[0]]

    samples = []

    class Probe(sysModel.SysModel):
        def UpdateState(self, tick):
            samples.append((tick, read_telemetry()))
            if callback_fails:
                raise RuntimeError("deliberate accepted callback failure")

    probe = Probe()
    scene.AddModelToDiffusionDynamicsTask(probe)  # Euler only runs this task after acceptance.
    for model in (primary, secondary):
        model.Reset(0)
    end = macros.sec2nano(1.0)  # [ns]
    if callback_fails:
        with pytest.raises(RuntimeError, match="director method error"):
            primary.UpdateState(end)
    else:
        primary.UpdateState(end)

    assert [tick for tick, _ in samples] == [end]
    assert sc.timeBeforeNanos == scene.timeBeforeNanos == end
    for ticks, positions in (samples[0][1], read_telemetry()):
        assert ticks == [end] * 4
        # Equal masses put the slider half its displacement ahead of the system center of mass.
        assert positions == pytest.approx([2.0, 0.5, 1.0, 0.5])  # [m]


def test_synchronized_outputs_after_failed_advance():
    """Both scenes retain their accepted slider state across a secondary update and an RKF45 failure."""
    xml = SLIDER_SITE_XML.replace('type="slide"', 'type="slide" stiffness="1"')  # [N/m]
    primary, secondary = mujoco.MJScene(xml), mujoco.MJScene(xml)
    integrator = svIntegrators.svIntegratorRKF45(primary)
    integrator.relTol = integrator.absTol = 1e-11
    primary.setIntegrator(integrator)
    primary.syncDynamicsIntegration(secondary)
    outputs = []
    for scene in (primary, secondary):
        scene.Reset(0)
        body = scene.getBody("slider")
        joint = body.getScalarJoint("slide")
        joint.setVelocity(1.0)  # [m/s]
        outputs.append((scene.stateOutMsg.addSubscriber(), body.getOrigin().stateOutMsg.addSubscriber(),
                        body.massPropertiesOutMsg.addSubscriber(), joint.stateOutMsg.addSubscriber(),
                        joint.stateDotOutMsg.addSubscriber()))

    def check(tick):
        t = tick * macros.NANO2SEC  # [s]
        for scene, readers in zip((primary, secondary), outputs):
            state, site, mass, position, rate = (reader() for reader in readers)
            assert scene.timeBeforeNanos == tick
            assert scene.getEvaluationTime() == t
            assert all(reader.timeWritten() == tick for reader in readers)
            assert mass.massSC == 1.0  # [kg]
            np.testing.assert_allclose([state.qpos[0][0], site.r_BN_N[0], position.state], np.sin(t), atol=1e-9)
            np.testing.assert_allclose([state.qvel[0][0], site.v_BN_N[0], rate.state], np.cos(t), atol=1e-9)

    step = macros.sec2nano(1.0)  # [ns]
    primary.UpdateState(step)
    check(step)
    secondary.UpdateState(step + step // 2)
    check(step)

    factors = integrator.safetyFactorForNextStepSize, integrator.minimumFactorDecreaseForNextStepSize
    integrator.safetyFactorForNextStepSize = integrator.minimumFactorDecreaseForNextStepSize = 0.0
    with pytest.raises(RuntimeError, match="representable forward progress"):
        primary.UpdateState(2 * step)
    check(step)
    integrator.safetyFactorForNextStepSize, integrator.minimumFactorDecreaseForNextStepSize = factors
    primary.UpdateState(2 * step)
    check(2 * step)


@pytest.mark.parametrize("solver", ["svIntegratorRK4", "svStochasticIntegratorSOSRI"])
def test_failed_stage_restores_published_site(solver):
    """A fixed-step or stochastic stage failure restores the site and native clock."""
    scene = mujoco.MJScene(SLIDER_SITE_XML)
    scene.setIntegrator(getattr(svIntegrators, solver)(scene))
    site = scene.getSite("tip").stateOutMsg.addSubscriber()
    trial_positions = []

    class FailAtPositiveStage(sysModel.SysModel):
        def UpdateState(self, tick):
            if scene.getEvaluationTime() > 0.0:
                trial_positions.append(site().r_BN_N[0])
                raise RuntimeError("deliberate stage failure")

    probe = FailAtPositiveStage()
    scene.AddModelToDynamicsTask(probe)
    scene.Reset(0)
    scene.getBody("slider").getScalarJoint("slide").setVelocity(1.0)  # [m/s]
    scene.UpdateState(0)
    with pytest.raises(RuntimeError):
        scene.UpdateState(macros.sec2nano(1.0))  # [ns]
    assert len(trial_positions) == 1 and trial_positions[0] != 0.0
    assert scene.getEvaluationTime() == 0.0  # [s]
    assert site.timeWritten() == 0  # [ns]
    assert site().r_BN_N[0] == 0.0  # [m]
    assert site().v_BN_N[0] == 1.0  # [m/s]


if __name__ == "__main__":
    test_siteVelocity()
