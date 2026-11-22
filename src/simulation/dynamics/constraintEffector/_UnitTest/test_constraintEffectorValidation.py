# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import constraintDynamicEffector, gravityEffector, prescribedMotionStateEffector, spacecraft
from Basilisk.simulation import spinningBodyOneDOFStateEffector, svIntegrators
from Basilisk.utilities import SimulationBaseClass, macros


def _spinning_body():
    body = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
    body.mass = 1.0  # [kg]
    body.sHat_S = [1.0, 0.0, 0.0]  # [-]
    body.dcm_S0B = np.eye(3).tolist()
    body.IPntSc_S = np.eye(3).tolist()  # [kg*m^2]
    return body


@pytest.mark.parametrize("integrator_name", ["svIntegratorRKF45", "svStochasticIntegratorSOSRI"])
@pytest.mark.parametrize("late_input", ["secondary-ephemeris", "primary-ephemeris", "translation", "branch-reference"])
def test_synchronized_constraint_at_time_zero(integrator_name, late_input):
    """Check both parents' restoring accelerations and the direction of motion after a delayed input."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    step = macros.sec2nano(0.1)  # [ns]
    process.addTask(sim.CreateNewTask("task", step))
    primary, secondary = spacecraft.Spacecraft(), spacecraft.Spacecraft()
    primary.setIntegrator(getattr(svIntegrators, integrator_name)(primary))
    primary.syncDynamicsIntegration(secondary)
    mass = 10.0  # [kg]
    separation = 2.0  # [m]
    rest_length = 1.0  # [m]
    stiffness = 10.0  # [N/m]
    constraint = constraintDynamicEffector.ConstraintDynamicEffector()
    constraint.setR_P2P1_B1Init([rest_length, 0.0, 0.0])
    constraint.setK_d(stiffness)
    secondary.hub.r_CN_NInit = [separation, 0.0, 0.0]
    for sc in (primary, secondary):
        sc.hub.mHub = mass
        sc.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
        sim.AddModelToTask("task", sc)
    primary.addDynamicEffector(constraint)
    if late_input == "branch-reference":
        spin = _spinning_body()
        rate_damping = 4.0  # [N*m*s]
        constraint.setC_a(rate_damping)
        attitude = messaging.AttRefMsgPayload()
        attitude_source = messaging.AttRefMsg().write(attitude)
        secondary.attRefInMsg.subscribeTo(attitude_source)
        secondary.hub.mHub = mass - spin.mass
        spin.addDynamicEffector(constraint)
        secondary.addStateEffector(spin)
    else:
        secondary.addDynamicEffector(constraint)
    sim.AddModelToTask("task", constraint)

    if late_input.endswith("ephemeris"):
        body = gravityEffector.GravBodyData()
        body.planetName = "distantBody"
        body.mu = 1.0  # [m^3/s^2]
        planet = messaging.SpicePlanetStateMsgPayload()
        planet.PositionVector = [1.0e6, 0.0, 0.0]  # [m]
        source = messaging.SpicePlanetStateMsg()
        body.planetBodyInMsg.subscribeTo(source)
        host = primary if late_input == "primary-ephemeris" else secondary
        host.gravField.addGravBody(body)
    else:
        translation = messaging.TransRefMsgPayload()
        translation.r_RN_N = [separation, 0.0, 0.0]
        source = messaging.TransRefMsg().write(translation)
        secondary.transRefInMsg.subscribeTo(source)

    sim.InitializeSimulation()
    primary_gravity = secondary_gravity = 0.0  # [m/s^2]
    if late_input.endswith("ephemeris"):
        if host is primary:
            # The secondary evaluated its reset derivatives while the primary was still waiting for gravity.
            velocity = secondary.dynManager.getStateObject(secondary.hub.nameOfHubVelocity)
            expected = -stiffness * (separation - rest_length) / mass  # [m/s^2]
            np.testing.assert_allclose(np.ravel(velocity.getStateDeriv()), [expected, 0.0, 0.0], rtol=1e-13)
            primary_gravity = body.mu / planet.PositionVector[0]**2
        else:
            secondary_gravity = body.mu / (planet.PositionVector[0] - separation)**2
        source.write(planet, 0)
    else:
        separation = 3.0  # [m]
        translation.r_RN_N = [separation, 0.0, 0.0]
        source.write(translation, 0)
        if late_input == "branch-reference":
            attitude.omega_RN_N = [0.0, 0.0, 2.0]  # [rad/s]
            attitude_source.write(attitude, 0)
    sim.ConfigureStopTime(0)
    sim.ExecuteSimulation()

    acceleration = stiffness * (separation - rest_length) / mass  # [m/s^2]
    assert secondary.scStateOutMsg.read().r_BN_N[0] == separation
    for sc, expected in ((primary, primary_gravity + acceleration), (secondary, secondary_gravity - acceleration)):
        velocity = sc.dynManager.getStateObject(sc.hub.nameOfHubVelocity)
        np.testing.assert_allclose(np.ravel(velocity.getStateDeriv()), [expected, 0.0, 0.0], rtol=1e-13)
    if late_input == "branch-reference":
        # At zero attitude error the damping torque is c_a * omega / 4. The spin adds inertia about z.
        torque = rate_damping * attitude.omega_RN_N[2] / 4.0  # [N*m]
        for sc, inertia, sign in ((primary, 1.0, 1), (secondary, 2.0, -1)):  # [kg*m^2]
            omega = sc.dynManager.getStateObject(sc.hub.nameOfHubOmega)
            np.testing.assert_allclose(np.ravel(omega.getStateDeriv()), [0.0, 0.0, sign * torque / inertia])
    if late_input.endswith("ephemeris"):
        sim.ConfigureStopTime(step)
        sim.ExecuteSimulation()
        assert primary.scStateOutMsg.read().v_BN_N[0] > 0.0
        assert secondary.scStateOutMsg.read().v_BN_N[0] < 0.0


def _run_constraint_with_central_origin(integrator_name, attachments, translated, pending=(), interruption=None):
    """Return both hub accelerations at zero and their central-body-relative velocities after one step."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    step = macros.sec2nano(0.1)  # [ns]
    process.addTask(sim.CreateNewTask("task", step))
    vehicles = [spacecraft.Spacecraft(), spacecraft.Spacecraft()]
    primary, secondary = vehicles
    integrator = getattr(svIntegrators, integrator_name)(primary)
    primary.setIntegrator(integrator)
    primary.syncDynamicsIntegration(secondary)
    constraint = constraintDynamicEffector.ConstraintDynamicEffector()
    constraint.setR_P2P1_B1Init([1.0, 0.0, 0.0])  # [m]
    constraint.setK_d(10.0)  # [N/m]
    constraint.setC_d(3.0)  # [N*s/m]
    sample = messaging.SpicePlanetStateMsgPayload()
    if translated:
        sample.PositionVector = [1000.0, 0.0, 0.0]  # [m]
        sample.VelocityVector = [0.0, 25.0, 0.0]  # [m/s]
    sources = [messaging.SpicePlanetStateMsg() for _ in vehicles]
    branches = []  # Keep the Python owners alive while the spacecraft uses the attached effectors.
    for i, (sc, attachment, source) in enumerate(zip(vehicles, attachments, sources)):
        sc.hub.r_CN_NInit = [100.0 + 2.0 * i, 0.0, 0.0]  # [m]
        sc.hub.mHub = 10.0  # [kg]
        sc.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
        if attachment == "branch":
            spin = _spinning_body()
            sc.hub.mHub -= spin.mass
            spin.addDynamicEffector(constraint)
            sc.addStateEffector(spin)
            branches.append(spin)
        else:
            sc.addDynamicEffector(constraint)
        center = gravityEffector.GravBodyData()
        center.planetName = "center"
        center.isCentralBody = True
        center.mu = 1.0  # [m^3/s^2]
        center.planetBodyInMsg.subscribeTo(source)
        if i not in pending:
            source.write(sample, 0)
        sc.gravField.addGravBody(center)
        sim.AddModelToTask("task", sc)
    sim.AddModelToTask("task", constraint)
    telemetry = constraint.constraintElements.addSubscriber()
    if interruption == "rejected-trial":
        hub_state = secondary.scStateOutMsg.addSubscriber()
        branch_state = branches[-1].spinningBodyConfigLogOutMsg.addSubscriber()
    sim.InitializeSimulation()
    sim.ConfigureStopTime(0)
    sim.ExecuteSimulation()
    if pending:
        constraint.writeOutputStateMessage(0)
    assert telemetry.isWritten() == (not pending)
    accelerations = [np.ravel(sc.dynManager.getStateObject(sc.hub.nameOfHubVelocity).getStateDeriv())
                     for sc in vehicles]
    if interruption == "unwritten-origin":
        replacement = messaging.SpicePlanetStateMsg()
        center.planetBodyInMsg.subscribeTo(replacement)
        with pytest.raises(BasiliskError, match="ephemeris must be written"):
            primary.UpdateState(step)
        constraint.UpdateState(step)
        constraint.writeOutputStateMessage(step)
        assert telemetry.timeWritten() == 0
        replacement.write(sample, 0)
    elif interruption == "rejected-trial":
        accepted_error = list(telemetry().psi_N)
        accepted_positions = [list(reader().r_BN_N) for reader in (hub_state, branch_state)]
        settings = (integrator.relTol, integrator.absTol, integrator.safetyFactorForNextStepSize,
                    integrator.minimumFactorDecreaseForNextStepSize)
        # Force a rejected candidate and prevent a smaller retry.
        integrator.relTol = integrator.absTol = 1e-15
        integrator.safetyFactorForNextStepSize = integrator.minimumFactorDecreaseForNextStepSize = 0.0
        with pytest.raises(RuntimeError, match="representable forward progress"):
            primary.UpdateState(step)
        constraint.UpdateState(step)
        assert telemetry.timeWritten() == 0
        np.testing.assert_array_equal(telemetry().psi_N, accepted_error)
        secondary.UpdateState(step)
        branches[-1].UpdateState(step)
        for reader, position in zip((hub_state, branch_state), accepted_positions):
            assert reader.timeWritten() == 0
            np.testing.assert_array_equal(reader().r_BN_N, position)
        (integrator.relTol, integrator.absTol, integrator.safetyFactorForNextStepSize,
         integrator.minimumFactorDecreaseForNextStepSize) = settings
    for i in pending:
        sources[i].write(sample, 0)
    sim.ConfigureStopTime(step)
    sim.ExecuteSimulation()
    assert telemetry.isWritten() and telemetry.timeWritten() == step
    if interruption == "rejected-trial":
        assert hub_state.timeWritten() == branch_state.timeWritten() == step
        np.testing.assert_allclose(branch_state().r_BN_N, hub_state().r_BN_N)
    velocities = [np.array(sc.scStateOutMsg.read().v_BN_N) - sample.VelocityVector for sc in vehicles]
    positions = [np.array(sc.scStateOutMsg.read().r_BN_N) for sc in vehicles]
    # These endpoints coincide with their hub origins and have zero angular velocity.
    error = positions[1] - positions[0] - np.ravel(constraint.getR_P2P1_B1Init())
    expected_force = constraint.getK_d() * error + constraint.getC_d() * (velocities[1] - velocities[0])
    np.testing.assert_allclose(telemetry().psi_N, error, rtol=0.0, atol=1e-12)  # [m]
    np.testing.assert_allclose(telemetry().Fc_N, expected_force, rtol=0.0, atol=1e-11)  # [N] on endpoint 1
    return np.array(accelerations), np.array(velocities)


@pytest.mark.parametrize("integrator_name", ["svIntegratorRKF45", "svStochasticIntegratorSOSRI"])
@pytest.mark.parametrize("attachments", [("hub", "branch"), ("branch", "hub")])
def test_constraint_invariant_to_central_origin(integrator_name, attachments):
    """Translating and boosting a common origin must preserve the spring and damping forces at every stage."""
    baseline = _run_constraint_with_central_origin(integrator_name, attachments, False)
    shifted = _run_constraint_with_central_origin(integrator_name, attachments, True)
    expected_x = [1.0 - 1.0 / 100.0**2, -1.0 - 1.0 / 102.0**2]  # [m/s^2], spring plus central gravity
    np.testing.assert_allclose(shifted[0][:, 0], expected_x, rtol=1e-12)
    np.testing.assert_allclose(shifted[0], baseline[0], rtol=0.0, atol=1e-11)  # [m/s^2]
    np.testing.assert_allclose(shifted[1], baseline[1], rtol=0.0, atol=1e-10)  # [m/s]


@pytest.mark.parametrize("integrator_name", ["svIntegratorRKF45", "svStochasticIntegratorSOSRI"])
@pytest.mark.parametrize("pending", [(1,), (0,), (0, 1)])
def test_constraint_recovers_on_first_positive_step(integrator_name, pending):
    """Hold the load while an origin is unknown, then recover when advancing directly from time zero."""
    attachments = ("branch", "branch")
    _, control = _run_constraint_with_central_origin(integrator_name, attachments, True)
    acceleration, recovered = _run_constraint_with_central_origin(integrator_name, attachments, True, pending)
    for i, radius in enumerate((100.0, 102.0)):  # [m]
        if i not in pending:
            np.testing.assert_allclose(acceleration[i], [-1.0 / radius**2, 0.0, 0.0], atol=1e-13)  # [m/s^2]
    np.testing.assert_allclose(recovered, control, rtol=0.0, atol=1e-10)  # [m/s]


@pytest.mark.parametrize("integrator_name, epoch", [
    ("svIntegratorRK4", 0),
    ("svIntegratorRK4", 120_000_000_123_456_789),
    ("svIntegratorEuler", 120_000_000_123_456_789),
    ("svIntegratorRKF45", 120_000_000_123_456_789),
    ("svStochasticIntegratorSOSRI", 120_000_000_123_456_789),
])  # [ns] reset epoch
@pytest.mark.parametrize("origin, boost, second_offset, second_boost", [
    (0.0, 0.0, 0.0, 0.0), (149_597_870_700.0, 30_000.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 0.5),
])  # [m], [m/s], [m], [m/s]
def test_hub_constraint_at_planetary_distance(origin, boost, second_offset, second_boost, integrator_name, epoch):
    """Both hub derivatives and accepted telemetry must use the same origins at reset."""
    sample = messaging.SpicePlanetStateMsgPayload()
    sample.PositionVector = [origin, 0.0, 0.0]
    sample.VelocityVector = [0.0, boost, 0.0]
    source = messaging.SpicePlanetStateMsg().write(sample, epoch)
    center = gravityEffector.GravBodyData()
    center.planetName = "earth"
    center.isCentralBody = True
    center.mu = 3.986e14  # [m^3/s^2]
    center.planetBodyInMsg.subscribeTo(source)
    constraint = constraintDynamicEffector.ConstraintDynamicEffector()
    constraint.setR_P2B2_B2([0.1, 0.0, 0.0])  # [m]
    hub_gap = 2.1  # [m]
    constraint.setR_P2P1_B1Init([hub_gap + 0.1, 0.0, 0.0])  # [m]
    constraint.setK_d(1e5)  # [N/m]
    constraint.setC_d(1e5)  # [N*s/m]
    primary, secondary = spacecraft.Spacecraft(), spacecraft.Spacecraft()
    primary.setIntegrator(getattr(svIntegrators, integrator_name)(primary))
    primary.syncDynamicsIntegration(secondary)
    spin_rate = 1.0  # [rad/s]
    for sc, offset in ((primary, 0.0), (secondary, hub_gap)):  # [m]
        sc.hub.mHub = 100.0  # [kg]
        sc.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
        sc.hub.r_CN_NInit = [offset, 7e6, 0.0]  # [m]
        sc.hub.omega_BN_BInit = [0.0, 0.0, spin_rate]
        sc.hub.v_CN_NInit = [0.0, offset * spin_rate, 0.0]  # [m/s]
        if sc is secondary and (second_offset or second_boost):
            shifted = gravityEffector.GravBodyData()
            shifted.planetName = center.planetName
            shifted.isCentralBody = True
            shifted.mu = center.mu
            sample.PositionVector = [origin + second_offset, 0.0, 0.0]
            sample.VelocityVector = [0.0, boost + second_boost, 0.0]
            shifted_source = messaging.SpicePlanetStateMsg().write(sample, epoch)
            shifted.planetBodyInMsg.subscribeTo(shifted_source)
            sc.gravField.addGravBody(shifted)
        else:
            sc.gravField.addGravBody(center)
        sc.addDynamicEffector(constraint)
    telemetry = constraint.constraintElements.addSubscriber()
    primary.Reset(epoch)
    secondary.Reset(epoch)
    constraint.Reset(epoch)
    expected_error = np.array([second_offset, 0.0, 0.0])  # [m]
    expected_velocity_error = np.array([0.0, second_boost - spin_rate * second_offset, 0.0])  # [m/s]
    expected_force = constraint.getK_d() * expected_error + constraint.getC_d() * expected_velocity_error  # [N]

    def check_acceleration(sc, force):
        position = np.ravel(sc.hub.r_CN_NInit)
        gravity = -center.mu * position / np.linalg.norm(position)**3  # [m/s^2]
        velocity = sc.dynManager.getStateObject(sc.hub.nameOfHubVelocity)
        np.testing.assert_allclose(np.ravel(velocity.getStateDeriv()), gravity + force / sc.hub.mHub,
                                   rtol=0.0, atol=1e-10)  # [m/s^2]

    # The second Reset sees both endpoints. The zero-duration update refreshes both hub derivatives.
    check_acceleration(secondary, -expected_force)
    primary.UpdateState(epoch)
    constraint.UpdateState(epoch)
    assert telemetry.isWritten() and telemetry.timeWritten() == epoch
    np.testing.assert_allclose(telemetry().psi_N, expected_error, rtol=0.0, atol=1e-14)  # [m]
    np.testing.assert_allclose(telemetry().Fc_N, expected_force, rtol=0.0, atol=1e-10)  # [N]
    check_acceleration(primary, expected_force)
    check_acceleration(secondary, -expected_force)
    for sc, x in ((primary, origin), (secondary, origin + second_offset)):
        planet_position = sc.dynManager.getPropertyReference("earth.r_PN_N")
        np.testing.assert_array_equal(np.ravel(planet_position), [x, 0.0, 0.0])  # [m] at the ephemeris write tick


@pytest.mark.parametrize("integrator_name", ["svIntegratorRKF45", "svStochasticIntegratorSOSRI"])
def test_rejected_constraint_step_invalidates_replaced_origin(integrator_name):
    """Retain the last message during an origin outage and recover on the next successful advance."""
    _run_constraint_with_central_origin(integrator_name, ("hub", "branch"), True, interruption="unwritten-origin")


def test_constraint_retains_accepted_message_after_rejected_trial():
    """A rejected RKF45 candidate must not overwrite the last accepted constraint measurement."""
    _run_constraint_with_central_origin("svIntegratorRKF45", ("branch", "branch"), True, interruption="rejected-trial")


@pytest.mark.parametrize("integrator_name", ["svIntegratorRKF45", "svStochasticIntegratorSOSRI"])
@pytest.mark.parametrize("hub_first", [True, False])
def test_constraint_on_same_hub_and_rotated_branch(integrator_name, hub_first):
    """Coincident points exert no force, and the bending torque restores the joint in either order."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    step = macros.sec2nano(0.1)  # [ns]
    process.addTask(sim.CreateNewTask("task", step))
    sc = spacecraft.Spacecraft()
    sc.setIntegrator(getattr(svIntegrators, integrator_name)(sc))
    sc.hub.mHub = 10.0  # [kg]
    sc.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
    spin = _spinning_body()
    spin.r_SB_B = [0.0, 2.0, 0.0]  # [m]
    spin.thetaInit = np.pi / 2  # [rad]
    sc.addStateEffector(spin)
    constraint = constraintDynamicEffector.ConstraintDynamicEffector()
    constraint.setK_d(10.0)  # [N/m]
    constraint.setK_a(1.0)  # [N*m]
    constraint.setFilter_Data(1.0, step * macros.NANO2SEC, 0.7)  # [rad/s], [s], [1/s]
    if hub_first:
        constraint.setR_P1B1_B1(spin.r_SB_B)
        sc.addDynamicEffector(constraint)
        spin.addDynamicEffector(constraint)
    else:
        constraint.setR_P2B2_B2(spin.r_SB_B)
        spin.addDynamicEffector(constraint)
        sc.addDynamicEffector(constraint)
    reader = constraint.constraintElements.addSubscriber()
    joint = spin.spinningBodyOutMsg.addSubscriber()
    sim.AddModelToTask("task", sc)
    sim.AddModelToTask("task", constraint)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(0)
    sim.ExecuteSimulation()
    np.testing.assert_allclose(reader().psi_N, 0.0, atol=1e-14)  # [m]
    np.testing.assert_allclose(reader().Fc_N, 0.0, atol=1e-13)  # [N]
    sim.ConfigureStopTime(step)
    sim.ExecuteSimulation()
    assert 0.0 < joint().theta < spin.thetaInit
    assert joint().thetaDot < 0.0
    torque = constraint.getK_a() * np.tan(joint().theta / 4)  # [N*m]
    first_torque = np.array([torque if hub_first else -torque, 0.0, 0.0])  # [N*m]
    np.testing.assert_allclose(reader().L1_B1, first_torque, rtol=1e-12, atol=1e-13)
    np.testing.assert_allclose(reader().L2_B2, -first_torque, rtol=1e-12, atol=1e-13)
    filtered_torque = reader().L1_mag_filtered  # [N*m]
    assert filtered_torque > 0.0
    status = messaging.DeviceStatusMsgPayload()
    status.deviceStatus = 0
    command = messaging.DeviceStatusMsg().write(status, step + step // 2)
    constraint.effectorStatusInMsg.subscribeTo(command)
    fields = ("Fc_N", "L1_B1", "L2_B2", "psi_N", "Fc_mag_filtered", "L1_mag_filtered", "L2_mag_filtered")
    sample = {field: np.array(getattr(reader(), field)) for field in fields}
    constraint.UpdateState(step + step // 2)
    constraint.writeOutputStateMessage(step + step // 2)
    assert reader.timeWritten() == step
    for field, expected in sample.items():
        np.testing.assert_array_equal(getattr(reader(), field), expected)
    sim.ConfigureStopTime(2 * step)
    sim.ExecuteSimulation()
    assert reader.timeWritten() == 2 * step
    np.testing.assert_array_equal(reader().L1_B1, np.zeros(3))
    np.testing.assert_array_equal(reader().L2_B2, np.zeros(3))

    # The first publication for a new epoch can also occur after the next command arrives.
    sc.UpdateState(3 * step)
    status.deviceStatus = 1
    command.write(status, 3 * step + step // 2)
    constraint.UpdateState(3 * step + step // 2)
    assert reader.timeWritten() == 3 * step
    np.testing.assert_array_equal(reader().L1_B1, np.zeros(3))
    sc.UpdateState(4 * step)
    constraint.UpdateState(4 * step)
    assert np.linalg.norm(reader().L1_B1) > 0.0


def test_same_tick_replaces_filtered_constraint_loads():
    """A revised force and both torques occupy one sample of the filter history."""
    primary, secondary = spacecraft.Spacecraft(), spacecraft.Spacecraft()
    primary.syncDynamicsIntegration(secondary)
    constraint = constraintDynamicEffector.ConstraintDynamicEffector()
    constraint.setK_d(1.0)  # [N/m]
    constraint.setR_P1B1_B1([0.0, 2.0, 0.0])  # [m]
    constraint.setR_P2B2_B2([0.0, 3.0, 0.0])  # [m]
    constraint.setR_P2P1_B1Init([0.0, 1.0, 0.0])  # [m]
    constraint.setFilter_Data(1.0, 1.0, 0.5)  # [rad/s], [s], [1/s]
    ref = messaging.TransRefMsgPayload()
    source = messaging.TransRefMsg().write(ref)
    secondary.transRefInMsg.subscribeTo(source)
    for sc in (primary, secondary):
        sc.hub.mHub = 10.0  # [kg]
        sc.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
        sc.addDynamicEffector(constraint)
    reader = constraint.constraintElements.addSubscriber()
    primary.Reset(0)
    secondary.Reset(0)
    constraint.Reset(0)

    def filtered(sample):
        return np.array([sample.Fc_mag_filtered, sample.L1_mag_filtered, sample.L2_mag_filtered])

    for gap, publish in [(1.0, constraint.UpdateState), (2.0, constraint.UpdateState),
                         (3.0, constraint.writeOutputStateMessage)]:  # [m]
        ref.r_RN_N = [gap, 0.0, 0.0]
        source.write(ref, 0)
        primary.UpdateState(0)
        publish(0)
        initial = gap * np.array([1.0, 2.0, 3.0])  # [N], [N*m], [N*m] from the lever arms
        np.testing.assert_allclose(filtered(reader()), initial / 7)

    # The configured filter has numerator [1, 2, 1] and denominator [7, -6, 3].
    inputs = [np.zeros(3), initial]
    outputs = [np.zeros(3), initial / 7]
    for seconds in (1, 2):
        tick = macros.sec2nano(seconds)  # [ns]
        primary.UpdateState(tick)
        constraint.UpdateState(tick)
        if seconds == 1:
            ref.r_RN_N = [4.0, 0.0, 0.0]  # [m] replace a sample with nonzero filter history
            source.write(ref, tick)
            primary.UpdateState(tick)
            constraint.UpdateState(tick)
        sample = reader()
        magnitudes = np.linalg.norm([sample.Fc_N, sample.L1_B1, sample.L2_B2], axis=1)
        expected = (magnitudes + 2 * inputs[-1] + inputs[-2] + 6 * outputs[-1] - 3 * outputs[-2]) / 7
        np.testing.assert_allclose(filtered(sample), expected)
        inputs.append(magnitudes)
        outputs.append(expected)


def test_articulated_output_uses_parent_epoch_between_steps():
    """A faster task publishes its articulated inertial pose at the synchronized parent's epoch."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    step = macros.sec2nano(1.0)  # [ns]
    process.addTask(sim.CreateNewTask("slow", 2 * step), 10)
    process.addTask(sim.CreateNewTask("fast", step))
    primary, secondary = spacecraft.Spacecraft(), spacecraft.Spacecraft()
    primary.syncDynamicsIntegration(secondary)
    for sc in (primary, secondary):
        sc.hub.mHub = 10.0  # [kg]
        sc.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
    secondary.hub.v_CN_NInit = [1.0, 0.0, 0.0]  # [m/s]
    spin = _spinning_body()
    secondary.addStateEffector(spin)
    state = spin.spinningBodyConfigLogOutMsg.addSubscriber()
    sim.AddModelToTask("slow", primary)
    sim.AddModelToTask("fast", secondary, 2)
    sim.AddModelToTask("fast", spin, 1)
    sim.InitializeSimulation()
    for tick, epoch, position in [(0, 0, 0.0), (step, 0, 0.0), (2 * step, 2 * step, 2.0)]:  # [ns], [ns], [m]
        sim.ConfigureStopTime(tick)
        sim.ExecuteSimulation()
        assert state.timeWritten() == epoch
        np.testing.assert_allclose(state().r_BN_N, [position, 0.0, 0.0], atol=1e-14)


@pytest.mark.parametrize("nested", [False, True], ids=["hub", "prescribed-parent"])
def test_articulated_inertial_output_waits_for_central_ephemeris(nested):
    """Publish joint state while waiting for the origin, then publish the body in inertial coordinates."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(0.1)))  # [ns]
    sc = spacecraft.Spacecraft()
    sc.hub.mHub = 9.0  # [kg]
    sc.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
    sc.hub.r_CN_NInit = [100.0, 0.0, 0.0]  # [m]
    spin = _spinning_body()
    spin.r_SB_B = [2.0, 0.0, 0.0]  # [m]
    if nested:
        platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()
        platform.addStateEffector(spin)
        sc.addStateEffector(platform)
        platform_state = platform.prescribedMotionConfigLogOutMsg.addSubscriber()
    else:
        sc.addStateEffector(spin)
    center = gravityEffector.GravBodyData()
    center.planetName = "center"
    center.isCentralBody = True
    center.mu = 1.0  # [m^3/s^2]
    source = messaging.SpicePlanetStateMsg()
    center.planetBodyInMsg.subscribeTo(source)
    sc.gravField.addGravBody(center)
    joint = spin.spinningBodyOutMsg.addSubscriber()
    inertial = spin.spinningBodyConfigLogOutMsg.addSubscriber()
    sim.AddModelToTask("task", sc)
    if nested:
        sim.AddModelToTask("task", platform)
    sim.AddModelToTask("task", spin)
    sim.InitializeSimulation()
    assert not inertial.isWritten()
    if nested:
        assert not platform_state.isWritten()
    else:
        assert joint.isWritten()
    sim.ConfigureStopTime(0)
    sim.ExecuteSimulation()
    assert joint.isWritten() and not inertial.isWritten()
    if nested:
        assert not platform_state.isWritten()

    sample = messaging.SpicePlanetStateMsgPayload()
    sample.PositionVector = [1.0e6, 0.0, 0.0]  # [m]
    source.write(sample, 0)
    if nested:
        # An upstream model also moves the prescribed attachment before the spacecraft's time-zero refresh.
        motion = messaging.PrescribedTranslationMsgPayload()
        motion.r_PM_M = [3.0, 0.0, 0.0]  # [m]
        motion_source = messaging.PrescribedTranslationMsg().write(motion)
        platform.prescribedTranslationInMsg.subscribeTo(motion_source)
        platform.UpdateState(0)
        assert not platform_state.isWritten()
    sc.UpdateState(0)
    if nested:
        assert platform_state.isWritten()
        spin.UpdateState(0)
    assert inertial.isWritten()
    # The hub origin is displaced from the system center of mass by the spinning body's mass fraction.
    offset = sc.hub.mHub / (sc.hub.mHub + spin.mass) * np.ravel(spin.r_SB_B)
    expected = np.array(sample.PositionVector) + np.ravel(sc.hub.r_CN_NInit) + offset
    if nested:
        expected += motion.r_PM_M
    np.testing.assert_allclose(spin.spinningBodyConfigLogOutMsg.read().r_BN_N, expected, rtol=0.0, atol=1e-9)


def _simulationWithConstraint(attachToBranch):
    """Create a spacecraft-only simulation with a constraint attached to a hub or branch."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    timeStep = macros.sec2nano(0.1)  # [ns]
    process.addTask(simulation.CreateNewTask("task", timeStep))

    spacecraftObject = spacecraft.Spacecraft()
    spacecraftObject.hub.mHub = 100.0  # [kg]
    spacecraftObject.hub.IHubPntBc_B = [[10.0, 0.0, 0.0],
                                        [0.0, 20.0, 0.0],
                                        [0.0, 0.0, 30.0]]  # [kg m^2]
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    spinningBody = None
    if attachToBranch:
        spinningBody = _spinning_body()
        spinningBody.addDynamicEffector(constraintEffector)
        spacecraftObject.addStateEffector(spinningBody)
    else:
        spacecraftObject.addDynamicEffector(constraintEffector)
    simulation.AddModelToTask("task", spacecraftObject)
    return simulation, spacecraftObject, constraintEffector, spinningBody


@pytest.mark.parametrize("attachToBranch", [False, True], ids=["hub", "branch"])
@pytest.mark.parametrize("validationPath", ["attachment", "reset"])
@pytest.mark.parametrize("missingParameter", ["Alpha", "Beta"])
def test_constraintEffector_validation(attachToBranch, validationPath, missingParameter):
    """Hub, branch, and direct Reset paths must reject missing tuning parameters."""
    simulation, spacecraftObject, constraintEffector, spinningBody = _simulationWithConstraint(
        attachToBranch)
    if missingParameter == "Beta":
        constraintEffector.setAlpha(1.0)  # [-]

    with pytest.raises(BasiliskError, match=missingParameter):
        if validationPath == "reset":
            constraintEffector.Reset(0)
        else:
            simulation.InitializeSimulation()


@pytest.mark.parametrize("attachToBranch", [False, True], ids=["hub", "branch"])
@pytest.mark.parametrize("initializationPath", ["attachment", "reset"])
def test_constraintEffector_gainInitialization(attachToBranch, initializationPath):
    """Hub, branch, and direct Reset paths must derive unspecified individual gains."""
    simulation, spacecraftObject, constraintEffector, spinningBody = _simulationWithConstraint(
        attachToBranch)
    constraintEffector.setAlpha(3.0)  # [-]
    constraintEffector.setBeta(2.0)  # [-]

    if initializationPath == "reset":
        constraintEffector.Reset(0)
    else:
        simulation.InitializeSimulation()

    assert constraintEffector.getK_d() == pytest.approx(9.0)  # [N/m]
    assert constraintEffector.getC_d() == pytest.approx(4.0)  # [N*s/m]
    assert constraintEffector.getK_a() == pytest.approx(9.0)  # [N*m]
    assert constraintEffector.getC_a() == pytest.approx(4.0)  # [N*m*s]


@pytest.mark.parametrize("path", ["reset", "hub", "branch"])
@pytest.mark.parametrize("explicit_mask", range(16))
@pytest.mark.parametrize("override_time", ["before", "after"])
def test_retuning_refreshes_only_derived_gains(path, explicit_mask, override_time):
    """Explicit gains remain fixed even when their values match the original derived gains."""
    sim, parent, effector, body = _simulationWithConstraint(path == "branch")
    effector.setAlpha(3.0)  # [-]
    effector.setBeta(2.0)  # [-]
    fields = ["K_d", "C_d", "K_a", "C_a"]
    initial = [9.0, 4.0, 9.0, 4.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    retuned = [25.0, 14.0, 25.0, 14.0]  # [N/m], [N*s/m], [N*m], [N*m*s]

    def set_overrides():
        """Set each selected gain to the value it would otherwise derive."""
        for index, field in enumerate(fields):
            if explicit_mask & (1 << index):
                getattr(effector, "set" + field)(initial[index])

    if override_time == "before":
        set_overrides()
    if path == "reset":
        effector.Reset(0)
    else:
        sim.InitializeSimulation()
    if override_time == "after":
        set_overrides()
    effector.setAlpha(5.0)  # [-]
    effector.setBeta(7.0)  # [-]
    for _ in range(3):
        effector.Reset(macros.sec2nano(1.0))  # [s]
        for index, field in enumerate(fields):
            expected = initial[index] if explicit_mask & (1 << index) else retuned[index]
            assert getattr(effector, "get" + field)() == expected


@pytest.mark.parametrize("field", ["Alpha", "Beta", "K_d", "C_d", "K_a", "C_a"])
@pytest.mark.parametrize("bad", [
    np.nan, np.inf, -np.inf, 0.0, -1.0,  # K_d: [N/m], C_d: [N*s/m], K_a: [N*m], C_a: [N*m*s], Alpha/Beta: [-]
])
def test_invalid_gain_setters_preserve_derived_status(field, bad):
    """Rejected setters must not change a value or turn a derived gain into an explicit one."""
    effector = constraintDynamicEffector.ConstraintDynamicEffector()
    effector.setAlpha(3.0)  # [-]
    effector.setBeta(2.0)  # [-]
    effector.Reset(0)
    prior = getattr(effector, "get" + field)()
    with pytest.raises(BasiliskError):
        getattr(effector, "set" + field)(bad)
    assert getattr(effector, "get" + field)() == prior
    effector.setAlpha(5.0)  # [-]
    effector.setBeta(7.0)  # [-]
    effector.Reset(0)
    expected = [25.0, 14.0, 25.0, 14.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected


@pytest.mark.parametrize("path", ["reset", "hub", "branch"])
@pytest.mark.parametrize("field", ["Alpha", "Beta"])
def test_derived_gain_overflow_preserves_prior_gains(path, field):
    """Overflow while retuning must fail before any active gain is replaced."""
    sim, parent, effector, body = _simulationWithConstraint(path == "branch")
    effector.setAlpha(3.0)  # [-]
    effector.setBeta(2.0)  # [-]
    effector.Reset(0)
    getattr(effector, "set" + field)(np.finfo(float).max)  # [-] Numerical alpha/beta tuning parameter
    with pytest.raises(BasiliskError, match="finite derived gains"):
        if path == "reset":
            effector.Reset(0)
        else:
            sim.InitializeSimulation()
    expected = [9.0, 4.0, 9.0, 4.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected


def test_partial_explicit_gains_preserve_zero_unspecified_gains():
    """An existing partial explicit setup may intentionally leave other gains at zero."""
    effector = constraintDynamicEffector.ConstraintDynamicEffector()
    effector.setK_d(3.0)  # [N/m]
    effector.Reset(0)
    expected = [3.0, 0.0, 0.0, 0.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected
    effector.setAlpha(5.0)  # [-]
    effector.setBeta(7.0)  # [-]
    effector.Reset(0)
    expected = [3.0, 14.0, 25.0, 14.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected


def test_fully_explicit_gains_do_not_derive_unused_tuning_parameters():
    """Unused tuning parameters need not be squared or doubled when every gain is explicit."""
    effector = constraintDynamicEffector.ConstraintDynamicEffector()
    effector.setAlpha(np.finfo(float).max)  # [-]
    effector.setBeta(np.finfo(float).max)  # [-]
    effector.setK_d(3.0)  # [N/m]
    effector.setC_d(4.0)  # [N*s/m]
    effector.setK_a(5.0)  # [N*m]
    effector.setC_a(6.0)  # [N*m*s]
    effector.Reset(0)
    expected = [3.0, 4.0, 5.0, 6.0]  # [N/m], [N*s/m], [N*m], [N*m*s]
    assert [effector.getK_d(), effector.getC_d(), effector.getK_a(), effector.getC_a()] == expected
