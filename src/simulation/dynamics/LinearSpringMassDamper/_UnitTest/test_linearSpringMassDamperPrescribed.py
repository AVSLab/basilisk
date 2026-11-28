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
from Basilisk.simulation import linearSpringMassDamper, prescribedMotionStateEffector, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, pythonVariableLogger
from Basilisk.utilities import RigidBodyKinematics as rbk


def create_simulation():
    """Create a simulation with a 1 ms dynamics task."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(0.001)))  # [s]
    return sim


@pytest.mark.parametrize("damping", [0.0, 4.0])  # [N s/m]
def test_prescribed_spring_analytical_motion(damping):
    r"""Compare the particle and hub motion with the two-mass oscillator solution.

    **Validation Test Description**

    A stationary prescribed body is centered on the hub. The spring passes through their
    common center of mass, so the hub and prescribed body recoil without rotating. The
    independent oscillator solution uses the reduced mass of the particle and its parent
    assembly. A nonzero mount attitude checks the prescribed-frame interpretation of the axis.

    **Test Parameter Discussion**

    ``damping`` selects an undamped and an underdamped oscillator. There is no gravity or
    external force. Both default and renamed hub states are covered by the integration test
    in :ref:`test_PrescribedMotionStateEffectorBranching`.

    **Description of Variables Being Tested**

    Compare every recorded displacement and velocity sample to the analytical solution, and
    check the hub recoil from conservation of the system center of mass.
    """
    sim = create_simulation()
    hub = spacecraft.Spacecraft()
    hub.hub.mHub = 800.0  # [kg]
    hub.hub.IHubPntBc_B = np.diag([150.0, 180.0, 210.0]).tolist()  # [kg m^2]
    parent = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    parent.setMass(100.0)  # [kg]
    parent.setIPntPc_P(np.diag([20.0, 25.0, 30.0]).tolist())  # [kg m^2]
    sigma_MB = np.array([0.1, -0.2, 0.15])  # [-] MRPs
    parent.setSigma_MB(sigma_MB.tolist())
    particle = linearSpringMassDamper.LinearSpringMassDamper()
    particle.massInit = 20.0  # [kg]
    particle.k = 100.0  # [N/m]
    particle.c = damping
    particle.rhoInit = 0.5  # [m]
    particle.rhoDotInit = 0.15  # [m/s]
    axis_P = np.array([0.0, 1.0, 0.0])  # [-]
    particle.pHat_B = axis_P.tolist()
    parent.addStateEffector(particle)
    hub.addStateEffector(parent)
    sim.AddModelToTask("task", hub)
    sim.AddModelToTask("task", parent)
    particle_log = pythonVariableLogger.PythonVariableLogger({
        "rho": lambda _: rho_state.getState(),
        "rhoDot": lambda _: rho_dot_state.getState(),
    })
    hub_log = hub.scStateOutMsg.recorder()
    sim.AddModelToTask("task", particle_log)
    sim.AddModelToTask("task", hub_log)
    sim.InitializeSimulation()
    rho_state = hub.dynManager.getStateObject(particle.nameOfRhoState)
    rho_dot_state = hub.dynManager.getStateObject(particle.nameOfRhoDotState)
    assert rho_state is not None and rho_dot_state is not None
    duration = 4.0  # [s]
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()

    parent_mass = hub.hub.mHub + parent.getMass()
    reduced_mass = 1 / (1 / particle.massInit + 1 / parent_mass)
    decay_rate = damping / (2 * reduced_mass)
    damped_frequency = np.sqrt(particle.k / reduced_mass - decay_rate**2)
    times = particle_log.times() * macros.NANO2SEC
    cosine = np.cos(damped_frequency * times)
    sine = np.sin(damped_frequency * times)
    sine_amplitude = (particle.rhoDotInit + decay_rate * particle.rhoInit) / damped_frequency
    expected_rho = np.exp(-decay_rate * times) * (particle.rhoInit * cosine + sine_amplitude * sine)
    expected_rho_dot = np.exp(-decay_rate * times) * (
        (-decay_rate * particle.rhoInit + damped_frequency * sine_amplitude) * cosine
        - (decay_rate * sine_amplitude + damped_frequency * particle.rhoInit) * sine
    )
    position_tolerance = 1e-10  # [m]
    velocity_tolerance = 1e-10  # [m/s]
    np.testing.assert_allclose(particle_log.rho, expected_rho, rtol=0, atol=position_tolerance)
    np.testing.assert_allclose(particle_log.rhoDot, expected_rho_dot, rtol=0, atol=velocity_tolerance)
    axis_N = rbk.MRP2C(sigma_MB).T @ axis_P
    expected_recoil = -particle.massInit / (parent_mass + particle.massInit) * np.outer(
        expected_rho - particle.rhoInit, axis_N
    )
    np.testing.assert_allclose(
        hub_log.r_BN_N - hub_log.r_BN_N[0], expected_recoil, rtol=0, atol=position_tolerance
    )


@pytest.mark.parametrize("seed", [1596, 1634, 2026])
def test_prescribed_spring_newton_euler_balance(seed):
    r"""Check the coupled accelerations against independent Newton/Euler equations.

    **Validation Test Description**

    Use non-collinear offsets, nonidentity mount and parent attitudes, simultaneous prescribed
    translation and rotation, and nonzero spring velocity and damping. Reconstruct the inertial
    accelerations from rigid-body kinematics and the integrated state derivatives. In the absence
    of external forces, total force and torque must vanish, and the particle acceleration along
    the spring must satisfy its scalar force equation.

    **Test Parameter Discussion**

    ``seed`` chooses three reproducible arbitrary geometries and prescribed kinematic states.
    The hub angular-velocity state is renamed to exercise callback-based rate access.

    **Description of Variables Being Tested**

    Check hub translational/angular acceleration and particle acceleration through total force,
    total torque, and axial spring-force residuals, without using the backsubstitution formulas.
    """
    rng = np.random.default_rng(seed)
    sim = create_simulation()
    hub = spacecraft.Spacecraft()
    hub.hub.nameOfHubOmega = "customHubOmega"
    hub.hub.mHub = 800.0  # [kg]
    inertia_hub_B = np.diag([150.0, 180.0, 210.0])  # [kg m^2]
    hub.hub.IHubPntBc_B = inertia_hub_B.tolist()
    omega_BN_B = rng.normal(size=3) * 0.2  # [rad/s]
    hub.hub.omega_BN_BInit = omega_BN_B.tolist()
    parent = prescribedMotionStateEffector.PrescribedMotionStateEffector()
    parent.setMass(100.0)  # [kg]
    inertia_parent_P = np.diag([20.0, 25.0, 30.0])  # [kg m^2]
    parent.setIPntPc_P(inertia_parent_P.tolist())
    r_PM_M = rng.normal(size=3)  # [m]
    r_MB_B = rng.normal(size=3)  # [m]
    r_prime_PM_M = rng.normal(size=3) * 0.2  # [m/s]
    r_prime_prime_PM_M = rng.normal(size=3) * 0.3  # [m/s^2]
    omega_PM_P = rng.normal(size=3) * 0.3  # [rad/s]
    omega_prime_PM_P = rng.normal(size=3) * 0.2  # [rad/s^2]
    sigma_PM = rng.normal(size=3) * 0.1  # [-] MRPs
    sigma_MB = rng.normal(size=3) * 0.1  # [-] MRPs
    r_PcP_P = rng.normal(size=3) * 0.3  # [m]
    parent.setR_MB_B(r_MB_B.tolist())
    parent.setSigma_MB(sigma_MB.tolist())
    parent.setSigma_PM(sigma_PM.tolist())
    parent.setR_PcP_P(r_PcP_P.tolist())
    translation = messaging.PrescribedTranslationMsgPayload()
    translation.r_PM_M = r_PM_M.tolist()
    translation.rPrime_PM_M = r_prime_PM_M.tolist()
    translation.rPrimePrime_PM_M = r_prime_prime_PM_M.tolist()
    translation_msg = messaging.PrescribedTranslationMsg().write(translation)
    parent.prescribedTranslationInMsg.subscribeTo(translation_msg)
    rotation = messaging.PrescribedRotationMsgPayload()
    rotation.sigma_PM = sigma_PM.tolist()
    rotation.omega_PM_P = omega_PM_P.tolist()
    rotation.omegaPrime_PM_P = omega_prime_PM_P.tolist()
    rotation_msg = messaging.PrescribedRotationMsg().write(rotation)
    parent.prescribedRotationInMsg.subscribeTo(rotation_msg)
    particle = linearSpringMassDamper.LinearSpringMassDamper()
    particle.massInit = 20.0  # [kg]
    particle.k = 100.0  # [N/m]
    particle.c = 2.0  # [N s/m]
    axis_P = rng.normal(size=3)
    axis_P /= np.linalg.norm(axis_P)
    equilibrium_P = rng.normal(size=3) * 0.5  # [m]
    particle.pHat_B = axis_P.tolist()
    particle.r_PB_B = equilibrium_P.tolist()
    particle.rhoInit = 0.3  # [m]
    particle.rhoDotInit = 0.1  # [m/s]
    parent.addStateEffector(particle)
    hub.addStateEffector(parent)
    sim.AddModelToTask("task", hub)
    sim.AddModelToTask("task", parent)
    sim.InitializeSimulation()
    parent.UpdateState(0)
    hub.equationsOfMotion(0.0, 0.001)  # [s] Initial time and integration step

    def state_derivative(name):
        """Return one registered state derivative as a flat array."""
        return np.array(hub.dynManager.getStateObject(name).getStateDeriv()).reshape(-1)

    # The initial hub attitude is identity, so inertial and hub components agree.
    acceleration_BN_B = state_derivative(hub.hub.nameOfHubVelocity)
    alpha_BN_B = state_derivative(hub.hub.nameOfHubOmega)
    rho_ddot = state_derivative(particle.nameOfRhoDotState)[0]
    dcm_BM = rbk.MRP2C(sigma_MB).T
    dcm_BP = dcm_BM @ rbk.MRP2C(sigma_PM).T
    omega_PB_B = dcm_BP @ omega_PM_P
    omega_prime_PB_B = dcm_BP @ omega_prime_PM_P
    r_PB_B = r_MB_B + dcm_BM @ r_PM_M
    velocity_PB_B = dcm_BM @ r_prime_PM_M
    acceleration_PB_B = dcm_BM @ r_prime_prime_PM_M

    def point_kinematics(position_P, velocity_P, acceleration_P):
        """Return hub-relative position and inertial acceleration, both in hub components."""
        arm_B = dcm_BP @ position_P
        arm_velocity_B = dcm_BP @ velocity_P
        position_B = r_PB_B + arm_B
        velocity_B = velocity_PB_B + np.cross(omega_PB_B, arm_B) + arm_velocity_B
        relative_acceleration_B = (
            acceleration_PB_B + np.cross(omega_prime_PB_B, arm_B)
            + np.cross(omega_PB_B, np.cross(omega_PB_B, arm_B))
            + 2 * np.cross(omega_PB_B, arm_velocity_B) + dcm_BP @ acceleration_P
        )
        inertial_acceleration_B = (
            acceleration_BN_B + np.cross(alpha_BN_B, position_B)
            + np.cross(omega_BN_B, np.cross(omega_BN_B, position_B))
            + 2 * np.cross(omega_BN_B, velocity_B) + relative_acceleration_B
        )
        return position_B, inertial_acceleration_B

    r_parent_B, acceleration_parent_B = point_kinematics(r_PcP_P, np.zeros(3), np.zeros(3))
    r_particle_B, acceleration_particle_B = point_kinematics(
        equilibrium_P + particle.rhoInit * axis_P, particle.rhoDotInit * axis_P, rho_ddot * axis_P
    )
    force_residual = (
        hub.hub.mHub * acceleration_BN_B + parent.getMass() * acceleration_parent_B
        + particle.massInit * acceleration_particle_B
    )
    inertia_parent_B = dcm_BP @ inertia_parent_P @ dcm_BP.T
    omega_PN_B = omega_BN_B + omega_PB_B
    alpha_PN_B = alpha_BN_B + omega_prime_PB_B + np.cross(omega_BN_B, omega_PB_B)
    torque_residual = (
        inertia_hub_B @ alpha_BN_B + np.cross(omega_BN_B, inertia_hub_B @ omega_BN_B)
        + inertia_parent_B @ alpha_PN_B + np.cross(omega_PN_B, inertia_parent_B @ omega_PN_B)
        + np.cross(r_parent_B, parent.getMass() * acceleration_parent_B)
        + np.cross(r_particle_B, particle.massInit * acceleration_particle_B)
    )
    spring_residual = (
        particle.massInit * np.dot(dcm_BP @ axis_P, acceleration_particle_B)
        + particle.k * particle.rhoInit + particle.c * particle.rhoDotInit
    )
    force_tolerance = 1e-10  # [N]
    torque_tolerance = 1e-10  # [N m]
    np.testing.assert_allclose(force_residual, np.zeros(3), rtol=0, atol=force_tolerance)
    np.testing.assert_allclose(torque_residual, np.zeros(3), rtol=0, atol=torque_tolerance)
    np.testing.assert_allclose(spring_residual, 0, rtol=0, atol=force_tolerance)


if __name__ == "__main__":
    for damping in (0.0, 4.0):  # [N s/m]
        test_prescribed_spring_analytical_motion(damping)
    for seed in (1596, 1634, 2026):
        test_prescribed_spring_newton_euler_balance(seed)
