#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Laboratory, University of Colorado at Boulder
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

import numpy as np
import math
import pytest
import matplotlib.pyplot as plt
from Basilisk.architecture import astroConstants
from Basilisk.simulation import spacecraft
from Basilisk.simulation import spacecraftChargingDynamics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging

temp_electrons = 2.0  # [eV] Plasma electron temperature
density_electrons = 950000.0  # [m^-3] Plasma electron density
temp_ions = 2.0  # [eV] Plasma ion temperature
density_ions = 950000.0  # [m^-3] Plasma ion density
temp_photons = 2.0  # [eV] Photoelectron temperature
flux_photons = 1e-6  # [A/m^2] Photoelectron flux

@pytest.mark.parametrize(
    ("servicer_potential_init", "target_potential_init"),  # ([Volts], [Volts])
    [
        (0.0, 0.0),
        (-5000.0, -5000.0),  # GEO eclipse
        (10.0, 10.0),  # GEO sunlit
        (11000.0, 0.0)  # Electron beam should not reach target
    ]
)
@pytest.mark.parametrize(
    ("servicer_radius", "target_radius"),  # ([m], [m])
    [
        (3.0, 3.0),
        (2.0, 4.0),
        (4.0, 2.0)
    ]
)
@pytest.mark.parametrize("bulk_velocity_ions", [0.0, 1.0, 400000.0])  # [m/s]
@pytest.mark.parametrize("electron_beam_energy", [0.0, 10000.0])  # [eV]
@pytest.mark.parametrize("electron_beam_current", [0.0, 0.001, 250e-6])  # [Amps]
@pytest.mark.parametrize("electron_beam_alpha", [0.0, 0.5, 1.0])  # [-]
def test_spacecraft_charging_dynamics(show_plots,
                                      servicer_potential_init,
                                      target_potential_init,
                                      servicer_radius,
                                      target_radius,
                                      bulk_velocity_ions,
                                      electron_beam_energy,
                                      electron_beam_current,
                                      electron_beam_alpha):
    r"""
    **Verification Test Description**

    This unit test verifies that the spacecraft charging dynamics module correctly computes the different types of
    currents impacting both a target and servicer spacecraft. Specifically, this test checks that the module
    correctly computes the photoelectric current, electron beam current, plasma electron current, and plasma ion
    current acting on both spacecraft. While the module defaults many required variables, the user has the ability
    to configure all information describing the electrons, ions, and photons using setter methods.

    The module requires several input messages to be connected, including messages for the spacecraft inertial states,
    surface areas, and sunlit areas. Configuration of the electron beam is optional. The spacecraft potentials and
    electric currents are output from the module.

    **Test Parameters**

    Args:
        servicer_potential_init (float): [Volts] Servicer initial potential
        target_potential_init (float): [Volts] Target initial potential
        servicer_radius (float): [m] Servicer spacecraft radius
        target_radius (float): [m] Target spacecraft radius
        bulk_velocity_ions (float): [m/s] Bulk velocity of plasma ions
        electron_beam_energy (float) [eV] Electron beam energy
        electron_beam_current (float) [Amps] Electron beam current
        electron_beam_alpha (float) [-] Scaling term for the fraction of current reaching the target

    **Description of Variables Being Tested**

    The test checks that the module correctly computes the photoelectric current, electron beam current, plasma
    electron current, and plasma ion current acting on both spacecraft.
    """

    task_name = "unitTask"
    process_name = "TestProcess"
    sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 1e-7
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = sim.CreateNewProcess(process_name)
    test_process.addTask(sim.CreateNewTask(task_name, test_process_rate))

    # Create the servicer spacecraft
    mass_servicer = 500  # [kg]
    length_servicer = servicer_radius  # [m]
    width_servicer = servicer_radius  # [m]
    height_servicer = servicer_radius  # [m]
    I_servicer_11 = (1 / 12) * mass_servicer * (length_servicer * length_servicer + height_servicer * height_servicer)  # [kg m^2]
    I_servicer_22 = (1 / 12) * mass_servicer * (length_servicer * length_servicer + width_servicer * width_servicer)  # [kg m^2]
    I_servicer_33 = (1 / 12) * mass_servicer * (width_servicer * width_servicer + height_servicer * height_servicer)  # [kg m^2]
    servicer_spacecraft = spacecraft.Spacecraft()
    servicer_spacecraft.ModelTag = "ServicerSpacecraft"
    servicer_spacecraft.hub.mHub = mass_servicer  # [kg]
    servicer_spacecraft.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    servicer_spacecraft.hub.IHubPntBc_B = [[I_servicer_11, 0.0, 0.0], [0.0, I_servicer_22, 0.0], [0.0, 0.0, I_servicer_33]]  # [kg m^2]
    servicer_spacecraft.hub.r_CN_NInit = [[10.0], [0.0], [0.0]]  # [m]
    servicer_spacecraft.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
    servicer_spacecraft.hub.omega_BN_BInit = [[0.0], [0.0], [macros.D2R * 1.5]]  # [rad/s]
    servicer_spacecraft.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sim.AddModelToTask(task_name, servicer_spacecraft)

    # Create the target spacecraft
    mass_target = 800  # [kg]
    length_target = target_radius  # [m]
    width_target = target_radius  # [m]
    height_target = target_radius  # [m]
    I_target_11 = (1 / 12) * mass_target * (length_target * length_target + height_target * height_target)  # [kg m^2]
    I_target_22 = (1 / 12) * mass_target * (length_target * length_target + width_target * width_target)  # [kg m^2]
    I_target_33 = (1 / 12) * mass_target * (width_target * width_target + height_target * height_target)  # [kg m^2]
    target_spacecraft = spacecraft.Spacecraft()
    target_spacecraft.ModelTag = "TargetSpacecraft"
    target_spacecraft.hub.mHub = mass_target  # [kg]
    target_spacecraft.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    target_spacecraft.hub.IHubPntBc_B = [[I_target_11, 0.0, 0.0], [0.0, I_target_22, 0.0], [0.0, 0.0, I_target_33]]  # [kg m^2]
    target_spacecraft.hub.r_CN_NInit = [[5.0], [0.0], [0.0]]  # [m]
    target_spacecraft.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
    target_spacecraft.hub.omega_BN_BInit = [[0.0], [0.0], [macros.D2R * -1.0]]  # [rad/s]
    target_spacecraft.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sim.AddModelToTask(task_name, target_spacecraft)

    # Create the spacecraft area messages
    servicer_surface_area = 4 * np.pi * servicer_radius * servicer_radius  # [m^2]
    target_surface_area = 4 * np.pi * target_radius * target_radius  # [m^2]
    servicer_sunlit_area = np.pi * servicer_radius * servicer_radius  # [m^2]
    target_sunlit_area = np.pi * target_radius * target_radius  # [m^2]

    servicer_surface_area_msg_data = messaging.ProjectedAreaMsgPayload()
    servicer_surface_area_msg_data.area = servicer_surface_area  # [m^2]
    servicer_surface_area_msg = messaging.ProjectedAreaMsg().write(servicer_surface_area_msg_data)

    target_surface_area_msg_data = messaging.ProjectedAreaMsgPayload()
    target_surface_area_msg_data.area = target_surface_area  # [m^2]
    target_surface_area_msg = messaging.ProjectedAreaMsg().write(target_surface_area_msg_data)

    servicer_sunlit_area_msg_data = messaging.ProjectedAreaMsgPayload()
    servicer_sunlit_area_msg_data.area = servicer_sunlit_area  # [m^2]
    servicer_sunlit_area_msg = messaging.ProjectedAreaMsg().write(servicer_sunlit_area_msg_data)

    target_sunlit_area_msg_data = messaging.ProjectedAreaMsgPayload()
    target_sunlit_area_msg_data.area = target_sunlit_area  # [m^2]
    target_sunlit_area_msg = messaging.ProjectedAreaMsg().write(target_sunlit_area_msg_data)

    # Create electron beam input message
    electron_beam_msg_data = messaging.ElectronBeamMsgPayload()
    electron_beam_msg_data.energyEB = electron_beam_energy  # [eV]
    electron_beam_msg_data.currentEB = electron_beam_current  # [Amps]
    electron_beam_msg_data.alphaEB = electron_beam_alpha  # [-]
    electron_beam_msg = messaging.ElectronBeamMsg().write(electron_beam_msg_data)

    # Create the spacecraft charging module
    capacitance = 1e-9  # [farads]
    spacecraft_charging = spacecraftChargingDynamics.SpacecraftChargingDynamics()
    spacecraft_charging.ModelTag = "SpacecraftChargingDynamics"
    spacecraft_charging.setServicerPotentialInit(servicer_potential_init)
    spacecraft_charging.setTargetPotentialInit(target_potential_init)
    spacecraft_charging.setServicerCapacitance(capacitance)
    spacecraft_charging.setTargetCapacitance(capacitance)
    spacecraft_charging.setFluxPhotoelectrons(flux_photons)
    spacecraft_charging.setTempElectrons(temp_electrons)
    spacecraft_charging.setDensityElectrons(density_electrons)
    spacecraft_charging.setTempIons(temp_ions)
    spacecraft_charging.setDensityIons(density_ions)
    spacecraft_charging.setBulkVelocityIons(bulk_velocity_ions)
    spacecraft_charging.servicerStateInMsg.subscribeTo(servicer_spacecraft.scStateOutMsg)
    spacecraft_charging.targetStateInMsg.subscribeTo(target_spacecraft.scStateOutMsg)
    spacecraft_charging.electronBeamInMsg.subscribeTo(electron_beam_msg)
    spacecraft_charging.servicerSurfaceAreaInMsg.subscribeTo(servicer_surface_area_msg)
    spacecraft_charging.targetSurfaceAreaInMsg.subscribeTo(target_surface_area_msg)
    spacecraft_charging.servicerSunlitAreaInMsg.subscribeTo(servicer_sunlit_area_msg)
    spacecraft_charging.targetSunlitAreaInMsg.subscribeTo(target_sunlit_area_msg)
    sim.AddModelToTask(task_name, spacecraft_charging)

    # Set up data logging
    target_state_data_log = target_spacecraft.scStateOutMsg.recorder()
    servicer_state_data_log = servicer_spacecraft.scStateOutMsg.recorder()
    servicer_potential_data_log = spacecraft_charging.servicerPotentialOutMsg.recorder()
    target_potential_data_log = spacecraft_charging.targetPotentialOutMsg.recorder()
    servicer_photoelectric_current_sim_data_log = spacecraft_charging.servicerPhotoelectricCurrentOutMsg.recorder()
    target_photoelectric_current_sim_data_log = spacecraft_charging.targetPhotoelectricCurrentOutMsg.recorder()
    servicer_plasma_electron_current_sim_data_log = spacecraft_charging.servicerPlasmaElectronCurrentOutMsg.recorder()
    target_plasma_electron_current_sim_data_log = spacecraft_charging.targetPlasmaElectronCurrentOutMsg.recorder()
    servicer_plasma_ion_current_sim_data_log = spacecraft_charging.servicerPlasmaIonCurrentOutMsg.recorder()
    target_plasma_ion_current_sim_data_log = spacecraft_charging.targetPlasmaIonCurrentOutMsg.recorder()
    servicer_electron_beam_current_sim_data_log = spacecraft_charging.servicerEBCurrentOutMsg.recorder()
    target_electron_beam_current_sim_data_log = spacecraft_charging.targetEBCurrentOutMsg.recorder()

    sim.AddModelToTask(task_name, target_state_data_log)
    sim.AddModelToTask(task_name, servicer_state_data_log)
    sim.AddModelToTask(task_name, servicer_potential_data_log)
    sim.AddModelToTask(task_name, target_potential_data_log)
    sim.AddModelToTask(task_name, servicer_photoelectric_current_sim_data_log)
    sim.AddModelToTask(task_name, target_photoelectric_current_sim_data_log)
    sim.AddModelToTask(task_name, servicer_plasma_electron_current_sim_data_log)
    sim.AddModelToTask(task_name, target_plasma_electron_current_sim_data_log)
    sim.AddModelToTask(task_name, servicer_plasma_ion_current_sim_data_log)
    sim.AddModelToTask(task_name, target_plasma_ion_current_sim_data_log)
    sim.AddModelToTask(task_name, servicer_electron_beam_current_sim_data_log)
    sim.AddModelToTask(task_name, target_electron_beam_current_sim_data_log)

    # Run the simulation
    sim.InitializeSimulation()
    sim_time = 0.00001  # [s]
    sim.ConfigureStopTime(macros.sec2nano(sim_time))
    sim.ExecuteSimulation()

    # Grab data for unit test check and plotting
    timespan = servicer_potential_data_log.times() * macros.NANO2SEC  # [s]
    v_SN_N_sim = servicer_state_data_log.v_BN_N  # [m/s]
    v_TN_N_sim = target_state_data_log.v_BN_N  # [m/s]
    servicer_potential_list_sim = servicer_potential_data_log.voltage  # [Volts]
    target_potential_list_sim = target_potential_data_log.voltage  # [Volts]
    servicer_photoelectric_current_sim = servicer_photoelectric_current_sim_data_log.current  # [Amps]
    target_photoelectric_current_sim = target_photoelectric_current_sim_data_log.current  # [Amps]
    servicer_plasma_electron_current_sim = servicer_plasma_electron_current_sim_data_log.current  # [Amps]
    target_plasma_electron_current_sim = target_plasma_electron_current_sim_data_log.current  # [Amps]
    servicer_plasma_ion_current_sim = servicer_plasma_ion_current_sim_data_log.current  # [Amps]
    target_plasma_ion_current_sim = target_plasma_ion_current_sim_data_log.current  # [Amps]
    servicer_electron_beam_current_sim = servicer_electron_beam_current_sim_data_log.current  # [Amps]
    target_electron_beam_current_sim = target_electron_beam_current_sim_data_log.current  # [Amps]

    # Compute current truth information
    servicer_plasma_electron_current_list_truth = []
    target_plasma_electron_current_list_truth = []
    servicer_plasma_ion_current_list_truth = []
    target_plasma_ion_current_list_truth = []
    servicer_photoelectric_current_truth = []
    target_photoelectric_current_truth = []
    servicer_electron_beam_current_list_truth = []
    target_electron_beam_current_list_truth = []
    for idx in range(len(timespan)):
        servicer_plasma_electron_current = compute_plasma_electron_current(servicer_potential_list_sim[idx],
                                                                           servicer_surface_area)
        target_plasma_electron_current = compute_plasma_electron_current(target_potential_list_sim[idx],
                                                                         target_surface_area)
        servicer_plasma_ion_current = compute_plasma_ion_current(servicer_potential_list_sim[idx],
                                                                 servicer_surface_area,
                                                                 servicer_sunlit_area,
                                                                 v_SN_N_sim[idx],
                                                                 bulk_velocity_ions)
        target_plasma_ion_current = compute_plasma_ion_current(target_potential_list_sim[idx],
                                                               target_surface_area,
                                                               target_sunlit_area,
                                                               v_TN_N_sim[idx],
                                                               bulk_velocity_ions)
        (servicer_photoelectric_current,
         target_photoelectric_current) = compute_photoelectric_current(servicer_potential_list_sim[idx],
                                                                       target_potential_list_sim[idx],
                                                                       servicer_sunlit_area,
                                                                       target_sunlit_area)

        servicer_electron_beam_current, target_electron_beam_current = compute_electron_beam_current(electron_beam_current,
                                                                                                     electron_beam_energy,
                                                                                                     electron_beam_alpha,
                                                                                                     servicer_potential_list_sim[idx],
                                                                                                     target_potential_list_sim[idx])

        servicer_plasma_electron_current_list_truth.append(servicer_plasma_electron_current)
        target_plasma_electron_current_list_truth.append(target_plasma_electron_current)
        servicer_plasma_ion_current_list_truth.append(servicer_plasma_ion_current)
        target_plasma_ion_current_list_truth.append(target_plasma_ion_current)
        servicer_photoelectric_current_truth.append(servicer_photoelectric_current)
        target_photoelectric_current_truth.append(target_photoelectric_current)
        servicer_electron_beam_current_list_truth.append(servicer_electron_beam_current)
        target_electron_beam_current_list_truth.append(target_electron_beam_current)

    # Compute servicer and target potential truth information
    servicer_potential_list_truth = [servicer_potential_init]
    target_potential_list_truth = [target_potential_init]
    for idx in range(len(timespan)-1):
        servicer_potential = servicer_potential_list_sim[idx]
        target_potential = target_potential_list_sim[idx]
        v_SN_N = v_SN_N_sim[idx]  # [m/s]
        v_TN_N = v_TN_N_sim[idx]  # [m/s]

        # Integrate first order odes using RK4 algorithm
        servicer_k1, target_k1 = equations_of_motion(servicer_potential,
                                                     target_potential,
                                                     servicer_surface_area,
                                                     target_surface_area,
                                                     servicer_sunlit_area,
                                                     target_sunlit_area,
                                                     v_SN_N,
                                                     v_TN_N,
                                                     bulk_velocity_ions,
                                                     capacitance,
                                                     electron_beam_current,
                                                     electron_beam_energy,
                                                     electron_beam_alpha)
        servicer_k2, target_k2 = equations_of_motion(servicer_potential + 0.5 * (test_time_step_sec * servicer_k1),
                                                     target_potential + 0.5 * (test_time_step_sec * target_k1),
                                                     servicer_surface_area,
                                                     target_surface_area,
                                                     servicer_sunlit_area,
                                                     target_sunlit_area,
                                                     v_SN_N,
                                                     v_TN_N,
                                                     bulk_velocity_ions,
                                                     capacitance,
                                                     electron_beam_current,
                                                     electron_beam_energy,
                                                     electron_beam_alpha)
        servicer_k3, target_k3 = equations_of_motion(servicer_potential + 0.5 * (test_time_step_sec * servicer_k2),
                                                     target_potential + 0.5 * (test_time_step_sec * target_k2),
                                                     servicer_surface_area,
                                                     target_surface_area,
                                                     servicer_sunlit_area,
                                                     target_sunlit_area,
                                                     v_SN_N,
                                                     v_TN_N,
                                                     bulk_velocity_ions,
                                                     capacitance,
                                                     electron_beam_current,
                                                     electron_beam_energy,
                                                     electron_beam_alpha)
        servicer_k4, target_k4 = equations_of_motion(servicer_potential + test_time_step_sec * servicer_k3,
                                                     target_potential + test_time_step_sec * target_k3,
                                                     servicer_surface_area,
                                                     target_surface_area,
                                                     servicer_sunlit_area,
                                                     target_sunlit_area,
                                                     v_SN_N,
                                                     v_TN_N,
                                                     bulk_velocity_ions,
                                                     capacitance,
                                                     electron_beam_current,
                                                     electron_beam_energy,
                                                     electron_beam_alpha)

        servicer_potential_list_truth = np.append(servicer_potential_list_truth, servicer_potential + (test_time_step_sec / 6) * (servicer_k1 + 2 * servicer_k2 + 2 * servicer_k3 + servicer_k4))
        target_potential_list_truth = np.append(target_potential_list_truth, target_potential + (test_time_step_sec / 6) * (target_k1 + 2 * target_k2 + 2 * target_k3 + target_k4))

    plt.close("all")
    if show_plots:

        # Plot the servicer currents
        plt.figure(1)
        plt.clf()
        plt.plot(timespan*1000000, servicer_photoelectric_current_sim, label=r"$I_{ph}$")
        plt.plot(timespan*1000000, servicer_plasma_electron_current_sim, label=r"$I_{e}$")
        plt.plot(timespan*1000000, servicer_plasma_ion_current_sim, label=r"$I_{i}$")
        plt.plot(timespan*1000000, servicer_electron_beam_current_sim, label=r"$I_{EB}$")
        plt.title('Servicer Currents', fontsize=16)
        plt.ylabel('Current (A)', fontsize=16)
        plt.xlabel(r'Time ($\mu$s)', fontsize=16)
        plt.grid(True)
        plt.legend()

        # Plot the target currents
        plt.figure(2)
        plt.clf()
        plt.plot(timespan*1000000, target_photoelectric_current_sim, label=r"$I_{ph}$")
        plt.plot(timespan*1000000, target_plasma_electron_current_sim, label=r"$I_{e}$")
        plt.plot(timespan*1000000, target_plasma_ion_current_sim, label=r"$I_{i}$")
        plt.plot(timespan*1000000, target_electron_beam_current_sim, label=r"$I_{EB}$")
        plt.title('Target Currents', fontsize=16)
        plt.ylabel('Current (A)', fontsize=16)
        plt.xlabel(r'Time ($\mu$s)', fontsize=16)
        plt.grid(True)
        plt.legend()

        # Plot the servicer and target potentials
        plt.figure(3)
        plt.clf()
        plt.plot(timespan*1000000, servicer_potential_list_sim, label=r"$\phi_{\text{S, sim}}$")
        plt.plot(timespan*1000000, target_potential_list_sim, label=r"$\phi_{\text{T, sim}}$")
        plt.suptitle(r'Servicer and Target Spacecraft Potentials', fontsize=16)
        plt.ylabel('(Volts)', fontsize=16)
        plt.xlabel(r'Time ($\mu$s)', fontsize=16)
        plt.legend(loc='lower right', prop={'size': 16})
        plt.grid(True)

        # Plot the difference between simulated and truth servicer and target potentials
        plt.figure(4)
        plt.clf()
        plt.plot(timespan*1000000, np.abs(servicer_potential_list_truth - servicer_potential_list_sim), label=r"$\epsilon_{\text{S}}$")
        plt.plot(timespan*1000000, np.abs(target_potential_list_truth - target_potential_list_sim), label=r"$\epsilon_{\text{ST}}$")
        plt.suptitle('Servicer and Target Potential Errors', fontsize=16)
        plt.ylabel('(Volts)', fontsize=16)
        plt.xlabel(r'Time ($\mu$s)', fontsize=16)
        plt.legend(loc='lower right', prop={'size': 16})
        plt.grid(True)
        plt.show()

    # Check the simulated values match the computed truth values
    for idx in range(len(timespan)):
        np.testing.assert_allclose(servicer_photoelectric_current_sim[idx],
                                   servicer_photoelectric_current_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(target_photoelectric_current_sim[idx],
                                   target_photoelectric_current_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(servicer_plasma_electron_current_sim[idx],
                                   servicer_plasma_electron_current_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(target_plasma_electron_current_sim[idx],
                                   target_plasma_electron_current_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(servicer_plasma_ion_current_sim[idx],
                                   servicer_plasma_ion_current_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(target_plasma_ion_current_sim[idx],
                                   target_plasma_ion_current_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(servicer_electron_beam_current_sim[idx],
                                   servicer_electron_beam_current_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(target_electron_beam_current_sim[idx],
                                   target_electron_beam_current_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(servicer_potential_list_sim[idx],
                                   servicer_potential_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)
        np.testing.assert_allclose(target_potential_list_sim[idx],
                                   target_potential_list_truth[idx],
                                   atol=1e-7,
                                   verbose=True)

def equations_of_motion(servicer_potential,
                        target_potential,
                        servicer_surface_area,
                        target_surface_area,
                        servicer_sunlit_area,
                        target_sunlit_area,
                        v_SN_N,
                        v_TN_N,
                        bulk_velocity_ions,
                        capacitance,
                        electron_beam_current,
                        electron_beam_energy,
                        electron_beam_alpha):

    # Compute instantaneous currents
    servicer_plasma_electron_current = compute_plasma_electron_current(servicer_potential,
                                                                       servicer_surface_area)
    target_plasma_electron_current = compute_plasma_electron_current(target_potential,
                                                                     target_surface_area)
    servicer_plasma_ion_current = compute_plasma_ion_current(servicer_potential,
                                                             servicer_surface_area,
                                                             servicer_sunlit_area,
                                                             v_SN_N,
                                                             bulk_velocity_ions)
    target_plasma_ion_current = compute_plasma_ion_current(target_potential,
                                                           target_surface_area,
                                                           target_sunlit_area,
                                                           v_TN_N,
                                                           bulk_velocity_ions)
    (servicer_photoelectric_current,
     target_photoelectric_current) = compute_photoelectric_current(servicer_potential,
                                                                   target_potential,
                                                                   servicer_sunlit_area,
                                                                   target_sunlit_area)

    servicer_electron_beam_current, target_electron_beam_current = compute_electron_beam_current(electron_beam_current,
                                                                                                 electron_beam_energy,
                                                                                                 electron_beam_alpha,
                                                                                                 servicer_potential,
                                                                                                 target_potential)
    # Charging EOM
    servicer_total_current = (servicer_plasma_electron_current
                              + servicer_plasma_ion_current
                              + servicer_photoelectric_current
                              + servicer_electron_beam_current)
    target_total_current = (target_plasma_electron_current
                            + target_plasma_ion_current
                            + target_photoelectric_current
                            + target_electron_beam_current)
    servicer_potential_rate = servicer_total_current / capacitance
    target_potential_rate = target_total_current / capacitance

    return servicer_potential_rate, target_potential_rate

def compute_plasma_electron_current(spacecraft_potential,
                                    surface_area):
    velocity_electrons = math.sqrt((8 * astroConstants.Q_CHARGE * temp_electrons) / (astroConstants.MASS_ELECTRON * astroConstants.MPI))
    if spacecraft_potential <= 0:
        plasma_electron_current = (-0.25 * surface_area * astroConstants.Q_CHARGE * density_electrons * velocity_electrons) * math.exp(spacecraft_potential / temp_electrons)
    else:
        plasma_electron_current = (-0.25 * surface_area * astroConstants.Q_CHARGE * density_electrons * velocity_electrons) * (1 + (spacecraft_potential / temp_electrons))

    return plasma_electron_current

def compute_plasma_ion_current(spacecraft_potential,
                               surface_area,
                               sunlit_area,
                               v_BN_N,
                               bulk_velocity_ions):
    thermal_velocity_ions = math.sqrt((8 * astroConstants.Q_CHARGE * temp_ions) / (astroConstants.MASS_PROTON * astroConstants.MPI))
    relative_velocity_ions = np.abs(np.linalg.norm(v_BN_N) - bulk_velocity_ions)

    if thermal_velocity_ions >= relative_velocity_ions:
        if spacecraft_potential <= 0:
            plasma_ion_current = (0.25 * surface_area * astroConstants.Q_CHARGE * density_ions * thermal_velocity_ions) * (1 - (spacecraft_potential / temp_ions))
        else:
            plasma_ion_current = (0.25 * surface_area * astroConstants.Q_CHARGE * density_ions * thermal_velocity_ions) * math.exp(-spacecraft_potential / temp_ions)
    else:
        plasma_ion_current = sunlit_area * astroConstants.Q_CHARGE * density_ions * relative_velocity_ions

    return plasma_ion_current

def compute_photoelectric_current(servicer_potential,
                                  target_potential,
                                  servicer_sunlit_area,
                                  target_sunlit_area):
    # Compute the servicer photoelectric current
    if servicer_potential <= 0:
        servicer_photoelectric_current = flux_photons * servicer_sunlit_area
    else:
        servicer_photoelectric_current = flux_photons * servicer_sunlit_area * math.exp(-1 * servicer_potential / temp_photons)

    # Compute the target photoelectric current
    if target_potential <= 0:
        target_photoelectric_current = flux_photons * target_sunlit_area
    else:
        target_photoelectric_current = flux_photons * target_sunlit_area * math.exp(-1 * target_potential / temp_photons)

    return servicer_photoelectric_current, target_photoelectric_current

def compute_electron_beam_current(electron_beam_current,
                                  electron_beam_energy,
                                  electron_beam_alpha,
                                  servicer_potential,
                                  target_potential):

    if electron_beam_energy > (servicer_potential - target_potential):
        intermediate_term = -1 * (electron_beam_energy - servicer_potential + target_potential) / 20.0
        servicer_electron_beam_current = electron_beam_current * (1 - math.exp(intermediate_term))
        target_electron_beam_current = - electron_beam_alpha * electron_beam_current * (1 - math.exp(intermediate_term))
    else:
        servicer_electron_beam_current = 0.0
        target_electron_beam_current = 0.0

    return servicer_electron_beam_current, target_electron_beam_current

if __name__ == "__main__":
    test_spacecraft_charging_dynamics(
        False,  # show_plots
        0.0,  # [Volts]
        0.0,  # [Volts]
        4.0,  # [m]
        2.0,  # [m]
        400000.0,  # [m/s]
        10000.0,  # [eV]
        250e-6,  # [Amps]
        1.0  # [-]
    )
