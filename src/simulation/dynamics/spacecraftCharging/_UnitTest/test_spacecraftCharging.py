#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Laboratory, University of Colorado at Boulder
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

import os
import matplotlib.pyplot as plt
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import spacecraft
from Basilisk.simulation import computeSCSunlitFacetArea
from Basilisk.simulation import spacecraftCharging
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.utilities import vizSupport
import numpy as np

filename = os.path.basename(os.path.splitext(__file__)[0])
def test_spacecraft_charging_photoelectric_current(show_plots):
    r"""
    **Verification Test Description**

    **Test Parameters**

    Args:

    **Description of Variables Being Tested**

    """

    task_name = "unitTask"
    process_name = "TestProcess"
    sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.001 # 1e-6
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = sim.CreateNewProcess(process_name)
    test_process.addTask(sim.CreateNewTask(task_name, test_process_rate))

    # Create the Sun
    grav_factory = simIncludeGravBody.gravBodyFactory()
    sun = grav_factory.createSun()
    sun.isCentralBody = True
    mu = sun.mu

    # Set custom Sun Spice data
    sun_state_msg = messaging.SpicePlanetStateMsgPayload()
    sun_state_msg.PositionVector = [0.0, 0.0, 0.0]
    sun_state_msg.VelocityVector = [0.0, 0.0, 0.0]
    sun_msg = messaging.SpicePlanetStateMsg().write(sun_state_msg)
    grav_factory.gravBodies['sun'].planetBodyInMsg.subscribeTo(sun_msg)

    mass_hub = 800  # [kg]
    length_hub = 1.0  # [m]
    width_hub = 1.0  # [m]
    depth_hub = 1.0  # [m]
    I_hub_11 = (1 / 12) * mass_hub * (length_hub * length_hub + depth_hub * depth_hub)  # [kg m^2]
    I_hub_22 = (1 / 12) * mass_hub * (length_hub * length_hub + width_hub * width_hub)  # [kg m^2]
    I_hub_33 = (1 / 12) * mass_hub * (width_hub * width_hub + depth_hub * depth_hub)  # [kg m^2]

    # Create the servicer spacecraft
    servicer_spacecraft = spacecraft.Spacecraft()
    servicer_spacecraft.ModelTag = "ServicerSpacecraft"
    servicer_spacecraft.hub.mHub = mass_hub  # kg
    servicer_spacecraft.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    servicer_spacecraft.hub.IHubPntBc_B = [[I_hub_11, 0.0, 0.0], [0.0, I_hub_22, 0.0], [0.0, 0.0, I_hub_33]]  # [kg m^2] (Hub approximated as a cube)
    servicer_spacecraft.hub.r_CN_NInit = [[10.0], [0.0], [0.0]]
    servicer_spacecraft.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    servicer_spacecraft.hub.omega_BN_BInit = [[0.0], [0.0], [macros.D2R * 1.5]]
    servicer_spacecraft.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sim.AddModelToTask(task_name, servicer_spacecraft)

    # Create the servicer spacecraft
    target_spacecraft = spacecraft.Spacecraft()
    target_spacecraft.ModelTag = "TargetSpacecraft"
    target_spacecraft.hub.mHub = mass_hub  # kg
    target_spacecraft.hub.r_BcB_B = [0.0, 0.0, 0.0]  # [m]
    target_spacecraft.hub.IHubPntBc_B = [[I_hub_11, 0.0, 0.0], [0.0, I_hub_22, 0.0], [0.0, 0.0, I_hub_33]]  # [kg m^2] (Hub approximated as a cube)
    target_spacecraft.hub.r_CN_NInit = [[5.0], [0.0], [0.0]]
    target_spacecraft.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    target_spacecraft.hub.omega_BN_BInit = [[0.0], [0.0], [macros.D2R * -1.0]]
    target_spacecraft.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sim.AddModelToTask(task_name, target_spacecraft)

    # Define facet areas
    num_facets = 10
    area1 = 1.5 * 1.5
    area2 = np.pi * (0.5 * 7.5) * (0.5 * 7.5)
    facet_area_list = [area1, area1, area1, area1, area1, area1, area2, area2, area2, area2]

    # Define the initial facet attitudes relative to B frame
    prv_F01B = (macros.D2R * -90.0) * np.array([0.0, 0.0, 1.0])
    prv_F02B = (macros.D2R * 0.0) * np.array([0.0, 0.0, 1.0])
    prv_F03B = (macros.D2R * 90.0) * np.array([0.0, 0.0, 1.0])
    prv_F04B = (macros.D2R * 180.0) * np.array([0.0, 0.0, 1.0])
    prv_F05B = (macros.D2R * 90.0) * np.array([1.0, 0.0, 0.0])
    prv_F06B = (macros.D2R * -90.0) * np.array([1.0, 0.0, 0.0])
    prv_F07B = (macros.D2R * 0.0) * np.array([0.0, 0.0, 1.0])
    prv_F08B = (macros.D2R * 180.0) * np.array([0.0, 0.0, 1.0])
    prv_F09B = (macros.D2R * 0.0) * np.array([0.0, 0.0, 1.0])
    prv_F010B = (macros.D2R * 180.0) * np.array([0.0, 0.0, 1.0])
    facetDcm_F0BList = [rbk.PRV2C(prv_F01B),
                        rbk.PRV2C(prv_F02B),
                        rbk.PRV2C(prv_F03B),
                        rbk.PRV2C(prv_F04B),
                        rbk.PRV2C(prv_F05B),
                        rbk.PRV2C(prv_F06B),
                        rbk.PRV2C(prv_F07B),
                        rbk.PRV2C(prv_F08B),
                        rbk.PRV2C(prv_F09B),
                        rbk.PRV2C(prv_F010B)]

    # Define the facet normal vectors in F frame components
    facetNHat_FList = [np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]),
                       np.array([0.0, 1.0, 0.0])]

    # Define facet articulation axes in F frame components
    facetRotHat_FList = [np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([0.0, 0.0, 0.0]),
                         np.array([1.0, 0.0, 0.0]),
                         np.array([-1.0, 0.0, 0.0]),
                         np.array([-1.0, 0.0, 0.0]),
                         np.array([1.0, 0.0, 0.0])]

    # Define facet center of pressure locations relative to point B
    facetR_CopB_BList = [np.array([0.75, 0.0, 0.0]),
                         np.array([0.0, 0.75, 0.0]),
                         np.array([-0.75, 0.0, 0.0]),
                         np.array([0.0, -0.75, 0.0]),
                         np.array([0.0, 0.0, 0.75]),
                         np.array([0.0, 0.0, -0.75]),
                         np.array([4.5, 0.0, 0.75]),
                         np.array([4.5, 0.0, 0.75]),
                         np.array([-4.5, 0.0, 0.75]),
                         np.array([-4.5, 0.0, 0.75])]

    # Define facet optical coefficients
    facetDiffuseCoeffList = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    facetSpecularCoeffList = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    # Create the sunlit facet area module for the servicer
    compute_servicer_sunlit_facet_area = computeSCSunlitFacetArea.ComputeSCSunlitFacetArea()
    compute_servicer_sunlit_facet_area.scStatesInMsg.subscribeTo(servicer_spacecraft.scStateOutMsg)
    compute_servicer_sunlit_facet_area.sunInMsg.subscribeTo(sun_msg)
    compute_servicer_sunlit_facet_area.setNumFacets(num_facets)
    compute_servicer_sunlit_facet_area.setNumArticulatedFacets(0)
    sim.AddModelToTask(task_name, compute_servicer_sunlit_facet_area)

    # Create the sunlit facet area module for the target
    compute_target_sunlit_facet_area = computeSCSunlitFacetArea.ComputeSCSunlitFacetArea()
    compute_target_sunlit_facet_area.scStatesInMsg.subscribeTo(target_spacecraft.scStateOutMsg)
    compute_target_sunlit_facet_area.sunInMsg.subscribeTo(sun_msg)
    compute_target_sunlit_facet_area.setNumFacets(num_facets)
    compute_target_sunlit_facet_area.setNumArticulatedFacets(0)
    sim.AddModelToTask(task_name, compute_target_sunlit_facet_area)

    for i in range(num_facets):
        compute_servicer_sunlit_facet_area.addFacet(facet_area_list[i],
                                                    facetDcm_F0BList[i],
                                                    facetNHat_FList[i],
                                                    facetRotHat_FList[i],
                                                    facetR_CopB_BList[i],
                                                    facetDiffuseCoeffList[i],
                                                    facetSpecularCoeffList[i])
        compute_target_sunlit_facet_area.addFacet(facet_area_list[i],
                                                    facetDcm_F0BList[i],
                                                    facetNHat_FList[i],
                                                    facetRotHat_FList[i],
                                                    facetR_CopB_BList[i],
                                                    facetDiffuseCoeffList[i],
                                                    facetSpecularCoeffList[i])

    # Charging parameters
    beam_current = 250e-6  # [Amps] (Don't set above 1)
    beam_energy = 10000.0  # [eV]
    capacitance = 1e-9  # [farads]  1e-9 more realistic

    # Create the spacecraft charging module
    spacecraft_charging = spacecraftCharging.SpacecraftCharging()
    spacecraft_charging.ModelTag = "SpacecraftCharging"
    spacecraft_charging.setEBeamCurrent(beam_current)
    spacecraft_charging.setEBeamEnergy(beam_energy)
    spacecraft_charging.setServicerCapacitance(capacitance)
    spacecraft_charging.setTargetCapacitance(capacitance)
    spacecraft_charging.servicerSunlitAreaInMsg.subscribeTo(compute_servicer_sunlit_facet_area.scSunlitFacetAreaOutMsg)
    spacecraft_charging.targetSunlitAreaInMsg.subscribeTo(compute_target_sunlit_facet_area.scSunlitFacetAreaOutMsg)
    sim.AddModelToTask(task_name, spacecraft_charging)

    # Set up data logging
    data_rec_time_step = 0.001  # [s]
    servicer_sunlit_area_data_log = compute_servicer_sunlit_facet_area.scSunlitFacetAreaOutMsg.recorder(macros.sec2nano(data_rec_time_step))
    target_sunlit_area_data_log = compute_target_sunlit_facet_area.scSunlitFacetAreaOutMsg.recorder(macros.sec2nano(data_rec_time_step))
    servicer_potential_data_log = spacecraft_charging.servicerPotentialOutMsg.recorder(macros.sec2nano(data_rec_time_step))
    target_potential_data_log = spacecraft_charging.targetPotentialOutMsg.recorder(macros.sec2nano(data_rec_time_step))
    servicer_photoelectric_current_data_log = spacecraft_charging.servicerPhotoElectricCurrentOutMsg.recorder(macros.sec2nano(data_rec_time_step))
    target_photoelectric_current_data_log = spacecraft_charging.targetPhotoElectricCurrentOutMsg.recorder(macros.sec2nano(data_rec_time_step))
    sim.AddModelToTask(task_name, servicer_sunlit_area_data_log)
    sim.AddModelToTask(task_name, target_sunlit_area_data_log)
    sim.AddModelToTask(task_name, servicer_potential_data_log)
    sim.AddModelToTask(task_name, target_potential_data_log)
    sim.AddModelToTask(task_name, servicer_photoelectric_current_data_log)
    sim.AddModelToTask(task_name, target_photoelectric_current_data_log)

    # Add Vizard
    spacecraft_list = [servicer_spacecraft, target_spacecraft]
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(sim, task_name, spacecraft_list,
                                                  saveFile=filename
                                                  )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[servicer_spacecraft.ModelTag]
                                     , modelPath="CUBE"
                                     , scale=[width_hub, length_hub, depth_hub]
                                     , color=vizSupport.toRGBA255("gray"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[target_spacecraft.ModelTag]
                                     , modelPath="CUBE"
                                     , scale=[width_hub, length_hub, depth_hub]
                                     , color=vizSupport.toRGBA255("red"))

    # Run the simulation
    sim.InitializeSimulation()
    sim_time = 5.0  # [s]
    sim.ConfigureStopTime(macros.sec2nano(sim_time))
    sim.ExecuteSimulation()

    timespan = servicer_sunlit_area_data_log.times() * macros.NANO2SEC  # [s]
    servicer_sunlit_area = servicer_sunlit_area_data_log.area  # [m]
    target_sunlit_area = target_sunlit_area_data_log.area  # [m]
    servicer_potential_sim = servicer_potential_data_log.voltage  # [Volts]
    target_potential_sim = target_potential_data_log.voltage  # [Volts]
    servicer_photoelectric_current = servicer_photoelectric_current_data_log.current  # [Amps]
    target_photoelectric_current = target_photoelectric_current_data_log.current  # [Amps]

    plt.close("all")

    # Plot the servicer and target sunlit areas and photoelectric current
    fig1, ax1 = plt.subplots()
    ax1.plot(timespan*1000, servicer_sunlit_area, label=r"$A_{S}$", color="teal")
    ax1.plot(timespan*1000, target_sunlit_area, "--", label=r"$A_{T}$", color="teal")
    ax1.tick_params(axis="y", labelcolor="teal")
    ax1.set_xlabel("Time (ms)", fontsize=16)
    ax1.set_ylabel("Sunlit Area (m)", color="teal", fontsize=16)
    ax2 = ax1.twinx()
    ax2.plot(timespan*1000, servicer_photoelectric_current, label=r"$I_{ph,S}$", color="darkviolet")
    ax2.plot(timespan*1000, target_photoelectric_current, "--", label=r"$I_{ph,T}$", color="darkviolet")
    ax2.set_ylabel("Current (A)", color="darkviolet", fontsize=16)
    ax2.tick_params(axis="y", labelcolor="darkviolet")
    handles_ax1, labels_ax1 = ax1.get_legend_handles_labels()
    handles_ax2, labels_ax2 = ax2.get_legend_handles_labels()
    handles = handles_ax1 + handles_ax2
    labels = labels_ax1 + labels_ax2
    plt.legend(handles=handles, labels=labels, loc="upper left", prop={"size": 16})
    plt.grid(True)

    # plt.figure(1)
    # plt.clf()
    # plt.plot(timespan*1000, servicer_sunlit_area, label="Servicer")
    # plt.plot(timespan*1000, target_sunlit_area, label="Target")
    # plt.title(r'Servicer and Target Total Sunlit Facet Area', fontsize=16)
    # plt.ylabel('(m)', fontsize=16)
    # plt.xlabel('Time (ms)', fontsize=16)
    # plt.grid(True)
    # plt.legend()

    # Plot the servicer and target potentials
    beam_energy_plotting = np.ones(len(timespan)) * beam_energy
    plt.figure(2)
    plt.clf()
    plt.plot(timespan*1000, beam_energy_plotting, '--', label=r"$E_{\text{EB}}$ (keV)", color="teal")
    plt.plot(timespan*1000, servicer_potential_sim, label=r"$\phi_{\text{S, sim}}$", color="darkviolet")
    plt.plot(timespan*1000, target_potential_sim, label=r"$\phi_{\text{T, sim}}$", color="blue")
    plt.suptitle(r'Servicer and Target Spacecraft Potentials with Electron Beam', fontsize=16)
    plt.title(r'$C = 10^{-9} F, \ I_{\text{EB}} = 250e^{-6} A$', fontsize=14)
    plt.ylabel('(Volts)', fontsize=16)
    plt.xlabel('Time (ms)', fontsize=16)
    plt.legend(loc='lower right', prop={'size': 16})
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    # test_spacecraft_charging(
    #     True,  # show_plots
    # )
    test_spacecraft_charging_photoelectric_current(
        True,  # show_plots
    )


















# def charging_eom(t, servicer_potential, beam_current, beam_energy, capacitance):
#     # Set the beam current to zero if the sc potential is greater or equal to the beam energy
#     if (servicer_potential >= beam_energy):
#         beam_current = 0.0
#
#     servicer_potential_dot = beam_current / capacitance
#     return servicer_potential_dot
#
#
#     def test_spacecraft_charging(show_plots):
#         r"""
#         **Verification Test Description**
#
#         **Test Parameters**
#
#         Args:
#
#         **Description of Variables Being Tested**
#
#         """
#
#     task_name = "unitTask"
#     process_name = "TestProcess"
#     test_sim = SimulationBaseClass.SimBaseClass()
#     test_time_step_sec = 1e-6
#     test_process_rate = macros.sec2nano(test_time_step_sec)
#     test_process = test_sim.CreateNewProcess(process_name)
#     test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))
#
#     # Charging parameters
#     beam_current = 250e-6  # [Amps] (Don't set above 1)
#     beam_energy = 10000.0  # [eV]
#     capacitance = 1e-9  # [farads]  1e-9 more realistic
#
#     # Create the spacecraft charging module
#     spacecraft_charging = spacecraftCharging.SpacecraftCharging()
#     spacecraft_charging.ModelTag = "SpacecraftCharging"
#     spacecraft_charging.setEBeamCurrent(beam_current)
#     spacecraft_charging.setEBeamEnergy(beam_energy)
#     spacecraft_charging.setServicerCapacitance(capacitance)
#     spacecraft_charging.setTargetCapacitance(capacitance)
#     test_sim.AddModelToTask(task_name, spacecraft_charging)
#
#     # Set up data logging
#     servicer_potential_data_log = spacecraft_charging.servicerPotentialOutMsg.recorder()
#     target_potential_data_log = spacecraft_charging.targetPotentialOutMsg.recorder()
#     test_sim.AddModelToTask(task_name, servicer_potential_data_log)
#     test_sim.AddModelToTask(task_name, target_potential_data_log)
#
#     # Run the simulation
#     test_sim.InitializeSimulation()
#     sim_time = 0.06  # [s]
#     test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
#     test_sim.ExecuteSimulation()
#
#     # Extract the logged data for plotting and data comparison
#     timespan = macros.NANO2SEC * servicer_potential_data_log.times()  # [s]
#     servicer_potential_sim = servicer_potential_data_log.voltage  # [Volts]
#     target_potential_sim = target_potential_data_log.voltage  # [Volts]
#
#     # Compute the true servicer spacecraft potential for the unit test check
#     servicer_potential_truth = compute_servicer_potential_truth(timespan,
#                                                                 test_time_step_sec,
#                                                                 beam_current,
#                                                                 beam_energy,
#                                                                 capacitance)
#
#     if show_plots:
#         # Plot servicer spacecraft potential
#         beam_energy_plotting = np.ones(len(timespan)) * beam_energy
#         plt.figure(1)
#         plt.clf()
#         plt.plot(timespan*1000, beam_energy_plotting, '--', label=r"$E_{\text{EB}}$ (keV)", color="teal")
#         # plt.plot(timespan*1000, servicer_potential_truth, label=r"$\phi_{\text{S, truth}}$", color="teal")
#         plt.plot(timespan*1000, servicer_potential_sim, label=r"$\phi_{\text{S, sim}}$", color="darkviolet")
#         plt.plot(timespan*1000, target_potential_sim, label=r"$\phi_{\text{T, sim}}$", color="blue")
#         plt.suptitle(r'Servicer and Target Spacecraft Potentials with Electron Beam', fontsize=16)
#         plt.title(r'$C = 10^{-9} F, \ I_{\text{EB}} = 250e^{-6} A$', fontsize=14)
#         plt.ylabel('(Volts)', fontsize=16)
#         plt.xlabel('Time (ms)', fontsize=16)
#         plt.legend(loc='center right', prop={'size': 16})
#         plt.grid(True)
#
#         # Plot difference between truth and simulated sc potential
#         plt.figure(2)
#         plt.clf()
#         plt.plot(timespan*1000, np.abs(servicer_potential_truth - servicer_potential_sim), color="darkviolet")
#         plt.title(r'Difference Between Truth and Simulated Potentials', fontsize=16)
#         plt.ylabel('(Volts)', fontsize=16)
#         plt.xlabel('Time (ms)', fontsize=16)
#         plt.grid(True)
#         plt.show()
#
#     plt.close("all")
#
#     # Test check verification
#     np.testing.assert_allclose(servicer_potential_truth,
#                                servicer_potential_sim,
#                                atol=1e-3,
#                                verbose=True)
#
#
# def compute_servicer_potential_truth(timespan,
#                                      test_time_step_sec,
#                                      beam_current,
#                                      beam_energy,
#                                      capacitance):
#     servicer_potential_init = 0.0
#     servicer_potential = [servicer_potential_init]
#     for idx in range(len(timespan) - 1):
#         t = timespan[idx]
#         servicer_potential_current = servicer_potential[idx]
#
#         k_1 = charging_eom(t,
#                            servicer_potential_current,
#                            beam_current,
#                            beam_energy,
#                            capacitance)
#         k_2 = charging_eom(t + (test_time_step_sec/2),
#                            servicer_potential_current + ((test_time_step_sec * k_1) / 2),
#                            beam_current,
#                            beam_energy,
#                            capacitance)
#         k_3 = charging_eom(t + (test_time_step_sec/2),
#                            servicer_potential_current + ((test_time_step_sec * k_2) / 2),
#                            beam_current,
#                            beam_energy,
#                            capacitance)
#         k_4 = charging_eom(t + test_time_step_sec,
#                            servicer_potential_current + test_time_step_sec * k_3,
#                            beam_current,
#                            beam_energy,
#                            capacitance)
#
#
#         servicer_potential_next = servicer_potential_current + (test_time_step_sec / 6) * (k_1 + 2 * k_2 + 2 * k_3 + k_4)
#
#         if servicer_potential_next >= beam_energy:
#             servicer_potential_next = beam_energy
#
#         servicer_potential = np.append(servicer_potential, servicer_potential_next)
#
#     return servicer_potential
#
