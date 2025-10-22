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

import matplotlib.pyplot as plt
from Basilisk.simulation import spacecraftCharging
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
import numpy as np

def test_spacecraft_charging(show_plots):
    r"""
    **Verification Test Description**

    **Test Parameters**

    Args:

    **Description of Variables Being Tested**

    """

    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 1e-6
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

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
    test_sim.AddModelToTask(task_name, spacecraft_charging)

    # Set up data logging
    servicer_potential_data_log = spacecraft_charging.servicerPotentialOutMsg.recorder()
    target_potential_data_log = spacecraft_charging.targetPotentialOutMsg.recorder()
    test_sim.AddModelToTask(task_name, servicer_potential_data_log)
    test_sim.AddModelToTask(task_name, target_potential_data_log)

    # Run the simulation
    test_sim.InitializeSimulation()
    sim_time = 0.06  # [s]
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    test_sim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * servicer_potential_data_log.times()  # [s]
    servicer_potential_sim = servicer_potential_data_log.voltage  # [Volts]
    target_potential_sim = target_potential_data_log.voltage  # [Volts]

    # Compute the true servicer spacecraft potential for the unit test check
    servicer_potential_truth = compute_servicer_potential_truth(timespan,
                                                                test_time_step_sec,
                                                                beam_current,
                                                                beam_energy,
                                                                capacitance)

    if show_plots:
        # Plot servicer spacecraft potential
        beam_energy_plotting = np.ones(len(timespan)) * beam_energy
        plt.figure(1)
        plt.clf()
        plt.plot(timespan*1000, beam_energy_plotting, '--', label=r"$E_{\text{EB}}$ (keV)", color="teal")
        # plt.plot(timespan*1000, servicer_potential_truth, label=r"$\phi_{\text{S, truth}}$", color="teal")
        plt.plot(timespan*1000, servicer_potential_sim, label=r"$\phi_{\text{S, sim}}$", color="darkviolet")
        plt.plot(timespan*1000, target_potential_sim, label=r"$\phi_{\text{T, sim}}$", color="blue")
        plt.suptitle(r'Servicer and Target Spacecraft Potentials with Electron Beam', fontsize=16)
        plt.title(r'$C = 10^{-9} F, \ I_{\text{EB}} = 250e^{-6} A$', fontsize=14)
        plt.ylabel('(Volts)', fontsize=16)
        plt.xlabel('Time (ms)', fontsize=16)
        plt.legend(loc='center right', prop={'size': 16})
        plt.grid(True)

        # Plot difference between truth and simulated sc potential
        plt.figure(2)
        plt.clf()
        plt.plot(timespan*1000, np.abs(servicer_potential_truth - servicer_potential_sim), color="darkviolet")
        plt.title(r'Difference Between Truth and Simulated Potentials', fontsize=16)
        plt.ylabel('(Volts)', fontsize=16)
        plt.xlabel('Time (ms)', fontsize=16)
        plt.grid(True)
        plt.show()

    plt.close("all")

    # Test check verification
    np.testing.assert_allclose(servicer_potential_truth,
                               servicer_potential_sim,
                               atol=1e-3,
                               verbose=True)


def compute_servicer_potential_truth(timespan,
                               test_time_step_sec,
                               beam_current,
                               beam_energy,
                               capacitance):
    servicer_potential_init = 0.0
    servicer_potential = [servicer_potential_init]
    for idx in range(len(timespan) - 1):
        t = timespan[idx]
        servicer_potential_current = servicer_potential[idx]

        k_1 = charging_eom(t,
                           servicer_potential_current,
                           beam_current,
                           beam_energy,
                           capacitance)
        k_2 = charging_eom(t + (test_time_step_sec/2),
                           servicer_potential_current + ((test_time_step_sec * k_1) / 2),
                           beam_current,
                           beam_energy,
                           capacitance)
        k_3 = charging_eom(t + (test_time_step_sec/2),
                           servicer_potential_current + ((test_time_step_sec * k_2) / 2),
                           beam_current,
                           beam_energy,
                           capacitance)
        k_4 = charging_eom(t + test_time_step_sec,
                           servicer_potential_current + test_time_step_sec * k_3,
                           beam_current,
                           beam_energy,
                           capacitance)


        servicer_potential_next = servicer_potential_current + (test_time_step_sec / 6) * (k_1 + 2 * k_2 + 2 * k_3 + k_4)

        if servicer_potential_next >= beam_energy:
            servicer_potential_next = beam_energy

        servicer_potential = np.append(servicer_potential, servicer_potential_next)



    return servicer_potential


def charging_eom(t, servicer_potential, beam_current, beam_energy, capacitance):
    # Set the beam current to zero if the sc potential is greater or equal to the beam energy
    if (servicer_potential >= beam_energy):
        beam_current = 0.0

    servicer_potential_dot = beam_current / capacitance
    return servicer_potential_dot


if __name__ == "__main__":
    test_spacecraft_charging(
        True,  # show_plots
    )
