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
    spacecraft_charging.setScCapacitance(capacitance)
    test_sim.AddModelToTask(task_name, spacecraft_charging)

    # Set up data logging
    spacecraft_potential_data_log = spacecraft_charging.scPotentialOutMsg.recorder()
    test_sim.AddModelToTask(task_name, spacecraft_potential_data_log)

    # Run the simulation
    test_sim.InitializeSimulation()
    sim_time = 0.06  # [s]
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    test_sim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * spacecraft_potential_data_log.times()  # [s]
    sc_voltage = spacecraft_potential_data_log.voltage  # [Volts]

    sc_potential_init = 0.0
    x = [sc_potential_init]
    for idx in range(len(timespan) - 1):
        t = timespan[idx]
        x_current = x[idx]

        k_1 = charging_eom(t, x_current, beam_current, beam_energy, capacitance)
        k_2 = charging_eom(t + (test_time_step_sec/2), x_current + ((test_time_step_sec * k_1) / 2), beam_current, beam_energy, capacitance)
        k_3 = charging_eom(t + (test_time_step_sec/2), x_current + ((test_time_step_sec * k_2) / 2), beam_current, beam_energy, capacitance)
        k_4 = charging_eom(t + test_time_step_sec, x_current + test_time_step_sec * k_3, beam_current, beam_energy, capacitance)

        x = np.append(x, x_current + (test_time_step_sec / 6) * (k_1 + 2 * k_2 + 2 * k_3 + k_4))

    if show_plots:
        # Plot spacecraft potential
        beam_energy_plotting = np.ones(len(timespan)) * beam_energy
        plt.figure()
        plt.clf()
        plt.plot(timespan*1000, beam_energy_plotting, '--', label=r"$E_{\text{EB}}$ (keV)", color="blue")
        plt.plot(timespan*1000, x, label=r"$\phi_{\text{truth}}$", color="teal")
        plt.plot(timespan*1000, sc_voltage, label=r"$\phi_{\text{sim}}$", color="darkviolet")
        plt.suptitle(r'Single Spacecraft with Electron Beam', fontsize=16)
        plt.title(r'$C = 10^{-9} F, \ I_{\text{EB}} = 250e^{-6} A$', fontsize=14)
        plt.ylabel('(Volts)', fontsize=16)
        plt.xlabel('Time (ms)', fontsize=16)
        plt.legend(loc='center right', prop={'size': 16})
        plt.grid(True)
        plt.show()

    plt.close("all")

def charging_eom(t, x, beam_current, beam_energy, capacitance):
    x_dot = 0.0
    if (beam_energy > x):
        x_dot = beam_current / capacitance

    return x_dot

if __name__ == "__main__":
    test_spacecraft_charging(
        True,  # show_plots
    )
