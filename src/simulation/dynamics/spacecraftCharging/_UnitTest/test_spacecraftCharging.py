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
    test_time_step_sec = 0.001
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Charging parameters
    beam_current = 250e-6  # [Amps] (Don't set above 1)
    beam_energy = 10.0  # [keV]
    capacitance = 1e-10  # [farads]

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
    sim_time = 10.0  # [s]
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    test_sim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * spacecraft_potential_data_log.times()  # [s]
    sc_voltage = spacecraft_potential_data_log.voltage  # [Volts]

    if show_plots:
        # Plot spacecraft potential
        plt.figure()
        plt.clf()
        plt.plot(timespan, sc_voltage, label=r"$\phi$")
        plt.title(r'Spacecraft Potential', fontsize=14)
        plt.ylabel('(Volts)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    test_spacecraft_charging(
        True,  # show_plots
    )
