#  ISC License
#
#  Copyright (c) 2025,
#  Autonomous Vehicle Systems Laboratory, University of Colorado at Boulder
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
import inspect
import math

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.architecture import messaging
from Basilisk.simulation import scCharging


def run_two_sc_case(scale_e=1.0, scale_i=1.0, show_plots=False, label="baseline"):
    """
    Run ScCharging with the canonical two-spacecraft configuration
    (target + servicer with e-gun) and return the equilibrium voltages.
    """
    # -------------------------------------------------------------------------
    # Locate Support/ files
    # -------------------------------------------------------------------------
    filename = inspect.getframeinfo(inspect.currentframe()).filename
    this_dir = os.path.dirname(os.path.abspath(filename))
    support_dir = os.path.join(this_dir, "Support")

    def support(name: str) -> str:
        return os.path.join(support_dir, name)

    energies = np.loadtxt(support("particleEnergies.txt"), delimiter=",")
    electron_flux = np.loadtxt(support("electronFlux.txt"), delimiter=",")
    ion_flux = np.loadtxt(support("ionFlux.txt"), delimiter=",")
    yield_see_electron = np.loadtxt(support("yieldSEEelectron.txt"), delimiter=",")
    yield_see_ion = np.loadtxt(support("yieldSEEion.txt"), delimiter=",")
    yield_backscatter = np.loadtxt(support("yieldBackscattered.txt"), delimiter=",")

    # -------------------------------------------------------------------------
    # Build Basilisk simulation
    # -------------------------------------------------------------------------
    sim = SimulationBaseClass.SimBaseClass()
    proc_name = "chargeProc"
    task_name = "unitTask"
    dt = 0.5  # seconds

    proc = sim.CreateNewProcess(proc_name)
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    module = scCharging.ScCharging()
    module.ModelTag = f"ScCharging_{label}"
    sim.AddModelToTask(task_name, module)

    # -------------------------------------------------------------------------
    # Two-spacecraft geometry (same as original unit test)
    # -------------------------------------------------------------------------
    LT = 12.0
    angle = LT * 360.0 / 24.0 * math.pi / 180.0 - math.pi
    orbit_radius = 42000.0e3

    r_target = np.array([
        orbit_radius * math.cos(angle),
        orbit_radius * math.sin(angle),
        0.0
    ])
    r_servicer_offset = np.array([0.0, 10.0, 0.0])
    r_servicer = r_target + r_servicer_offset

    # Servicer state (sc0)
    sc0_state = messaging.SCStatesMsgPayload()
    sc0_state.r_BN_N = r_servicer
    sc0_msg = messaging.SCStatesMsg().write(sc0_state)

    # Target state (sc1)
    sc1_state = messaging.SCStatesMsgPayload()
    sc1_state.r_BN_N = r_target
    sc1_msg = messaging.SCStatesMsg().write(sc1_state)

    # Adding servicer first, then target
    module.addSpacecraft(sc0_msg)  # servicer (index 0)
    module.addSpacecraft(sc1_msg)  # target   (index 1)

    # Setting Beam parameters for sc0:
    beam_payload = messaging.ElectronBeamMsgPayload()
    beam_payload.energyEB = 33000.0
    beam_payload.currentEB = 0.0000045 # 4.5 microAmps
    beam_payload.alphaEB = 1.1

    beam_msg = messaging.ElectronBeamMsg().write(beam_payload)
    # Subscribe the servicer (index 0) to the beam message
    module.eBeamInMsgs[0].subscribeTo(beam_msg)

    # Recorders for both spacecraft voltages
    v_rec0 = module.voltOutMsgs[0].recorder()  # servicer
    v_rec1 = module.voltOutMsgs[1].recorder()  # target
    sim.AddModelToTask(task_name, v_rec0)
    sim.AddModelToTask(task_name, v_rec1)

    # Plasma flux
    plasma_payload = messaging.PlasmaFluxMsgPayload()
    plasma_payload.energies = energies
    plasma_payload.meanElectronFlux = electron_flux * scale_e
    plasma_payload.meanIonFlux = ion_flux * scale_i
    plasma_msg = messaging.PlasmaFluxMsg().write(plasma_payload)
    module.plasmaFluxInMsg.subscribeTo(plasma_msg)

    # Yield tables
    module.yieldSEEelectron = yield_see_electron
    module.yieldSEEion = yield_see_ion
    module.yieldBackscattered = yield_backscatter

    # -------------------------------------------------------------------------
    # Run simulation
    # -------------------------------------------------------------------------
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(0.5))
    sim.ExecuteSimulation()

    t = v_rec0.times() * macros.NANO2SEC
    v_servicer = np.copy(v_rec0.voltage)
    v_target   = np.copy(v_rec1.voltage)

    phi_servicer = v_servicer[-1] if len(v_servicer) > 0 else np.nan
    phi_target = v_target[-1] if len(v_target) > 0 else np.nan

    print(f"[{label}] target   equilibrium: {phi_target:.6f} V")
    print(f"[{label}] servicer equilibrium: {phi_servicer:.6f} V")

    if show_plots:
        plt.figure()
        plt.plot(t, v_target, label="target")
        plt.plot(t, v_servicer, label="servicer")
        plt.xlabel("Time [s]")
        plt.ylabel("Potential [V]")
        plt.title(f"ScCharging equilibrium ({label})")
        plt.grid(True)
        plt.legend()
        plt.show()

    return phi_target, phi_servicer


def test_spacecraft_charging_single_sc(show_plots=True):
    """
    Basic sanity: equilibrium potential should move negative with more
    electron flux and positive with more ion flux.
    """

    print("=== Two-spacecraft ScCharging trend tests ===")

    # 1) Baseline
    phi_t_base, phi_s_base = run_two_sc_case( 1, 1, show_plots=True, label="baseline")

    print(f"basecase: \nTarget: {phi_t_base} \n Servicer: {phi_s_base}")


    # 2) Electron flux scaling
    phi_t_e_more, phi_s_e_more = run_two_sc_case(2.0, 1.0, show_plots=False,
                                                 label="e_flux_x2")
    print(f"double E-flux: \nTarget: {phi_t_e_more} \n Servicer: {phi_s_e_more}")
    phi_t_e_less, phi_s_e_less = run_two_sc_case(0.5, 1.0, show_plots=False,
                                                 label="e_flux_x0p5")
    print(f"half E-flux: \nTarget: {phi_t_e_less} \n Servicer: {phi_s_e_less}")

    print("\nElectron-flux trend (target):",
          phi_t_e_more, "<", phi_t_base, "<", phi_t_e_less)


    # Check trend for target (pick one SC to focus on)
    assert phi_t_e_more < phi_t_base < phi_t_e_less, \
        "Target potential should move more negative with more electron flux."

    # 3) Ion flux scaling
    phi_t_i_more, phi_s_i_more = run_two_sc_case(1.0, 2.0, show_plots=False,
                                                 label="i_flux_x2")
    phi_t_i_less, phi_s_i_less = run_two_sc_case(1.0, 0.5, show_plots=False,
                                                 label="i_flux_x0p5")

    print("Ion-flux trend (target):",
          phi_t_i_less, "<", phi_t_base, "<", phi_t_i_more)

    assert phi_t_i_less < phi_t_base < phi_t_i_more, \
        "Target potential should move more positive with more ion flux."

if __name__ == "__main__":
    # Set show_plots=True if you want a time history plot for the baseline case
    test_spacecraft_charging_single_sc(show_plots=True)
