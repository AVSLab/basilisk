# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Illustrate automatic and custom effector state names across fresh simulations."""

import numpy as np

from Basilisk.simulation import hingedRigidBodyStateEffector, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros


def build_simulation(custom_names=False):
    """Build a spacecraft with two hinged panels and configure a short run.

    :param custom_names: Assign repeatable angle and angular-rate state names.
    :return: Simulation retaining its vehicle, panels, and message recorders.
    """
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("dynamics")
    time_step = 0.05  # [s]
    process.addTask(sim.CreateNewTask("dynamicsTask", macros.sec2nano(time_step)))

    vehicle = spacecraft.Spacecraft()
    vehicle.ModelTag = "vehicle"
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.hub.IHubPntBc_B = 100.0 * np.eye(3)  # [kg m^2]
    sim.AddModelToTask("dynamicsTask", vehicle)

    panels = []
    recorders = []
    for index, label in enumerate(("leftPanel", "rightPanel")):
        panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
        panel.ModelTag = label  # A diagnostic label; this does not rename states.
        if custom_names:
            panel.nameOfThetaState = f"{label}Angle"
            panel.nameOfThetaDotState = f"{label}Rate"

        panel.mass = 1.0  # [kg]
        panel.IPntS_S = np.eye(3)  # [kg m^2]
        panel.d = 0.5  # [m]
        panel.k = 0.2  # [N m/rad]
        panel.thetaInit = 0.1 * (index + 1)  # [rad]
        vehicle.addStateEffector(panel)
        sim.AddModelToTask("dynamicsTask", panel)

        # Record messages without depending on any generated state-name strings.
        recorder = panel.hingedRigidBodyOutMsg.recorder()
        sim.AddModelToTask("dynamicsTask", recorder)
        panels.append(panel)
        recorders.append(recorder)

    # Retain access to the models and recorders for the caller's state lookups.
    sim.vehicle = vehicle
    sim.panels = panels
    sim.recorders = recorders
    duration = 0.5  # [s]
    sim.ConfigureStopTime(macros.sec2nano(duration))
    return sim


def run_cases():
    """Run two builds per naming choice and print the actual state names.

    :return: Four dictionaries containing names and final state/message values.
        Each numerical array has one row per panel and columns for angle [rad]
        and angular rate [rad/s]. No simulation objects are retained.
    """
    results = []
    for custom_names in (False, True):
        naming = "custom" if custom_names else "automatic"
        print(f"\n{naming.capitalize()} state names: two fresh simulations")
        print(f"{'Build':<7}{'Panel':<12}{'Angle state':<32}Rate state")
        for case in range(1, 3):
            sim = build_simulation(custom_names)
            names_before = tuple(
                (panel.nameOfThetaState, panel.nameOfThetaDotState)
                for panel in sim.panels
            )
            sim.InitializeSimulation()
            sim.ExecuteSimulation()

            # Ask these panels for their names; do not guess numeric suffixes.
            names = tuple(
                (panel.nameOfThetaState, panel.nameOfThetaDotState)
                for panel in sim.panels
            )
            for index, (angle_name, rate_name) in enumerate(names):
                label = sim.panels[index].ModelTag
                print(f"{case:<7}{label:<12}{angle_name:<32}{rate_name}")

            final_states = np.array([
                [sim.vehicle.dynManager.getStateObject(name).getState()[0][0]
                 for name in panel_names]
                for panel_names in names
            ])  # Columns: angle [rad], angular rate [rad/s].
            message_states = np.array([
                [recorder.theta[-1], recorder.thetaDot[-1]]
                for recorder in sim.recorders
            ])  # Columns: angle [rad], angular rate [rad/s].
            print(f"       Left panel final angle: {final_states[0, 0]:.6f} rad "
                  f"(message: {message_states[0, 0]:.6f} rad)")
            results.append({
                "custom_names": custom_names,
                "names_before": names_before,
                "names": names,
                "states": final_states,
                "messages": message_states,
            })
            # Ending a BSK run or dropping its Python reference does not reset
            # the hinged-body module's static constructor counter.
            del sim
    return results


def run():
    """Run the tutorial and print its tables; no plots are produced."""
    run_cases()


if __name__ == "__main__":
    run()
