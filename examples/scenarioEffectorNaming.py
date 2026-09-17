# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

r"""
Demonstrate automatic and custom effector names using a spacecraft with two
spring-loaded hinged panels. The panels start at small angular displacements and
move under their spring torques. No gravity or external forces are configured.

Several kinds of names appear in a Basilisk script:

* ``first`` is a Python variable referring to the first panel object.
* ``first.ModelTag`` is the module label used in diagnostics, here ``"panel1"``.
* ``first.nameOfThetaState`` identifies the panel's integrated angle state in
  ``vehicle.dynManager``, the spacecraft's dynamics state manager. The panel also
  has an angular-rate state and inertial properties with their own names.

The naming policy controls these state and property names. Names must be unique
within the relevant state or property namespace in one manager; independent
spacecraft can use the same names. The flag does not change Python variables,
``ModelTag`` values, or message attributes.

From the ``examples`` folder, run the script with::

    python3 scenarioEffectorNaming.py

The call at the bottom of this file explicitly selects manager-local automatic
names. Basilisk and ``run()`` still default to legacy naming. To compare all four
configurations, start Python in the same folder and try these calls, closing each
plot window to continue:

.. code-block:: python

    from scenarioEffectorNaming import run

    # Legacy naming: constructor counters determine automatic panel names.
    run(show_plots=True, useManagerLocalEffectorNames=False, customNames=False)
    run(show_plots=True, useManagerLocalEffectorNames=False, customNames=True)

    # Manager-local naming: attachment order determines automatic panel names.
    run(show_plots=True, useManagerLocalEffectorNames=True, customNames=False)
    run(show_plots=True, useManagerLocalEffectorNames=True, customNames=True)

Set ``show_plots=False`` to print the tables without opening plot windows.
``customNames=True`` gives only the first panel's angle state the explicit name
``"leftPanelAngle"``. Its other names remain automatic. Manager-local generated
names become final during ``InitializeSimulation()``; read them from the effector
after that call. A callback configured earlier must read the name when it runs,
as the named logging function below does.

Each run prints the selected policy and both panels' angle-state names before and
after initialization, identifying automatic and custom names. The table lists
panels in attachment order; the second panel was constructed first.
Legacy suffixes reflect constructor counters and can depend on earlier panel
creations in the same Python session. In this example, manager-local allocation
uses group indices 1 and 2 in attachment order. Customizing the first panel's
angle name still leaves its other automatic names in group 1, so the second
panel's angle name ends in 2. Read the names from the objects instead of assuming
particular suffixes in your own scripts.

The panel message recorder can be configured before initialization and does not
depend on state names. Prefer such recorders when the desired output is available.
The figure overlays the message recorder as a line and the state logger as
occasional markers. They agree because both report the same panel angle. Changing
the naming policy or assigning a custom name leaves the physical motion unchanged.
See :ref:`effectorNaming` for the preparation sequence and migration details.
"""

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.simulation import hingedRigidBodyStateEffector, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, pythonVariableLogger


def run(show_plots=False, useManagerLocalEffectorNames=False, customNames=False):
    """Run a two-panel spacecraft and compare state logging with message recording.

    :param show_plots: Display the panel-angle plot.
    :param useManagerLocalEffectorNames: Opt into manager-local automatic names;
        False retains the legacy default.
    :param customNames: Give only the first panel's angle state a custom name.
    :return: Final angle-state names for both panels in attachment order, the
        first panel's logged and recorded angles [rad], and a figure dictionary.
    """
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("dynamics")
    step = 0.01  # [s]
    # A task schedules its models at this interval; Basilisk times use nanoseconds.
    process.addTask(simulation.CreateNewTask("dynamicsTask", macros.sec2nano(step)))
    vehicle = spacecraft.Spacecraft()
    # Select the policy before initialization registers any states or properties.
    vehicle.dynManager.useManagerLocalEffectorNames = useManagerLocalEffectorNames
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.hub.IHubPntBc_B = 100.0 * np.eye(3)  # [kg m^2]
    simulation.AddModelToTask("dynamicsTask", vehicle)

    # Constructor order is deliberately different from attachment order.
    second = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    first = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panels = [first, second]
    if customNames:
        first.nameOfThetaState = "leftPanelAngle"
    for index, panel in enumerate(panels):
        panel.ModelTag = f"panel{index + 1}"
        panel.mass = 1.0  # [kg]
        panel.IPntS_S = np.eye(3)  # [kg m^2]
        panel.d = 0.5  # [m]
        panel.k = 0.2  # [N m/rad]
        panel.thetaInit = 0.1 * (index + 1)  # [rad]
        # Include the panel's states and mass properties in spacecraft dynamics.
        vehicle.addStateEffector(panel)
        # Schedule the panel's input-message handling and output-message updates.
        simulation.AddModelToTask("dynamicsTask", panel)

    # The recorder reads an output message without looking up any state names.
    recorder = first.hingedRigidBodyOutMsg.recorder()
    simulation.AddModelToTask("dynamicsTask", recorder)

    def readPanelAngle(_currentTimeNanos):
        """Read the angle [rad] when the logger calls this function during simulation."""
        # The logger supplies simulation time [ns], which this lookup does not need.
        # Read the finalized name here, after initialization, on each logging call.
        stateName = first.nameOfThetaState
        angleState = vehicle.dynManager.getStateObject(stateName)
        # A scalar integrated state is stored as a 1-by-1 matrix.
        return angleState.getState()[0][0]

    stateLog = pythonVariableLogger.PythonVariableLogger({"angle": readPanelAngle})
    simulation.AddModelToTask("dynamicsTask", stateLog)

    # These setup-time snapshots are only for display, never for state lookups.
    namesBeforeInit = [panel.nameOfThetaState for panel in panels]
    simulation.InitializeSimulation()
    # Generated names are now final and can safely be copied for later lookups.
    names = [panel.nameOfThetaState for panel in panels]
    policy = "manager-local" if useManagerLocalEffectorNames else "legacy"
    print(f"\nEffector angle-state names (policy: {policy})")
    print("Panels are listed in attachment order; panel 2 was constructed first.")
    print(f"{'Panel':<8}{'Naming':<12}{'Before initialization':<32}After initialization")
    for index, (before, after) in enumerate(zip(namesBeforeInit, names), start=1):
        naming = "custom" if customNames and index == 1 else "automatic"
        print(f"{index:<8}{naming:<12}{before:<32}{after}")
    if useManagerLocalEffectorNames:
        print("Automatic names are finalized during initialization in attachment order.")
        if customNames:
            print("Panel 1's other automatic names keep group index 1, so panel 2 still uses index 2.")
    else:
        print("Legacy automatic names retain constructor counters, which can reflect earlier panel creations.")

    duration = 1.0  # [s]
    simulation.ConfigureStopTime(macros.sec2nano(duration))
    simulation.ExecuteSimulation()

    angles = np.asarray(stateLog.angle).copy()  # [rad]
    recordedAngles = np.asarray(recorder.theta).copy()  # [rad]
    figure, axes = plt.subplots()
    axes.plot(recorder.times() * macros.NANO2SEC, recordedAngles, label="Message recorder")
    axes.plot(stateLog.times() * macros.NANO2SEC, angles, linestyle="none",
              marker="o", markerfacecolor="none", markevery=10, label="State logger")
    axes.set_title(f"First panel ({policy} naming)")
    axes.set_xlabel("Time [s]")
    axes.set_ylabel("First panel angle [rad]")
    axes.legend()
    figure.tight_layout()
    if show_plots:
        plt.show()
    plt.close(figure)
    return names, angles, recordedAngles, {"scenarioEffectorNaming": figure}


if __name__ == "__main__":
    run(True, useManagerLocalEffectorNames=True, customNames=False)
