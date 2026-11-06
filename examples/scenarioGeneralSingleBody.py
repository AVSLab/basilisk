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

r"""
Overview
--------
Three independent :ref:`generalSingleBodyStateEffector` instances move rectangular
blocks relative to a free spacecraft hub in a circular, 400 km altitude Mars orbit.
The simulation runs for 30 seconds with a 0.1-second time step. All three blocks
move simultaneously, illustrating three different joint configurations:

.. list-table:: Block motions
   :header-rows: 1
   :widths: 15 40 45

   * - Color
     - DOFs, in addition order
     - Constant references
   * - Blue
     - Translation along x
     - 1.4 m
   * - Orange
     - Rotation about y, then rotation about z
     - -40 deg, +30 deg
   * - Green
     - Rotation about z, translation along y, then rotation about x
     - +45 deg, 1.2 m, -35 deg

Each axis is expressed in its joint frame. In particular, the green block's first
rotation turns the direction of its subsequent translation. These are three
separate rigid bodies attached to the hub, not a chain of three bodies.
The blocks use solid-box inertias consistent with their displayed sizes. The hub's
solid-box inertia uses ``hub_dimensions``, independently of its default Vizard model.

All joint positions and rates start at zero. Constant position-reference messages
are written before initialization. The effector's spring/damper terms drive the
motion toward these references; motor inputs are unused.
Reference rates are zero. This is dynamic motion with hub reaction, not prescribed
kinematics or an attitude-controlled hub. The stiffness and damping deliberately
produce underdamped motion: each joint overshoots its reference and oscillates
with decreasing amplitude, approaching its target over 20--30 seconds. The mixed
block's changing geometry also changes its effective rotational inertia.
Point-mass Mars gravity is the only external force; SPICE kernels and external
data files are not needed.

Run the script from ``basilisk/examples`` with::

    python3 scenarioGeneralSingleBody.py

Implementation and Vizard
-------------------------
Configure each effector and its DOFs before spacecraft initialization. The order
of ``addRotDOF()`` and ``addTransDOF()`` calls defines the joint sequence. Reference
ports are indexed separately by rotational or translational DOF order, whereas
``getDOF()`` uses the combined sequence. Keep the reference-message objects alive.
Schedule the effectors before the spacecraft so their inputs affect that update.

Vizard receives the hub plus each effector's ``generalSingleBodyConfigLogOutMsg``.
These messages provide the block center-of-mass positions and body attitudes, so
the centered ``CUBE`` models need no extra geometry offsets. The hub uses Vizard's
default spacecraft model, and the three blocks match the plot colors. Joint
hardware is not modeled or drawn.

Binary export is disabled by default. Uncomment the ``saveFile`` argument in
``enableUnityVisualization()`` to make the script write
``examples/_VizFiles/scenarioGeneralSingleBody_UnityViz.bin``, independently of
the working directory. The script hides orbit lines and leaves the camera and
playback settings at Vizard's defaults. Open the file in Vizard and use its
interactive camera controls to frame ``marsHub`` and all three blocks. Play at
real time to inspect the oscillations. Plots also work in builds without
``vizInterface``; binary export requires that module.

Illustration of Simulation Results
----------------------------------
The first figure shows joint angles and displacements above their corresponding
rates. Dotted lines show the fixed position references. The responses overshoot
and oscillate with decreasing amplitude as the joints approach their references.
The path plot is expressed in the moving hub frame; circles and squares mark
the initial and final block centers. The orange block rotates about its center,
so its center stays at its mounting point. Hub angular rates show the coupled
reaction to all three mechanisms.

.. image:: /_images/Scenarios/scenarioGeneralSingleBodyJoints.svg
   :align: center
   :alt: Joint positions and rates for the three mechanisms.

.. image:: /_images/Scenarios/scenarioGeneralSingleBodyMotion.svg
   :align: center
   :alt: Block center paths in the hub frame and hub angular rates.

The shared ``test_scenarioTest.py`` runner saves these figures for the generated RST page.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging
from Basilisk.simulation import generalSingleBodyStateEffector, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import simIncludeGravBody, vizSupport


def box_inertia(mass, dimensions):
    """Return solid-box principal inertias [kg*m^2] from mass [kg] and full side lengths [m]."""
    x, y, z = dimensions
    return np.diag([y*y + z*z, x*x + z*z, x*x + y*y]) * mass / 12.0


def add_joint(effector, rotational, axis, target, stiffness, damping):
    """Add a zero-initialized joint and connect a constant position reference.

    :param effector: Owning general body effector.
    :param rotational: True for rotation, False for translation.
    :param axis: Unit axis in this joint's frame [-].
    :param target: Reference position [rad] or [m].
    :param stiffness: Spring constant [N*m/rad] or [N/m].
    :param damping: Damping constant [N*m*s/rad] or [N*s/m].
    :return: Reference message (retain its lifetime) and joint-state recorder.
    """
    joint = generalSingleBodyStateEffector.DOF()
    joint.setDOFAxis(axis)
    joint.setSpringConstantK(stiffness)
    joint.setDampingConstantC(damping)
    if rotational:
        effector.addRotDOF(joint)
        payload = messaging.HingedRigidBodyMsgPayload()
        payload.theta = target
        reference = messaging.HingedRigidBodyMsg().write(payload)
        effector.spinningBodyRefInMsg[-1].subscribeTo(reference)
        recorder = effector.spinningBodyOutMsgs[-1].recorder()
    else:
        effector.addTransDOF(joint)
        payload = messaging.LinearTranslationRigidBodyMsgPayload()
        payload.rho = target
        reference = messaging.LinearTranslationRigidBodyMsg().write(payload)
        effector.translatingBodyRefInMsgs[-1].subscribeTo(reference)
        recorder = effector.translatingBodyOutMsgs[-1].recorder()
    return reference, recorder


def run(show_plots):
    """Simulate three moving blocks in Mars orbit and create the tutorial plots.

    :param show_plots: Display the figures when True.
    :return: Dictionary of figures for the shared scenario test and documentation.
    """
    simulation = SimulationBaseClass.SimBaseClass()
    task_name = "dynamicsTask"
    time_step = 0.1  # [s]
    duration = 30.0  # [s]
    process = simulation.CreateNewProcess("dynamicsProcess")
    process.addTask(simulation.CreateNewTask(task_name, macros.sec2nano(time_step)))

    hub = spacecraft.Spacecraft()
    hub.ModelTag = "marsHub"
    hub_mass = 400.0  # [kg]
    hub_dimensions = [2.0, 1.6, 1.2]  # [m], full box side lengths
    hub.hub.mHub = hub_mass
    hub.hub.IHubPntBc_B = box_inertia(hub_mass, hub_dimensions)
    simulation.AddModelToTask(task_name, hub, 0)

    gravity = simIncludeGravBody.gravBodyFactory()
    mars = gravity.createMars()
    mars.isCentralBody = True
    gravity.addBodiesTo(hub)
    elements = orbitalMotion.ClassicElements()
    altitude = 400.0e3  # [m]
    elements.a = mars.radEquator + altitude
    elements.e = 0.0  # [-]
    elements.i = 25.0 * macros.D2R  # [rad]
    elements.Omega = 40.0 * macros.D2R  # [rad]
    elements.omega = 0.0  # [rad]
    elements.f = 0.0  # [rad]
    hub.hub.r_CN_NInit, hub.hub.v_CN_NInit = orbitalMotion.elem2rv(mars.mu, elements)

    # Name, color, mass [kg], full box dimensions [m], mounting position in B [m].
    specifications = [
        ("slider", "tab:blue", 12.0, [0.8, 0.5, 0.4], [1.4, 0.0, 0.0]),
        ("gimbal", "tab:orange", 10.0, [1.0, 0.35, 0.65], [-1.7, 0.0, 0.0]),
        ("mixed", "tab:green", 8.0, [0.8, 0.5, 0.45], [0.0, 1.6, 0.0]),
    ]
    blocks = []
    block_recorders = []
    for name, color, mass, dimensions, mount in specifications:
        block = generalSingleBodyStateEffector.GeneralSingleBodyStateEffector()
        block.ModelTag = name
        block.setMass(mass)
        block.setIPntGc_G(box_inertia(mass, dimensions))
        block.setR_G0B_B(mount)
        # Identity mounting attitude and zero COM offset are the defaults.
        hub.addStateEffector(block)
        simulation.AddModelToTask(task_name, block, 1)
        blocks.append(block)
        block_recorders.append(block.generalSingleBodyConfigLogOutMsg.recorder())

    # Block index, rotation flag, unit axis, target, spring constant, damping constant, label.
    # Rotational target/k/c use [rad, N*m/rad, N*m*s/rad]; translation uses [m, N/m, N*s/m].
    # Light damping makes the overshoot and free oscillations about each reference visible.
    joint_specs = [
        (0, False, [1, 0, 0], 1.4, 10.0, 4.5, "Slide x"),
        (1, True, [0, 1, 0], -40.0*macros.D2R, 1.5, 0.55, "Gimbal y"),
        (1, True, [0, 0, 1], 30.0*macros.D2R, 1.5, 0.5, "Gimbal z"),
        (2, True, [0, 0, 1], 45.0*macros.D2R, 4.0, 4.0, "Mixed z"),
        (2, False, [0, 1, 0], 1.2, 10.0, 4.0, "Mixed y slide"),
        (2, True, [1, 0, 0], -35.0*macros.D2R, 0.5, 0.2, "Mixed x"),
    ]
    references = []
    joint_recorders = []
    for index, rotational, axis, target, stiffness, damping, label in joint_specs:
        reference, recorder = add_joint(blocks[index], rotational, axis, target, stiffness, damping)
        references.append(reference)
        joint_recorders.append(recorder)

    hub_recorder = hub.scStateOutMsg.recorder()
    for recorder in [hub_recorder] + block_recorders + joint_recorders:
        simulation.AddModelToTask(task_name, recorder)

    if vizSupport.vizFound:
        viz_bodies = [hub] + [[block.ModelTag, block.generalSingleBodyConfigLogOutMsg] for block in blocks]
        viz = vizSupport.enableUnityVisualization(
            simulation, task_name, viz_bodies,
            # saveFile=str(Path(__file__).resolve()),
        )
        for name, color, mass, dimensions, mount in specifications:
            vizSupport.createCustomModel(viz, modelPath="CUBE", simBodiesToModify=[name],
                                         scale=dimensions, color=vizSupport.toRGBA255(color))
        viz.settings.orbitLinesOn = -1

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(duration))
    simulation.ExecuteSimulation()

    time_s = hub_recorder.times() * macros.NANO2SEC
    joints = []
    for spec, recorder in zip(joint_specs, joint_recorders):
        index, rotational, axis, target, stiffness, damping, label = spec
        joints.append({"label": label, "block": index, "rotational": rotational, "target": target,
                       "position": recorder.theta if rotational else recorder.rho,
                       "rate": recorder.thetaDot if rotational else recorder.rhoDot})
    dcm_BN = np.array([rbk.MRP2C(sigma) for sigma in hub_recorder.sigma_BN])
    positions_B = [np.einsum("nij,nj->ni", dcm_BN, record.r_BN_N - hub_recorder.r_BN_N)
                   for record in block_recorders]
    joint_figure, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True, layout="constrained")
    line_styles = ["-", "-", "--", "-", "-", "--"]
    for joint, style in zip(joints, line_styles):
        column = 0 if joint["rotational"] else 1
        factor = macros.R2D if joint["rotational"] else 1.0
        color = specifications[joint["block"]][1]
        axes[0, column].plot(time_s, factor*joint["position"], style, color=color, label=joint["label"])
        axes[0, column].axhline(factor*joint["target"], color=color, linestyle=":", alpha=0.6)
        axes[1, column].plot(time_s, factor*joint["rate"], style, color=color, label=joint["label"])
    for axis, ylabel in zip(axes.flat, ["Angle [deg]", "Displacement [m]", "Angular rate [deg/s]", "Rate [m/s]"]):
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize="small")
    for axis in axes[1]:
        axis.set_xlabel("Time [s]")
    joint_figure.suptitle("Three general rigid bodies: joint responses in Mars orbit")

    motion_figure, motion_axes = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
    for spec, position in zip(specifications, positions_B):
        name, color = spec[:2]
        motion_axes[0].plot(position[:, 0], position[:, 1], color=color, label=name)
        motion_axes[0].plot(*position[0, :2], "o", color=color)
        motion_axes[0].plot(*position[-1, :2], "s", color=color)
    motion_axes[0].add_patch(plt.Rectangle((-hub_dimensions[0]/2, -hub_dimensions[1]/2),
                                          hub_dimensions[0], hub_dimensions[1], color="gray", alpha=0.3))
    motion_axes[0].set(xlabel="Hub-frame x [m]", ylabel="Hub-frame y [m]", title="Block center paths (x-y projection)")
    motion_axes[0].set_aspect("equal", adjustable="datalim")
    for index, label in enumerate(["x", "y", "z"]):
        motion_axes[1].plot(time_s, macros.R2D*hub_recorder.omega_BN_B[:, index], label=label)
    motion_axes[1].set(xlabel="Time [s]", ylabel="Hub angular rate [deg/s]", title="Hub reaction")
    for axis in motion_axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    figures = {"scenarioGeneralSingleBodyJoints": joint_figure, "scenarioGeneralSingleBodyMotion": motion_figure}
    if show_plots:
        plt.show()
    for figure in figures.values():
        plt.close(figure)
    return figures


if __name__ == "__main__":
    run(True)
