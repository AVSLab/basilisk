# ISC License
#
# Copyright (c) 2026, Suhas Beemineni
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
Demonstrate two independently moving instrument cameras on a spacecraft with
a prescribed, fixed attitude. This implements the moving-platform approach
suggested in `issue 1309 <https://github.com/AVSLab/basilisk/issues/1309>`_.
Rotation and translation commands enter native motion profilers through
messages. No per-step assignment to ``camera.sigma_CB`` or ``cameraPos_B``
is needed, and no new camera input port is introduced.

Each camera is fixed to a :ref:`prescribedMotionStateEffector` platform P.
The platform's configuration output supplies its inertial pose to Vizard.
Its visualization name becomes the camera's ``parentName``. Consequently,
``cameraPos_B`` and ``sigma_CB`` in the camera configuration are interpreted
relative to P, even though the payload retains its B-frame field names.
Set ``r_PcP_P`` to zero so the platform's logged center of mass and camera
parent-frame origin coincide.

The platforms are massless kinematic frames, not a model of actuator torques
or hardware inertia. The spacecraft attitude is prescribed using an
``AttRefMsg``. The first platform rotates about its local y axis and translates
along x; the second rotates about x with a differently oriented mount. At
15 seconds, new commands change both rotations and the first translation.
This illustrates independent pose commands, not closed-loop target tracking.

Run from the repository root::

    python examples/scenarioMovingCamera.py

The native dynamics and numerical results also run without ``vizInterface``.
With that feature available, the scenario registers the platform poses and
two camera configurations for Vizard. Uncomment ``saveFile`` in the source
to export a replay. Rendering images requires Vizard; the offline tests check
native pose messages and camera-parent wiring, not rendered pixels.
"""

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging
from Basilisk.simulation import (
    prescribedLinearTranslation,
    prescribedMotionStateEffector,
    prescribedRotation1DOF,
    spacecraft,
)
from Basilisk.utilities import SimulationBaseClass, macros, vizSupport


def run(show_plots=False, enable_viz=True):
    """Execute two message-driven camera-platform maneuvers.

    :param show_plots: Display platform position and attitude histories.
    :param enable_viz: Register instrument cameras when visualization is built.
    :return: Dictionary of recorded hub/platform states, profiler references,
        command values, camera-parent names, and visualization message buffers.
    """
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("cameraProcess")
    step_s = 0.1  # [s]
    process.addTask(sim.CreateNewTask("cameraTask", macros.sec2nano(step_s)))

    hub = spacecraft.Spacecraft()
    hub.ModelTag = "cameraHost"
    hub.hub.mHub = 100.0  # [kg]
    hub.hub.IHubPntBc_B = np.diag([20.0, 30.0, 40.0]).tolist()  # [kg*m^2]
    hub.hub.r_CN_NInit = [100.0, -200.0, 300.0]  # [m]
    hub_sigma = [0.1, -0.2, 0.05]  # [-] MRP
    hub.hub.sigma_BNInit = hub_sigma
    attitude = messaging.AttRefMsgPayload()
    attitude.sigma_RN = hub_sigma
    attitude_msg = messaging.AttRefMsg().write(attitude)
    hub.attRefInMsg.subscribeTo(attitude_msg)

    platforms = []
    rotations = []
    rotation_msgs = []
    # Offsets are relative to hub B; rotation axes are relative to mount M.
    offsets = [[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]  # [m]
    axes = [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]  # [-]
    mount_sigmas = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]]  # [-] MRP
    commands_rad = np.deg2rad([[20.0, -30.0], [-10.0, 40.0]])  # [rad]
    translations_m = [0.3, -0.2]  # [m]
    for index in range(2):
        platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()
        platform.ModelTag = f"cameraPlatform{index + 1}"
        platform.setMass(0.0)  # [kg] Kinematic camera frame
        platform.setIPntPc_P(np.zeros((3, 3)).tolist())  # [kg*m^2]
        platform.setR_PcP_P([0.0, 0.0, 0.0])  # [m]
        platform.setR_MB_B(offsets[index])
        platform.setSigma_MB(mount_sigmas[index])
        hub.addStateEffector(platform)
        platforms.append(platform)

        rotation = prescribedRotation1DOF.PrescribedRotation1DOF()
        rotation.ModelTag = f"cameraRotation{index + 1}"
        rotation.setRotHat_M(axes[index])
        rotation.setThetaDDotMax(0.1)  # [rad/s^2]
        reference = messaging.HingedRigidBodyMsgPayload()
        reference.theta = commands_rad[0, index]
        reference_msg = messaging.HingedRigidBodyMsg().write(reference)
        rotation.spinningBodyInMsg.subscribeTo(reference_msg)
        platform.prescribedRotationInMsg.subscribeTo(rotation.prescribedRotationOutMsg)
        sim.AddModelToTask("cameraTask", rotation, 30)
        rotations.append(rotation)
        rotation_msgs.append(reference_msg)

    translation = prescribedLinearTranslation.PrescribedLinearTranslation()
    translation.ModelTag = "cameraTranslation1"
    translation.setTransHat_M([1.0, 0.0, 0.0])  # [-]
    translation.setTransAccelMax(0.05)  # [m/s^2]
    translation_reference = messaging.LinearTranslationRigidBodyMsgPayload()
    translation_reference.rho = translations_m[0]
    translation_msg = messaging.LinearTranslationRigidBodyMsg().write(translation_reference)
    translation.linearTranslationRigidBodyInMsg.subscribeTo(translation_msg)
    platforms[0].prescribedTranslationInMsg.subscribeTo(translation.prescribedTranslationOutMsg)
    sim.AddModelToTask("cameraTask", translation, 30)
    sim.AddModelToTask("cameraTask", hub, 20)
    for platform in platforms:
        sim.AddModelToTask("cameraTask", platform, 10)

    hub_log = hub.scStateOutMsg.recorder()
    pose_logs = [platform.prescribedMotionConfigLogOutMsg.recorder() for platform in platforms]
    rotation_logs = [rotation.prescribedRotationOutMsg.recorder() for rotation in rotations]
    translation_log = translation.prescribedTranslationOutMsg.recorder()
    for recorder in [hub_log, *pose_logs, *rotation_logs, translation_log]:
        sim.AddModelToTask("cameraTask", recorder, -10)

    viz = None
    camera_msgs = []
    camera_configs = []
    if enable_viz and vizSupport.vizFound:
        bodies = [hub] + [
            [platform.ModelTag, platform.prescribedMotionConfigLogOutMsg] for platform in platforms
        ]
        viz = vizSupport.enableUnityVisualization(
            sim, "cameraTask", bodies,
            # saveFile=__file__,
        )
        for index, platform in enumerate(platforms):
            camera_config, camera_msg = vizSupport.createCameraConfigMsg(
                viz, cameraID=index + 1, parentName=platform.ModelTag,
                fieldOfView=0.2,  # [rad]
                resolution=[256, 256],
                cameraPos_B=[0.0, 0.0, 0.0],  # [m] Relative to P
                sigma_CB=[0.0, 0.0, 0.0],  # [-] Camera aligned with P
                renderRate=step_s,  # [s]
            )
            camera_msgs.append(camera_msg)
            camera_configs.append(camera_config)

    switch_time_s = 15.0  # [s]
    final_time_s = 30.0  # [s]
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(switch_time_s))
    sim.ExecuteSimulation()
    # Retarget by writing command messages; never mutate camera pose properties.
    for index, reference_msg in enumerate(rotation_msgs):
        reference = messaging.HingedRigidBodyMsgPayload()
        reference.theta = commands_rad[1, index]
        reference_msg.write(reference, macros.sec2nano(switch_time_s))
    translation_reference.rho = translations_m[1]
    translation_msg.write(translation_reference, macros.sec2nano(switch_time_s))
    sim.ConfigureStopTime(macros.sec2nano(final_time_s))
    sim.ExecuteSimulation()

    # Retain SWIG vector wrappers while reading their element views.
    viz_data = [] if viz is None else viz.scData
    camera_buffers = [] if viz is None else viz.cameraConfigBuffers
    results = {
        "time_s": hub_log.times()*macros.NANO2SEC,
        "hub_sigma": hub_log.sigma_BN.copy(),
        "hub_position": hub_log.r_BN_N.copy(),
        "platform_sigma": [log.sigma_BN.copy() for log in pose_logs],
        "platform_position": [log.r_BN_N.copy() for log in pose_logs],
        "rotation_reference": [log.sigma_PM.copy() for log in rotation_logs],
        "translation_reference": translation_log.r_PM_M.copy(),
        "offsets": np.array(offsets),
        "axes": np.array(axes),
        "mount_sigmas": np.array(mount_sigmas),
        "commands_rad": commands_rad,
        "translations_m": translations_m,
        "switch_time_s": switch_time_s,
        "camera_parents": [config.parentName for config in camera_configs],
        "viz_parents": [data.parentSpacecraftName for data in viz_data],
        "viz_camera_parents": [data.parentName for data in camera_buffers],
        "viz_platform_sigma": [np.array(viz_data[index].scStateMsgBuffer.sigma_BN) for index in range(1, len(viz_data))],
    }
    if show_plots:
        _, axes_plot = plt.subplots(2, 1, sharex=True)
        for index in range(2):
            axes_plot[0].plot(results["time_s"], results["platform_sigma"][index], label=f"platform {index + 1}")
        axes_plot[0].set_ylabel("Inertial attitude (MRP)")
        axes_plot[1].plot(results["time_s"], results["translation_reference"][:, 0])
        axes_plot[1].set_ylabel("Platform 1 translation (m)")
        axes_plot[1].set_xlabel("Time (s)")
        plt.show()
    return results


if __name__ == "__main__":
    run(show_plots=True)
