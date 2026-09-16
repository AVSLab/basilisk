#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from Basilisk.architecture import messaging
from Basilisk.simulation import jointArrayRefProfiler
from Basilisk.utilities import macros


JOINT_ANGLES = np.array([0.1, 0.2, -0.3])  # [rad]
JOINT_RATES = np.array([0.05, -0.07, 0.04])  # [rad/s]
DES_JOINT_ANGLES = np.array([1.0, -1.0, 1.5])  # [rad]
DES_JOINT_RATES = np.zeros(3)  # [rad/s]
DES_JOINT_ACCELS = np.zeros(3)  # [rad/s^2]


def setup_joint_array_ref_profiler(profile_type: str):
    """Create a jointArrayRefProfiler module with three joint inputs."""
    module = jointArrayRefProfiler.JointArrayRefProfiler()
    module.setProfileType(profile_type)
    joint_state_in_msgs = []
    joint_state_dot_in_msgs = []

    for i in range(3):
        module.addHingedJoint()

        joint_state_in = messaging.ScalarJointStateMsgPayload()
        joint_state_in.state = JOINT_ANGLES[i]
        joint_state_in_msg = messaging.ScalarJointStateMsg().write(joint_state_in)
        module.jointStatesInMsgs[i].subscribeTo(joint_state_in_msg)
        joint_state_in_msgs.append(joint_state_in_msg)

        joint_state_dot_in = messaging.ScalarJointStateMsgPayload()
        joint_state_dot_in.state = JOINT_RATES[i]
        joint_state_dot_in_msg = messaging.ScalarJointStateMsg().write(joint_state_dot_in)
        module.jointStateDotsInMsgs[i].subscribeTo(joint_state_dot_in_msg)
        joint_state_dot_in_msgs.append(joint_state_dot_in_msg)

    des_payload = messaging.JointArrayStateMsgPayload()
    des_payload.states.clear()
    des_payload.stateDots.clear()
    des_payload.stateDDots.clear()
    for value in DES_JOINT_ANGLES:
        des_payload.states.push_back(float(value))
    for value in DES_JOINT_RATES:
        des_payload.stateDots.push_back(float(value))
    for value in DES_JOINT_ACCELS:
        des_payload.stateDDots.push_back(float(value))
    des_msg = messaging.JointArrayStateMsg().write(des_payload)
    module.desJointStatesInMsg.subscribeTo(des_msg)

    return module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg

def test_joint_array_ref_profiler_filter():
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointArrayRefProfiler.JointArrayRefProfiler` can be
    constructed from the standard ``Basilisk.simulation`` import path and
    that the low pass filter mode works correctly.

    **Description of Variables Being Tested**

    This unit test checks the outputs of ``desJointStatesOutMsg`` when using
    the low pass filter mode.
    """

    # Set up the module and its parameters
    module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg = setup_joint_array_ref_profiler("lowPass")
    filter_dt = 0.1  # [s]
    module.setFilterDt(filter_dt)  # [s]
    filter_cutoff_freq = 0.5  # [Hz]
    wc = 2.0 * np.pi * filter_cutoff_freq  # [rad/s]
    module.setWc(wc)  # [rad/s]

    module.Reset(0)
    module.UpdateState(0)

    refOut = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(refOut.states, JOINT_ANGLES)  # [rad]
    np.testing.assert_allclose(refOut.stateDots, JOINT_RATES)  # [rad/s]
    np.testing.assert_allclose(refOut.stateDDots, np.zeros(3))  # [rad/s^2]

    module.UpdateState(macros.sec2nano(filter_dt))

    refOut = module.desJointStatesOutMsg.read()
    beta = 1.0 - np.exp(-wc * filter_dt)
    refAngles = np.zeros(3)
    refRates = np.zeros(3)
    refAccels = np.zeros(3)
    for i in range(3):
        refAngles[i] = JOINT_ANGLES[i] + beta * (DES_JOINT_ANGLES[i] - JOINT_ANGLES[i])
        refRates[i] = (refAngles[i] - JOINT_ANGLES[i]) / filter_dt
        refAccels[i] = (refRates[i] - JOINT_RATES[i]) / filter_dt
    np.testing.assert_allclose(refOut.states, refAngles)  # [rad]
    np.testing.assert_allclose(refOut.stateDots, refRates)  # [rad/s]
    np.testing.assert_allclose(refOut.stateDDots, refAccels)  # [rad/s^2]

    module.UpdateState(macros.sec2nano(2.0 * filter_dt))

    refOut = module.desJointStatesOutMsg.read()
    prev_ref_angles = refAngles.copy()
    prev_ref_rates = refRates.copy()
    for i in range(3):
        refAngles[i] = prev_ref_angles[i] + beta * (DES_JOINT_ANGLES[i] - prev_ref_angles[i])
        refRates[i] = (refAngles[i] - prev_ref_angles[i]) / filter_dt
        refAccels[i] = (refRates[i] - prev_ref_rates[i]) / filter_dt
    np.testing.assert_allclose(refOut.states, refAngles)  # [rad]
    np.testing.assert_allclose(refOut.stateDots, refRates)  # [rad/s]
    np.testing.assert_allclose(refOut.stateDDots, refAccels)  # [rad/s^2]


def test_joint_array_ref_profiler_filter_wrap_shortest_path():
    """
    **Validation Test Description**

    This unit test verifies that a low-pass profile converges across a
    configured wrapping boundary when ``useShortestPath`` is enabled.

    **Description of Variables Being Tested**

    This unit test checks the wrapped output angle and the continuous rate and
    acceleration recurrence for a target that crosses the upper boundary of
    the configured joint-angle wrapping interval.
    """

    module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg = setup_joint_array_ref_profiler("lowPass")
    filter_dt = 0.1  # [s]
    wc = 1.0  # [rad/s]
    joint_angle_wrap_start = -np.pi  # [rad]
    module.setFilterDt(filter_dt)  # [s]
    module.setWc(wc)  # [rad/s]
    module.setJointAngleWrapStart(joint_angle_wrap_start)  # [rad]
    module.setUseShortestPath(True)

    joint_angles = np.array([3.0, 0.0, 0.0])  # [rad]
    joint_rates = np.zeros(3)  # [rad/s]
    des_joint_angles = np.array([3.2, 0.0, 0.0])  # [rad]
    for i in range(3):
        joint_state_in = messaging.ScalarJointStateMsgPayload()
        joint_state_in.state = joint_angles[i]
        joint_state_in_msg = messaging.ScalarJointStateMsg().write(joint_state_in)
        module.jointStatesInMsgs[i].subscribeTo(joint_state_in_msg)
        joint_state_in_msgs[i] = joint_state_in_msg

        joint_state_dot_in = messaging.ScalarJointStateMsgPayload()
        joint_state_dot_in.state = joint_rates[i]
        joint_state_dot_in_msg = messaging.ScalarJointStateMsg().write(joint_state_dot_in)
        module.jointStateDotsInMsgs[i].subscribeTo(joint_state_dot_in_msg)
        joint_state_dot_in_msgs[i] = joint_state_dot_in_msg

    des_payload = messaging.JointArrayStateMsgPayload()
    des_payload.states.clear()
    des_payload.stateDots.clear()
    des_payload.stateDDots.clear()
    for value in des_joint_angles:
        des_payload.states.push_back(float(value))
    for value in joint_rates:
        des_payload.stateDots.push_back(float(value))
    for value in DES_JOINT_ACCELS:
        des_payload.stateDDots.push_back(float(value))
    des_msg = messaging.JointArrayStateMsg().write(des_payload)
    module.desJointStatesInMsg.subscribeTo(des_msg)

    module.Reset(0)
    module.UpdateState(0)

    stored_des_joint_angle = float(des_joint_angles[0])  # [rad]
    wrapped_des_joint_angle = (
        np.remainder(stored_des_joint_angle - joint_angle_wrap_start, 2.0 * np.pi)
        + joint_angle_wrap_start
    )  # [rad]
    target_joint_angle = joint_angles[0] + np.remainder(
        wrapped_des_joint_angle - joint_angles[0] + np.pi,
        2.0 * np.pi,
    ) - np.pi  # [rad]
    beta = 1.0 - np.exp(-wc * filter_dt)
    expected_unwrapped_angle = joint_angles[0]  # [rad]
    expected_rate = joint_rates[0]  # [rad/s]

    for step in range(1, 61):
        previous_unwrapped_angle = expected_unwrapped_angle  # [rad]
        previous_rate = expected_rate  # [rad/s]
        expected_unwrapped_angle += beta * (target_joint_angle - previous_unwrapped_angle)
        expected_rate = (expected_unwrapped_angle - previous_unwrapped_angle) / filter_dt
        expected_accel = (expected_rate - previous_rate) / filter_dt  # [rad/s^2]
        expected_output_angle = (
            np.remainder(expected_unwrapped_angle - joint_angle_wrap_start, 2.0 * np.pi)
            + joint_angle_wrap_start
        )  # [rad]

        module.UpdateState(macros.sec2nano(step * filter_dt))
        ref_out = module.desJointStatesOutMsg.read()
        np.testing.assert_allclose(ref_out.states[0], expected_output_angle)  # [rad]
        np.testing.assert_allclose(ref_out.stateDots[0], expected_rate)  # [rad/s]
        np.testing.assert_allclose(ref_out.stateDDots[0], expected_accel)  # [rad/s^2]


def test_joint_array_ref_profiler_linear():
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointArrayRefProfiler.JointArrayRefProfiler` can be
    constructed from the standard ``Basilisk.simulation`` import path and
    that the linear profile mode works correctly.

    **Description of Variables Being Tested**

    This unit test checks the outputs of ``desJointStatesOutMsg`` when using
    the linear profile mode at the start of the profile, during the profile,
    and after the profile completes.
    """

    module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg = setup_joint_array_ref_profiler("linear")
    profile_duration = 0.4  # [s]
    module.setProfileDuration(profile_duration)  # [s]

    module.Reset(0)

    module.UpdateState(0)
    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, JOINT_ANGLES)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, (DES_JOINT_ANGLES - JOINT_ANGLES) / profile_duration)  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, np.zeros(3))  # [rad/s^2]

    mid_time = profile_duration / 2.0  # [s]
    module.UpdateState(macros.sec2nano(mid_time))
    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, JOINT_ANGLES + (DES_JOINT_ANGLES - JOINT_ANGLES) * mid_time / profile_duration)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, (DES_JOINT_ANGLES - JOINT_ANGLES) / profile_duration)  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, np.zeros(3))  # [rad/s^2]

    module.UpdateState(macros.sec2nano(profile_duration + 0.1))  # [s]
    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, DES_JOINT_ANGLES)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, np.zeros(3))  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, np.zeros(3))  # [rad/s^2]


def test_joint_array_ref_profiler_cubic():
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointArrayRefProfiler.JointArrayRefProfiler` can be
    constructed from the standard ``Basilisk.simulation`` import path and
    that the cubic profile mode works correctly.

    **Description of Variables Being Tested**

    This unit test checks the outputs of ``desJointStatesOutMsg`` when using
    the cubic profile mode at the start of the profile, during the profile,
    and after the profile completes. It also verifies that multi-turn joint
    angles remain unwrapped when joint-angle wrapping retains its default
    disabled state.
    """

    module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg = setup_joint_array_ref_profiler("cubic")
    profile_duration = 0.4  # [s]
    module.setProfileDuration(profile_duration)  # [s]
    assert not module.getJointAngleWrappingEnabled()

    unwrapped_joint_angles = JOINT_ANGLES + np.array([8.0, -6.0, 10.0]) * np.pi  # [rad]
    unwrapped_des_joint_angles = DES_JOINT_ANGLES + np.array([-8.0, 10.0, -6.0]) * np.pi  # [rad]
    stored_des_joint_angles = np.array([float(value) for value in unwrapped_des_joint_angles])  # [rad]
    for i in range(3):
        joint_state_in = messaging.ScalarJointStateMsgPayload()
        joint_state_in.state = unwrapped_joint_angles[i]
        joint_state_in_msg = messaging.ScalarJointStateMsg().write(joint_state_in)
        module.jointStatesInMsgs[i].subscribeTo(joint_state_in_msg)
        joint_state_in_msgs[i] = joint_state_in_msg

    des_payload = messaging.JointArrayStateMsgPayload()
    des_payload.states.clear()
    des_payload.stateDots.clear()
    des_payload.stateDDots.clear()
    for value in stored_des_joint_angles:
        des_payload.states.push_back(float(value))
    for value in DES_JOINT_RATES:
        des_payload.stateDots.push_back(float(value))
    for value in DES_JOINT_ACCELS:
        des_payload.stateDDots.push_back(float(value))
    des_msg = messaging.JointArrayStateMsg().write(des_payload)
    module.desJointStatesInMsg.subscribeTo(des_msg)

    module.Reset(0)

    module.UpdateState(0)
    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, unwrapped_joint_angles)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, JOINT_RATES)  # [rad/s]
    expected_accels = np.zeros(3)
    for i in range(3):
        expected_accels[i] = (
            2.0 * ((3.0 * (stored_des_joint_angles[i] - unwrapped_joint_angles[i])) / (profile_duration ** 2)
            - (2.0 * JOINT_RATES[i]) / profile_duration)
        )
    np.testing.assert_allclose(ref_out.stateDDots, expected_accels)  # [rad/s^2]

    tau = profile_duration / 2.0  # [s]
    module.UpdateState(macros.sec2nano(tau))
    ref_out = module.desJointStatesOutMsg.read()
    expected_angles = np.zeros(3)
    expected_rates = np.zeros(3)
    expected_accels = np.zeros(3)
    for i in range(3):
        theta0 = unwrapped_joint_angles[i]
        thetaf = stored_des_joint_angles[i]
        theta_dot0 = JOINT_RATES[i]
        a0 = theta0
        a1 = theta_dot0
        a2 = (3.0 * (thetaf - theta0)) / (profile_duration ** 2) - (2.0 * theta_dot0) / profile_duration
        a3 = -(2.0 * (thetaf - theta0)) / (profile_duration ** 3) + theta_dot0 / (profile_duration ** 2)
        expected_angles[i] = a0 + a1 * tau + a2 * tau ** 2 + a3 * tau ** 3
        expected_rates[i] = a1 + 2.0 * a2 * tau + 3.0 * a3 * tau ** 2
        expected_accels[i] = 2.0 * a2 + 6.0 * a3 * tau
    np.testing.assert_allclose(ref_out.states, expected_angles)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, expected_rates)  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, expected_accels)  # [rad/s^2]

    module.UpdateState(macros.sec2nano(profile_duration + 0.1))  # [s]
    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, stored_des_joint_angles)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, np.zeros(3))  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, np.zeros(3))  # [rad/s^2]


def test_joint_array_ref_profiler_cubic_joint_angle_wrap_range():
    """
    **Validation Test Description**

    This unit test verifies that the cubic profile mode wraps multi-turn
    current and desired joint angles when a joint-angle wrapping range is
    configured.

    **Description of Variables Being Tested**

    This unit test checks the cubic-profile outputs at the start, midway
    through, and after a profile whose input angles differ from their
    canonical equivalents by integer multiples of ``2*pi``. It also verifies
    that the configured interval can start at zero and that wrapping can be
    disabled.
    """

    module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg = setup_joint_array_ref_profiler("cubic")
    profile_duration = 0.4  # [s]
    joint_angle_wrap_start = 0.0  # [rad]
    module.setProfileDuration(profile_duration)  # [s]
    module.setJointAngleWrapStart(joint_angle_wrap_start)  # [rad]
    assert module.getJointAngleWrappingEnabled()
    assert module.getJointAngleWrapStart() == joint_angle_wrap_start

    unwrapped_joint_angles = JOINT_ANGLES + np.array([8.0, -6.0, 10.0]) * np.pi  # [rad]
    unwrapped_des_joint_angles = DES_JOINT_ANGLES + np.array([-8.0, 10.0, -6.0]) * np.pi  # [rad]
    stored_des_joint_angles = np.array([float(value) for value in unwrapped_des_joint_angles])  # [rad]
    for i in range(3):
        joint_state_in = messaging.ScalarJointStateMsgPayload()
        joint_state_in.state = unwrapped_joint_angles[i]
        joint_state_in_msg = messaging.ScalarJointStateMsg().write(joint_state_in)
        module.jointStatesInMsgs[i].subscribeTo(joint_state_in_msg)
        joint_state_in_msgs[i] = joint_state_in_msg

    des_payload = messaging.JointArrayStateMsgPayload()
    des_payload.states.clear()
    des_payload.stateDots.clear()
    des_payload.stateDDots.clear()
    for value in stored_des_joint_angles:
        des_payload.states.push_back(float(value))
    for value in DES_JOINT_RATES:
        des_payload.stateDots.push_back(float(value))
    for value in DES_JOINT_ACCELS:
        des_payload.stateDDots.push_back(float(value))
    des_msg = messaging.JointArrayStateMsg().write(des_payload)
    module.desJointStatesInMsg.subscribeTo(des_msg)

    expected_start_angles = (
        np.remainder(unwrapped_joint_angles - joint_angle_wrap_start, 2.0 * np.pi)
        + joint_angle_wrap_start
    )  # [rad]
    expected_des_angles = (
        np.remainder(stored_des_joint_angles - joint_angle_wrap_start, 2.0 * np.pi)
        + joint_angle_wrap_start
    )  # [rad]

    module.Reset(0)
    module.UpdateState(0)

    ref_out = module.desJointStatesOutMsg.read()
    expected_accels = np.zeros(3)
    for i in range(3):
        expected_accels[i] = (
            2.0 * ((3.0 * (expected_des_angles[i] - expected_start_angles[i])) / (profile_duration ** 2)
            - (2.0 * JOINT_RATES[i]) / profile_duration)
        )
    np.testing.assert_allclose(ref_out.states, expected_start_angles)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, JOINT_RATES)  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, expected_accels)  # [rad/s^2]

    tau = profile_duration / 2.0  # [s]
    module.UpdateState(macros.sec2nano(tau))

    ref_out = module.desJointStatesOutMsg.read()
    expected_angles = np.zeros(3)
    expected_rates = np.zeros(3)
    expected_accels = np.zeros(3)
    for i in range(3):
        theta0 = expected_start_angles[i]
        thetaf = expected_des_angles[i]
        theta_dot0 = JOINT_RATES[i]
        a0 = theta0
        a1 = theta_dot0
        a2 = (3.0 * (thetaf - theta0)) / (profile_duration ** 2) - (2.0 * theta_dot0) / profile_duration
        a3 = -(2.0 * (thetaf - theta0)) / (profile_duration ** 3) + theta_dot0 / (profile_duration ** 2)
        expected_angles[i] = np.remainder(
            a0 + a1 * tau + a2 * tau ** 2 + a3 * tau ** 3 - joint_angle_wrap_start,
            2.0 * np.pi,
        ) + joint_angle_wrap_start
        expected_rates[i] = a1 + 2.0 * a2 * tau + 3.0 * a3 * tau ** 2
        expected_accels[i] = 2.0 * a2 + 6.0 * a3 * tau
    np.testing.assert_allclose(ref_out.states, expected_angles)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, expected_rates)  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, expected_accels)  # [rad/s^2]

    module.UpdateState(macros.sec2nano(profile_duration + 0.1))  # [s]

    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, expected_des_angles)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, np.zeros(3))  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, np.zeros(3))  # [rad/s^2]

    module.disableJointAngleWrapping()
    assert not module.getJointAngleWrappingEnabled()

    new_des_joint_angles = stored_des_joint_angles + 0.1  # [rad]
    new_des_payload = messaging.JointArrayStateMsgPayload()
    new_des_payload.states.clear()
    new_des_payload.stateDots.clear()
    new_des_payload.stateDDots.clear()
    for value in new_des_joint_angles:
        new_des_payload.states.push_back(float(value))
    for value in DES_JOINT_RATES:
        new_des_payload.stateDots.push_back(float(value))
    for value in DES_JOINT_ACCELS:
        new_des_payload.stateDDots.push_back(float(value))
    new_des_msg = messaging.JointArrayStateMsg().write(new_des_payload)
    module.desJointStatesInMsg.subscribeTo(new_des_msg)

    module.UpdateState(macros.sec2nano(profile_duration + 0.2))  # [s]

    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, unwrapped_joint_angles)  # [rad]


def test_joint_array_ref_profiler_cubic_shortest_path():
    """
    **Validation Test Description**

    This unit test verifies that enabling ``useShortestPath`` makes a cubic
    profile cross the ``+-pi`` boundary along the shorter angular path.

    **Description of Variables Being Tested**

    This unit test compares cubic profiles for the same boundary-crossing
    commands with ``useShortestPath`` disabled and enabled. The default
    profile takes the long route, while the enabled profile takes the short
    route.
    """

    joint_angles = np.deg2rad(np.array([-170.0, 170.0, -170.0]))  # [rad]
    joint_rates = np.zeros(3)  # [rad/s]
    des_joint_angles = np.deg2rad(np.array([170.0, -170.0, 170.0]))  # [rad]
    profile_duration = 0.4  # [s]

    def configure_cubic_module(use_shortest_path):
        module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg = setup_joint_array_ref_profiler("cubic")
        module.setProfileDuration(profile_duration)  # [s]
        module.setUseShortestPath(use_shortest_path)

        for i in range(3):
            joint_state_in = messaging.ScalarJointStateMsgPayload()
            joint_state_in.state = joint_angles[i]
            joint_state_in_msg = messaging.ScalarJointStateMsg().write(joint_state_in)
            module.jointStatesInMsgs[i].subscribeTo(joint_state_in_msg)
            joint_state_in_msgs[i] = joint_state_in_msg

            joint_state_dot_in = messaging.ScalarJointStateMsgPayload()
            joint_state_dot_in.state = joint_rates[i]
            joint_state_dot_in_msg = messaging.ScalarJointStateMsg().write(joint_state_dot_in)
            module.jointStateDotsInMsgs[i].subscribeTo(joint_state_dot_in_msg)
            joint_state_dot_in_msgs[i] = joint_state_dot_in_msg

        des_payload = messaging.JointArrayStateMsgPayload()
        des_payload.states.clear()
        des_payload.stateDots.clear()
        des_payload.stateDDots.clear()
        for value in des_joint_angles:
            des_payload.states.push_back(float(value))
        for value in joint_rates:
            des_payload.stateDots.push_back(float(value))
        for value in DES_JOINT_ACCELS:
            des_payload.stateDDots.push_back(float(value))
        des_msg = messaging.JointArrayStateMsg().write(des_payload)
        module.desJointStatesInMsg.subscribeTo(des_msg)

        return module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg

    long_path_module, long_path_joint_msgs, long_path_rate_msgs, long_path_des_msg = configure_cubic_module(False)
    short_path_module, short_path_joint_msgs, short_path_rate_msgs, short_path_des_msg = configure_cubic_module(True)
    long_path_module.Reset(0)
    short_path_module.Reset(0)

    long_path_module.UpdateState(0)
    short_path_module.UpdateState(0)

    tau = profile_duration / 2.0  # [s]
    long_path_module.UpdateState(macros.sec2nano(tau))
    short_path_module.UpdateState(macros.sec2nano(tau))

    stored_des_joint_angles = np.array([float(value) for value in des_joint_angles])  # [rad]
    long_path_delta = stored_des_joint_angles - joint_angles  # [rad]
    short_path_delta = np.remainder(long_path_delta + np.pi, 2.0 * np.pi) - np.pi  # [rad]
    profile_fraction = tau / profile_duration  # [-]
    cubic_position_scale = 3.0 * profile_fraction ** 2 - 2.0 * profile_fraction ** 3  # [-]
    cubic_rate_scale = (6.0 * profile_fraction - 6.0 * profile_fraction ** 2) / profile_duration  # [1/s]
    cubic_accel_scale = (6.0 - 12.0 * profile_fraction) / profile_duration ** 2  # [1/s^2]

    long_path_out = long_path_module.desJointStatesOutMsg.read()
    short_path_out = short_path_module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(
        long_path_out.states,
        joint_angles + cubic_position_scale * long_path_delta,
        atol=1e-12,  # [rad]
    )
    np.testing.assert_allclose(short_path_out.states, joint_angles + cubic_position_scale * short_path_delta)  # [rad]
    np.testing.assert_allclose(long_path_out.stateDots, cubic_rate_scale * long_path_delta)  # [rad/s]
    np.testing.assert_allclose(short_path_out.stateDots, cubic_rate_scale * short_path_delta)  # [rad/s]
    np.testing.assert_allclose(
        long_path_out.stateDDots,
        cubic_accel_scale * long_path_delta,
        atol=1e-12,  # [rad/s^2]
    )
    np.testing.assert_allclose(
        short_path_out.stateDDots,
        cubic_accel_scale * short_path_delta,
        atol=1e-12,  # [rad/s^2]
    )

    long_path_module.UpdateState(macros.sec2nano(profile_duration + 0.1))  # [s]
    short_path_module.UpdateState(macros.sec2nano(profile_duration + 0.1))  # [s]

    long_path_out = long_path_module.desJointStatesOutMsg.read()
    short_path_out = short_path_module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(long_path_out.states, stored_des_joint_angles)  # [rad]
    np.testing.assert_allclose(short_path_out.states, joint_angles + short_path_delta)  # [rad]
    np.testing.assert_allclose(long_path_out.stateDots, np.zeros(3), atol=1e-12)  # [rad/s]
    np.testing.assert_allclose(short_path_out.stateDots, np.zeros(3), atol=1e-12)  # [rad/s]
    np.testing.assert_allclose(long_path_out.stateDDots, np.zeros(3), atol=1e-12)  # [rad/s^2]
    np.testing.assert_allclose(short_path_out.stateDDots, np.zeros(3), atol=1e-12)  # [rad/s^2]


def test_joint_array_ref_profiler_quintic():
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointArrayRefProfiler.JointArrayRefProfiler` can be
    constructed from the standard ``Basilisk.simulation`` import path and
    that the quintic profile mode works correctly.

    **Description of Variables Being Tested**

    This unit test checks the outputs of ``desJointStatesOutMsg`` when using
    the quintic profile mode at the start of the profile, during the profile,
    and after the profile completes.
    """

    module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg = setup_joint_array_ref_profiler("quintic")
    profile_duration = 0.4  # [s]
    module.setProfileDuration(profile_duration)  # [s]

    module.Reset(0)

    module.UpdateState(0)
    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, JOINT_ANGLES)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, JOINT_RATES)  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, np.zeros(3))  # [rad/s^2]

    tau = profile_duration / 2.0  # [s]
    module.UpdateState(macros.sec2nano(tau))
    ref_out = module.desJointStatesOutMsg.read()
    expected_angles = np.zeros(3)
    expected_rates = np.zeros(3)
    expected_accels = np.zeros(3)
    for i in range(3):
        theta0 = JOINT_ANGLES[i]
        thetaf = DES_JOINT_ANGLES[i]
        theta_dot0 = JOINT_RATES[i]
        delta_theta = thetaf - theta0
        a0 = theta0
        a1 = theta_dot0
        a2 = 0.0
        a3 = (20.0 * delta_theta - 12.0 * theta_dot0 * profile_duration) / (2.0 * profile_duration ** 3)
        a4 = (-30.0 * delta_theta + 16.0 * theta_dot0 * profile_duration) / (2.0 * profile_duration ** 4)
        a5 = (12.0 * delta_theta - 6.0 * theta_dot0 * profile_duration) / (2.0 * profile_duration ** 5)
        expected_angles[i] = a0 + a1 * tau + a2 * tau ** 2 + a3 * tau ** 3 + a4 * tau ** 4 + a5 * tau ** 5
        expected_rates[i] = a1 + 2.0 * a2 * tau + 3.0 * a3 * tau ** 2 + 4.0 * a4 * tau ** 3 + 5.0 * a5 * tau ** 4
        expected_accels[i] = 2.0 * a2 + 6.0 * a3 * tau + 12.0 * a4 * tau ** 2 + 20.0 * a5 * tau ** 3
    np.testing.assert_allclose(ref_out.states, expected_angles)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, expected_rates)  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, expected_accels)  # [rad/s^2]

    module.UpdateState(macros.sec2nano(profile_duration + 0.1))  # [s]
    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, DES_JOINT_ANGLES)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, np.zeros(3))  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, np.zeros(3))  # [rad/s^2]


def test_joint_array_ref_profiler_filter_reinitializes_on_new_command():
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointArrayRefProfiler.JointArrayRefProfiler` can be
    constructed from the standard ``Basilisk.simulation`` import path and
    that the low pass filter mode reinitializes correctly when a new desired
    joint state message is written after the filter has already stepped once.

    **Description of Variables Being Tested**

    This unit test checks the outputs of ``desJointStatesOutMsg`` when using
    the low pass filter mode after a second desired joint state message is
    written. The expected behavior is that the module reinitializes the
    internal reference state to the current joint states instead of
    continuing from the previous filtered reference.
    """

    module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg = setup_joint_array_ref_profiler("lowPass")
    filter_dt = 0.1  # [s]
    module.setFilterDt(filter_dt)  # [s]
    filter_cutoff_freq = 0.5  # [Hz]
    wc = 2.0 * np.pi * filter_cutoff_freq  # [rad/s]
    module.setWc(wc)  # [rad/s]

    module.Reset(0)
    module.UpdateState(0)
    module.UpdateState(macros.sec2nano(filter_dt))

    new_joint_angles = np.array([0.3, -0.1, 0.8])  # [rad]
    new_joint_rates = np.array([-0.02, 0.06, -0.01])  # [rad/s]
    new_joint_state_in_msgs = []
    new_joint_state_dot_in_msgs = []
    for i in range(3):
        joint_state_in = messaging.ScalarJointStateMsgPayload()
        joint_state_in.state = new_joint_angles[i]
        joint_state_in_msg = messaging.ScalarJointStateMsg().write(joint_state_in)
        module.jointStatesInMsgs[i].subscribeTo(joint_state_in_msg)
        new_joint_state_in_msgs.append(joint_state_in_msg)

        joint_state_dot_in = messaging.ScalarJointStateMsgPayload()
        joint_state_dot_in.state = new_joint_rates[i]
        joint_state_dot_in_msg = messaging.ScalarJointStateMsg().write(joint_state_dot_in)
        module.jointStateDotsInMsgs[i].subscribeTo(joint_state_dot_in_msg)
        new_joint_state_dot_in_msgs.append(joint_state_dot_in_msg)

    new_des_joint_angles = np.array([-0.4, 0.6, 0.9])  # [rad]
    new_des_payload = messaging.JointArrayStateMsgPayload()
    new_des_payload.states.clear()
    new_des_payload.stateDots.clear()
    new_des_payload.stateDDots.clear()
    for value in new_des_joint_angles:
        new_des_payload.states.push_back(float(value))
    for value in DES_JOINT_RATES:
        new_des_payload.stateDots.push_back(float(value))
    for value in DES_JOINT_ACCELS:
        new_des_payload.stateDDots.push_back(float(value))
    new_des_msg = messaging.JointArrayStateMsg().write(new_des_payload, macros.sec2nano(2.0 * filter_dt))
    module.desJointStatesInMsg.subscribeTo(new_des_msg)

    module.UpdateState(macros.sec2nano(2.0 * filter_dt))

    ref_out = module.desJointStatesOutMsg.read()
    np.testing.assert_allclose(ref_out.states, new_joint_angles)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, new_joint_rates)  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, np.zeros(3))  # [rad/s^2]


def test_joint_array_ref_profiler_quintic_repeated_identical_writes():
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointArrayRefProfiler.JointArrayRefProfiler` can be
    constructed from the standard ``Basilisk.simulation`` import path and
    that repeated writes of an identical desired joint state message do not
    restart the profile.

    **Description of Variables Being Tested**

    This unit test checks the outputs of ``desJointStatesOutMsg`` when using
    the quintic profile mode after an identical desired joint state message is
    rewritten at a later time. The expected behavior is that the profile
    continues based on accumulated profile time instead of resetting to the
    initial conditions.
    """

    module, joint_state_in_msgs, joint_state_dot_in_msgs, des_msg = setup_joint_array_ref_profiler("quintic")
    profile_duration = 0.4  # [s]
    module.setProfileDuration(profile_duration)  # [s]

    module.Reset(0)
    module.UpdateState(0)

    first_tau = 0.1  # [s]
    module.UpdateState(macros.sec2nano(first_tau))

    identical_des_payload = messaging.JointArrayStateMsgPayload()
    identical_des_payload.states.clear()
    identical_des_payload.stateDots.clear()
    identical_des_payload.stateDDots.clear()
    for value in DES_JOINT_ANGLES:
        identical_des_payload.states.push_back(float(value))
    for value in DES_JOINT_RATES:
        identical_des_payload.stateDots.push_back(float(value))
    for value in DES_JOINT_ACCELS:
        identical_des_payload.stateDDots.push_back(float(value))
    identical_des_msg = messaging.JointArrayStateMsg().write(
        identical_des_payload,
        macros.sec2nano(0.2)
    )
    module.desJointStatesInMsg.subscribeTo(identical_des_msg)

    second_tau = 0.2  # [s]
    module.UpdateState(macros.sec2nano(second_tau))

    ref_out = module.desJointStatesOutMsg.read()
    expected_angles = np.zeros(3)
    expected_rates = np.zeros(3)
    expected_accels = np.zeros(3)
    for i in range(3):
        theta0 = JOINT_ANGLES[i]
        thetaf = DES_JOINT_ANGLES[i]
        theta_dot0 = JOINT_RATES[i]
        delta_theta = thetaf - theta0
        a0 = theta0
        a1 = theta_dot0
        a2 = 0.0
        a3 = (20.0 * delta_theta - 12.0 * theta_dot0 * profile_duration) / (2.0 * profile_duration ** 3)
        a4 = (-30.0 * delta_theta + 16.0 * theta_dot0 * profile_duration) / (2.0 * profile_duration ** 4)
        a5 = (12.0 * delta_theta - 6.0 * theta_dot0 * profile_duration) / (2.0 * profile_duration ** 5)
        expected_angles[i] = a0 + a1 * second_tau + a2 * second_tau ** 2 + a3 * second_tau ** 3 + a4 * second_tau ** 4 + a5 * second_tau ** 5
        expected_rates[i] = a1 + 2.0 * a2 * second_tau + 3.0 * a3 * second_tau ** 2 + 4.0 * a4 * second_tau ** 3 + 5.0 * a5 * second_tau ** 4
        expected_accels[i] = 2.0 * a2 + 6.0 * a3 * second_tau + 12.0 * a4 * second_tau ** 2 + 20.0 * a5 * second_tau ** 3

    np.testing.assert_allclose(ref_out.states, expected_angles)  # [rad]
    np.testing.assert_allclose(ref_out.stateDots, expected_rates)  # [rad/s]
    np.testing.assert_allclose(ref_out.stateDDots, expected_accels)  # [rad/s^2]
