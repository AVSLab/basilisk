#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import math
import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import stepperMotor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


@pytest.mark.parametrize("motor_theta_init", [0.0 * macros.D2R, 10.0 * macros.D2R, -5.0 * macros.D2R])
@pytest.mark.parametrize("steps_commanded_1", [0, 10, -15])
@pytest.mark.parametrize("steps_commanded_2", [0, 10, -15])
@pytest.mark.parametrize("step_angle", [0.1 * macros.D2R, 0.5 * macros.D2R, 1.0 * macros.D2R])
@pytest.mark.parametrize("step_time", [0.3, 0.5, 1.0])
def test_stepper_motor_nominal(show_plots,
                               motor_theta_init,
                               steps_commanded_1,
                               steps_commanded_2,
                               step_angle,
                               step_time):
    r"""
    **Verification Test Description**

    This nominal unit test ensures that the stepper motor simulation module correctly actuates the motor from an
    initial angle to a final reference angle, given an input integer number of commanded steps. The module outputs
    the motor scalar states (angle, angle rate, acceleration), the motor step count, and steps commanded as a function
    of time. The motor actuation is profiled using a bang-bang acceleration profile. The motor acceleration is
    calculated in the module using the given motor step angle and step time constants. The capability for the motor to
    actuate both forwards and backwards is checked by commanding both positive and negative steps to the module.
    Zero steps commanded are also included in the test to check that the module correctly updates the motor states for
    a zero command message.

    **Test Parameters**

    Args:
        motor_theta_init (float): [rad] Initial stepper motor angle
        steps_commanded_1 (int): [steps] Number of steps commanded to the stepper motor (first command)
        steps_commanded_2 (int): [steps] Number of steps commanded to the stepper motor (second command)
        step_angle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        step_time (float): [sec] Time required for a single motor step (constant)

    **Description of Variables Being Tested**

     The motor angle, rate, acceleration, and step count are checked to converge to the reference values at the end of
     each simulation chunk.

    """

    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.01
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Create the stepperMotor module
    stepper_motor = stepperMotor.StepperMotor()
    stepper_motor.ModelTag = "StepperMotor"
    stepper_motor.setThetaInit(motor_theta_init)
    stepper_motor.setStepAngle(step_angle)
    stepper_motor.setStepTime(step_time)
    test_sim.AddModelToTask(task_name, stepper_motor)

    # Create the first stepperMotor input message
    motor_step_command_msg_data = messaging.MotorStepCommandMsgPayload()
    motor_step_command_msg_data.stepsCommanded = steps_commanded_1
    motor_step_command_msg = messaging.MotorStepCommandMsg().write(motor_step_command_msg_data)
    stepper_motor.motorStepCommandInMsg.subscribeTo(motor_step_command_msg)

    # Set up data logging
    stepper_motor_data_log = stepper_motor.stepperMotorOutMsg.recorder()
    test_sim.AddModelToTask(task_name, stepper_motor_data_log)

    # Run the simulation
    test_sim.InitializeSimulation()
    sim_time_1 = step_time * abs(steps_commanded_1)  # [s]
    sim_time_extra = 5.0  # [s]
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time_1 + sim_time_extra))
    test_sim.ExecuteSimulation()

    # Create the second stepperMotor input message
    motor_step_command_msg_data = messaging.MotorStepCommandMsgPayload()
    motor_step_command_msg_data.stepsCommanded = steps_commanded_2
    motor_step_command_msg = messaging.MotorStepCommandMsg().write(motor_step_command_msg_data,
                                                                   macros.sec2nano(sim_time_1 + sim_time_extra))
    stepper_motor.motorStepCommandInMsg.subscribeTo(motor_step_command_msg)

    # Run the simulation
    sim_time_2 = step_time * abs(steps_commanded_2)  # [s]
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time_1
                                               + sim_time_extra
                                               + sim_time_2
                                               + sim_time_extra))
    test_sim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * stepper_motor_data_log.times()  # [s]
    theta = macros.R2D * stepper_motor_data_log.theta  # [deg]
    theta_dot = macros.R2D * stepper_motor_data_log.thetaDot  # [deg/s]
    theta_ddot = macros.R2D * stepper_motor_data_log.thetaDDot  # [deg/s^2]
    motor_step_count = stepper_motor_data_log.stepCount
    motor_steps_commanded = stepper_motor_data_log.stepsCommanded

    if show_plots:
        plot_results(timespan,
                     theta,
                     theta_dot,
                     theta_ddot,
                     motor_step_count,
                     motor_steps_commanded)
        plt.show()
    plt.close("all")

    accuracy = 1e-12

    # Check the motor states converge to the reference values for the first actuation
    motor_theta_final_1_index = int(round(sim_time_1 / test_time_step_sec))
    motor_theta_ref_1_true = motor_theta_init + (steps_commanded_1 * step_angle)
    np.testing.assert_allclose(theta[motor_theta_final_1_index],
                               macros.R2D * motor_theta_ref_1_true,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_dot[motor_theta_final_1_index],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_ddot[motor_theta_final_1_index + 1],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(motor_step_count[motor_theta_final_1_index],
                               steps_commanded_1,
                               atol=accuracy,
                               verbose=True)

    # Check the motor states converge to the reference values for the second actuation
    motor_theta_final_2_index = int(round((sim_time_1 + sim_time_extra + sim_time_2) / test_time_step_sec))
    motor_theta_ref_2_true = motor_theta_ref_1_true + (steps_commanded_2 * step_angle)
    np.testing.assert_allclose(theta[motor_theta_final_2_index],
                               macros.R2D * motor_theta_ref_2_true,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_dot[motor_theta_final_2_index],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_ddot[motor_theta_final_2_index + 1],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(motor_step_count[motor_theta_final_2_index + 1],
                               steps_commanded_2,
                               atol=accuracy,
                               verbose=True)


@pytest.mark.parametrize("steps_commanded_1", [10, -16, -20])
@pytest.mark.parametrize("steps_commanded_2", [-10, -16, 20])
@pytest.mark.parametrize("interrupt_fraction", [0.0, 0.25, 0.5, 0.75, 1.0])
def test_stepper_motor_interrupt(show_plots, steps_commanded_1, steps_commanded_2, interrupt_fraction):
    r"""
    **Verification Test Description**

    The interruption unit test ensures that the module correctly handles reference messages that interrupt an
    unfinished motor actuation sequence. The initial motor angle, motor step angle, and step time are not varied in the
    interruption test because these parameters were already varied in the nominal test. The interruption test interrupts
    the first command sequence after half of the commanded motor steps are completed. The time the second command
    message is written is determined using an interruption factor to specify what fraction of the next step is
    completed before the second command message is written. Interruption factors of 0 and 1 are also included to
    ensure the module correctly resets the motor states when the interruption falls precisely when a step is completed.
    A rest period of 5 seconds is added to the end of the simulation for clarity when viewing the generated plots.

    **Test Parameters**

    Args:
        steps_commanded_1 (int): [steps] Number of steps commanded to the stepper motor (First)
        steps_commanded_2 (int): [steps] Number of steps commanded to the stepper motor (Second)
        interrupt_fraction (float): Specifies what step fraction is completed when the interrupted message is written

    **Description of Variables Being Tested**

    The motor angle, rate, acceleration, and step count are checked to converge to the reference values at the end of
    each simulation chunk.

    """

    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.01
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Create the stepperMotor module
    step_time = 1.0  # [s]
    step_angle = 1.0 * macros.D2R  # [rad]
    stepper_motor = stepperMotor.StepperMotor()
    stepper_motor.ModelTag = "StepperMotor"
    stepper_motor.setThetaInit(0.0)
    stepper_motor.setStepAngle(step_angle)
    stepper_motor.setStepTime(step_time)
    test_sim.AddModelToTask(task_name, stepper_motor)

    # Create the first stepperMotor input message
    motor_step_command_msg_data = messaging.MotorStepCommandMsgPayload()
    motor_step_command_msg_data.stepsCommanded = steps_commanded_1
    motor_step_command_msg = messaging.MotorStepCommandMsg().write(motor_step_command_msg_data)
    stepper_motor.motorStepCommandInMsg.subscribeTo(motor_step_command_msg)

    # Set up data logging
    stepper_motor_data_log = stepper_motor.stepperMotorOutMsg.recorder()
    test_sim.AddModelToTask(task_name, stepper_motor_data_log)

    # Determine the simulation time
    sim_time_1_no_interrupt = step_time * abs(steps_commanded_1)
    sim_time_1_interrupt = 0.5 * sim_time_1_no_interrupt + interrupt_fraction * step_time

    # Run the first simulation chunk
    test_sim.InitializeSimulation()
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time_1_interrupt))
    test_sim.ExecuteSimulation()

    # Create the second stepperMotor input message
    motor_step_command_msg_data = messaging.MotorStepCommandMsgPayload()
    motor_step_command_msg_data.stepsCommanded = steps_commanded_2
    motor_step_command_msg = messaging.MotorStepCommandMsg().write(motor_step_command_msg_data,
                                                                   macros.sec2nano(sim_time_1_interrupt))
    stepper_motor.motorStepCommandInMsg.subscribeTo(motor_step_command_msg)

    # Run the second simulation chunk
    sim_time_2 = step_time * abs(steps_commanded_2)  # [s]
    sim_time_extra = 5.0  # [s]
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time_1_interrupt + sim_time_2 + sim_time_extra))
    test_sim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = macros.NANO2SEC * stepper_motor_data_log.times()  # [s]
    theta = macros.R2D * stepper_motor_data_log.theta  # [deg]
    theta_dot = macros.R2D * stepper_motor_data_log.thetaDot  # [deg/s]
    theta_ddot = macros.R2D * stepper_motor_data_log.thetaDDot  # [deg/s^2]
    motor_step_count = stepper_motor_data_log.stepCount
    motor_steps_commanded = stepper_motor_data_log.stepsCommanded

    if show_plots:
        plot_results(timespan,
                     theta,
                     theta_dot,
                     theta_ddot,
                     motor_step_count,
                     motor_steps_commanded)
        plt.show()
    plt.close("all")

    # Determine the true number of motor steps taken for the first actuation segment
    steps_taken_1 = math.ceil(sim_time_1_interrupt / step_time)
    if steps_commanded_1 < 0:
        steps_taken_1 = - steps_taken_1

    # Determine the true simulation time for the first actuation segment
    sim_time_1 = abs(step_time * steps_taken_1)

    accuracy = 1e-12
    # Check the motor states converge to the correct values for the first actuation
    motor_theta_final_1_index = int(round(sim_time_1 / test_time_step_sec))
    motor_theta_ref_1_true = step_angle * steps_taken_1
    np.testing.assert_allclose(theta[motor_theta_final_1_index],
                               macros.R2D * motor_theta_ref_1_true,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_dot[motor_theta_final_1_index],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(motor_step_count[motor_theta_final_1_index],
                               steps_taken_1,
                               atol=accuracy,
                               verbose=True)

    # Check the motor states converge to the reference values for the second actuation
    motor_theta_final_2_index = int(round((sim_time_1 + sim_time_2) / test_time_step_sec))
    motor_theta_ref_2_true = motor_theta_ref_1_true + step_angle * steps_commanded_2
    np.testing.assert_allclose(theta[motor_theta_final_2_index],
                               macros.R2D * motor_theta_ref_2_true,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_dot[motor_theta_final_2_index],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(theta_ddot[motor_theta_final_2_index + 1],
                               0.0,
                               atol=accuracy,
                               verbose=True)
    np.testing.assert_allclose(motor_step_count[motor_theta_final_2_index],
                               steps_commanded_2,
                               atol=accuracy,
                               verbose=True)


def plot_results(timespan,
                 theta,
                 theta_dot,
                 theta_ddot,
                 motor_step_count,
                 motor_steps_commanded):

    # Plot motor angle
    plt.figure()
    plt.clf()
    plt.plot(timespan, theta, label=r"$\theta$")
    plt.title(r'Stepper Motor Angle $\theta$', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor theta_dot
    plt.figure()
    plt.clf()
    plt.plot(timespan, theta_dot, label=r"$\dot{\theta}$")
    plt.title(r'Stepper Motor Angle Rate $\dot{\theta}$', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor theta_ddot
    plt.figure()
    plt.clf()
    plt.plot(timespan, theta_ddot, label=r"$\ddot{\theta}$")
    plt.title(r'Stepper Motor Angular Acceleration $\ddot{\theta}$ ', fontsize=14)
    plt.ylabel('(deg/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot steps commanded and motor steps taken
    plt.figure()
    plt.clf()
    plt.plot(timespan, motor_step_count, label='Step Count')
    plt.plot(timespan, motor_steps_commanded, '--', label='Commanded')
    plt.title(r'Motor Step History', fontsize=14)
    plt.ylabel('Steps', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)


if __name__ == "__main__":
    test_stepper_motor_nominal(
        True,
        0.0,  # [rad] motor_theta_init
        15,  # steps_commanded_1
        -15,  # steps_commanded_2
        1.0 * macros.D2R,  # [rad] step_angle
        1.0,  # [s] step_time
    )

    test_stepper_motor_interrupt(
        True,
        10,  # steps_commanded_1
        -10,  # steps_commanded_2
        0.5,  # interrupt_fraction
    )
