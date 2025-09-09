Executive Summary
-----------------
The stepper motor module simulates the actuation of a stepper motor. Given the initial motor angle :math:`\theta_0`, a
fixed motor step angle :math:`\Delta\theta`, a fixed motor step time :math:`\Delta t`, and an input message containing
an integer number of steps commanded, the motor states are computed at each time step and output from the module.
The motor states include the scalar motor angle :math:`\theta`, scalar angle rate :math:`\dot{\theta}`, scalar angular
acceleration :math:`\ddot{\theta}`, the current motor step count :math:`c_s`, and the number of steps commanded to the
motor :math:`n_s`. The motor actuation through each step of the command sequence is profiled using a bang-bang
acceleration profile. This module also includes logic for handling incoming reference commands that interrupt an
unfinished motor actuation sequence. Because the stepper motor is unable to stop actuating during a step, it must
finish actuating through the current step before it can begin following a new reference command.

Message Connection Description
------------------------------
The following table lists the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - motorStepCommandInMsg
      - :ref:`MotorStepCommandMsgPayload`
      - Input message containing the number of commanded motor steps
    * - stepperMotorOutMsg
      - :ref:`StepperMotorMsgPayload`
      - Output message containing the stepper motor states

Motor Step Actuation Profile
----------------------------
The motor states are profiled identically for each step in the command sequence. Specifically, a bang-bang acceleration
profile is used to profile the motor states as a function of time. Given the motor step time :math:`\Delta t` and motor
step angle :math:`\Delta \theta` as fixed parameters, the module calculates the required acceleration
:math:`\ddot{\theta}_{\text{max}}` that must be applied during each motor step.

.. math::
    | \ddot{\theta}_{\text{max}} | = \frac{4 \Delta \theta}{\Delta t^2}

Note that the motor can take either positive or negative steps. For a forward actuation command (:math:`n_s > 0`), the
calculated acceleration is applied positively for the first half of the step and negatively during the second half of
the step. For a backward actuation command (:math:`n_s < 0`), the acceleration is applied negatively during the first
half of the step and positively during the second half of the step.

Given the initial time :math:`t_0`, the switch time :math:`t_s` where the acceleration is alternated and the final time
:math:`t_f` when the step is complete is determined as

.. math::
    t_s = t_0 + \frac{\Delta t}{2}

    t_f = t_0 + \Delta t

The other motor states can be kinematically profiled as a function of time by integrating the applied acceleration
profile. The equations used to profile each motor step are

.. math::
    \ddot{\theta}(t) =
    \begin{cases}
    \pm \ddot{\theta}_{\text{max}}
    & \text{if }
    t_0 \leq t < t_s
    \\
    \mp \ddot{\theta}_{\text{max}}
    & \text{if }
    t_s \leq t \leq t_f
    \\
    0
    & \text{if }
    t > t_f
    \end{cases}

    \dot{\theta}(t) =
    \begin{cases}
    \pm \ddot{\theta}_{\text{max}} (t - t_0)
    & \text{if }
    t_0 \leq t < t_s
    \\
    \mp \ddot{\theta}_{\text{max}} (t - t_f)
    & \text{if }
    t_s \leq t \leq t_f
    \\
    0
    & \text{if }
    t > t_f
    \end{cases}

    \theta(t) =
    \begin{cases}
    \frac{\pm \Delta \theta (t - t_0)^2}{2 (t_s - t_0)^2} + \theta_0
    & \text{if }
    t_0 \leq t < t_s
    \\
    \frac{\mp \Delta \theta (t - t_f)^2}{2 (t_s - t_f)^2} + \theta_{\text{ref}}
    & \text{if }
    t_s \leq t \leq t_f
    \\
    \theta_{\text{ref}}
    & \text{if }
    t > t_f
    \end{cases}

Note that the parameters :math:`t_0, t_s` and :math:`t_f` must be continually updated after each step is complete to
reflect the advancement of time. Doing so enables use of the above equations for each motor step.

.. important::
    If the motor actuation is interrupted by a new reference message while actuating through a step,
    the motor must finish actuating through the current step before it can begin following a new reference command.
    If the interrupting message is written when the motor is not in the midst of a step, the module resets the motor
    step count and immediately begins actuating to follow the new reference command.

Module Functions
----------------
Below is a list of functions that this simulation module performs

    - Reads the incoming motor step command message
    - Computes the motor states as a function of time
    - Writes the motor states to the module output message
    - Handles interruptions to motor actuation by resetting the motor actuation after the current step is complete

Module Assumptions and Limitations
----------------------------------
    - The motor step angle and step time are fixed parameters (Cannot be negative)
    - The motor cannot stop actuating in the middle of a step
    - When the motor actuation is interrupted by a new reference command, the motor must complete its actuation through the current step before following the new command
    - The module update rate must be faster than or equal to the provided motor step time :math:`\Delta t`
    - The module update rate cannot be slower than the motor step rate, or the motor actuation cannot be resolved

Test Description and Success Criteria
-------------------------------------
There are two tests for this module. The two tests are described in :ref:`test_stepperMotor`.
The first test is a nominal test named ``test_stepper_motor_nominal``. The second test named
``test_stepper_motor_interrupt`` tests the module logic for commands interrupting the motor actuation. Both tests
configure two actuation commands. The nominal test separates the actuation commands by a rest period of 5 seconds. The
interruption test interrupts the first command sequence after half of the commanded motor steps are completed. The
time the second command message is written is determined using an interruption factor to specify what fraction of the
next step is completed before the second command message is written. Both tests add 5 seconds to the end of each
simulation for clarity when viewing the generated plots.

The success criteria for all tests is that the motor states converge to the computed reference values in the test at
the end of each actuation sequence. Specifically, the motor angle, rate, acceleration, and step count are checked to
converge to the reference values at the end of each simulation chunk. The motor rate and acceleration are checked to
be zero at the end of each actuation sequence. Note that the motor acceleration is checked at one time step after the
other motor states are checked because the motor acceleration is nonzero at the completion of each motor step. The motor
acceleration is not zero until after the motor step is complete. This differs from the motor rate profile, which
is zero at the completion of each motor step.

Nominal Test
~~~~~~~~~~~~
The nominal unit test configures two actuation command segments with a rest period of 5 seconds between the commands.
A rest period of 5 seconds is also added to the end of the simulation for clarity when viewing the generated plots.
The initial motor angle, motor step angle, step time, and steps commanded for each actuation sequence are varied so
that the motor actuates both forwards and backwards during the test. Zero steps commanded are also included in the
test to check that the module correctly updates the motor states for a zero command message. The motor angle, rate,
acceleration, and step count are checked to converge to the reference values at the end of each simulation chunk.

Interruption Test
~~~~~~~~~~~~~~~~~
The interruption unit test ensures that the module correctly handles reference messages that interrupt an unfinished
motor actuation sequence. The initial motor angle, motor step angle, and step time are not varied in the interruption
test because these parameters were already varied in the nominal test. The interruption test interrupts the first
command sequence after half of the commanded motor steps are completed. The time the second command message is
written is determined using an interruption factor to specify what fraction of the next step is completed before the
second command message is written. Interruption factors of 0 and 1 are also included to ensure the module correctly
resets the motor states when the interruption falls precisely when a step is completed. A rest period of 5 seconds is
added to the end of the simulation for clarity when viewing the generated plots. The motor angle, rate,
acceleration, and step count are checked to converge to the reference values at the end of each simulation chunk.
