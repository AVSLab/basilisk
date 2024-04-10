Executive Summary
-----------------
This prescribed motion-specific module computes the hub-relative rotational states of a spinning body following
prescribed rotational one degree-of-freedom (DOF) motion about a single hub-fixed axis. The body frame of the spinning
body is designated by the frame :math:`\mathcal{F}`. The spinning body's states are profiled relative to a hub-fixed
frame :math:`\mathcal{M}`. The input message to this module is the :ref:`StepperMotorMsgPayload` message which provides
the scalar spinning body states relative to the hub ``theta``, ``thetaDot``, and ``thetaDDot``. Given these scalar
states together with the spinning body rotation axis specified by the user, the prescribed rotational states
``sigma_FM``, ``omega_FM_F``, and ``omegaPrime_FM_F`` are computed and output from the module using the
:ref:`PrescribedRotationMsgPayload` message. The only required input to this module that must be set by the user is
the spinning body rotation axis expressed as a unit vector in Mount frame components ``rotHat_M``.

.. important::
    To use this module for prescribed motion, it must be connected to the :ref:`PrescribedMotionStateEffector`
    dynamics module. This ensures the spinning body's states are correctly incorporated into the spacecraft dynamics.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what the message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - stepperMotorInMsg
      - :ref:`StepperMotorMsgPayload`
      - input msg with the stepper motor scalar states
    * - prescribedRotationOutMsg
      - :ref:`PrescribedRotationMsgPayload`
      - output message with the prescribed spinning body rotational states

Detailed Module Description
---------------------------
To compute the spinning body attitude relative to the hub-fixed mount frame, the given axis of rotation and the current
stepper motor scalar angle are used in the following expression:

.. math::
    \boldsymbol{\sigma}_{\mathcal{F}/\mathcal{M}} = \tan \left ( \frac{\theta}{4} \right ) \hat{\boldsymbol{s}}

where :math:`\hat{\boldsymbol{s}}` is the given axis of rotation.

The spinning body angular velocity relative to the mount frame is computed using the following expression:

.. math::
    {}^{\mathcal{F}} \boldsymbol{\omega}_{\mathcal{F}/\mathcal{M}} = \dot{\theta} \ {}^{\mathcal{M}} \hat{\boldsymbol{s}}

The spinning body angular acceleration relative to the mount frame is similarly computed:

.. math::
    {}^{\mathcal{F}} \boldsymbol{\omega}^{'}_{\mathcal{F}/\mathcal{M}} = \ddot{\theta} \ {}^{\mathcal{M}} \hat{\boldsymbol{s}}

The computed prescribed rotational states ``sigma_FM``, ``omega_FM_F``, and ``omegaPrime_FM_F`` are then written to the
:ref:`PrescribedRotationMsgPayload` module output message.

Module Testing
^^^^^^^^^^^^^^
The unit test for this module ensures that the single-axis hub-relative rotational scalar states ``theta``,
``thetaDot``, and ``thetaDDot`` are properly converted to prescribed hub-relative rotational states ``sigma_FM``,
``omega_FM_F``, and ``omegaPrime_FM_F``. The scalar states are varied in the test, along with a single angle
``rotAxis_MAngle`` that is used to vary the axis of rotation ``rotAxis_M`` associated with the given
scalar information. To verify the module, the final prescribed rotational states are logged from the module. The
final attitude is checked with the computed true attitude. The dot product between the given rotation axis and
the final angular velocity and angular acceleration is checked to match the given scalar angle rate and angle
acceleration.

User Guide
----------
The only required input to this module that must be set by the user is the spinning body rotation axis expressed
as a unit vector in Mount frame components ``rotHat_M``. This section outlines the steps needed to set up this
single axis profiler module in python using Basilisk.

#. Import the singleAxisProfiler class::

    from Basilisk.simulation import singleAxisProfiler

#. Create an instantiation of the module::

    singleAxisRotProfiler = singleAxisProfiler.SingleAxisProfiler()

#. Define all of the configuration data associated with the module::

    singleAxisRotProfiler.ModelTag = "singleAxisProfiler"
    singleAxisRotProfiler.setRotHat_M([1.0, 0.0, 0.0])

#. Connect the :ref:`StepperMotorMsgPayload` input message to the module that tracks the stepper motor states in time. For example, the user can create a stand-alone message to specify a non-rotating spinning body::

    stepperMotorMessageData = messaging.StepperMotorMsgPayload()
    stepperMotorMessageData.theta = 10.0 * np.pi / 180.0  # [rad]
    stepperMotorMessageData.thetaDot = 0.0 * np.pi / 180.0  # [rad/s]
    stepperMotorMessageData.thetaDDot = 0.0 * np.pi / 180.0  # [rad/s^2]
    stepperMotorMessage = messaging.StepperMotorMsg().write(stepperMotorMessageData)

#. Subscribe the singleAxisProfiler module input message to the stepper motor message::

    singleAxisRotProfiler.stepperMotorInMsg.subscribeTo(stepperMotorMessage)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, singleAxisRotProfiler)
