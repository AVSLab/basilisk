Executive Summary
-----------------
The ``JointPIDController`` module implements a proportional-integral-derivative (PID) controller for scalar MuJoCo
joints (rotational or translational). It reads measured and desired joint state messages, computes a control output,
and publishes it as a ``SingleActuatorMsgPayload``.

Message Interfaces
------------------
.. bsk-module-io:: JointPIDController

   input measuredPosInMsg ScalarJointStateMsgPayload
      Measured joint position. ``state`` field is used as the measured position.

   input measuredVelInMsg ScalarJointStateMsgPayload
      Measured joint velocity. ``state`` field is used as the measured velocity.

   input desiredPosInMsg ScalarJointStateMsgPayload
      Desired joint position. Required when ``Kp`` is non-zero.

   input desiredVelInMsg ScalarJointStateMsgPayload
      Desired joint velocity. Required when ``Kd`` is non-zero.

   output outputOutMsg SingleActuatorMsgPayload
      Control output written to ``input`` field.

Module Description
------------------
The controller computes

.. math::

   u = K_p\,(q_d - q) + K_d\,(\dot{q}_d - \dot{q}) + K_i\,\int(q_d - q)\,\text{d}t

where :math:`q`, :math:`\dot{q}` are the measured position and velocity, :math:`q_d`, :math:`\dot{q}_d` are the
desired values, and :math:`K_p`, :math:`K_d`, :math:`K_i` are the proportional, derivative, and integral gains.

The integral error is stored as a registered state and evolved by the simulation integrator.

The module inherits from the generic ``PIDController`` base class (in ``_GeneralModuleFiles/PIDController.h``),
which is templated on ``ScalarJointStateMsgPayload`` (measured and desired) and ``SingleActuatorMsgPayload`` (output).

Verification and Testing
------------------------
The module is verified in
``src/simulation/mujocoDynamics/JointPIDController/_UnitTest/test_JointPIDController.py`` by simulating a
single-joint arm with a linearly ramping desired position. The test checks that:

- The PID output matches the expected formula at every time step.
- The joint position converges to the desired final position within 1%.
- The joint velocity settles near the desired final velocity within 1%.
