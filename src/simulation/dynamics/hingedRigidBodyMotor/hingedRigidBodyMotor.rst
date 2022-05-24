Executive Summary
-----------------
Calculates a motor torque given a sensed and reference hinged rigid body state using a simple PD control law.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - hingedBodyStateSensedInMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - sensed rigid body state (theta, theta dot)
    * - hingedBodyStateReferenceInMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - reference hinged rigid body state (theta, theta dot)
    * - motorTorqueOutMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - motor torque on hinged rigid body

Detailed Model Description
--------------------------

This module takes in a reference angle and angle rate, as well as a sensed angle and angle rate, and calculates the motor torque according to:

.. math::
    u = -K(\theta_s-\theta_r)-P(\dot{\theta}_s-\dot{\theta}_r)

K and P are the constant gains. Both should be set to positive values.

User Guide
----------

This section contains a conceptual overview of the code and an example for the prospective user.

Module Setup
~~~~~~~~~~~~

The interface module is created in python using:

.. code-block:: python
    :linenos:

    testModule = hingedRigidBodyMotor.hingedRigidBodyMotor()
    testModule.ModelTag = "hingedRigidBodyMotor"


A sample setup is done using:

.. code-block:: python
    :linenos:

    testModule.K = 1 # proportional gain constant
    testModule.P = 1 # derivative gain constant

If :math:`K` and :math:`P` are not set, they default to 0.
