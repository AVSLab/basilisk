Executive Summary
-----------------
Outputs measured angle and angle rate for a hinged rigid body.  By default the angle measurement does not contain
any measurement errors.  The module can be configured to add gaussian noise, a bias or a discretization error to the
measured state.

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
    * - hingedRigidBodyMotorSensorInMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - input hinged rigid body state
    * - hingedRigidBodyMotorSensorOutMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - sensed hinged rigid body state

Detailed Model Description
--------------------------

This module adds optional noise, bias, and discretization to a hinged rigid body state.

User Guide
----------

This section contains a conceptual overview of the code and an example for the prospective user.

Module Setup
~~~~~~~~~~~~

The interface module is created in python using:

.. code-block:: python
    :linenos:

    testModule = hingedRigidBodyMotorSensor.HingedRigidBodyMotorSensor()
    testModule.ModelTag = "motorSensor"

The sensor adds an optional noise (defined by a standard deviation), bias, and discretization to ``theta`` and ``thetaDot``.
If no noise is specified then the default behavior is for the actual panel states to be passed along as the
sensed state.

A sample setup is done using:

.. code-block:: python
    :linenos:

    testModule.thetaNoiseStd = thetaNoiseStd # [rad]
    testModule.thetaDotNoiseStd = thetaDotNoiseStd # [rad/s]
    testModule.thetaBias = thetaBias # [rad]
    testModule.thetaDotBias = thetaDotBias # [rad/s]
    testModule.thetaLSB = thetaLSB # [rad] smallest resolution of sensor
    testModule.thetaDotLSB = thetaDotLSB # [rad/s] smallest sensor rate resolution

When discretizing, the value is rounded up or down to the nearest sensor state resolution.
The seed used for the noise can also be changed using:

.. code-block:: python
    :linenos:

    testModule.RNGSeed = newSeed

Where ``newSeed`` is the unsigned integer value of the new RNG seed.
