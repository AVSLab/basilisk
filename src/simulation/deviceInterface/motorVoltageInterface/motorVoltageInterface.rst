Executive Summary
-----------------

Interface module to convert an array of motor input voltages to an array of motor torques.

Message Connection Descriptions
-------------------------------

The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - motorVoltageInMsg
      - :ref:`ArrayMotorVoltageMsgPayload`
      - Message that contains motor voltage input states
    * - motorTorqueOutMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - Output message for motor torques

Detailed Model Description
--------------------------

This module is a simulation environment module which simulates the analog voltage interface of a motor
cluster. The input is an array of voltages :math:`V_i`. The motor torque :math:`u_{s_i}` is evaluated
using a linear mapping

.. math::
    u_{s_i}=V_i\gamma_iSF_i+b_i

where :math:`\gamma` is a constant value gain. SF is the scale factor error (i.e. a constant 1% gain error SF = 1.01).
:math:`b` is the bias term (i.e. a constant 1 Nm error on torque :math:`b` = 1.) The output of the module is an array
of motor torques.

Model Functions
---------------

The functions of the voltage to torque model are:

- **Voltage to Torque:** Converts given voltage to a torque command.

Model Assumptions and Limitations
---------------------------------

This code makes assumptions which are common to IMU modeling.

- **No noise:** There is no random noise input when making the voltage to torque conversion.
- **All motors are the same:** All motors utilize the same gain, scale factor, and bias. No motor can be singled out to have higher scale factor or bias, etc.

User Guide
----------

This section contains conceptual overviews of the code and clear examples for the prospective user.

Module Setup
~~~~~~~~~~~~

The interface module is created in python using:

.. code-block:: python
    :linenos:

    testModule = motorVoltageInterface.MotorVoltageInterface()
    testModule.ModelTag = "motorVoltageInterface"

If you have :math:`N` motors being modeled, the module parameters are set as `N`-dimensional lists of values.  The only parameter that must be set is the voltage to torque conversion gain :math:`\gamma` called ``voltage2TorqueGain``.  The scale factor :math:`SF` is called ``scaleFactor`` and defaults to 1.  The bias :math:`b_i` is called ``bias`` and defaults to zero.

A sample setup is done using:

.. code-block:: python
    :linenos:

    testModule.voltage2TorqueGain =[ 1.32, 0.99, 1.31] # [Nm/V] conversion gain
    testModule.scaleFactor =[ 1.01, 1.00, 1.02] # [unitless] scale factor
    testModule.bias =[0.01, 0.02, 0.04] # [Nm] bias
