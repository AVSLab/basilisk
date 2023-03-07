Executive Summary
-----------------
Models the addition of noise, bias, and faults to temperature measurements.

Message Connection Descriptions
-------------------------------
The following table lists all module input and output messages. The module msg connection 
is set by the user from python. The msg type contains a link to the message structure definition, 
while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - tempInMsg
      - :ref:`TemperatureMsgPayload`
      - True temperature measurement
    * - tempOutMsg
      - :ref:`TemperatureMsgPayload`
      - Sensed temperature measurement with corruptions

Detailed Module Description
---------------------------

This module simulates the corruption of a true thermal measurement by noise,
bias, and three faults:

- ``TEMP_FAULT_STUCK_VALUE`` is faulty behavior where the measurement sticks to a specific value
- ``TEMP_FAULT_STUCK_CURRENT`` fixes the measurement to the value
- ``TEMP_FAULT_SPIKING`` is faulty behavior where the measurement spikes 
  to a specified multiplier times the actual value, with a given probability
- ``TEMP_FAULT_NOMINAL`` has no faulty behavior but may still have noise and bias

User Guide
----------

Fault Parameters
~~~~~~~~~~~~~~~~

This module has several parameters that are set to default values:

.. list-table:: Default Module Parameters
    :widths: 25 50 25
    :header-rows: 1

    * - Parameter
      - Description
      - Default Value
    * - faultState
      - Sets the fault status.
      - ``TEMP_FAULT_NOMINAL``
    * - senBias
      - Sets the bias value.
      - 0.0
    * - senNoiseStd
      - Sets the standard deviation for sensor noise.
      - 0.0
    * - walkBounds
      - Sets the random walk bounds for sensor noise.
      - 1E-15
    * - stuckValue
      - Temperature at which the reading is stuck for fault mode ``TEMP_FAULT_STUCK_VALUE``.
      - 0.0
    * - spikeProbability
      - Probability of a spike when in fault mode ``TEMP_FAULT_SPIKING``. Between 0 and 1.
      - 0.1
    * - spikeAmount
      - Sensed temperature multiplier when spiking for fault mode ``TEMP_FAULT_SPIKING``.
      - 2.0


Module Setup
~~~~~~~~~~~~

The module is created in python using, for example:

.. code-block:: python
    :linenos:

    tempMeasurementModel = tempMeasurement.TempMeasurement()
    tempMeasurementModel.ModelTag = 'tempMeasModel'

A sample setup is done using:

.. code-block:: python
    :linenos:

    tempMeasurementModel.senBias = 1.0  # [C] bias amount
    tempMeasurementModel.senNoiseStd = 5.0  # [C] noise standard devation
    tempMeasurementModel.walkBounds = 2.0  # 
    tempMeasurementModel.stuckValue = 10.0  # [C] if the sensor gets stuck, stuck at 10 degrees C
    tempMeasurementModel.spikeProbability = 0.3  # [-] 30% chance of spiking at each time step
    tempMeasurementModel.spikeAmount = 10.0  # [-] 10x the actual sensed value if the spike happens

The incomping temperature message must be connected to the module:

.. code-block:: python
    :linenos:

    tempMeasurementModel.tempInMsg.subscribeTo(sensorThermalModel.temperatureOutMsg)

The fault state is changed by the user to spiking, for example, by setting:

.. code-block:: python
    :linenos:

    tempMeasurementModel.faultState = tempMeasurement.TEMP_FAULT_SPIKING
