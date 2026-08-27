Executive Summary
-----------------
Models the addition of noise, bias, and faults to temperature measurements.

Message Connection Descriptions
-------------------------------
The following table lists all module input and output messages. The module msg connection
is set by the user from python. The msg type contains a link to the message structure definition,
while the description provides information on what this message is used for.

.. bsk-module-io:: tempMeasurement
    :caption: Module I/O Messages

    input tempInMsg TemperatureMsgPayload
        True temperature measurement.
    output tempOutMsg TemperatureMsgPayload
        Sensed temperature measurement with corruptions.

Detailed Module Description
---------------------------

This module simulates the corruption of a true thermal measurement by noise,
bias, and three faults:

- ``TEMP_FAULT_STUCK_VALUE`` is faulty behavior where the measurement sticks to a specific value
- ``TEMP_FAULT_STUCK_CURRENT`` fixes the measurement to the value
- ``TEMP_FAULT_SPIKING`` is faulty behavior where the measurement spikes
  to a specified multiplier times the actual value, with a given probability
- ``TEMP_FAULT_NOMINAL`` has no faulty behavior but may still have noise and bias

The sensor noise state follows

.. math::

    e_{k+1} = a e_k + \sigma z_k,
    \qquad z_k \sim \mathcal{N}(0, 1),

where ``senNoiseStd`` configures :math:`\sigma` and ``setAMatrix()`` configures
the one-by-one propagation matrix :math:`a`. The propagation matrix defaults to
zero, so setting ``senNoiseStd`` alone produces independent white Gaussian
measurement noise with the requested standard deviation. ``RNGSeed`` controls
the repeatable random sequence.

A correlated process or random walk must be configured explicitly. For
example, identity propagation with a positive ``walkBounds`` creates a bounded
random walk::

    tempMeasurementModel.setAMatrix([[1.0]])
    tempMeasurementModel.walkBounds = 10.0  # [C]

A positive ``walkBounds`` value is an exact hard bound on the noise state. A
non-positive value disables clipping. Bias and noise are applied before fault
handling. The stuck-value and stuck-current faults replace the noisy, biased
measurement, while the spiking fault multiplies it by ``spikeAmount`` when a
spike occurs.

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
      - Sets an optional hard bound on the sensor noise state.
      - -1.0 (disabled)
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
    tempMeasurementModel.senNoiseStd = 5.0  # [C] white-noise standard deviation
    tempMeasurementModel.stuckValue = 10.0  # [C] if the sensor gets stuck, stuck at 10 degrees C
    tempMeasurementModel.spikeProbability = 0.3  # [-] 30% chance of spiking at each time step
    tempMeasurementModel.spikeAmount = 10.0  # [-] 10x the actual sensed value if the spike happens

The incoming temperature message must be connected to the module:

.. code-block:: python
    :linenos:

    tempMeasurementModel.tempInMsg.subscribeTo(sensorThermalModel.temperatureOutMsg)

The fault state is changed by the user to spiking, for example, by setting:

.. code-block:: python
    :linenos:

    tempMeasurementModel.faultState = tempMeasurement.TEMP_FAULT_SPIKING
