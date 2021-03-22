Executive Summary
-----------------

Module that acts as an encoder to the reaction wheel speeds. It models both the discretization of the speed measurement, as well as signal status such as nominal, off and stuck.

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
    * - rwSpeedInMsg
      - :ref:`RWSpeedMsgPayload`
      - Input reaction wheel speed message.
    * - rwSpeedOutMsg
      - :ref:`RWSpeedMsgPayload`
      - Output reaction wheel speed message.

Detailed Module Description
---------------------------

This module simulates two different encoder features: discretization and signal status. Discretization truncates the output reaction wheel speed to a set resolution. As for the signal status, it can be set as ``SIGNAL_NOMINAL`` for normal working conditions, but can also simulate failures
of the type ``SIGNAL_OFF`` (output wheel speed is 0) and ``SIGNAL_STUCK`` (output wheel speed is constant and equal to the one given at the previous iteration).

This module applies each of its features to each wheel individually, and so it is important to set the number of reaction wheels in use. It is also important to note that, because
for the first iteration the time step is set at 0, the wheel speeds will not be discretized for the first time step and the encoder will output the true wheel speeds.

Discretization
~~~~~~~~~~~~~~

Under normal operating conditions, the discretizer works by setting the wheel speed as a multiple of the resolution of the sensor. This resolution is calculated through the number of 
clicks per rotation that the sensor can handle. Let :math:`N` be the number of clicks measured during a time step:

.. math::
    N = \texttt{trunc}(\Omega_{\text{in}}\Delta t \frac{n}{2\pi} + \Delta N)

where :math:`\Omega_{\text{in}}` is the input wheel speed, :math:`\Delta t` is the time step, :math:`n` is the number of clicks per rotation and :math:`\Delta N` is the remaining clicks that were not accounted for
in the previous iteration. The ``trunc()`` function truncates the result, effectively rounding down to the nearest integer. The output wheel speed :math:`\Omega_{\text{out}}` is computed using the
following equation:

.. math::
    \Omega_{\text{out}} = N\frac{2\pi}{n\Delta t}

The remaining clicks value is also updated as follows:

.. math::
    \Delta N = \Omega_{\text{in}}\Delta t \frac{n}{2\pi} + \Delta N - N

As expected, the smaller the value of :math:`n`, the greater the discretization errors will be. Also, the discretization will be more noticeable when the wheel speeds are close to 0.

Signal Status
~~~~~~~~~~~~~

The module accepts three different signal status, which are ``SIGNAL_NOMINAL``, ``SIGNAL_OFF`` and ``SIGNAL_STUCK``. When the signal is nominal, all the previous features apply and the wheel
speeds are only affected by the discretization. When the signal is off, both the wheel speed and the remaining clicks are set to zero, which models a shutdown of the encoder.
Finally, when the signal is stuck, all the variables remain constant from the previous iteration, including the wheel speeds and the remaining clicks.


Model Functions
---------------

The functions of the encoder model are:

- **Discretization:** Discretizes the wheel speeds in terms of the given resolution.
- **Signal Status:** Changes the output wheel speed depending on the encoder signal for each wheel.


Model Assumptions and Limitations
---------------------------------

This code makes the following assumptions:

- **Wheel speed is constant for each time step:** A simple Euler integration is used to compute the number of clicks.
- **All encoders precision are the same:** The encoder module aplies the same resolution to each reaction wheel.


User Guide
----------

This section contains conceptual overviews of the code and clear examples for the prospective user.

Module Setup
~~~~~~~~~~~~

The encoder module is created in python using:

.. code-block:: python
    :linenos:

    wheelSpeedEncoder = encoder.Encoder()
    wheelSpeedEncoder.ModelTag = 'rwSpeedsEncoder'

A sample setup is done using:

.. code-block:: python
    :linenos:

    wheelSpeedEncoder.clicksPerRotation = 2048
    wheelSpeedEncoder.numRW = numRW
