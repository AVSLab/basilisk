Executive Summary
-----------------

This class is designed to model the state of a single coarse sun sensor
attached to a spacecraft.  It emulates the "counts" that will typically be
output by the ADC on board of a spacecraft.

The module
:download:`PDF Description </../../src/simulation/sensors/coarseSunSensor/_Documentation/Basilisk-CoarseSunSensor-20170803.pdf>`
contains further information on this module's function,
how to run it, as well as testing.
The corruption types are outlined in this
:download:`PDF Description </../../src/simulation/sensors/imuSensor/_Documentation/BasiliskCorruptions.pdf>`.

.. warning::
  Be careful when generating CSS objects in a loop or using a function! It is often
  convenient to initialize many CSS' with the same attributes using a loop with a
  function like ``setupCSS(CSS)`` where ``setupCSS`` initializes the field of view,
  sensor noise, min / max outputs, etc. To prevent the CSS object from being garbage collected,
  you need to keep a reference to it by storing it on your simulation object. For example:

.. code-block:: python

    # Create and configure CSS
    css = coarseSunSensor.CoarseSunSensor()
    setupCSS(css)
    # Store reference on simulation object to keep it alive
    scSim.css = css  # This prevents Python from garbage collecting the CSS object

The coarse sun sensor module also supports user-enabled faulty behavior by assigning
a values to the ``.faultState`` member of a given sensor. An example of such call
would is ``cssSensor.faultState = coarse_sun_sensor.CSSFAULT_OFF`` where ``cssSensor``
is an instantiated sensor, and ``coarse_sun_sensor`` is the imported module.

The module currently supports the following faults:

.. list-table:: ``CoarseSunSensor`` Fault Types
    :widths: 35 50
    :header-rows: 1

    * - Fault Enumeration
      - Description
    * - ``CSSFAULT_OFF``
      - CSS no longer provides a signal (output = 0)
    * - ``CSSFAULT_STUCK_CURRENT``
      - CSS signal is stuck on value from past timestep
    * - ``CSSFAULT_STUCK_MAX``
      - CSS signal is stuck on the maximum value
    * - ``CSSFAULT_STUCK_RAND``
      - CSS signal is stuck on a random value
    * - ``CSSFAULT_RAND``
      - CSS produces random values



Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. _ModuleIO_CSS:
.. figure:: /../../src/simulation/sensors/coarseSunSensor/_Documentation/Images/moduleDiagramCSS.svg
    :align: center

    Figure 1: ``CoarseSunSensor()`` Module I/O Illustration

.. list-table:: ``CoarseSunSensor`` Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - sunInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - input message for sun data
    * - stateInMsg
      - :ref:`SCStatesMsgPayload`
      - input message for spacecraft state
    * - cssDataOutMsg
      - :ref:`CSSRawDataMsgPayload`
      - output message for CSS output data
    * - cssConfigLogOutMsg
      - :ref:`CSSConfigLogMsgPayload`
      - output message for CSS configuration log data
    * - sunEclipseInMsg
      - :ref:`EclipseMsgPayload`
      - (optional) input message for sun eclipse state message
    * - albedoInMsg
      - :ref:`AlbedoMsgPayload`
      - (optional) input message for albedo message

.. _ModuleIO_CSS_Constellation:
.. figure:: /../../src/simulation/sensors/coarseSunSensor/_Documentation/Images/moduleDiagramConstellation.svg
    :align: center

    Figure 2: ``CSSConstellation()`` Module I/O Illustration


.. list-table:: ``CSSConstellation`` Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - constellationOutMsg
      - :ref:`CSSArraySensorMsgPayload`
      - CSS constellation output message
