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
