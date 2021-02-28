Executive Summary
-----------------

Sensor model to simulate an IMU.

The module
:download:`PDF Description </../../src/simulation/sensors/imuSensor/_Documentation/Basilisk-IMU-20170712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.
The corruption types are outlined in this
:download:`PDF Description </../../src/simulation/sensors/imuSensor/_Documentation/BasiliskCorruptions.pdf>`.


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
    * - scStateInMsg
      - :ref:`SCStatesMsgPayload`
      - input essage name for spacecraft state
    * - sensorOutMsg
      - :ref:`IMUSensorMsgPayload`
      - output message name for IMU output data

