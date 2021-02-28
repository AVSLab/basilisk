
Executive Summary
-----------------

Radiation pressure dynamics class used to compute

The module
:download:`PDF Description </../../src/simulation/dynamics/RadiationPressure/_Documentation/Basilisk-RadiationPressure-20170712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - sunEphmInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - sun state input message
    * - sunEclipseInMsg
      - :ref:`EclipseMsgPayload`
      - (optional) sun eclipse input message















