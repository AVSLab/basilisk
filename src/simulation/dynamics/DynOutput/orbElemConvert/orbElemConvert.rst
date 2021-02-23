
Executive Summary
-----------------

An orbital element/cartesian position and velocity converter.
The module :download:`PDF Description </../../src/simulation/dynamics/DynOutput/boreAngCalc/_Documentation/Basilisk-BOREANGLECALC-20170722.pdf>` contains further information on this module's function, how to run it, as well as testing.

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
    * - scStateInMsg
      - :ref:`SCStatesMsgPayload`
      - Spacecraft state input message
    * - spiceStateInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Spice state input message
    * - elemInMsg
      - :ref:`ClassicElementsMsgPayload`
      - classical orbit elements input message
    * - scStateOutMsg
      - :ref:`SCStatesMsgPayload`
      - Spacecraft state output message
    * - spiceStateOutMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Spice state output message
    * - elemOutMsg
      - :ref:`ClassicElementsMsgPayload`
      - classical orbit elements output message


.. warning::

    Only one of the input messages can be connected at the same time.  However, you can have
    1 or more output message connected simultaneously.
