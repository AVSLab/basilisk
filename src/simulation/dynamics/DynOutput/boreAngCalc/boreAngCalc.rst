
Executive Summary
-----------------

A class to perform a range of boresight related calculations. The module :download:`PDF Description </../../src/simulation/dynamics/DynOutput/boreAngCalc/_Documentation/Basilisk-BOREANGLECALC-20170722.pdf>` contains further information on this module's function, how to run it, as well as testing.

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
      - spacecraft state input message
    * - celBodyInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - (optional) celestial body state msg at which we pointing at
    * - angOutMsg
      - :ref:`BoreAngleMsgPayload`
      - bore sight output message
