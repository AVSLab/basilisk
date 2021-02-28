
Executive Summary
-----------------

This class is an instantiation of the stateEffector abstract class and implements an effector representing a fuel tank. This fuel tank has one state associated with it and is the mass of the fuel inside the tank

The module
:download:`PDF Description </../../src/simulation/dynamics/FuelTank/_Documentation/Basilisk-FUELTANK-20171203.pdf>`
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
    * - fuelTankOutMsg
      - :ref:`FuelTankMsgPayload`
      - fuel tank output message name

