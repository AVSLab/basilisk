
Executive Summary
-----------------

This class is an instantiation of the stateEffector abstract class and implements an effector representing a fuel tank.
This fuel tank has one state associated with it and is the mass of the fuel inside the tank. The module is a fuel tank
effector attached to a rigid body hub and has the following functions:

- Compute tank properties depending on the tank model being used
- Provides its contributions to the mass properties of the spacecraft
- Provides its contributions to the back-substitution matrices
- Computes its derivative for its mass flow rate using the vector of attached thrusters
- Provides its contributions to energy and momentum of the spacecraft

The module
:download:`PDF Description </../../src/simulation/dynamics/FuelTank/_Documentation/Basilisk-FUELTANK-20171203.pdf>`
contains further information on this module's computations.


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

User Guide
----------
The fuel tank effector module must be passed a tank model

.. code-block:: python

    fuelTankEffector = fuelTank.FuelTank()
    tankModel = fuelTank.FuelTankModelConstantVolume()
    fuelTankEffector.setTankModel(tankModel)

A thruster effector can be attached to a tank effector to simulate fuel mass depletion by thruster operation.

.. code-block:: python

    fuelTankEffector.addThrusterSet(thrustersEffector)