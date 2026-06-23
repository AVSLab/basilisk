
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

.. bsk-module-io:: fuelTank
    :caption: Module I/O Messages

    output fuelTankOutMsg FuelTankMsgPayload
        fuel tank output message name

User Guide
----------
The fuel tank effector module must be passed a tank model

.. code-block:: python

    fuelTankEffector = fuelTank.FuelTank()
    tankModel = fuelTank.FuelTankModelConstantVolume()
    fuelTankEffector.setTankModel(tankModel)

Fuel tank configuration values should be set and read with the module setter and getter methods. Direct Python access to
the legacy public variables ``nameOfMassState``, ``dcm_TB``, ``r_TB_B``, ``updateOnly``, and ``fuelLeakRate`` is
deprecated and raises a deprecation warning.

The configurable values are:

- ``setNameOfMassState()`` / ``getNameOfMassState()``: optional state name used when registering the fuel tank mass
  state. Set this before simulation initialization if a custom state name is required.
- ``setDcm_TB()`` / ``getDcm_TB()``: direction cosine matrix from the body frame ``B`` to the tank frame ``T``.
- ``setR_TB_B()`` / ``getR_TB_B()``: position vector from the body-frame origin to the tank point, expressed in body-frame
  components in meters.
- ``setUpdateOnly()`` / ``getUpdateOnly()``: flag selecting update-only mass depletion. The default value is ``True``.
  Set this to ``False`` to include the additional mass-depletion back-substitution contributions.
- ``setFuelLeakRate()`` / ``getFuelLeakRate()``: positive fuel mass flow rate leaving the tank in kg/s. This leak rate is
  added to any attached thruster fuel consumption and reduces the reported fuel mass without applying force or torque to
  the spacecraft. The leak stops when the available tank propellant reaches zero.

.. code-block:: python

    fuelTankEffector.setNameOfMassState("fuelTankMass1")
    fuelTankEffector.setDcm_TB([[1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0]])
    fuelTankEffector.setR_TB_B([[0.0], [0.0], [0.1]])  # [m]
    fuelTankEffector.setUpdateOnly(True)
    fuelTankEffector.setFuelLeakRate(1.0e-5)  # [kg/s]

A thruster effector can be attached to a tank effector to simulate fuel mass depletion by thruster operation.

.. code-block:: python

    fuelTankEffector.addThrusterSet(thrustersEffector)
