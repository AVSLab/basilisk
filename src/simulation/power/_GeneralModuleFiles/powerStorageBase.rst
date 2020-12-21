Executive Summary
-----------------
The PowerStorageBase is a base class that is used generate a standard interface and list of features for modules that store electrical power.  This class is used by other modules as a parent class and cannot be instantiated by itself.  All Basilisk power storage modules based on this PowerStorageBase inherit the following common properties:

1. Writes out a :ref:`PowerStorageStatusMsgPayload` containing the current stored power (in Watt-Seconds or Joules), the current net power (in Watts), and the battery storage capacity (in Watt-Seconds or Joules).
2. Allows for multiple :ref:`PowerNodeUsageMsgPayload` corresponding to individual :ref:`powerNodeBase` instances to be subscribed to using the addPowerNodeToModel method.
3. Iterates through attached :ref:`PowerNodeUsageMsgPayload` instances and computes the net power over all messages using ``sumAllInputs()``
4. Computes the conversion between net power in and storage using the ``evaluateBatteryModel`` method, which must be overridden in child classes and is therefore module-specific.

Core functionality is wrapped in the ``evaluateBatteryModel`` protected virtual void method, which is assumed to compute power storage on a module specific mathematical model.

Protected methods prepended with ``custom`` are intended for module developers to override with additional, module-specific functionality.

For more information on how to set up and use classes derived from this module, see the simple power system example: :ref:`scenarioPowerDemo`

Module Assumptions and Limitations
----------------------------------
The base class makes no specific energy storage device related assumptions.

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
    * - batPowerOutMsg
      - :ref:`PowerStorageStatusMsgPayload`
      - power storage status output message
    * - nodePowerUseInMsgs
      - :ref:`PowerNodeUsageMsgPayload`
      - vector of power node input messages (these are linked through the ``addPowerNodeToModel(msg)`` method


User Guide
----------
- The base class behavior requires the initial energy storage to be specified through ``storedCharge_Init``.
- The integration time step is evaluated as the time between module calls.
- The user must set the output message name variable ``batPowerOutMsg``
- The input message names are provided by calling the method ``addPowerNodeToModel(msg)``