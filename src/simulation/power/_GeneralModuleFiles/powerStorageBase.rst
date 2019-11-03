Executive Summary
-----------------
The PowerStorageBase is a base class that is used generate a standard interface and list of features for modules that store electrical power.  This class is used by other modules as a parent class and cannot be instantiated by itself.  All Basilisk power storage modules based on this PowerStorageBase inherit the following common properties:

1. Writes out a :ref:`PowerStorageStatusSimMsg` containing the current stored power (in Watt-Seconds or Joules), the current net power (in Watts), and the battery storage capacity (in Watt-Seconds or Joules).
2. Allows for multiple :ref:`PowerNodeUsageSimMsg` corresponding to individual :ref:`simPowerNodeBase` instances to be subscribed to using the addPowerNodeToModel method.
3. Iterates through attached :ref:`PowerNodeUsageSimMsg` instances and computes the net power over all messages using ``sumAllInputs()``
4. Computes the conversion between net power in and storage using the ``evaluateBatteryModel`` method, which must be overriden in child classes and is therefore module-specific.

Core functionality is wrapped in the ``evaluateBatteryModel`` protected virtual void method, which is assumed to compute power storage on a module specific mathematical model.

Protected methods prepended with ``custom`` are intended for module developers to override with additional, module-specific functionality.

For more information on how to set up and use classes derived from this module, see the simple power system example: :ref:`scenarioPowerDemo`

Module Assumptions and Limitations
----------------------------------
The base class makes no specific energy storate device related assumptions.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. table:: Module I/O Messages
        :widths: 25 25 100

        +-----------------------+---------------------------------+---------------------------------------------------+
        | Msg Variable Name     | Msg Type                        | Description                                       |
        +=======================+=================================+===================================================+
        | nodePowerUseMsgNames  | :ref:`powerNodeUsageSimMsg`     | Input messages. Vector of power node usage        |
        |                       |                                 | usage messages. Set using ``addPowerNodeToModel`` |
        +-----------------------+---------------------------------+---------------------------------------------------+
        | batPowerOutMsgName    | :ref:`PowerStorageStatusSimMsg` | Output message. Describes battery                 |
        |                       |                                 | capacity, charge level, net power.                |
        +-----------------------+---------------------------------+---------------------------------------------------+

User Guide
----------
- The base class behavior requires the initial energy storage to be specified through ``storedCharge_Init``.
- The integration time step is evaluated as the time between module calls.
- The user must set the output message name variable ``batPowerOutMsgName``
- The input message names are provided by calling the method ``addPowerNodeToModel()``