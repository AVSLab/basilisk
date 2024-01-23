Executive Summary
-----------------
DataStorageUnitBase is a base class that is used generate a standard interface and list of features for modules that store simulated onboard data.  This class is used by other modules as a parent class and cannot be instantiated by itself.  All Basilisk data storage modules based on this DataStorageUnitBase inherit the following common properties:

1. Writes out a :ref:`DataStorageStatusMsgPayload` containing the sum of the current stored data (in bits), the storage capacity (bits), the current net data rate (in baud), an array of char array containing the names of the stored data (ex. Instrument 1, Instrument 2), and an array of integers containing the stored data associated with each type (bits).
2. Allows for multiple :ref:`DataNodeUsageMsgPayload` corresponding to individual :ref:`dataNodeBase` instances to be subscribed to using the ``addDataNodeToModel(msg)`` method.
3. Iterates through attached :ref:`DataNodeUsageMsgPayload` instances, integrates the data for each data node, and adds it to its respective entry using ``integrateDataStatus()`` method, which may be overwritten in child classes.
4. Loops through the vector of storedData to sum the total amount of data contained within the storage unit.
5. Add or remove data from specified partitions using ``setDataBuffer()`` method.

Core functionality is wrapped in the ``integrateDataStatus`` protected virtual void method, which computes the amount of data stored in a storage unit on a module basis. This base class automatically implements a partitioned storage unit (different data buffers for each device). See :ref:`simpleStorageUnit` for an example of how this functionality can be overwritten.

Protected methods prepended with ``custom`` are intended for module developers to override with additional, module-specific functionality.

For more information on how to set up and use classes derived from this module, see the simple data system example: :ref:`scenarioDataDemo`

Module Assumptions and Limitations
----------------------------------
The base class makes no specific data storage device related assumptions.

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
    * - nodeDataUseInMsgs
      - :ref:`dataNodeUsageMsgPayload`
      - Vector of data node usage input messages. Subscribed to using ``addDataNodeToModel(msg)``
    * - storageUnitDataOutMsg
      - :ref:`DataStorageStatusMsgPayload`
      - Output message. Describes storage unit capacity, storage level, net data rate, and contents.


User Guide
----------
- The user can connect to the output message using ``storageUnitDataOutMsg`` in Python.
- The input message (data nodes) are provided by calling the method ``addDataNodeToModel()``