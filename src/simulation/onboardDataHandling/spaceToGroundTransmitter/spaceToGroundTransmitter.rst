Executive Summary
-----------------
This module provides a model of access-dependent data downlink from a simple transmitter at a fixed baud rate. Specifically, the transmitter reads a :ref:`DataStorageStatusSimMsg` from the storage unit it is subscribed to, searches for the data buffer with the maximum amount of data, and downlinks a packet of data at a fixed baud rate. The transmitter writes out a :ref:`DataNodeUsageSimMsg` describing the data name and baud rate of the data it wants to downlink.

In comparison to :ref:`SimpleTransmitter`, downlink is only enabled when a :ref:`AccessSimMsg` indicates that the spacecraft for which the transmitter is associated has access to an object that can provide it (such as a :ref:`GroundLocation`.)

For more information on how to set up and use this module, see the simple data system example: :ref:`scenarioDataDemo`

Module Assumptions and Limitations
----------------------------------
This module makes no additional assumptions outside of those already made in the :ref:`DataNodeBase` base class.

Message Connection Descriptions
-------------------------------
This module uses the input and output messages of the :ref:`DataNodeBase` base class, plus an additional :ref:`DataStorageStatusSimMsg` input message subscribed to and read in `customCrossInit()` and `customRead()` methods, respectively.

.. table:: Module I/O Messages
    :widths: 25 25 100

    +-----------------------+---------------------------------+---------------------------------------------------+
    | Msg Variable Name     | Msg Type                        | Description                                       |
    +=======================+=================================+===================================================+
    | nodeDataOutMsgName    | :ref:`DataNodeUsageSimMsg`      | Writes out the data name and amount used/generated|
    |                       |                                 | by a DataNodeBase instance.                       |
    +-----------------------+---------------------------------+---------------------------------------------------+
    | deviceStatusInMsgName | :ref:`DeviceStatusIntMsg`       | (optional). If dataStatus is 0,                   |
    |                       |                                 | the node is disabled; other values indicate       |
    |                       |                                 | various data modes depending on the module.       |
    +-----------------------+---------------------------------+---------------------------------------------------+
    | storageUnitMsgNames   | :ref:`DataStorageStatusSimMsg`  | Vector of storage units that are connected        |
    |                       |                                 | to the transmitter. Add storage unit with the     |
    |                       |                                 | ``addStorageUnitToTransmitter`` method.           |
    +-----------------------+---------------------------------+---------------------------------------------------+
    | groundAccessMsgNames  | :ref:`AccessSimMsg`             | Vector of access messages available               |
    |                       |                                 | to the transmitter. Add access msg.  with the     |
    |                       |                                 | ``addAccessMsgToTransmitter`` method.             |
    +-----------------------+---------------------------------+---------------------------------------------------+

User Guide
----------
This module inherits the user guide from the :ref:`DataNodeBase` base class, but there are several key differences from other :ref:`DataNodeBase` child classes:

- Unlike other child classes of :ref:`DataNodeBase`, this module does not require a user to set a `nodeDataName`
- The `nodeBaudRate` variable should be set to a negative value in order to remove data from the storage unit.
- The user must specify a `packetSize` variable (negative value) in addition to the `nodeBaudRate` variable
- The user must specify the number of buffers `numBuffers` variable so the transmitter knows how many buffers to search through
- The user must specify the storage unit the transmitter should subscribe to using the `addStorageUnitToTransmitter()`` method

To set up this module users must create a SimpleTransmitter instance::

   transmitter = spaceToGroundTransmitter.SpaceToGroundTransmitter()
   transmitter.ModelTag = "transmitter"

Set the `nodeBaudRate`, `packetSize`, and numBuffers variables::

   transmitter.nodeBaudRate = -16000. # baud
   transmitter.packetSize = -1E6 # bits
   transmitter.numBuffers = 2

The next step is to attach one or more :ref:`DataStorageStatusSimMsg` instances to it using the ``addStorageUnitToTransmitter()`` method::

   transmitter.addStorageUnitToTransmitter("msg name")

Next, attach available :ref:`AccessSimMsg` instances using the ``addAccessMsgToTransmitter()`` method::

    transmitter.addAccessMsgToTransmitter("msg name")

The final step is to specify the output message name and add the model to task::

    transmitter.nodeDataOutMsgName = "TransmitterMsg"
    scenarioSim.AddModelToTask(taskName, transmitter)

Follow the :ref:`partitionedStorageUnit` or :ref:`simpleStorageUnit` instructions to add the transmitter to a storage unit.

For more information on how to set up and use this module, see the simple data system example: :ref:`scenarioDataDemo`
