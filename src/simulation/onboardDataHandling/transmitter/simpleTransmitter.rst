Executive Summary
-----------------
This module provides first-order modeling of data downlinking from a simple transmitter at a fixed baud rate. Specifically, the transmitter reads a :ref:`DataStorageStatusMsgPayload` from the storage unit it is subscribed to, searches for the data buffer with the maximum amount of data, and downlinks a packet of data at a fixed baud rate. The transmitter writes out a :ref:`DataNodeUsageMsgPayload` describing the data name and baud rate of the data it wants to downlink.

For more information on how to set up and use this module, see the simple data system example: :ref:`scenarioDataDemo`

Module Assumptions and Limitations
----------------------------------
This module makes no additional assumptions outside of those already made in the :ref:`DataNodeBase` base class.


Message Connection Descriptions
-------------------------------
This module uses the input and output messages of the :ref:`DataNodeBase` base class, plus an additional
:ref:`DataStorageStatusMsgPayload` input message subscribed to and read in `customRead()` method.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - storageUnitInMsgs
      - :ref:`DataStorageStatusMsgPayload`
      - vector of data storage input messages.  These are set using the ``addStorageUnitToTransmitter`` method


User Guide
----------
This module inherits the user guide from the :ref:`DataNodeBase` base class, but there are several key differences from other :ref:`DataNodeBase` child classes:

- Unlike other child classes of :ref:`DataNodeBase`, this module does not require a user to set a `nodeDataName`
- The `nodeBaudRate` variable should be set to a negative value in order to remove data from the storage unit.
- The user must specify a `packetSize` variable (negative value) in addition to the `nodeBaudRate` variable
- The user must specify the number of buffers `numBuffers` variable so the transmitter knows how many buffers to search through
- The user must specify the storage unit the transmitter should subscribe to using the `addStorageUnitToTransmitter()`` method

To set up this module users must create a SimpleTransmitter instance::

   transmitter = simpleTransmitter.SimpleTransmitter()
   transmitter.ModelTag = "transmitter"

Set the `nodeBaudRate`, `packetSize`, and numBuffers variables::

   transmitter.nodeBaudRate = -16000. # baud
   transmitter.packetSize = -1E6 # bits
   transmitter.numBuffers = 2

The next step is to attach one or more :ref:`DataStorageStatusMsgPayload` instances to it using the ``addStorageUnitToTransmitter()`` method::

   transmitter.addStorageUnitToTransmitter("msg name")

The final step is to add the model to task::

    scenarioSim.AddModelToTask(taskName, transmitter)

Follow the :ref:`partitionedStorageUnit` or :ref:`simpleStorageUnit` instructions to add the transmitter to a storage unit.

For more information on how to set up and use this module, see the simple data system example: :ref:`scenarioDataDemo`
