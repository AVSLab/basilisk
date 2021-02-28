Executive Summary
-----------------
This module provides first-order modeling of data generation from a simple instrument at a fixed baud rate. Specifically, it writes out a :ref:`DataNodeUsageMsgPayload` describing its data name and baud rate at each time step.

For more information on how to set up and use this module, see the simple data system example: :ref:`scenarioDataDemo`

Module Assumptions and Limitations
----------------------------------
This module only uses the input and output messages of the DataNodeBase base class.

Message Connection Descriptions
-------------------------------
There are no additional module input messages beyond those specified in :ref:`DataNodeBase`.

User Guide
----------
To set up this module users must create a SimpleInstrument instance::

   simpleInstrument = simpleInstrument.SimpleInstrument()
   simpleInstrument.ModelTag = "instrument1"

Set the `nodeBaudRate` and `nodeDataName` variables::

   instrument.nodeBaudRate = 1200. # baud
   instrument.nodeDataName = "Instrument 1" # baud

The final step is to add the model to task::

    scenarioSim.AddModelToTask(taskName, instrument)

Follow the :ref:`partitionedStorageUnit` or :ref:`simpleStorageUnit` instructions to add the instrument to a storage unit.

For more information on how to set up and use this module, see the simple data system example: :ref:`scenarioDataDemo`
