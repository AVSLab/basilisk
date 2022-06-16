Executive Summary
-----------------
This module receives a vector of accessMsgPayloads and outputs a vector of DataNodeUsageMsgPayloads for each accessible
point. This module is meant to keep track of which points have been imaged, and which have not, passing the output messages
to a :ref:`partitionedStorageUnit` module.

Module Assumptions and Limitations
----------------------------------
This module assumes a single baudRate for all mapping points. This module provides a zero baudRate for points that
are not accessible.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - accessInMsgs
      - :ref:`AccessMsgPayload`
      - vector of ground location input access messages
    * - dataNodeOutMsgs
      - :ref:`DataNodeUsageMsgPayload`
      - vector of data node output messages

User Guide
----------
To use this module, the user must first instantiate the module

.. code-block:: python

    mapInstrument = mappingInstrument.mappingInstrument()
    mapInstrument.ModelTag = "mapInstrument"

The user must then set the nodeBaudRate, in bits/second, within the module. This is a required parameter.

.. code-block:: python

    mapInstrument.nodeBaudRate = 1

The mapping points and names should then be added to the module one at a time. The access message is the ``accessMsg``
associated with the point. The ``pointName`` is the name of the point.

.. code-block:: python

    mapInstrument.addMappingPoint(accessMsg, pointName)

Finally, logs for every mapping point can be created as follows. In this example, N = 1:

.. code-block:: python

    dataLogs = []
    for idx in range(0, N):
        dataLogs.append(mapInstrument.dataNodeOutMsgs[idx].recorder())
        scSim.AddModelToTask(simTaskName, dataLogs[idx])
