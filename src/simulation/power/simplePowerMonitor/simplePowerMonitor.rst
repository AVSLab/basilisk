Executive Summary
-----------------
The SimplePowerMonitor class is a minimal representation of the PowerStorageBase base class that could represent an electrical battery.  It tracks the integrated net power of a set of attached devices analagous to the behavior of the :ref:`SimpleBattery` module. The main difference is that SimplePowerMonitor does not limit the amount of energy that can be stored.  Thus, it is useful as a power monitor module that simple tracks the positive or negative net energy usage history.  The functionality includes:

1. Compute the integrated and instantaneous net power of all attached devices. Integration is performed with a simple Euler method.

    .. math::

        W_{stored} = \dot{W}_{net} (t_{current} - t_{previous})


Module Assumptions and Limitations
----------------------------------
See :ref:`PowerStorageBase` class for inherited assumption and limitations.  The SimpleBattery class assumes that the net energy storage amount is a fixed value.

Message Connection Descriptions
-------------------------------
This module only uses the input and output messages of the :ref:`PowerStorageBase` base class.  Because this module does not use the storage capacity, the output message sets the variable ``storageCapacity`` to -1.

User Guide
----------
To set up this module users must create a SimplePowerMonitor instance.

.. code-block:: python
    :linenos:

    battery = simplePowerMonitor.SimplePowerMonitor()
    battery.ModelTag = "powerMonitorModel"


The next step is to attach one or more :ref:`PowerNodeUsageMsgPayload` instances to it using the ``addNodeToStorage()`` method.

.. code-block:: python
    :linenos:

    battery.addPowerNodeToModel(powerMsg)

For more information on how to set up and use this module, see the simple power system example: :ref:`scenarioPowerDemo`
