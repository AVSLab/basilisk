Executive Summary
-----------------

The SimpleBattery class is a minimal model of battery functionality that considers:

1. Integrated net input power of the attached modules
2. The battery's maximum storage capacity as defined by the ``storageCapacity`` attribute.
   Integration of the net input power is performed with a simple Euler method.

     :math:`W_{stored} = \dot{W}_{net} (t_{current} - t_{previous})`


Module Assumptions and Limitations
----------------------------------
See :ref:`PowerStorageBase` class for inherited assumption and limitations.  The SimpleBattery class assumes that the net energy storage amount is a fixed value.

Message Connection Descriptions
-------------------------------
This module only uses the input and output messages of the :ref:`PowerStorageBase` base class.

User Guide
----------

To set up this module users must create a SimpleBattery instance::

   battery = simpleBattery.SimpleBattery()
   battery.ModelTag = "batteryModel"

In addition to the variables that must be set for the :ref:`PowerStorageBase` base class, this module requires the ``storageCapacity`` attribute to be specified.  The total power stored in the battery will be limited to not exceed this capacity value::

   battery.storageCapacity = 10.0 # Given in Joules or Watt-seconds

The next step is to attach one or more :ref:`PowerNodeUsageMsgPayload` instances to it using the ``addNodeToStorage()`` method::

   battery.addPowerNodeToModel(powerMsg)


For more information on how to set up and use this module, see the simple power system example :ref:`scenarioPowerDemo`.

Users may configure the fault message to simulate a battery capacity fault that reduces the actual storage capacity while the ``storageCapacity`` value remains unchanged. The faulted battery capacity is determined using the ``faultCapacityRatio``, calculated as (actual capacity) / (set capacity). ::

   faultMsg = messaging.PowerStorageFaultMsgPayload()
   faultMsg.faultCapacityRatio = 0.3 # Actual capacity is 30% of the nominal capacity
   faultStatusMsg = messaging.PowerStorageFaultMsg().write(faultMsg)
   battery.batteryFaultInMsg.subscribeTo(faultStatusMsg)
