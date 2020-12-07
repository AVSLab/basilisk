Executive Summary
-----------------
This module is intended to serve as a basic power node with a constant power load or draw. Specifically, it:

1. Writes out a :ref:`PowerNodeUsageMsgPayload` describing its power consumption at each sim update based on its power consumption attribute;
2. Can be switched on or off using an optional message of type :ref:`DeviceStatusMsgPayload`.

Module Assumptions and Limitations
----------------------------------
See :ref:`PowerNodeBase` class for inherited assumption and limitations.  The power draw or supply for this module is assumed to be constant.

Message Connection Descriptions
-------------------------------
This module only uses the input and output messages of the :ref:`PowerNodeBase` base class.

User Guide
----------
This module inherits the user guide from the :ref:`PowerNodeBase` base class.

    For more information on how to set up and use this module, see the simple power system example: :ref:`scenarioPowerDemo`
