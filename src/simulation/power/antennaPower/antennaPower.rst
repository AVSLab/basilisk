Executive Summary
-----------------
This module evaluates the power draw of an antenna device. It reads the ``antennaSetStateInMsg`` containing the current antenna state and calculates the power draw accordingly.

This module is a sub-class of :ref:`PowerNodeBase`. As such it:

1. Writes out a :ref:`PowerNodeUsageMsgPayload` describing its power consumption at each sim update
2. Can be switched on or off using an optional message of type :ref:`DeviceStatusMsgPayload`


Message Connection Descriptions
-------------------------------

.. _ModuleIO_ANTENNA_POWER:
.. figure:: /../../src/simulation/power/antennaPower/_Documentation/Images/PowerAnt.svg
    :align: center

    Figure 1: ``antennaPower()`` Module I/O Illustration


.. table:: Module I/O Messages
    :widths: 25 25 100

    +-----------------------+-----------------------------------+-----------------------------------------------+
    | Msg Variable Name     | Msg Type                          | Description                                   |
    +=======================+===================================+===============================================+
    | antennaSetStateInMsg  | :ref:`AntennaLogMsgPayload`       | Antenna state input message providing current |
    |                       |                                   | antenna state and power values.               |
    +-----------------------+-----------------------------------+-----------------------------------------------+


Detailed Module Description
---------------------------
The antenna power consumption depends on its operational state:

- **ANTENNA_OFF**: Only base power is consumed (default 0.0 W)
- **ANTENNA_RX**: Base power and receive power ``P_Rx`` are consumed
- **ANTENNA_TX**: Base power and transmit power ``P_Tx`` are consumed
- **ANTENNA_RXTX**: Base power and both ``P_Rx`` and ``P_Tx`` power are consumed

The net power output is:

.. math::

    p_{\text{out}} = -(p_{\text{base}} + p_{\text{mode}})

where :math:`p_{\text{mode}}` is zero, ``P_Rx``, ``P_Tx``, or ``P_Rx + P_Tx`` depending on the antenna state.

User Guide
----------

This module must be connected to an antenna module that outputs the :ref:`AntennaLogMsgPayload` message.

Minimum Module Setup
^^^^^^^^^^^^^^^^^^^^
::

    antPower = antennaPower.AntennaPower()
    antPower.ModelTag = "antennaPower"
    antPower.basePowerNeed = 5.0  # [W] baseline power draw (optional, default = 0.0 W)
    antPower.antennaSetStateInMsg.subscribeTo(antenna.antennaLogOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, antPower)

The ``basePowerNeed`` parameter specifies the base power consumption in Watts when the antenna is powered on (should be non-negative).
