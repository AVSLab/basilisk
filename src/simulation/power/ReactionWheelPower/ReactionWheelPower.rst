Executive Summary
-----------------
This module evaluates the power draw of a :ref:`reactionWheelStateEffector`.  It reads a message with
the RW states containing the wheel speed and motor torque value.  These are used to evaluate
the power required to change the wheel speed.  In addition, there is a base power draw of the RW device
that accounts for the electrical power required for the RW device just to be on.

This module is a sub-class of :ref:`PowerNodeBase` class.  As such it has the common properties:

1. Writes out a :ref:`PowerNodeUsageMsgPayload` describing its power consumption at each sim update based on its power
   consumption attribute
2. Can be switched on or off using an optional message of type :ref:`DeviceStatusMsgPayload`


Message Connection Descriptions
-------------------------------
This module uses the input and output messages of the :ref:`PowerNodeBase` base class.
The following table lists all the module input and output messages in addition to the base class.
The module message variable name is set by the
user from python.  The message type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. _ModuleIO_RW_POWER:
.. figure:: /../../src/simulation/power/ReactionWheelPower/_Documentation/Images/PowerRW.svg
    :align: center

    Figure 1: ``ReactionWheelPower()`` Module I/O Illustration


.. table:: Module I/O Messages
    :widths: 25 25 100

    +-----------------------+-----------------------------------+---------------------------------------------------+
    | Msg Variable Name     | Msg Type                          | Description                                       |
    +=======================+===================================+===================================================+
    | rwStateInMsg          | :ref:`RWConfigLogMsgPayload`      | RW state input message to provide the reaction    |
    |                       |                                   | wheel speed and motor torque information.         |
    +-----------------------+-----------------------------------+---------------------------------------------------+


Detailed Module Description
---------------------------
Let :math:`p_{\text{RW}}` be the net electrical power required to operate a reaction wheel device.  This power
requirement is computed from the base power requirement to operate the device and the mechancial power requirement
to torque a spinning disk.  Let :math:`p_{\text{base}}` be the base RW device power requirement.  This is a positive
value indicating what power is required to just turn on the RW device, never mind control the wheel speed.

To compute the power required to change the energy state of a RW device the work/energy principle is used as
discussed in `Analytical Mechanics of Space Systems <http://dx.doi.org/10.2514/4.105210>`_.  The mechanical power
required to change the wheel speed relative to the spacecraft body is

.. math::
    :label: eq:prw:1

    p_{\text{mech}} = \Omega \cdot u_s

where :math:`u_s` is the RW motor torque.

Case 1: No Mechanical Energy Recovery
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The most basic RW power behavior assumes that it takes electrical power to both accelerate and decelerate the RW spin
rate. Let the efficiency factor of converting electrical to mechanical power be given by :math:`\eta_{e2m}`.  The net
RW power is then

.. math::
    :label: eq:prw:2

    p_{\text{RW}} = p_{\text{base}} + \frac{| p_{\text{mech}} |}{\eta_{e2m}}

Case 2: Recovering Partially Mechanical Energy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The next case considers the RW device ability to partially recover the mechanical energy when breaking
the RW speed.  Let the mechanical to electrical conversion efficiency factor be :math:`\eta_{m2e}`.  If
the wheel speed is being made larger then :math:`p_{\text{mech}} > 0` and Eq. :eq:`eq:prw:2` is used to evaluate
the net RW power requirement.

If the wheel speed is being made smaller and :math:`p_{\text{mech}} < 0`, then if :math:`\eta_{m2e} >= 0` the model
assumes that breaking the wheel generates some power and thus decreases the net RW power requirement.  This is
evaluated using:

.. math::
    :label: eq:prw:3

    p_{\text{RW}} = p_{\text{base}} +  p_{\text{mech}}  \cdot \eta_{e2m}

Note that :math:`p_{\text{RW}}` could become negative in this situation, illustrating power being returned to the
spacecraft power system.

Case 3: No Power Requirement for Breaking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If the user wants to model a case where breaking the RW speed requires no power, then
simply set :math:`\eta_{m2e}` = 0.

Module Power Output Evaluation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Having computed the net RW power need :math:`p_{\text{RW}}`, next the module power outpute :math:`p_{\text{out}}`
must be determined.  Note that
:math:`p_{\text{RW}}` is typically a positive value, indicating it takes power to run this RW device.  Thus, the
power draw on the network is simply

.. math::
    :label: eq:prw:4

    p_{\text{out}} = -p_{\text{RW}}



Module Assumptions and Limitations
----------------------------------
See :ref:`PowerNodeBase` class for inherited assumption and limitations.  This RW power module assumes a
positive RW power requirement manifests as a negative power draw on the spacecraft power system.  A negative
RW power requirement :math:`p_{\text{RW}}` means the devices is converting mechanical energy back to the power grid.


User Guide
----------

Inheritance
^^^^^^^^^^^
This module inherits the user guide from the :ref:`PowerNodeBase` base class.  See that documentation for features
common to that base class.


Minimum Module Setup
^^^^^^^^^^^^^^^^^^^^
The following code illustrates the minimum module setup within Python assuming the module is
connected to the first RW (thus the ``0`` label)::

    testModule = PowerRW.PowerRW()
    testModule.ModelTag = "bskSat"
    testModule.basePowerNeed = 10.   # baseline power draw, Watts
    unitTestSim.AddModelToTask(unitTaskName, testModule)

The user needs to specify a base power consumption :math:`p_{\text{base}}` through the variable ``basePowerNeed``.
This should be a positive value to reflect the power required just to turn on the RW device, even without
any motor torque commands being applied.

You also need to subscribe to the RW state message with the module variable ``rwStateInMsg``.

This setup will evaluate the RW power using Eq. :eq:`eq:prw:2` where 100% efficiency is assumed in converting
electrical to mechanical energy  with ``elecToMechEfficiency`` = :math:`\eta_{e2m}` = 1, and no electrical energy is recovered
from breaking the wheel speeds with ``mechToElecEfficiency`` = :math:`eta_{m2}` = 1.

Accounting for Non-Ideal Power Conversion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
For example, if 10W of electrical power does not lead to 10W of mechanical power, then this is modeled by setting
the module variable ``elecToMechEfficiency`` to a strictly positive value less than 1.  The value of 1 represents 100% conversion
efficiency and is the default value for this parameter.

To account for harvesting mechanical power during the RW speed braking process, converting mechanical to electrical
power, then the variable :math:`1 \ge` ``mechToElecEfficiency``:math:`\ge 0` must be set to a positive value.  The value of
1 again 100% conversion efficiency (not realisitic).  Typically this is a smaller percentage.

To account that breaking does not require any electrical power, then simply set
``mechToElecEfficiency`` to 0.
