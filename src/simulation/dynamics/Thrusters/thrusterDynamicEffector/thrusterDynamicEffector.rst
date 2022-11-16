
Executive Summary
-----------------

Thruster dynamics class used to provide thruster effects on body.  This class is used to hold and operate a set of thrusters that are located
on the spacecraft.  It contains all of the configuration data for the thruster
set, reads an array of on-time requests (double precision in seconds).  It is
intended to be attached to the dynamics plant in the system using the
DynEffector interface and as such, does not directly write the current force
or torque into the messaging system.  The nominal interface to dynamics are the
dynEffectorForce and dynEffectorTorque arrays that are provided by the DynEffector base class.
There is technically double inheritance here, but both the DynEffector and
SysModel classes are abstract base classes so there is no risk of diamond.

The module
:download:`PDF Description </../../src/simulation/dynamics/Thrusters/thrusterDynamicEffector/_Documentation/Basilisk-THRUSTERS20170712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

.. danger::

    This thruster module is not compatible with variable time step integrators.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdsInMsg
      - :ref:`THRArrayOnTimeCmdMsgPayload`
      - (optional) input message with thruster commands. If not connected the thruster commands are set to zero.
    * - thrusterOutMsgs
      - :ref:`THROutputMsgPayload`
      - output message vector for thruster data

.. note::
  The dynamic behaviour of this module is governed by the variables inside :ref:`THRTimePair`, which determine the on and off-ramp characteristics. The default behaviour is to not have on and off-ramps active. The ``cutoffFrequency`` variable inside :ref:`THRSimConfig` has no impact on this module and is instead supposed to be used to determine the dynamic behaviour within :ref:`thrusterStateEffector`.
