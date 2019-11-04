
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
:download:`PDF Description </../../src/simulation/dynamics/Thrusters/_Documentation/Basilisk-THRUSTERS20170712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


