
This module reads in the desired impulse that each thruster must produce to create inertial momentum change to despin the RWs.

The output of the module is a setup of thruster firing times.  Each thruster can only fire for a maximum time that matches a single control period.  After this the thrusters are off for an integer number of control periods to let the RW re-stabilize the attitude about an inertial pointing scenario. The module
:download:`PDF Description </../../src/fswAlgorithms/effectorInterfaces/thrMomentumDumping/_Documentation/Basilisk-thrMomentumDumping-20160820.pdf>` contains further information on this module's function, how to run it, as well as testing.


