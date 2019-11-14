
A thruster force message is read in and converted to a thruster on-time output message. The module ensures the requested on-time is at least as large as the thruster's minimum on time.  If not then the on-time is zeroed, but the unimplemented thrust time is kept as a remainder calculation.  If these add up to reach the minimum on time, then a thruster pulse is requested.  If the thruster on time is larger than the control period, then an on-time that is 1.1 times the control period is requested. More information can be found in the
:download:`PDF Description </../../src/fswAlgorithms/effectorInterfaces/thrFiringRemainder/_Documentation/Basilisk-thrFiringRemainder-2019-03-28.pdf>`.
The paper `Steady-State Attitude and Control Effort Sensitivity Analysis of Discretized Thruster Implementations <https://doi.org/10.2514/1.A33709>`__ includes a detailed discussion on the Remainder Trigger algorithm and compares it to other thruster firing methods.




