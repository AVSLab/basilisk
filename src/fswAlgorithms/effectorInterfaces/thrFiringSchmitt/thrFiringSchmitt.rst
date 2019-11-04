
A Schmitt trigger logic is implemented to map a desired thruster force value into a thruster on command time.

The module reads in the attitude control thruster force values for both on- and off-pulsing scenarios, and then maps this into a time which specifies how long a thruster should be on.  The thruster configuration data is read in through a separate input message in the reset method.  The Schmitt trigger allows for an upper and lower bound where the thruster is either turned on or off. More information can be found in the
:download:`PDF Description </../../src/fswAlgorithms/effectorInterfaces/thrFiringSchmitt/_Documentation/Basilisk-thrFiringSchmitt-2019-03-29.pdf>`.
The paper `Steady-State Attitude and Control Effort Sensitivity Analysis of Discretized Thruster Implementations <https://doi.org/10.2514/1.A33709>`__ includes a detailed discussion on the Schmitt Trigger and compares it to other thruster firing methods.
