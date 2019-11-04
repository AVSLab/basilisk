
This module maps a desired torque to control the spacecraft, and maps it to the available wheels using a minimum norm inverse fit.

The optional wheel availability message is used to include or exclude particular reaction wheels from the torque solution.  The desired control torque can be mapped onto particular orthogonal control axes to implement a partial solution for the overall attitude control torque.  More information can be found in the
:download:`PDF Description </../../src/fswAlgorithms/effectorInterfaces/rwMotorTorque/_Documentation/Basilisk-rwMotorTorque-20190320.pdf>`.



