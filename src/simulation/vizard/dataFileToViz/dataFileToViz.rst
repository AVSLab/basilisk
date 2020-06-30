
Executive Summary
-----------------
This module reads in simulation data of one or more spacecraft, likely created outside of Basilisk,
and creates associated Basilisk messages such that :ref:`vizInterface` can stream of save a :ref:`Vizard <vizard>`
compatible data file.  This makes it possible to use Vizard to illustrate a simulation.  The use of this module
is demonstrates in :ref:`scenarioDataToViz`.

Module Assumptions and Limitations
----------------------------------
The module currently reads in the spacecraft position, velocity and orientation states.

Message Connection Descriptions
-------------------------------
The following messages are set directly within ``vizInterface``.  Additional messages are set within the
``VizSpacecraftData`` data structures for each spacecraft.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - scStateOutMsgNames
      - :ref:`SCPlusStatesSimMsg`
      - Vector of spacecraft names. These also become the output message names



User Guide
----------
The module assumes the data file is in plain text form and the following format:

- time
- inertial position states (m)
- inertial velocity states (m/s)
- inertial attitude state in terms of either MRPs, quaternions or 3-2-1 Euler angles (rad)
- inertial angular velocity vector in radians per second (rad/s)
- (optional) thruster force values (N)
- repeat on the same line for additional spacecraft

The required module parameters are listed in the following table.

.. list-table:: Module Required Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Type
     - Description
   * - ``dataFileName``
     - string
     - Absolute path to the simulation data file
   * - ``scStateOutMsgNames``
     - Vector of strings
     - List of spacecraft names.  The state messages will have the same name.

The module is configurable with the following optional parameters:

.. list-table:: Module Optional Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``numSatellites``
     - 1
     - Number of satellites being simulated
   * - ``delimiter``
     - " "
     - delimiter string that separates data on a line
   * - ``convertPosToMeters``
     - 1000.0
     - conversion factor to convert position and velocity measures to meters and meters per second.
   * - ``headerLine``
     - True
     - Boolean flag if the data file contains a header line that should be dismissed
   * - ``attitudeType``
     - 0
     - Specify the attitude coordinate set used in the data file.  0 - MRP, 1 - quaternions as :math:`(q_0, q_1, q_2, q_3)`,
       and 2 - (3-2-1) Euler angles in radians
   * - ``thrMsgDataSC``
     - empty
     - vector of spacecraft thruster configuration vectors.  Each element contains a ThrClusterMap container
       (defined in :ref:`vizStructures`).
       This allows for each spacecraft to have distinct sets of thrusters to be included.
       See :ref:`test_dataFileToViz` for an example on how to configure for thruster information.





