
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
    * - scStateOutMsgs
      - :ref:`SCStatesMsgPayload`
      - Vector of spacecraft output messages
    * - thrScOutMsgs
      - :ref:`THROutputMsgPayload`
      - (optional) vector of vectors of thruster output messages per spacecraft
    * - rwScOutMsgs
      - :ref:`RWConfigLogMsgPayload`
      - (optional) vector of vectors of RW output messages per spacecraft



User Guide
----------
The module assumes the data file is in plain text form and the following format:

- time
- inertial position states (m)
- inertial velocity states (m/s)
- inertial attitude state in terms of either MRPs, quaternions or 3-2-1 Euler angles (rad)
- inertial angular velocity vector in radians per second (rad/s)
- (optional) thruster force values (N)
- (optional) RW Speed :math:`\Omega` (rad/s) and RW motor torque :math:`u_s` (N)
- repeat on the same line for additional spacecraft

The required module configuration is::

    testModule = dataFileToViz.DataFileToViz()
    testModule.ModelTag = "testModule"
    testModule.setNumOfSatellites(2)
    testModule.dataFileName = "dataFile.dat"
    unitTestSim.AddModelToTask(unitTaskName, testModule)

Note that ``setNumOfSatellites()`` must be called with at least 1 spacecraft.

The module is configurable with the following optional parameters:

.. list-table:: Module Optional Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
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
     - Specify the attitude coordinate set used in the data file.  0 - MRP, 1 - quaternions
       as :math:`(q_0, q_1, q_2, q_3)`, and 2 - (3-2-1) Euler angles in radians

To add Thrusters to the setup, for each of the spacecraft included do the following steps.  The spacecraft
can contain a number of thruster clusters defined through ``ThrClusterMap``.  In the examle below, the
spacecraft contains 2 clusters (ADCS and DV) which contain one thruster each.

.. code-block:: python

    # setup thruster cluster for the current spacecraft
    thSetAdcs1 = dataFileToViz.ThrClusterMap()
    # set the number of thruster in this cluster
    thSetAdcs1.thrCount = 1
    # set the thruster cluster tag label string
    thSetAdcs1.thrTag = "adcs_sc_0"
    # (Optional) set the color for the thruster visualization in this cluster.
    thSetAdcs1.color = vizSupport.toRGBA255("red")

    thSetDV1 = dataFileToViz.ThrClusterMap()
    thSetDV1.thrCount = 1
    thSetDV1.thrTag = "dv_sc_0"
    thSetDV1.color = vizSupport.toRGBA255("blue")

    # assign this thruster cluster to module
    thList1 = [thSetAdcs1, thSetDV1]
    testModule.appendThrClusterMap(dataFileToViz.VizThrConfig(thList1))

    # add the position and orientation information for each thruster in this cluster
    # ADCS1
    testModule.appendThrPos([0, 0, 3.])  # thr location in B frame, meters
    testModule.appendThrDir([0, 0, -1])  # thr force direction in B frame
    testModule.appendThrForceMax(1.0)
    # DV1
    testModule.appendThrPos([0, 0, -3.])  # thr location in B frame, meters
    testModule.appendThrDir([0, 0, 1])  # thr force direction in B frame
    testModule.appendThrForceMax(1.0)

These steps must be done for each spacecraft in the data file.  If a spacecraft does not have thrusters, then
an empty thruster cluster vector must be added for that spacecraft.
See :ref:`test_dataFileToViz` for an example on how to configure for thruster information.

.. code-block:: python

    testModule.appendThrClusterMap([])

To add RW devices to the list, for each spacecraft you must specify the number of RW that it contains through::

    testModule.appendNumOfRWs(2)

Next, the RW position, spin axis direction, the wheel speed and the maximum motor torque value is setup using::

    testModule.appendRwPos([0, 0, 0])
    testModule.appendRwDir([1, 0, 0])
    testModule.appendOmegaMax(3000.*macros.RPM)
    testModule.appendUMax(0.5)

Repeat the above steps for each wheel.

