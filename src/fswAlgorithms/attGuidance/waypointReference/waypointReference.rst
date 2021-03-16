Executive Summary
-----------------

Module that reads the reorientation maneuver of a spacecraft from a text file, likely created outside of Basilisk, and outputs an 
Attitude Reference Message. This module makes it possible to reproduce on Basilisk attitude orientation maneuvers computed externally.


Module Assumptions and Limitations
----------------------------------
The module assumes that the text file is written in a compatible form. This means that times, attitude parameters, angular rates and
accelerations need to be provided in this exact order for the module to read che correct information. Each line of the text file
must contain the information relative to one and only one intermediate point along the maneuver.
The module is conceptually very simple and makes no major assumptions. However, the user might want to use a sampling frequency in the
Basilisk simulation that is equal or higher than the frequency with which the waypoints are given in the text file. For lower sampling
frequencies, the module output does not give a trustworthy representation of the maneuver.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - Output Attitude Reference Message.
	  

Detailed Module Description
---------------------------
The module reads a sequence of time-tagged waypoints. Defining :math:`t=[t_0,...,t_N]` the times of the N+1 waypoints, and :math:`t_{sim}` the simulation time, we have that:

- for :math:`t_{sim} < t_0`: the attitude is held constant to the attitude of the first waypoint; angular rates and acceleration are kept at zero;
- for :math:`t_0 \leq t_{sim} \leq t_N`: attitude, angular rates and accelerations are the result of linear interpolation between the closest two waypoints;
- for :math:`t_{sim} > t_N`: the attitude is held constant to the attitude of the last waypoint; angular rates and acceleration are kept at zero.
		
		
User Guide
----------
The module assumes the data file is in plain text form and the following format:

- time
- attitude parameters (MRPs or EPs)
- angular rates (rad/s) either expressed in inertial frame or reference frame
- angular accelerations (rad/s^2) either expressed in inertial frame or reference frame

where each line contains information about only one intermediate point of the maneuver.


The required module configuration is::

    waypointReferenceConfig = waypointReference.WaypointReference()
    waypointReferenceConfig.ModelTag = "waypointReference"
    waypointReferenceConfig.dataFileName = dataFileName
    waypointReferenceConfig.attitudeType = 0
    unitTestSim.AddModelToTask(unitTaskName, waypointReferenceConfig)
	
Note that for ``attitudeType``, a valid input must be provided by the user: 0 - MRP, 1 - EP or quaternions (q0, q1, q2, q3), 2 - EP or quaternions (q1, q2, q3, qs).
No default attitude type is used by the module, therefore faliure to specify this parameter results in breaking the simulation.

The module is configurable with the following optional parameters:

.. list-table:: Module Optional Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``delimiter``
     - ","
     - delimiter string that separates data on a line
   * - ``useReferenceFrame``
     - false
     - if true, reads angular rates and accelerations in the reference frame instead of inertial frame
   * - ``headerLines``
     - 0
     - number of header lines in the data file that should be ignored before starting to read in the waypoints
