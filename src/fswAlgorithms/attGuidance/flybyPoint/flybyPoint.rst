Executive Summary
-----------------
This module computes a reference attitude frame for a spacecraft in relative motion about a small body. The implicit assumption is that the small body's mass does not perturb the motion of the spacecraft significantly. Conceptually, this module is equivalent to :ref:`hillPoint`, but for the relative motion of a spacecraft about a body that is not the main center of gravity.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - transNavInMsg
      - :ref:`NavTransMsgPayload`
      - Input message containing the relative position and velocity of the spacecraft with respect to the small body, estimated from a filter.
    * - ephemerisInMsg
      - :ref:`EphemerisMsgPayload`
      - Input message containing the inertial position of the small body. This is needed only when the flyby is modeled using the Clohessy-Wiltshire equations.
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - Output attitude reference message containing reference attitude, reference angular rates and accelerations.


Detailed Module Description
---------------------------
The relative position and velocity vector of the spacecraft with respect to the small body are obtained as noisy estimates. Therefore the desire is, for this module, to only read the filter message every so often. The input parameter ``dtFilterData`` allows the user to specify the desired time interval between two subsequent reads of the filter output. For every call of this module that happens between two consecutive filter reads, the reference attitude needs to be propagated from the last filter read according to a dynamic model of the flyby.

Rectilinear Motion Model
........................
In this case the flyby is modeled as rectilinear motion of the spacecraft, i.e., the spacecraft moves with a constant velocity. At every filter read, the relative position and velocity vectors :math:`\boldsymbol{r}_0` and :math:`\boldsymbol{v}_0` of the spacecraft with respect to the small body are provided. The following coefficients are defined: the flight path angle :math:`\gamma_0` of the spacecraft, and the ratio between velocity and radius magnitudes :math:`f_0 = \frac{v_0}{r_0}`. From these quantities, the rotation of the reference frame is given by the following equations:

.. math::
    \theta(t) = \arctan \left( \tan \gamma_0 + \frac{f_0}{\cos \gamma_0} t \right) - \gamma_0
.. math::
    \dot{\theta}(t) = \frac{f_0 \cos \gamma_0}{f_0^2 t^2 + 2 f_0 \sin \gamma_0 t + 1}
.. math::
    \ddot{\theta}(t) = -2 f_0^2 \cos \gamma_0 \frac{f_0t + \sin \gamma_0}{(f_0^2 t^2 + 2 f_0 \sin \gamma_0 t + 1)^2}

where :math:`t` is the time passes since the last filter read. Note that using the flight path angle :math:`gamma_0` makes these equation always nonsingular. :math:`\theta(t)` is used to compute the additional frame rotation from the Hill frame computed at the read time. Such rotation happens about the angular momentum direction vector. :math:`\dot{\theta}(t)` and :math:`\ddot{\theta}(t)` projected onto the angular momentum direction vector give the angular rate and acceleration vectors of the reference frame.


Clohessy-Wiltshire Equations Model
..................................
T.B.D.


Module Assumptions and Limitations
----------------------------------
The limitations of this module are inherent to the geometry of the problem, which determines whether or not all the constraints can be satisfied. For example, as shown in  in R. Calaon, C. Allard and H. Schaub, "Attitude Reference Generation for Spacecraft with Rotating Solar Arrays and Pointing Constraints," In preparation for Journal of Spacecraft and Rockets, depending on the relative orientation of :math:`{}^\mathcal{B}h` and :math:`{}^\mathcal{B}a_1`, it may not be possible to  achieve perfect incidence angle on the solar arrays. Only when perfect incidence is obtained, it is possible to solve for the solution that also drives the body-fixed direction :math:`{}^\mathcal{B}a_2` close to the Sun. When perfect incidence is achievable, two solutions exist. If :math:`{}^\mathcal{B}a_2` is provided as input, this is used to determine which solution to pick. If this input is not provided, one of the two solution is chosen arbitrarily.

Due to the difficulty in developing an analytical formulation for the reference angular rate and angular acceleration vectors, these are computed via second-order finite differences. At every time step, the current reference attitude and time stamp are stored in a module variable and used in the following time updates to compute angular rates and accelerations via finite differences.


User Guide
----------
The required module configuration is::

    flybyGuid = flybyPoint.FlybyPoint()
    flybyWrap.ModelTag = "flybyPoint"
    flybyGuid.dtFilterData = 60
    flybyGuid.signOfOrbitNormalFrameVector = 1
    unitTestSim.AddModelToTask(unitTaskName, flybyGuid)
	
The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``dtFilterData``
     - 0
     - time between two consecutive filter reads. If defaulted to zero, the filter information is read at every update call
   * - ``signOfOrbitNormalFrameVector``
     - 1
     - Sign of the orbit normal rxv vector used to build the frame. If equal to 1, the frame is a traditional Hill frame if -1, it flips the orbit normal axis to point "down" relative to the orbtial momentum
   * - ``flybyModel``
     - 0
     - 0 for rectilinear flyby model, 1 for Clohessy-Wiltshire model

