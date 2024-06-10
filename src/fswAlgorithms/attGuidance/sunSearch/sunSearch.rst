Executive Summary
-----------------
This module lets the spacecraft perform three consecutive rotations about the three principal body axes. The axes are user defined and any combination of the three is allowed, including repeated rotations about the same axis. Moreover, the user is required to specify the desired slew time for each rotation, the total slew angle, the maximum angular rate, and maximum deliverable torques about each axis. The module reads in the spacecraft current state and computes a guidance message based on current angular rates and reference angular rates and acceleration. The output of this module should be paired with a control module such as :ref:`mrpPD`. The resulting closed-loop guidance and control strategy is designed such that the output kinematic profile corresponds to a bang-bang type of slew maneuver.


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
    * - attNavInMsg
      - :ref:`NavAttMsgPayload`
      - Input message containing current spacecraft rates in body-frame coordinates.
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - Input message containing the spacecraft inertia tensor about point B, in the body-frame.
    * - attGuidOutMsg
      - :ref:`AttGuidMsgPayload`
      - C++-wrapped output guidance message.
    * - attGuidOutMsgC
      - :ref:`AttGuidMsgPayload`
      - C-wrapped output guidance message.


Detailed Module Description
---------------------------
For each consecutive rotation, this module attempts to meet all the user-defined requirements. However, the problem is inherently overly-prescribed. This section describes the flow of operations that allows to compute the optimal reference rate and acceleration for each principal axis rotation. For each axis, the maximum angular acceleration :math:`\alpha_M` is obtained from the system's principal inertias and the user-specified maximum deliverable torques:

.. math::
    \alpha_{M_i} = \frac{u_i}{I_{ii}} \text{   for   } i=1,2,3.

The subscripts :math:`i` are to indicate each principal body axis, and are dropped from now on for ease of notation. The maximum angular rates about each axis :math:`\omega_M` are user-defined. The total required slew angle about each axis :math:`\theta_R` and the requested slew time :math:`T_R` are also user-defined.

Bang-bang maneuver, no coasting, matching the requested time
+++++++++++++++++++++++++++
First, this module computes a bang-bang maneuver with constant, opposite acceleration in the first and second half of the maneuer. The total maneuver time coincides with :math:`T_R`. The constant acceleration throughout the maneuver is:

.. math::
    \alpha^* = \frac{4 \theta_R}{T_R^2}.

Defining :math:`t_c` as the control time, i.e., the time during which actuator control needs to be delivered in order to accelerate and decelerate the spacecraft, it is in this case :math:`t_c = T_R / 2`. Note that there need to be two identical control time windows in order to ramp the spacecraft's rate up and then down back to zero. The maximum angular rate is reached at :math:`t = T_R/2` and it is :math:`\omega^* = \frac{2\theta_R}{T_R}`.

Bang-bang maneuver, no coasting, exceeding the requested time
+++++++++++++++++++++++++++
If the computed angular acceleration exceeds the maximum acceleration :math:`(\alpha^* > \alpha_M)`, the aforementioned maneuver is infeasible. In this case the acceleration is set to the maximum value :math:`\alpha_M`. The priority is to ensure that the rotation sweeps the required angle :math:`\theta_R` exactly, but in order for this to happend with a lower angular acceleration that what is optima, the maneuver happens in a total slew time :math:`T > T_R`. For a slew maneuver consisting of two accelerated arcs and one coasting arc in the middle, happening at :math:`\omega^*`, the following equation holds true:

.. math::
    \alpha_M {t_c}^2 - \alpha_M T t_c + \theta_R = 0

where, for the controlled time :math:`t_c` to be real, it needs to be :math:`T \geq \sqrt{ \frac{4\theta_R}{\alpha_M} }`. To maintain the slew time as short as possible, the choice is :math:`T = \sqrt{ \frac{4\theta_R}{\alpha_M} }`, where from :math:`\alpha_M < \alpha^*` it follows that :math:`T > T_R`. Solving the second-order equation for the control time gives :math:`t_c = T/2`, ultimately resulting in a bang-bang maneuver with no coasting, whose total duration exceeds the requested maneuver time. The maximum angular rate is reached at :math:`t = T/2` and it is :math:`\omega^* = \frac{2\theta_R}{T}`.

Bang-bang maneuver with coasting
+++++++++++++++++++++++++++
If the maximum angular rate reached during the slew exceeds the maximum admissible rate :math:`(\omega^* > \omega_M)`, the maneuver must be adjusted further. Imposing that the maximum rate is :math:`\omega_M`, the and that the spacecraft stops accelerating and coasts and that rate before decelerating back to zero, gives the following relation:

.. math::
    \theta_R = (T - t_c) \omega_M.

The control time is obtained as :math:`t_c = \frac{\omega_M}{\alpha}` where :math:`\alpha = \min(\alpha_M, \alpha^*)`. Solving the previous equation for the slew time gives:

.. math::
    T = \frac{\theta_R}{\omega_M} + \frac{\omega_M}{\alpha} \geq \sqrt{ \frac{4\theta_R}{\alpha} }

which indicates that adding a coasting arc to meet the requirement on the angular rate further increases the total slew time.


User Guide
----------
The required module configuration is::

    attGuid = sunSearch.SunSearch()
    attGuid.setSlewTime(90, 90, 90)                             # [s]
    attGuid.setSlewAngle(np.pi/2, np.pi, 2*np.pi)               # [rad]
    attGuid.setMaxRate(np.pi/180, 2*np.pi/180, 3*np.pi/180)     # [rad/s]
    attGuid.setMaxTorque(12.5, 25.0, 50)                        # [Nm]
    attGuid.setRotAxis(1, 2, 3)                                 # 1 = 'x', 2 = 'y', 3 = 'z'
