Executive Summary
-----------------
This module computes a reference attitude frame that simultaneously satisfies multiple pointing constraints. The first constraint consists of aligning a body-frame direction :math:`{}^\mathcal{B}\hat{h}_1` with a certain inertial reference direction :math:`{}^\mathcal{N}\hat{h}_\text{ref}`. This locks two out of three degrees of freedom that characterize a rigid body rotation. The second constraint consists in achieving maximum power generation on the solar arrays, assuming that the solar arrays can rotate about their drive axis. This condition is obtained ensuring that the body-fixed solar array drive direction :math:`{}^\mathcal{B}\hat{a}_1` is as close to perpendicular as possible to the Sun direction. The third constraint consists in keeping a certain body-fixed, Sun-constrained direction, :math:`{}^\mathcal{B}\hat{a}_2`, within an angle :math:`beta` from the Sun direction.

The second and third constraint can be switched in priority. When solar array incidence is prioritized, there is no guarantee that the Sun-constrained axis can be kept within a certain angular distance from the Sun. Conversely, when the Sun-constrained direction is prioritized, the algorithm searches the space of compliant solutions for the solution that also maximizes Sun incidence on the solar arrays.


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
      - Input message containing current attitude and Sun direction in body-frame coordinates. Note that, for the Sun direction to appear in the message, the :ref:`SpicePlanetStateMsgPayload` must be provided as input msg to :ref:`simpleNav`, otherwise the Sun direction is zeroed by default.
    * - bodyHeadingInMsg
      - :ref:`BodyHeadingMsgPayload`
      - Input message containing the body-frame direction :math:`{}^\mathcal{B}\hat{h}`.
    * - inertialHeadingInMsg
      - :ref:`InertialHeadingMsgPayload`
      - Input message containing the inertial-frame direction :math:`{}^\mathcal{N}\hat{h}_\text{ref}`.
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - Output attitude reference message containing reference attitude, reference angular rates and accelerations.


Detailed Module Description
---------------------------
A detailed mathematical derivation of the equations applied by this module can be found in R. Calaon, C. Allard and H. Schaub, "Attitude Reference Generation for Spacecraft with Rotating Solar Arrays and Pointing Constraints", in preparation for Journal of Spacecraft and Rockets.

The input parameter ``alignmentPriority`` allows to choose which between the second and the third constraint are being prioritized. Regardless of the chosen ``alignmentPriority``, the reference attitude aligns the body-frame and the inertial-frame headings contained in the message before any other constraint.

When ``alignmentPriority = 0``, the reference attitude makes it such that the solar array drive axis :math:`{}^\mathcal{B}\hat{a}_1` is as perpendicular as possible to the Sun direction. When perfectly perpendicular, two compliant solution exist, therefore the module picks the one that keeps the Sun-constrained direction :math:`{}^\mathcal{B}\hat{a}_2` closer to the Sun.

When ``alignmentPriority = 0``, the reference attitude makes it such that the Sun-constrained vector :math:`{}^\mathcal{B}\hat{a}_2` is never more than :math:`\beta` degrees apart from the Sun direction. Within this range of admissible solutions, the module picks the one that also result in maximum incidence on the solar arrays, i.e., the one that drives the solar array drive axis :math:`{}^\mathcal{B}\hat{a}_1` as perpendicular as possible to sunlight.


Module Assumptions and Limitations
----------------------------------
The limitations of this module are inherent to the geometry of the problem, which determines which of the constraints can be satisfied. For example, as shown in  in R. Calaon, C. Allard and H. Schaub, "Attitude Reference Generation for Spacecraft with Rotating Solar Arrays and Pointing Constraints," In preparation for Journal of Spacecraft and Rockets, depending on the relative orientation of :math:`{}^\mathcal{B}h` and :math:`{}^\mathcal{B}a_1`, it may not be possible to  achieve perfect incidence angle on the solar arrays. Only when perfect incidence is obtained, it is possible to solve for the solution that also drives the body-fixed direction :math:`{}^\mathcal{B}a_2` close to the Sun. When perfect incidence is achievable, two solutions exist. If :math:`{}^\mathcal{B}a_2` is provided as input, this is used to determine which solution to pick. If this input is not provided, one of the two solution is chosen arbitrarily.

Due to the difficulty in developing an analytical formulation for the reference angular rate and angular acceleration vectors, these are computed via second-order finite differences. At every time step, the current reference attitude and time stamp are stored in a module variable and used in the following time updates to compute angular rates and accelerations via finite differences.


User Guide
----------
The required module configuration is::

    attGuid = sepPoint.SepPoint()
    attGuid.a1Hat_B = a1Hat_B
    attGuid.a2Hat_B = a2Hat_B
    attGuid.beta = beta
    attGuid.alignmentPriority = alignmentPriority
    attGuid.ModelTag = "sepPoint"
	
The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``a1Hat_B``
     - [0, 0, 0]
     - solar array drive direction in body-frame coordinates
   * - ``a2Hat_B`` (optional)
     - [0, 0, 0]
     - Sun-constrained direction in body frame coordinates
   * - ``beta`` (optional)
     - [0, 0, 0]
     - half-cone angle of the keep-in Sun constraint
   * - ``alignmentPriority``
     - 0
     - 0 to prioritize incidence on the arrays, 1 to prioritize Sun-constrained direction.