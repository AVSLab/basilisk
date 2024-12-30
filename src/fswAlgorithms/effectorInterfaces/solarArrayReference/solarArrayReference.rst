Executive Summary
-----------------

This module is used to calculate the required rotation angle for a solar array that is able to rotate about its drive axis. The degree of freedom associated with the rotation of the array about the drive axis makes it such that it is possible to improve the incidence angle between the sun and the array surface, thus ensuring maximum power generation.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - hingedRigidBodyRefOutMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Output Hinged Rigid Body Reference Message.
    * - attNavInMsg
      - :ref:`NavAttMsgPayload`
      - Input Attitude Navigation Message.
    * - attRefInMsg
      - :ref:`AttRefMsgPayload`
      - Input Attitude Reference Message.
    * - hingedRigidBodyInMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Input Hinged Rigid Body Message Message.
    * - vehConfigInMsg (optional)
      - :ref:`VehicleConfigMsgPayload`
      - Input vehicle configuration message containing the position of the center of mass of the system.
    * - rwConfigDataInMsg (optional)
      - :ref:`RWArrayConfigMsgPayload`
      - Input message containing the number of reaction wheels, relative inertias and orientations with respect to the body frame.
    * - rwSpeedsInMsg (optional)
      - :ref:`RWSpeedMsgPayload`
      - Input message containing the relative speeds of the reaction wheels with respect to the hub.


Module Assumptions and Limitations
----------------------------------
This module computes the rotation angle required to achieve the best incidence angle between the Sun direction and the solar array surface. This does not mean that
perfect incidence (Sun direction perpendicular to array surface) is guaranteed. This module assumes that the solar array has only one surface that is able to generate power. This bounds the output reference angle :math:`\theta_R` between :math:`0` and :math:`2\pi`. Perfect incidence is achievable when the solar array drive direction and the Sun direction are perpendicular. Conversely, when they are parallel, no power generation is possible, and the reference angle is set to the current angle, to avoid pointless energy consumption attempting to rotate the array.

The Sun direction in body-frame components is extracted from the ``attNavInMsg``. The output reference angle :math:`\theta_R`, however, can be computed either based on the reference attitude contained in ``attRefInMsg``, or the current spacecraft attitude contained also in ``attNavInMsg``. This depends on the frequency with which the arrays need to be actuated, in comparison with the frequency with which the motion of the spacecraft hub is controlled. The module input ``attitudeFrame`` allows the user to set whether to compute the reference angle based on the reference attitude or current spacecraft attitude.


Detailed Module Description
---------------------------
For this module to operate, the user needs to provide two unit directions as inputs:

- :math:`{}^\mathcal{B}\boldsymbol{\hat{a}}_1`: direction of the solar array drive, about which the rotation happens;
- :math:`{}^\mathcal{B}\boldsymbol{\hat{a}}_2`: direction perpendicular to the solar array surface, with the array at a zero rotation.

The same math applies to the case where the body reference is used. In that case, the same vectors are expressed in body-frame coordinates. Note that the unit directions :math:`\boldsymbol{\hat{a}}_i` have the same components in both the body and reference frame, because they are body-fixed and rotate with the spacecraft hub.

Some logic is implemented such that the computed reference angle :math:`\theta_R` and the current rotation angle :math:`\theta_C` received as input from the ``hingedRigidBodyInMsg`` are never more than 360 degrees apart.

The derivative of the reference angle :math:`\dot{\theta}_R` is computed via finite differences.

Maximum Power Generation
+++++++++++++++++++++++++++
To compute the reference rotation that maximizes Sun incidence :math:`\theta_{\text{Sun,}R}`, the module computes the unit vector :math:`{}^\mathcal{R}\boldsymbol{\hat{a}}_2`, which is coplanar with
:math:`{}^\mathcal{B}\boldsymbol{\hat{a}}_1` and the Sun direction :math:`{}^\mathcal{R}\boldsymbol{\hat{r}}_S`. This is obtained as:

.. math::
    {}^\mathcal{R}\boldsymbol{a}_2 = {}^\mathcal{R}\boldsymbol{\hat{r}}_S - ({}^\mathcal{R}\boldsymbol{\hat{r}}_S \cdot {}^\mathcal{B}\boldsymbol{\hat{a}}_1) {}^\mathcal{B}\boldsymbol{\hat{a}}_1

and then normalizing to obtain :math:`{}^\mathcal{R}\boldsymbol{\hat{a}}_2`. The reference angle :math:`\theta_{\text{Sun,}R}` is the angle between :math:`{}^\mathcal{B}\boldsymbol{\hat{a}}_2` and :math:`{}^\mathcal{R}\boldsymbol{\hat{a}}_2`:

.. math::
    \theta_{\text{Sun,}R} = \arccos ({}^\mathcal{B}\boldsymbol{\hat{a}}_2 \cdot {}^\mathcal{R}\boldsymbol{\hat{a}}_2).


Momentum Dumping
+++++++++++++++++++++++++++
In this pointing mode, the reference angle is computed in order to leverage solar radiation pressure (SRP) to produce a torque on the spacecraft opposing the local net reaction wheel momentum (:math:`\boldsymbol{H}`). This functionality applies to a set of two rotating solar arrays whose rotation axes are opposite to one another, and it consists in articulating the two arrays differentially in order to generate a net SRP torque.

With respect to a ''zero rotation'' configuration, where zero rotation consists in having the power-generating surface of the array directly facing the Sun, the desire is to drive one of the two arrays at an offset inclination angle :math:`\theta` with respect to the Sun direction. This offset is defined as:

.. math::
    \theta = \left\{ \begin{array}[c|c|c] \\ \Theta_\text{max} \cdot \arctan(\sigma \nu^n) & \text{if} & \nu \geq 0 \\ 0 & \text{if} & \nu < 0 \\  \end{array} \right.


where :math:`\Theta_\text{max}`, :math:`\sigma`, and :math:`n` are user-defined parameters, and :math:`\nu` is defined as:

.. math::
    \nu = - (\boldsymbol{r}_{O/C} \times \boldsymbol{\hat{r}}_\text{S}) \cdot \boldsymbol{H}_\text{RW}

where :math:`\boldsymbol{r}_{O/C}` is the location of the array center of pressure with recpect to the system CM, :math:`\boldsymbol{\hat{r}}_\text{S}` is the direction of the Sun wirh respect to the spacecraft, and :math:`\boldsymbol{H}_\text{RW}` is the net wheel momentum.

The parameter :math:`\Theta_\text{max}` allows to specify a maximum deflection of the array with respect to the Sun, in order to prevent it from ever reaching an edge-on configuration. The parameter :math:`\sigma` is a tuning gain, whether the exponent :math:`n > 1` allows to introduce a deadband around the zero rotation to avoid chattering.

For more details on the mathematical derivation and stability considerations, see R. Calaon, C. Allard and H. Schaub, "Momentum Management of a Spacecraft equipped with a Dual-Gimballed Electric Thruster", currently in preparation for submission to the Journal of Spacecraft and Rockets.


User Guide
----------
The required module configuration is::

    saReference = solarArrayReference.solarArrayReference()
    saReference.ModelTag = "solarArrayReference"
    saReference.a1Hat_B = [1, 0, 0]
    saReference.a2Hat_B = [0, 0, 1]
    saReference.attitudeFrame = 0
    saReference.pointingMode = 0
    saReference.ThetaMax = np.pi/2
    saReference.sigma = 1
    saReference.n = 1
    unitTestSim.AddModelToTask(unitTaskName, saReference)

The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 34 66
   :header-rows: 1

   * - Parameter
     - Description
   * - ``a1Hat_B``
     - solar array drive direction in B-frame coordinates
   * - ``a2Hat_B``
     - solar array zero-rotation direction, in B-frame coordinates
   * - ``attitudeFrame``
     - 0 for reference angle computed w.r.t reference frame; 1 for reference angle computed w.r.t. body frame; defaults to 0 if not specified
   * - ``pointingMode``
     - 0 for maximum power generation; 1 maximum momentum dumping; defaults to 0 if not specified
   * - ``ThetaMax``
     - between 0 and Pi
   * - ``sigma``
     - tuning gain; setting to zero removes momentum management capability
   * - ``n``
     - n > 1 introduces a deadband around zero rotation.
