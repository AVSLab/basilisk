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


Maximum Momentum Dumping
+++++++++++++++++++++++++++
In this pointing mode, the reference angle is computed in order to leverage solar radiation pressure (SRP) to produce a torque whose component in the opposite direction to the local net reaction wheel momentum (:math:`\boldsymbol{H}`) is maximum. Because the array can only rotate about the :math:`\boldsymbol{\hat{a}}_1` axis, the desired solar array normal :math:`\boldsymbol{\hat{y}}` can be expressed as follows, in the solar array frame :math:`\mathcal{A}`:

.. math::
    {}^\mathcal{A}\boldsymbol{\hat{y}} = \{0, \cos t, \sin t \}^T.

The dot product between the solar array torque and the net wheel momentum is the function to minimize:

.. math::
    f(t) = \boldsymbol{L} \cdot \boldsymbol{H} = (\boldsymbol{\hat{s}} \cdot \boldsymbol{\hat{y}})^2 (\boldsymbol{r} \times (-\boldsymbol{\hat{y}})) \cdot \boldsymbol{H} = (\boldsymbol{\hat{s}} \cdot \boldsymbol{\hat{y}})^2 (\boldsymbol{h} \cdot \boldsymbol{\hat{y}})

where :math:`\boldsymbol{h} = \boldsymbol{r} \times \boldsymbol{H}` and :math:`\boldsymbol{r}` is the position of the array center of pressure with respect to the system's center of mass. Taking the derivative with respect to :math:`t` and equating it to zero results in the third-order equation in :math:`\tan t`:

.. math::
    (s_2 \cos t + s_3 \sin t) \left[(2s_2h_3+s_3h_2) \tan^2 t -3(s_3h_3-s_2h_2) \tan t - (2s_3h_2+s_2h_3) \right] = 0

whose desired solution is:

.. math::
    \theta_{\text{Srp,}R} = t = \frac{3(s_3h_3-s_2h_2) - \sqrt{\Delta}}{4s_3h_2+2s_2h_3}.

In the presence of two solar arrays, it is not desirable to maneuver both to the angle that minimizes :math:`f(t)`. This is because one array will always reach an edge-on configuration, thus resulting in a SRP torque imbalance between the arrays. For this reason, the array(s) are maneuvered to the reference angle

.. math::
    \theta_R = \theta_{\text{Sun,}R} - f(t) \left( \theta_{\text{Srp,}R} - \theta_{\text{Sun,}R} \right)

to ensure that both arrays remain, on average, pointed at the Sun. When one array has the ability to generate a lot of dumping torque (:math:`f(t) \rightarrow -1`), its reference angle is skewed towards :math:`\theta_{\text{Srp,}R}`. Conversely, when :math:`f(t) \rightarrow 0` and there is not much momentum dumping capability, the array remains close to the maximum power-generating angle :math:`\theta_{\text{Sun,}R}`.

For more details on the mathematical derivation, see R. Calaon, C. Allard and H. Schaub, "Momentum Management of a Spacecraft equipped with a Dual-Gimballed Electric Thruster", currently in preparation for submission to the Journal of Spacecraft and Rockets.


User Guide
----------
The required module configuration is::

    saReference = solarArrayReference.solarArrayReference()
    saReference.ModelTag = "solarArrayReference"
    saReference.a1Hat_B = [1, 0, 0]
    saReference.a2Hat_B = [0, 0, 1]
    saReference.attitudeFrame = 0
    saReference.pointingMode = 0
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
