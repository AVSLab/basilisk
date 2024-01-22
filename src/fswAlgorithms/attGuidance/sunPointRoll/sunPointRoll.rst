Executive Summary
-----------------
This module computes a reference attitude for a Sun-pointing spacecraft. The reference is defined such that the body-frame-defined axis :math:`{}^\mathcal{B}\hat{h}_\text{ref}` is aligned with the direction of the Sun relative to the spacecraft. The third degree of freedom to uniquely define the reference frame is chosen arbitrarily. A roll rate can be defined by the user such that the reference frame rolls about the sunline at a constant rate. This ensures constant pointing towards the Sun while mitigating solar radiation pressure effects.


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
    * - attRefOutMsg
      - :ref:`AttRefMsgPayload`
      - Output attitude reference message containing reference attitude, reference angular rates and accelerations.


Detailed Module Description
---------------------------
Two of the three degrees of freedom associated with the definition of a reference frame are determined by the condition to align the desired axis with the Sun direction. The third degree of freedom is determined using the J2000.0 :math:`z` axis, which in Basilisk coincides with the third axis of the inertial frame :math:`\mathcal{N}`.

The user is required to define the solar array drive axis :math:`{}^\mathcal{B}\hat{a}_1` to help with the definition of a right-handed frame. When this information is not available or does not apply to the spacecraft, this can be any axis that is not collinear with :math:`{}^\mathcal{B}\hat{h}_\text{ref}`.


User Guide
----------
The required module configuration is::

    attGuid = sunPointRoll.SunPointRoll()
    attGuid.hRefHat_B = hRefHat_B
    attGuid.a1Hat_B = a1Hat_B
    attGuid.omegaRoll = omegaRoll
    attGuid.ModelTag = "sepPoint"

The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``hRefHat_B``
     - [0, 0, 0]
     - body-frame axis that needs to point at the Sun
   * - ``a1Hat_B``
     - [0, 0, 0]
     - solar array drive direction in body-frame coordinates
   * - ``omegaRoll``
     - 0
     - roll rate about the sunline axis.
