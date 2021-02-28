Executive Summary
-----------------
This module provides first-order modeling of power generation from an attitude and orbitally coupled solar panel. Specifically, it:

1.  Evaluates the impact of shadowing using an assigned :ref:`EclipseMsgPayload`
2.  Computes power generation using a cosine law based on the panel area, efficiency, and attitude
3.  Allows for the panel body-fixed orientation ``nHat_B``, the panel area, and the panel efficiency to be set via ``setPanelParameters()``.
4.  Writes out a :ref:`PowerNodeUsageMsgPayload` describing its power generation.

    Power generation is computed according to `SMAD <https://www.springer.com/gp/book/9780792309710>`__:

    .. math::

        W_{out} = W_{base} * C_{eclipse} * C_{panel} * (\hat{n}\cdot \hat{s}) A_{panel}

    where :math:`W_{base}` is the base power (in :math:`\mbox{W}/\mbox{m}^2`) at the spacecraft location from the sun, :math:`C_{eclipse}` is the eclipse/penumbra mitigator on the sun's power (1 corresponds to no shadow, 0 corresponds to eclipse), :math:`C_{panel}` represents the panel's efficiency at converting solar energy into electrical energy, :math:`(\hat{n}\cdot \hat{s})` represents the alignment between the panel's normal vector and the spaceraft-sun unit vector, and :math:`A_{panel}` represents the panel area in meters squared.

For more information on how to set up and use this module, see the simple power system example: :ref:`scenarioPowerDemo`

Module Assumptions and Limitations
----------------------------------
This module only uses the input and output messages of the PowerNodeBase base class.  Further, the module does not include any self-shadowing in the solar panel power generation evaluation.

Message Connection Descriptions
-------------------------------
The following table lists additional module input messages beyond those specified in :ref:`PowerNodeBase`.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - sunInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - Describes sun position
    * - stateInMsg
      - :ref:`SCStatesMsgPayload`
      - Describes spacecraft position, attitude.
    * - sunEclipseInMsg
      - :ref:`EclipseMsgPayload`
      - (optional) Describes shadow factor due to planetary bodies.



User Guide
----------
This module inherits the user guide from the PowerNodeBase base class.  Module specific instructions include:

- must connect ``sunInMsg`` and ``stateInMsg`` input messages
- the ``sunEclipseInMsg`` message is optional.  If provided the modules uses the eclipse shadow factor to adjust the power generation if needed.
- must specify the variables ``panelArea``, ``panelEfficiency`` and ``nHat_B``.  These there parameters can also be set at the same time through ``setPanelParameters(nHat_B, panelArea, panelEfficiency)``

For more information on how to set up and use this module, see the simple power system example: :ref:`scenarioPowerDemo`
