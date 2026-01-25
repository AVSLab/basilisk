Executive Summary
-----------------
The ``saturationSingleActuator`` module applies optional saturation limits to a
scalar actuator command. The module reads a
:ref:`SingleActuatorMsgPayload` input message containing a scalar command
:math:`u`. When saturation is enabled, the command is clamped between a
user-defined minimum and maximum value,

.. math::
    u_{\text{out}} = \mathrm{clip}(u, u_{\min}, u_{\max})

When saturation is disabled, the input command is passed through unchanged.

Message Connection Description
------------------------------
The following table lists the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - actuatorInMsg
      - :ref:`SingleActuatorMsgPayload`
      - Input scalar actuator command
    * - actuatorOutMsg
      - :ref:`SingleActuatorMsgPayload`
      - Output saturated or pass-through actuator command

Module Functions
----------------
This module performs the following functions:

    - Reads the incoming scalar actuator command
    - Applies optional minimum and maximum saturation limits
    - Writes the resulting scalar actuator command to the output message

Module Assumptions and Limitations
----------------------------------
    - Saturation behavior is enabled or disabled via a configuration flag
    - Minimum and maximum limits are independent and user-configurable
    - No rate limits, deadbands, or actuator dynamics are modeled

Test Description and Success Criteria
-------------------------------------
The unit test for this module is defined in
:ref:`test_saturationSingleActuator`.
The test applies a set of commands spanning values below, within, and above the
configured limits. The test passes if the output command matches the expected
clamped value when saturation is enabled, and exactly matches the input when
saturation is disabled, within numerical tolerance.
