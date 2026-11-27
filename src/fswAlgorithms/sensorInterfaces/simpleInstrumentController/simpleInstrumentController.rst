Executive Summary
-----------------
This module generates a command in the form of a :ref:`DeviceCmdMsgPayload` that turns on a :ref:`simpleInstrument`
if the spacecraft has access to a ground or spacecraft target and the attitude
and optional attitude-rate errors are within tolerance. Optional constraints
include target illumination, message freshness, capture readiness, and either
continuous or cumulative acquisition hold. The default remains immediate imaging.

Message Connection Descriptions
-------------------------------
The following diagram and table list all the module input and output messages.  The module message connection is
set by the user from Python.  The message type contains a link to the message structure definition, while the
description provides information on what this message is used for.

.. bsk-module-io:: simpleInstrumentController
    :caption: Module I/O Messages

    input locationAccessInMsg AccessMsgPayload
        Input message containing ground or spacecraft target access
    input attGuidInMsg AttGuidMsgPayload
        Input message containing the attitude guidance
    input deviceStatusInMsg DeviceStatusMsgPayload
        (optional) input message containing the device status
    input captureReadyInMsg DeviceStatusMsgPayload
        (optional) capture readiness, such as sufficient instrument storage
    input illuminationInMsg EclipseMsgPayload
        (optional) target illumination; required when useIlluminationConstraint is enabled
    output deviceCmdOutMsg DeviceCmdMsgPayload
        Output message with the device command

Detailed Module Description
---------------------------
This module writes out a :ref:`DeviceCmdMsgPayload` to turn on an instrument, i.e. :ref:`simpleInstrument`.

.. note::

    This module assumes that the simulated data is generated instantaneously. Therefore, the ``baudRate`` of the
    simpleInstrument must be sized as such.

The module first checks if the instrument controller is active using the ``controllerStatus`` variable. This
variable defaults to 1 (active), but the user may set the variable to 0 (inactive) with the ``deviceStatusInMsg``
or ``controllerStatus`` variable. Then, the module checks if target has already been imaged.  If the target has
not been imaged, the module then checks if the norm of the attitude error is less than the user specified tolerance
and if the :ref:`groundLocation` is accessible. If both are true, the module sets the ``deviceCmd`` to 1. Otherwise,
the ``deviceCmd`` is set to 0.

User Guide
----------
Two variables must be set. The variable ``attErrTolerance`` is the norm of the acceptable attitude error in MRPs.
It must be set at the beginning of the sim.

The ``imaged`` variable is always initialized to 0 (i.e. the target has not been imaged). However, if the simulation
is stopped and restarted again this variable should be reinitialized to 0 in between. If it is not and the previous
target was imaged, the new target will not be imaged.

Optionally, attitude rate error checking may be enabled by setting ``useRateTolerance`` to ``1``. If enabled,
``rateErrTolerance`` should be set to the norm of the acceptable attitude rate error in rad/s when imaging.

If the user desires to make the instrument controller inactive, the ``controllerStatus`` variable should be set to 0.
This may be done if the controller is to match the status of corresponding instrument data and power modules. This
may also be accomplished by writing out a :ref:`DeviceStatusMsgPayload` to the ``deviceStatusInMsg``.

Optionally, imaging can be limited to a desired time window. Set ``imagingTime`` in nanoseconds as the desired
imaging time. A nonzero ``timeToleranceLower`` permits imaging no earlier than
``imagingTime - timeToleranceLower``, while a nonzero ``timeToleranceUpper`` permits imaging no later than
``imagingTime + timeToleranceUpper``. Each bound is enabled independently; a tolerance of zero disables that bound,
which preserves the default behavior without time-based constraints.

The ``useDurationImaging`` flag enables duration-based imaging, where the instrument must maintain valid
constraints for a specified period before capturing an image. By default, this feature is disabled. The
``acquisitionTime`` (optional, in nanoseconds) specifies how long the constraints must remain satisfied to trigger an
image capture. The ``allowedTime`` (optional, in nanoseconds) defines the maximum period during which the system will
attempt imaging. The ``elapsedTime`` state variable stores the elapsed duration, in nanoseconds, from the start of the
current duration-based imaging attempt. This value is accessible from Python and remains available if the attempt does
not complete. If ``acquisitionTime`` is less than or equal to ``allowedTime``, the image will be captured once the
required duration is met. If ``acquisitionTime`` exceeds ``allowedTime``, the image will not be captured, although the
system will continue attempting for the full ``allowedTime`` period. If either ``acquisitionTime`` or ``allowedTime``
is negative, the value is automatically capped to zero. Unset duration values
are zero. Nonfinite duration values are rejected during reset.

Sampled acquisition holds
~~~~~~~~~~~~~~~~~~~~~~~~~

With ``useDurationImaging=1``, a sampled interval contributes to ``elapsedTime``
only if its two endpoints satisfy all enabled acquisition constraints. By
default, an invalid sample clears the continuous hold. Set ``useCumulativeHold=1``
to preserve previously accumulated valid intervals across interruptions. The
first valid sample after an interruption starts a new interval; it cannot count
the preceding invalid interval. ``acquisitionTime``, ``allowedTime``, and
``elapsedTime`` are all in nanoseconds. ``allowedTime`` limits accumulated valid
duration, including any wait for a capture window or readiness. A mission-level
wall-clock deadline remains the responsibility of the task scheduler.

Reset the controller before switching targets or starting a new acquisition.
``Reset()`` clears hold state, ``imaged``, capture diagnostics, and the output
command. Configuration settings and ``controllerStatus`` are preserved. Once
an image command is issued, subsequent updates write zero commands until the
controller is rearmed. The command is a single update pulse, so the instrument
must run at the same rate or faster and read the command after the controller.

Target illumination and freshness
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Connect ``illuminationInMsg`` to the target's eclipse output. Set
``useIlluminationConstraint=1`` and ``illuminationThreshold`` in [0, 1] to require
a minimum illuminated fraction. The threshold comparison is inclusive. The
legacy message field ``shadowFactor`` and its ``illuminationFactor`` alias both
mean zero for full shadow and one for full illumination.

Even with illumination gating disabled, a connected illumination message is
used to record quality. ``captureIllumination`` is the trapezoidal mean over valid
hold intervals, or the instantaneous fraction for a zero-duration hold.
``captureTime`` records the image-command timestamp in nanoseconds. These are
command diagnostics; storage must confirm the resulting image independently.

The controller starts accumulating quality at the first qualifying sample.
``lastCallTime`` supplies the actual interval length, and ``previousIllumination``
supplies the previous endpoint for trapezoidal averaging. ``illuminationIntegral``
sums illumination times valid interval length. At capture, dividing by
``elapsedTime`` gives ``captureIllumination``, which is then frozen so later
updates cannot alter the acquired image's quality. An invalid sample adds no
duration or illumination. Continuous mode clears both sums; cumulative mode
retains them and starts a new interval when the constraints qualify again.

Set ``requireFreshMessages=1`` to reject unwritten or future-dated guidance,
access, and connected illumination inputs. Guidance must have been written
strictly after the most recent reset. ``maximumGuidanceAge`` bounds guidance
age, while ``maximumAccessAge`` bounds access and illumination age, in
nanoseconds. Zero permits only a sample written at the current controller time.
Freshness checking is disabled by default for existing ground-imaging scenarios.

Capture readiness
~~~~~~~~~~~~~~~~~

An optional ``captureReadyInMsg`` can defer the image command until an instrument
can accept it, for example when storage has enough free capacity. A linked input
must be written and have nonzero ``deviceStatus``. Readiness does not count as a
pointing/access constraint and does not clear an accumulated hold. Unlinked
readiness inputs preserve the previous behavior.
