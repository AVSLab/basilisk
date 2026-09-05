
Executive Summary
-----------------

Radiation pressure dynamics class used to compute

The module
:download:`PDF Description </../../src/simulation/dynamics/RadiationPressure/_Documentation/Basilisk-RadiationPressure-20170712.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: radiationPressure
    :caption: Module I/O Messages

    input sunEphmInMsg SpicePlanetStateMsgPayload
        sun state input message.
    input sunEclipseInMsg EclipseMsgPayload
        (optional) sun eclipse input message.

Initialization and Reset
------------------------
When the spacecraft links this effector to the hub states, initialization verifies that ``sunEphmInMsg`` is
connected.  This validation occurs even if only the spacecraft is added to a task.

Attachment-time validation does not replace task scheduling for this effector.  Add it to a task so that
``UpdateState()`` refreshes the cached Sun ephemeris and optional eclipse data used during force evaluation.

When the effector is scheduled, its ``Reset()`` repeats the required-message check.
``Reset()`` does not clear the cached ephemeris, illumination, force, or torque data.
