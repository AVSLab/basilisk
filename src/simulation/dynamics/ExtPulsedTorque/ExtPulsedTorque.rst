
Executive Summary
-----------------

:ref:`dynamicEffector` Class used to provide a direct pulsed external torque on body
The module
:download:`PDF Description </../../src/simulation/dynamics/ExtPulsedTorque/_Documentation/Basilisk-extPulsedTorque-20170324.pdf>`
contains further information on this module's function,
how to run it, as well as testing.
The current timing and reset semantics are specified below.


Message Connection Descriptions
-------------------------------
This module has no input or output messages.  It is a pure dynamic effector.


Pulse Timing
------------
The sequence is anchored at simulation time zero and consists of a positive pulse, a negative
pulse of equal duration, and an optional off period. ``pulseInterval`` specifies the duration of
one count in seconds and defaults to one second. Each positive or negative pulse lasts
``countOnPulse * pulseInterval`` seconds. The following off period lasts
``countOff * pulseInterval`` seconds, after which the sequence repeats.

For example::

    pulse = ExtPulsedTorque.ExtPulsedTorque()
    pulse.pulsedTorqueExternalPntB_B = [0.1, 0.0, 0.0]  # [N*m]
    pulse.countOnPulse = 2
    pulse.countOff = 3
    pulse.pulseInterval = 0.125  # [s]

This configuration applies positive torque for :math:`0 \leq t < 0.25` seconds, negative torque
for :math:`0.25 \leq t < 0.5` seconds, and zero torque for :math:`0.5 \leq t < 0.875` seconds.
At each transition, evaluation selects the segment beginning at that time. Times within
floating-point roundoff of a transition also select the new segment, so decimal intervals
such as ``0.1`` seconds do not delay a boundary by one evaluation.
The roundoff window uses the evaluation time and the relevant transition time; a long future
off period does not enlarge the window around the initial pulse transitions.

``computeForceTorque()`` uses its integration time in seconds. Its ``timeStep`` argument does
not set the pulse interval. Repeated evaluations, intermediate integrator stages, and evaluations
at earlier times return the torque for the supplied time without advancing internal pulse state.
Choose an integration step and error tolerance that resolve the pulse transitions; the torque
is discontinuous at those transitions.

Existing configurations that express pulse counts in units of a nominal integration step should
set ``pulseInterval`` to that step duration in seconds. This replaces the historical behavior
that advanced the sequence on every dynamics evaluation, including intermediate integrator stages.


Initialization and Reset
------------------------
The torque and both counts default to zero, so an unconfigured effector applies no load.
``countOnPulse`` and ``countOff`` must be non-negative; a zero ``countOnPulse`` disables the torque.
Torque components and ``pulseInterval`` must be finite, the interval must be strictly positive,
and the combined cycle duration must remain finite. Invalid configuration raises ``BasiliskError``.

Validation runs during spacecraft attachment through ``linkInStates()``, from ``Reset()``, and
before every dynamics evaluation. The effector therefore works without task scheduling.
``Reset()`` preserves the phase anchored at simulation time zero and leaves the last output
unchanged until the next dynamics evaluation. ``UpdateState()`` does not advance the pulse.
Changing pulse parameters selects the corresponding waveform with the same time-zero origin.
See :ref:`effectorInitialization`.
