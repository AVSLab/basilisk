Executive Summary
-----------------
This module converts instantaneous reference joint angle changes to either
filtered or time-profiled reference joint angles, rates, and accelerations.

Message Connection Descriptions
-------------------------------
The following diagram and table list the module input and output messages.

.. bsk-module-io:: jointArrayRefProfiler
    :caption: Module I/O Messages

    input jointStatesInMsgs ScalarJointStateMsgPayload
        Vector of current joint state input messages.
    input jointStateDotsInMsgs ScalarJointStateMsgPayload
        Vector of current joint state-derivative input messages.
    input desJointStatesInMsg JointArrayStateMsgPayload
        Desired joint-array state input message.
    output desJointStatesOutMsg JointArrayStateMsgPayload
        Joint-array reference output message.

Module Assumptions and Limitations
----------------------------------
This module assumes the desired joint-array command remains constant
between message updates and that a new profile should begin whenever
the values within the ``desJointStatesInMsg`` are updated. In instances where
there is a timing mismatch between the write time of
``desJointStatesInMsg`` and the module update time, the new profile
begins based on the module update time rather than the time the message
was written.

The ``lowPass`` mode applies a first-order discrete low-pass filter
to the desired joint angles using the user-specified ``wc`` and ``filterDt``
parameters. This mode smooths the reference command but does not guarantee
finite-time convergence like the time-profiled modes.

For the ``linear``, ``cubic``, and ``quintic`` modes, the module is
designed to drive each joint to the desired final angle over the
specified ``profileDuration``. The ``cubic`` and ``quintic``
implementations assume the terminal joint rate and terminal joint
acceleration are zero. The ``linear`` mode does not preserve the
initial joint rates, performs constant-rate interpolation, and only
reaches zero rate once the profile completes.

Angle Handling
--------------
By default, joint-angle wrapping is disabled and ``useShortestPath`` is
``False``. The module therefore preserves the input joint-angle
representation and profiles directly to the commanded angle.

Call ``setJointAngleWrapStart()`` to enable wrapping and define the lower
bound of the output interval. The module normalizes rotational joint inputs,
commanded angles, and output references to
:math:`[\theta_{start}, \theta_{start} + 2\pi)`. For example,
a lower bound of :math:`-\pi` selects :math:`[-\pi, \pi)`, while a lower
bound of :math:`0` selects :math:`[0, 2\pi)`. Call
``disableJointAngleWrapping()`` to disable wrapping after it has been
configured. This feature is intended for periodic rotational joints and
should not be used for multi-turn joint-coordinate commands.

In ``lowPass`` mode, the internal filter state remains unwrapped so that it
can converge continuously across an interval boundary. Only the joint-angle
value written to the output message is normalized to the configured interval.

Set ``useShortestPath`` to ``True`` to profile to the closest
:math:`2\pi`-equivalent of each commanded angle relative to the joint angle at
the start of a new profile. The effective target displacement is in
:math:`[-\pi, \pi]`. For example, a transition from :math:`-170^\circ` to
:math:`170^\circ` follows a :math:`-20^\circ` path.

The shortest-path option is independent of angle wrapping. When
``useShortestPath`` is enabled while joint-angle wrapping is disabled, the
output reference retains an unwrapped angle representation to preserve a
continuous short-path profile across a canonical interval boundary.

.. note::

    Changing the wrapping interval or enabling or disabling wrapping affects
    the next output message. The sampled start angle and effective target
    remain fixed for the current profile, including after changes to
    ``useShortestPath``. They are recomputed only when the desired message
    values change or on the first ``UpdateState()`` following ``Reset()``.
    Rewriting an identical command with a new timestamp does not restart the
    profile. Call ``Reset()`` before the next update to apply changed settings
    to an unchanged command.

User Guide
----------
This section outlines the steps needed to set up the ``jointArrayRefProfiler`` module in
Python using Basilisk.

#. Import the module::

    from Basilisk.simulation import jointArrayRefProfiler

#. Create an instance of the module::

    module = jointArrayRefProfiler.JointArrayRefProfiler()
    module.ModelTag = "jointArrayRefProfiler"

#. Set the profile type to either ``linear``, ``cubic``, ``quintic``, or ``lowPass``::

    module.setProfileType("linear")

#. For the low pass filter mode, set the cutoff frequency and filter time step::

    module.setWc(1.0)  # [rad/s]
    module.setFilterDt(0.01)  # [s]

#. For the time-profiled modes, set the profile duration::

    module.setProfileDuration(2.0)  # [s]

#. (Optional) Enable joint angle wrapping and set its output interval::

    module.setJointAngleWrapStart(0.0)  # [rad], wraps to [0.0, 2*pi)

#. (Optional) Enable shortest path routing::

    module.setUseShortestPath(True)

#. For each hinged joint in the system, add a hinged joint to the module::

    module.addHingedJoint()

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, module)
