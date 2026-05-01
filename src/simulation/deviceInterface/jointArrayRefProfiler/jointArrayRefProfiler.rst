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
The values within the ``desJointStatesInMsg`` are updated. In instances where
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

#. For each hinged joint in the system, add a hinged joint to the module::

    module.addHingedJoint()

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, module)
