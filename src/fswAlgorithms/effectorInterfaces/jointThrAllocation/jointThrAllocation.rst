Executive Summary
-----------------

This module allocates commanded translational force and body torque across
thrusters mounted on articulated arms.  It solves for joint angle commands and
thruster force commands using the arm configuration message and spacecraft
state messages.

The optimizer uses SciPy.  Importing the module from
``Basilisk.fswAlgorithms`` does not require SciPy, but executing
``JointThrAllocation.UpdateState`` does.


Message Connection Descriptions
-------------------------------
The following diagram and table list the module input and output messages.

.. bsk-module-io:: JointThrAllocation
    :caption: Module I/O Messages

    input armConfigInMsg THRArmConfigMsgPayload
        Input articulated thruster-arm configuration message.

    input CoMStatesInMsg SCStatesMsgPayload
        Input spacecraft center-of-mass state message.

    input hubStatesInMsg SCStatesMsgPayload
        Input spacecraft hub state message.

    input transForceInMsg CmdForceInertialMsgPayload
        Input inertial-frame commanded force message.

    input rotTorqueInMsg CmdTorqueBodyMsgPayload
        Input body-frame commanded torque message.

    output thrForceOutMsg THRArrayCmdForceMsgPayload
        Output thruster force command message.

    output desJointAnglesOutMsg JointArrayStateMsgPayload
        Output desired joint angle command message.


Module Assumptions and Limitations
----------------------------------
The implementation assumes serial arm chains packed in arm order, one
spacecraft tree, and one thruster per arm.  The thruster parent joint is
assumed to be the configured joint index on that arm.


User Guide
----------
The module is imported through the standard flight-software package:

.. code-block:: python

    from Basilisk.fswAlgorithms import jointThrAllocation

    allocation = jointThrAllocation.JointThrAllocation()
    allocation.ModelTag = "jointThrAllocation"

The thrust upper bound can be provided as either a scalar or a per-thruster
vector:

.. code-block:: python

    allocation.setThrForceMax(2.5)

The wrench tracking weights can be provided as a scalar, a length-six vector,
or a 6-by-6 matrix:

.. code-block:: python

    allocation.setWc(1.0)

The thrust weighting term can be provided as either a scalar or a per-thruster
vector:

.. code-block:: python

    allocation.setWf(1.0e-6)
