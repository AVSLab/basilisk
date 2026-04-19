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
The following table lists the module input and output messages.  The message
type contains a link to the message structure definition, while the description
provides information on what the message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - armConfigInMsg
      - :ref:`THRArmConfigMsgPayload`
      - Input articulated thruster-arm configuration message.
    * - CoMStatesInMsg
      - :ref:`SCStatesMsgPayload`
      - Input spacecraft center-of-mass state message.
    * - hubStatesInMsg
      - :ref:`SCStatesMsgPayload`
      - Input spacecraft hub state message.
    * - transForceInMsg
      - :ref:`CmdForceInertialMsgPayload`
      - Input inertial-frame commanded force message.
    * - rotTorqueInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - Input body-frame commanded torque message.
    * - thrForceOutMsg
      - :ref:`THRArrayCmdForceMsgPayload`
      - Output thruster force command message.
    * - desJointAnglesOutMsg
      - :ref:`JointArrayStateMsgPayload`
      - Output desired joint angle command message.


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
