Executive Summary
-----------------

This module allocates commanded translational force and body torque across
thrusters mounted on articulated arms. It solves for joint angle commands and
thruster force commands using the arm configuration message and spacecraft
state messages.

The vector from the hub body-frame origin to the instantaneous spacecraft
center of mass is computed internally from the current joint configuration.
As a result, the articulated arm configuration message must provide both
the arm kinematics and the mass properties needed to reconstruct the system
center of mass.

The optimizer uses SciPy.  Importing the module from
``Basilisk.fswAlgorithms`` does not require SciPy, but executing
``JointThrAllocation.UpdateState`` does.


Message Connection Descriptions
-------------------------------
The following diagram and table list the module input and output messages.

.. bsk-module-io:: JointThrAllocation
    :caption: Module I/O Messages

    input armConfigInMsg THRArmConfigMsgPayload
        Input articulated thruster-arm configuration and mass-property message.

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

The ``armConfigInMsg`` input must define both the articulated thruster-arm
kinematics and the body mass properties used to compute the instantaneous
spacecraft center of mass. The required message fields are:

- ``hubMass``: hub mass :math:`[\text{kg}]`
- ``r_BcB_B``: hub center of mass relative to the hub body-frame origin
  :math:`[\text{m}]`
- ``bodyArmIdx``: arm index for each mass-carrying body
- ``bodyJointIdx``: local parent-joint index for each mass-carrying body
- ``bodyMass``: mass of each arm body :math:`[\text{kg}]`
- ``r_LcP_P``: body center of mass relative to the parent joint, expressed in
  the parent-joint frame :math:`[\text{m}]`
- ``armJointCount``: number of joints in each arm
- ``r_CP_P``: parent-joint to child-joint position vectors :math:`[\text{m}]`
- ``shat_P``: child-joint spin axes
- ``dcm_C0P``: zero-angle child-to-parent direction cosine matrices
- ``armTreeIdx``: kinematic-tree index for each arm
- ``thrArmIdx``: arm index for each thruster
- ``thrArmJointIdx``: local parent-joint index for each thruster
- ``r_TP_P``: thruster position relative to the parent joint
  :math:`[\text{m}]`
- ``fhat_P``: thruster force direction unit vectors

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
