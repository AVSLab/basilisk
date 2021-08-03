Executive Summary
-----------------
This module maps commanded forces and torques defined in the body frame of the spacecraft to a set of thrusters. It is
capable of handling CoM offsets and directions with no thrusters.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  
The module msg connection is set by the user from python.  
The msg type contains a link to the message structure definition, while the description 
provides information on what this message is used for.
Both the `cmdTorqueInMsg` and `cmdForceInMsg` are optional.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - The name of the vehicle control (Lr) input message
    * - cmdForceInMsg
      - :ref:`CmdForceBodyMsgPayload`
      - The name of the vehicle control force input message
    * - thrConfigInMsg
      - :ref:`THRArrayConfigMsgPayload`
      - The name of the thruster cluster input message
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - The name of the vehicle config input message
    * - thrForceCmdOutMsg
      - :ref:`THRArrayCmdForceMsgPayload`
      - The name of the output thruster force message

Detailed Module Description
---------------------------
Force and Torque Mapping
^^^^^^^^^^^^^^^^^^^^^^^^
The desired force and torque are given as :math:`\mathbf{F}_{req}` and :math:`\boldsymbol{\tau}_{req}`, respectively.
These are both stacked into a single vector.

.. math::
    :label: eq:augmented_ft

    \begin{bmatrix}
    \boldsymbol{\tau}_{req} \\
    \mathbf{F}_{req}
    \end{bmatrix}

The :math:`i_{th}` thruster position expressed in spacecraft body-fixed coordinates is given by :math:`\mathbf{r}_i`. The
unit direction vector of the thruster force is :math:`\hat{\mathbf{g}}_{t_i}`. The thruster force is given as:

.. math::
    :label: eq:force_direction

    \mathbf{F}_i = F_i\hat{\mathbf{g}}_{t_i}

The torque produced by each thruster about the body-fixed CoM is:

.. math::
    :label: eq:torques

    \boldsymbol{\tau}_i = ((\mathbf{r}_i - \mathbf{r}_{\text{COM}}) \times \hat{\mathbf{g}}_{t_i})F_i = \mathbf{d}_iF_i

The total force and torque on the spacecraft may be represented as:

.. math::
    :label: eq:sys_eqs

    \begin{bmatrix}
        \boldsymbol{\tau}_{req} \\
        \mathbf{F}_{req}
    \end{bmatrix} =
    \begin{bmatrix}
        \mathbf{d}_i \ldots \mathbf{d}_N \\
        \hat{\mathbf{g}}_{t_i} \ldots \hat{\mathbf{g}}_{t_N}
    \end{bmatrix}
    \begin{bmatrix}
        F_1 \\
        \vdots \\
        F_N
    \end{bmatrix} = [D]\mathbf{F}

The force required by each thruster can computed by the following equation. Any rows within the :math:`[D]` matrix
that contain only zeros are removed beforehand.

.. math::
    :label: eq:soln

    \mathbf{F} = [D]^T([D][D]^T)^{-1}\begin{bmatrix}
                                         \boldsymbol{\tau}_{req} \\
                                         \mathbf{F}_{req}
                                         \end{bmatrix}

To ensure no commanded thrust is less than zero, the minimum thrust is subtracted from the thrust vector

.. math::
    :label: eq:F_min

    \mathbf{F} = \mathbf{F} - \text{min}(\mathbf{F})

These thrust commands are then written to the output message.