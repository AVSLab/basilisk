Executive Summary
-----------------

This module computes the feedforward torque command from the expected torque to be produced by the torque rods given their current dipole commands.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - vehControlInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - input message containing the current control torque in the Body frame
    * - dipoleRequestMtbInMsg
      - :ref:`MTBCmdMsgPayload`
      - input message containing the individual dipole requests for each torque bar on the vehicle
    * - tamSensorBodyInMsg
      - :ref:`TAMSensorBodyMsgPayload`
      - input message for magnetic field sensor data
    * - mtbParamsInMsg
      - :ref:`MTBArrayConfigMsgPayload`
      - input message for MTB layout
    * - vehControlOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - output message containing the current control torque in the Body frame

Detailed Module Description
---------------------------
This section presents underlying mathematics utilized in the development of the reaction wheel
momentum management and torque rod dipole command generation algorithms. It’s important to point
out that these algorithms are developed with the target of dumping the net momentum of the reaction wheels.
This does not necessarily mean driving the individual reaction wheel speeds to zero since the wheels
can be spun up in their null space, if it exists, and still have a net momentum of zero.
Additionally, saturating the dipole commands described in the mathematics below may result in an
imperfect projection of the desired torque to be produced by the rods into the plane
orthogonal to the local magnetic field, though it has been observed in simulation that this is not an issue.

Wheel Momentum to Torque
~~~~~~~~~~~~~~~~~~~~~~~~

Assume the spacecraft contains :math:`N_{\text{RW}}` RWs. The net RW angular momentum is given by

.. math::
    {}^{\cal B} {\bf h}_{\text{wheels}} = \sum_{i=1}^{N_{\text{RW}}} \hat{\bf g}_{s_i} J_{s_i} \Omega_i

where :math:`\hat{\bf g}_{s_i}` is the RW spin axis in the Body frame :math:`\cal B`, :math:`J_{s_i}`
is the spin axis RW inertia and :math:`\Omega_i` is the RW speed rate about this axis.
The desired torque to be produced by the torque rods to drive the wheel momentum to zero is
then given by the proportional control law

.. math::
    {}^{\cal B} {\pmb\tau}_{\text{desired}} = - K_p \ {}^{\cal B} {\bf h}_{\text{wheels}}

where :math:`K_p` is the proportional feedback gain with units of 1/s.


Torque to Dipole
~~~~~~~~~~~~~~~~

The desired body frame dipole to be produced by the torque rods is given by

.. math::
    {}^{\cal B} {\pmb\mu}_{\text{desired}} = \frac{1}{|\bf b|^2}
    {}^{\cal B}{\bf b} \times \ {}^{\cal B} {\pmb\tau}_{\text{desired}} = [G_t] {\pmb\mu}_{\text{cmd}}

where :math:`\bf b` is the local magnetic field vector and :math:`[G_t]` is a 3 :math:`\times N_{\text{MTB}}`
matrix that transforms the individual rod dipoles to the Body frame.

Dipole Mapping and Saturation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The individual rod dipoles are then given by

.. math::
    {\pmb \mu}_{\text{cmd}} = [G_t]^{\dagger} \ {}^{\cal B} {\pmb\mu}_{\text{desired}}

where the :math:`\dagger` symbol denotes the pseudo inverse. The dipole commands may need to be
saturated at this point. The saturated commands are referred to as :math:`{\pmb\mu}_{\text{saturated}}`
from here on out in this document.

Feed Forward Torque
~~~~~~~~~~~~~~~~~~~
The expected torque produced by the torque rods is given by

.. math::
    {}&{\cal B} {\pmb \tau}_{\text{rods}} = [G_t] {\pmb\mu}_{\text{saturated}} \times \ {}^{\cal B}{\bf b}

and the feed forward command used to dump the momentum of the reaction wheels is simply the
negation of the expected torque produced by the rods.

.. math::
    {}^{\cal B} {\pmb\tau}_{\text{ff}} = - {}^{\cal B}{\pmb\tau}_{\text{rods}}
