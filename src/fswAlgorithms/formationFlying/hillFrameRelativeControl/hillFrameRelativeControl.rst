Executive Summary
-----------------
This module computes an inertial control force command for deputy spacecraft relative motion using a Hill-frame
proportional-derivative (PD) law with additional feedforward compensation terms. The chief translational navigation
input is :ref:`NavTransMsgPayload`, which matches the output type of ``simpleNav.transOutMsg``.

Relative Hill-frame position and velocity can be supplied either directly via :ref:`HillRelStateMsgPayload` (for
example from ``hillStateConverter``) or computed internally from deputy inertial state. Exactly one of these input
paths must be connected. The controller evaluates tracking error against user-provided Hill-frame references and maps
the commanded acceleration back to inertial-frame force using deputy mass.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from Python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - chiefTransInMsg
      - :ref:`NavTransMsgPayload`
      - chief translational navigation input message
    * - hillStateInMsg
      - :ref:`HillRelStateMsgPayload`
      - deputy-relative-to-chief Hill-frame state input message (exclusive with ``deputyTransInMsg``)
    * - deputyTransInMsg
      - :ref:`NavTransMsgPayload`
      - deputy translational navigation input message (exclusive with ``hillStateInMsg``)
    * - deputyVehicleConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - deputy mass configuration input message
    * - forceOutMsg
      - :ref:`CmdForceInertialMsgPayload`
      - inertial-frame force command output message
    * - forceOutMsgC
      - :ref:`CmdForceInertialMsgPayload`
      - C-wrapped inertial-frame force command output message

Detailed Module Description
---------------------------
Let :math:`\mathbf{r}_{rel,H}` and :math:`\mathbf{v}_{rel,H}` be deputy-relative-to-chief position and velocity in the
chief Hill frame. Let :math:`\mathbf{r}_{ref,H}` and :math:`\mathbf{v}_{ref,H}` be the desired Hill-frame references.
The command acceleration includes PD feedback plus feedforward compensation :math:`-(A_1\mathbf{r}_{rel,H} + A_2\mathbf{v}_{rel,H})`.

.. math::
    \mathbf{a}_{cmd,H} = -A_1\mathbf{r}_{rel,H} - A_2\mathbf{v}_{rel,H}
                         -\mathbf{K}(\mathbf{r}_{rel,H}-\mathbf{r}_{ref,H})
                         -\mathbf{P}(\mathbf{v}_{rel,H}-\mathbf{v}_{ref,H}).

where

.. math::
    A_1 =
    \begin{bmatrix}
    2\mu/R^3 + \dot{\theta}^2 & \ddot{\theta} & 0 \\
    -\ddot{\theta} & \dot{\theta}^2 - \mu/R^3 & 0 \\
    0 & 0 & -\mu/R^3
    \end{bmatrix},
    \qquad
    A_2 =
    \begin{bmatrix}
    0 & 2\dot{\theta} & 0 \\
    -2\dot{\theta} & 0 & 0 \\
    0 & 0 & 0
    \end{bmatrix}.

Here, :math:`R` is chief radius magnitude, :math:`\theta` is chief true latitude, :math:`\mu` is central-body
gravitational parameter, and overdots denote time derivatives.

This formulation follows Schaub and Junkins,
`Analytical Mechanics of Space Systems <https://doi.org/10.2514/4.107597>`_.

The inertial command force is

.. math::
    \mathbf{F}_{cmd,N} = m_d \mathbf{C}_{NH}\mathbf{a}_{cmd,H},

with :math:`m_d` the deputy mass from :ref:`VehicleConfigMsgPayload`.

Model Assumptions and Limitations
---------------------------------
- Deputy mass from ``deputyVehicleConfigInMsg.massSC`` must be positive.
- ``mu`` must be set to a positive value using ``setMu``.
- ``K`` and ``P`` must be set before simulation start using ``setK`` and ``setP``.
- Exactly one of ``hillStateInMsg`` or ``deputyTransInMsg`` must be connected.
- Chief radius and angular momentum vectors must be non-zero.
- Gains are full 3x3 matrices in Hill frame and must be symmetric positive definite.

User Guide
----------
The module is created in Python using:

.. code-block:: python
    :linenos:

    from Basilisk.fswAlgorithms import hillFrameRelativeControl

    ctrl = hillFrameRelativeControl.HillFrameRelativeControl()
    ctrl.ModelTag = "hillFrameRelativeControl"

Then connect messages for one of the following two modes:

1. direct deputy navigation input mode

.. code-block:: python
    :linenos:

    ctrl.chiefTransInMsg.subscribeTo(chiefSimpleNav.transOutMsg)
    ctrl.deputyTransInMsg.subscribeTo(deputySimpleNav.transOutMsg)
    ctrl.deputyVehicleConfigInMsg.subscribeTo(vehicleConfigMsg)

2. precomputed Hill-state input mode

.. code-block:: python
    :linenos:

    from Basilisk.fswAlgorithms import hillStateConverter

    hillConv = hillStateConverter.hillStateConverter()
    hillConv.chiefStateInMsg.subscribeTo(chiefSimpleNav.transOutMsg)
    hillConv.depStateInMsg.subscribeTo(deputySimpleNav.transOutMsg)

    ctrl.chiefTransInMsg.subscribeTo(chiefSimpleNav.transOutMsg)
    ctrl.hillStateInMsg.subscribeTo(hillConv.hillStateOutMsg)
    ctrl.deputyVehicleConfigInMsg.subscribeTo(vehicleConfigMsg)

Configure gains, references and gravitational parameter:

.. code-block:: python
    :linenos:

    ctrl.setMu(3.986004418e14)  # [m^3/s^2]
    ctrl.setK([2e-6, 0.0, 0.0, 0.0, 2e-6, 0.0, 0.0, 0.0, 2e-6])  # [1/s^2]
    ctrl.setP([2e-3, 0.0, 0.0, 0.0, 2e-3, 0.0, 0.0, 0.0, 2e-3])  # [1/s]
    ctrl.setReferencePosition([100.0, 0.0, 0.0])  # [m]
    ctrl.setReferenceVelocity([0.0, 0.0, 0.0])  # [m/s]
