Executive Summary
-----------------
The ``inertialCartFeedback`` module computes an inertial-frame deputy force command for translational tracking of a desired deputy inertial state.
This module uses Lyapunov control theory described in chapter 14 of `Analytical Mechanics of Space Systems <http://doi.org/10.2514/4.105210>`__.

The control force :math:`\mathbf{F}_N` used in this module is found through Eq. :eq:`eq:control_law`:

.. math::
    \mathbf{F}_N = -m_d\left(\mathbf{a}_d - \mathbf{a}_d^*\right) - m_d[K]\Delta \mathbf{r} - m_d[P]\Delta \dot{\mathbf{r}} + \mathbf{F}_N^*
    :label: eq:control_law

where

.. math::
    \Delta \mathbf{r} = \mathbf{r}_d - \mathbf{r}_d^*, \qquad
    \Delta \dot{\mathbf{r}} = \dot{\mathbf{r}}_d - \dot{\mathbf{r}}_d^*
    :label: eq:state_errors

:math:`m_d` is the deputy mass, and a superscript * denotes the desired state.

The gravity-difference term is found using:

.. math::
    \mathbf{a}_d = -\mu \frac{\mathbf{r}_d}{\|\mathbf{r}_d\|^3}, \qquad
    \mathbf{a}_d^* = -\mu \frac{\mathbf{r}_d^*}{\|\mathbf{r}_d^*\|^3}
    :label: eq:gravity_diff


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
    * - deputyTransInMsg
      - :ref:`NavTransMsgPayload`
      - Deputy inertial position and velocity input msg
    * - deputyTransDesiredInMsg
      - :ref:`NavTransMsgPayload`
      - Desired deputy inertial position and velocity input msg
    * - deputyVehicleConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - Deputy spacecraft configuration input msg
    * - forceFeedforwardInMsg
      - :ref:`CmdForceInertialMsgPayload`
      - (Optional) inertial feed-forward force input msg
    * - forceOutMsg
      - :ref:`CmdForceInertialMsgPayload`
      - Inertial force command output msg

User Guide
----------
This section outlines the steps needed to set up the ``inertialCartFeedback`` module in Python using Basilisk.

#. Import the module::

    from Basilisk.fswAlgorithms import inertialCartFeedback

#. Create an instance of ``InertialCartFeedback``::

    module = inertialCartFeedback.InertialCartFeedback()

#. Set gain matrices using row-major flattened 3x3 lists::

    K = [2.0e-5, 0.0, 0.0,
         0.0, 3.0e-5, 0.0,
         0.0, 0.0, 4.0e-5]
    P = [5.0e-2, 0.0, 0.0,
         0.0, 6.0e-2, 0.0,
         0.0, 0.0, 7.0e-2]
    module.setK(K)
    module.setP(P)

#. (Optional) set ``mu`` for gravity compensation::

    module.setMu(3.986004418e14)

#. Add the module to a task::

    unitTestSim.AddModelToTask(unitTaskName, module)
