Executive Summary
-----------------
The ``SingleActuatorLTI`` module implements a single-input, single-output (SISO) linear time-invariant model by specializing :ref:`LinearTimeInvariantSystem <LinearTimeInvariantSystem>`. It maps scalar actuator messages into the LTI input and publishes the scalar LTI output using ``SingleActuatorMsgPayload``.

Module Description
------------------
The inherited LTI dynamics are

.. math::
   \dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}, \qquad
   \mathbf{y} = \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u}

with fixed dimensions for this subclass:

- input size: :math:`m=1`
- output size: :math:`p=1`

The message mapping is

.. math::
   \mathbf{u} = \begin{bmatrix}u\end{bmatrix}, \qquad
   u = \texttt{inMsg().input}

and the published output is

.. math::
   \texttt{outMsg.input} = y_0

System matrices :math:`\mathbf{A},\mathbf{B},\mathbf{C},\mathbf{D}` are configured through the base class (for example with ``setA``, ``setB``, ``setC``, ``setD`` or ``configureSecondOrder``).

Message Interfaces
------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - inMsg
      - :ref:`SingleActuatorMsgPayload`
      - Input scalar actuator command. The ``input`` field maps to :math:`u`.
    * - outMsg
      - :ref:`SingleActuatorMsgPayload`
      - Output scalar actuator command. The ``input`` field is populated from :math:`y_0`.

Module Assumptions and Limitations
----------------------------------
- The model is strictly SISO (input/output dimensions are always 1).
- The state dimension is inherited from the configured base-class matrices.
- If no state is configured, output uses direct feedthrough from :math:`\mathbf{D}` when available.
