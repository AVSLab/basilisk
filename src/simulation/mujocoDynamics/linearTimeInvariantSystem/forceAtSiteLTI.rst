Executive Summary
-----------------
The ``ForceAtSiteLTI`` module implements a three-input, three-output linear time-invariant force model by specializing :ref:`LinearTimeInvariantSystem <LinearTimeInvariantSystem>`. It maps site-frame force vectors from ``ForceAtSiteMsgPayload`` into the LTI input and publishes filtered or shaped site-frame forces through the same payload type.

Module Description
------------------
The inherited LTI dynamics are

.. math::
   \dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}, \qquad
   \mathbf{y} = \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u}

with fixed dimensions for this subclass:

- input size: :math:`m=3`
- output size: :math:`p=3`

The input mapping is

.. math::
   \mathbf{u} =
   \begin{bmatrix}
   F_x \\
   F_y \\
   F_z
   \end{bmatrix}, \qquad
   \mathbf{u} = \texttt{inMsg().force_S}

and output publication is

.. math::
   \texttt{outMsg.force_S} = \mathbf{y}_{0:3}

All quantities are expressed in the site frame :math:`S`, consistent with ``ForceAtSiteMsgPayload``.

Message Interfaces
------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - inMsg
      - :ref:`ForceAtSiteMsgPayload`
      - Input 3D force vector in site frame. ``force_S`` maps to :math:`\mathbf{u}`.
    * - outMsg
      - :ref:`ForceAtSiteMsgPayload`
      - Output 3D force vector in site frame. ``force_S`` is populated from the first three components of :math:`\mathbf{y}`.

Module Assumptions and Limitations
----------------------------------
- The model is strictly 3D (input/output dimensions are always 3).
- The state dimension is inherited from the configured base-class matrices.
- If no state is configured, output uses direct feedthrough from :math:`\mathbf{D}` when available.
