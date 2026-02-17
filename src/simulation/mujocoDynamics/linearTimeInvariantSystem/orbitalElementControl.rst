Executive Summary
-----------------
The ``OrbitalElementControl`` module specializes :ref:`LinearTimeInvariantSystem <LinearTimeInvariantSystem>` to
implement orbital-element feedback control that outputs an inertial force command. It reads target and current
classical orbital elements, forms a six-element error vector, applies an LTI law, maps the result through the
classical Gauss control matrix, and publishes ``CmdForceInertialMsgPayload``.

Module Description
------------------
The inherited LTI dynamics are

.. math::
   \dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}, \qquad
   \mathbf{y} = \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u}

with fixed input and output size for this subclass:

- input size: :math:`m = 6`
- output size: :math:`p = 6`

The input vector is constructed from target and current classical elements as

.. math::
   \mathbf{u} =
   \begin{bmatrix}
   \Delta a / a \\
   \Delta e \\
   \Delta i \\
   \Delta \Omega \\
   \Delta \omega \\
   \Delta M
   \end{bmatrix}

where:

- :math:`\Delta a / a = (a_{current} - a_{target})/a_{target}`
- :math:`\Delta e = e_{current} - e_{target}`
- :math:`\Delta i, \Delta\Omega, \Delta\omega` are wrapped into :math:`[-\pi,\pi]`
- :math:`\Delta M` is formed by converting each true anomaly to mean anomaly and wrapping the difference into
  :math:`[-\pi,\pi]`

The LTI output :math:`\mathbf{y}` is interpreted in orbital-element space and mapped to LVLH force using the
classical Gauss matrix :math:`\mathbf{B}_{oe}`:

.. math::
   \mathbf{f}_{LVLH} = -\mathbf{B}_{oe}^{T}\mathbf{y}

Then LVLH force is rotated into inertial coordinates and written to
``forceOutMsg.forceRequestInertial``.

Convenience gain helpers are provided:

- ``setProportionalGain(K)``: assigns :math:`\mathbf{D}=K` (6x6)
- ``setIntegralGain(K)``: assigns :math:`\mathbf{B}=I_{6}` and :math:`\mathbf{C}=K` (6x6), enabling integral action
  through the internal LTI state

Message Interfaces
------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - targetOEInMsg
      - :ref:`ClassicElementsMsgPayload`
      - Input target classical orbital elements used as the controller reference.
    * - currentOEInMsg
      - :ref:`ClassicElementsMsgPayload`
      - Input current classical orbital elements used to form feedback errors and to build the Gauss mapping and
        frame transforms.
    * - forceOutMsg
      - :ref:`CmdForceInertialMsgPayload`
      - Output inertial-frame commanded force vector in ``forceRequestInertial``.

Detailed Behavior
-----------------
At each update step, the module performs the following operations:

1. Reads ``targetOEInMsg`` and ``currentOEInMsg``.
2. Constructs the six-element input error vector :math:`\mathbf{u}` with wrapped angular terms and mean-anomaly
   difference.
3. Uses inherited LTI dynamics to compute :math:`\dot{\mathbf{x}}` and :math:`\mathbf{y}`.
4. Builds the classical Gauss matrix :math:`\mathbf{B}_{oe}` from current orbital elements and configured ``mu``.
5. Computes LVLH force :math:`\mathbf{f}_{LVLH} = -\mathbf{B}_{oe}^{T}\mathbf{y}`.
6. Converts current orbital elements to inertial position/velocity, builds the LVLH-to-inertial rotation, and
   rotates :math:`\mathbf{f}_{LVLH}` into inertial force.
7. Publishes inertial force through ``forceOutMsg``.

During ``Reset()``, the module:

- runs base-class matrix-size consistency checks,
- verifies ``mu > 0``,
- verifies both ``targetOEInMsg`` and ``currentOEInMsg`` are linked.

Verification and Testing
------------------------
The module is verified in
``src/simulation/mujocoDynamics/linearTimeInvariantSystem/_UnitTest/test_orbitalElementControl.py``.

The test is parameterized for both proportional and integral gain configurations and compares
``OrbitalElementControl`` output against ``meanOEFeedback`` configured with effectively zero :math:`J_2`. For the same
initial chief/deputy orbit setup and gains, the inertial force command from ``OrbitalElementControl`` is required to
match the reference ``meanOEFeedback`` force within tolerance.
