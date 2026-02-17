Executive Summary
-----------------
The ``OrbElemOffset`` module combines two classical orbital element messages into a single output element set. One
input provides the nominal (main) orbit, and the second provides element offsets. By default, all fields are added
element wise, including true anomaly. Optionally, the anomaly offset can be interpreted as a mean anomaly increment.

Module Description
------------------
The module reads:

- ``mainElementsInMsg``: baseline classical elements
- ``offsetElementsInMsg``: element offsets to apply

For the following fields, the output is always computed through direct addition:

.. math::
   a_{out} = a_{main} + a_{off}

.. math::
   e_{out} = e_{main} + e_{off}

.. math::
   i_{out} = i_{main} + i_{off}

.. math::
   \Omega_{out} = \Omega_{main} + \Omega_{off}

.. math::
   \omega_{out} = \omega_{main} + \omega_{off}

The handling of ``f`` depends on ``useMeanAnomalyOffset``:

- ``False`` (default): treat ``offset.f`` as a true-anomaly increment

  .. math::
     f_{out} = f_{main} + f_{off}

- ``True``: treat ``offset.f`` as a mean-anomaly increment :math:`\Delta M`

  1. Convert :math:`(f_{main}, e_{main})` to :math:`M_{main}`
  2. Form :math:`e_{out} = e_{main} + e_{off}`
  3. Form :math:`M_{out} = M_{main} + \Delta M`
  4. Convert :math:`(M_{out}, e_{out})` back to :math:`f_{out}`

This preserves a mean-anomaly interpretation for the offset while still allowing eccentricity to be updated through
``offset.e``.

Message Interfaces
------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - mainElementsInMsg
      - :ref:`ClassicElementsMsgPayload`
      - Input message containing the baseline classical elements
        (``a``, ``e``, ``i``, ``Omega``, ``omega``, ``f``).
    * - offsetElementsInMsg
      - :ref:`ClassicElementsMsgPayload`
      - Input message containing additive offsets for the classical elements. The ``f`` field is interpreted either
        as :math:`\Delta f` or :math:`\Delta M` depending on ``useMeanAnomalyOffset``.
    * - elementsOutMsg
      - :ref:`ClassicElementsMsgPayload`
      - Output message containing the combined classical elements after applying offsets.

Detailed Behavior
-----------------
At each update step, the module performs the following operations:

1. Reads baseline elements from ``mainElementsInMsg``.
2. Reads element offsets from ``offsetElementsInMsg``.
3. Computes ``a``, ``e``, ``i``, ``Omega``, and ``omega`` via element wise addition.
4. Computes ``f`` either as a direct true-anomaly addition or via mean-anomaly conversion/update based on
   ``useMeanAnomalyOffset``.
5. Writes the resulting ``ClassicElementsMsgPayload`` to ``elementsOutMsg``.

During ``Reset()``, the module checks that both input messages are linked and logs an error if either input is not
connected.

Verification and Testing
------------------------
The module is verified by an automated unit test in
``src/fswAlgorithms/orbitControl/orbElemOffset/_UnitTest/test_orbElemOffset.py``.

The test is parameterized over both anomaly handling modes:

- ``useMeanAnomalyOffset = False``: verifies all six fields are direct sums.
- ``useMeanAnomalyOffset = True``: verifies the non-anomaly fields are direct sums and verifies that the output mean
  anomaly differs from the baseline mean anomaly by exactly ``offset.f`` within numerical tolerance.

This validates both the baseline additive behavior and the mean-anomaly-offset conversion path.
