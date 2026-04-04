.. Warning::

   :beta:`downlinkHandling` is a beta module in an initial public release. This module might be
   subject to changes in future releases.

Executive Summary
-----------------
This document describes how the downlinkHandling module maps radio link quality into realistic
data transfer outcomes.

At each simulation step, the module:

- reads link quality from :ref:`linkBudget`
- converts CNR into BER and packet error rate
- applies retry-limited ARQ reliability
- removes data from onboard storage at the resulting effective rate
- publishes detailed diagnostics for analysis and fault studies

The module is designed to sit between RF-link modeling (:ref:`simpleAntenna`, :ref:`linkBudget`)
and onboard data-buffer dynamics (:ref:`DataStorageStatusMsgPayload`, :ref:`DataNodeUsageMsgPayload`).

System Role and Data Flow
-------------------------

.. _downlinkhandling-figure-flow:
.. figure:: /../../src/simulation/communication/downlinkHandling/_Documentation/Images/DownlinkHandlingFlow.svg
   :width: 90%
   :align: center
   :alt: downlinkHandling integration flow with simpleAntenna, linkBudget, and storage

   Figure 1: Integration flow and message-level role of downlinkHandling.

Message Connection Descriptions
-------------------------------
The following table lists all module input and output messages.
The message connection is set by the user from Python.

.. list-table:: Module I/O Messages
   :widths: 22 22 36 20
   :header-rows: 1

   * - Msg Variable Name
     - Msg Type
     - Description
     - Note
   * - ``linkBudgetInMsg``
     - :ref:`LinkBudgetMsgPayload`
     - Link-quality input from :ref:`linkBudget` (receiver states, ``CNR1``, ``CNR2``, overlap bandwidth).
     - Required for non-zero downlink
   * - ``storageUnitInMsgs`` (via ``addStorageUnitToDownlink``)
     - :ref:`DataStorageStatusMsgPayload`
     - Storage state input (partition names, partition bits, total storage level).
     - Required for actual data removal
   * - ``nodeDataOutMsg``
     - :ref:`DataNodeUsageMsgPayload`
     - Data-node output inherited from ``DataNodeBase``. Negative baud rate removes bits from storage.
     - Output
   * - ``downlinkOutMsg``
     - :ref:`DownlinkHandlingMsgPayload`
     - Diagnostics output with link metrics, reliability metrics, rates, and cumulative counters.
     - Output

Detailed Module Description
---------------------------
The module extends ``DataNodeBase`` and runs once per simulation step.

The per-step sequence is:

1. Read ``LinkBudgetMsgPayload`` and all connected storage status messages.
2. Select one storage target (largest finite partition across all connected storage units; use ``storageLevel`` only when a message has no partition vector).
3. Select receiver path (forced receiver index or auto mode).
4. Convert selected CNR and overlap bandwidth into :math:`C/N_0`.
5. Convert :math:`C/N_0` and requested bit rate into :math:`E_b/N_0`.
6. Compute BER and PER.
7. Apply retry-limited ARQ model to obtain success/drop probability and expected attempts.
8. Compute attempted, removed, delivered, and dropped rates.
9. Apply packet gating and storage saturation.
10. Write ``nodeDataOutMsg`` and ``downlinkOutMsg``.

Configurable Parameters
^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Module Parameters
   :widths: 22 14 14 50
   :header-rows: 1

   * - Parameter
     - Default
     - Unit
     - Description
   * - ``bitRateRequest``
     - 0.0
     - bit/s
     - Requested raw channel bit rate :math:`R_b`. If :math:`R_b \le 0`, throughput is zero.
   * - ``packetSizeBits``
     - 256.0
     - bit
     - Packet length :math:`L` for BER-to-PER conversion.
   * - ``maxRetransmissions``
     - 10
     - -
     - Retry cap used in the ARQ model. Current implementation enforces :math:`N \ge 1` and treats :math:`N` as maximum transmission attempts.
   * - ``receiverAntenna``
     - 0
     - -
     - Receiver selection: 0=auto, 1=use receiver path 1, 2=use receiver path 2.
   * - ``removalPolicy``
     - ``REMOVE_ATTEMPTED`` (0)
     - -
     - Storage removal mode: ``REMOVE_ATTEMPTED`` removes delivered+drop-limited bits, ``REMOVE_DELIVERED_ONLY`` removes only successfully delivered bits.
   * - ``requireFullPacket``
     - ``True``
     - bool
     - If ``True``, downlink waits until selected storage has at least one full packet.

Configuration Interface and Validation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The module exposes validated setters in C++/Python:

- ``setBitRateRequest(bitRateRequest)`` with :math:`bitRateRequest \ge 0`
- ``setPacketSizeBits(packetSizeBits)`` with :math:`packetSizeBits > 0`
- ``setMaxRetransmissions(maxRetransmissions)`` with :math:`maxRetransmissions \ge 1`
- ``setReceiverAntenna(receiverAntenna)`` with ``receiverAntenna in {0,1,2}``
- ``setRemovalPolicy(removalPolicy)`` with ``removalPolicy in {0,1}``
- ``setRequireFullPacket(requireFullPacket)``

If a setter receives an invalid value, the module rejects it and keeps the last valid value.
The Python wrapper also maps ``bitRateRequest``, ``packetSizeBits``, ``maxRetransmissions``,
``receiverAntenna``, ``removalPolicy``, and ``requireFullPacket`` to these validated setter/getter paths.

Receiver Selection and CNR1/CNR2 Usage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The module uses receiver-specific fields from :ref:`LinkBudgetMsgPayload`:

- ``CNR1`` corresponds to receiver path 1
- ``CNR2`` corresponds to receiver path 2

These are not duplicates. They represent two possible receiving directions/modes in the bidirectional
link-budget result.

Selection behavior:

- ``receiverAntenna = 1``: use path 1 (only if antenna state 1 is RX or RXTX)
- ``receiverAntenna = 2``: use path 2 (only if antenna state 2 is RX or RXTX)
- ``receiverAntenna = 0``: auto-select valid RX path with higher CNR

If no valid receiver path exists, the link is treated as inactive for that step.

Reliability and Throughput Model
--------------------------------

.. _downlinkhandling-figure-model:
.. figure:: /../../src/simulation/communication/downlinkHandling/_Documentation/Images/DownlinkHandlingReliabilityChain.svg
   :width: 95%
   :align: center
   :alt: CNR to BER to PER to ARQ to effective throughput chain

   Figure 2: Computation chain from RF link quality to storage removal and delivered data rates.

For valid inputs (linked/written link budget, selected CNR :math:`>0`, overlap bandwidth :math:`>0`,
:math:`R_b>0`, packet size :math:`>0`):

.. math::

   \mathrm{CNR}_{dB} = 10\log_{10}(\mathrm{CNR})

.. math::

   \frac{C}{N_0}\,[dBHz] = \mathrm{CNR}_{dB} + 10\log_{10}(B_{\mathrm{overlap}})

.. math::

   \frac{E_b}{N_0}\,[dB] = \frac{C}{N_0}[dBHz] - 10\log_{10}(R_b)

Current BER model (BPSK over AWGN):

.. math::

   \mathrm{BER} = Q\left(\sqrt{2E_b/N_0}\right)
   = \frac{1}{2}\,\mathrm{erfc}\left(\sqrt{E_b/N_0}\right)

Packet error model (independent bit errors, any bit error fails packet):

.. math::

   \mathrm{PER} = 1 - (1-\mathrm{BER})^L

Let :math:`N=\max(1,\texttt{maxRetransmissions})`. Retry-limited ARQ model:

.. math::

   P_{\mathrm{drop}} = \mathrm{PER}^{N}

.. math::

   P_{\mathrm{success}} = 1 - P_{\mathrm{drop}}

Expected attempts per source packet (truncated geometric expectation):

.. math::

   \mathbb{E}[A] =
   \begin{cases}
   \dfrac{P_{\mathrm{success}}}{1-\mathrm{PER}}, & \mathrm{PER}<1 \\
   N, & \mathrm{PER}=1
   \end{cases}

Unscaled rates:

.. math::

   R_{\mathrm{attempt,pot}} = R_b

.. math::

   R_{\mathrm{remove,pot}} = \frac{R_{\mathrm{attempt,pot}}}{\mathbb{E}[A]}

.. math::

   R_{\mathrm{delivered,pot}} = R_{\mathrm{remove,pot}}\,P_{\mathrm{success}}

.. math::

   R_{\mathrm{dropped,pot}} = R_{\mathrm{remove,pot}} - R_{\mathrm{delivered,pot}}

Storage and packet gating scale factor:

.. math::

   s = \mathrm{clamp}\!\left(
   \frac{B_{\mathrm{available}}/\Delta t}{R_{\mathrm{remove,pot}}},
   0, 1\right)

with additional logic:

- if ``requireFullPacket`` is ``True``, enforce :math:`B_{\mathrm{available}} \ge L`
- if :math:`\Delta t \le 0` or :math:`R_{\mathrm{remove,pot}} \le 0`, set :math:`s=0`

Final rates:

.. math::

   R_{\mathrm{attempt}} = s\,R_{\mathrm{attempt,pot}}, \quad
   R_{\mathrm{remove,modeled}} = s\,R_{\mathrm{remove,pot}}, \quad
   R_{\mathrm{delivered}} = s\,R_{\mathrm{delivered,pot}}, \quad
   R_{\mathrm{dropped}} = s\,R_{\mathrm{dropped,pot}}

Actual storage removal follows ``removalPolicy``:

.. math::

   R_{\mathrm{remove}} =
   \begin{cases}
   R_{\mathrm{remove,modeled}}, & \text{REMOVE\_ATTEMPTED} \\
   R_{\mathrm{delivered}}, & \text{REMOVE\_DELIVERED\_ONLY}
   \end{cases}

The value written to storage through ``nodeDataOutMsg`` is then:

.. math::

   \texttt{nodeDataOutMsg.baudRate} = -R_{\mathrm{remove}}

Output Diagnostics
------------------
The custom output :ref:`DownlinkHandlingMsgPayload` reports:

- link/selection state (``linkActive``, ``receiverIndex``, antenna names, ``removalPolicy``)
- physical-layer quality terms (CNR, :math:`C/N_0`, :math:`E_b/N_0`, BER, PER)
- ARQ reliability terms (success/drop probabilities, expected attempts)
- rate terms (attempted, removed, delivered, dropped)
- storage terms (available and estimated remaining bits)
- cumulative counters (attempted/removed/delivered/dropped bits)

Integration with simpleAntenna and linkBudget
----------------------------------------------
Typical chain:

1. :ref:`simpleAntenna` modules compute antenna logs.
2. :ref:`linkBudget` computes overlap bandwidth and CNR per receiver path.
3. :ref:`downlinkHandling` converts link quality to effective data transfer and storage removal.
4. Storage modules consume ``nodeDataOutMsg`` and reduce onboard buffered data.

.. warning::

   Also important integration note:
   Do not run ``spaceToGroundTransmitter`` and ``downlinkHandling`` as competing downlink removers
   on the same storage partitions. Pick one downlink path.

This separation is useful for fault modeling: upstream RF degradation (pointing, frequency mismatch,
atmospheric attenuation, receive-state changes) naturally propagates into BER/PER and delivered data.

Assumptions and Current Limits
------------------------------

- BER model is analytic BPSK/AWGN.
- Bit errors are independent.
- Any bit error fails the packet.
- ARQ is expectation-based, not packet-by-packet Monte Carlo.
- No explicit ACK latency, coding gain, framing overhead, or adaptive coding/modulation.
- ``REMOVE_DELIVERED_ONLY`` preserves dropped/undelivered bits onboard, but the module still uses an expected-rate ARQ model instead of explicit packet ACK/NACK timelines.
- Storage target selection prioritizes per-partition values. ``storageLevel`` is only used as fallback for messages that do not provide ``storedData`` entries.
- ``nodeDataOutMsg`` identifies storage by ``dataName`` only. If multiple linked storage units reuse the selected partition name, downlinkHandling forces removal to zero for that step to avoid ambiguous multi-unit draining. For multi-storage use, keep partition names globally unique across linked storage units.

Unit Test Coverage
------------------
Test file:

``src/simulation/communication/downlinkHandling/_UnitTest/test_downlinkHandling.py``

The tests verify:

- equation parity versus a Python-equivalent BER/PER/ARQ model
- zero-flow behavior for invalid link inputs
- retry-cap effects on drop probability and removal/delivery behavior
- removal-policy behavior (``REMOVE_ATTEMPTED`` vs ``REMOVE_DELIVERED_ONLY``)
- storage-limited rate capping and drain behavior
- automatic receiver selection from antenna RX states and CNR values
- duplicate-storage input rejection
- storage-target selection across multiple storage status messages
- ambiguous duplicate partition-name behavior across multiple storage status messages

User Guide
----------

Basic setup example:

.. code-block:: python

   from Basilisk.simulation import downlinkHandling, simpleStorageUnit

   dlh = downlinkHandling.DownlinkHandling()
   dlh.setBitRateRequest(1.0e5)      # [bit/s]
   dlh.setPacketSizeBits(1024.0)     # [bit]
   dlh.setMaxRetransmissions(8)      # [-]
   dlh.setReceiverAntenna(0)         # [-] auto-select valid RX path with highest CNR
   dlh.setRemovalPolicy(0)           # [-] 0=REMOVE_ATTEMPTED, 1=REMOVE_DELIVERED_ONLY
   dlh.setRequireFullPacket(True)    # [-]

   storage = simpleStorageUnit.SimpleStorageUnit()
   storage.storageCapacity = int(8e9)
   storage.addDataNodeToModel(dlh.nodeDataOutMsg)
   dlh.addStorageUnitToDownlink(storage.storageUnitDataOutMsg)

   # linkBudgetOutPayload is produced by the linkBudget module:
   # dlh.linkBudgetInMsg.subscribeTo(linkBudgetModule.linkBudgetOutPayload)
