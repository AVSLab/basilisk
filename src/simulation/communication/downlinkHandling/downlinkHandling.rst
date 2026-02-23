.. _downlinkHandling:

.. Warning::

   **[BETA]** :ref:`downlinkHandling` is a beta module in an initial public release. This module might be
   subject to changes in future releases.

Executive Summary
-----------------
This document describes how the :ref:`downlinkHandling` module operates within the Basilisk simulation framework.
The module maps radio link quality from :ref:`linkBudget` into communication reliability and effective telemetry throughput.
It then converts this throughput into a data sink request that removes data from onboard storage.

In short, the module provides:

- Link-quality to BER/PER conversion for a configurable bit rate and packet size.
- Retry-limited delivery modeling (ARQ-style retransmissions).
- Effective throughput estimates (attempted, removed, delivered, dropped).
- Tight coupling to Basilisk storage units through :ref:`DataNodeUsageMsgPayload`.

Message Connection Descriptions
-------------------------------
The following table lists all module input and output messages.
The module message connections are set by the user from Python.

.. list-table:: Module I/O Messages
   :widths: 24 22 34 20
   :header-rows: 1

   * - Msg Variable Name
     - Msg Type
     - Description
     - Note
   * - linkBudgetInMsg
     - :ref:`LinkBudgetMsgPayload`
     - Link-quality input from :ref:`linkBudget` (CNR, overlap bandwidth, frequency, antenna states).
     - Required
   * - storageUnitInMsgs (via ``addStorageUnitToDownlink``)
     - :ref:`DataStorageStatusMsgPayload`
     - Storage state input used to determine available data and selected partition name.
     - Required for actual data removal
   * - nodeDataOutMsg
     - :ref:`DataNodeUsageMsgPayload`
     - Data-node output where negative baud rate removes data from storage.
     - Output (inherited from ``DataNodeBase``)
   * - downlinkOutMsg
     - :ref:`DownlinkHandlingMsgPayload`
     - Detailed diagnostics (BER/PER, retry probabilities, rates, cumulative counters).
     - Output

Detailed Module Description
---------------------------
The module extends ``DataNodeBase`` and executes once per simulation step.
At each step it:

1. Reads ``LinkBudgetMsgPayload`` and storage status messages.
2. Selects the receive path (forced receiver index or auto-select).
3. Converts CNR and overlap bandwidth to :math:`C/N_0`.
4. Converts :math:`C/N_0` and requested bit rate to :math:`E_b/N_0`.
5. Computes BER and packet error rate (PER).
6. Applies retry limit to estimate packet delivery/drop probabilities.
7. Converts reliability into effective source-data outflow from storage.
8. Writes both standard data-node output and a detailed diagnostics output.

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
     - Requested raw channel bit rate :math:`R_b`.
   * - ``packetSizeBits``
     - 256.0
     - bit
     - Packet length :math:`L` used for BER-to-PER conversion.
   * - ``maxRetransmissions``
     - 10
     - -
     - Retry cap used in truncated ARQ probability model.
   * - ``receiverAntenna``
     - 0
     - -
     - Receiver selection: 0=auto, 1=use antenna1 CNR, 2=use antenna2 CNR.
   * - ``requireFullPacket``
     - True
     - bool
     - If enabled, downlink only starts when at least one full packet exists in storage.

Receiver Path Selection
^^^^^^^^^^^^^^^^^^^^^^^
The module uses antenna state information from ``LinkBudgetMsgPayload``:

- ``ANTENNA_RX`` and ``ANTENNA_RXTX`` are treated as valid receiver states.
- For ``receiverAntenna = 0`` (auto), the module chooses the valid receiver path with highest CNR.
- If no valid receiver path exists, the link is treated as inactive for this step.

BER/PER and Throughput Model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For valid inputs (:math:`\mathrm{CNR} > 0`, overlap bandwidth :math:`>0`, :math:`R_b>0`, packet size :math:`>0`):

.. math::

   \mathrm{CNR}_{dB} = 10 \log_{10}(\mathrm{CNR})

.. math::

   \frac{C}{N_0}\,[dBHz] = \mathrm{CNR}_{dB} + 10\log_{10}(B_\mathrm{overlap})

.. math::

   \frac{E_b}{N_0}\,[dB] = \frac{C}{N_0}[dBHz] - 10\log_{10}(R_b)

The current BER model is BPSK over AWGN:

.. math::

   \mathrm{BER} = Q\left(\sqrt{2E_b/N_0}\right) = \frac{1}{2}\,\mathrm{erfc}\left(\sqrt{E_b/N_0}\right)

Packet error model (independent bit errors, any bit error fails packet):

.. math::

   \mathrm{PER} = 1 - (1-\mathrm{BER})^L

Retry-limited completion/drop model:

.. math::

   P_\mathrm{drop} = \mathrm{PER}^{N}

.. math::

   P_\mathrm{success} = 1 - P_\mathrm{drop}

where :math:`N` is ``maxRetransmissions``.

Expected attempts per source packet (truncated geometric form):

.. math::

   \mathbb{E}[A] = \frac{P_\mathrm{success}}{1-\mathrm{PER}}

Rate mapping:

.. math::

   R_\mathrm{attempt} = R_b

.. math::

   R_\mathrm{remove} = \frac{R_\mathrm{attempt}}{\mathbb{E}[A]}

.. math::

   R_\mathrm{delivered} = R_\mathrm{remove}\,P_\mathrm{success}

.. math::

   R_\mathrm{dropped} = R_\mathrm{remove} - R_\mathrm{delivered}

The module then applies storage/time-step saturation so removed data in a step never exceeds available bits.
The final storage removal is written as negative baud rate to ``nodeDataOutMsg``.

Output Diagnostics Message
^^^^^^^^^^^^^^^^^^^^^^^^^^
The custom output message :ref:`DownlinkHandlingMsgPayload` includes:

- Link state and selected receiver path.
- CNR, :math:`C/N_0`, :math:`E_b/N_0`, BER, PER.
- Packet success/drop probabilities and expected attempts.
- Attempted/removal/delivered/dropped rates.
- Available and estimated remaining bits for selected data partition.
- Cumulative attempted/removed/delivered/dropped bit counters.

Integration with simpleAntenna and linkBudget
----------------------------------------------
Typical communication chain:

1. Two :ref:`simpleAntenna` modules compute antenna-level properties and output :ref:`AntennaLogMsgPayload`.
2. :ref:`linkBudget` consumes both antenna logs and computes CNR and overlap bandwidth in :ref:`LinkBudgetMsgPayload`.
3. :ref:`downlinkHandling` consumes :ref:`LinkBudgetMsgPayload` and storage status to produce effective downlink throughput.
4. Storage units consume ``nodeDataOutMsg`` and decrement onboard data.

This separation lets RF/pointing/frequency effects propagate naturally through CNR into BER/PER and finally into delivered data.

Module Assumptions and Limitations
----------------------------------

- BER model is currently analytic BPSK/AWGN.
- Bit errors are assumed independent.
- Packet failure occurs if any bit is wrong.
- Retry model is expectation-based (not packet-by-packet Monte Carlo simulation).
- No explicit ACK latency, framing overhead, coding gain, or adaptive modulation/coding.
- If no valid receiver or invalid link inputs exist, output rates are zero for that step.

Unit Test Coverage
------------------
The module unit test is located at:

``src/simulation/communication/downlinkHandling/_UnitTest/test_downlinkHandling.py``

The test suite verifies:

- Numerical parity with the Python-equivalent BER/PER/ARQ equations.
- Zero-flow behavior when link quality is invalid.
- Retry-cap effects on drop probability and storage removal behavior.
- Storage-limited behavior and remaining-data estimates.
- Automatic receiver-path selection from link-budget antenna states.

User Guide Snippet
------------------

.. code-block:: python

   from Basilisk.simulation import downlinkHandling, simpleStorageUnit

   dlh = downlinkHandling.DownlinkHandling()
   dlh.bitRateRequest = 1.0e5       # bit/s
   dlh.packetSizeBits = 1024.0      # bit
   dlh.maxRetransmissions = 8
   dlh.receiverAntenna = 0          # auto
   dlh.requireFullPacket = True

   storage = simpleStorageUnit.SimpleStorageUnit()
   storage.storageCapacity = int(8e9)

   # storage consumes downlink node output (negative baud removes data)
   storage.addDataNodeToModel(dlh.nodeDataOutMsg)
   dlh.addStorageUnitToDownlink(storage.storageUnitDataOutMsg)

   # linkBudgetOutMsg would be connected from linkBudget module
   # dlh.linkBudgetInMsg.subscribeTo(linkBudgetModule.linkBudgetOutPayload)
