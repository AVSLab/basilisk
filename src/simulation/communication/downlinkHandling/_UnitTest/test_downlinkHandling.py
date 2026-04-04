# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""
Unit tests for downlinkHandling.

Coverage:
- Formula parity against the Python-equivalent BER/PER/ARQ model.
- Zero-throughput behavior when link quality inputs are invalid.
- Retry-cap effects on storage drawdown and packet drop probability.
- Removal-policy behavior (attempted removal vs delivered-only removal).
- Storage-limited operation and remaining-data estimate behavior.
- Automatic receiver-path selection from linkBudget antenna states/CNR values.

Debug toggle:
- Default is off.
- Set ``BSK_DOWNLINK_TEST_DEBUG=1`` to print case setup and actual-vs-expected metrics.
- Or run this file directly with ``--debug``.
"""

import math
import importlib
import os
import sys

import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import downlinkHandling
from Basilisk.simulation import simpleStorageUnit
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

DownlinkHandlingMsgPayload = importlib.import_module("Basilisk.architecture.messaging.DownlinkHandlingMsgPayload")
LinkBudgetMsgPayload = importlib.import_module("Basilisk.architecture.messaging.LinkBudgetMsgPayload")
DataStorageStatusMsgPayload = importlib.import_module("Basilisk.architecture.messaging.DataStorageStatusMsgPayload")

REMOVE_ATTEMPTED = 0
REMOVE_DELIVERED_ONLY = 1

# Debug is false by default; can be enabled by environment variable or --debug when running this file directly.
DEBUG_DOWNLINK_TEST = False
if os.getenv("BSK_DOWNLINK_TEST_DEBUG", "0").strip().lower() in {"1", "true", "yes", "on"}:
    DEBUG_DOWNLINK_TEST = True


def debug_print(msg):
    if DEBUG_DOWNLINK_TEST:
        print(f"[downlinkHandlingTest] {msg}")


def debug_compare(name, actual, expected):
    if DEBUG_DOWNLINK_TEST:
        abs_err = abs(actual - expected)
        rel_err = abs_err / max(abs(expected), 1.0e-30)
        print(
            f"[downlinkHandlingTest] {name}: "
            f"actual={actual:.16e}, expected={expected:.16e}, "
            f"abs_err={abs_err:.3e}, rel_err={rel_err:.3e}"
        )


def q_function(x):
    return 0.5 * math.erfc(x / math.sqrt(2.0))


def python_equivalent_from_link(cnr_linear, bandwidth_hz, bit_rate_bps, packet_bits, max_retx, removal_policy=REMOVE_ATTEMPTED):
    cnr_db = 10.0 * math.log10(cnr_linear)
    c_n0_dbhz = cnr_db + 10.0 * math.log10(bandwidth_hz)
    eb_n0_db = c_n0_dbhz - 10.0 * math.log10(bit_rate_bps)

    eb_n0_linear = 10.0 ** (eb_n0_db / 10.0)
    ber = q_function(math.sqrt(2.0 * eb_n0_linear))
    per = 1.0 - (1.0 - ber) ** packet_bits

    one_try_success = 1.0 - per
    packet_drop = per ** max_retx
    packet_success = 1.0 - packet_drop

    if one_try_success <= 0.0:
        expected_attempts = float(max_retx)
    else:
        expected_attempts = packet_success / one_try_success

    modeled_storage_removal_rate = bit_rate_bps / expected_attempts
    delivered_rate = modeled_storage_removal_rate * packet_success
    dropped_rate = modeled_storage_removal_rate - delivered_rate
    if removal_policy == REMOVE_DELIVERED_ONLY:
        storage_removal_rate = delivered_rate
    else:
        storage_removal_rate = modeled_storage_removal_rate

    return {
        "c_n0_dbhz": c_n0_dbhz,
        "eb_n0_db": eb_n0_db,
        "ber": ber,
        "per": per,
        "packet_success": packet_success,
        "packet_drop": packet_drop,
        "expected_attempts": expected_attempts,
        "storage_removal_rate": storage_removal_rate,
        "delivered_rate": delivered_rate,
        "dropped_rate": dropped_rate
    }


def make_storage_status_msg(storage_level_bits, partition_entries):
    payload = DataStorageStatusMsgPayload.DataStorageStatusMsgPayload()
    payload.storageLevel = storage_level_bits          # [bit]

    names = DataStorageStatusMsgPayload.StringVector()
    values = DataStorageStatusMsgPayload.DoubleVector()
    for name, bits in partition_entries:
        names.push_back(name)
        values.push_back(bits)                         # [bit]

    payload.storedDataName = names
    payload.storedData = values
    return DataStorageStatusMsgPayload.DataStorageStatusMsg().write(payload)


def run_downlink_case(
    cnr1=0.0,
    cnr2=0.5,
    ant_state1=2,
    ant_state2=1,
    bandwidth_hz=1.0e6,
    bit_rate_bps=1.0e5,
    packet_bits=256.0,
    max_retx=10,
    receiver_index=2,
    removal_policy=REMOVE_ATTEMPTED,
    initial_bits=1.0e9,
    task_dt_s=1.0,
    stop_time_s=3.0
):
    debug_print(
        "run_downlink_case: "
        f"cnr1={cnr1}, cnr2={cnr2}, bandwidth_hz={bandwidth_hz}, "
        f"bit_rate_bps={bit_rate_bps}, packet_bits={packet_bits}, "
        f"max_retx={max_retx}, receiver_index={receiver_index}, "
        f"removal_policy={removal_policy}, "
        f"initial_bits={initial_bits}, task_dt_s={task_dt_s}, stop_time_s={stop_time_s}"
    )
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, macros.sec2nano(task_dt_s)))

    test_module = downlinkHandling.DownlinkHandling()
    test_module.ModelTag = "downlink"
    assert test_module.setBitRateRequest(bit_rate_bps)
    assert test_module.setPacketSizeBits(packet_bits)
    assert test_module.setMaxRetransmissions(max_retx)
    assert test_module.setReceiverAntenna(receiver_index)
    assert test_module.setRemovalPolicy(removal_policy)
    test_module.setRequireFullPacket(True)
    unit_test_sim.AddModelToTask(unit_task_name, test_module)

    data_storage = simpleStorageUnit.SimpleStorageUnit()
    data_storage.ModelTag = "storage"
    data_storage.storageCapacity = int(max(2.0 * initial_bits, initial_bits + 1.0e6))
    data_storage.addDataNodeToModel(test_module.nodeDataOutMsg)
    unit_test_sim.AddModelToTask(unit_task_name, data_storage)
    test_module.addStorageUnitToDownlink(data_storage.storageUnitDataOutMsg)
    data_storage.setDataBuffer(int(initial_bits))

    link_payload = LinkBudgetMsgPayload.LinkBudgetMsgPayload()
    link_payload.antennaName1 = "TX_Ant"
    link_payload.antennaName2 = "RX_Ant"
    link_payload.antennaState1 = ant_state1
    link_payload.antennaState2 = ant_state2
    link_payload.CNR1 = cnr1
    link_payload.CNR2 = cnr2
    link_payload.bandwidth = bandwidth_hz
    link_payload.frequency = 2.2e9
    link_msg = LinkBudgetMsgPayload.LinkBudgetMsg().write(link_payload)
    test_module.linkBudgetInMsg.subscribeTo(link_msg)

    node_log = test_module.nodeDataOutMsg.recorder()
    storage_log = data_storage.storageUnitDataOutMsg.recorder()
    downlink_reader = DownlinkHandlingMsgPayload.DownlinkHandlingMsgReader()
    downlink_reader.subscribeTo(test_module.downlinkOutMsg)
    downlink_log = downlink_reader.recorder()

    unit_test_sim.AddModelToTask(unit_task_name, node_log)
    unit_test_sim.AddModelToTask(unit_task_name, storage_log)
    unit_test_sim.AddModelToTask(unit_task_name, downlink_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time_s))
    unit_test_sim.ExecuteSimulation()

    if DEBUG_DOWNLINK_TEST:
        debug_print(
            "case results: "
            f"linkActive={downlink_log.linkActive[-1]}, "
            f"removalPolicy={downlink_log.removalPolicy[-1]}, "
            f"receiverIndex={downlink_log.receiverIndex[-1]}, "
            f"ber={downlink_log.ber[-1]:.16e}, per={downlink_log.per[-1]:.16e}, "
            f"storageRemovalRate={downlink_log.storageRemovalRate[-1]:.16e}, "
            f"deliveredRate={downlink_log.deliveredDataRate[-1]:.16e}, "
            f"droppedRate={downlink_log.droppedDataRate[-1]:.16e}, "
            f"nodeBaud={node_log.baudRate[-1]:.16e}, "
            f"storageLevel={storage_log.storageLevel[-1]:.16e}"
        )

    return test_module, node_log, storage_log, downlink_log


def test_downlink_matches_python_equivalent():
    """Verify C++ module outputs match the Python-equivalent BER/PER/ARQ model."""
    debug_print("test_downlink_matches_python_equivalent")
    cnr = 0.5
    bandwidth = 1.0e6
    bit_rate = 1.0e5
    packet_bits = 256.0
    max_retx = 10

    _, node_log, _, downlink_log = run_downlink_case(
        cnr2=cnr,
        bandwidth_hz=bandwidth,
        bit_rate_bps=bit_rate,
        packet_bits=packet_bits,
        max_retx=max_retx,
        initial_bits=1.0e9
    )

    expected = python_equivalent_from_link(cnr, bandwidth, bit_rate, packet_bits, max_retx)
    debug_compare("ber", downlink_log.ber[-1], expected["ber"])
    debug_compare("per", downlink_log.per[-1], expected["per"])
    debug_compare("expectedAttemptsPerPacket", downlink_log.expectedAttemptsPerPacket[-1], expected["expected_attempts"])
    debug_compare("deliveredDataRate", downlink_log.deliveredDataRate[-1], expected["delivered_rate"])
    debug_compare("storageRemovalRate", downlink_log.storageRemovalRate[-1], expected["storage_removal_rate"])
    debug_compare("droppedDataRate", downlink_log.droppedDataRate[-1], expected["dropped_rate"])
    debug_compare("nodeBaudRate", node_log.baudRate[-1], -expected["storage_removal_rate"])

    assert downlink_log.linkActive[-1] == 1
    assert downlink_log.removalPolicy[-1] == REMOVE_ATTEMPTED
    assert downlink_log.ber[-1] == pytest.approx(expected["ber"], rel=1e-12, abs=1e-15)
    assert downlink_log.per[-1] == pytest.approx(expected["per"], rel=1e-12, abs=1e-15)
    assert downlink_log.expectedAttemptsPerPacket[-1] == pytest.approx(expected["expected_attempts"], rel=1e-12, abs=1e-15)
    assert downlink_log.deliveredDataRate[-1] == pytest.approx(expected["delivered_rate"], rel=1e-12, abs=1e-9)
    assert downlink_log.storageRemovalRate[-1] == pytest.approx(expected["storage_removal_rate"], rel=1e-12, abs=1e-9)
    assert downlink_log.droppedDataRate[-1] == pytest.approx(expected["dropped_rate"], rel=1e-12, abs=1e-9)
    assert node_log.baudRate[-1] == pytest.approx(-expected["storage_removal_rate"], rel=1e-12, abs=1e-9)


def test_downlink_invalid_link_outputs_zero_flow():
    """Verify invalid link inputs produce zero link-active flag and zero throughput."""
    debug_print("test_downlink_invalid_link_outputs_zero_flow")
    _, node_log, _, downlink_log = run_downlink_case(
        cnr2=0.0,
        bandwidth_hz=1.0e6,
        bit_rate_bps=1.0e5,
        packet_bits=256.0,
        max_retx=10,
        initial_bits=1.0e9
    )

    assert downlink_log.linkActive[-1] == 0
    assert downlink_log.storageRemovalRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert downlink_log.deliveredDataRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert downlink_log.droppedDataRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert node_log.baudRate[-1] == pytest.approx(0.0, abs=1e-12)


def test_downlink_disabling_node_clears_diagnostics():
    """Verify disabling the node zeroes downlink diagnostics instead of republishing stale values."""
    debug_print("test_downlink_disabling_node_clears_diagnostics")
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, macros.sec2nano(1.0)))  # [s]

    test_module = downlinkHandling.DownlinkHandling()
    test_module.ModelTag = "downlink"
    assert test_module.setBitRateRequest(1.0e5)       # [bit/s]
    assert test_module.setPacketSizeBits(256.0)       # [bit]
    assert test_module.setMaxRetransmissions(4)       # [-]
    assert test_module.setReceiverAntenna(2)          # [-]
    unit_test_sim.AddModelToTask(unit_task_name, test_module)

    data_storage = simpleStorageUnit.SimpleStorageUnit()
    data_storage.ModelTag = "storage"
    data_storage.storageCapacity = int(2.0e9)         # [bit]
    data_storage.addDataNodeToModel(test_module.nodeDataOutMsg)
    unit_test_sim.AddModelToTask(unit_task_name, data_storage)
    test_module.addStorageUnitToDownlink(data_storage.storageUnitDataOutMsg)
    data_storage.setDataBuffer(int(1.0e9))            # [bit]

    link_payload = LinkBudgetMsgPayload.LinkBudgetMsgPayload()
    link_payload.antennaName1 = "TX_Ant"
    link_payload.antennaName2 = "RX_Ant"
    link_payload.antennaState1 = 2                    # [-] TX
    link_payload.antennaState2 = 1                    # [-] RX
    link_payload.CNR1 = 0.0                           # [-]
    link_payload.CNR2 = 0.5                           # [-]
    link_payload.bandwidth = 1.0e6                    # [Hz]
    link_payload.frequency = 2.2e9                    # [Hz]
    link_msg = LinkBudgetMsgPayload.LinkBudgetMsg().write(link_payload)
    test_module.linkBudgetInMsg.subscribeTo(link_msg)

    status_payload = messaging.DeviceCmdMsgPayload()
    status_payload.deviceCmd = 1                      # [-] on
    status_msg = messaging.DeviceCmdMsg()
    status_msg.write(status_payload)
    test_module.nodeStatusInMsg.subscribeTo(status_msg)

    node_log = test_module.nodeDataOutMsg.recorder()
    downlink_reader = DownlinkHandlingMsgPayload.DownlinkHandlingMsgReader()
    downlink_reader.subscribeTo(test_module.downlinkOutMsg)
    downlink_log = downlink_reader.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, node_log)
    unit_test_sim.AddModelToTask(unit_task_name, downlink_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))        # [s]
    unit_test_sim.ExecuteSimulation()

    assert downlink_log.linkActive[-1] == 1
    assert downlink_log.storageRemovalRate[-1] > 0.0
    assert node_log.baudRate[-1] < 0.0

    status_payload.deviceCmd = 0                      # [-] off
    status_msg.write(status_payload, unit_test_sim.TotalSim.CurrentNanos)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))        # [s]
    unit_test_sim.ExecuteSimulation()

    assert downlink_log.linkActive[-1] == 0
    assert downlink_log.receiverIndex[-1] == 0
    assert downlink_log.storageRemovalRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert downlink_log.deliveredDataRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert downlink_log.droppedDataRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert downlink_log.ber[-1] == pytest.approx(0.0, abs=1e-12)
    assert downlink_log.per[-1] == pytest.approx(0.0, abs=1e-12)
    assert node_log.baudRate[-1] == pytest.approx(0.0, abs=1e-12)


def test_downlink_reenable_does_not_integrate_disabled_downtime():
    """Verify re-enabling after several disabled ticks does not collapse off-time into one large downlink step."""
    debug_print("test_downlink_reenable_does_not_integrate_disabled_downtime")
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"
    task_dt_s = 1.0                                          # [s]

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, macros.sec2nano(task_dt_s)))

    test_module = downlinkHandling.DownlinkHandling()
    test_module.ModelTag = "downlink"
    assert test_module.setBitRateRequest(1.0e5)               # [bit/s]
    assert test_module.setPacketSizeBits(256.0)               # [bit]
    assert test_module.setMaxRetransmissions(4)               # [-]
    assert test_module.setReceiverAntenna(2)                  # [-]
    unit_test_sim.AddModelToTask(unit_task_name, test_module)

    data_storage = simpleStorageUnit.SimpleStorageUnit()
    data_storage.ModelTag = "storage"
    data_storage.storageCapacity = int(2.0e9)                 # [bit]
    data_storage.addDataNodeToModel(test_module.nodeDataOutMsg)
    unit_test_sim.AddModelToTask(unit_task_name, data_storage)
    test_module.addStorageUnitToDownlink(data_storage.storageUnitDataOutMsg)
    data_storage.setDataBuffer(int(1.0e9))                    # [bit]

    link_payload = LinkBudgetMsgPayload.LinkBudgetMsgPayload()
    link_payload.antennaName1 = "TX_Ant"
    link_payload.antennaName2 = "RX_Ant"
    link_payload.antennaState1 = 2                            # [-] TX
    link_payload.antennaState2 = 1                            # [-] RX
    link_payload.CNR1 = 0.0                                   # [-]
    link_payload.CNR2 = 0.5                                   # [-]
    link_payload.bandwidth = 1.0e6                            # [Hz]
    link_payload.frequency = 2.2e9                            # [Hz]
    link_msg = LinkBudgetMsgPayload.LinkBudgetMsg().write(link_payload)
    test_module.linkBudgetInMsg.subscribeTo(link_msg)

    status_payload = messaging.DeviceCmdMsgPayload()
    status_payload.deviceCmd = 1                              # [-] on
    status_msg = messaging.DeviceCmdMsg()
    status_msg.write(status_payload)
    test_module.nodeStatusInMsg.subscribeTo(status_msg)

    downlink_reader = DownlinkHandlingMsgPayload.DownlinkHandlingMsgReader()
    downlink_reader.subscribeTo(test_module.downlinkOutMsg)
    downlink_log = downlink_reader.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, downlink_log)

    unit_test_sim.InitializeSimulation()

    # Active baseline tick
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))     # [s]
    unit_test_sim.ExecuteSimulation()
    baseline_rate = downlink_log.storageRemovalRate[-1]        # [bit/s]
    baseline_cum_removed = downlink_log.cumulativeRemovedBits[-1]  # [bit]
    assert downlink_log.timeStep[-1] == pytest.approx(task_dt_s, abs=1e-12)
    assert baseline_rate > 0.0

    # Disable for multiple ticks and verify cumulative removal does not change.
    status_payload.deviceCmd = 0                               # [-] off
    status_msg.write(status_payload, unit_test_sim.TotalSim.CurrentNanos)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(4.0))     # [s]
    unit_test_sim.ExecuteSimulation()
    cum_removed_after_off = downlink_log.cumulativeRemovedBits[-1]  # [bit]
    assert cum_removed_after_off == pytest.approx(baseline_cum_removed, abs=1e-9)

    # Re-enable and verify single-step timing/removal behavior.
    status_payload.deviceCmd = 1                               # [-] on
    status_msg.write(status_payload, unit_test_sim.TotalSim.CurrentNanos)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(5.0))     # [s]
    unit_test_sim.ExecuteSimulation()

    assert downlink_log.timeStep[-1] == pytest.approx(task_dt_s, abs=1e-12)
    assert downlink_log.storageRemovalRate[-1] == pytest.approx(baseline_rate, rel=1e-12, abs=1e-9)
    reenable_delta_removed = downlink_log.cumulativeRemovedBits[-1] - cum_removed_after_off  # [bit]
    assert reenable_delta_removed == pytest.approx(baseline_rate * task_dt_s, rel=1e-12, abs=1e-6)


def test_downlink_retry_limit_changes_storage_draw_not_goodput():
    """Verify higher retry caps reduce storage draw while preserving expected delivered-rate level."""
    debug_print("test_downlink_retry_limit_changes_storage_draw_not_goodput")
    common = dict(
        cnr2=0.5,
        bandwidth_hz=1.0e6,
        bit_rate_bps=1.0e5,
        packet_bits=256.0,
        initial_bits=1.0e9
    )

    _, _, _, downlink_log_m1 = run_downlink_case(max_retx=1, **common)
    _, _, _, downlink_log_m8 = run_downlink_case(max_retx=8, **common)
    debug_print(
        "retry comparison: "
        f"m1(storageRemovalRate={downlink_log_m1.storageRemovalRate[-1]:.16e}, "
        f"packetDropProb={downlink_log_m1.packetDropProb[-1]:.16e}, "
        f"deliveredDataRate={downlink_log_m1.deliveredDataRate[-1]:.16e}) "
        f"m8(storageRemovalRate={downlink_log_m8.storageRemovalRate[-1]:.16e}, "
        f"packetDropProb={downlink_log_m8.packetDropProb[-1]:.16e}, "
        f"deliveredDataRate={downlink_log_m8.deliveredDataRate[-1]:.16e})"
    )

    assert downlink_log_m1.storageRemovalRate[-1] > downlink_log_m8.storageRemovalRate[-1]
    assert downlink_log_m1.packetDropProb[-1] > downlink_log_m8.packetDropProb[-1]
    assert downlink_log_m1.deliveredDataRate[-1] == pytest.approx(downlink_log_m8.deliveredDataRate[-1], rel=1e-12, abs=1e-9)


def test_downlink_remove_delivered_only_retains_undelivered_bits():
    """Verify delivered-only removal mode keeps dropped/undelivered bits onboard."""
    debug_print("test_downlink_remove_delivered_only_retains_undelivered_bits")
    common = dict(
        cnr2=0.5,
        bandwidth_hz=1.0e6,
        bit_rate_bps=1.0e5,
        packet_bits=256.0,
        max_retx=4,
        initial_bits=1.0e9,
        stop_time_s=3.0
    )

    _, node_attempted, storage_attempted, downlink_attempted = run_downlink_case(
        removal_policy=REMOVE_ATTEMPTED, **common
    )
    _, node_delivered, storage_delivered, downlink_delivered = run_downlink_case(
        removal_policy=REMOVE_DELIVERED_ONLY, **common
    )

    assert downlink_attempted.removalPolicy[-1] == REMOVE_ATTEMPTED
    assert downlink_delivered.removalPolicy[-1] == REMOVE_DELIVERED_ONLY

    assert downlink_attempted.attemptedDataRate[-1] == pytest.approx(
        downlink_delivered.attemptedDataRate[-1], rel=1e-12, abs=1e-9
    )
    assert downlink_attempted.deliveredDataRate[-1] == pytest.approx(
        downlink_delivered.deliveredDataRate[-1], rel=1e-12, abs=1e-9
    )
    assert downlink_attempted.droppedDataRate[-1] == pytest.approx(
        downlink_delivered.droppedDataRate[-1], rel=1e-12, abs=1e-9
    )

    assert downlink_attempted.storageRemovalRate[-1] > downlink_delivered.storageRemovalRate[-1]
    assert downlink_delivered.storageRemovalRate[-1] == pytest.approx(
        downlink_delivered.deliveredDataRate[-1], rel=1e-12, abs=1e-9
    )

    assert node_attempted.baudRate[-1] == pytest.approx(-downlink_attempted.storageRemovalRate[-1], rel=1e-12, abs=1e-9)
    assert node_delivered.baudRate[-1] == pytest.approx(-downlink_delivered.storageRemovalRate[-1], rel=1e-12, abs=1e-9)
    assert storage_delivered.storageLevel[-1] > storage_attempted.storageLevel[-1]


def test_downlink_storage_limited_case_caps_rate_and_drains_storage():
    """Verify storage-limited operation caps removal rate and drains remaining data correctly."""
    debug_print("test_downlink_storage_limited_case_caps_rate_and_drains_storage")
    initial_bits = 300.0
    _, _, storage_log, downlink_log = run_downlink_case(
        cnr2=100.0,
        bandwidth_hz=1.0e6,
        bit_rate_bps=1.0e5,
        packet_bits=256.0,
        max_retx=10,
        initial_bits=initial_bits,
        stop_time_s=1.0
    )

    expected_cap_rate = initial_bits / downlink_log.timeStep[-1]
    debug_compare("storage_limited_rate_cap", downlink_log.storageRemovalRate[-1], expected_cap_rate)
    debug_print(
        "storage-limited results: "
        f"remainingBits={downlink_log.estimatedRemainingDataBits[-1]:.16e}, "
        f"storageLevel={storage_log.storageLevel[-1]:.16e}"
    )
    assert downlink_log.storageRemovalRate[-1] == pytest.approx(expected_cap_rate, rel=1e-12, abs=1e-9)
    assert downlink_log.estimatedRemainingDataBits[-1] == pytest.approx(0.0, abs=1.0)
    assert storage_log.storageLevel[-1] == pytest.approx(0.0, abs=1.0)


def test_downlink_auto_receiver_selects_valid_rx_path():
    """Verify auto receiver selection chooses a valid RX path with nonzero CNR."""
    debug_print("test_downlink_auto_receiver_selects_valid_rx_path")
    _, _, _, downlink_log = run_downlink_case(
        cnr1=0.8,
        cnr2=0.0,
        ant_state1=1,
        ant_state2=2,
        receiver_index=0,
        initial_bits=1.0e9
    )
    debug_print(f"auto receiver selected index={downlink_log.receiverIndex[-1]}")

    assert downlink_log.linkActive[-1] == 1
    assert downlink_log.receiverIndex[-1] == 1


def test_downlink_setter_validation_rejects_invalid_inputs():
    """Check that module setters reject invalid values and preserve prior valid configuration."""
    debug_print("test_downlink_setter_validation_rejects_invalid_inputs")
    test_module = downlinkHandling.DownlinkHandling()

    assert test_module.setBitRateRequest(1.0e5)    # [bit/s]
    assert test_module.setPacketSizeBits(1024.0)   # [bit]
    assert test_module.setMaxRetransmissions(6)    # [-]
    assert test_module.setReceiverAntenna(2)       # [-]
    assert test_module.setRemovalPolicy(REMOVE_DELIVERED_ONLY)
    test_module.setRequireFullPacket(True)

    with pytest.raises(Exception):
        test_module.setBitRateRequest(-1.0)                   # [bit/s]
    with pytest.raises(Exception):
        test_module.setBitRateRequest(float("nan"))           # [bit/s]
    with pytest.raises(Exception):
        test_module.setPacketSizeBits(0.0)                    # [bit]
    with pytest.raises(Exception):
        test_module.setPacketSizeBits(float("inf"))           # [bit]
    with pytest.raises(Exception):
        test_module.setMaxRetransmissions(0)                  # [-]
    with pytest.raises(Exception):
        test_module.setReceiverAntenna(4)                     # [-]
    with pytest.raises(Exception):
        test_module.setRemovalPolicy(2)                       # [-]

    assert test_module.getBitRateRequest() == pytest.approx(1.0e5, abs=1e-12)
    assert test_module.getPacketSizeBits() == pytest.approx(1024.0, abs=1e-12)
    assert test_module.getMaxRetransmissions() == 6
    assert test_module.getReceiverAntenna() == 2
    assert test_module.getRemovalPolicy() == REMOVE_DELIVERED_ONLY
    assert test_module.getRequireFullPacket() is True


def test_downlink_forced_receiver_invalid_path_disables_link():
    """Verify that forcing an unavailable receiver path yields no selected receiver and zero throughput."""
    debug_print("test_downlink_forced_receiver_invalid_path_disables_link")
    _, node_log, _, downlink_log = run_downlink_case(
        cnr1=0.8,
        cnr2=0.0,
        ant_state1=1,
        ant_state2=2,
        receiver_index=2,
        initial_bits=1.0e8
    )

    assert downlink_log.receiverIndex[-1] == 0
    assert downlink_log.linkActive[-1] == 0
    assert downlink_log.storageRemovalRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert node_log.baudRate[-1] == pytest.approx(0.0, abs=1e-12)


def test_downlink_duplicate_storage_message_is_rejected():
    """Verify duplicate storage message registration is rejected by the module."""
    debug_print("test_downlink_duplicate_storage_message_is_rejected")

    test_module = downlinkHandling.DownlinkHandling()
    storage_msg = make_storage_status_msg(100.0, [("DATA", 100.0)])  # [bit]

    assert test_module.addStorageUnitToDownlink(storage_msg) is True
    assert test_module.addStorageUnitToDownlink(storage_msg) is False


def test_downlink_selects_largest_partition_across_storage_messages():
    """Verify storage selection uses the largest partition across all linked storage status inputs."""
    debug_print("test_downlink_selects_largest_partition_across_storage_messages")

    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, macros.sec2nano(1.0)))  # [s]

    test_module = downlinkHandling.DownlinkHandling()
    test_module.ModelTag = "downlink"
    assert test_module.setBitRateRequest(1.0e5)       # [bit/s]
    assert test_module.setPacketSizeBits(256.0)       # [bit]
    assert test_module.setMaxRetransmissions(4)       # [-]
    assert test_module.setReceiverAntenna(2)          # [-]
    test_module.setRequireFullPacket(True)            # [-]
    unit_test_sim.AddModelToTask(unit_task_name, test_module)

    storage_msg_a = make_storage_status_msg(
        900.0,                                         # [bit]
        [("A_BIG", 400.0), ("A_SMALL", 100.0)]        # [bit]
    )
    storage_msg_b = make_storage_status_msg(
        2000.0,                                        # [bit]
        [("B_TOP", 1200.0), ("B_OTHER", 800.0)]       # [bit]
    )
    assert test_module.addStorageUnitToDownlink(storage_msg_a)
    assert test_module.addStorageUnitToDownlink(storage_msg_b)

    link_payload = LinkBudgetMsgPayload.LinkBudgetMsgPayload()
    link_payload.antennaName1 = "TX_Ant"
    link_payload.antennaName2 = "RX_Ant"
    link_payload.antennaState1 = 2                    # [-] TX
    link_payload.antennaState2 = 1                    # [-] RX
    link_payload.CNR1 = 0.0                           # [-]
    link_payload.CNR2 = 20.0                          # [-]
    link_payload.bandwidth = 1.0e6                    # [Hz]
    link_payload.frequency = 2.2e9                    # [Hz]
    link_msg = LinkBudgetMsgPayload.LinkBudgetMsg().write(link_payload)
    test_module.linkBudgetInMsg.subscribeTo(link_msg)

    downlink_reader = DownlinkHandlingMsgPayload.DownlinkHandlingMsgReader()
    downlink_reader.subscribeTo(test_module.downlinkOutMsg)
    downlink_log = downlink_reader.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, downlink_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))    # [s]
    unit_test_sim.ExecuteSimulation()

    assert downlink_log.availableDataBits[-1] == pytest.approx(1200.0, abs=1e-12)
    assert downlink_log.dataName[-1] == "B_TOP"


def test_downlink_ambiguous_multi_storage_name_blocks_removal():
    """Verify duplicate partition names across linked storage units force removal rate to zero."""
    debug_print("test_downlink_ambiguous_multi_storage_name_blocks_removal")

    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, macros.sec2nano(1.0)))  # [s]

    test_module = downlinkHandling.DownlinkHandling()
    test_module.ModelTag = "downlink"
    assert test_module.setBitRateRequest(1.0e5)       # [bit/s]
    assert test_module.setPacketSizeBits(256.0)       # [bit]
    assert test_module.setMaxRetransmissions(4)       # [-]
    assert test_module.setReceiverAntenna(2)          # [-]
    test_module.setRequireFullPacket(True)            # [-]
    unit_test_sim.AddModelToTask(unit_task_name, test_module)

    storage_msg_a = make_storage_status_msg(
        800.0,                                         # [bit]
        [("STORED DATA", 400.0)]                      # [bit]
    )
    storage_msg_b = make_storage_status_msg(
        3000.0,                                        # [bit]
        [("STORED DATA", 1200.0)]                     # [bit]
    )
    assert test_module.addStorageUnitToDownlink(storage_msg_a)
    assert test_module.addStorageUnitToDownlink(storage_msg_b)

    link_payload = LinkBudgetMsgPayload.LinkBudgetMsgPayload()
    link_payload.antennaName1 = "TX_Ant"
    link_payload.antennaName2 = "RX_Ant"
    link_payload.antennaState1 = 2                    # [-] TX
    link_payload.antennaState2 = 1                    # [-] RX
    link_payload.CNR1 = 0.0                           # [-]
    link_payload.CNR2 = 20.0                          # [-]
    link_payload.bandwidth = 1.0e6                    # [Hz]
    link_payload.frequency = 2.2e9                    # [Hz]
    link_msg = LinkBudgetMsgPayload.LinkBudgetMsg().write(link_payload)
    test_module.linkBudgetInMsg.subscribeTo(link_msg)

    node_log = test_module.nodeDataOutMsg.recorder()
    downlink_reader = DownlinkHandlingMsgPayload.DownlinkHandlingMsgReader()
    downlink_reader.subscribeTo(test_module.downlinkOutMsg)
    downlink_log = downlink_reader.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, node_log)
    unit_test_sim.AddModelToTask(unit_task_name, downlink_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))    # [s]
    unit_test_sim.ExecuteSimulation()

    assert downlink_log.linkActive[-1] == 1
    assert downlink_log.dataName[-1] == "STORED DATA"
    assert downlink_log.availableDataBits[-1] == pytest.approx(1200.0, abs=1e-12)
    assert downlink_log.storageRemovalRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert downlink_log.deliveredDataRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert downlink_log.droppedDataRate[-1] == pytest.approx(0.0, abs=1e-12)
    assert node_log.baudRate[-1] == pytest.approx(0.0, abs=1e-12)


if __name__ == "__main__":
    # Allow direct execution:
    # python test_downlinkHandling.py -k test_name -q -s --debug
    args = sys.argv[1:]
    debug_flags = {"--debug", "--debug-downlink", "--debug-downlink-test"}
    filtered_args = []
    debug_requested = False

    for arg in args:
        if arg in debug_flags:
            debug_requested = True
        else:
            filtered_args.append(arg)

    # If no explicit test file/path is passed, run this file only.
    explicit_target = any(a.endswith(".py") or os.path.exists(a) for a in filtered_args if not a.startswith("-"))
    if not explicit_target:
        filtered_args.append(os.path.abspath(__file__))

    if debug_requested:
        if "-s" not in filtered_args:
            filtered_args.append("-s")
        os.environ["BSK_DOWNLINK_TEST_DEBUG"] = "1"
        DEBUG_DOWNLINK_TEST = True
    raise SystemExit(pytest.main(filtered_args))
