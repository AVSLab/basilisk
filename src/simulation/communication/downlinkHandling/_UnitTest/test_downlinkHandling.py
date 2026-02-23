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


def python_equivalent_from_link(cnr_linear, bandwidth_hz, bit_rate_bps, packet_bits, max_retx):
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

    storage_removal_rate = bit_rate_bps / expected_attempts
    delivered_rate = storage_removal_rate * packet_success
    dropped_rate = storage_removal_rate - delivered_rate

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
    initial_bits=1.0e9,
    task_dt_s=1.0,
    stop_time_s=3.0
):
    debug_print(
        "run_downlink_case: "
        f"cnr1={cnr1}, cnr2={cnr2}, bandwidth_hz={bandwidth_hz}, "
        f"bit_rate_bps={bit_rate_bps}, packet_bits={packet_bits}, "
        f"max_retx={max_retx}, receiver_index={receiver_index}, "
        f"initial_bits={initial_bits}, task_dt_s={task_dt_s}, stop_time_s={stop_time_s}"
    )
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, macros.sec2nano(task_dt_s)))

    test_module = downlinkHandling.DownlinkHandling()
    test_module.ModelTag = "downlink"
    test_module.bitRateRequest = bit_rate_bps
    test_module.packetSizeBits = packet_bits
    test_module.maxRetransmissions = max_retx
    test_module.receiverAntenna = receiver_index
    test_module.requireFullPacket = True
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
    assert downlink_log.ber[-1] == pytest.approx(expected["ber"], rel=1e-12, abs=1e-15)
    assert downlink_log.per[-1] == pytest.approx(expected["per"], rel=1e-12, abs=1e-15)
    assert downlink_log.expectedAttemptsPerPacket[-1] == pytest.approx(expected["expected_attempts"], rel=1e-12, abs=1e-15)
    assert downlink_log.deliveredDataRate[-1] == pytest.approx(expected["delivered_rate"], rel=1e-12, abs=1e-9)
    assert downlink_log.storageRemovalRate[-1] == pytest.approx(expected["storage_removal_rate"], rel=1e-12, abs=1e-9)
    assert downlink_log.droppedDataRate[-1] == pytest.approx(expected["dropped_rate"], rel=1e-12, abs=1e-9)
    assert node_log.baudRate[-1] == pytest.approx(-expected["storage_removal_rate"], rel=1e-12, abs=1e-9)


def test_downlink_invalid_link_outputs_zero_flow():
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


def test_downlink_retry_limit_changes_storage_draw_not_goodput():
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


def test_downlink_storage_limited_case_caps_rate_and_drains_storage():
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
